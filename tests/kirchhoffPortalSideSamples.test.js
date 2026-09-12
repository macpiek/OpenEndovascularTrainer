import test from 'node:test';
import assert from 'node:assert/strict';
import { buildKirchhoffPortalSideSamples as build } from '../src/physics/kirchhoffPortalSideSamples.js';
import { captureKirchhoffCoupledTrialState as capture, restoreKirchhoffCoupledTrialState as restore } from '../src/physics/kirchhoffCoupledTrialState.js';

const body = (a, b) => Object.fromEntries(['x', 'y', 'z'].map((key, k) => [key, new Float64Array([a[k], b[k]])]));
const xyz = ['x', 'y', 'z'];

test('compact rollback rebuilds portal geometry after its clipped interval disappears', () => {
    const inner = { ...body([1, .2, -.1], [12, .4, .15]), count: 2 },
        outer = { ...body([0, 0, 0], [10, .03, -.02]), count: 2 };
    const samples = build(inner, outer, 0, 0, .0405, .15);
    const expected = structuredClone(samples.samples);
    const record = { portalSideGradients: samples.samples[0].gradients, normalGradients: samples.samples[0].gradients };
    const constraint = { innerBody: inner, outerBody: outer, kirchhoffContacts: [record],
        _kirchhoffRuntimeRecordPool: [[record]], _jointPortalSideSamples: [samples] };
    const snapshot = capture(constraint, { physicalStateOnly: true, frozenFrictionBatches: true, reusePropertyLayout: true });
    assert.ok(!snapshot.records.some(r => r.object === samples || r.object === record.portalSideGradients));
    inner.x.fill(20);
    build(inner, outer, 0, 0, .0405, .15, samples);
    assert.equal(samples.samples.length, 0);
    restore(snapshot);
    assert.deepEqual(build(inner, outer, 0, 0, .0405, .15, samples).samples, expected);
    assert.equal(record.portalSideGradients, samples.samples[0].gradients);
});

for (const [a, b] of [[[1, .2, -.1], [12, .4, .15]], [[-2, .2, -.1], [8, .4, .15]]])
test(`clipped side endpoint Jacobians match finite differences and conserve moment (${a[0]})`, () => {
    const inner = body(a, b), outer = body([0, 0, 0], [10, .03, -.02]);
    const result = build(inner, outer, 0, 0, .0405, .15);
    assert.equal(result.samples.length, 2);
    for (const sample of result.samples) {
        const index = sample.endpoint;
        const force = [0, 0, 0], moment = [0, 0, 0];
        for (let side = 0; side < 2; side++) for (let node = 0; node < 2; node++) for (let axis = 0; axis < 3; axis++) {
            const target = side ? outer : inner, original = target[xyz[axis]][node], epsilon = 1e-6;
            target[xyz[axis]][node] = original + epsilon;
            const plus = build(inner, outer, 0, 0, .0405, .15).samples[index].gap;
            target[xyz[axis]][node] = original - epsilon;
            const minus = build(inner, outer, 0, 0, .0405, .15).samples[index].gap;
            target[xyz[axis]][node] = original;
            const gradient = sample.gradients.find(g => g.side === side && g.dof === node * 6 + axis)?.value ?? 0;
            assert.ok(Math.abs(gradient - (plus - minus) / (2 * epsilon)) < 2e-8);
            force[axis] += gradient;
            const p = xyz.map(key => target[key][node]), v = [0, 0, 0]; v[axis] = gradient;
            moment[0] += p[1] * v[2] - p[2] * v[1];
            moment[1] += p[2] * v[0] - p[0] * v[2];
            moment[2] += p[0] * v[1] - p[1] * v[0];
        }
        assert.ok(Math.hypot(...force) < 1e-12);
        assert.ok(Math.hypot(...moment) < 1e-12);
    }
});

test('equal-depth endpoint exchange retains two distinct witness identities', () => {
    const inner = body([0, .1, 0], [10, .1, 0]), outer = body([0, 0, 0], [10, 0, 0]);
    const out = build(inner, outer, 0, 0, .0405), identities = [...out.samples];
    for (const tilt of [-1e-5, 0, 1e-5, -1e-5]) {
        inner.y[1] = .1 + tilt;
        build(inner, outer, 0, 0, .0405, 0, out);
        assert.equal(out.samples[0], identities[0]); assert.equal(out.samples[1], identities[1]);
        assert.equal(out.samples[0].innerT, 0); assert.equal(out.samples[1].innerT, 1);
        const largestRadius = Math.max(...out.samples.map(s => s.radius));
        for (let i = 0; i <= 20; i++) assert.ok(.1 + tilt * i / 20 <= largestRadius + 1e-14);
    }
});

test('open, parallel and empty clipped intervals have finite deterministic geometry', () => {
    const outer = body([0, 0, 0], [10, 0, 0]);
    assert.equal(build(body([11, .1, 0], [12, .1, 0]), outer, 0, 0, .0405).samples.length, 0);
    const crossing = build(body([5, -.1, 0], [5, .1, 0]), outer, 0, 0, .0405);
    assert.equal(crossing.samples.length, 2);
    for (const s of crossing.samples) assert.ok(s.gradients.every(g => Number.isFinite(g.value)));
    const axis = build(body([2, 0, 0], [8, 0, 0]), outer, 0, 0, .0405);
    assert.ok(axis.samples.every(s => s.gap === .0405 && Math.abs(Math.hypot(...s.normal) - 1) < 1e-14));
});

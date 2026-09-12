import test from 'node:test';
import assert from 'node:assert/strict';
import { measureKirchhoffFrictionMerit } from '../src/physics/kirchhoffFrictionMerit.js';

function batch(normal = 1, lambda = [0, 0], slip = [0, 0]) {
    const body = () => ({ count: 2, activeStart: 0, activeEnd: 1,
        inverseMass: [2, 2], inverseInertia1: [3], inverseInertia2: [4], inverseInertia3: [5] });
    return { entries: [{ contact: { normalLambda: normal }, surface: {
        group: { mu: [.2, .2] }, _surfaceScratch: { bodies: [body(), body()] },
        rows: [0, 1].map(axis => ({ lambda: lambda[axis], strain: slip[axis], gradients: [
            { side: 0, dof: axis, value: 1 }, { side: 1, dof: axis, value: -1 }
        ] }))
    } }] };
}

test('friction line-search merit stays continuous as normal load appears and vanishes', () => {
    const out = {};
    assert.equal(measureKirchhoffFrictionMerit(batch(0, [0, 0], [1, .5]), out).maximumMm, 0);
    let previous = Infinity;
    for (const normal of [1, .1, 1e-3, 1e-6, 1e-12]) {
        const value = measureKirchhoffFrictionMerit(batch(normal, [0, 0], [1, .5]), out).maximumMm;
        assert.ok(value < previous); previous = value;
    }
    assert.ok(previous < 1e-11);
    assert.equal(measureKirchhoffFrictionMerit(batch(0, [.1, 0], [1, .5]), out).maximumMm, .4);
});

test('merit has the exact sticking and sliding zeros without changing input forces', () => {
    for (const [lambda, slip] of [[[.1, .1], [0, 0]], [[-.2, 0], [1, 0]], [[0, -.2], [0, 1]]]) {
        const value = batch(1, lambda, slip), before = structuredClone(value);
        assert.ok(measureKirchhoffFrictionMerit(value).maximumMm < 1e-14);
        assert.deepEqual(value, before);
    }
    assert.ok(measureKirchhoffFrictionMerit(batch(1, [0, 0], [1, 0])).maximumMm > 0);
});

test('mobility sums duplicate gradients before squaring and respects prescribed frames', () => {
    const value = batch(0, [1, 0]), entry = value.entries[0];
    entry.surface.rows[0].gradients = [{ side: 0, dof: 0, value: 1 }, { side: 0, dof: 0, value: -1 },
        { side: 0, dof: 3, value: 2 }];
    entry.surface.rows[1].gradients = [];
    assert.equal(measureKirchhoffFrictionMerit(value).maximumMm, 12);
    Object.assign(entry.surface._surfaceScratch.bodies[0], { orientationControlCompliance: 0, orientationControlSegment: 0 });
    assert.equal(measureKirchhoffFrictionMerit(value).maximumMm, 1, 'zero-mobility reference scale');
});

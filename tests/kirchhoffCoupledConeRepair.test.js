import assert from 'node:assert/strict';
import test from 'node:test';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { prepareKirchhoffCoupledConeRepair as prepare, applyKirchhoffCoupledConeRepair as apply } from '../src/physics/kirchhoffCoupledConeRepair.js';
import { measureRodSystemCorrection } from './helpers/rodSystemMomentum.js';

const dt = 1 / 120;
function fixture(lambda = [.2000000001, 0], mu = [.2, .2]) {
    const world = new EndovascularPhysicsWorld({ fixedDt: dt });
    const inner = world.createRod('wire', 2, 5, { radius: .4445, mass: 1 });
    const outer = world.createRod('catheter', 2, 5, { radius: .8, mass: 3 });
    for (let i = 0; i < 2; i++) inner.setNodePosition(i, i * 5, .0405, 0);
    const contact = { normalLambda: 1, tangentLambda: Float64Array.from(lambda), twistLambda: 0,
        innerTwistImpulse: 0, outerTwistImpulse: 0, tangentU: [1, 0, 0], tangentV: [0, 0, -1],
        normal: [0, 1, 0], innerSegmentIndex: 0, outerSegmentIndex: 0 };
    const record = { id: 'repair-contact', kind: 'side', gap: 0, normal: [0, 1, 0],
        _innerSegmentIndex: 0, _outerSegmentIndex: 0, innerT: .5, outerT: .5,
        innerWeights: [.5, .5], outerWeights: [.5, .5], manifoldContact: contact };
    const constraint = { innerBody: inner, outerBody: outer, innerRadius: .485,
        axialFriction: mu[0], circumferentialFriction: mu[1], kirchhoffContacts: [record] };
    return { world, inner, outer, constraint, contact, record };
}
const state = f => JSON.stringify([...[f.inner, f.outer].map(b => Object.fromEntries([
    'x', 'y', 'z', 'orientationX', 'orientationY', 'orientationZ', 'orientationW',
    'previousX', 'previousY', 'previousZ', 'velocityX', 'velocityY', 'velocityZ',
    'angularVelocityX', 'angularVelocityY', 'angularVelocityZ', 'adaptationLambdaX'
].map(key => [key, [...b[key]]]))), f.contact]);

test('numerical cone excess receives reciprocal positions and local-frame moments with exactly the projected force', () => {
    const f = fixture(), before = state(f), velocities = [f.inner, f.outer].map(b => [...b.velocityX]);
    const plan = prepare(f.constraint, dt);
    assert.equal(state(f), before, 'preparation is read-only for mechanics and multipliers');
    assert.equal(plan.accepted, true); assert.equal(plan.changedContacts, 1);
    assert.ok(plan.maximumPositionCorrectionMm > 0 && plan.maximumPositionCorrectionMm < 1e-9);
    assert.ok(plan.maximumAngleCorrectionRad > 0 && plan.maximumAngleCorrectionRad < 1e-9);
    const wrench = measureRodSystemCorrection([f.inner, f.outer], plan.responses, 1, dt);
    assert.ok(Math.hypot(...wrench.linearImpulse) < 1e-20);
    assert.ok(Math.hypot(...wrench.momentImpulse) < 1e-20);
    const previous = [...f.inner.previousX], material = [...f.inner.adaptationLambdaX];
    apply(plan);
    assert.equal(plan.status, 'applied-needs-residual-check');
    assert.ok(Math.hypot(...f.contact.tangentLambda) / .2 <= 1 + 1e-12);
    assert.equal(f.contact.normalLambda, 1);
    assert.ok(f.inner.x[0] < 0 && f.outer.x[0] > 0, 'both free rods receive matched translation');
    assert.notEqual(f.inner.orientationZ[0], plan.responses[0].before.orientationZ[0]);
    assert.deepEqual([...f.inner.previousX], previous);
    assert.deepEqual([...f.inner.adaptationLambdaX], material);
    assert.deepEqual([f.inner, f.outer].map(b => [...b.velocityX]), velocities);
    assert.throws(() => apply(plan), /already applied/);
});

test('anisotropic ellipse and zero-load repair preserve fixed normal force', () => {
    for (const mu of [[.2, .4], [0, .2], [0, 0]]) {
        const f = fixture([.3, -.5], mu);
        if (mu.every(v => v === 0)) f.contact.normalLambda = 0;
        const normal = f.contact.normalLambda;
        const plan = prepare(f.constraint, dt, { maximumPositionCorrectionMm: 10, maximumAngleCorrectionRad: 10 });
        apply(plan);
        const lambda = [...f.contact.tangentLambda];
        const axes = mu.map(v => v * normal);
        for (let i = 0; i < 2; i++) if (!axes[i]) assert.ok(Math.abs(lambda[i]) < 1e-15);
        if (axes.every(v => v > 0)) assert.ok(Math.hypot(...lambda.map((v, i) => v / axes[i])) <= 1 + 1e-12);
        assert.equal(f.contact.normalLambda, normal);
    }
});

test('pinned translations and a hard prescribed local frame use zero mobility', () => {
    const f = fixture([.3, -.2]);
    f.outer.setPinned(0, true);
    f.outer.orientationControlCompliance = 0;
    f.outer.orientationControlSegment = 0;
    const before = [f.outer.x[0], f.outer.y[0], f.outer.z[0], f.outer.orientationX[0], f.outer.orientationY[0], f.outer.orientationZ[0], f.outer.orientationW[0]];
    const plan = prepare(f.constraint, dt, { maximumPositionCorrectionMm: 1, maximumAngleCorrectionRad: 1 });
    assert.ok(plan.responses[1].correction.slice(0, 6).every(value => value === 0));
    apply(plan);
    assert.deepEqual([f.outer.x[0], f.outer.y[0], f.outer.z[0], f.outer.orientationX[0], f.outer.orientationY[0], f.outer.orientationZ[0], f.outer.orientationW[0]], before);
});

test('large correction is rejected atomically and stale load, pose or mobility cannot apply', () => {
    const f = fixture([.4, -.3]), before = state(f), rejected = prepare(f.constraint, dt);
    assert.equal(rejected.accepted, false);
    assert.throws(() => apply(rejected), /exceeds/);
    assert.equal(state(f), before);
    for (const mutation of [f => f.contact.normalLambda *= 2, f => f.inner.x[0] += 1, f => f.inner.inverseMass[0] *= 2]) {
        const f = fixture(), plan = prepare(f.constraint, dt);
        mutation(f); const beforeApply = state(f);
        assert.throws(() => apply(plan), /changed/);
        assert.equal(state(f), beforeApply);
    }
});

test('already feasible contact is a mechanical and multiplier no-op', () => {
    const f = fixture([.1, 0]), before = state(f), plan = prepare(f.constraint, dt);
    assert.equal(plan.changedContacts, 0);
    apply(plan);
    assert.equal(plan.status, 'already-feasible');
    assert.equal(state(f), before);
});

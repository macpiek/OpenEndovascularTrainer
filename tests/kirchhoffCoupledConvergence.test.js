import assert from 'node:assert/strict';
import test from 'node:test';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { measureKirchhoffContactMotion } from '../src/physics/kirchhoffCoupledConvergence.js';

const profile = {
    radius: 0.2, innerRadius: 0.6, mass: 1,
    linearDamping: 1, angularDamping: 1,
    foldLimitStrength: 0, projectionVelocityRetention: 1,
    sleepFrames: 1_000_000
};

function snapshot(body) {
    body.postPassStartX.set(body.x);
    body.postPassStartY.set(body.y);
    body.postPassStartZ.set(body.z);
}

test('contact convergence is invariant to remote relaxation and common translation', () => {
    const world = new EndovascularPhysicsWorld();
    const inner = world.createRod('wire', 201, 5, profile);
    const outer = world.createRod('catheter', 5, 5, profile);
    snapshot(inner);
    snapshot(outer);
    const record = {
        manifoldContact: { normalLambda: 1, tangentLambda: [0, 0] },
        gap: 0, normal: [0, 1, 0],
        _innerSegmentIndex: 1, _outerSegmentIndex: 1,
        innerWeights: [0.25, 0.75], outerWeights: [0.25, 0.75]
    };
    const constraint = { innerBody: inner, outerBody: outer, axialFriction: 0, kirchhoffContacts: [record] };
    inner.y[150] += 20;
    assert.equal(measureKirchhoffContactMotion(constraint), 0);
    for (const b of [inner, outer]) { b.y[1] += 2; b.y[2] += 2; }
    assert.equal(measureKirchhoffContactMotion(constraint), 0);
    inner.y[2] += 0.02;
    assert.ok(Math.abs(measureKirchhoffContactMotion(constraint) - 0.015) < 1e-6);
});

test('cubic contact support, sticking and admissible sliding use their actual coordinates', () => {
    const world = new EndovascularPhysicsWorld();
    const inner = world.createRod('wire', 8, 5, profile);
    const outer = world.createRod('catheter', 8, 5, profile);
    snapshot(inner);
    snapshot(outer);
    const record = {
        manifoldContact: { normalLambda: 1, tangentLambda: [0, 0] },
        gap: 0, normal: [0, 1, 0],
        _innerNodeIndices: [0, 1, 2, 3], _innerNodeWeights: [0.1, 0.4, 0.4, 0.1], _innerNodeCount: 4,
        _outerNodeIndices: [0, 1, 2, 3], _outerNodeWeights: [0.1, 0.4, 0.4, 0.1], _outerNodeCount: 4
    };
    const constraint = { innerBody: inner, outerBody: outer, axialFriction: 0.1, kirchhoffContacts: [record] };
    inner.x[3] += 1;
    assert.ok(Math.abs(measureKirchhoffContactMotion(constraint) - 0.1) < 1e-6, 'a cubic neighbour participates in sticking');
    record.manifoldContact.tangentLambda[0] = 0.1;
    assert.equal(measureKirchhoffContactMotion(constraint), 0, 'slip at the Coulomb limit is admissible');
    inner.y[0] = 0.2;
    assert.ok(Math.abs(measureKirchhoffContactMotion(constraint) - 0.02) < 1e-6, 'sliding still constrains normal motion');
    record.manifoldContact.normalLambda = 0;
    record.gap = 0.1;
    assert.equal(measureKirchhoffContactMotion(constraint), 0, 'an open unloaded contact has no sticking constraint');
});

function relaxingWire(withCatheter) {
    const world = new EndovascularPhysicsWorld();
    world.captureCoupledClosureTrace = true;
    const wire = world.createRod('wire', 201, 5, profile);
    wire.setPinned(0, true);
    for (let i = 125; i <= 155; i++) wire.y[i] = 0.3 * Math.sin(Math.PI * (i - 125) / 30);
    wire.captureKirchhoffRestConfiguration({ captureRestRotation: false });
    // A distal wall correction arriving after ordinary body stabilization.
    // It must receive the same global relaxation policy with an open lumen.
    wire.debugConstraintPhase = phase => {
        if (phase === 'final') wire.y[140] += 0.5;
    };
    if (withCatheter) {
        const catheter = world.createRod('catheter', 3, 5, profile);
        world.addContainment(wire, catheter, { innerRadius: 0.6, endNode: 2, containedLength: 10 });
    }
    world.stepFixed();
    return { world, wire };
}

test('an unloaded 1 cm catheter does not demand equilibrium of a remote wire bend', () => {
    const solo = relaxingWire(false);
    const coupled = relaxingWire(true);
    assert.equal(coupled.world.lastCoupledClosureConverged, true);
    assert.equal(coupled.world.lastCoupledClosurePasses, solo.world.lastCoupledClosurePasses);
    assert.ok(coupled.world.coupledClosureTrace.at(-1).bodies[0].maximumPositionDelta > 0.001,
        'the fixture must actually exercise distant relaxation above the old threshold');
    for (const axis of ['x', 'y', 'z']) for (let i = 0; i < solo.wire.count; i++) {
        assert.ok(Math.abs(coupled.wire[axis][i] - solo.wire[axis][i]) < 1e-5, `remote response changed at ${axis}/${i}`);
    }
});

test('side quadrature and distal side contact own distinct records and multipliers', () => {
    const world = new EndovascularPhysicsWorld();
    const wire = world.createRod('wire', 9, 5, profile);
    const catheter = world.createRod('catheter', 5, 5, profile);
    wire.y.fill(0.5);
    const constraint = world.addContainment(wire, catheter, { innerRadius: 0.6, endNode: 4, containedLength: 20 });
    world.stepFixed();
    const records = constraint.kirchhoffContacts;
    assert.ok(records.some(r => r.kind === 'side'));
    assert.ok(records.some(r => r.kind === 'material-side'));
    assert.equal(new Set(records).size, records.length, 'a portal record must not overwrite a quadrature point');
    assert.equal(new Set(records.map(r => r.manifoldContact)).size, records.length,
        'independent contacts must not share a normal multiplier');
});

function loadedCoupledRods(contactPasses) {
    const world = new EndovascularPhysicsWorld({ coupledContactMaxPasses: contactPasses });
    const stiff = { ...profile, kirchhoffBendCompliance: 2e-7, kirchhoffTwistCompliance: 2e-7 };
    const wire = world.createRod('wire', 61, 2, stiff);
    const catheter = world.createRod('catheter', 11, 2, { ...stiff, mass: 3 });
    wire.y.fill(0.7);
    world.addContainment(wire, catheter, {
        innerRadius: 0.6, endNode: 10, containedLength: 20,
        axialFriction: 0, torsionalFriction: 0,
        radialVelocityDamping: 0, coupledBendingRateDamping: 0
    });
    world.stepFixed();
    return { world, wire, catheter };
}

test('short lumen contact retains full-length force transmission and reciprocal momentum', () => {
    // Verify physical invariants with and without contact batching. Iteration
    // order may change finite-tolerance trajectories, but cannot cut the rod
    // response at the overlap or create an unbalanced internal force.
    for (const passes of [1, 32]) {
        const { world, wire, catheter } = loadedCoupledRods(passes);
        assert.ok(catheter.y[0] > 0, 'the catheter must react to the wire');
        assert.ok(wire.y[40] < 0.7 - 1e-5, 'contact at 0–20 mm must load the free rod at 80 mm in the same step');
        const massMoment = wire.y.reduce((a, b) => a + b, 0) + 3 * catheter.y.reduce((a, b) => a + b, 0);
        assert.ok(Math.abs(massMoment - 61 * Math.fround(0.7)) < 1e-4, 'internal contact must preserve the mass-weighted center');
        assert.equal(world.lastCoupledClosureConverged, true);
    }
});

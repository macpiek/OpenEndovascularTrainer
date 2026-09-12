import assert from 'node:assert/strict';
import test from 'node:test';
import { fixture, DT, close, VELOCITY, POSITION } from './fixtures/splitMotionAnalyticWorld.js';
import { configureKirchhoffSplitBias } from '../src/physics/kirchhoffSplitMotion.js';

function jointFixture(options) {
    const f = fixture(options);
    configureKirchhoffSplitBias(f.constraint, { materialMode: 'coupled-compliance' });
    return f;
}
function accepted(f) {
    const before = f.world.stepCount, result = f.world.stepFixed();
    assert.equal(result.accepted, true, JSON.stringify(result));
    assert.equal(f.world.stepCount, before + 1);
    assert.equal(result.diagnostics.historyCommits, 1);
    assert.equal(result.diagnostics.biasPasses, 0, 'there is no second bias solve');
    assert.ok(result.diagnostics.jointPasses > 0);
    assert.equal(f.constraint._jointDiagnostics.channels, 2);
    assert.ok(f.constraint._jointDiagnostics.reconstructionResidual <= f.world.coupledContainmentTolerance * .2);
    const m = f.constraint._jointMaterialResidual;
    assert.ok(m.physicalResidual.adaptationMm <= f.world.coupledContainmentTolerance);
    assert.ok(m.biasResidual.adaptationMm <= f.world.coupledContainmentTolerance);
    assert.ok(m.physicalResidual.bendTwistRad <= f.world.coupledAngularToleranceRad);
    assert.ok(m.biasResidual.bendTwistRad <= f.world.coupledAngularToleranceRad);
    return result.diagnostics;
}

for (const tolerance of [.001, POSITION]) test(`joint World repairs initial wall overlap at requested ${tolerance} mm accuracy without changing physical motion`, () => {
    const f = jointFixture({ wall: true, y: -.25 }), initialY = f.wire.y.slice();
    // The analytic position oracle requires requesting matching solver
    // accuracy. Also exercise the application's unchanged default tolerance.
    f.world.coupledContainmentTolerance = tolerance;
    f.wire.velocityX.fill(3);
    const d = accepted(f);
    assert.ok(d.contacts.some(c => c.normalBias > 0));
    for (let i = 0; i < f.wire.count; i++) {
        assert.ok(f.wire.y[i] + f.wire.nodeRadius[i] <= tolerance);
        close(f.constraint._splitMotion.twoChannel.physicalPose[0].y[i], initialY[i], POSITION, 'bias does not move physical trajectory');
        close(f.wire.velocityX[i], 3, VELOCITY, 'tangential incoming motion');
        close(f.wire.velocityY[i], 0, VELOCITY, 'geometric correction adds no physical normal speed');
    }
    const normalImpulse = d.contacts.reduce((sum, c) => sum + c.normalPhysical / DT, 0);
    close(normalImpulse, 0, f.wire.count * VELOCITY, 'bias creates no resolvable physical impulse at fixture precision');
});

test('joint World retains independent axial sliding with positive friction but no physical normal budget', () => {
    const f = jointFixture({ nested: true, y: .75, mu: .3 });
    f.wire.velocityX.fill(4);
    const d = accepted(f);
    assert.ok(d.contacts.some(c => c.kind === 'lumen' && c.normalBias > 0));
    for (const v of f.wire.velocityX) close(v, 4, VELOCITY, 'free axial sliding');
    // Each free node has unit mass. Compare the summed impulses in momentum
    // units with the fixture's existing velocity precision; an arbitrary
    // multiplier epsilon would change with dt and contact redundancy.
    const normalImpulse = d.contacts.reduce((sum, c) => sum + c.normalPhysical / DT, 0);
    const tangentImpulse = d.contacts.reduce((sum, c) => sum + Math.hypot(...c.tangentPhysical) / DT, 0);
    close(normalImpulse, 0, f.wire.count * VELOCITY, 'no physical normal budget');
    close(tangentImpulse, 0, f.wire.count * VELOCITY, 'no physical friction impulse');
    for (const v of f.wire.velocityY) close(v, 0, VELOCITY, 'no normal kinetic motion from bias');
});

test('joint World normal and friction impulses balance the momentum of both tools after one force prediction', () => {
    const f = jointFixture({ nested: true, y: .5, mu: .3 });
    f.wire.velocityX.fill(4); f.wire.forceY.fill(1 / DT);
    const d = accepted(f), normalImpulse = d.contacts.reduce((s, c) => s + c.normalPhysical / DT, 0);
    const py = [...f.wire.velocityY].reduce((s, v, i) => s + v / f.wire.inverseMass[i], 0);
    close(normalImpulse, 3 - py, .002, 'normal impulse balances momentum');
    assert.ok(normalImpulse > 0);
    assert.ok([...f.wire.velocityX].reduce((s, v) => s + v, 0) < 11.99);
    assert.ok(f.wire.forceY.every(v => v === 0));
    assert.ok(d.physicalConeViolation <= 1e-9);
});

test('rejected joint World trial restores physical pose, geometry, velocity and pending input before retry', () => {
    const f = jointFixture({ wall: true, y: -.25 });
    const originalApply = f.world.coupledSystem.apply;
    let reject = true, callbacks = 0;
    f.world.coupledSystem.apply = (joint, result) => {
        originalApply(joint, result);
        if (reject) throw new Error('injected-after-total-pose');
    };
    const before = ['x', 'y', 'z', 'velocityX', 'velocityY', 'velocityZ'].map(k => f.wire[k].slice());
    const prepare = () => { callbacks++; f.wire.forceX.fill(1 / DT); };
    assert.throws(() => f.world.advance(DT, prepare), /injected-after-total-pose/);
    assert.equal(callbacks, 1); assert.equal(f.world.stepCount, 0); assert.equal(f.world.accumulator, DT);
    ['x', 'y', 'z', 'velocityX', 'velocityY', 'velocityZ'].forEach((k, i) => assert.deepEqual(f.wire[k], before[i]));
    assert.equal(f.constraint._splitMotion, undefined);
    assert.ok(f.wire.forceX.every(v => v === 120));
    reject = false;
    assert.equal(f.world.advance(0, prepare), 1);
    assert.equal(callbacks, 1); assert.equal(f.world.stepCount, 1); assert.equal(f.world.accumulator, 0);
    for (const v of f.wire.velocityX) close(v, 1, VELOCITY, 'force kick occurs exactly once after rollback');
});

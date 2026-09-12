import assert from 'node:assert/strict';
import test from 'node:test';
import { EndovascularPhysicsWorld, DEFAULT_TOOL_PROFILES } from '../src/physics/endovascularPhysicsWorld.js';
import { RodState } from '../src/physics/rodState.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../src/physics/kirchhoffCoupledSystem.js';
import { measureKirchhoffCoupledMaterialResidual } from '../src/physics/kirchhoffCoupledResidual.js';

const coupledSystem = { solve: solveKirchhoffCoupledSystem, apply: applyKirchhoffCoupledCorrection };
const profile = { radius: 0.4, foldLimitStrength: 0, linearDamping: 1, angularDamping: 1,
    projectionVelocityRetention: 1, sleepFrames: 1e6 };

test('Kirchhoff wire profile does not inherit the historical positional solver angle cap', () => {
    const world = new EndovascularPhysicsWorld(), storage = new RodState(3, 5);
    storage.nodeStorage.bendAngleLimit.fill(10);
    const wire = world.createRod('wire', 3, 5, DEFAULT_TOOL_PROFILES.guidewire);
    wire.syncFromRodState(storage);
    assert.ok([...wire.maxBendAngleByNode].every(value => value === DEFAULT_TOOL_PROFILES.guidewire.maxBendAngle));
    const explicit = world.createRod('explicit', 3, 5, { inheritRodStateBendLimit: true });
    explicit.syncFromRodState(storage);
    assert.ok([...explicit.maxBendAngleByNode].every(value => value === 10));
});

test('runtime reports the actual joint path even when external tool contact is enabled', () => {
    const world = new EndovascularPhysicsWorld({ coupledSystem });
    const inner = world.createRod('wire', 5, 5, profile);
    const outer = world.createRod('catheter', 5, 5, profile);
    inner.y.fill(0.15);
    world.addContainment(inner, outer, { innerRadius: 0.5, axialFriction: 0, torsionalFriction: 0 });
    world.addToolContact(inner, outer, { enabled: true, openDistalB: true,
        startSegmentA: 3, endSegmentA: 3, startSegmentB: 0, endSegmentB: 0 });
    world.stepFixed();
    const stats = world.getStats();
    assert.equal(stats.coupledSolver, 'joint');
    assert.ok(stats.jointMaximumRows > 0);
    assert.equal(stats.coupledClosureConverged, true);
});

test('a connected sleeping rod wakes before its prediction state is captured', () => {
    const world = new EndovascularPhysicsWorld({ coupledSystem });
    const inner = world.createRod('wire', 3, 5, profile);
    const outer = world.createRod('catheter', 3, 5, profile);
    world.addContainment(inner, outer, { innerRadius: 1, axialFriction: 0, torsionalFriction: 0 });
    inner.sleeping = true;
    inner.previousX.fill(-100);
    world.stepFixed();
    assert.equal(inner.sleeping, false);
    for (let i = 0; i < inner.count; i++) assert.ok(Math.abs(inner.previousX[i] - i * 5) < 1e-6);
    assert.equal(world.getStats().coupledSolver, 'joint');
});

test('a settled coupled pair sleeps atomically even with staggered eligibility counters', () => {
    const world = new EndovascularPhysicsWorld({ coupledSystem });
    const p = { ...profile, sleepFrames: 2 };
    const inner = world.createRod('wire', 3, 5, p), outer = world.createRod('catheter', 3, 5, p);
    world.addContainment(inner, outer, { innerRadius: 1, axialFriction: 0, torsionalFriction: 0 });
    inner.sleepCounter = 1;
    world.stepFixed();
    assert.equal(world.lastCoupledClosureConverged, true);
    assert.equal(inner.sleeping, false, 'one member cannot sleep and be woken/reset by its awake neighbour');
    assert.equal(outer.sleeping, false);
    assert.ok(inner.sleepCounter >= 2); assert.equal(outer.sleepCounter, 1);
    world.stepFixed();
    assert.equal(inner.sleeping && outer.sleeping, true);
    world.stepFixed(); assert.equal(world.lastCoupledSolver, 'sleeping');
    outer.setControlTarget(0, 0, .01, 0, 1e-6);
    world.stepFixed();
    assert.equal(world.lastCoupledSolver, 'joint');
    assert.equal(inner.sleeping || outer.sleeping, false, 'a new input wakes and solves the complete component');
});

test('nonlinear material residual measures the applied state, including constraint multipliers', () => {
    const world = new EndovascularPhysicsWorld();
    const innerBody = world.createRod('wire', 4, 5, profile);
    const outerBody = world.createRod('catheter', 4, 5, profile);
    const constraint = { innerBody, outerBody, kirchhoffContacts: [] };
    assert.ok(measureKirchhoffCoupledMaterialResidual(constraint, 1 / 120).adaptationMm < 1e-12);
    innerBody.y[2] = 0.1;
    const state = measureKirchhoffCoupledMaterialResidual(constraint, 1 / 120);
    assert.ok(Math.abs(state.adaptationMm - 0.1) < 1e-7);
    assert.equal(state.worstAdaptationSide, 0);
    innerBody.adaptationCompliance = 1 / 14400;
    innerBody.adaptationLambdaY[1] = -0.1;
    innerBody.adaptationLambdaY[2] = 0.1;
    assert.ok(measureKirchhoffCoupledMaterialResidual(constraint, 1 / 120).adaptationMm < 1e-7);
});


test('an empty catheter active range still solves the guidewire material equations', () => {
    const world = new EndovascularPhysicsWorld({ coupledSystem });
    const inner = world.createRod('wire', 3, 2, profile);
    const outer = world.createRod('catheter', 3, 2, profile);
    outer.setActiveRange(0, 0);
    inner.y[1] = 1;
    const c = world.addContainment(inner, outer, { innerRadius: 10, axialFriction: 0, torsionalFriction: 0 });
    world.stepFixed();
    assert.equal(world.lastCoupledClosureConverged, true);
    assert.ok(measureKirchhoffCoupledMaterialResidual(c, world.fixedDt).adaptationMm < 0.001);
    assert.ok(inner.y[1] < 0.9);
});

test('an unconverged joint component cannot sleep or report a successful idle step', () => {
    const world = new EndovascularPhysicsWorld({ coupledSystem, coupledClosureMaxPasses: 8 });
    const p = { ...profile, projectionVelocityRetention: 0, sleepFrames: 1 };
    const inner = world.createRod('wire', 3, 2, p);
    const outer = world.createRod('catheter', 3, 2, p);
    inner.y[1] = 50;
    world.addContainment(inner, outer, { innerRadius: 100, openDistal: false,
        axialFriction: 0, torsionalFriction: 0, radialVelocityDamping: 0, coupledBendingRateDamping: 0 });
    world.stepFixed();
    assert.equal(world.lastCoupledClosureConverged, false);
    assert.equal(inner.sleeping || outer.sleeping, false);
    // Defend the idle fast path too, even if a caller marks this state asleep.
    inner.sleeping = outer.sleeping = true;
    world.stepFixed();
    assert.equal(world.lastCoupledSolver, 'joint');
    assert.equal(world.lastCoupledClosureConverged, false);
});

test('joint acceptance queries the curved wall again instead of trusting its tangent plane', async () => {
    const { createContactResult } = await import('../src/physics/collision/vesselContactField.js');
    const contactField = {
        queryCapsule(start, end, radius, out = createContactResult()) {
            const p = Math.hypot(start.y, start.z) >= Math.hypot(end.y, end.z) ? start : end;
            const d = Math.hypot(p.y, p.z), ny = -p.y / d, nz = -p.z / d;
            Object.assign(out, { signedDistance: 5 - d, signedGap: 5 - d - radius,
                penetration: Math.max(0, d + radius - 5), inside: d <= 5,
                violation: d + radius > 5, branchId: 0, faceIndex: 0, segmentT: p === start ? 0 : 1 });
            Object.assign(out.inward, { x: 0, y: ny, z: nz });
            Object.assign(out.closestPoint, { x: p.x, y: p.y - ny * (5 - d), z: p.z - nz * (5 - d) });
            return out;
        },
        sweepSphere(a, b, radius, out = createContactResult()) {
            out.violation = false; out.timeOfImpact = 1; return out;
        }
    };
    const world = new EndovascularPhysicsWorld({ coupledSystem, contactField });
    const p = { ...profile, radius: 0.5, wallCompliance: 0 };
    const inner = world.createRod('wire', 3, 2, p), outer = world.createRod('catheter', 3, 2, p);
    for (const body of [inner, outer]) {
        body.y.fill(4.5);
        for (let i = 0; i < 3; i++) body.setControlTarget(i, i * 2, 4.5, 1, 0);
    }
    const c = world.addContainment(inner, outer, { innerRadius: 2, openDistal: false,
        axialFriction: 0, torsionalFriction: 0 });
    world.stepFixed();
    // The prescribed point is outside the curved lumen but on its old plane.
    assert.ok(world.settledMaxPenetration > 0.02);
    assert.ok(Math.abs(world.settledMaxPenetration - (Math.hypot(inner.y[0], inner.z[0]) + 0.5 - 5)) < 1e-7);
    assert.equal(world.lastCoupledClosureConverged, false);
    assert.equal(c._jointLinearFailure.converged, false);
});


test('compliant orientation control participates in the joint solve and final residual', async () => {
    const { evaluateBendTwistConstraint, quaternionExp, multiplyQuaternions } =
        await import('../src/physics/discreteKirchhoffRod.js');
    const world = new EndovascularPhysicsWorld({ coupledSystem });
    const inner = world.createRod('wire', 3, 2, profile), outer = world.createRod('catheter', 3, 2, profile);
    inner.inverseMass.fill(0);
    const current = () => ({ x: inner.orientationX[0], y: inner.orientationY[0],
        z: inner.orientationZ[0], w: inner.orientationW[0] });
    const target = multiplyQuaternions(quaternionExp({ x: 0, y: 0, z: 0.2 }), current());
    inner.setProximalOrientationControl(target.x, target.y, target.z, target.w, 1e-4);
    world.addContainment(inner, outer, { innerRadius: 10, openDistal: false, axialFriction: 0, torsionalFriction: 0 });
    world.stepFixed();
    const strain = evaluateBendTwistConstraint(target, current()).strain;
    const residual = ['x', 'y', 'z'].map((axis, i) => strain[axis] + 1.44 * inner.orientationControlLambda[i]);
    assert.equal(world.lastCoupledClosureConverged, true);
    assert.ok(Math.hypot(...residual) < 0.001);
    assert.ok(Math.hypot(...inner.orientationControlLambda) > 0.1);
});

test('hard orientation is prescribed before building frame-dependent joint rows', async () => {
    const { quaternionExp, multiplyQuaternions } = await import('../src/physics/discreteKirchhoffRod.js');
    let target, calls = 0;
    const world = new EndovascularPhysicsWorld({ coupledSystem: {
        solve(c, dt, options) {
            const body = c.innerBody;
            for (const axis of ['x', 'y', 'z', 'w']) assert.equal(body['orientation' + axis.toUpperCase()][0], target[axis]);
            calls++;
            return solveKirchhoffCoupledSystem(c, dt, options);
        }, apply: applyKirchhoffCoupledCorrection
    } });
    const inner = world.createRod('wire', 3, 2, profile), outer = world.createRod('catheter', 3, 2, profile);
    target = multiplyQuaternions(quaternionExp({ x: 1.6, y: 1, z: 0.6 }),
        { x: inner.orientationX[0], y: inner.orientationY[0], z: inner.orientationZ[0], w: inner.orientationW[0] });
    inner.setProximalOrientationControl(target.x, target.y, target.z, target.w, 0);
    // The public control setter normalizes its target.
    target = Object.fromEntries(['x', 'y', 'z', 'w'].map(axis => [axis, inner['orientationControl' + axis.toUpperCase()]]));
    world.addContainment(inner, outer, { innerRadius: 10, openDistal: false, axialFriction: 0, torsionalFriction: 0 });
    world.stepFixed();
    assert.ok(calls > 0);
});

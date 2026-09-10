import test from 'node:test';
import assert from 'node:assert/strict';
import { EndovascularPhysicsWorld, DEFAULT_TOOL_PROFILES } from '../src/physics/endovascularPhysicsWorld.js';
import { configureKirchhoffToolRuntime } from '../src/physics/kirchhoffToolRuntime.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../src/physics/kirchhoffCoupledSystem.js';
import { measureKirchhoffCoupledMaterialResidual } from '../src/physics/kirchhoffCoupledResidual.js';
import { PigtailCatheter } from '../src/pigtailCatheter.js';
import { RodState } from '../src/physics/rodState.js';

const coupledSystem = {
    independentComponents: true,
    solve(component, dt, options) {
        return solveKirchhoffCoupledSystem(component, dt,
            { ...options, activeCondensation: true, simultaneousCoulomb: true });
    },
    apply: applyKirchhoffCoupledCorrection
};
const mechanicalFields = ['x', 'y', 'z', 'previousX', 'previousY', 'previousZ',
    'velocityX', 'velocityY', 'velocityZ', 'orientationX', 'orientationY', 'orientationZ', 'orientationW',
    'angularVelocityX', 'angularVelocityY', 'angularVelocityZ', 'wallLambda'];
function sameMechanics(a, b, message) {
    for (const key of mechanicalFields) assert.deepEqual(a[key], b[key], `${message}: ${key}`);
}
function closure(world, body) {
    assert.equal(world.lastCoupledClosureConverged, true, 'nonlinear world closure');
    const residual = measureKirchhoffCoupledMaterialResidual({ bodies: [body] }, world.fixedDt);
    assert.ok(residual.adaptationMm <= world.coupledContainmentTolerance, 'material translation residual');
    assert.ok(residual.bendTwistRad <= world.coupledAngularToleranceRad, 'material angular residual');
}
class PlaneWall {
    voxelSize = .5;
    write(p, radius, out) {
        const gap = -p.y - radius, penetration = Math.max(0, -gap);
        Object.assign(out, { signedDistance: -p.y, signedGap: gap, penetration, inside: p.y <= 0,
            violation: gap < 0, branchId: 0, faceIndex: 0, source: 'runtime-plane', timeOfImpact: gap < 0 ? 0 : 1 });
        Object.assign(out.point, p); Object.assign(out.closestPoint, { x: p.x, y: 0, z: p.z });
        Object.assign(out.normal, { x: 0, y: -1, z: 0 }); Object.assign(out.inward, out.normal);
        Object.assign(out.target, { x: p.x, y: p.y - penetration, z: p.z });
        return out;
    }
    querySphere(p, r, out) { return this.write(p, r, out); }
    queryCapsule(a, b, r, out) {
        const p = a.y >= b.y ? a : b;
        this.write(p, r, out); out.segmentT = p === a ? 0 : 1; return out;
    }
    sweepSphere(a, b, r, out) {
        this.write(b, r, out);
        const ga = -a.y - r, gb = -b.y - r;
        out.timeOfImpact = gb >= 0 ? 1 : ga <= 0 ? 0 : ga / (ga - gb);
        return out;
    }
}

test('identical tools globally relax against a wall identically through movement, hold and reversal regardless of identity', () => {
    const fixtures = ['guidewire', 'catheter'].map(id => {
        const world = new EndovascularPhysicsWorld({ coupledSystem, contactField: new PlaneWall() });
        const body = configureKirchhoffToolRuntime(world.createRod(id, 8, 5,
            { radius: .5, linearDamping: .98, angularDamping: .96, sleepFrames: 1e6 }));
        for (let i = 0; i < body.count; i++) body.setNodePosition(i, 5 * i, -.55, 0);
        body.setPinned(0, true); body.y[5] = body.previousY[5] = -.49;
        return { world, body };
    });
    let remoteMotion = 0, contactLoad = 0;
    for (const speed of [.2, .2, 0, 0, -.2, 0]) {
        for (const { world, body } of fixtures) {
            body.x[0] += speed * world.fixedDt;
            body.previousX[0] = body.x[0]; body.velocityX[0] = speed;
            world.stepFixed(); closure(world, body);
            remoteMotion = Math.max(remoteMotion, Math.abs(body.y[1] - Math.fround(-.55)));
            contactLoad = Math.max(contactLoad, ...body.wallLambda);
            for (let i = 0; i < body.count; i++)
                assert.ok(body.y[i] + body.nodeRadius[i] <= world.coupledContainmentTolerance, 'wall nonpenetration');
        }
        sameMechanics(fixtures[0].body, fixtures[1].body, `command ${speed}`);
    }
    assert.ok(remoteMotion > 0, 'contact correction must move distant material beyond storage rounding');
    assert.ok(contactLoad > 0, 'the comparison must include a loaded contact');
});

test('coupled motion is independent of body registration order during relative feed and hold', () => {
    const fixtures = [false, true].map(reverse => {
        const world = new EndovascularPhysicsWorld({ coupledSystem });
        const profile = { radius: .4, linearDamping: .98, sleepFrames: 1e6 };
        const bodies = {};
        for (const id of reverse ? ['catheter', 'guidewire'] : ['guidewire', 'catheter'])
            bodies[id] = configureKirchhoffToolRuntime(world.createRod(id, 5, 5, profile));
        const inner = bodies.guidewire, outer = bodies.catheter;
        inner.y.fill(.15); inner.previousY.fill(.15);
        world.addContainment(inner, outer, { innerRadius: .5, axialFriction: .015, torsionalFriction: .006 });
        return { world, inner, outer };
    });
    for (const speed of [.2, 0, -.2, 0]) {
        for (const { world, inner, outer } of fixtures) {
            inner.velocityX.fill(speed);
            world.stepFixed(); closure(world, inner); closure(world, outer);
        }
        sameMechanics(fixtures[0].inner, fixtures[1].inner, 'inner registration order');
        sameMechanics(fixtures[0].outer, fixtures[1].outer, 'outer registration order');
    }
});

const policyFields = ['postStabilizationPasses', 'finalStructuralClosurePasses', 'postStabilizeBending',
    'projectionVelocityRetention', 'distalProjectionVelocityRetention', 'distalProjectionVelocityRetentionStartNode',
    'maxFrameDisplacement', 'wallProjectionVelocityRetention', 'toolProjectionVelocityRetention',
    'sweptContactPreserveTangentialMotion', 'wallFrictionUsesCurrentLoad', 'wallFrictionUsesSmoothedLoad'];
const materialFields = ['kirchhoffBendCompliance1', 'kirchhoffBendCompliance2', 'kirchhoffTwistCompliance',
    'restRotation1', 'restRotation2', 'restRotation3'];

test('public catheter feed, hold, withdrawal and twist use the wire runtime without changing material coefficients', () => {
    const sheath = { start: { x: 0, y: 0, z: 0 }, end: { x: 8, y: 0, z: 0 } };
    const world = new EndovascularPhysicsWorld();
    const reference = configureKirchhoffToolRuntime(world.createRod('guidewire', 21, 4, DEFAULT_TOOL_PROFILES.guidewire));
    const body = world.createRod('catheter', 21, 4, { ...DEFAULT_TOOL_PROFILES.catheter,
        linearDamping: .93, angularDamping: .91 });
    const catheter = new PigtailCatheter({ wire: new RodState(41, 2), segmentLength: 2, guidewireLength: 80,
        tailProgressRef: () => 60, vessel: { sheath, segments: [] }, maxLength: 60 });
    try {
        catheter.progress = 30;
        catheter.syncXpbdBody(body);
        const material = Object.fromEntries(materialFields.map(key => [key, body[key].slice(-3)]));
        const radii = body.nodeRadius.slice();
        for (const guideSupport of [0, 60, 0]) for (const [feed, rotation] of [[1, 0], [0, 0], [-1, 0], [0, 1], [0, -1], [0, 0]]) {
            catheter.advance(feed, 1 / 120, guideSupport);
            catheter.rotate(rotation, 1 / 120);
            catheter.stepPhysics(1 / 120);
            catheter.syncXpbdBody(body);
            for (const key of policyFields) assert.equal(body[key], reference[key], `${key} at ${feed}/${rotation}/${guideSupport}`);
            assert.equal(body.linearDamping, .93); assert.equal(body.angularDamping, .91);
            assert.deepEqual(body.nodeRadius, radii);
            for (const key of materialFields) assert.deepEqual(body[key].slice(-3), material[key], `${key} changes on command`);
        }
    } finally { catheter.dispose(); }
});

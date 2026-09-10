import assert from 'node:assert/strict';
import test from 'node:test';
import { Quaternion, Vector3 } from 'three';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { assembleKirchhoffDirect } from '../src/physics/kirchhoffDirectSolver.js';
import { applyKirchhoffCoupledCorrection } from '../src/physics/kirchhoffCoupledSystem.js';
import { beginKirchhoffSplitMotion, prescribeKirchhoffSplitOrientation } from '../src/physics/kirchhoffSplitMotion.js';
import { captureKirchhoffCoupledTrialState, restoreKirchhoffCoupledTrialState } from '../src/physics/kirchhoffCoupledTrialState.js';
import { beginKirchhoffTwoChannelMotion, applyKirchhoffTwoChannelPhysicalMotion,
    commitKirchhoffTwoChannelBiasMaterial, measureKirchhoffTwoChannelMaterial } from '../src/physics/kirchhoffTwoChannelMotion.js';

const DT = 1 / 120, XYZ = ['X', 'Y', 'Z'], Q = [...XYZ, 'W'];
const poseKeys = ['x', 'y', 'z', ...Q.map(a => 'orientation' + a)];
const motionKeys = XYZ.flatMap(a => ['velocity' + a, 'angularVelocity' + a]);
const materialKeys = ['adaptationLambdaX', 'adaptationLambdaY', 'adaptationLambdaZ', 'bendTwistLambda1', 'bendTwistLambda2', 'bendTwistLambda3'];
const near = (a, b, why, tol = 1e-11) => assert.ok(Number.isFinite(a) && Math.abs(a - b) <= tol, `${why}: ${a} vs ${b}`);
const quat = (pose, i) => new Quaternion(...Q.map(a => pose['orientation' + a][i])).normalize();
const putQuat = (pose, i, q) => Q.forEach((a, j) => { pose['orientation' + a][i] = q.toArray()[j]; });
const rotation = vector => { const angle = vector.length(); return angle ? new Quaternion().setFromAxisAngle(vector.clone().divideScalar(angle), angle) : new Quaternion(); };
const snapshot = (object, keys) => Object.fromEntries(keys.map(key => [key, object[key].slice()]));
function fixture({ activeStart = 0 } = {}) {
    const world = new EndovascularPhysicsWorld({ fixedDt: DT });
    const profile = { radius: .25, mass: 1, inverseAngularInertia: 1, adaptationCompliance: DT * DT / 99,
        kirchhoffBendCompliance: .02, kirchhoffTwistCompliance: .03, linearDamping: 1, angularDamping: 1 };
    const a = world.createRod('two-channel-motion-a', 3, 1, profile), b = world.createRod('two-channel-motion-b', 4, 1, profile);
    for (const body of [a, b]) {
        for (let i = 0; i < body.count; i++) body.setNodePosition(i, 2 + i, body === a ? 0 : -4, 0);
        body.activeStart = activeStart;
        body.copyCurrentToPrevious();
    }
    const joint = { innerBody: a, outerBody: b, kirchhoffContacts: [] };
    beginKirchhoffSplitMotion(joint, world); beginKirchhoffTwoChannelMotion(joint);
    return { world, joint, a, b, state: joint._splitMotion.twoChannel };
}
function resultFor(f, scale = 1) {
    const bodies = [f.a, f.b], responses = () => bodies.map(body => ({ correction: new Float64Array(body.count * 6), lambda: new Float64Array(body.segmentCount * 6) }));
    return { physical: responses(), bias: responses(), scale, additionalIncrement: new Float64Array(0), diagnostics: { converged: true } };
}
function totalResult(f, result) {
    [f.a, f.b].forEach((body, side) => {
        const m = assembleKirchhoffDirect(body, DT);
        result[side ? 'outer' : 'inner'] = { correction: Float64Array.from(result.physical[side].correction, (v, i) => v + result.bias[side].correction[i]),
            lambda: result.physical[side].lambda, start: m.start, end: m.end, redundantAxes: m.redundantAxes.slice() };
    });
    return result;
}
function apply(f, result) {
    totalResult(f, result);
    applyKirchhoffTwoChannelPhysicalMotion(f.joint, result);
    applyKirchhoffCoupledCorrection(f.joint, result);
    commitKirchhoffTwoChannelBiasMaterial(f.joint, result);
}

test('begin captures a separate predicted pose once and outer passes retain qP and time-start history', () => {
    const f = fixture(), physical = f.state.physicalPose[0], start = snapshot(f.joint._splitMotion.start[0], ['X', 'Y', 'Z', ...Q.map(a => 'orientation' + a)]);
    for (const key of poseKeys) { assert.notEqual(physical[key], f.a[key]); assert.ok(physical[key] instanceof Float64Array); }
    assert.ok(f.state.materialLambda.every(array => array.every(value => value === 0)));
    const x = physical.x[1];
    for (let i = 0; i < 2; i++) { const r = resultFor(f); r.physical[0].correction[6] = .01; apply(f, r); }
    near(physical.x[1], x + .02, 'cumulative physical pose');
    assert.throws(() => beginKirchhoffTwoChannelMotion(f.joint), /must not restart/);
    assert.equal(f.state.physicalPose[0], physical);
    assert.deepEqual(snapshot(f.joint._splitMotion.start[0], Object.keys(start)), start);
});

test('a hard prescribed frame belongs to physical motion and is not counted again as geometric material error', () => {
    const f = fixture(), initial = quat(f.a, 0), angle = .12;
    const target = rotation(new Vector3(angle, 0, 0)).multiply(initial).normalize();
    f.a.setProximalOrientationControl(target.x, target.y, target.z, target.w, 0, 0);
    const beta = f.state.materialLambda.map(values => values.slice());
    for (let pass = 0; pass < 2; pass++) {
        prescribeKirchhoffSplitOrientation(f.joint, f.a, 0);
        putQuat(f.a, 0, target); // World prescribes qG immediately after hook.
        assert.ok(Math.abs(quat(f.state.physicalPose[0], 0).dot(target)) > 1 - 1e-14);
        near(f.joint._splitMotion.bodies[0].angularVelocityX[0], angle / DT, 'commanded angular velocity is not accumulated twice');
        const measured = measureKirchhoffTwoChannelMaterial(f.joint);
        near(measured.biasResidual.bendTwistRad, 0, 'same controlled frame in qG and qP');
        assert.deepEqual(f.state.materialLambda, beta);
    }
});

test('bias-only updates total geometry and beta without changing qP, physical velocity, or physical material lambda', () => {
    const f = fixture(), p = snapshot(f.state.physicalPose[0], poseKeys), v = snapshot(f.joint._splitMotion.bodies[0], motionKeys), lambda = snapshot(f.a, materialKeys);
    const r = resultFor(f, .5); r.bias[0].correction[7] = .1; r.bias[0].correction[9] = .16; r.bias[0].lambda[9] = .3;
    apply(f, r);
    assert.deepEqual(snapshot(f.state.physicalPose[0], poseKeys), p);
    assert.deepEqual(snapshot(f.joint._splitMotion.bodies[0], motionKeys), v);
    assert.deepEqual(snapshot(f.a, materialKeys), lambda);
    near(f.a.y[1], .05, 'root applied total bias geometry', 3e-9); near(f.state.materialLambda[0][9], .15, 'bias multiplier');
});

test('physical displacement adds one impulse and leaves body qG and physical multipliers for root application', () => {
    const f = fixture(), geometry = snapshot(f.a, poseKeys), lambda = snapshot(f.a, materialKeys);
    const r = resultFor(f, .25); r.physical[0].correction[6] = .04; r.physical[0].lambda[3] = 7;
    applyKirchhoffTwoChannelPhysicalMotion(f.joint, r);
    near(f.state.physicalPose[0].x[1], geometry.x[1] + .01, 'physical pose increment');
    near(f.joint._splitMotion.bodies[0].velocityX[1], .01 / DT, 'physical impulse once');
    assert.deepEqual(snapshot(f.a, poseKeys), geometry); assert.deepEqual(snapshot(f.a, materialKeys), lambda);
    assert.throws(() => applyKirchhoffTwoChannelPhysicalMotion(f.joint, r), /already applied/);
    near(f.joint._splitMotion.bodies[0].velocityX[1], .01 / DT, 'duplicate application did not change motion');
});

test('physical local angular correction is transported from current qG before left-multiplying the separate qP', () => {
    const f = fixture(), first = resultFor(f);
    first.bias[0].correction[3] = .23; first.bias[0].correction[4] = -.17; apply(f, first);
    const p0 = quat(f.state.physicalPose[0], 0), g0 = quat(f.a, 0), r = resultFor(f, .375);
    const localP = new Vector3(.09, -.13, .04), localB = new Vector3(-.02, .05, .11);
    localP.toArray().forEach((v, i) => { r.physical[0].correction[3 + i] = v; r.bias[0].correction[3 + i] = localB.toArray()[i]; });
    const worldP = localP.clone().applyQuaternion(g0).multiplyScalar(r.scale);
    const expectedP = rotation(worldP).multiply(p0).normalize();
    const wrongP = p0.clone().multiply(rotation(localP.clone().multiplyScalar(r.scale))).normalize();
    const expectedG = g0.clone().multiply(rotation(localP.clone().add(localB).multiplyScalar(r.scale))).normalize();
    apply(f, r);
    assert.ok(1 - Math.abs(quat(f.state.physicalPose[0], 0).dot(expectedP)) < 1e-14);
    assert.ok(1 - Math.abs(quat(f.state.physicalPose[0], 0).dot(wrongP)) > 1e-7, 'qP local axes must not be substituted for qG axes');
    assert.ok(1 - Math.abs(quat(f.a, 0).dot(expectedG)) < 1e-14);
    XYZ.forEach((a, i) => near(f.joint._splitMotion.bodies[0]['angularVelocity' + a][0], worldP.toArray()[i] / DT, 'world angular impulse', 2e-8));
});

test('common fractional scale applies separately to each physical and bias material channel exactly once', () => {
    const f = fixture(), r = resultFor(f, .125);
    r.physical[0].correction[6] = .08; r.bias[0].correction[6] = -.024;
    r.physical[0].lambda[3] = .8; r.bias[0].lambda[3] = -.4;
    const x = f.a.x[1]; apply(f, r);
    near(f.state.physicalPose[0].x[1], x + .01, 'physical channel scale'); near(f.a.x[1], x + .007, 'total geometry scale', 2e-7);
    near(f.state.materialLambda[0][3], -.05, 'bias bank scale'); near(f.a.adaptationLambdaX[0], .1, 'physical bank scale', 1e-8);
    assert.throws(() => commitKirchhoffTwoChannelBiasMaterial(f.joint, r), /one physical application/);
    const other = resultFor(f, .5); applyKirchhoffTwoChannelPhysicalMotion(f.joint, other); other.scale = .25;
    assert.throws(() => commitKirchhoffTwoChannelBiasMaterial(f.joint, other), /same scale/);
});

test('fresh material measurement returns C(qG)-C(qP)+alpha beta using unchanged finite compliance', () => {
    const f = fixture(), r = resultFor(f, .5);
    r.physical[0].correction[6] = .012; r.bias[0].correction[7] = .004; r.bias[0].correction[9] = .08;
    r.bias[0].lambda[9] = .003; r.bias[0].lambda[10] = -.002; apply(f, r);
    const geometry = snapshot(f.a, poseKeys), physical = snapshot(f.state.physicalPose[0], poseKeys), lambda = snapshot(f.a, materialKeys);
    const measured = measureKirchhoffTwoChannelMaterial(f.joint), p = measured.physicalPoseStrain[0], g = measured.geometryStrain[0];
    assert.equal(measured.finite, true); near(measured.alpha[0][3], 1 / 99, 'physical finite compliance retained');
    for (let row = 0; row < p.length; row++) {
        near(measured.biasStrain[0][row], g[row] - p[row], 'nonlinear strain difference');
        near(measured.bodies[0].biasResidual[row], g[row] - p[row] + measured.alpha[0][row] * measured.biasLambda[0][row], 'bias residual');
        near(measured.bodies[0].physicalResidual[row], g[row] + measured.alpha[0][row] * measured.physicalLambda[0][row], 'physical material residual reads qG');
    }
    assert.ok(measured.biasResidual.bendTwistRad > 0, 'finite rotation changes native bending strain');
    assert.deepEqual(snapshot(f.a, poseKeys), geometry); assert.deepEqual(snapshot(f.state.physicalPose[0], poseKeys), physical);
    assert.deepEqual(snapshot(f.a, materialKeys), lambda);
    assert.deepEqual(f.a.kirchhoffScratch.direct.strain.slice(0, g.length), g, 'borrowed scratch restored to qG');
    const owned = structuredClone(measured); f.a.y[1] += .001; assembleKirchhoffDirect(f.a, DT);
    assert.deepEqual(measured, owned, 'measurement does not borrow mutable native arrays');
});

test('hard-frame measurement preserves both posed frames, control targets, pose references and native mobility mask', () => {
    const f = fixture(); f.a.orientationControlSegment = 0; f.a.orientationControlCompliance = 0;
    const target = rotation(new Vector3(.3, .2, -.1)); Q.forEach((a, i) => { f.a['orientationControl' + a] = target.toArray()[i]; });
    putQuat(f.a, 0, rotation(new Vector3(.1, -.07, .02)).multiply(quat(f.a, 0)));
    const refs = poseKeys.map(key => f.a[key]), geometry = snapshot(f.a, poseKeys), physical = snapshot(f.state.physicalPose[0], poseKeys);
    const m = measureKirchhoffTwoChannelMaterial(f.joint);
    assert.deepEqual(snapshot(f.a, poseKeys), geometry); assert.deepEqual(snapshot(f.state.physicalPose[0], poseKeys), physical);
    poseKeys.forEach((key, i) => assert.equal(f.a[key], refs[i]));
    Q.forEach((a, i) => near(f.a['orientationControl' + a], target.toArray()[i], 'external target restored'));
    assert.deepEqual([...f.a.kirchhoffScratch.direct.weight.slice(3, 6)], [0, 0, 0]);
    assert.ok(m.biasStrain[0].some(value => Math.abs(value) > .001), 'measurement did not snap qP onto qG/control target');
});

test('a failing qP assembly restores body references and reconstructs current geometric scratch in finally', () => {
    const f = fixture(), refs = poseKeys.map(key => f.a[key]), current = assembleKirchhoffDirect(f.a, DT).strain.slice();
    const compliance = f.a.adaptationCompliance;
    Object.defineProperty(f.a, 'adaptationCompliance', { configurable: true, get() {
        if (this.x === f.state.physicalPose[0].x) throw new Error('test physical-pose assembly failure');
        return compliance;
    } });
    assert.throws(() => measureKirchhoffTwoChannelMaterial(f.joint), /test physical-pose assembly failure/);
    poseKeys.forEach((key, i) => assert.equal(f.a[key], refs[i]));
    assert.deepEqual(f.a.kirchhoffScratch.direct.strain, current);
    Object.defineProperty(f.a, 'adaptationCompliance', { configurable: true, writable: true, value: compliance });
});

test('rejected combined trial restores qP, beta, physical motion, geometry and once-only receipts in place', () => {
    const f = fixture(), r = resultFor(f, .5); r.physical[0].correction[6] = .02; r.bias[0].correction[7] = .04; r.bias[0].lambda[9] = .1;
    totalResult(f, r);
    const state = f.state, pose = state.physicalPose[0], px = pose.x, beta = state.materialLambda[0], velocity = f.joint._splitMotion.bodies[0].velocityX;
    const before = { geometry: snapshot(f.a, poseKeys), physical: snapshot(pose, poseKeys), beta: beta.slice(), velocity: velocity.slice() };
    const trial = captureKirchhoffCoupledTrialState(f.joint, { reusePropertyLayout: true, frozenFrictionBatches: true });
    apply(f, r); restoreKirchhoffCoupledTrialState(trial);
    assert.equal(f.joint._splitMotion.twoChannel, state); assert.equal(state.physicalPose[0], pose); assert.equal(pose.x, px);
    assert.equal(state.materialLambda[0], beta); assert.equal(f.joint._splitMotion.bodies[0].velocityX, velocity);
    assert.deepEqual({ geometry: snapshot(f.a, poseKeys), physical: snapshot(pose, poseKeys), beta, velocity }, before);
    assert.equal(state.applications.size, 0);
    r.scale = .25; apply(f, r); near(beta[9], .025, 'same result may be retried after rollback at new common scale');
});

test('native local material indexing is preserved for an active range starting after node zero', () => {
    const f = fixture({ activeStart: 1 }), r = resultFor(f, .5); r.bias[0].lambda[3] = .08;
    apply(f, r); const measured = measureKirchhoffTwoChannelMaterial(f.joint);
    assert.equal(measured.bodies[0].start, 1); assert.equal(measured.bodies[0].rowCount, 6); near(measured.biasLambda[0][3], .04, 'local row3 bank');
    f.a.activeStart = 0; assert.throws(() => measureKirchhoffTwoChannelMaterial(f.joint), /layout changed/);
});

test('the reused split hook journals physical sheath increments without recording bias normals', () => {
    const f = fixture(), sheath = {}, split = f.joint._splitMotion;
    const history = { reactionHistory: Object.fromEntries(['physical', 'bias', 'legacy'].map(phase =>
        [phase, [new Float64Array(f.a.count), new Float64Array(f.b.count)]])) };
    split.sheathHistory.set(sheath, history);
    f.joint._coupledBoundaries = { rows: [{ kind: 'sheath', side: 0, node: 1, lambda: 0, sheathWitness: { geometry: { sheath } } }] };
    const r = resultFor(f, .5); r.additionalIncrement = new Float64Array([.2]); r.biasAdditionalIncrement = new Float64Array([.9]);
    applyKirchhoffTwoChannelPhysicalMotion(f.joint, r);
    near(history.reactionHistory.physical[0][1], .1, 'actual scaled physical normal increment');
    assert.equal(history.reactionHistory.bias[0][1], 0, 'bias normal belongs to root staging');
    assert.deepEqual(split.diagnostics.unverifiedHistoryKinds, []);
});

test('an unconverged direction is rejected before qP, velocity or beta changes', () => {
    const f = fixture(), r = resultFor(f); r.physical[0].correction[6] = .1; r.diagnostics.converged = false;
    const p = snapshot(f.state.physicalPose[0], poseKeys), v = snapshot(f.joint._splitMotion.bodies[0], motionKeys);
    assert.throws(() => applyKirchhoffTwoChannelPhysicalMotion(f.joint, r), /unconverged/);
    assert.deepEqual(snapshot(f.state.physicalPose[0], poseKeys), p); assert.deepEqual(snapshot(f.joint._splitMotion.bodies[0], motionKeys), v);
    assert.equal(f.state.applications.size, 0); assert.ok(f.state.materialLambda[0].every(value => value === 0));
});

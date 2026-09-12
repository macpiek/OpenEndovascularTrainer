import assert from 'node:assert/strict';
import test from 'node:test';
import { Quaternion } from 'three';
import { fixture, frame, DT } from './fixtures/splitMotionAnalyticWorld.js';
import { configureKirchhoffSplitBias } from '../src/physics/kirchhoffSplitMotion.js';

function tilt(f) {
    for (let i = 0; i < f.wire.count; i++) f.wire.setNodePosition(i, i - 1, -.5 + .2 * (i - 1), 0);
    f.wire.captureRestConfiguration(); f.wire.copyCurrentToPrevious();
}
function removeOverlap(f) {
    for (let i = 0; i < f.wire.count; i++) f.wire.setNodePosition(i, i - 1, -.6, 0);
    f.wire.captureRestConfiguration(); f.wire.copyCurrentToPrevious();
}
function numericBytes(body) {
    return Object.entries(body).filter(([, value]) => ArrayBuffer.isView(value)).map(([key, array]) =>
        ({ key, array, bytes: Buffer.from(array.buffer, array.byteOffset, array.byteLength).toString('hex') }));
}
function sameBytes(body, saved) {
    for (const { key, array, bytes } of saved) {
        assert.equal(body[key], array, `${key} original storage reference`);
        assert.equal(Buffer.from(array.buffer, array.byteOffset, array.byteLength).toString('hex'), bytes, `${key} original bytes`);
    }
}

test('uncertified tilted dt restores body/history/manifold/tools/sheath and force inputs before damping', () => {
    const f = fixture({ wall: true, y: -.5 });
    assert.equal(f.world.stepFixed().accepted, true);
    const oldHistory = f.constraint._acceptedPhysicalMotion, oldSplit = f.constraint._splitMotion;
    const oldBodyMotion = f.wire._splitPhysicalMotion, beforeStep = f.world.stepCount;
    tilt(f);
    f.wire.linearDamping = .6; f.wire.angularDamping = .7;
    f.wire.velocityX.fill(1); f.wire.angularVelocityX.fill(.1); f.wire.forceX.fill(.25);
    const tool = f.world.addToolContact(f.wire, f.catheter, { enabled: false });
    tool.lambdas[0] = .123;
    tool._jointReactions = new Map([['prior', { wrenches: [], normalWrenches: [], data: Float64Array.of(.5, .75) }]]);
    const sheath = f.world.addSheath({ start: { x: 10, y: 0, z: 0 }, end: { x: 11, y: 0, z: 0 }, bodies: [] });
    sheath.lambdas.set(f.wire, Float32Array.of(.2, .4));
    const contact = f.constraint.manifold.upsertContact({ innerMaterialSegmentId: 'old-inner', outerMaterialSegmentId: 'old-outer',
        feature: 'review-history', innerSegmentIndex: 0, outerSegmentIndex: 0, normal: [0, 1, 0], tangentU: [1, 0, 0] });
    contact.normalLambda = 2; contact.tangentLambda[0] = .3; f.constraint.manifold.endStep();
    const contacts = Array.from(f.constraint.manifold.contacts()), manifoldStep = f.constraint.manifold.step;
    const toolLambdas = tool.lambdas, reactions = tool._jointReactions, reactionBytes = reactions.get('prior').data.slice();
    const sheathMap = sheath.lambdas, sheathValues = sheathMap.get(f.wire), sheathBytes = sheathValues.slice();
    const wire = numericBytes(f.wire), catheter = numericBytes(f.catheter);
    const result = f.world.stepFixed();
    assert.equal(result.accepted, false); assert.equal(result.status, 'split-uncertified');
    assert.equal(f.world.stepCount, beforeStep); assert.equal(f.world.lastCoupledClosureConverged, false);
    assert.ok(result.diagnostics.finalMaterialResidual.bendTwistRad > .001);
    assert.equal(result.diagnostics.historyCommits, 0);
    sameBytes(f.wire, wire); sameBytes(f.catheter, catheter);
    assert.equal(f.constraint._acceptedPhysicalMotion, oldHistory); assert.equal(f.constraint._splitMotion, oldSplit);
    assert.equal(f.wire._splitPhysicalMotion, oldBodyMotion);
    assert.deepEqual(Array.from(f.constraint.manifold.contacts()), contacts);
    assert.equal(f.constraint.manifold.step, manifoldStep); assert.equal(contact.normalLambda, 2);
    assert.deepEqual(Array.from(contact.tangentLambda), [.3, 0]);
    assert.equal(tool.lambdas, toolLambdas); assert.equal(tool.lambdas[0], .123);
    assert.equal(tool._jointReactions, reactions); assert.deepEqual(reactions.get('prior').data, reactionBytes);
    assert.equal(sheath.lambdas, sheathMap); assert.equal(sheathMap.get(f.wire), sheathValues); assert.deepEqual(sheathValues, sheathBytes);
    assert.ok(f.world.lastJointFactorizations > 0, 'rejected solve costs remain visible');
    assert.ok(f.world.timings.total.last > 0);
    assert.equal(f.world.getStats().jointMotion.certified, false, 'restored old accepted graph cannot conceal failed diagnostics');
    const savedFailure = structuredClone(result);
    // Remove the rejected geometry and old test reactions. The force input
    // restored above is consumed by one accepted physical dt, not lost.
    contact.normalLambda = 0; contact.tangentLambda.fill(0);
    f.world.toolContacts.length = 0; f.world.sheaths.length = 0;
    removeOverlap(f);
    const accepted = f.world.stepFixed();
    assert.equal(accepted.accepted, true, JSON.stringify(accepted));
    assert.equal(f.world.stepCount, beforeStep + 1); assert.equal(accepted.diagnostics.historyCommits, 1);
    assert.ok(f.wire.forceX.every(value => value === 0));
    assert.deepEqual(result, savedFailure, 'failed result is an owned stable diagnostic snapshot');
});

test('advance keeps rejected dt queued and prepares callback inputs only once across retries', () => {
    const f = fixture({ wall: true, y: -.5 }); tilt(f);
    let callbacks = 0;
    const prepare = () => { callbacks++; f.wire.forceX.fill(.125 * callbacks); };
    assert.equal(f.world.advance(DT, prepare), 0);
    assert.equal(callbacks, 1); assert.equal(f.world.accumulator, DT); assert.equal(f.world.stepCount, 0);
    assert.equal(f.world.lastSubsteps, 0); assert.ok(f.wire.forceX.every(value => value === .125));
    assert.equal(f.world.advance(0, prepare), 0);
    assert.equal(callbacks, 1); assert.equal(f.world.accumulator, DT); assert.ok(f.wire.forceX.every(value => value === .125));
    removeOverlap(f);
    assert.equal(f.world.advance(0, prepare), 1);
    assert.equal(callbacks, 1); assert.equal(f.world.accumulator, 0); assert.equal(f.world.stepCount, 1);
    assert.equal(f.world.advance(DT, prepare), 1);
    assert.equal(callbacks, 2); assert.equal(f.world.stepCount, 2); assert.equal(f.world.droppedTime, 0);
    f.world.maxSubsteps = 1;
    assert.equal(f.world.advance(2 * DT, prepare), 1);
    assert.equal(f.world.accumulator, DT); assert.equal(callbacks, 3);
    assert.equal(f.world.advance(0, prepare), 1);
    assert.equal(f.world.accumulator, 0); assert.equal(callbacks, 4); assert.equal(f.world.stepCount, 4);
});

test('a rejected internal physical trial can still commit exactly one accepted dt', () => {
    const f = fixture(), q = new Quaternion(Math.sin(.1), 0, 0, Math.cos(.1)).multiply(frame(f.wire, 1));
    ['X', 'Y', 'Z', 'W'].forEach((axis, i) => { f.wire['orientation' + axis][1] = q.toArray()[i]; });
    f.wire.copyCurrentToPrevious();
    let rejected = 0;
    f.world.debugJointTrial = (joint, state, pass, trial) => {
        if (state.motionPhase !== 'physical') return;
        if (pass === 0) { state.settled = false; state.merit = 1; }
        if (pass === 1 && trial === 0) { state.settled = false; state.merit = Infinity; rejected++; }
    };
    const result = f.world.stepFixed();
    assert.equal(result.accepted, true, JSON.stringify(result)); assert.equal(rejected, 1);
    assert.equal(f.world.stepCount, 1); assert.equal(result.diagnostics.historyCommits, 1);
    assert.ok(result.diagnostics.rollbackCount >= 1);
});

test('whole-step rollback reconnects original storage and topology even if a failing hook replaces them', () => {
    const f = fixture(), bodies = f.world.bodies, x = f.wire.x, activeEnd = f.wire.activeEnd;
    const before = numericBytes(f.wire);
    configureKirchhoffSplitBias(f.constraint, { materialMode: 'preserve-strain' });
    f.world.debugJointTrial = () => {
        f.wire.x = f.wire.x.slice(); f.wire.x[0] = 100;
        f.wire.activeEnd--;
        f.world.bodies = [f.catheter, f.wire];
        f.world.fixedDt = 2 * DT;
        f.constraint._splitBiasMaterialMode = 'physical-compliance';
        throw new Error('intentional failed topology edit');
    };
    assert.throws(() => f.world.stepFixed(), /intentional failed topology edit/);
    assert.equal(f.world.bodies, bodies); assert.deepEqual(f.world.bodies, [f.wire, f.catheter]);
    assert.equal(f.wire.x, x); assert.equal(f.wire.activeEnd, activeEnd); sameBytes(f.wire, before);
    assert.equal(f.world.stepCount, 0); assert.equal(f.world.lastStepResult.accepted, false);
    assert.equal(f.world.fixedDt, DT);
    assert.equal(f.constraint._splitMotion, undefined); assert.equal(f.wire._splitPhysicalMotion, undefined);
    assert.equal(f.constraint._splitBiasMaterialMode, 'preserve-strain');
});

test('accepted transaction retains the selected per-joint bias material mode', () => {
    const f = fixture(); configureKirchhoffSplitBias(f.constraint, { materialMode: 'preserve-strain' });
    const result = f.world.stepFixed();
    assert.equal(result.accepted, true, JSON.stringify(result));
    assert.equal(f.constraint._splitBiasMaterialMode, 'preserve-strain');
    assert.equal(result.diagnostics.biasMaterialMode, 'preserve-strain');
});

test('pending split time cannot silently switch to another dt or another physical law', () => {
    const f = fixture({ wall: true, y: -.5 }); tilt(f);
    f.world.advance(DT);
    f.world.fixedDt = DT / 2;
    assert.throws(() => f.world.advance(0), /pending split timestep/);
    f.world.fixedDt = DT; f.world.jointMotionMode = 'position-history';
    assert.throws(() => f.world.advance(0), /pending split timestep/);
    assert.equal(f.world.accumulator, DT); assert.equal(f.world.stepCount, 0);
});

test('direct retry consumes the pending dt once and cannot fall back when the joint is unavailable', () => {
    const f = fixture({ wall: true, y: -.5 }); tilt(f);
    let calls = 0; f.world.advance(DT, () => calls++);
    f.constraint.enabled = false;
    const missing = f.world.stepFixed();
    assert.equal(missing.accepted, false); assert.equal(missing.status, 'split-joint-unavailable');
    assert.equal(f.world.stepCount, 0); assert.equal(f.world.accumulator, DT);
    f.constraint.enabled = true; removeOverlap(f);
    const accepted = f.world.stepFixed();
    assert.equal(accepted.accepted, true); assert.equal(accepted.consumedPendingDt, true);
    assert.equal(f.world.accumulator, 0); assert.equal(f.world.stepCount, 1);
    assert.equal(f.world.advance(0, () => calls++), 0); assert.equal(calls, 1);
});

test('default position-history retains its return behavior and per-step callback scheduling', () => {
    const f = fixture(); f.world.jointMotionMode = 'position-history';
    assert.equal(f.world.stepFixed(), undefined);
    let calls = 0; assert.equal(f.world.advance(2 * DT, () => calls++), 2);
    assert.equal(calls, 2); assert.equal(f.world.stepCount, 3); assert.equal(f.world.accumulator, 0);
});

test('split configuration permits independent preparation and activates a transaction when the pair becomes eligible', () => {
    const f = fixture(); f.constraint.enabled = false;
    assert.equal(f.world.stepFixed(), undefined);
    assert.equal(f.world.stepCount, 1); assert.equal(f.world.lastStepResult, null);
    let calls = 0, sawPending = false;
    f.world.debugJointTrial = () => { sawPending ||= f.world._pendingSplitSubstep?.dt === DT; };
    const completed = f.world.advance(2 * DT, () => {
        calls++;
        if (calls === 2) f.constraint.enabled = true;
    });
    assert.equal(completed, 2); assert.equal(calls, 2);
    assert.equal(f.world.stepCount, 3); assert.equal(f.world.accumulator, 0);
    assert.equal(sawPending, true, 'newly eligible pair owns its prepared timestep');
    assert.equal(f.world.lastStepResult.accepted, true);
    assert.equal(f.world.lastStepResult.consumedPendingDt, true);
    assert.equal(f.world._pendingSplitSubstep, null);
});

test('leaving the eligible pair retires phase-local motion and cannot report its old certificate', () => {
    const f = fixture({ wall: true, y: -.5 });
    assert.equal(f.world.stepFixed().accepted, true);
    const history = f.constraint._acceptedPhysicalMotion;
    f.constraint.enabled = false;
    assert.equal(f.world.stepFixed(), undefined);
    assert.equal(f.world.stepCount, 2);
    assert.equal(f.world.getStats().coupledSolver, 'independent');
    assert.equal(f.world.getStats().jointMotion, null);
    assert.equal(f.world.lastStepResult, null);
    assert.equal(f.constraint._splitMotion, undefined);
    assert.equal(f.constraint.surfaceMotion, undefined);
    assert.equal(f.wire._splitPhysicalMotion, null);
    assert.equal(f.catheter._splitPhysicalMotion, null);
    assert.equal(f.constraint._acceptedPhysicalMotion, history, 'last accepted snapshot is not a current phase');
    f.constraint.enabled = true;
    const result = f.world.stepFixed();
    assert.equal(result.accepted, true);
    assert.equal(result.diagnostics.historyCommits, 1);
    assert.notEqual(f.constraint._acceptedPhysicalMotion, history);
});

test('losing pair eligibility cannot suppress the partitioned radial damping path', () => {
    const f = fixture(); assert.equal(f.world.stepFixed().accepted, true);
    const third = f.world.createRod('third', 2, 1);
    third.setNodePosition(0, 10, 0, 0); third.setNodePosition(1, 11, 0, 0);
    third.copyCurrentToPrevious();
    f.constraint.radialVelocityDamping = .5;
    f.wire.projectionVelocityRetention = 1;
    f.wire.velocityY.fill(1);
    assert.equal(f.world.stepFixed(), undefined);
    assert.equal(f.world.getStats().coupledSolver, 'partitioned');
    assert.equal(f.world.getStats().jointMotion, null);
    for (const node of [0, 1]) assert.ok(Math.abs(f.wire.velocityY[node] - .5) < 1e-6);
    assert.ok(Math.abs(f.wire.velocityY[2] - 1) < 1e-6, 'terminal node outside damping support');
});

test('a physical-phase failure survives bias initialization and whole-step rollback as owned diagnostics', () => {
    const f = fixture();
    f.wire.y[1] = .1; f.wire.copyCurrentToPrevious();
    f.wire.inverseMass.fill(0);
    for (let axis = 1; axis <= 3; axis++) f.wire['inverseInertia' + axis].fill(0);
    const result = f.world.stepFixed();
    assert.equal(result.accepted, false);
    assert.equal(result.diagnostics.physicalFailure.linear.status, 'immovable-constraints');
    assert.equal(result.diagnostics.physicalFailure.linear.converged, false);
    assert.equal(f.world.stepCount, 0);
    assert.equal(f.constraint._jointLinearFailure, undefined);
    const saved = structuredClone(result.diagnostics.physicalFailure);
    f.wire.y[1] = 0; f.wire.copyCurrentToPrevious();
    assert.equal(f.world.stepFixed().accepted, true);
    assert.deepEqual(result.diagnostics.physicalFailure, saved);
});

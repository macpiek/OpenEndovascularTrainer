import assert from 'node:assert/strict';
import test from 'node:test';
import { fixture, begin, build, solveAttempt, retry, step, DT, TOL } from './fixtures/wallFrictionModeHarness.js';
import { captureKirchhoffCoupledTrialState, restoreKirchhoffCoupledTrialState } from '../src/physics/kirchhoffCoupledTrialState.js';
import { captureKirchhoffWallFrictionIncoming, evaluateKirchhoffWallFrictionCandidate,
    certifyKirchhoffWallFrictionModes, commitKirchhoffWallFrictionHistory, prepareKirchhoffWallFrictionRetry } from '../src/physics/kirchhoffWallFrictionMode.js';
import { buildKirchhoffSplitWallFriction } from '../src/physics/kirchhoffSplitWallFriction.js';

const near = (a, b, text) => assert.ok(Number.isFinite(a) && Math.abs(a - b) < 1e-8, `${text}: ${a} vs ${b}`);
const impulse = trial => trial.certificate.contacts[0].lambda[0] / DT;

test('rest under drive between kinetic and static budgets stays stuck across accepted steps', () => {
    const f = fixture();
    for (let i = 0; i < 3; i++) {
        const t = step(f, { drive: .4 });
        assert.equal(t.certificate.accepted, true); assert.equal(f.attempts.length, 1);
        near(impulse(t), -.4, 'static impulse'); near(f.joint.innerBody.velocityZ[0], 0, 'static velocity');
        assert.equal(t.certificate.contacts[0].stopped, true);
        assert.equal(f.joint._wallFrictionHistory.records.get('wall:0:0').mode, 'stick');
    }
});

test('breakaway rolls back the entire static candidate and applies only the kinetic branch impulse', () => {
    const f = fixture(), t = step(f, { drive: .8 });
    assert.equal(t.certificate.accepted, true); assert.deepEqual(f.attempts.map(a => a.mode), ['stick', 'slide']);
    assert.equal(f.attempts[0].decision.status, 'restart');
    near(f.attempts[0].delta[1] / DT, -.6, 'discarded static impulse');
    near(impulse(t), -.2, 'accepted kinetic impulse'); near(f.joint.innerBody.velocityZ[0], .6, 'accepted velocity');
    assert.equal(f.joint._splitMotion.wallFrictionModes.records.get('wall:0:0').appliedCount, 1);
});

test('existing sliding uses the kinetic budget even where the static cone could stop it', () => {
    const f = fixture(); f.joint.innerBody.velocityZ[0] = .1;
    const t = step(f, { drive: .3 });
    assert.equal(f.attempts.length, 1); near(impulse(t), -.2, 'kinetic impulse'); near(f.joint.innerBody.velocityZ[0], .2, 'continued slip');
});

test('steady sliding is invariant to static coefficient and dissipates its predicted kinetic energy', () => {
    for (const muStatic of [.3, .6, .9]) {
        const f = fixture(muStatic, .2); f.joint.innerBody.velocityZ[0] = 1;
        const t = step(f, { drive: .2 }); near(impulse(t), -.2, 'kinetic impulse'); near(f.joint.innerBody.velocityZ[0], 1, 'steady velocity');
        const p = impulse(t), u = f.joint.innerBody.velocityZ[0];
        near(.5 * (u * u - 1.2 ** 2), p * u - .5 * p * p, 'discrete work identity');
        assert.ok(p * u <= 0);
    }
});

test('stopping does not reverse velocity; next-step reversal first sticks then breaks away in the opposite direction', () => {
    const f = fixture(); f.joint.innerBody.velocityZ[0] = .1;
    let t = step(f); near(impulse(t), -.1, 'capture impulse'); near(f.joint.innerBody.velocityZ[0], 0, 'capture velocity');
    assert.equal(t.certificate.contacts[0].stopped, true);
    t = step(f, { drive: -.4 }); near(impulse(t), .4, 'reverse static reaction'); near(f.joint.innerBody.velocityZ[0], 0, 'reverse held velocity');
    t = step(f, { drive: -.8 }); near(impulse(t), .2, 'reverse kinetic reaction'); near(f.joint.innerBody.velocityZ[0], -.6, 'reverse sliding');
});

test('strong reversal in one dt follows the explicit post-prediction kinetic law and dissipates energy', () => {
    const f = fixture(); f.joint.innerBody.velocityZ[0] = .1;
    const t = step(f, { drive: -.5 }); near(impulse(t), .2, 'reverse impulse'); near(f.joint.innerBody.velocityZ[0], -.2, 'reverse speed');
    assert.ok(.5 * f.joint.innerBody.velocityZ[0] ** 2 < .5 * .4 ** 2);
});

test('zero physical normal load releases all friction regardless of positive static coefficient', () => {
    const f = fixture(); step(f, { drive: .4 });
    const t = step(f, { drive: .3, normalImpulse: 0 });
    assert.equal(t.certificate.accepted, true); near(impulse(t), 0, 'unloaded impulse'); near(f.joint.innerBody.velocityZ[0], .3, 'free motion');
    assert.equal(f.joint._wallFrictionHistory.records.size, 0);
});

test('zero kinetic coefficient still permits static holding and gives zero friction during sliding', () => {
    const f = fixture(.6, 0); let t = step(f, { drive: .4 }); near(impulse(t), -.4, 'static hold');
    t = step(f, { drive: .8 }); near(impulse(t), 0, 'frictionless breakaway'); near(f.joint.innerBody.velocityZ[0], .8, 'free slide');
});

test('standalone unequal builder retains the guard; equal coefficients need no controller', () => {
    const f = fixture(); begin(f, { initialize: false }); build(f);
    assert.ok(f.joint._splitMotion.diagnostics.unverifiedHistoryKinds.includes('unequal-wall-static-kinetic'));
    const equal = fixture(.2, .2); assert.equal(captureKirchhoffWallFrictionIncoming(equal.joint, equal.world), null);
    begin(equal); const b = build(equal).batch;
    assert.equal(equal.joint._splitMotion.wallFrictionModes, undefined); assert.deepEqual([...b.groups[0].mu], [.2, .2]);
    assert.equal(equal.joint._splitMotion.diagnostics.unverifiedHistoryKinds.length, 0);
});

test('small nonzero static-boundary slip is ambiguous, and solver failure never causes a retry', () => {
    const f = fixture(); begin(f, { drive: .8 }); const t = solveAttempt(f);
    const entry = t.fresh.entries[0];
    f.joint._splitMotion.bodies[0].velocityZ[0] = TOL / (2 * DT);
    const d = evaluateKirchhoffWallFrictionCandidate(f.joint, t.fresh, { converged: true });
    assert.equal(d.status, 'ambiguous'); assert.equal(d.restart, false);
    assert.equal(evaluateKirchhoffWallFrictionCandidate(f.joint, t.fresh, { converged: false }).status, 'unconverged');
    f.joint._splitMotion.phase = 'complete';
    assert.equal(certifyKirchhoffWallFrictionModes(f.joint, t.fresh, { converged: true }).status, 'ambiguous');
    assert.ok(f.joint._splitMotion.diagnostics.unverifiedHistoryKinds.includes('wall-friction-mode-ambiguous'));
    assert.equal(entry.modeRecord.mode, 'stick');
});

test('resolved slip with valid KKT proves breakaway despite a small numerical interior force offset', () => {
    const f = fixture(); begin(f, { drive: .8 }); const t = solveAttempt(f);
    const contact = t.fresh.entries[0].contact;
    contact.tangentLambda[0] *= 1 - 3e-8;
    const d = evaluateKirchhoffWallFrictionCandidate(f.joint, t.fresh, { converged: true });
    const c = d.contacts[0];
    assert.ok(Math.hypot(...c.lambda) < c.muStatic * c.normalLambda * (1 - 1e-9), 'force lies numerically inside static cone');
    assert.ok(Math.hypot(...c.displacement) > TOL && c.residualMm <= TOL, 'resolved sliding with accepted native KKT');
    assert.equal(d.status, 'restart'); assert.equal(d.overrides[0].mode, 'slide');
});

test('applied half-scale reaction journal and phase mode restore with rejected trial identity', () => {
    const f = fixture(); begin(f, { drive: .4 }); build(f);
    const state = f.joint._splitMotion, record = state.wallFrictionModes.records.get('wall:0:0'), velocity = state.bodies[0].velocityZ;
    const snapshot = captureKirchhoffCoupledTrialState(f.joint, { reusePropertyLayout: true, frozenFrictionBatches: true });
    const t = solveAttempt(f, { scale: .5 });
    near(record.lastTangentIncrement[0], .5 * t.result.increment[1], 'actual scaled delta');
    assert.ok(record.maximumReaction > 0);
    record.mode = 'slide'; restoreKirchhoffCoupledTrialState(snapshot);
    assert.equal(f.joint._splitMotion, state); assert.equal(state.wallFrictionModes.records.get(record.key), record);
    assert.equal(record.maximumReaction, 0); assert.equal(record.mode, 'stick'); assert.equal(state.bodies[0].velocityZ, velocity);
    near(velocity[0], .4, 'restored prediction');
});

test('loaded foot migration remains guarded even after current reaction arrays are erased', () => {
    const f = fixture(); begin(f, { drive: .4 }); solveAttempt(f);
    f.joint.innerBody.wallLambda.fill(0);
    for (const contact of f.joint._splitMotion.wallFrictionContacts.values()) contact.tangentLambda.fill(0);
    f.joint.innerBody.wallT[0] = 1;
    const fresh = build(f).batch, d = evaluateKirchhoffWallFrictionCandidate(f.joint, fresh, { converged: true });
    assert.equal(d.status, 'unsupported'); assert.ok(f.joint._splitMotion.diagnostics.unverifiedHistoryKinds.includes('wall-friction-loaded-witness-changed'));
});

test('retry plan owns its witness and rejects unbounded attempts', () => {
    const f = fixture(); begin(f, { drive: .8 }); const t = solveAttempt(f), original = t.decision.overrides[0].identity.weights[0];
    t.fresh.entries[0].modeRecord.identity.weights[0] = .5;
    assert.equal(t.decision.overrides[0].identity.weights[0], original);
    retry(f, t.decision); assert.equal(f.joint._splitMotion.wallFrictionModes.attempt, 2);
    assert.throws(() => prepareKirchhoffWallFrictionRetry(f.joint, { ...t.decision, attempt: 1000 }), /Invalid or exhausted/);
});

test('history commit is once-only and a whole-step rollback restores the prior history map', () => {
    const f = fixture(); step(f, { drive: .4 }); const old = f.joint._wallFrictionHistory;
    const snapshot = captureKirchhoffCoupledTrialState(f.joint);
    const t = step(f, { drive: .8 }); assert.notEqual(f.joint._wallFrictionHistory, old);
    assert.throws(() => commitKirchhoffWallFrictionHistory(f.joint, t.certificate), /one accepted/);
    restoreKirchhoffCoupledTrialState(snapshot); assert.equal(f.joint._wallFrictionHistory, old);
    assert.equal(old.records.get('wall:0:0').mode, 'stick');
});

test('externally changed incoming velocity invalidates a prior stop certificate', () => {
    const f = fixture(); step(f, { drive: .4 }); f.joint.innerBody.velocityZ[0] = .1;
    const t = step(f, { drive: .3 }); assert.equal(f.attempts[0].mode, 'slide'); near(impulse(t), -.2, 'kinetic after edited input');
});

test('material relabeling after incoming capture cannot borrow the old physical velocity history', () => {
    const f = fixture(); begin(f, { drive: .4 }); f.joint.innerBody.materialCoordinate[0] = 10;
    const fresh = build(f).batch;
    assert.equal(evaluateKirchhoffWallFrictionCandidate(f.joint, fresh, { converged: true }).status, 'unsupported');
    assert.ok(f.joint._splitMotion.diagnostics.unverifiedHistoryKinds.includes('wall-friction-incoming-material-changed'));
});

test('a rejected final certificate cannot publish candidate mode history', () => {
    const f = fixture(); step(f, { drive: .4 }); const history = f.joint._wallFrictionHistory;
    begin(f, { drive: .8 }); const t = solveAttempt(f);
    f.joint._splitMotion.phase = 'complete';
    const certificate = certifyKirchhoffWallFrictionModes(f.joint, t.fresh, { converged: true });
    assert.equal(certificate.accepted, false); assert.equal(certificate.restart, false);
    assert.throws(() => commitKirchhoffWallFrictionHistory(f.joint, certificate), /one accepted/);
    assert.equal(f.joint._wallFrictionHistory, history);
});

function zeroLoadPoint(f, node) {
    return { kind: 'split-point-wall', side: 0, node, owner: f.joint.innerBody, lambda: 0,
        normal: [0, -1, 0], wallFrictionWitness: { branchId: 0, faceIndex: 0, planeOffset: 0 } };
}

test('a redundant zero-load point retains a stop certificate by equal kinematics, without copying force or zeroing velocity', () => {
    const f = fixture(); begin(f, { drive: .4 }); solveAttempt(f);
    const tinyVelocity = 3.6e-9;
    f.joint._splitMotion.bodies[0].velocityZ[0] = tinyVelocity;
    const rows = [build(f).normal, zeroLoadPoint(f, 0)];
    const fresh = buildKirchhoffSplitWallFriction(f.joint, rows, DT);
    const d = evaluateKirchhoffWallFrictionCandidate(f.joint, fresh, { converged: true });
    const c = d.contacts.find(c => c.key === 'split-point-wall:0:0');
    assert.equal(c.normalLambda, 0); assert.deepEqual(c.lambda, [0, 0]);
    assert.equal(c.stopped, true); assert.equal(c.stopCertificate, 'kinematic-equivalence-to-loaded-stop');
    assert.equal(c.kinematicStopProof.supportKey, 'wall:0:0');
    f.joint._splitMotion.phase = 'complete'; f.joint.innerBody.velocityZ[0] = tinyVelocity;
    const certificate = certifyKirchhoffWallFrictionModes(f.joint, fresh, { converged: true });
    f.joint._splitMotion.diagnostics.certified = true;
    commitKirchhoffWallFrictionHistory(f.joint, certificate);
    assert.equal(f.joint.innerBody.velocityZ[0], tinyVelocity, 'no epsilon velocity projection');
    assert.equal(f.joint._wallFrictionHistory.records.get(c.key).stopped, true);
    begin(f, { drive: .4 });
    const next = buildKirchhoffSplitWallFriction(f.joint, [build(f).normal, zeroLoadPoint(f, 0)], DT);
    assert.equal(next.entries[1].modeRecord.mode, 'stick');
    assert.equal(next.entries[1].modeRecord.incomingCertificate, 'committed-stop');
});

test('the same tiny observed speed at a different mobile node is not a kinematic stop proof', () => {
    const f = fixture(); f.joint.innerBody.inverseMass[1] = 1;
    begin(f, { drive: .4 }); solveAttempt(f);
    f.joint._splitMotion.bodies[0].velocityZ.fill(3.6e-9);
    const fresh = buildKirchhoffSplitWallFriction(f.joint, [build(f).normal, zeroLoadPoint(f, 1)], DT);
    const d = evaluateKirchhoffWallFrictionCandidate(f.joint, fresh, { converged: true });
    const c = d.contacts.find(c => c.key === 'split-point-wall:0:1');
    assert.equal(c.normalLambda, 0); assert.equal(c.stopped, false); assert.equal(c.stopCertificate, null);
    assert.equal(c.kinematicStopProof, undefined);
});

test('a freely moving zero-load witness cannot preserve stop history merely because its speed is below the solve error bound', () => {
    const f = fixture(); step(f, { drive: .4 });
    const t = step(f, { drive: 3.6e-9, normalImpulse: 0 });
    assert.equal(t.certificate.contacts[0].stopped, false); assert.equal(f.joint._wallFrictionHistory.records.size, 0);
    near(f.joint.innerBody.velocityZ[0], 3.6e-9, 'tiny physical motion is preserved');
});

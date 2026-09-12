import { captureKirchhoffWallFrictionIncoming, initializeKirchhoffWallFrictionModes,
    evaluateKirchhoffWallFrictionCandidate, prepareKirchhoffWallFrictionRetry,
    certifyKirchhoffWallFrictionModes, commitKirchhoffWallFrictionHistory } from '../../src/physics/kirchhoffWallFrictionMode.js';
import { buildKirchhoffSplitWallFriction, appendKirchhoffSplitWallFriction, commitKirchhoffSplitWallFriction } from '../../src/physics/kirchhoffSplitWallFriction.js';
import { captureKirchhoffCoupledTrialState, restoreKirchhoffCoupledTrialState } from '../../src/physics/kirchhoffCoupledTrialState.js';
import { solveCoupledLoadQP } from '../../src/physics/kirchhoffCoupledLoadSolver.js';

export const DT = 1 / 120, TOL = 1e-10;
const XYZ = ['X', 'Y', 'Z'];
export function body(muStatic = .6, muKinetic = .2) {
    const b = { count: 2, segmentCount: 1, activeStart: 0, activeEnd: 1,
        wallStaticFriction: muStatic, wallKineticFriction: muKinetic, orientationControlSegment: -1,
        nodeRadius: new Float64Array([.5, .5]), materialCoordinate: new Float64Array([0, 1]),
        inverseMass: new Float64Array([1, 0]), wallLambda: new Float64Array(1), wallT: new Float64Array(1),
        wallBranchId: new Int32Array([0]), wallFaceIndex: new Int32Array([0]) };
    for (const a of XYZ) {
        for (const key of [a.toLowerCase(), 'previous' + a, 'velocity' + a, 'angularVelocity' + a]) b[key] = new Float64Array(2);
        b['wall' + a] = new Float64Array(1); b['wallNormal' + a] = new Float64Array([a === 'Y' ? -1 : 0]);
    }
    for (const a of [...XYZ, 'W']) for (const p of ['orientation', 'previousOrientation']) b[p + a] = new Float64Array([a === 'W' ? 1 : 0]);
    for (let a = 1; a <= 3; a++) b['inverseInertia' + a] = new Float64Array(1);
    b.y.fill(-.5); b.previousY.fill(-.5); b.z[1] = b.previousZ[1] = 1;
    return b;
}
export function fixture(muStatic = .6, muKinetic = .2) {
    return { joint: { innerBody: body(muStatic, muKinetic), outerBody: body(0, 0) }, world: { fixedDt: DT, stepCount: 0 }, attempts: [] };
}
export function begin(f, { drive = 0, normalImpulse = 1, initialize = true } = {}) {
    const { joint, world } = f, incoming = captureKirchhoffWallFrictionIncoming(joint, world);
    f.normalImpulse = normalImpulse; f.free = joint.innerBody.velocityZ[0] + drive;
    joint.innerBody.wallLambda.fill(0);
    joint._splitMotion = { phase: 'physical', step: world.stepCount, dt: DT,
        diagnostics: { unverifiedHistoryKinds: [], certified: false },
        bodies: [joint.innerBody, joint.outerBody].map(b => Object.fromEntries(XYZ.flatMap(a =>
            [['velocity' + a, b['velocity' + a].slice()], ['angularVelocity' + a, b['angularVelocity' + a].slice()]]))) };
    joint._splitMotion.bodies[0].velocityZ[0] = f.free;
    joint._splitMotion.bodies[0].velocityY[0] = normalImpulse;
    if (initialize) initializeKirchhoffWallFrictionModes(joint, incoming, { displacementToleranceMm: TOL });
    f.snapshot = captureKirchhoffCoupledTrialState(joint, { reusePropertyLayout: true, frozenFrictionBatches: true });
    return incoming;
}
export function build(f) {
    const normal = { kind: 'wall', side: 0, node: 0, owner: f.joint.innerBody, lambda: f.joint.innerBody.wallLambda[0],
        alpha: 0, strain: -DT * f.normalImpulse, lower: 0, upper: Infinity,
        gradients: [{ side: 0, dof: 1, value: -1 }] };
    const rows = [normal], groups = [], batch = buildKirchhoffSplitWallFriction(f.joint, rows, DT);
    appendKirchhoffSplitWallFriction(batch, rows, groups);
    return { normal, rows, groups, batch };
}
/** A scalar contact with unit mobility in each translational direction. The
 * frozen J W J^T is independently known as I: angular DOFs and node 1 are
 * prescribed. These are REAL wall rows and the native final-load QP solver. */
export function solveAttempt(f, { scale = 1 } = {}) {
    const { normal, batch } = build(f), mu = batch.groups[0].mu;
    const result = solveCoupledLoadQP(new Float64Array([1, 1, 1]), new Float64Array([DT * f.normalImpulse, -DT * f.free, 0]),
        new Float64Array([0, -Infinity, -Infinity]), new Float64Array([Infinity, Infinity, Infinity]), 3, 1,
        [{ rows: [1, 2], lambda: [0, 0], normalRow: 0, normalLambda: 0, mu: [...mu], radii: [0, 0] }], { tolerance: TOL });
    if (!result.diagnostics.converged) throw new Error('Native scalar solve failed: ' + result.diagnostics.status);
    const delta = result.increment, motion = f.joint._splitMotion.bodies[0];
    motion.velocityY[0] -= scale * delta[0] / DT;
    motion.velocityZ[0] += scale * delta[1] / DT;
    motion.velocityX[0] -= scale * delta[2] / DT;
    normal.owner.wallLambda[0] += scale * delta[0];
    commitKirchhoffSplitWallFriction(batch, delta, scale);
    const fresh = build(f).batch, decision = evaluateKirchhoffWallFrictionCandidate(f.joint, fresh, { converged: true });
    f.attempts.push({ mode: batch.entries[0].modeRecord?.mode ?? 'equal', mu: [...mu], scale,
        delta: [...delta], velocity: motion.velocityZ[0], decision: structuredClone(decision) });
    return { decision, fresh, result, batch };
}
export function retry(f, decision) {
    restoreKirchhoffCoupledTrialState(f.snapshot);
    prepareKirchhoffWallFrictionRetry(f.joint, decision);
}
export function finish(f, fresh) {
    const state = f.joint._splitMotion;
    state.phase = 'complete';
    for (const [side, b] of [f.joint.innerBody, f.joint.outerBody].entries()) for (const a of XYZ)
        for (const prefix of ['velocity', 'angularVelocity']) b[prefix + a].set(state.bodies[side][prefix + a]);
    const certificate = certifyKirchhoffWallFrictionModes(f.joint, fresh, { converged: true });
    state.diagnostics.certified = certificate.accepted;
    if (certificate.accepted) commitKirchhoffWallFrictionHistory(f.joint, certificate);
    return certificate;
}
export function step(f, options = {}) {
    f.attempts.length = 0; begin(f, options);
    let trial;
    for (let i = 0; i < 8; i++) {
        trial = solveAttempt(f);
        if (!trial.decision.restart) break;
        retry(f, trial.decision);
    }
    if (trial.decision.accepted) trial.certificate = finish(f, trial.fresh);
    f.world.stepCount++;
    return trial;
}

import { assembleKirchhoffDirect } from './kirchhoffDirectSolver.js';
import { applyKirchhoffSplitPhysicalIncrement } from './kirchhoffSplitMotion.js';
import { multiplyQuaternions, normalizeQuaternion, quaternionExp, rotateVectorByQuaternion } from './discreteKirchhoffRod.js';

const XYZ = ['X', 'Y', 'Z'];
const POSE_KEYS = ['x', 'y', 'z', ...[...XYZ, 'W'].map(a => 'orientation' + a)];
const CONTROL_KEYS = [...XYZ, 'W'].map(a => 'orientationControl' + a);
const bodiesOf = joint => [joint.innerBody, joint.outerBody];
const frame = (pose, segment) => Object.fromEntries([...XYZ, 'W'].map(a => [a.toLowerCase(), pose['orientation' + a][segment]]));

// This weak identity cache owns no mechanics and needs no rollback. Receipts
// keyed by its inert ids live in s.twoChannel.applications and ARE restored.
// Unlike retaining result objects in the snapshot, this does not retain every
// old dense solver matrix or borrowed assembly through later outer passes.
const resultIds = new WeakMap();
let nextResultId = 1;
function resultId(result) {
    let id = resultIds.get(result);
    if (id === undefined) { id = nextResultId++; resultIds.set(result, id); }
    return id;
}
function layout(body) {
    return { count: body.count, segmentCount: body.segmentCount, start: Math.max(0, body.activeStart),
        end: Math.min(body.segmentCount, body.activeEnd), sleeping: Boolean(body.sleeping) };
}
function stateOf(joint) {
    const split = joint._splitMotion, state = split?.twoChannel;
    if (!state || split.dt !== state.dt || split.step !== state.step || split.phase === 'bias')
        throw new Error('Two-channel motion requires its original split step and physical material banks');
    for (const [side, body] of bodiesOf(joint).entries()) {
        const current = layout(body), old = state.layout[side];
        if (Object.keys(current).some(key => current[key] !== old[key]))
            throw new Error('Two-channel material/active-range layout changed within the timestep');
    }
    return state;
}
function validArray(values, count, name) {
    if (!values || values.length < count) throw new RangeError(name + ' has the wrong shape');
    for (let i = 0; i < count; i++) if (!Number.isFinite(values[i])) throw new RangeError(name + ' must be finite');
}
function validateResult(joint, result, state) {
    if (!result?.diagnostics?.converged) throw new Error('Cannot apply an unconverged two-channel result');
    if (!(Number.isFinite(result?.scale) && result.scale >= 0 && result.scale <= 1) ||
        result.physical?.length !== 2 || result.bias?.length !== 2) throw new RangeError('A two-channel result requires one common finite scale');
    for (const [side, body] of bodiesOf(joint).entries()) {
        validArray(result.physical[side].correction, body.count * 6, 'Physical correction');
        validArray(result.bias[side].lambda, (state.layout[side].end - state.layout[side].start) * 6, 'Bias material increment');
    }
}

/** Exactly once after beginSplit, at the predicted pose BEFORE geometric
 * repair. qP is a separate Float64 pose; current body arrays remain qG. No
 * force integration, velocity reconstruction or rest/EI change occurs here. */
export function beginKirchhoffTwoChannelMotion(joint) {
    const split = joint._splitMotion;
    if (!split || split.phase !== 'physical' || !(Number.isFinite(split.dt) && split.dt > 0))
        throw new Error('Begin two-channel motion immediately after physical split prediction');
    if (split.twoChannel) throw new Error('Two-channel motion must not restart qP within an existing timestep');
    const bodies = bodiesOf(joint);
    const state = { dt: split.dt, step: split.step, layout: bodies.map(layout),
        physicalPose: bodies.map(body => Object.fromEntries(POSE_KEYS.map(key => [key, Float64Array.from(body[key])]))),
        materialLambda: bodies.map(body => new Float64Array(body.segmentCount * 6)), applications: new Map() };
    split.twoChannel = state;
    return state;
}

/** BEFORE applying dqP+dqB to the body. dqP's angular coordinates are LOCAL
 * in CURRENT qG, not in qP. Thus qP <- Exp(scale*R_G*dthetaP) qP, while the
 * physical angular velocity gets the same WORLD vector divided by dt.
 * qG, physical material lambdas and every bias normal bank remain root-owned. */
export function applyKirchhoffTwoChannelPhysicalMotion(joint, result) {
    const state = stateOf(joint);
    if (joint._splitMotion.phase !== 'physical') throw new Error('Physical two-channel increments require the physical phase');
    validateResult(joint, result, state);
    const id = resultId(result);
    if (state.applications.get(id)?.physicalApplied) throw new Error('Physical two-channel result already applied');
    const nextFrames = bodiesOf(joint).map((body, side) => {
        const frames = [], p = state.physicalPose[side], correction = result.physical[side].correction;
        for (let segment = state.layout[side].start; segment < state.layout[side].end; segment++) {
            const current = frame(body, segment), previous = frame(p, segment);
            if (![...Object.values(current), ...Object.values(previous)].every(Number.isFinite) ||
                !(Math.hypot(...Object.values(current)) > 0 && Math.hypot(...Object.values(previous)) > 0))
                throw new RangeError('Finite nonzero material frames required');
            const i = segment * 6;
            const world = rotateVectorByQuaternion(current, { x: result.scale * correction[i + 3],
                y: result.scale * correction[i + 4], z: result.scale * correction[i + 5] });
            if (world.x === 0 && world.y === 0 && world.z === 0) { frames.push(previous); continue; }
            const increment = quaternionExp(world, {});
            frames.push(normalizeQuaternion(multiplyQuaternions(increment, previous, {}), {}));
        }
        return frames;
    });
    // The existing hook applies only dqP/dt and records PHYSICAL sheath
    // increments from additionalIncrement. It must never see dqP+dqB here.
    applyKirchhoffSplitPhysicalIncrement(joint, { ...result, inner: result.physical[0], outer: result.physical[1] });
    for (const [side, p] of state.physicalPose.entries()) {
        const { start, end } = state.layout[side], correction = result.physical[side].correction;
        for (let node = start; node <= end; node++) {
            for (let axis = 0; axis < 3; axis++) if (correction[node * 6 + axis] !== 0)
                p[XYZ[axis].toLowerCase()][node] += result.scale * correction[node * 6 + axis];
            if (node === end) continue;
            for (const a of [...XYZ, 'W']) p['orientation' + a][node] = nextFrames[side][node - start][a.toLowerCase()];
        }
    }
    state.applications.set(id, { physicalApplied: true, biasApplied: false, scale: result.scale });
    return state;
}

/** Commit beta_M once, with exactly the scale used by physical motion. Rows
 * use native material LOCAL indexing (row zero belongs to activeStart).
 * These arrays never alias or overwrite the body's physical multipliers. */
export function commitKirchhoffTwoChannelBiasMaterial(joint, result) {
    const state = stateOf(joint);
    validateResult(joint, result, state);
    const receipt = state.applications.get(resultId(result));
    if (!receipt?.physicalApplied || receipt.biasApplied || receipt.scale !== result.scale)
        throw new Error('Bias material commit requires one physical application at the same scale');
    for (let side = 0; side < 2; side++) {
        const count = (state.layout[side].end - state.layout[side].start) * 6;
        for (let row = 0; row < count; row++) state.materialLambda[side][row] += result.scale * result.bias[side].lambda[row];
    }
    receipt.biasApplied = true;
    return state.materialLambda;
}

function assembleAtPose(body, pose, dt) {
    const savedPose = POSE_KEYS.map(key => body[key]);
    const savedControls = CONTROL_KEYS.map(key => ({ key, value: body[key], own: Object.hasOwn(body, key) }));
    try {
        for (const key of POSE_KEYS) body[key] = pose[key];
        const segment = body.orientationControlSegment;
        if (body.orientationControlCompliance === 0 && segment >= body.activeStart && segment < Math.min(body.segmentCount, body.activeEnd)) {
            // Native assembly prescribes the target frame as a side effect.
            // For this measurement, preserve the selected pose exactly while
            // retaining the native hard-control mobility mask.
            for (const a of [...XYZ, 'W']) body['orientationControl' + a] = pose['orientation' + a][segment];
        }
        const material = assembleKirchhoffDirect(body, dt);
        if (!material) return { start: Math.max(0, body.activeStart), end: Math.max(0, body.activeStart), rowCount: 0,
            strain: new Float64Array(0), alpha: new Float64Array(0), lambda: new Float64Array(0), redundantAxes: new Int8Array(0) };
        return { start: material.start, end: material.end, rowCount: material.rowCount,
            strain: material.strain.slice(0, material.rowCount), alpha: material.alpha.slice(0, material.rowCount),
            lambda: material.lambda.slice(0, material.rowCount), redundantAxes: material.redundantAxes.slice(material.start, material.end) };
    } finally {
        POSE_KEYS.forEach((key, i) => { body[key] = savedPose[i]; });
        for (const item of savedControls) {
            if (item.own) body[item.key] = item.value;
            else delete body[item.key];
        }
    }
}
function residualNorm(values) {
    let adaptationMm = 0, bendTwistRad = 0;
    for (let row = 0; row < values.length; row += 6) {
        bendTwistRad = Math.max(bendTwistRad, Math.hypot(values[row], values[row + 1], values[row + 2]));
        adaptationMm = Math.max(adaptationMm, Math.hypot(values[row + 3], values[row + 4], values[row + 5]));
    }
    return { adaptationMm, bendTwistRad };
}

/** Fresh C(qP) and C(qG), without changing either pose, rest data, EI, physical
 * lambda or velocity. qP assembly borrows body pose references temporarily;
 * finally reassembles qG so native scratch/J/mobility refer to current geometry
 * again, including after a failed qP assembly. Returned row arrays are owned.
 * Physical material: C(qG)+alpha*lambda_M.
 * Bias material: C(qG)-C(qP)+alpha*beta_M, with the SAME finite native alpha.
 * Controls/folds need their own correctly transported geometric residuals;
 * quaternion pose differences are not differences of rotation vectors. */
export function measureKirchhoffTwoChannelMaterial(joint, out = {}) {
    const state = stateOf(joint), measurements = [];
    for (const [side, body] of bodiesOf(joint).entries()) {
        const geometryPose = Object.fromEntries(POSE_KEYS.map(key => [key, body[key]]));
        let physical, geometry;
        try { physical = assembleAtPose(body, state.physicalPose[side], state.dt); }
        finally { geometry = assembleAtPose(body, geometryPose, state.dt); }
        if (physical.rowCount !== geometry.rowCount || physical.start !== geometry.start || physical.end !== geometry.end)
            throw new Error('Physical and geometric native material row layouts differ');
        if (physical.redundantAxes.some((axis, i) => axis !== geometry.redundantAxes[i]))
            throw new Error('Physical and geometric redundant material row identities differ');
        const beta = state.materialLambda[side].slice(0, geometry.rowCount);
        const biasStrain = Float64Array.from(geometry.strain, (value, row) => value - physical.strain[row]);
        const physicalResidual = Float64Array.from(geometry.strain, (value, row) => value + geometry.alpha[row] * geometry.lambda[row]);
        const biasResidual = Float64Array.from(biasStrain, (value, row) => value + geometry.alpha[row] * beta[row]);
        measurements.push({ start: geometry.start, end: geometry.end, rowCount: geometry.rowCount,
            physicalPoseStrain: physical.strain, geometryStrain: geometry.strain, biasStrain,
            alpha: geometry.alpha, physicalLambda: geometry.lambda, biasLambda: beta, physicalResidual, biasResidual });
    }
    out.bodies = measurements;
    for (const key of ['physicalPoseStrain', 'geometryStrain', 'biasStrain', 'alpha', 'physicalLambda', 'biasLambda'])
        out[key] = measurements.map(body => body[key]);
    for (const key of ['physicalResidual', 'biasResidual']) {
        const norms = measurements.map(body => residualNorm(body[key]));
        out[key] = { adaptationMm: Math.max(...norms.map(value => value.adaptationMm)), bendTwistRad: Math.max(...norms.map(value => value.bendTwistRad)) };
    }
    out.adaptationMm = Math.max(out.physicalResidual.adaptationMm, out.biasResidual.adaptationMm);
    out.bendTwistRad = Math.max(out.physicalResidual.bendTwistRad, out.biasResidual.bendTwistRad);
    out.finite = measurements.every(body => [body.physicalPoseStrain, body.geometryStrain, body.alpha, body.physicalLambda,
        body.biasLambda, body.physicalResidual, body.biasResidual].every(values => values.every(Number.isFinite)));
    return out;
}

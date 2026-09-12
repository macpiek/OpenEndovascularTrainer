import { createCompositeWallSdfBranchesWorkspace, evaluateCompositeWallSdfBranches } from './kirchhoffCompositeWallSdfBranches.js';
import { createCompositeWallGeometryWorkspace, differentiateCompositeWallContact } from './kirchhoffCompositeWallGeometry.js';
import { createCompositeWallBvhGeometryWorkspace, differentiateCompositeWallBvhContact } from './kirchhoffCompositeWallBvhGeometry.js';
import { createCompositeJointSurfaceIncrementWorkspace, evaluateCompositeJointSurfaceIncrement,
    createCompositeJointSurfaceForceMapWorkspace, evaluateCompositeJointSurfaceForceMap } from './kirchhoffCompositeJointSurfaceMotion.js';
import { readCompositeJointSurfacePosePath } from './kirchhoffCompositeJointSurfacePoseHistory.js';
import { createCompositeJointReservoirSurfaceWorkspace, evaluateCompositeJointReservoirSurface } from './kirchhoffCompositeJointReservoirSurface.js';

const plans = new WeakMap(), N = 7, Q = 10, ROUND = 512 * Number.EPSILON;
const dot = (a, b) => a.reduce((s, x, k) => s + x * b[k], 0), norm = a => Math.hypot(...a);
const sub = (a, b) => a.map((x, k) => x - b[k]), add = (a, b) => a.map((x, k) => x + b[k]), scale = (a, s) => Array.from(a, x => x * s);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const finite = (x, name) => { if (!Number.isFinite(x)) throw new RangeError(`${name} must be finite`); return x; };
function vector(value, n, name) {
    if (value?.length !== n || !Array.from(value).every(Number.isFinite)) throw new RangeError(`${name} needs ${n} finite entries`);
    return Array.from(value);
}
const close = (a, b) => Math.abs(a - b) <= ROUND * Math.max(1, Math.abs(a), Math.abs(b));
function unsupported(reason) { const error = new RangeError(reason); error.code = 'joint-wall-surface-unsupported'; throw error; }
function invalidate(out, reason = 'not-evaluated') {
    out.supported = out.operatorReady = out.incrementValid = out.slipJacobianValid = out.forceMapValid = out.DforceMapValid = false;
    out.reason = reason; out.order = null; out.tools = []; out.identity = out.motion = out.physicalForce = null;
    for (const key of ['increment', 'slipJacobian', 'forceMap', 'DforceMap', 'currentQueryJacobian', 'point', 'normal', 'tangent', 'center']) out[key].fill(NaN);
    out.gap = out.fraction = NaN;
    if ('requiredTransport' in out) out.requiredTransport = null;
}

/** Optional prepared pose path expands G/B/DB to its explicit physical N
 * columns. physicalDofCount remains the seven coordinates of the current
 * contact edge; configurationDofs/Columns describe the expanded packing.
 * No path preserves the original same-edge workspace and evaluation API.
 */
export function createCompositeJointWallSurfaceWorkspace({ surfacePosePath } = {}) {
    const data = surfacePosePath === undefined ? null : readCompositeJointSurfacePosePath(surfacePosePath), N = data?.configurationColumns.length ?? 7;
    const pose = data ? createCompositeJointReservoirSurfaceWorkspace(surfacePosePath) : null;
    const target = data?.edges[data.target], physicalColumns = data ? [
        ...target.nodes.flatMap(j => [0, 1, 2].map(component => data.configurationColumns.findIndex(c => c.kind === 'position' && c.node === data.nodes[j].node && c.component === component))),
        data.configurationColumns.findIndex(c => c.kind === 'angle' && c.edge === target.edge)
    ] : [0, 1, 2, 3, 4, 5, 6];
    const out = { scope: 'stationary-material-wall-affine-surface', physicalDofCount: 7, queryCount: 0, certified: false,
        increment: new Float64Array(2), slipJacobian: new Float64Array(2 * N), forceMap: new Float64Array(N * 2), DforceMap: new Float64Array(N * 2 * N),
        currentQueryJacobian: new Float64Array(Q * N), point: new Float64Array(3), normal: new Float64Array(3), tangent: new Float64Array(3), center: new Float64Array(3),
        wallPointPolicy: 'same-current-original-wall-witness-at-both-times', previousNearestQueryUsed: false, stationaryWallVelocityKnown: true };
    if (pose) {
        Object.assign(out, { configurationDofs: N, configurationColumns: pose.configurationColumns, currentTools: pose.currentTools,
            reservoirIdentity: data.reservoirIdentity, requiredTransport: null });
        // Prepared column identities and owned storage remain stable through
        // value calls, failed evaluations and retries.
        for (const key of ['configurationDofs', 'configurationColumns', 'currentTools', 'reservoirIdentity',
            'increment', 'slipJacobian', 'forceMap', 'DforceMap', 'currentQueryJacobian', 'point', 'normal', 'tangent', 'center'])
            Object.defineProperty(out, key, { writable: false, configurable: false });
    }
    plans.set(out, { sdfBranches: createCompositeWallSdfBranchesWorkspace(1), sdf: [createCompositeWallGeometryWorkspace(1), createCompositeWallGeometryWorkspace(2)],
        bvh: [createCompositeWallBvhGeometryWorkspace(1), createCompositeWallBvhGeometryWorkspace(2)],
        finite: createCompositeJointSurfaceIncrementWorkspace(1), force: createCompositeJointSurfaceForceMapWorkspace(1),
        data, pose, physicalColumns, configurationDofs: N,
        gapGradient: new Float64Array(6), normalJacobian: new Float64Array(18), normalColumn: new Float64Array(6), normalDerivative: new Float64Array(36), busy: false });
    invalidate(out); return out;
}

function verifyPoseConfiguration(input, tool, geometry, plan) {
    const { data, physicalColumns } = plan, target = data.edges[data.target], map = data.maps[data.target];
    const configuration = vector(input.configuration, data.configurationColumns.length, 'Prepared own physical configuration');
    if (input.dt !== data.dt || tool.id !== data.toolId || input.tool.edge !== target.edge || geometry.edge !== target.edge ||
        tool.edgeId !== target.edgeId || tool.materialSegmentId !== target.materialSegmentId)
        unsupported('prepared-pose-target-identity-or-dt-mismatch');
    if (!tool.coordinates.every((x, j) => x === target.coordinates[j]) ||
        geometry.positions.some((p, end) => p.some((x, k) => x !== configuration[physicalColumns[3 * end + k]])) ||
        tool.angle !== configuration[physicalColumns[6]]) unsupported('prepared-pose-current-configuration-mismatch');
    const length = target.coordinates[1] - target.coordinates[0], dsDx = (map.labels[1] - map.labels[0]) / length;
    if (!Number.isFinite(tool.materialMap?.sStart) || !Number.isFinite(tool.materialMap?.dsDx) ||
        !close(tool.materialMap.sStart, map.labels[0]) || !close(tool.materialMap.dsDx, dsDx)) unsupported('prepared-pose-current-material-map-mismatch');
    const rateClose = (rate, expected, label, oldLabel) => Number.isFinite(rate) &&
        Math.abs(rate - expected) <= ROUND * Math.max(1, Math.abs(rate), Math.abs(expected)) + 64 * Number.EPSILON * (Math.abs(label) + Math.abs(oldLabel)) / data.dt;
    const f = geometry.fraction, expected = (1 - f) * map.rates[0] + f * map.rates[1];
    if (!rateClose(tool.materialMap.dsDt, expected, (1-f)*map.labels[0]+f*map.labels[1], (1-f)*target.labels[0]+f*target.labels[1]) ||
        tool.materialMap.dsDtEnds !== undefined && !tool.materialMap.dsDtEnds.every((r, j) => rateClose(r, map.rates[j], map.labels[j], target.labels[j])))
        unsupported('prepared-pose-material-rate-mismatch');
    // The prepared accepted pose is authoritative. Legacy previousPositions
    // are deliberately not read or extrapolated to invent an external pose.
    if (tool.reference !== undefined && ['tangent', 'director'].some(key => !vector(tool.reference[key], 3, 'Own accepted frame').every((x, j) => close(x, target.reference[key][j]))) ||
        tool.previousAngle !== undefined && tool.previousAngle !== target.angle) unsupported('prepared-pose-own-frame-or-spin-mismatch');
    if (tool.materialPath !== undefined) {
        const p = tool.materialPath;
        if (p.kind !== 'linear-affine-maps' || p.previousEdgeId !== target.edgeId ||
            !Number.isFinite(p.previousMap?.sStart) || !Number.isFinite(p.previousMap?.dsDx) ||
            !close(p.previousMap.sStart, target.labels[0]) || !close(p.previousMap.dsDx, (target.labels[1]-target.labels[0])/length))
            unsupported('prepared-pose-accepted-material-map-mismatch');
    }
    return configuration;
}

function verifiedGeometry(current, tool, wall, plan) {
    const { row, field } = current ?? {}, raw = row?.rawContact;
    if (!row || !raw || !field) unsupported('original-wall-row-field-and-raw-contact-required');
    if (!['analytic-plane', 'sparse-sdf', 'sparse-sdf-bvh'].includes(wall.source)) unsupported('explicit-supported-wall-source-required');
    if (row.source !== wall.source || raw.source !== wall.source || current.sdfBranch === undefined && row.derivativeSource !== wall.source) unsupported('wall-source-or-differential-source-changed');
    if (row.included !== true || row.owner !== tool.id || !Number.isInteger(row.edge) || row.edge < 0 || tool.edge !== undefined && tool.edge !== row.edge)
        unsupported('wall-owner-or-physical-edge-mismatch');
    if (current.sdfBranch === undefined && (current.seam != null || row.derivativeUnavailable !== false)) unsupported(`unsupported-original-wall-differential:${row.derivativeReason ?? 'seam'}`);
    const positions = current.positions?.map(p => vector(p, 3, 'Original own wall endpoint'));
    if (positions?.length !== 2 || tool.positions?.length !== 2 || positions.some((p, end) => p.some((x, k) => x !== tool.positions[end][k]))) unsupported('own-positions-do-not-match-original-wall-input');
    const radius = finite(row.radius, 'Original wall radius'); if (!(radius > 0)) unsupported('positive-original-wall-radius-required');
    if (tool.radius !== undefined && tool.radius !== radius) unsupported('own-wall-radius-mismatch');
    const role = row.role ?? 'capsule'; if (!['capsule', 'proximal', 'distal', 'material-point'].includes(role)) unsupported('unsupported-wall-envelope-role');
    const fraction = finite(row.t, 'Original physical wall fraction');
    if (fraction < 0 || fraction > 1 || role === 'proximal' && fraction !== 0 || role === 'distal' && fraction !== 1 || role === 'capsule' && fraction !== raw.segmentT)
        unsupported('wall-sample-or-envelope-fraction-mismatch');
    if (!Number.isInteger(raw.capsuleSampleCount) || raw.capsuleSampleCount < 1 || row.sampleCount !== raw.capsuleSampleCount || row.faceIndex !== raw.faceIndex)
        unsupported('wall-original-sample-or-face-metadata-mismatch');
    const n = vector(raw.inward?.values, 3, 'Original inward normal'), point = vector(raw.closestPoint?.values, 3, 'Original closest wall point');
    if (!close(dot(n, n), 1) || !vector(row.normal, 3, 'Original row normal').every((x, k) => x === n[k]) ||
        !vector(row.closestPoint, 3, 'Original row closest point').every((x, k) => x === point[k])) unsupported('wall-row-and-raw-witness-mismatch');
    const sd = finite(raw.signedDistance, 'Original signed wall distance'), gap = finite(raw.signedGap, 'Original signed wall gap');
    if (row.gap !== gap || !close(gap, sd - radius)) unsupported('wall-gap-or-radius-mismatch');
    const weights = [1 - fraction, fraction], center = positions[0].map((x, k) => fraction === 0 ? x : fraction === 1 ? positions[1][k] : x + (positions[1][k] - x) * fraction);
    if (!point.every((x, k) => close(x, center[k] - sd * n[k]))) unsupported('original-wall-witness-is-not-signed-normal-projection');
    const original = { positions, radius, fraction, weights, n, point, center, sd, gap, role, edge: row.edge,
        sampleCount: raw.capsuleSampleCount, rawFraction: raw.segmentT, faceIndex: raw.faceIndex };
    if (current.sdfBranch !== undefined) return verifiedSdfBranchGeometry(current, wall, plan, original);
    plan.gapGradient.fill(0); plan.normalColumn.fill(0); plan.normalDerivative.fill(0); plan.normalJacobian.fill(0);
    let branchSignature;
    if (wall.source === 'analytic-plane') {
        const pn = vector(current.plane?.normal, 3, 'Declared stationary plane normal'), offset = finite(current.plane?.offset, 'Declared stationary plane offset');
        if (!close(dot(pn, pn), 1) || !pn.every((x, k) => close(x, n[k])) || !close(sd, dot(pn, center) - offset)) unsupported('original-contact-does-not-match-declared-plane');
        for (let j = 0; j < 6; j++) plan.gapGradient[j] = plan.normalColumn[j] = weights[Math.floor(j / 3)] * n[j % 3];
        branchSignature = `analytic-plane:${pn.join(',')}:${offset}:${role}:${fraction}`;
    } else {
        const endpoint = role !== 'capsule', index = endpoint ? 0 : 1;
        const geometry = wall.source === 'sparse-sdf'
            ? differentiateCompositeWallContact({ field, contact: raw, positions: endpoint ? [center] : positions, radius }, plan.sdf[index])
            : differentiateCompositeWallBvhContact({ field, contact: raw, positions: endpoint ? [center] : positions, radius, localFaceIndices: current.localFaceIndices ?? [] }, plan.bvh[index]);
        if (!geometry.supported) unsupported(`unsupported-current-wall-branch:${geometry.reason}`);
        // A material point keeps its fraction when the capsule minimum moves.
        // Pull the point differential back to BOTH native chord endpoints.
        for (let i = 0; i < 6; i++) {
            const gi = endpoint ? i % 3 : i, wi = endpoint ? weights[Math.floor(i / 3)] : 1;
            plan.gapGradient[i] = wi * geometry.gapGradient[gi]; plan.normalColumn[i] = wi * geometry.normalForceColumn[gi];
            for (let j = 0; j < 6; j++) {
                const gj = endpoint ? j % 3 : j, wj = endpoint ? weights[Math.floor(j / 3)] : 1;
                plan.normalDerivative[i * 6 + j] = wi * wj * geometry.normalForceJacobian[gi * (endpoint ? 3 : 6) + gj];
            }
        }
        branchSignature = geometry.branchSignature;
    }
    const originalG = vector(row.gapJacobian, 6, 'Original gap differential'), originalB = vector(row.forceColumn, 6, 'Original signed normal column'), originalDB = vector(row.normalDerivative, 36, 'Original physical normal derivative');
    if (!originalG.every((x, j) => close(x, plan.gapGradient[j])) || !originalB.every((x, j) => close(x, -plan.normalColumn[j])) ||
        !originalDB.every((x, j) => close(x, plan.normalDerivative[j]))) unsupported('stale-original-wall-differential');
    for (let axis = 0; axis < 3; axis++) for (let j = 0; j < 6; j++) plan.normalJacobian[axis * 6 + j] = plan.normalDerivative[axis * 6 + j] + plan.normalDerivative[(3 + axis) * 6 + j];
    return { positions, radius, fraction, weights, n, point, center, sd, gap, role, edge: row.edge, branchSignature, sampleCount: raw.capsuleSampleCount, rawFraction: raw.segmentT, faceIndex: raw.faceIndex };
}

// A branch is an explicit continuation of a proved cell polynomial, never a
// replacement detector witness. The original raw scalar/normal identity above
// is retained and independently checked by the fresh two-cell chart proof.
function verifiedSdfBranchGeometry(current, wall, plan, original) {
    const spec = current.sdfBranch;
    if (wall.source !== 'sparse-sdf' || original.role !== 'material-point' || !spec ||
        ![0, 1].includes(spec.branchIndex) || spec.fraction !== original.fraction)
        unsupported('invalid-explicit-sdf-material-point-branch');
    const chart = evaluateCompositeWallSdfBranches({ field: current.field, face: spec.face,
        positions: [original.center], radius: original.radius, contact: current.row.rawContact,
        domainBox: spec.domainBox }, plan.sdfBranches);
    if (!chart.supported) unsupported(`unsupported-explicit-sdf-branch:${chart.reason}`);
    const branch = chart.rows[spec.branchIndex], { weights } = original;
    if (!branch.enabled) unsupported('disabled-explicit-sdf-branch');
    for (let i = 0; i < 6; i++) {
        const wi = weights[Math.floor(i / 3)], a = i % 3;
        plan.gapGradient[i] = wi * branch.pointGapGradient[a];
        plan.normalColumn[i] = wi * branch.normal[a];
        for (let j = 0; j < 6; j++) plan.normalDerivative[i * 6 + j] =
            wi * weights[Math.floor(j / 3)] * branch.pointNormalDerivative[a * 3 + j % 3];
    }
    for (let a = 0; a < 3; a++) for (let j = 0; j < 6; j++)
        plan.normalJacobian[a * 6 + j] = weights[Math.floor(j / 3)] * branch.pointNormalDerivative[a * 3 + j % 3];
    const identity = { branchIndex: spec.branchIndex, face: { ...chart.face },
        domainBox: { lower: Array.from(chart.domain.lower), upper: Array.from(chart.domain.upper) }, fraction: original.fraction };
    return { ...original, n: Array.from(branch.normal), sd: branch.signedDistance, gap: branch.gap,
        point: original.center.map((x, k) => x - branch.signedDistance * branch.normal[k]),
        branchSignature: `proved-sdf-material-point:${JSON.stringify(identity)}`, sdfBranch: identity };
}

function tangentQuery(tool, geometry, basis, plan, full) {
    const chord = sub(geometry.positions[1], geometry.positions[0]), length = finite(norm(chord), 'Own wall chord length');
    if (!(length > 1e-12)) unsupported('degenerate-own-wall-tangent');
    const tangent = scale(chord, 1 / length), Dt = new Float64Array(3 * N);
    if (full) for (let k = 0; k < 3; k++) for (let j = 0; j < 6; j++) Dt[k * N + j] = (j < 3 ? -1 : 1) * ((k === j % 3 ? 1 : 0) - tangent[k] * tangent[j % 3]) / length;
    let direction = tangent, derivative = Dt;
    if (basis === 'projected-own-reference-director') {
        const oldT = vector(tool.reference?.tangent, 3, 'Own accepted tangent'), oldD = vector(tool.reference?.director, 3, 'Own accepted director');
        const denominator = 1 + dot(oldT, tangent); if (!(denominator > 1e-10)) unsupported('antiparallel-own-reference-wall-chart');
        const axis = cross(oldT, tangent), first = cross(axis, oldD), second = cross(axis, first);
        direction = add(add(oldD, first), scale(second, 1 / denominator)); derivative = new Float64Array(3 * N);
        if (full) for (let j = 0; j < 6; j++) {
            const dt = [0, 1, 2].map(k => Dt[k * N + j]), da = cross(oldT, dt), df = cross(da, oldD), ds = add(cross(da, first), cross(axis, df)), dd = dot(oldT, dt);
            for (let k = 0; k < 3; k++) derivative[k * N + j] = df[k] + ds[k] / denominator - second[k] * dd / (denominator * denominator);
        }
    } else if (basis !== 'projected-own-tangent') unsupported('explicit-supported-wall-tangent-basis-required');
    const axial = dot(direction, geometry.n), projected = sub(direction, scale(geometry.n, axial));
    if (!(norm(projected) > 1e-10)) unsupported('degenerate-wall-tangent-projection-select-explicit-own-reference-basis');
    const D = new Float64Array(3 * N);
    if (full) for (let j = 0; j < 6; j++) {
        const dn = [0, 1, 2].map(k => plan.normalJacobian[k * 6 + j]), dd = [0, 1, 2].map(k => derivative[k * N + j]), da = dot(dd, geometry.n) + dot(direction, dn);
        for (let k = 0; k < 3; k++) D[k * N + j] = dd[k] - da * geometry.n[k] - axial * dn[k];
    }
    return { projected, derivative: D };
}

/** The current original wall point/frame names ONE stationary wall material
 * point at both ends of this dt. It is NOT the previous nearest-point query.
 * Both finite query blocks therefore have the SAME current geometry chain
 * derivative; tracing each current TOOL label remains independent.
 * Analytic-plane requires current.plane={normal,offset}; SDF/BVH use their
 * original smooth branch differentiator without detection. wall.source is
 * the caller's prepared source expectation, never inferred after a switch.
 * A prepared surfacePosePath additionally requires configuration in the
 * path's declared order and the target's type-preserving materialSegmentId.
 * It supplies the accepted frame/pose and any explicit proximal reservoir;
 * legacy previousPositions are unused. Both of its query blocks are chained
 * through this same current witness, including all N configuration columns.
 */
export function evaluateCompositeJointWallSurface(input, out = createCompositeJointWallSurfaceWorkspace()) {
    const plan = plans.get(out); if (!plan) throw new TypeError('Use a prepared joint wall surface workspace');
    if (plan.busy) throw new RangeError('Joint wall surface workspace is busy'); plan.busy = true; invalidate(out);
    try {
        const { current, tool, dt, wall, order = 'full' } = input ?? {}, full = order === 'full';
        const N = plan.configurationDofs;
        if (order !== 'full' && order !== 'value') throw new RangeError('Wall surface order must be full or value');
        if (wall?.motion !== 'stationary-material') unsupported('explicit-stationary-wall-material-required');
        if (!(finite(dt, 'Wall surface dt') > 0) || !tool) throw new RangeError('Positive dt and one own physical tool are required');
        const geometry = verifiedGeometry(current, tool, wall, plan), coordinates = vector(tool.coordinates, 2, 'Own material coordinate span'), length = finite(coordinates[1] - coordinates[0], 'Own coordinate length');
        if (!(length > 0)) unsupported('increasing-own-coordinate-span-required');
        const coordinate = coordinates[0] + geometry.fraction * length;
        if (tool.coordinate !== undefined && (!Number.isFinite(tool.coordinate) || !close(tool.coordinate, coordinate))) unsupported('stale-own-wall-coordinate');
        let materialMap = tool.materialMap;
        if (materialMap?.dsDtEnds !== undefined) {
            const rates = vector(materialMap.dsDtEnds, 2, 'Own endpoint label rates'), dsDt = (1 - geometry.fraction) * rates[0] + geometry.fraction * rates[1];
            if (materialMap.dsDt !== undefined && !close(materialMap.dsDt, dsDt)) unsupported('stale-own-wall-label-rate');
            materialMap = { ...materialMap, dsDt };
        }
        const preparedTool = { ...tool, edge: geometry.edge, coordinates, coordinate, materialMap }, basis = wall.tangentBasis ?? 'projected-own-tangent';
        const configuration = plan.pose ? verifyPoseConfiguration(input, preparedTool, geometry, plan) : null;
        if (plan.pose) preparedTool.reference = plan.data.edges[plan.data.target].reference;
        const tangent = tangentQuery(preparedTool, geometry, basis, plan, full);
        out.point.set(geometry.point); out.normal.set(geometry.n); out.tangent.set(tangent.projected); out.center.set(geometry.center); out.gap = geometry.gap; out.fraction = geometry.fraction;
        if (full) {
            out.currentQueryJacobian.fill(0);
            for (let j = 0; j < 6; j++) for (let k = 0; k < 3; k++) {
                const column = plan.physicalColumns[j];
                const dn = plan.normalJacobian[k * 6 + j], dx = k === j % 3 ? geometry.weights[Math.floor(j / 3)] : 0;
                out.currentQueryJacobian[(1 + k) * N + column] = dx - geometry.n[k] * plan.gapGradient[j] - geometry.sd * dn;
                out.currentQueryJacobian[(4 + k) * N + column] = dn;
                out.currentQueryJacobian[(7 + k) * N + column] = tangent.derivative[k * 7 + j];
            }
        }
        const query = { point: out.point, normal: out.normal, tangent: out.tangent };
        const implicitRate=wall.rateMode==='backward-euler-grid';
        if(wall.rateMode!==undefined&&!implicitRate)unsupported('unknown-wall-surface-rate-model');
        if(implicitRate&&plan.pose)unsupported('implicit-local-rate-does-not-use-a-finite-pose-path');
        const motion = implicitRate?null:plan.pose ? evaluateCompositeJointReservoirSurface({ configuration, order,
            query: { coordinate, trace: tool.trace, previousTrace: tool.materialPath?.previousTrace },
            finiteGeometry: { kind: 'explicit-affine-side-queries', current: query, previous: query } }, plan.pose)
            : evaluateCompositeJointSurfaceIncrement({ tools: [preparedTool], dt, order, wall: { velocity: [0, 0, 0] },
            rotationPath: 'short-contact-frame-own-unwrapped-spins', finiteGeometry: { kind: 'explicit-affine-side-queries', current: query, previous: query } }, plan.finite);
        const force = plan.pose ? motion.physicalForce : evaluateCompositeJointSurfaceForceMap({ tools: [preparedTool], order, forceGeometry: { kind: 'explicit-affine-side-query', ...query } }, plan.force);
        if (full) {
            if(!implicitRate)for (let row = 0; row < 2; row++) for (let j = 0; j < N; j++) {
                let value = motion.configurationJacobian[row * N + j];
                for (let k = 0; k < Q; k++) value += (motion.queryJacobian[row * motion.queryDofs + k] + (k === 0 ? 0 : motion.queryJacobian[row * motion.queryDofs + k + 9])) * out.currentQueryJacobian[k * N + j];
                out.slipJacobian[row * N + j] = value;
            }
            for (let entry = 0; entry < N * 2; entry++) for (let j = 0; j < N; j++) {
                let value = plan.pose ? motion.DforceMap[entry * N + j] : force.configurationDerivative[entry * N + j];
                for (let k = 0; k < Q; k++) value += (plan.pose
                    ? motion.forceMapQueryDerivative[entry * 19 + k] + (k === 0 ? 0 : motion.forceMapQueryDerivative[entry * 19 + k + 9])
                    : force.queryDerivative[entry * Q + k]) * out.currentQueryJacobian[k * N + j];
                out.DforceMap[entry * N + j] = value;
            }
        }
        out.forceMap.set(plan.pose ? motion.forceMap : force.forceMap);
        let rateMotion=null;
        if(implicitRate) {
            const old=tool.previousPositions?.map(p=>vector(p,3,'Own previous rate endpoint')),metric=finite(materialMap.dsDx,'Own material metric'),
                feed=finite(materialMap.dsDt,'Own material feed')/metric;
            if(old?.length!==2||!(metric>0))unsupported('implicit-rate-needs-owned-previous-geometry-and-positive-metric');
            const qx=geometry.positions[1].map((v,k)=>(v-geometry.positions[0][k])/length),rates=new Float64Array(7),Drates=new Float64Array(49);
            // Grid pose is linear in time. On one open native edge the director
            // field is spatially constant; feed acts on translation. Its same
            // velocity at both endpoints adds no artificial bending rotation.
            for(let end=0;end<2;end++)for(let k=0;k<3;k++) {
                const i=3*end+k;rates[i]=(geometry.positions[end][k]-old[end][k])/dt-feed*qx[k];
                Drates[i*7+i]=1/dt;Drates[i*7+k]+=feed/length;Drates[i*7+3+k]-=feed/length;
            }
            rates[6]=(finite(tool.angle,'Own current angle')-finite(tool.previousAngle,'Own previous angle'))/dt;Drates[48]=1/dt;
            out.increment.fill(0);if(full)out.slipJacobian.fill(0);
            for(let c=0;c<2;c++)for(let i=0;i<7;i++) {
                out.increment[c]+=dt*out.forceMap[2*i+c]*rates[i];
                if(full)for(let j=0;j<7;j++)out.slipJacobian[c*7+j]+=dt*(out.DforceMap[(2*i+c)*7+j]*rates[i]+out.forceMap[2*i+c]*Drates[i*7+j]);
            }
            rateMotion={rateMode:'backward-euler-grid',rateModel:'endpoint-derivative-of-linear-grid-pose-path',slipModel:'implicit-backward-euler-surface-rate',
                finiteStepSlipKnown:false,includesHingeTransport:false,increment:out.increment,axes:force.axes,
                materialLabel:materialMap.sStart+metric*(coordinate-coordinates[0]),rates,spatialOrientationField:'constant-on-own-open-DER-edge'};
        } else out.increment.set(motion.increment);
        for (const key of full ? ['increment', 'slipJacobian', 'forceMap', 'DforceMap', 'currentQueryJacobian'] : ['increment', 'forceMap']) if (!out[key].every(Number.isFinite)) unsupported('nonfinite-composed-wall-surface-operator');
        out.tools = plan.pose ? out.currentTools : [{ id: tool.id, edgeId: tool.edgeId, edge: geometry.edge }]; out.motion = rateMotion??motion; out.physicalForce = force;
        out.identity = { source: wall.source, branchSignature: geometry.branchSignature, role: geometry.role, fraction: geometry.fraction,
            rawFraction: geometry.rawFraction, sampleCount: geometry.sampleCount, faceIndex: geometry.faceIndex, radius: geometry.radius, basis,
            ...(geometry.sdfBranch ? { sdfBranch: geometry.sdfBranch } : {}),
            wallWitness: geometry.sdfBranch ? 'proved-sdf-branch-normal-projection' : wall.source === 'sparse-sdf' ? 'provider-normal-projection-not-exact-isosurface' : 'original-geometric-wall-foot' };
        out.supported = out.incrementValid = out.forceMapValid = true; out.operatorReady = out.slipJacobianValid = out.DforceMapValid = full; out.order = order; out.reason = null;
        return out;
    } catch (error) { invalidate(out, error.message); if (plan.pose) out.requiredTransport = error.requiredTransport ? structuredClone(error.requiredTransport) : null; throw error; } finally { plan.busy = false; }
}

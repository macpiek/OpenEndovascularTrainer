import { createCompositeLumenSideGeometryWorkspace, differentiateCompositeLumenSideContact } from './kirchhoffCompositeLumenSideGeometry.js';
import { createCompositeJointSurfaceIncrementWorkspace, evaluateCompositeJointSurfaceIncrement,
    createCompositeJointSurfaceForceMapWorkspace, evaluateCompositeJointSurfaceForceMap } from './kirchhoffCompositeJointSurfaceMotion.js';

const plans = new WeakMap(), N = 14, Q = 11, EPS = 1e-12;
const dot = (a, b) => a.reduce((s, x, k) => s + x * b[k], 0);
const norm = a => Math.hypot(...a);
const finite = (x, name) => { if (!Number.isFinite(x)) throw new RangeError(`${name} must be finite`); return x; };
const vector = (v, name) => {
    if (!v || v.length !== 3 || !Array.from(v).every(Number.isFinite)) throw new RangeError(`${name} needs three finite entries`);
    return Array.from(v);
};
const positionColumn = j => j < 6 ? j : j + 1;
const sameNumber = (a, b) => Math.abs(a - b) <= 128 * Number.EPSILON * Math.max(1, Math.abs(a), Math.abs(b));
function unsupported(reason) { const error = new RangeError(reason); error.code = 'joint-lumen-surface-unsupported'; throw error; }
function invalidate(out, reason = 'not-evaluated') {
    out.supported = out.operatorReady = out.incrementValid = out.slipJacobianValid = out.forceMapValid = out.DforceMapValid = false;
    out.reason = reason; out.certified = false; out.queryCount = 0; out.tools = []; out.motion = out.physicalForce = null; out.identity = null; out.order = null;
    for (const key of ['increment', 'slipJacobian', 'forceMap', 'DforceMap', 'currentQueryJacobian']) out[key].fill(NaN);
    for (const query of [out.currentWitness, out.previousWitness]) {
        for (const key of ['point', 'normal', 'tangent', 'innerSurface', 'outerSurface', 'separation']) query[key].fill(NaN);
        query.separationNorm = query.innerT = query.outerT = NaN;
    }
}
const witness = () => ({ point: new Float64Array(3), normal: new Float64Array(3), tangent: new Float64Array(3),
    innerSurface: new Float64Array(3), outerSurface: new Float64Array(3), separation: new Float64Array(3), separationNorm: NaN, innerT: NaN, outerT: NaN });

/** Reusable, invalidated-on-refresh scratch for ONE declared side sample.
 * Physical order is [inner q0.xyz,q1.xyz,theta, outer q0.xyz,q1.xyz,theta].
 * Both supplied original records are reused; this module performs ZERO
 * detector queries, winner searches, manifold changes or history commits.
 */
export function createCompositeJointLumenSurfaceWorkspace() {
    const out = { scope: 'strict-side-affine-joint-lumen-surface', physicalDofCount: N, queryCount: 0,
        increment: new Float64Array(2), slipJacobian: new Float64Array(2 * N), forceMap: new Float64Array(N * 2),
        DforceMap: new Float64Array(N * 2 * N), currentQueryJacobian: new Float64Array(Q * N),
        currentWitness: witness(), previousWitness: witness(), virtualCommonPoint: true,
        witnessPolicy: 'midpoint-of-inner-projected-radial-and-outer-lumen-witnesses',
        exactCylinderIntersection: false, previousQueryPolicy: 'immutable-original-fixed-inner-sample',
        currentLabelHistoryPolicy: 'trace-each-current-label-through-own-old-affine-map' };
    plans.set(out, { current: createCompositeLumenSideGeometryWorkspace(), previous: createCompositeLumenSideGeometryWorkspace(),
        motion: createCompositeJointSurfaceIncrementWorkspace(2), force: createCompositeJointSurfaceForceMapWorkspace(2), busy: false });
    invalidate(out); return out;
}

function composeWitness(geometry, input, out, derivatives = null) {
    const [A, W] = geometry.positions, chord = W.map((x, k) => x - A[k]), length = finite(norm(chord), 'Inner physical chord length');
    if (!(length > EPS)) unsupported('degenerate-inner-surface-tangent');
    const tangent = Array.from(chord, x => x / length), n = geometry.normal, axial = dot(n, tangent);
    const projected = tangent.map((x, k) => n[k] - axial * x), projectedLength = finite(norm(projected), 'Projected inner radial length');
    if (!(projectedLength > EPS)) unsupported('undefined-projected-inner-radial-witness');
    const radial = projected.map(x => x / projectedLength), ri = input.innerRadius, ro = input.lumenRadius;
    for (let k = 0; k < 3; k++) {
        out.innerSurface[k] = geometry.innerPoint[k] + ri * radial[k];
        out.outerSurface[k] = geometry.outerPoint[k] + ro * n[k];
        out.point[k] = .5 * (out.innerSurface[k] + out.outerSurface[k]);
        out.separation[k] = out.innerSurface[k] - out.outerSurface[k];
    }
    out.normal.set(n); out.tangent.set(geometry.outerDirection); out.separationNorm = norm(out.separation);
    out.innerT = geometry.innerT; out.outerT = geometry.outerT;
    if (derivatives) {
        derivatives.fill(0);
        for (let j = 0; j < 12; j++) {
            const block = Math.floor(j / 3), axis = j % 3, column = positionColumn(j), dt = geometry.outerTGradient[j];
            const dn = [0, 1, 2].map(k => geometry.normalJacobian[k * 12 + j]);
            const chordSign = block === 0 ? -1 : block === 1 ? 1 : 0;
            const dTangent = tangent.map((x, k) => chordSign * ((k === axis ? 1 : 0) - x * tangent[axis]) / length);
            const dAxial = dot(dn, tangent) + dot(n, dTangent);
            const dProjected = tangent.map((x, k) => dn[k] - dAxial * x - axial * dTangent[k]);
            const dNorm = dot(radial, dProjected), dRadial = radial.map((x, k) => (dProjected[k] - x * dNorm) / projectedLength);
            const outerDirectionSign = block === 2 ? -1 : block === 3 ? 1 : 0;
            for (let k = 0; k < 3; k++) {
                const dp = block < 2 && k === axis ? geometry.innerWeights[block] : 0;
                const dq = (block >= 2 && k === axis ? geometry.outerWeights[block - 2] : 0) + geometry.outerDirection[k] * dt;
                derivatives[(2 + k) * N + column] = .5 * (dp + ri * dRadial[k] + dq + ro * dn[k]);
                derivatives[(5 + k) * N + column] = dn[k];
                derivatives[(8 + k) * N + column] = k === axis ? outerDirectionSign : 0;
            }
        }
    }
}

/** Compose original strict-side geometry with finite slip G and physical B/DB.
 * Each tool also supplies materialSegmentId matching its detector input and
 * a provider edgeId (plus optional layout edge index for SurfacePullback).
 * Its own current/previous endpoints MUST equal the respective detector input.
 * Current feet are DERIVED from fixed original inner s and moving outer t
 * using tools[i].coordinates; an optional supplied coordinate must agree.
 * materialMap.dsDtEnds:[rate0,rate1], when present, is interpolated at this
 * derived fraction. An additionally supplied scalar dsDt must agree. These
 * endpoint rates and both material maps remain frozen preparation parameters.
 * Previous query geometry is immutable at that same declared original s.
 * Current material labels still select their OWN old centers inside the
 * finite operator, including the current moving outer foot derivative.
 *
 * The common force point is the midpoint of the inner cylinder radial witness
 * (project original n perpendicular to the own inner chord) and outer lumen
 * radial witness. This is an explicit virtual common-force-point extension
 * of the detector's gap, NOT an exact intersection of two finite cylinders.
 * The original detector normal/gap are unchanged. Both tools use this same
 * point, preserving action/reaction and physical wrench. See separation data.
 * Any failure revokes all old map validity and numeric scratch before throwing.
 * order:'value' preserves all geometry/history validation and returns only
 * the finite increment and physical B. G/DB/query-derivative buffers stay NaN,
 * their validity is false and operatorReady is false. Full is the default.
 */
export function evaluateCompositeJointLumenSurface(input, out = createCompositeJointLumenSurfaceWorkspace()) {
    const plan = plans.get(out); if (!plan) throw new TypeError('Use a prepared joint lumen surface workspace');
    if (plan.busy) throw new RangeError('Joint lumen surface workspace is busy');
    plan.busy = true; invalidate(out);
    try {
        const { current, previous, tools, dt, order = 'full' } = input ?? {}, full = order === 'full';
        if (order !== 'full' && order !== 'value') throw new RangeError('Joint lumen surface order must be full or value');
        if (!(finite(dt, 'Surface history dt') > 0)) throw new RangeError('Surface history dt must be positive');
        if (!current?.input || !previous?.input || !current.contact || !previous.contact) unsupported('current-and-previous-original-side-records-required');
        if (!Array.isArray(tools) || tools.length !== 2 || new Set(tools.map(t => t.id)).size !== 2) throw new RangeError('Ordered distinct inner and outer physical tools are required');
        for (const state of [current, previous]) {
            if (!(state.input.innerRadius > 0 && state.input.lumenRadius > state.input.innerRadius)) unsupported('strict-positive-inner-and-lumen-radii-required');
        }
        for (const key of ['innerMaterialSegmentId', 'outerMaterialSegmentId', 'innerRadius', 'lumenRadius'])
            if (current.input[key] !== previous.input[key]) unsupported(`changed-original-${key}`);
        if (current.contact.innerT !== previous.contact.innerT || current.contact.id !== previous.contact.id || current.contact.feature !== previous.contact.feature)
            unsupported('changed-original-sample-or-contact-identity');
        // Surface G/B/DB need first derivatives of the current normal and
        // witness, not the normal-contact Hessian. Old witness geometry is
        // fixed input even in a full surface evaluation.
        differentiateCompositeLumenSideContact(current, plan.current, {order:full?'witness':'gradient'}); if (!plan.current.supported) unsupported(`current:${plan.current.reason}`);
        differentiateCompositeLumenSideContact(previous, plan.previous, {order:'gradient'}); if (!plan.previous.supported) unsupported(`previous:${plan.previous.reason}`);
        if (!(plan.current.innerT > 0 && plan.current.innerT < 1)) unsupported('strict-interior-inner-side-sample-required');
        const preparedTools = tools.map((tool, index) => {
            const materialKey = index === 0 ? 'innerMaterialSegmentId' : 'outerMaterialSegmentId';
            if (!Object.hasOwn(tool, 'materialSegmentId') || tool.materialSegmentId !== current.input[materialKey]) unsupported('tool-material-segment-identity-mismatch');
            if (tool.edge !== undefined && (!Number.isInteger(tool.edge) || tool.edge < 0)) throw new RangeError('Optional physical layout edge must be a nonnegative integer');
            if (tool.positions?.length !== 2 || tool.previousPositions?.length !== 2) throw new RangeError('Each tool needs own current and previous endpoints');
            const positions = tool.positions.map(p => vector(p, 'Own current endpoint')), previousPositions = tool.previousPositions.map(p => vector(p, 'Own previous endpoint'));
            for (let end = 0; end < 2; end++) for (let k = 0; k < 3; k++) {
                if (positions[end][k] !== plan.current.positions[2 * index + end][k] || previousPositions[end][k] !== plan.previous.positions[2 * index + end][k])
                    unsupported('tool-endpoints-do-not-match-original-detector-input');
            }
            if (tool.coordinates?.length !== 2 || !Array.from(tool.coordinates).every(Number.isFinite)) throw new RangeError('Own coordinate span needs two finite endpoints');
            const coordinates = Array.from(tool.coordinates), length = finite(coordinates[1] - coordinates[0], 'Own coordinate length');
            if (!(length > 0)) throw new RangeError('Own coordinate span must increase');
            const fraction = index === 0 ? plan.current.innerT : plan.current.outerT, coordinate = coordinates[0] + length * fraction;
            if (tool.coordinate !== undefined && (!Number.isFinite(tool.coordinate) || !sameNumber(tool.coordinate, coordinate))) unsupported('supplied-tool-coordinate-is-stale');
            let materialMap = tool.materialMap;
            if (materialMap?.dsDtEnds !== undefined) {
                if (materialMap.dsDtEnds?.length !== 2 || !Array.from(materialMap.dsDtEnds).every(Number.isFinite)) throw new RangeError('Own material dsDtEnds needs two finite endpoint rates');
                const dsDt = finite((1 - fraction) * materialMap.dsDtEnds[0] + fraction * materialMap.dsDtEnds[1], 'Own interpolated label rate');
                if (materialMap.dsDt !== undefined && (!Number.isFinite(materialMap.dsDt) || !sameNumber(materialMap.dsDt, dsDt))) unsupported('supplied-scalar-label-rate-is-stale');
                materialMap = { ...materialMap, dsDt };
            }
            return { ...tool, coordinates, coordinate, positions, previousPositions, materialMap };
        });
        composeWitness(plan.current, current.input, out.currentWitness, full ? out.currentQueryJacobian : null);
        composeWitness(plan.previous, previous.input, out.previousWitness);
        const outerSpan = preparedTools[1].coordinates[1] - preparedTools[1].coordinates[0];
        if (full) for (let j = 0; j < 12; j++) out.currentQueryJacobian[N + positionColumn(j)] = outerSpan * plan.current.outerTGradient[j];
        const query = witness => ({ point: witness.point, normal: witness.normal, tangent: witness.tangent });
        const motion = evaluateCompositeJointSurfaceIncrement({ tools: preparedTools, dt, order, rotationPath: 'short-contact-frame-own-unwrapped-spins',
            finiteGeometry: { kind: 'explicit-affine-side-queries', current: query(out.currentWitness), previous: query(out.previousWitness) } }, plan.motion);
        const force = evaluateCompositeJointSurfaceForceMap({ tools: preparedTools, order, forceGeometry: { kind: 'explicit-affine-side-query', ...query(out.currentWitness) } }, plan.force);
        if (full) for (let row = 0; row < 2; row++) for (let j = 0; j < N; j++) {
            let value = motion.configurationJacobian[row * N + j];
            for (let k = 0; k < Q; k++) value += motion.queryJacobian[row * motion.queryDofs + k] * out.currentQueryJacobian[k * N + j];
            out.slipJacobian[row * N + j] = value;
        }
        if (full) for (let entry = 0; entry < N * 2; entry++) for (let j = 0; j < N; j++) {
            let value = force.configurationDerivative[entry * N + j];
            for (let k = 0; k < Q; k++) value += force.queryDerivative[entry * Q + k] * out.currentQueryJacobian[k * N + j];
            out.DforceMap[entry * N + j] = value;
        }
        out.increment.set(motion.increment); out.forceMap.set(force.forceMap);
        for (const key of (full ? ['increment', 'slipJacobian', 'forceMap', 'DforceMap', 'currentQueryJacobian'] : ['increment', 'forceMap']))
            if (!out[key].every(Number.isFinite)) unsupported('nonfinite-composed-surface-operator');
        out.tools = preparedTools.map(t => ({ id: t.id, edgeId: t.edgeId, ...(t.edge !== undefined ? { edge: t.edge } : {}) }));
        out.identity = { contactId: current.contact.id, feature: current.contact.feature, innerT: plan.current.innerT,
            innerMaterialSegmentId: current.input.innerMaterialSegmentId, outerMaterialSegmentId: current.input.outerMaterialSegmentId,
            innerRadius: current.input.innerRadius, lumenRadius: current.input.lumenRadius, edgeIds: preparedTools.map(t => t.edgeId), coordinateSpans: preparedTools.map(t => t.coordinates.slice()) };
        out.motion = motion; out.physicalForce = force;
        out.supported = out.incrementValid = out.forceMapValid = true;
        out.operatorReady = out.slipJacobianValid = out.DforceMapValid = full; out.reason = null; out.order = order;
        return out;
    } catch (error) { invalidate(out, error.message); throw error; } finally { plan.busy = false; }
}

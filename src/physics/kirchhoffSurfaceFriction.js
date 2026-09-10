const EPSILON = 1e-12;
const PARTITION_TOLERANCE = 1e-8;
const XYZ = ['x', 'y', 'z'];
const XYZW = ['X', 'Y', 'Z', 'W'];

function finite(value, name) {
    if (!Number.isFinite(value)) throw new TypeError(`${name} must be finite`);
    return value;
}

function nonNegative(value, name) {
    finite(value, name);
    if (value < 0) throw new RangeError(`${name} must be non-negative`);
    return value;
}

function vector(source, target, name) {
    if (source == null) throw new TypeError(`${name} is required`);
    for (let i = 0; i < 3; i++) target[i] = finite(source[i] ?? source[XYZ[i]], `${name}[${i}]`);
    return target;
}

function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

function cross(a, b, out) {
    const x = a[1] * b[2] - a[2] * b[1], y = a[2] * b[0] - a[0] * b[2];
    const z = a[0] * b[1] - a[1] * b[0];
    out[0] = x; out[1] = y; out[2] = z;
    return out;
}

function normalize(v) {
    const length = Math.hypot(...v);
    if (length <= EPSILON) return false;
    for (let i = 0; i < 3; i++) v[i] /= length;
    return true;
}

function projectPlane(source, normal, out) {
    const axial = dot(source, normal);
    for (let i = 0; i < 3; i++) out[i] = source[i] - normal[i] * axial;
    return normalize(out);
}

function quaternion(body, segment, previous, out) {
    const prefix = previous ? 'previousOrientation' : 'orientation';
    for (let i = 0; i < 4; i++) out[i] = finite(body[prefix + XYZW[i]]?.[segment], `${prefix}[${segment}]`);
    const length = Math.hypot(...out);
    if (length <= EPSILON) throw new RangeError('Material quaternion must have nonzero norm');
    for (let i = 0; i < 4; i++) out[i] /= length;
    return out;
}

// q is normalized by quaternion(); inverse means R(q)^T, not a left-sided dof.
function rotate(q, v, out, inverse = false) {
    const sign = inverse ? -1 : 1, x = q[0] * sign, y = q[1] * sign, z = q[2] * sign, w = q[3];
    const tx = 2 * (y * v[2] - z * v[1]);
    const ty = 2 * (z * v[0] - x * v[2]);
    const tz = 2 * (x * v[1] - y * v[0]);
    const vx = v[0] + w * tx + y * tz - z * ty;
    const vy = v[1] + w * ty + z * tx - x * tz;
    const vz = v[2] + w * tz + x * ty - y * tx;
    out[0] = vx; out[1] = vy; out[2] = vz;
    return out;
}

function scratchState(out) {
    if (out._surfaceScratch) return out._surfaceScratch;
    const v3 = () => new Float64Array(3);
    const side = () => ({ nodes: [], weights: [], count: 0, segment: 0,
        center: v3(), previousCenter: v3(), lever: v3(), localLever: v3(), previousLever: v3(),
        displacement: v3(), q: new Float64Array(4), previousQ: new Float64Array(4),
        director: v3(), witnessDirection: v3(), witness: v3() });
    const row = () => ({ strain: 0, alpha: 0, lambda: 0, lower: -Infinity, upper: Infinity, gradients: [] });
    const state = out._surfaceScratch = { sides: [side(), side()], bodies: [null, null], radii: new Float64Array(2), torque: v3(), localTorque: v3(),
        axial: v3(), oldForce: v3(), normal: v3(), u: v3(), v: v3(),
        rows: [row(), row()], basisZ: new Float64Array([0, 0, 1]) };
    out.rows = [];
    out.group = { kind: 'coulomb-disk', rowIndices: new Uint32Array([0, 1]),
        mu: new Float64Array(2), normalLambda: 0, normalContact: null };
    out.point = v3();
    out.normal = state.normal;
    out.axes = [state.u, state.v];
    out.centers = state.sides.map(value => value.center);
    out.levers = state.sides.map(value => value.lever);
    out.localLevers = state.sides.map(value => value.localLever);
    out.relativeSurfaceDisplacement = v3();
    out.averageRelativeSurfaceVelocity = v3();
    out.diagnostics = {};
    return state;
}

function readStencil(body, record, contact, sideIndex, out) {
    const prefix = sideIndex === 0 ? '_inner' : '_outer';
    const segment = record[prefix + 'SegmentIndex'] ?? contact[sideIndex === 0 ? 'innerSegmentIndex' : 'outerSegmentIndex'];
    if (!Number.isInteger(segment) || segment < 0 || segment + 1 >= body.count) throw new RangeError('Invalid contact material segment');
    const nodes = record[prefix + 'NodeIndices'];
    const weights = nodes ? record[prefix + 'NodeWeights'] : record[sideIndex === 0 ? 'innerWeights' : 'outerWeights'];
    const count = nodes ? record[prefix + 'NodeCount'] : 2;
    if (!Number.isInteger(count) || count < 1 || count > body.count || !weights || weights.length < count) throw new RangeError('Invalid contact interpolation stencil');
    if (nodes && nodes.length < count) throw new RangeError('Contact interpolation nodes are missing');
    out.segment = segment; out.count = count;
    out.nodes.length = out.weights.length = count;
    out.center.fill(0); out.previousCenter.fill(0);
    let total = 0;
    for (let i = 0; i < count; i++) {
        const node = nodes ? nodes[i] : segment + i;
        if (!Number.isInteger(node) || node < 0 || node >= body.count) throw new RangeError('Contact interpolation node is outside body');
        const weight = finite(weights[i], 'contact interpolation weight');
        total += weight; out.nodes[i] = node; out.weights[i] = weight;
        for (let axis = 0; axis < 3; axis++) {
            const key = XYZ[axis];
            out.center[axis] += finite(body[key]?.[node], `${key}[${node}]`) * weight;
            out.previousCenter[axis] += finite(body['previous' + key.toUpperCase()]?.[node], `previous ${key}[${node}]`) * weight;
        }
    }
    // Negative cubic weights are allowed. Renormalizing malformed stencils
    // would change geometry and silently hide force/moment non-reciprocity.
    if (Math.abs(total - 1) > PARTITION_TOLERANCE) throw new RangeError(`Contact weights must sum to one (got ${total})`);
    quaternion(body, segment, false, out.q);
    quaternion(body, segment, true, out.previousQ);
}

function unsupported(out, reason) {
    out.supported = false; out.reason = reason; out.rows.length = 0;
    return out;
}

function appendGradient(row, count, side, dof, value) {
    if (value === 0) return count;
    const entry = row.gradients[count] ??= Object.seal({ side: 0, dof: 0, value: 0 });
    entry.side = side; entry.dof = dof; entry.value = value;
    return count + 1;
}

/**
 * Two additionalRows for a SINGLE surface Coulomb group at fixed normal load.
 * gradients are [{side:0|1,dof:node*6+axis,value}]. Translation is world xyz;
 * angular axes 3..5 are LOCAL right-sided q*exp(deltaTheta). Rows have no group
 * or separate box limits: the returned group must be solved as one disk/ellipse.
 * Positive lambda applies +tangent to inner and -tangent to outer.
 *
 * A common point gives each body its own r x F moment. Strain uses the current
 * lever mapped by R_previous*R_current^T, with the SAME material interpolation
 * at both times. Anchors/basis/stencils are frozen for this linearization;
 * rebuild after changing contact geometry. Average velocity is displacement/dt,
 * not an assumption that finite rotations equal omega*dt cross r exactly.
 *
 * side/material-side: infer the midpoint of circular surface witnesses.
 * Other features (fillet/portal/rim) REQUIRE record.surfaceContactPoint. That
 * explicit point's geometric correctness is the caller's responsibility.
 * record.surfaceAxialTangent optionally supplies the exact smooth tangent;
 * otherwise U is the projected outer segment tangent. No arbitrary anisotropic
 * basis is chosen when that projection degenerates: supported=false.
 *
 * Legacy twistLambda is NOT added. These two rows replace sliding twist friction
 * and consume one shared normal-load budget. Caller must retire the old twist
 * state/solver path on migration, not apply both. Inputs are never modified.
 * out and its arrays are reused; all returned views live until its next build.
 */
export function buildKirchhoffSurfaceFriction(constraint, record, dt, out = {}) {
    if (!Number.isFinite(dt) || dt <= 0) throw new RangeError('Positive finite dt is required');
    const state = scratchState(out), bodies = state.bodies;
    bodies[0] = constraint?.innerBody; bodies[1] = constraint?.outerBody;
    if (!bodies[0] || !bodies[1] || bodies[0] === bodies[1]) throw new TypeError('Two distinct bodies are required');
    const contact = record?.manifoldContact;
    out.supported = false; out.reason = null; out.rows.length = 0;
    out.group.normalContact = contact ?? null;
    if (!contact) return unsupported(out, 'no-manifold-contact');
    const normalLambda = nonNegative(contact.normalLambda, 'normalLambda');
    const muU = nonNegative(constraint.axialFriction, 'axialFriction');
    const muV = nonNegative(constraint.circumferentialFriction ?? constraint.torsionalFriction ?? constraint.axialFriction, 'circumferentialFriction');
    out.group.normalLambda = normalLambda;
    out.group.mu[0] = muU; out.group.mu[1] = muV;
    out.group.kind = muU === muV ? 'coulomb-disk' : 'coulomb-ellipse';
    out.diagnostics.replacesLegacyTwist = true;
    out.diagnostics.ignoredLegacyTwistLambda = finite(contact.twistLambda ?? 0, 'twistLambda');
    out.diagnostics.requiresLegacyTwistRetirement = out.diagnostics.ignoredLegacyTwistLambda !== 0;
    vector(record.normal ?? contact.normal, state.normal, 'contact normal');
    if (!normalize(state.normal)) throw new RangeError('Contact normal must be nonzero');
    for (let side = 0; side < 2; side++) readStencil(bodies[side], record, contact, side, state.sides[side]);
    if (record.surfaceAxialTangent) {
        vector(record.surfaceAxialTangent, state.axial, 'surfaceAxialTangent');
        out.diagnostics.axialTangentSource = 'provided';
    } else {
        const outer = bodies[1], segment = state.sides[1].segment;
        for (let i = 0; i < 3; i++) {
            const values = outer[XYZ[i]];
            state.axial[i] = values[segment + 1] - values[segment];
        }
        out.diagnostics.axialTangentSource = 'outer-segment';
    }
    if (!projectPlane(state.axial, state.normal, state.u)) return unsupported(out, 'axial-tangent-parallel-to-normal');
    cross(state.normal, state.u, state.v);
    if (record.surfaceContactPoint) {
        vector(record.surfaceContactPoint, out.point, 'surfaceContactPoint');
        out.diagnostics.pointSource = 'provided';
    } else {
        if (record.kind !== 'side' && record.kind !== 'material-side') return unsupported(out, 'feature-requires-explicit-surface-point');
        const inner = bodies[0], segment = state.sides[0].segment;
        const radii = state.radii;
        radii[0] = Math.max(nonNegative(inner.nodeRadius?.[segment] ?? inner.radius, 'wire radius'),
            nonNegative(inner.nodeRadius?.[segment + 1] ?? inner.radius, 'wire radius'));
        radii[1] = nonNegative(constraint.innerRadius, 'lumen radius');
        for (let side = 0; side < 2; side++) {
            const s = state.sides[side];
            rotate(s.q, state.basisZ, s.director);
            if (!projectPlane(state.normal, s.director, s.witnessDirection)) return unsupported(out, 'normal-parallel-to-material-director');
            for (let i = 0; i < 3; i++) s.witness[i] = s.center[i] + radii[side] * s.witnessDirection[i];
        }
        for (let i = 0; i < 3; i++) out.point[i] = (state.sides[0].witness[i] + state.sides[1].witness[i]) * 0.5;
        out.diagnostics.pointSource = 'midpoint-circular-side';
    }
    for (const s of state.sides) {
        for (let i = 0; i < 3; i++) s.lever[i] = out.point[i] - s.center[i];
        rotate(s.q, s.lever, s.localLever, true);
        rotate(s.previousQ, s.localLever, s.previousLever);
        for (let i = 0; i < 3; i++) s.displacement[i] = s.center[i] - s.previousCenter[i] + s.lever[i] - s.previousLever[i];
    }
    const physicalMotion = constraint.surfaceMotion;
    if (physicalMotion) {
        if (physicalMotion.dt !== dt || physicalMotion.bodies?.length !== 2)
            throw new RangeError('Physical surface motion requires the same physical timestep and two channels');
        for (let side = 0; side < 2; side++) {
            const s = state.sides[side], motion = physicalMotion.bodies[side];
            s.displacement.fill(0);
            for (let i = 0; i < s.count; i++) for (let axis = 0; axis < 3; axis++)
                s.displacement[axis] += dt * s.weights[i] * finite(motion['velocity' + XYZW[axis]][s.nodes[i]], 'physical surface velocity');
            const omega = [0, 1, 2].map(axis => finite(motion['angularVelocity' + XYZW[axis]][s.segment], 'physical surface angular velocity'));
            cross(omega, s.lever, state.torque);
            for (let axis = 0; axis < 3; axis++) s.displacement[axis] += dt * state.torque[axis];
        }
    }
    for (let i = 0; i < 3; i++) {
        out.relativeSurfaceDisplacement[i] = state.sides[0].displacement[i] - state.sides[1].displacement[i];
        out.averageRelativeSurfaceVelocity[i] = out.relativeSurfaceDisplacement[i] / dt;
    }
    // Existing manifold lambdas are components in its stored tangent basis.
    // Re-express that force; never rotate an anisotropic ellipse with history.
    const oldU = finite(contact.tangentLambda?.[0], 'old tangent lambda U');
    const oldV = finite(contact.tangentLambda?.[1], 'old tangent lambda V');
    for (let i = 0; i < 3; i++) state.oldForce[i] =
        finite(contact.tangentU?.[i], 'old tangent U') * oldU + finite(contact.tangentV?.[i], 'old tangent V') * oldV;
    for (let axis = 0; axis < 2; axis++) {
        const direction = out.axes[axis], row = state.rows[axis];
        row.strain = dot(direction, out.relativeSurfaceDisplacement);
        row.lambda = dot(direction, state.oldForce);
        row.alpha = 0; row.lower = -Infinity; row.upper = Infinity;
        let count = 0;
        for (let side = 0; side < 2; side++) {
            const s = state.sides[side], sign = side === 0 ? 1 : -1;
            for (let i = 0; i < s.count; i++) for (let component = 0; component < 3; component++)
                count = appendGradient(row, count, side, s.nodes[i] * 6 + component, sign * s.weights[i] * direction[component]);
            cross(s.lever, direction, state.torque);
            rotate(s.q, state.torque, state.localTorque, true);
            for (let component = 0; component < 3; component++)
                count = appendGradient(row, count, side, s.segment * 6 + 3 + component, sign * state.localTorque[component]);
        }
        row.gradients.length = count;
        out.rows[axis] = row;
    }
    out.supported = true;
    out.diagnostics.motionSource = physicalMotion ? 'physical-velocity' : 'finite-pose-history';
    return out;
}

function ellipseInputs(lambda, normalLambda, mu, out) {
    const normal = nonNegative(normalLambda, 'normalLambda');
    out.axes ??= new Float64Array(2);
    out.lambda ??= new Float64Array(2);
    for (let i = 0; i < 2; i++) {
        finite(lambda?.[i], `lambda[${i}]`);
        const coefficient = nonNegative(mu?.[i], `mu[${i}]`);
        out.axes[i] = finite(coefficient * normal, `mu[${i}]*normalLambda`);
    }
    out.normalLambda = normal;
    return out;
}

/** Exact Euclidean projection onto an anisotropic Coulomb ellipse at FIXED Fn.
 * Handles a zero coefficient as a zero-width axis, and Fn=0 as {0,0}. This is
 * not a joint normal/tangent cone projection: it cannot increase normal load.
 * lambda and mu are [U,V]. Output storage can be reused. */
export function projectKirchhoffSurfaceFriction(lambda, normalLambda, mu, out = {}) {
    ellipseInputs(lambda, normalLambda, mu, out);
    const z0 = lambda[0], z1 = lambda[1], a = out.axes[0], b = out.axes[1];
    out.iterations = 0;
    if (a === 0 || b === 0) {
        out.lambda[0] = a ? Math.max(-a, Math.min(a, z0)) : 0;
        out.lambda[1] = b ? Math.max(-b, Math.min(b, z1)) : 0;
    } else if (Math.hypot(z0 / a, z1 / b) <= 1) {
        out.lambda[0] = z0; out.lambda[1] = z1;
    } else if (a === b) {
        const scale = Math.max(Math.abs(z0), Math.abs(z1));
        const length = Math.hypot(z0 / scale, z1 / scale);
        out.lambda[0] = a * (z0 / scale) / length;
        out.lambda[1] = a * (z1 / scale) / length;
    } else {
        // Normalize first to avoid overflow in the secular equation.
        const scale = Math.max(a, b, Math.abs(z0), Math.abs(z1));
        const ax = a / scale, by = b / scale, x = z0 / scale, y = z1 / scale;
        let lo = 0, hi = Math.hypot(ax * x, by * y);
        const aa = ax * ax, bb = by * by;
        for (let i = 0; i < 80; i++) {
            const t = (lo + hi) * 0.5;
            if (Math.hypot(ax * x / (aa + t), by * y / (bb + t)) > 1) lo = t;
            else hi = t;
            out.iterations++;
        }
        out.lambda[0] = scale * x * aa / (aa + hi);
        out.lambda[1] = scale * y * bb / (bb + hi);
    }
    out.distance = Math.hypot(z0 - out.lambda[0], z1 - out.lambda[1]);
    out.projected = out.distance > 0;
    return out;
}

/** Feasibility + fixed-normal maximum-dissipation residual.
 * Stationarity is lambda = P_E(lambda - inverseMobility * displacement).
 * inverseMobility has units multiplier/mm; caller selects the physical solver
 * scaling. At fixed Fn this evaluates non-associated Coulomb, not a normal-load
 * correction. No source lambda, displacement, mu or normal state is modified.
 */
export function evaluateKirchhoffSurfaceFriction(lambda, displacement, normalLambda, mu,
    { inverseMobility = 1 } = {}, out = {}) {
    if (!Number.isFinite(inverseMobility) || inverseMobility <= 0) throw new RangeError('inverseMobility must be positive and finite');
    const d0 = finite(displacement?.[0], 'displacement U'), d1 = finite(displacement?.[1], 'displacement V');
    out._feasible ??= {};
    out._stationary ??= {};
    out._candidate ??= new Float64Array(2);
    const feasible = projectKirchhoffSurfaceFriction(lambda, normalLambda, mu, out._feasible);
    out._candidate[0] = lambda[0] - inverseMobility * d0;
    out._candidate[1] = lambda[1] - inverseMobility * d1;
    const stationary = projectKirchhoffSurfaceFriction(out._candidate, normalLambda, mu, out._stationary);
    out.normalLambda = normalLambda;
    out.feasibilityResidual = feasible.distance;
    out.stationarityResidual = Math.hypot(lambda[0] - stationary.lambda[0], lambda[1] - stationary.lambda[1]);
    out.residual = Math.max(out.feasibilityResidual, out.stationarityResidual);
    out.work = lambda[0] * d0 + lambda[1] * d1;
    out.minimumWork = -normalLambda * Math.hypot(mu[0] * d0, mu[1] * d1);
    out.dissipationGap = out.work - out.minimumWork;
    return out;
}

/** Continuous nonlinear fixed-load maximum-dissipation residual in mm.
 * Exact force feasibility is evaluated separately; no mobility normalization.
 */
export { evaluateKirchhoffContinuousFrictionKKT as evaluateKirchhoffSurfaceFrictionKKT }
    from './kirchhoffContinuousFrictionKKT.js';

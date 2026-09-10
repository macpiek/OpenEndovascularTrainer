const finite = (v, name) => { if (!Number.isFinite(v)) throw new TypeError(`${name} must be finite`); return v; };
const vector = (v, n, name) => {
    if (!v || v.length !== n) throw new TypeError(`${name} requires ${n} entries`);
    return Array.from(v, x => finite(x, name));
};
const dot = (a, b) => a.reduce((sum, x, i) => sum + x * b[i], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = a => Math.hypot(...a);

// Local first/second forward derivatives of the AFFINE physical edge frame.
// Copied arena infrastructure only; no quadratic reconstruction is used.
// Fixed arenas can be reused; no numerical differentiation enters evaluation.
function arena(dimension, second, capacity = 4096, storage = null) {
    const stride = 1 + dimension + (second ? dimension * dimension : 0), count=stride*capacity;
    if(storage!==null&&(!(storage instanceof Float64Array)||storage.length<count))throw new RangeError('Derivative arena storage is too small');
    const data=storage===null?new Float64Array(count):storage.subarray(0,count);
    let cursor = 0;
    const alloc = () => {
        const at = cursor; cursor += stride;
        if (cursor > data.length) throw new RangeError('Surface derivative arena capacity exceeded');
        data.fill(0, at, at + stride); return at;
    };
    const constant = value => { const r = alloc(); data[r] = value; return r; };
    const unary = (a, value, first, secondDerivative = 0) => {
        const r = constant(value);
        for (let i = 0; i < dimension; i++) data[r + 1 + i] = first * data[a + 1 + i];
        if (second) for (let i = 0; i < dimension; i++) for (let j = 0; j < dimension; j++) {
            const h = 1 + dimension + i * dimension + j;
            data[r + h] = first * data[a + h] + secondDerivative * data[a + 1 + i] * data[a + 1 + j];
        }
        return r;
    };
    const binary = (a, b, value, da, db, daa = 0, dab = 0, dbb = 0) => {
        const r = constant(value);
        for (let i = 0; i < dimension; i++) data[r + 1 + i] = da * data[a + 1 + i] + db * data[b + 1 + i];
        if (second) for (let i = 0; i < dimension; i++) for (let j = 0; j < dimension; j++) {
            const h = 1 + dimension + i * dimension + j, ai = data[a + 1 + i], aj = data[a + 1 + j], bi = data[b + 1 + i], bj = data[b + 1 + j];
            data[r + h] = da * data[a + h] + db * data[b + h] + daa * ai * aj + dab * (ai * bj + bi * aj) + dbb * bi * bj;
        }
        return r;
    };
    const api = { data, dimension, stride, reset() { cursor = 0; }, used: () => cursor / stride, constant,
        checkpoint:()=>cursor,
        rewind(at){if(!Number.isInteger(at)||at<0||at>cursor||at%stride)throw new RangeError('Invalid derivative arena checkpoint');cursor=at;},
        variable(value, index) { const r = constant(value); data[r + 1 + index] = 1; return r; },
        add: (a, b) => binary(a, b, data[a] + data[b], 1, 1),
        sub: (a, b) => binary(a, b, data[a] - data[b], 1, -1),
        mul: (a, b) => binary(a, b, data[a] * data[b], data[b], data[a], 0, 1),
        scale: (a, scale) => unary(a, data[a] * scale, scale),
        reciprocal(a) { const v = data[a]; if (v === 0) throw new RangeError('Zero surface derivative denominator'); return unary(a, 1 / v, -1 / (v * v), 2 / (v * v * v)); },
        sqrt(a) { const v = Math.sqrt(data[a]); if (!(v > 0)) throw new RangeError('Surface reconstruction requires nonzero tangents'); return unary(a, v, .5 / v, -.25 / (v * v * v)); },
        sin: a => unary(a, Math.sin(data[a]), Math.cos(data[a]), -Math.sin(data[a])),
        cos: a => unary(a, Math.cos(data[a]), -Math.sin(data[a]), -Math.cos(data[a])),
        atan2(y, x) {
            const xv = data[x], yv = data[y], d = xv * xv + yv * yv;
            if (!(d > 0)) throw new RangeError('Unresolved surface reference connection');
            return binary(y, x, Math.atan2(yv, xv), xv / d, -yv / d, -2 * xv * yv / (d * d), (yv * yv - xv * xv) / (d * d), 2 * xv * yv / (d * d));
        },
        finite() { for (let i = 0; i < cursor; i++) if (!Number.isFinite(data[i])) return false; return true; }
    };
    return api;
}
function constantArena(capacity = 4096) {
    const out = arena(0, false, capacity);
    // The ordinary variable writer cannot be used with dimension zero:
    // writing its derivative entry would overwrite an adjacent scalar node.
    out.variable = value => out.constant(value);
    return out;
}
function evaluationOrder(input) {
    const order = input.order ?? 'full';
    if (order !== 'full' && order !== 'value') throw new RangeError('Surface order must be full or value');
    return order;
}
function vectors(a) {
    const add = (u, v) => u.map((x, i) => a.add(x, v[i]));
    const sub = (u, v) => u.map((x, i) => a.sub(x, v[i]));
    const times = (u, scalar) => u.map(x => a.mul(x, scalar));
    const dot = (u, v) => u.reduce((sum, x, i) => a.add(sum, a.mul(x, v[i])), a.constant(0));
    const cross = (u, v) => [a.sub(a.mul(u[1], v[2]), a.mul(u[2], v[1])), a.sub(a.mul(u[2], v[0]), a.mul(u[0], v[2])), a.sub(a.mul(u[0], v[1]), a.mul(u[1], v[0]))];
    const unit = v => times(v, a.reciprocal(a.sqrt(dot(v, v))));
    const transport = (v, from, to) => {
        const denominator = a.add(a.constant(1), dot(from, to));
        if (!(a.data[denominator] > 1e-10)) throw new RangeError('Antiparallel surface tangents require another reconstruction chart');
        const axis = cross(from, to), first = cross(axis, v), second = cross(axis, first);
        return add(add(v, first), times(second, a.reciprocal(denominator)));
    };
    return { add, sub, times, dot, cross, unit, transport };
}

// Shared differentiation primitives also serve the continuous material field.
// The affine operators keep the same arena implementation and packing.
export { arena as createCompositeSurfaceDifferentialArena, vectors as compositeSurfaceDifferentialVectors };

export function createCompositeJointSurfaceMotionWorkspace(toolCount = 2) {
    if (toolCount !== 1 && toolCount !== 2) throw new RangeError('One tool against an explicit wall, or two actual tools, is required');
    const configurationDofs = 7 * toolCount;
    return { toolCount, configurationDofs, dofCount: 2 * configurationDofs,
        second: arena(configurationDofs, true, 2048), first: arena(2 * configurationDofs, false, 2048) };
}
function scaleUnit(value) {
    const length = norm(value); if (!(length > 1e-12)) throw new RangeError('A nondegenerate affine tangent is required');
    return value.map(v => v / length);
}
function unsupported(reason, details) {
    const error = new RangeError(reason); error.code = 'surface-material-transport-required'; error.requiredTransport = details; throw error;
}

function prepareAffineSurfaceInputs({ tools, dt, contact, wall }, toolCount, requireRates) {
    if (toolCount !== 1 && toolCount !== 2) throw new RangeError('One or two actual surface tools are required');
    if (!(finite(dt, 'history dt') > 0)) throw new RangeError('Positive history dt is required');
    if (!Array.isArray(tools) || tools.length !== toolCount || new Set(tools.map(t => t.id)).size !== tools.length)
        throw new TypeError('Distinct physical tools must match the workspace');
    const point = vector(contact?.point, 3, 'Common world contact point'), axes = contact?.axes?.map(v => vector(v, 3, 'World contact tangent'));
    if (axes?.length !== 2 || axes.some(v => Math.abs(norm(v) - 1) > 1e-10) || Math.abs(dot(axes[0], axes[1])) > 1e-10)
        throw new RangeError('Two orthonormal explicit world contact tangents are required');
    if (tools.length === 2 && wall !== undefined) throw new RangeError('Two physical tools must not also specify a wall');
    const wallVelocity = tools.length === 1 ? vector(wall?.velocity, 3, 'Explicit wall velocity at the common point') : [0, 0, 0];
    const inputs = tools.map(t => {
        if (typeof t.id !== 'string' || !t.id) throw new TypeError('A named physical tool is required');
        const x = vector(t.coordinates, 2, 'Own affine coordinates'), coordinate = finite(t.coordinate, 'Own contact foot');
        const L = finite(x[1] - x[0], 'Own affine coordinate length'); if (!(L > 0)) throw new RangeError('Own affine coordinates must increase');
        const f = (coordinate - x[0]) / L;
        if (f < 0 || f > 1 || f === 0 && t.trace !== 'right' || f === 1 && t.trace !== 'left')
            unsupported('Current affine endpoint needs its explicit one-sided trace', { toolId: t.id, fraction: f });
        if (t.positions?.length !== 2 || t.previousPositions?.length !== 2 || requireRates && t.positionRates?.length !== 2)
            throw new RangeError('Own current/previous physical endpoints and instantaneous endpoint rates are required');
        const positions = t.positions.map(v => vector(v, 3, 'Own current position')), previous = t.previousPositions.map(v => vector(v, 3, 'Own previous position'));
        const rates = requireRates ? t.positionRates.map(v => vector(v, 3, 'Own instantaneous position rate')) : null;
        const angle = finite(t.angle, 'Own unwrapped angle'), previousAngle = finite(t.previousAngle, 'Own previous unwrapped angle'), angleRate = requireRates ? finite(t.angleRate, 'Own instantaneous angle rate') : null;
        const ref = { tangent: vector(t.reference?.tangent, 3, 'Own accepted tangent'), director: vector(t.reference?.director, 3, 'Own accepted director') };
        const oldChord = previous[1].map((v, k) => v - previous[0][k]), oldLength = norm(oldChord);
        if (!(oldLength > 0) || !Number.isFinite(oldLength) || Math.abs(dot(ref.tangent, ref.tangent) - 1) > 1e-10 || Math.abs(dot(ref.director, ref.director) - 1) > 1e-10 ||
            Math.abs(dot(ref.tangent, ref.director)) > 1e-10 || norm(ref.tangent.map((v, k) => v - oldChord[k] / oldLength)) > 1e-10)
            throw new RangeError('Each accepted frame must belong to its own previous physical edge');
        const map = t.materialMap, oldMap = t.materialPath?.previousMap;
        const sx = finite(map?.dsDx, 'Own current dsDx'), s0 = finite(map?.sStart, 'Own current start label'), st = finite(map?.dsDt, 'Own current label rate at the query');
        const oldSx = finite(oldMap?.dsDx, 'Own previous dsDx'), oldS0 = finite(oldMap?.sStart, 'Own previous start label');
        if (!(sx > 0 && oldSx > 0)) throw new RangeError('Own material label slopes must be positive');
        if (t.materialPath?.kind !== 'linear-affine-maps' || typeof t.edgeId !== 'string' || !t.edgeId || t.materialPath.previousEdgeId !== t.edgeId)
            unsupported('An explicit same-edge linear affine material-map history is required', { toolId: t.id, currentEdgeId: t.edgeId, previousEdgeId: t.materialPath?.previousEdgeId });
        const materialLabel = finite(s0 + sx * L * f, 'Own current material label'), previousFraction = finite((materialLabel - oldS0) / finite(oldSx * L, 'Own previous material span'), 'Own previous material fraction');
        if (previousFraction < 0 || previousFraction > 1 || previousFraction === 0 && t.materialPath.previousTrace !== 'right' || previousFraction === 1 && t.materialPath.previousTrace !== 'left')
            unsupported('Material crossed a physical DER hinge; its own finite orientation/surface-point path is required', { toolId: t.id, materialLabel, previousFraction, currentFraction: f });
        const impliedRate = finite(((s0 - oldS0) + (sx - oldSx) * L * f) / dt, 'Declared material path rate');
        const roundoff = finite(64 * Number.EPSILON * (Math.abs(s0) + Math.abs(oldS0) + (Math.abs(sx) + Math.abs(oldSx)) * L * Math.abs(f)) / dt, 'Material path arithmetic bound');
        if (Math.abs(st - impliedRate) > roundoff) throw new RangeError('Instantaneous label rate must match the declared linear affine-map path');
        return { id: t.id, positions, previous, rates, angle, previousAngle, angleRate, ref, f, L, u: -st / sx, materialLabel, previousFraction, edgeId: t.edgeId, trace: t.trace ?? null };
    });
    return { inputs, point, axes, wallVelocity };
}

/** INSTANTANEOUS velocity and virtual-power map on each tool's OWN actual
 * affine physical edge. Local configurations/rates are [q0.xyz,q1.xyz,theta]
 * per tool. Rates are explicit instantaneous quantities, never inferred from
 * a BE chord or a principal angle. Independent theta values remain unwrapped.
 *
 * Each current material label is traced through its declared linear-in-time
 * affine maps back into this same edge. Crossing a real DER hinge requires an
 * explicit per-material orientation/surface-point transport path and rejects.
 * Inside the open affine edge the DER orientation field is spatially constant;
 * it has no invented curvature/spin interpolation or artificial midpoint seam.
 * One-sided edge-end traces must be explicit. This is NOT finite dt slip.
 *
 * contact.point and contact.axes are frozen world query data. Both lever arms
 * are derived from that same point and their distinct physical affine feet.
 * The frame's full time derivative follows its accepted time-PT chart and own
 * unwrapped theta; omega=1/2 sum d_j cross dot(d_j). Translation is qt+u*qx.
 * The velocity Jacobian separates configuration and rate columns. The transpose
 * of the RATE Jacobian is the exact instantaneous virtual-power force map;
 * it must not be replaced with a BE slip derivative. Prescribed feed/wall
 * velocity supplies a separate explicit power contribution.
 */
export function evaluateCompositeJointSurfaceMotion({ tools, dt, rateMode, contact, wall }, workspace = createCompositeJointSurfaceMotionWorkspace(tools?.length)) {
    if (rateMode !== 'instantaneous') throw new TypeError('Explicit instantaneous rates are required; this operator does not infer a finite rotation path');
    const { inputs, point, axes, wallVelocity } = prepareAffineSurfaceInputs({ tools, dt, contact, wall }, workspace.toolCount, true);
    const a = workspace.second, b = workspace.first, av = vectors(a), bv = vectors(b), n = workspace.configurationDofs, total = workspace.dofCount;
    a.reset(); b.reset();
    const lift = offset => {
        const out = b.constant(a.data[offset]);
        for (let j = 0; j < n; j++) b.data[out + 1 + j] = a.data[offset + 1 + j];
        return out;
    };
    const partial = (offset, index) => {
        const out = b.constant(a.data[offset + 1 + index]);
        for (let j = 0; j < n; j++) b.data[out + 1 + j] = a.data[offset + 1 + n + index * n + j];
        return out;
    };
    const responses = inputs.map((input, index) => {
        const start = 7 * index, q = input.positions.map((p, end) => p.map((v, k) => a.variable(v, start + 3 * end + k)));
        const theta = a.variable(input.angle, start + 6), rates = input.rates.flat().concat(input.angleRate).map((v, j) => b.variable(v, n + start + j));
        const tangent = av.unit(av.sub(q[1], q[0])), reference = av.transport(input.ref.director.map(a.constant), input.ref.tangent.map(a.constant), tangent);
        const perpendicular = av.cross(tangent, reference), d1 = av.add(av.times(reference, a.cos(theta)), av.times(perpendicular, a.sin(theta))), d2 = av.cross(tangent, d1);
        const triad = [d1, d2, tangent], lifted = triad.map(d => d.map(lift));
        const timeDerivative = offset => rates.reduce((sum, rate, j) => b.add(sum, b.mul(partial(offset, start + j), rate)), b.constant(0));
        let omega = [0, 0, 0].map(b.constant);
        for (let j = 0; j < 3; j++) omega = bv.add(omega, bv.cross(lifted[j], triad[j].map(timeDerivative)).map(v => b.scale(v, .5)));
        const center = bv.add(q[0].map(v => b.scale(lift(v), 1 - input.f)), q[1].map(v => b.scale(lift(v), input.f)));
        const qx = av.sub(q[1], q[0]).map(v => b.scale(lift(v), 1 / input.L));
        const qt = [0, 1, 2].map(k => b.add(b.scale(rates[k], 1 - input.f), b.scale(rates[3 + k], input.f)));
        const feed = qx.map(v => b.scale(v, input.u)), velocity = bv.add(qt, feed), lever = bv.sub(point.map(b.constant), center);
        return { input, center, qx, qt, velocity, feed, omega, lever, triad: lifted, surface: bv.add(velocity, bv.cross(omega, lever)) };
    });
    const relative = responses.length === 2 ? bv.sub(responses[0].surface, responses[1].surface) : bv.sub(responses[0].surface, wallVelocity.map(b.constant));
    const prescribed = responses.length === 2 ? bv.sub(responses[0].feed, responses[1].feed) : bv.sub(responses[0].feed, wallVelocity.map(b.constant));
    const slipRate = axes.map(axis => bv.dot(axis.map(b.constant), relative));
    const prescribedSlipRate = axes.map(axis => bv.dot(axis.map(b.constant), prescribed));
    if (!a.finite() || !b.finite()) throw new RangeError('Nonfinite affine surface velocity/Jacobian');
    const values = v => Float64Array.from(v, offset => b.data[offset]);
    const jacobian = (v, start, length) => Float64Array.from(v.flatMap(offset => Array.from(b.data.subarray(offset + 1 + start, offset + 1 + start + length))));
    const rateJacobian = jacobian(slipRate, n, n);
    return { scope: 'instantaneous-own-affine-edge-surface-motion', certified: false, finiteStepSlipKnown: false, includesHingeTransport: false,
        dofCount: total, configurationDofs: n, velocity: values(relative), slipRate: values(slipRate), prescribedVelocity: values(prescribed), prescribedSlipRate: values(prescribedSlipRate),
        configurationJacobian: jacobian(slipRate, 0, n), rateJacobian, jacobian: jacobian(slipRate, 0, total), velocityJacobian: jacobian(relative, 0, total),
        forceMap: Float64Array.from({ length: n * 2 }, (_, i) => rateJacobian[(i % 2) * n + Math.floor(i / 2)]),
        tools: responses.map(r => ({ id: r.input.id, edgeId: r.input.edgeId, trace: r.input.trace, fraction: r.input.f, previousFraction: r.input.previousFraction,
            materialLabel: r.input.materialLabel, u: r.input.u, angle: r.input.angle, previousAngle: r.input.previousAngle, unwrappedAngleDifference: r.input.angle - r.input.previousAngle,
            center: values(r.center), positionDx: values(r.qx), positionDt: values(r.qt), velocity: values(r.velocity), omega: values(r.omega), lever: values(r.lever),
            directors: r.triad.map(values), surfaceVelocity: values(r.surface), omegaJacobian: jacobian(r.omega, 0, total), spatialOrientationField: 'constant-on-own-open-DER-edge' })),
        derivatives: { finiteDifferences: false, configurationAndRateColumnsSeparate: true, forceMap: 'transpose-of-instantaneous-rate-jacobian' } };
}

/** FINITE CONTROL, not a general nonlinear friction row. Parallel actual
 * affine axes use an objective geometric contact frame (n,t cross n,t), with
 * the shortest regular tangent/normal transport branch. Separate unwrapped
 * spin increments are retained; relative winding alone is insufficient.
 * Current material labels select both old center histories. No time trajectory
 * beyond this explicit discrete contact-frame rule is claimed. General
 * nonparallel axes, hinge crossings and a long/unknown contact-frame orbit
 * need another explicit transport chart. There is intentionally no claimed
 * full finite configuration Jacobian outside this restricted parallel chart.
 */
export function evaluateCompositeJointParallelSurfaceIncrement(input) {
    if (input.rotationPath !== 'short-contact-frame-own-unwrapped-spins') throw new TypeError('Explicit short contact-frame and own unwrapped spin path is required');
    const { inputs } = prepareAffineSurfaceInputs(input, input.tools?.length, false);
    const tools = inputs.map(t => ({ id: t.id, center: t.positions[0].map((v, k) => (1 - t.f) * v + t.f * t.positions[1][k]),
        previousFraction: t.previousFraction, materialLabel: t.materialLabel, unwrappedAngleDifference: t.angle - t.previousAngle,
        directors: [null, null, scaleUnit(t.positions[1].map((v, k) => v - t.positions[0][k]))] }));
    const subtract = (a, b) => a.map((v, k) => v - b[k]), unit = v => {
        const length = norm(v); if (!(length > 1e-12)) throw new RangeError('Regular finite contact-frame direction is required'); return v.map(x => x / length);
    };
    const oldCenter = (t, i) => input.tools[i].previousPositions[0].map((v, k) =>
        (1 - t.previousFraction) * v + t.previousFraction * input.tools[i].previousPositions[1][k]);
    const currentCenters = tools.map(t => Array.from(t.center)), previousCenters = tools.map(oldCenter);
    const radius = finite(input.radii?.inner, 'Inner surface radius'); if (!(radius > 0)) throw new RangeError('A positive inner surface radius is required');
    const tangent = Array.from(tools.at(-1).directors[2]), oldTangent = vector(input.tools.at(-1).reference.tangent, 3, 'Previous outer tangent');
    if (tools.some(t => norm(subtract(Array.from(t.directors[2]), tangent)) > 1e-10) ||
        input.tools.some(t => norm(subtract(t.reference.tangent, oldTangent)) > 1e-10)) throw new RangeError('Finite control requires parallel physical axes in both states');
    let normal, oldNormal, outerRadius, relative, previousRelative;
    if (tools.length === 2) {
        outerRadius = finite(input.radii?.outer, 'Outer lumen radius'); if (!(outerRadius > radius)) throw new RangeError('Regular inner/lumen radii are required');
        relative = subtract(currentCenters[0], currentCenters[1]); previousRelative = subtract(previousCenters[0], previousCenters[1]);
        normal = unit(relative.map((v, k) => v - dot(relative, tangent) * tangent[k]));
        oldNormal = unit(previousRelative.map((v, k) => v - dot(previousRelative, oldTangent) * oldTangent[k]));
    } else {
        if (input.finiteWall?.mode !== 'stationary-plane' || input.wall.velocity.some(v => v !== 0)) throw new RangeError('Finite one-tool control requires an explicit stationary plane');
        normal = unit(vector(input.finiteWall.normal, 3, 'Stationary plane normal')); oldNormal = normal.slice();
        const wallTangent = unit(vector(input.finiteWall.tangent, 3, 'Stationary plane tangent'));
        if (norm(subtract(tangent, wallTangent)) > 1e-10 || norm(subtract(oldTangent, wallTangent)) > 1e-10 || Math.abs(dot(normal, tangent)) > 1e-10)
            throw new RangeError('Finite plane control requires its fixed parallel physical axis');
        const point = vector(input.finiteWall.point, 3, 'Stationary plane point');
        outerRadius = 0; relative = subtract(currentCenters[0], point); previousRelative = subtract(previousCenters[0], point);
    }
    const denominator = 1 + dot(oldTangent, tangent);
    if (!(denominator > 1e-10)) throw new RangeError('Finite contact-frame tangent transport needs another short chart');
    const rotationAxis = cross(oldTangent, tangent), first = cross(rotationAxis, oldNormal), second = cross(rotationAxis, first);
    const carriedNormal = oldNormal.map((v, k) => v + first[k] + second[k] / denominator);
    const beta = Math.atan2(dot(tangent, cross(carriedNormal, normal)), dot(carriedNormal, normal));
    if (!(Math.PI - Math.abs(beta) > 1e-10)) throw new RangeError('Finite contact-frame normal turn is ambiguous on its short branch');
    const circumferential = cross(tangent, normal), oldCircumferential = cross(oldTangent, oldNormal);
    const centerIncrement = [dot(relative, tangent) - dot(previousRelative, oldTangent), dot(relative, circumferential) - dot(previousRelative, oldCircumferential)];
    const spinIncrements = tools.map(t => t.unwrappedAngleDifference - beta);
    const increment = [centerIncrement[0], centerIncrement[1] + radius * spinIncrements[0] - outerRadius * (spinIncrements[1] ?? 0)];
    if (![...increment, ...spinIncrements, beta].every(Number.isFinite)) throw new RangeError('Nonfinite parallel contact-frame increment');
    return { scope: 'finite-parallel-affine-contact-frame-control', increment: Float64Array.from(increment),
        axes: [Float64Array.from(tangent), Float64Array.from(circumferential)], normal: Float64Array.from(normal), previousNormal: Float64Array.from(oldNormal),
        centerIncrement: Float64Array.from(centerIncrement), ownRelativeSpinIncrements: Float64Array.from(spinIncrements), contactFrameSpinIncrement: beta,
        currentCenters, previousCenters, materialLabels: tools.map(t => t.materialLabel), radii: [radius, outerRadius],
        certified: false, nonlinearReady: false, finiteJacobian: null, includesGeneralHingeTransport: false,
        path: 'short-contact-frame-own-unwrapped-spins', instantaneousRatesRequired: false };
}

export function createCompositeJointSurfaceIncrementWorkspace(toolCount = 2) {
    if (toolCount !== 1 && toolCount !== 2) throw new RangeError('One or two actual surface tools are required');
    const configurationDofs = 7 * toolCount, queryDofs = toolCount + 18;
    return { toolCount, configurationDofs, queryDofs, dofCount: configurationDofs + queryDofs,
        first: arena(configurationDofs + queryDofs, false, 4096), value: constantArena(4096) };
}

/** A finite, objective discrete surface increment on an OPEN own-edge chart.
 * This is not the derivative of the instantaneous virtual-power functional.
 * The separate instantaneous rate-J transpose remains the physical force map.
 *
 * Each material label selects its own old affine center. In geometric contact
 * coordinates C=(n,t cross n,t), transport each old tangent by its short swing
 * b, and apply its OWN unwrapped relative spin alpha during that swing:
 * t(tau)=exp(tau*[b]) t_old, omega(tau)=b+alpha*t(tau), 0<=tau<=1.
 * alpha=theta_new-theta_old+the short reference-director connection in C.
 * The discrete increment is dc + integral(omega) cross mean(old,new lever).
 * This symmetric lever quadrature defines the rule; it is not an assertion
 * that an unknown finite contact trajectory can be recovered from endpoints.
 * Both theta lifts are retained, even when relative winding is zero.
 *
 * finiteGeometry supplies two objective queries of the SAME contact chart:
 * {kind:'explicit-affine-side-queries',current:{point,normal,tangent},
 * previous:{point,normal,tangent}}. Raw tangent and normal are normalized and
 * orthogonalized INSIDE AD. Caller owns their geometry/provenance and chain
 * rule. No foot, point, normal or contact-frame derivative is silently frozen:
 * columns are [7T configurations, T current material feet (coordinate units),
 * current point/normal/tangent, previous point/normal/tangent]. Maps and own
 * accepted frames are fixed preparation inputs. Previous feet are obtained by
 * tracing CURRENT labels, and their derivatives are included.
 * A wall currently requires the SAME stationary wall point/frame in both
 * queries; moving-wall material history is not inferred from a query velocity.
 * order:'value' evaluates the SAME scalar algebra with a dimension-zero
 * arena. Derivative arrays are null/invalid, not read from scalar storage.
 */
export function evaluateCompositeJointSurfaceIncrement(input, workspace = createCompositeJointSurfaceIncrementWorkspace(input.tools?.length)) {
    const order = evaluationOrder(input), full = order === 'full';
    if (input.rotationPath !== 'short-contact-frame-own-unwrapped-spins') throw new TypeError('Explicit short contact-frame and own unwrapped spin path is required');
    const geometry = input.finiteGeometry;
    if (geometry?.kind !== 'explicit-affine-side-queries') throw new TypeError('Explicit current and previous objective affine-side queries are required');
    const query = [geometry.current, geometry.previous].map(q => ({ point: vector(q?.point, 3, 'Finite contact point'),
        normal: vector(q?.normal, 3, 'Finite contact normal'), tangent: vector(q?.tangent, 3, 'Finite contact tangent') }));
    const numericFrame = q => {
        const t = scaleUnit(q.tangent), n = scaleUnit(q.normal.map((v, k) => v - dot(q.normal, t) * t[k]));
        return [n, cross(t, n), t];
    };
    const frames = query.map(numericFrame);
    const { inputs } = prepareAffineSurfaceInputs({ ...input, contact: { point: query[0].point, axes: [frames[0][2], frames[0][1]] } }, workspace.toolCount, false);
    if (workspace.toolCount === 1 && (input.wall.velocity.some(v => v !== 0) ||
        ['point', 'normal', 'tangent'].some(key => query[0][key].some((v, k) => v !== query[1][key][k]))))
        throw new RangeError('Finite wall motion requires an explicit stationary identical material point/frame');
    const a = full ? workspace.first : workspace.value, v = vectors(a), n = workspace.configurationDofs, count = workspace.toolCount;
    a.reset();
    const one = a.constant(1), constants = values => values.map(a.constant);
    const frame = (q, at) => {
        const variable = (values, start) => values.map((x, k) => a.variable(x, start + k));
        const point = variable(q.point, at), rawN = variable(q.normal, at + 3), t = v.unit(variable(q.tangent, at + 6));
        const normal = v.unit(v.sub(rawN, v.times(t, v.dot(rawN, t))));
        return { point, columns: [normal, v.cross(t, normal), t] };
    };
    const current = frame(query[0], n + count), previous = frame(query[1], n + count + 9);
    const toFrame = (vector, frame) => frame.columns.map(axis => v.dot(axis, vector));
    const signedPhase = (from, to, t) => {
        const phase = a.atan2(v.dot(t, v.cross(from, to)), v.dot(from, to));
        if (!(Math.PI - Math.abs(a.data[phase]) > 1e-10)) throw new RangeError('Finite reference/contact normal connection needs another short chart');
        return phase;
    };
    // A short geometric contact branch is explicit even though coordinates in
    // C cancel its common motion. Endpoints cannot encode a long world orbit.
    const carriedNormal = v.transport(previous.columns[0], previous.columns[2], current.columns[2]);
    const contactPhase = signedPhase(carriedNormal, current.columns[0], current.columns[2]);
    const interpolate = (ends, fraction) => v.add(ends[0], v.times(v.sub(ends[1], ends[0]), fraction));
    const responses = inputs.map((tool, index) => {
        const source = input.tools[index], start = index * 7;
        const q = tool.positions.map((p, end) => p.map((x, k) => a.variable(x, start + 3 * end + k))), theta = a.variable(tool.angle, start + 6);
        const coordinate = a.variable(source.coordinate, n + index), fraction = a.scale(a.sub(coordinate, a.constant(source.coordinates[0])), 1 / tool.L);
        const label = a.add(a.constant(source.materialMap.sStart), a.scale(fraction, source.materialMap.dsDx * tool.L));
        const oldFraction = a.scale(a.sub(label, a.constant(source.materialPath.previousMap.sStart)), 1 / (source.materialPath.previousMap.dsDx * tool.L));
        const centerWorld = interpolate(q, fraction), oldCenterWorld = interpolate(tool.previous.map(constants), oldFraction);
        const center = toFrame(v.sub(centerWorld, current.point), current), oldCenter = toFrame(v.sub(oldCenterWorld, previous.point), previous);
        const worldT = v.unit(v.sub(q[1], q[0])), oldWorldT = constants(tool.ref.tangent);
        const worldReference = v.transport(constants(tool.ref.director), oldWorldT, worldT);
        const t = toFrame(worldT, current), oldT = toFrame(oldWorldT, previous);
        const reference = toFrame(worldReference, current), oldReference = toFrame(constants(tool.ref.director), previous);
        const phase = signedPhase(v.transport(oldReference, oldT, t), reference, t);
        const alpha = a.add(a.sub(theta, a.constant(tool.previousAngle)), phase);
        const c = v.dot(oldT, t), axis = v.cross(oldT, t), denominator = a.add(one, c);
        if (!(a.data[denominator] > 1e-10)) throw new RangeError('Finite own tangent swing needs another short chart');
        // Stable evaluation of acos(c)/sqrt(1-c*c). At c=1 the expression
        // and its derivative have removable singularities. The degree-6
        // analytic series in 1-c has truncation <2e-23 for |1-c|<1e-3.
        // Its AD differentiates this roundoff-accurate evaluation, never FD.
        let sincInverse;
        const delta = a.sub(one, c);
        if (Math.abs(a.data[delta]) < 1e-3) {
            const coefficients = [1]; for (let k = 1; k <= 6; k++) coefficients.push(coefficients[k - 1] * k / (2 * k + 1));
            sincInverse = a.constant(coefficients[6]);
            for (let k = 5; k >= 0; k--) sincInverse = a.add(a.constant(coefficients[k]), a.mul(delta, sincInverse));
        } else {
            const sine = a.sqrt(v.dot(axis, axis));
            sincInverse = a.mul(a.atan2(sine, c), a.reciprocal(sine));
        }
        const swing = v.times(axis, sincInverse), meanTangent = v.times(v.add(oldT, t), a.reciprocal(a.mul(sincInverse, denominator)));
        const angularIncrement = v.add(swing, v.times(meanTangent, alpha));
        const meanLever = v.times(v.add(center, oldCenter), a.constant(-.5));
        const centerIncrement = v.sub(center, oldCenter), increment = v.add(centerIncrement, v.cross(angularIncrement, meanLever));
        return { tool, centerWorld, oldCenterWorld, center, oldCenter, phase, alpha, swing, meanTangent, angularIncrement, meanLever, centerIncrement, increment };
    });
    const relative = count === 2 ? v.sub(responses[0].increment, responses[1].increment) : responses[0].increment;
    const increment = [relative[2], relative[1]];
    if (!a.finite()) throw new RangeError('Nonfinite finite affine surface increment/Jacobian');
    const values = vector => Float64Array.from(vector, at => a.data[at]);
    const jacobian = (vector, start, length) => full ? Float64Array.from(vector.flatMap(at => Array.from(a.data.subarray(at + 1 + start, at + 1 + start + length)))) : null;
    return { scope: 'finite-own-affine-contact-frame-increment', increment: values(increment), relativeIncrement: values(relative),
        configurationDofs: n, queryDofs: workspace.queryDofs, dofCount: workspace.dofCount,
        configurationJacobian: jacobian(increment, 0, n), queryJacobian: jacobian(increment, n, workspace.queryDofs),
        jacobian: jacobian(increment, 0, workspace.dofCount), relativeIncrementJacobian: jacobian(relative, 0, workspace.dofCount),
        axes: [values(current.columns[2]), values(current.columns[1])], normal: values(current.columns[0]), contactFrameSpinIncrement: a.data[contactPhase],
        tools: responses.map(r => ({ id: r.tool.id, edgeId: r.tool.edgeId, materialLabel: r.tool.materialLabel, previousFraction: r.tool.previousFraction,
            center: values(r.centerWorld), previousCenter: values(r.oldCenterWorld), centerInContactFrame: values(r.center), previousCenterInContactFrame: values(r.oldCenter),
            ownReferencePhase: a.data[r.phase], ownRelativeSpinIncrement: a.data[r.alpha], tangentSwing: values(r.swing), meanTangent: values(r.meanTangent),
            angularIncrement: values(r.angularIncrement), meanLever: values(r.meanLever), centerIncrement: values(r.centerIncrement), increment: values(r.increment) })),
        certified: false, nonlinearReady: full, includesGeneralHingeTransport: false, instantaneousRatesRequired: false,
        order, incrementValid: true, configurationJacobianValid: full, queryJacobianValid: full, jacobianValid: full,
        path: 'short-contact-frame-own-unwrapped-spins', discreteLeverRule: 'symmetric-endpoint-average',
        derivatives: { finiteDifferences: false, callerOwnsQueryChainRule: true, forceMap: 'use-separate-instantaneous-rate-jacobian',
            columns: 'per-tool(q0.xyz,q1.xyz,theta);per-tool(coordinate);current(point,normal,tangent);previous(point,normal,tangent)' } };
}

export function createCompositeJointSurfaceForceMapWorkspace(toolCount = 2) {
    if (toolCount !== 1 && toolCount !== 2) throw new RangeError('One or two actual surface tools are required');
    const configurationDofs = 7 * toolCount, queryDofs = toolCount + 9;
    return { toolCount, configurationDofs, queryDofs, dofCount: configurationDofs + queryDofs,
        first: arena(configurationDofs + queryDofs, false, 4096), valueOutput: constantArena(4096) };
}

/** Physical instantaneous force map B and its configuration/query derivative.
 * No endpoint/angular/wall rates are inputs: B maps a tangential traction to
 * the work-conjugate generalized forces for rates at fixed own coordinates.
 * Prescribed feed and wall velocity supply a SEPARATE power contribution;
 * this operator neither guesses them nor returns a total surface velocity.
 * No material-map, dt or previous angular rate is needed to determine B.
 *
 * forceGeometry={kind:'explicit-affine-side-query',point,normal,tangent} is
 * the caller-owned current objective query. Its normal/tangent are normalized
 * and orthogonalized inside AD, just as in the finite increment operator.
 * Derivative columns: [7T current configurations,T own current coordinates,
 * point.xyz,normal.xyz,tangent.xyz]. Entries are ordered [dof,component,column]
 * with component 0 axial, 1 circumferential. The caller chains ALL moving
 * foot/point/normal/tangent arguments to its detector geometry.
 *
 * For old/current unit tangents a,t, differentiating the shortest time-PT
 * rotation R=I+[a cross t]+[a cross t]^2/(1+a dot t) gives
 * omega_j = t cross dt_j - t ((a cross t) dot dt_j)/(1+a dot t).
 * Here dt_j=(I-t t^T) deltaChord_j/L; the own-theta column is t.
 * The minus sign retains the axial time-PT spin connection. The circular
 * affine surface removes the reference-director gauge and current angle
 * from B, but their original own-frame/finite-angle validation still applies.
 * First-order AD of this explicit omega gives exact DB, including the moving
 * query feet, point and frame. No triad Hessian or repeated rate evaluation
 * is needed. order:'value' runs the same algebra with scalar-only storage
 * and returns null derivatives with false derivative-validity flags.
 */
export function evaluateCompositeJointSurfaceForceMap(input, workspace = createCompositeJointSurfaceForceMapWorkspace(input.tools?.length)) {
    const order = evaluationOrder(input), full = order === 'full';
    const count = workspace.toolCount, tools = input.tools, geometry = input.forceGeometry;
    if (!Array.isArray(tools) || tools.length !== count || new Set(tools.map(t => t.id)).size !== count)
        throw new TypeError('Distinct physical tools must match the force-map workspace');
    if (geometry?.kind !== 'explicit-affine-side-query') throw new TypeError('An explicit current objective affine-side force query is required');
    const point = vector(geometry.point, 3, 'Force query point'), rawNormal = vector(geometry.normal, 3, 'Force query normal'), rawTangent = vector(geometry.tangent, 3, 'Force query tangent');
    const numericTangent = scaleUnit(rawTangent);
    scaleUnit(rawNormal.map((x, k) => x - dot(rawNormal, numericTangent) * numericTangent[k]));
    const prepared = tools.map(t => {
        if (typeof t.id !== 'string' || !t.id || typeof t.edgeId !== 'string' || !t.edgeId) throw new TypeError('A named physical tool and own edge are required');
        const coordinates = vector(t.coordinates, 2, 'Own force coordinates'), coordinate = finite(t.coordinate, 'Own force contact foot'), length = finite(coordinates[1] - coordinates[0], 'Own force coordinate length');
        if (!(length > 0)) throw new RangeError('Own force coordinates must increase');
        const fraction = (coordinate - coordinates[0]) / length;
        if (fraction < 0 || fraction > 1 || fraction === 0 && t.trace !== 'right' || fraction === 1 && t.trace !== 'left')
            unsupported('Force-map affine endpoint needs its explicit one-sided trace', { toolId: t.id, fraction });
        if (t.positions?.length !== 2 || t.previousPositions?.length !== 2) throw new RangeError('Own current and accepted previous physical endpoints are required');
        const positions = t.positions.map(p => vector(p, 3, 'Own current force position')), previous = t.previousPositions.map(p => vector(p, 3, 'Own accepted force position'));
        const reference = { tangent: vector(t.reference?.tangent, 3, 'Own accepted force tangent'), director: vector(t.reference?.director, 3, 'Own accepted force director') };
        const chord = previous[1].map((x, k) => x - previous[0][k]), oldLength = finite(norm(chord), 'Own accepted force chord length');
        if (!(oldLength > 1e-12)) throw new RangeError('A nondegenerate accepted force tangent is required');
        const oldTangent = chord.map(x => x / oldLength);
        if (Math.abs(dot(reference.tangent, reference.tangent) - 1) > 1e-10 || Math.abs(dot(reference.director, reference.director) - 1) > 1e-10 ||
            Math.abs(dot(reference.tangent, reference.director)) > 1e-10 || norm(reference.tangent.map((x, k) => x - oldTangent[k])) > 1e-10)
            throw new RangeError('Each force-map accepted frame must belong to its own previous physical edge');
        return { id: t.id, edgeId: t.edgeId, coordinates, coordinate, length, fraction, positions, reference, angle: finite(t.angle, 'Own current unwrapped force angle') };
    });
    const b = full ? workspace.first : workspace.valueOutput,
        bv = vectors(b), n = workspace.configurationDofs, m = workspace.dofCount;
    b.reset();
    const queryStart = n + count, contactPoint = point.map((x, k) => b.variable(x, queryStart + k)),
        normalInput = rawNormal.map((x, k) => b.variable(x, queryStart + 3 + k)),
        tangent = bv.unit(rawTangent.map((x, k) => b.variable(x, queryStart + 6 + k))),
        normal = bv.unit(bv.sub(normalInput, bv.times(tangent, bv.dot(normalInput, tangent)))), axes = [tangent, bv.cross(tangent, normal)];
    const entries = [], responses = prepared.map((tool, index) => {
        const start = 7 * index, q = tool.positions.map((p, end) => p.map((x, k) => b.variable(x, start + 3 * end + k)));
        const chord = bv.sub(q[1], q[0]), inverseLength = b.reciprocal(b.sqrt(bv.dot(chord, chord))), toolTangent = bv.times(chord, inverseLength);
        const oldTangent = tool.reference.tangent.map(b.constant), connectionAxis = bv.cross(oldTangent, toolTangent),
            denominator = b.add(b.constant(1), bv.dot(oldTangent, toolTangent));
        if (!(b.data[denominator] > 1e-10)) throw new RangeError('Antiparallel surface tangents require another reconstruction chart');
        const inverseDenominator = b.reciprocal(denominator);
        const endpointOmega = [0, 1, 2].map(k => {
            const basis = [0, 1, 2].map(j => b.constant(j === k ? 1 : 0)),
                deltaTangent = bv.times(bv.sub(basis, bv.times(toolTangent, toolTangent[k])), inverseLength),
                axialConnection = b.mul(bv.dot(connectionAxis, deltaTangent), inverseDenominator);
            return bv.sub(bv.cross(toolTangent, deltaTangent), bv.times(toolTangent, axialConnection));
        });
        const omegaColumns = [...endpointOmega.map(column => column.map(at => b.scale(at, -1))), ...endpointOmega, toolTangent];
        const fraction = b.scale(b.sub(b.variable(tool.coordinate, n + index), b.constant(tool.coordinates[0])), 1 / tool.length);
        const center = bv.add(q[0], bv.times(chord, fraction)), lever = bv.sub(contactPoint, center);
        for (let local = 0; local < 7; local++) {
            const omega = omegaColumns[local];
            const centerRate = [0, 0, 0].map(b.constant);
            if (local < 6) centerRate[local % 3] = local < 3 ? b.sub(b.constant(1), fraction) : fraction;
            const field = bv.add(centerRate, bv.cross(omega, lever)), sign = index === 0 ? 1 : -1;
            for (const axis of axes) entries.push(b.scale(bv.dot(axis, field), sign));
        }
        return { tool, center, lever, tangent: toolTangent, omegaColumns };
    });
    if (!b.finite()) throw new RangeError('Nonfinite physical surface force map/derivative');
    const values = vector => Float64Array.from(vector, at => b.data[at]);
    const derivative = (start, length) => full ? Float64Array.from(entries.flatMap(at => Array.from(b.data.subarray(at + 1 + start, at + 1 + start + length)))) : null;
    return { scope: 'instantaneous-own-affine-surface-force-map', forceMap: values(entries), configurationDofs: n, queryDofs: workspace.queryDofs, dofCount: m,
        configurationDerivative: derivative(0, n), queryDerivative: derivative(n, workspace.queryDofs), derivative: derivative(0, m),
        point: values(contactPoint), axes: axes.map(values), normal: values(normal),
        tools: responses.map(r => ({ id: r.tool.id, edgeId: r.tool.edgeId, fraction: r.tool.fraction, center: values(r.center), lever: values(r.lever), tangent: values(r.tangent),
            omegaMap: Float64Array.from([0, 1, 2].flatMap(k => r.omegaColumns.map(column => b.data[column[k]]))) })),
        instantaneousRatesRequired: false, velocityKnown: false, prescribedFeedAndWallPowerKnown: false, certified: false,
        order, forceMapValid: true, configurationDerivativeValid: full, queryDerivativeValid: full, derivativeValid: full,
        derivatives: { finiteDifferences: false, repeatedUnitRateEvaluations: false, callerOwnsQueryChainRule: true,
            columns: 'per-tool(q0.xyz,q1.xyz,theta);per-tool(coordinate);current(point,normal,tangent)',
            storage: 'dof,tangent-component,derivative-column', forceMap: 'transpose-of-instantaneous-rate-jacobian' } };
}

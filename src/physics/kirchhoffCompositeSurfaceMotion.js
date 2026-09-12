const finite = (v, name) => { if (!Number.isFinite(v)) throw new TypeError(`${name} must be finite`); return v; };
const vector = (v, n, name) => {
    if (!v || v.length !== n) throw new TypeError(`${name} requires ${n} entries`);
    return Array.from(v, x => finite(x, name));
};
const dot = (a, b) => a.reduce((sum, x, i) => sum + x * b[i], 0);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = a => Math.hypot(...a);

// First/second forward derivatives. The extra second-order coordinate is x;
// mixed derivatives differentiate the frame's space/time connection in q.
// Fixed arenas can be reused; no numerical differentiation enters evaluation.
function arena(dimension, second, capacity = 4096) {
    const stride = 1 + dimension + (second ? dimension * dimension : 0), data = new Float64Array(stride * capacity);
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

export function createCompositeSurfaceMotionWorkspace(toolCount = 2) {
    if (toolCount !== 1 && toolCount !== 2) throw new RangeError('One or two surface materials are required');
    const dofCount = 9 + 2 * toolCount;
    return { toolCount, dofCount, second: arena(dofCount + 1, true), first: arena(dofCount, false) };
}

/**
 * Explicit QUADRATIC-HINGE contact reconstruction on [mid(x0,x1),mid(x1,x2)].
 * q_h interpolates the SAME three shared q nodes; t=q_h,x/|q_h,x|. This is a
 * declared local continuous reconstruction, not the affine DER edge field
 * between nodes and not a geometric/contact certificate. No shape is defaulted.
 *
 * On the two midpoint endpoints the quadratic's tangents equal the original
 * edge tangents. Time-PT each accepted old edge frame onto its current tangent;
 * spatial-PT the left frame onto t(x), then rotate by w*phi, where phi is the
 * unwrapped reference angle to the right frame. Interpolate independent theta
 * linearly between those endpoints. This states a continuous material frame
 * across the hinge; piecewise-constant DER directors alone have no unique
 * classical pointwise spatial derivative there.
 *
 * The accepted reference tangents must match previousPositions' edge tangents.
 * previousAngles are at the SAME fixed mesh coordinates, not already advected
 * material labels. All accepted frame/winding/history and map inputs stay frozen.
 * qt=N(q-qold)/dt, qx=N' q, u=-s_t/s_x; v=qt+u*qx (qx is NOT normalized).
 * t_t and frame A_t differentiate their time-PT chart along nodal BE qdot.
 * omega=t cross (t_t+u*t_x) + (theta_BE_t+u*theta_x+A_t+u*A_x)*t.
 * A=d2_ref dot derivative(d1_ref). Mixed second derivatives retain ALL frame
 * connection terms in the returned first slip Jacobian.
 *
 * Local columns: q0.xyz,q1.xyz,q2.xyz,thetaTool0.left/right,thetaTool1.left/right.
 * One tool means stationary wall; two tools are evaluated on the same common
 * centerline foot. levers are explicit {frame:'world'|'material',vector:[3]};
 * material coordinates use (d1,d2,t). Axes are {frame:'world',vectors:[2][3]}
 * or {frame:'material',toolId,vectors:[2][3]}. World vectors are frozen inputs;
 * material vectors, lever arms AND axes differentiate with q/theta.
 * slip=dt*axes dot [(v1+omega1 cross lever1)-(v2+omega2 cross lever2)].
 *
 * Outputs own their arrays and survive workspace reuse. No input is mutated.
 * This computes a stated local slip map and exact local Jacobian; it does not
 * choose contact feet, solve friction, infer unknown old spin or advance time.
 */
export function evaluateCompositeSurfaceMotion({ positions, previousPositions, coordinates, coordinate, reference, tools, dt,
    axes, reconstruction }, workspace = createCompositeSurfaceMotionWorkspace(tools?.length)) {
    if (reconstruction !== 'quadratic-hinge') throw new TypeError('Explicit reconstruction: quadratic-hinge is required');
    if (!(finite(dt, 'dt') > 0)) throw new RangeError('dt must be positive');
    if (!Array.isArray(tools) || tools.length !== workspace.toolCount || new Set(tools.map(t => t.id)).size !== tools.length ||
        tools.some(t => typeof t.id !== 'string' || !t.id)) throw new TypeError('One or two distinct named surface tools must match the workspace');
    const x = vector(coordinates, 3, 'coordinates'), at = finite(coordinate, 'coordinate');
    if (!(x[0] < x[1] && x[1] < x[2])) throw new RangeError('Surface coordinates must strictly increase');
    const leftX = x[0] + (x[1] - x[0]) / 2, rightX = x[1] + (x[2] - x[1]) / 2;
    if (at < leftX || at > rightX) throw new RangeError('Surface sample must lie in its explicit hinge dual interval');
    if (positions?.length !== 3 || previousPositions?.length !== 3 || reference?.length !== 2) throw new TypeError('Three current/previous positions and two accepted reference frames are required');
    const current = positions.map(p => vector(p, 3, 'position')), previous = previousPositions.map(p => vector(p, 3, 'previous position'));
    const refs = reference.map((f, edge) => {
        const tangent = vector(f.tangent, 3, 'accepted reference tangent'), director = vector(f.director, 3, 'accepted reference director');
        const old = previous[edge + 1].map((v, axis) => v - previous[edge][axis]), length = norm(old);
        if (!(length > 0) || Math.abs(dot(tangent, tangent) - 1) > 1e-10 || Math.abs(dot(director, director) - 1) > 1e-10 ||
            Math.abs(dot(tangent, director)) > 1e-10 || norm(tangent.map((v, axis) => v - old[axis] / length)) > 1e-10)
            throw new RangeError('Reference frames must be orthonormal accepted time frames at previous edge tangents');
        return { tangent, director };
    });
    const inputs = tools.map(t => {
        const map = t.materialMap;
        const dsDx = finite(map?.dsDx, 'dsDx'), dsDt = finite(map?.dsDt, 'dsDt'), sStart = finite(map?.sStart, 'sStart');
        if (!(dsDx > 0)) throw new RangeError('Material dsDx must be positive');
        if (!['world', 'material'].includes(t.lever?.frame)) throw new TypeError('Each lever requires an explicit world or material frame');
        return { id: t.id, angles: vector(t.angles, 2, 'current edge angles'), previousAngles: vector(t.previousAngles, 2, 'previous edge angles'),
            referenceTwist: finite(t.referenceTwist, 'accepted unwrapped referenceTwist'), dsDx, dsDt, sStart,
            lever: { frame: t.lever.frame, vector: vector(t.lever.vector, 3, 'lever') } };
    });
    if (!['world', 'material'].includes(axes?.frame) || axes.vectors?.length !== 2) throw new TypeError('Two explicit world/material tangent axes are required');
    const axisVectors = axes.vectors.map(v => vector(v, 3, 'contact tangent axis'));
    if (Math.abs(norm(axisVectors[0]) - 1) > 1e-10 || Math.abs(norm(axisVectors[1]) - 1) > 1e-10 || Math.abs(dot(...axisVectors)) > 1e-10)
        throw new RangeError('Contact tangent axes must be orthonormal');
    if (axes.frame === 'material' && !inputs.some(t => t.id === axes.toolId)) throw new RangeError('Material axes must identify a present surface tool');

    const a = workspace.second, b = workspace.first, n = workspace.dofCount, av = vectors(a), bv = vectors(b);
    a.reset(); b.reset();
    const q = current.map((p, node) => p.map((v, axis) => a.variable(v, 3 * node + axis))), sx = a.variable(at, n);
    const shape = [], shapeDx = [];
    for (let i = 0; i < 3; i++) {
        const [j, k] = [0, 1, 2].filter(index => index !== i), denominator = (x[i] - x[j]) * (x[i] - x[k]);
        shape.push(a.scale(a.mul(a.sub(sx, a.constant(x[j])), a.sub(sx, a.constant(x[k]))), 1 / denominator));
        shapeDx.push(a.scale(a.sub(a.scale(sx, 2), a.constant(x[j] + x[k])), 1 / denominator));
    }
    const combine = (coefficients, points) => [0, 1, 2].map(axis => coefficients.reduce((sum, weight, node) => a.add(sum, a.mul(weight, points[node][axis])), a.constant(0)));
    const center = combine(shape, q), qx = combine(shapeDx, q), tangent = av.unit(qx);
    const timeDisplacement = q.map((p, node) => p.map((v, axis) => a.sub(v, a.constant(previous[node][axis]))));
    const qt = combine(shape, timeDisplacement).map(v => a.scale(v, 1 / dt));
    const edgeTangents = [av.unit(av.sub(q[1], q[0])), av.unit(av.sub(q[2], q[1]))];
    const timeFrames = refs.map((f, edge) => av.transport(f.director.map(a.constant), f.tangent.map(a.constant), edgeTangents[edge]));
    const transportedRight = av.transport(timeFrames[0], edgeTangents[0], edgeTangents[1]);
    const rawPhi = a.atan2(av.dot(edgeTangents[1], av.cross(transportedRight, timeFrames[1])), av.dot(transportedRight, timeFrames[1]));
    const spatialBase = av.transport(timeFrames[0], edgeTangents[0], tangent), basePerpendicular = av.cross(tangent, spatialBase);
    const w = a.scale(a.sub(sx, a.constant(leftX)), 1 / (rightX - leftX));
    const lift = offset => {
        const r = b.constant(a.data[offset]);
        for (let i = 0; i < n; i++) b.data[r + 1 + i] = a.data[offset + 1 + i];
        return r;
    };
    const partial = (offset, index) => {
        const r = b.constant(a.data[offset + 1 + index]);
        for (let i = 0; i < n; i++) b.data[r + 1 + i] = a.data[offset + 1 + a.dimension + index * a.dimension + i];
        return r;
    };
    const qdot = q.flat().map((v, i) => b.scale(b.sub(lift(v), b.constant(previous[Math.floor(i / 3)][i % 3])), 1 / dt));
    const timeDerivative = offset => qdot.reduce((sum, rate, i) => b.add(sum, b.mul(partial(offset, i), rate)), b.constant(0));
    const t = tangent.map(lift), tx = tangent.map(v => partial(v, n)), tt = tangent.map(timeDerivative), vqt = qt.map(lift), vqx = qx.map(lift);
    const frames = inputs.map((input, i) => {
        const winding = 2 * Math.PI * Math.round((input.referenceTwist - a.data[rawPhi]) / (2 * Math.PI));
        const phi = a.add(rawPhi, a.constant(winding)), margin = Math.PI - Math.abs(a.data[phi] - input.referenceTwist);
        if (!(margin > 0)) throw new RangeError('Surface reference winding is ambiguous at its branch boundary');
        const angle = a.mul(w, phi), c = a.cos(angle), s = a.sin(angle);
        const d1 = av.add(av.times(spatialBase, c), av.times(basePerpendicular, s)), d2 = av.cross(tangent, d1);
        const thetaLeft = a.variable(input.angles[0], 9 + 2 * i), thetaRight = a.variable(input.angles[1], 10 + 2 * i);
        const theta = a.add(thetaLeft, a.mul(w, a.sub(thetaRight, thetaLeft)));
        const oldTheta = a.add(a.constant(input.previousAngles[0]), a.scale(w, input.previousAngles[1] - input.previousAngles[0]));
        const thetaDt = lift(a.scale(a.sub(theta, oldTheta), 1 / dt)), thetaDx = partial(theta, n);
        const d1dt = d1.map(timeDerivative), d1dx = d1.map(v => partial(v, n)), aTime = bv.dot(d2.map(lift), d1dt), aSpace = bv.dot(d2.map(lift), d1dx);
        const u = finite(-input.dsDt / input.dsDx, 'material advection');
        const spin = b.add(b.add(thetaDt, b.scale(thetaDx, u)), b.add(aTime, b.scale(aSpace, u)));
        const materialTangentRate = bv.add(tt, tx.map(v => b.scale(v, u)));
        const omega = bv.add(bv.cross(t, materialTangentRate), bv.times(t, spin));
        const velocity = bv.add(vqt, vqx.map(v => b.scale(v, u)));
        const m1 = av.add(av.times(d1, a.cos(theta)), av.times(d2, a.sin(theta))).map(lift), m2 = bv.cross(t, m1);
        const materialVector = components => components.reduce((sum, value, axis) => bv.add(sum, [m1, m2, t][axis].map(v => b.scale(v, value))), [0, 0, 0].map(b.constant));
        const lever = input.lever.frame === 'world' ? input.lever.vector.map(b.constant) : materialVector(input.lever.vector);
        const surfaceVelocity = bv.add(velocity, bv.cross(omega, lever));
        return { input, phi, margin, theta, thetaDt, thetaDx, aTime, aSpace, u, spin, omega, velocity, materialTangentRate,
            m1, m2, lever, surfaceVelocity, materialVector, referenceDirector: d1.map(lift) };
    });
    const tangentAxes = axes.frame === 'world' ? axisVectors.map(v => v.map(b.constant))
        : axisVectors.map(frames.find(f => f.input.id === axes.toolId).materialVector);
    const relative = frames.length === 1 ? frames[0].surfaceVelocity : bv.sub(frames[0].surfaceVelocity, frames[1].surfaceVelocity);
    const slip = tangentAxes.map(axis => b.scale(bv.dot(axis, relative), dt));
    if (!a.finite() || !b.finite()) throw new RangeError('Nonfinite surface motion or Jacobian');
    const values = v => Float64Array.from(v, offset => b.data[offset]);
    const jacobian = v => Float64Array.from(v.flatMap(offset => Array.from(b.data.subarray(offset + 1, offset + 1 + n))));
    // Report the actual pointwise reconstruction mismatch. This is not an
    // interval-wide error bound or permission to substitute one field for the
    // other in inertia/contact assembly. At a node use its outgoing edge.
    const affineEdge = at < x[1] ? 0 : 1, affineLength = x[affineEdge + 1] - x[affineEdge], fraction = (at - x[affineEdge]) / affineLength;
    const affinePosition = current[affineEdge].map((v, axis) => v + fraction * (current[affineEdge + 1][axis] - v));
    const affineDx = current[affineEdge].map((v, axis) => (current[affineEdge + 1][axis] - v) / affineLength);
    const affineDt = current[affineEdge].map((v, axis) => ((1 - fraction) * (v - previous[affineEdge][axis]) +
        fraction * (current[affineEdge + 1][axis] - previous[affineEdge + 1][axis])) / dt);
    const positionDifference = Float64Array.from(center, (offset, axis) => a.data[offset] - affinePosition[axis]);
    return { dofCount: n, slip: values(slip), jacobian: jacobian(slip), relativeVelocity: values(relative), relativeVelocityJacobian: jacobian(relative),
        axes: tangentAxes.map(values), position: Float64Array.from(center, offset => a.data[offset]), positionDt: values(vqt), positionDx: values(vqx),
        tangent: values(t), tangentDt: values(tt), tangentDx: values(tx), shape: Float64Array.from(shape, offset => a.data[offset]),
        tools: frames.map(f => ({ id: f.input.id, materialLabel: finite(f.input.sStart + f.input.dsDx * (at - x[0]), 'material sample label'),
            u: f.u, velocity: values(f.velocity), surfaceVelocity: values(f.surfaceVelocity), omega: values(f.omega), omegaJacobian: jacobian(f.omega),
            spin: b.data[f.spin], spinJacobian: jacobian([f.spin]), thetaDt: b.data[f.thetaDt], thetaDx: b.data[f.thetaDx],
            referenceTimeConnection: b.data[f.aTime], referenceSpaceConnection: b.data[f.aSpace],
            materialTangentRate: values(f.materialTangentRate), director1: values(f.m1), director2: values(f.m2),
            referenceDirector: values(f.referenceDirector), lever: values(f.lever), unwrappedReferenceTwist: a.data[f.phi], windingBranchMargin: f.margin,
            affineCenterlineVelocityDifference: Float64Array.from(f.velocity, (offset, axis) => finite(b.data[offset] - affineDt[axis] - f.u * affineDx[axis], 'affine velocity difference')),
            spinDofs: [9 + 2 * frames.indexOf(f), 10 + 2 * frames.indexOf(f)] })),
        reconstruction: { kind: 'quadratic-hinge', interval: [leftX, rightX], coordinate: at, field: 'local continuous contact reconstruction of shared q',
            equalsAffineEdgeField: false, contactCertified: false, timeChart: 'accepted-edge-time-parallel-transport', axesFrame: axes.frame,
            affineComparison: { edge: affineEdge, positionDifference, positionDifferenceNorm: finite(norm(positionDifference), 'affine position difference'),
                scope: 'pointwise difference only; no acceptance threshold or interval error certificate' } },
        derivatives: { kind: 'analytic-forward-second-order-connections', finiteDifferences: false,
            secondOrderOperations: a.used(), firstOrderOperations: b.used(), workspaceBytes: a.data.byteLength + b.data.byteLength } };
}

// VesselContactField normalizes a sparse-SDF gradient above this existing
// threshold. Below it the provider may substitute a centerline normal. This
// helper rejects that derivative contract; it never changes the query policy.
const PROVIDER_NORMAL_EPSILON = 1e-8;
const MISSING_BRICK = 0xffff;
const ROUND_OFF = 128 * Number.EPSILON;
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

function finiteVector(v, n, name) {
    if (!v || v.length !== n) throw new TypeError(`${name} needs ${n} finite values`);
    for (let i = 0; i < n; i++) if (!Number.isFinite(v[i])) throw new TypeError(`${name} must be finite`);
}
function positive(v, name) {
    if (!Number.isFinite(v) || v <= 0) throw new RangeError(`${name} must be positive and finite`);
}
function close(a, b, scale = 1) { return Math.abs(a - b) <= ROUND_OFF * Math.max(Number.MIN_VALUE, scale, Math.abs(a), Math.abs(b)); }
function unsupported(out, reason) { out.supported = false; out.reason = reason; return out; }
function cellWorkspace() {
    return { supported: false, reason: null, value: 0, cell: new Float64Array(3), fraction: new Float64Array(3),
        corners: new Float64Array(8), gradient: new Float64Array(3), hessian: new Float64Array(9), atCellBoundary: false };
}

/** Exact derivatives of the provider's UNSIGNED trilinear polynomial, before
 * sign selection, normalization, centerline fallback or BVH refinement.
 * Reads the same quantized corner values, including cross-brick cells.
 * It performs no detection, sign query, perturbation, resampling or clamping.
 */
export function evaluateSparseSdfTrilinearDerivatives({ field, position, contact }, out = cellWorkspace()) {
    unsupported(out, 'not-evaluated');
    finiteVector(position, 3, 'point');
    finiteVector(field?.sdfOrigin, 3, 'SDF origin'); finiteVector(field?.sdfDimensions, 3, 'SDF dimensions');
    positive(field.voxelSize, 'voxel size'); positive(field.sdfQuantization, 'distance quantization');
    if (!Number.isInteger(field.brickSize) || field.brickSize < 2 || !field.sdfBrickLookup || !field.sdfDistances ||
        field.sdfDimensions.some(v => !Number.isInteger(v) || v < 1))
        throw new TypeError('The provider sparse-SDF storage is required');
    const size = field.brickSize, h = field.voxelSize, dim = field.sdfDimensions, c = out.corners;
    const sourceGrid=field.sdfGridCoordinates?.(Array.from(position),contact);
    if(sourceGrid)finiteVector(sourceGrid,3,'Source SDF grid coordinates');
    for (let axis = 0; axis < 3; axis++) {
        const grid = sourceGrid?.[axis]??(position[axis] - field.sdfOrigin[axis]) / h;
        if (!Number.isFinite(grid) || !Number.isSafeInteger(Math.floor(grid))) return unsupported(out, 'unresolved-grid-coordinate');
        out.cell[axis] = Math.floor(grid); out.fraction[axis] = grid - out.cell[axis];
    }
    for (let k = 0; k < 8; k++) {
        const ix = out.cell[0] + (k & 1), iy = out.cell[1] + ((k >> 1) & 1), iz = out.cell[2] + (k >> 2);
        const bx = Math.floor(ix / size), by = Math.floor(iy / size), bz = Math.floor(iz / size);
        if (ix < 0 || iy < 0 || iz < 0 || bx >= dim[0] || by >= dim[1] || bz >= dim[2])
            return unsupported(out, 'missing-sdf-corner');
        const brick = field.sdfBrickLookup[bx + dim[0] * (by + dim[1] * bz)];
        if (!Number.isInteger(brick) || brick === MISSING_BRICK) return unsupported(out, 'missing-sdf-corner');
        const at = brick * size ** 3 + ix - bx * size + size * (iy - by * size + size * (iz - bz * size));
        c[k] = field.sdfDistances[at] * field.sdfQuantization;
        if (!Number.isFinite(c[k]) || c[k] < 0) return unsupported(out, 'invalid-unsigned-sdf-corner');
    }
    const [x, y, z] = out.fraction;
    // Keep value interpolation in the provider's arithmetic order.
    const x00 = c[0] + (c[1] - c[0]) * x, x10 = c[2] + (c[3] - c[2]) * x;
    const x01 = c[4] + (c[5] - c[4]) * x, x11 = c[6] + (c[7] - c[6]) * x;
    const y0 = x00 + (x10 - x00) * y, y1 = x01 + (x11 - x01) * y;
    out.value = y0 + (y1 - y0) * z;
    out.gradient[0] = (((c[1] - c[0]) * (1 - y) + (c[3] - c[2]) * y) * (1 - z) +
        ((c[5] - c[4]) * (1 - y) + (c[7] - c[6]) * y) * z) / h;
    out.gradient[1] = (((c[2] - c[0]) * (1 - x) + (c[3] - c[1]) * x) * (1 - z) +
        ((c[6] - c[4]) * (1 - x) + (c[7] - c[5]) * x) * z) / h;
    out.gradient[2] = (y1 - y0) / h;
    out.hessian.fill(0);
    out.hessian[1] = out.hessian[3] = (((c[3] - c[2] - c[1] + c[0]) * (1 - z) + (c[7] - c[6] - c[5] + c[4]) * z) / h) / h;
    out.hessian[2] = out.hessian[6] = (((c[5] - c[4] - c[1] + c[0]) * (1 - y) + (c[7] - c[6] - c[3] + c[2]) * y) / h) / h;
    out.hessian[5] = out.hessian[7] = (((c[6] - c[4] - c[2] + c[0]) * (1 - x) + (c[7] - c[5] - c[3] + c[1]) * x) / h) / h;
    if (![out.value, ...out.gradient, ...out.hessian].every(Number.isFinite)) return unsupported(out, 'nonfinite-sdf-derivative');
    out.atCellBoundary = out.fraction.some(f => f === 0 || f === 1);
    out.supported = true; out.reason = null;
    return out;
}

export function createCompositeWallGeometryWorkspace(pointCount = 2) {
    if (pointCount !== 1 && pointCount !== 2) throw new RangeError('A point or capsule has one or two position blocks');
    const n = 3 * pointCount;
    return { pointCount, dofCount: n, supported: false, reason: 'not-evaluated', cellData: cellWorkspace(),
        point: new Float64Array(3), normal: new Float64Array(3), pointGapGradient: new Float64Array(3),
        pointGapHessian: new Float64Array(9), pointNormalJacobian: new Float64Array(9), weights: new Float64Array(pointCount),
        gapGradient: new Float64Array(n), gapHessian: new Float64Array(n * n),
        normalForceColumn: new Float64Array(n), normalForceJacobian: new Float64Array(n * n),
        gap: 0, signedDistance: 0, radius: 0, gradientNorm: 0, source: null, sampleFraction: 0, sampleCount: 0,
        branchSign: 0, branchSignature: null, certified: false, derivativeScope: 'fixed-cell-sign-sampled-contact-branch' };
}

/** Differentiate an ALREADY selected provider contact; detection is unchanged.
 * Sparse SDF only: other sources require their own derivative contracts and
 * return supported:false. A changed cell/sign/source/sample needs a refresh.
 * This is a branch derivative, not a certificate of winner uniqueness or CCD.
 *
 * gapGradient is the true Jg. normalForceColumn distributes the physical UNIT
 * normal. The two are NOT interchangeable. normalForceJacobian includes Dn,
 * not just a frozen normal, for a Newton step with fixed physical Fn.
 */
export function differentiateCompositeWallContact({ field, contact, positions, radius }, out) {
    unsupported(out, 'not-evaluated');
    if (!positions || positions.length !== out.pointCount) throw new RangeError('Geometry must match the point/capsule workspace');
    positions.forEach(p => finiteVector(p, 3, 'position'));
    if (!Number.isFinite(radius) || radius < 0) throw new RangeError('A finite nonnegative radius is required');
    if (!contact || !Number.isFinite(contact.signedGap) || !Number.isFinite(contact.signedDistance))
        throw new TypeError('A finite provider contact is required');
    out.source = contact.source; out.gap = contact.signedGap; out.signedDistance = contact.signedDistance; out.radius = radius;
    if (contact.source !== 'sparse-sdf') return unsupported(out, `unsupported-source:${contact.source}`);
    finiteVector(contact.inward?.values, 3, 'provider normal');
    const t = out.pointCount === 1 ? 0 : contact.segmentT;
    if (!Number.isFinite(t) || t < 0 || t > 1) throw new RangeError('The selected capsule fraction must lie on its edge');
    out.sampleFraction = t; out.sampleCount = out.pointCount === 1 ? 0 : contact.capsuleSampleCount;
    if (out.pointCount === 2 && (!Number.isInteger(out.sampleCount) || out.sampleCount < 1))
        return unsupported(out, 'missing-capsule-sampling-branch');
    if (out.pointCount === 2 && !close(t * out.sampleCount, Math.round(t * out.sampleCount)))
        return unsupported(out, 'unrecognized-capsule-sampling-branch');
    out.weights[0] = out.pointCount === 1 ? 1 : 1 - t;
    if (out.pointCount === 2) out.weights[1] = t;
    for (let i = 0; i < 3; i++) {
        out.point[i] = out.pointCount === 1 || t === 0 ? positions[0][i] : t === 1 ? positions[1][i] :
            positions[0][i] + (positions[1][i] - positions[0][i]) * t;
        out.normal[i] = contact.inward.values[i];
    }
    if (!close(dot(out.normal, out.normal), 1)) return unsupported(out, 'provider-normal-is-not-unit');
    const cell = evaluateSparseSdfTrilinearDerivatives({ field, position: out.point, contact }, out.cellData);
    if (!cell.supported) return unsupported(out, cell.reason);
    // No claim of a unique classical derivative exactly on a voxel boundary.
    if (cell.atCellBoundary) return unsupported(out, 'sdf-cell-boundary');
    if (contact.signedDistance === 0 || cell.value === 0) return unsupported(out, 'unresolved-sign-branch');
    const sign = contact.signedDistance > 0 ? 1 : -1;
    if (!close(contact.signedDistance, sign * cell.value, Math.abs(cell.value)) ||
        !close(contact.signedGap, contact.signedDistance - radius, Math.max(Math.abs(contact.signedDistance), radius)))
        return unsupported(out, 'contact-does-not-match-sdf-polynomial');
    out.branchSign = sign;
    for (let i = 0; i < 3; i++) out.pointGapGradient[i] = sign * cell.gradient[i];
    for (let i = 0; i < 9; i++) out.pointGapHessian[i] = sign * cell.hessian[i];
    const m = Math.hypot(...out.pointGapGradient); out.gradientNorm = m;
    const providerNorm = Math.sqrt(dot(cell.gradient, cell.gradient));
    if (!(providerNorm > PROVIDER_NORMAL_EPSILON) || !Number.isFinite(providerNorm) || !Number.isFinite(m))
        return unsupported(out, 'provider-fallback-or-unresolved-normal');
    for (let i = 0; i < 3; i++) if (!close(out.pointGapGradient[i] / m, out.normal[i]))
        return unsupported(out, 'normal-is-not-normalized-signed-sdf-gradient');
    // Dn=(I-n*n^T) Hgap / |grad gap|. It is generally NONSYMMETRIC.
    for (let col = 0; col < 3; col++) {
        let projected = 0;
        for (let k = 0; k < 3; k++) projected += out.normal[k] * out.pointGapHessian[3 * k + col];
        for (let row = 0; row < 3; row++) out.pointNormalJacobian[3 * row + col] =
            (out.pointGapHessian[3 * row + col] - out.normal[row] * projected) / m;
    }
    const n = out.dofCount;
    for (let row = 0; row < n; row++) {
        const wr = out.weights[Math.floor(row / 3)], a = row % 3;
        out.gapGradient[row] = wr * out.pointGapGradient[a];
        out.normalForceColumn[row] = wr * out.normal[a];
        for (let col = 0; col < n; col++) {
            const weight = wr * out.weights[Math.floor(col / 3)], at = 3 * a + col % 3;
            out.gapHessian[n * row + col] = weight * out.pointGapHessian[at];
            out.normalForceJacobian[n * row + col] = weight * out.pointNormalJacobian[at];
        }
    }
    if ([out.gapGradient, out.gapHessian, out.normalForceColumn, out.normalForceJacobian].some(v => !v.every(Number.isFinite)))
        return unsupported(out, 'nonfinite-contact-derivative');
    out.branchSignature = `${contact.source}:${Array.from(cell.cell).join(',')}:${sign}:${out.sampleCount}:${t}:${radius}`;
    out.supported = true; out.reason = null;
    return out;
}

/** Convenience queries: exactly one original query, using the caller's
 * optional contactResult buffer. No thresholds/options of field are changed.
 */
export function queryCompositeWallPointGeometry({ field, position, radius, contactResult }, out = createCompositeWallGeometryWorkspace(1)) {
    unsupported(out, 'query-not-completed');
    finiteVector(position, 3, 'point');
    if (!Number.isFinite(radius) || radius < 0) throw new RangeError('A finite nonnegative radius is required');
    if (typeof field?.querySphere !== 'function') throw new TypeError('The original point-query provider is required');
    const contact = field.querySphere(position, radius, contactResult);
    return differentiateCompositeWallContact({ field, contact, positions: [position], radius }, out);
}
export function queryCompositeWallCapsuleGeometry({ field, positions, radius, contactResult }, out = createCompositeWallGeometryWorkspace(2)) {
    unsupported(out, 'query-not-completed');
    if (!positions || positions.length !== 2) throw new RangeError('Two capsule endpoints are required');
    positions.forEach(p => finiteVector(p, 3, 'endpoint'));
    if (!Number.isFinite(radius) || radius < 0) throw new RangeError('A finite nonnegative radius is required');
    if (typeof field?.queryCapsuleCoordinates !== 'function') throw new TypeError('The original capsule-query provider is required');
    const contact = field.queryCapsuleCoordinates(...positions[0], ...positions[1], radius, contactResult);
    return differentiateCompositeWallContact({ field, contact, positions, radius }, out);
}

function requireGeometry(g) {
    if (!g?.supported) throw new RangeError(`A supported fresh geometry branch is required (${g?.reason})`);
}

/** Original physical mixed equations on this local branch:
 * Rq contribution = -Fn*B, NCP = Fn-max(0,Fn-mu*g).
 * Jacobian blocks: -Fn*DB, -B; and active ? mu*Jg : 0,
 * active ? 0 : 1. The saddle system is generally NOT symmetric.
 * These are local residual contributions, never an acceptance certificate.
 */
export function evaluateCompositeWallMixedRow({ geometry: g, normalForce, penalty }) {
    requireGeometry(g); positive(penalty, 'NCP scaling');
    if (!Number.isFinite(normalForce) || normalForce < 0) throw new RangeError('Physical Fn must be finite and nonnegative');
    const trial = normalForce - penalty * g.gap;
    if (!Number.isFinite(trial)) throw new RangeError('Nonfinite NCP trial force');
    const n = g.dofCount, active = trial > 0;
    const mechanicalGradient = Float64Array.from(g.normalForceColumn, v => -normalForce * v);
    const mechanicalJacobian = Float64Array.from(g.normalForceJacobian, v => -normalForce * v);
    const forceColumn = Float64Array.from(g.normalForceColumn, v => -v);
    const ncpGeometryRow = Float64Array.from(g.gapGradient, v => active ? penalty * v : 0);
    const ncpResidual = normalForce - Math.max(0, trial);
    if (![ncpResidual, ...mechanicalGradient, ...mechanicalJacobian, ...ncpGeometryRow].every(Number.isFinite))
        throw new RangeError('Nonfinite physical mixed row');
    return { mechanicalGradient, mechanicalJacobian, forceColumn, ncpGeometryRow, ncpResidual,
        ncpForceDerivative: active ? 0 : 1, active, normalForce, dofCount: n,
        symmetric: false, scope: 'fixed-branch-physical-normal-NCP' };
}

/** Explicit alternative ENERGY model with a multiplier conjugate to g.
 * mode MUST be 'gap-potential'. A fixed physical Fn generally has no scalar
 * potential for normalized trilinear normals; that mode is rejected.
 * The effective normal magnitude is gapMultiplier*|grad g|, not gapMultiplier.
 * Changing Fn/|grad g| during minimization is not a frozen-multiplier AL step.
 * Effective trial force is not a committed physical friction budget.
 */
export function evaluateCompositeWallGapAugmented({ geometry: g, gapMultiplier, penalty, mode }) {
    requireGeometry(g);
    if (mode !== 'gap-potential') throw new RangeError('Choose explicit gap-potential mode; a fixed physical-normal energy is unsupported');
    positive(penalty, 'AL penalty');
    if (!Number.isFinite(gapMultiplier) || gapMultiplier < 0) throw new RangeError('The conjugate gap multiplier must be nonnegative and finite');
    const trial = Math.max(0, gapMultiplier - penalty * g.gap), n = g.dofCount;
    const energy = (trial - gapMultiplier) * (trial / 2 + gapMultiplier / 2) / penalty;
    const gradient = Float64Array.from(g.gapGradient, v => -trial * v), hessian = new Float64Array(n * n), gaussNewton = new Float64Array(n * n);
    if (trial > 0) for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
        gaussNewton[n * i + j] = penalty * g.gapGradient[i] * g.gapGradient[j];
        hessian[n * i + j] = gaussNewton[n * i + j] - trial * g.gapHessian[n * i + j];
    }
    const effectiveNormalForce = gapMultiplier * g.gradientNorm, effectiveTrialNormalForce = trial * g.gradientNorm;
    if (![energy, effectiveNormalForce, effectiveTrialNormalForce, ...gradient, ...hessian, ...gaussNewton].every(Number.isFinite))
        throw new RangeError('Nonfinite gap-conjugate augmented operator');
    return { energy, gradient, hessian, gaussNewton, trialGapMultiplier: trial, gapMultiplier,
        effectiveNormalForce, effectiveTrialNormalForce, hessianType: 'exact-fixed-branch', scope: 'explicit-gap-conjugate-energy' };
}

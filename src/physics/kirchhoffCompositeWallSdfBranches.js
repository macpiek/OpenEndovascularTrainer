// Bounded contact chart for ONE proved min/intersection trilinear cell seam.
// No provider calls, gap smoothing, collision selection, force rescaling or dt
// acceptance. Other contact sources and higher-codimension seams are separate.
const ROUND_OFF = 128 * Number.EPSILON;
const NORMAL_EPSILON = 1e-8; // Existing VesselContactField normalization limit.
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const close = (a, b) => Math.abs(a - b) <= ROUND_OFF * Math.max(1, Math.abs(a), Math.abs(b));
function vector(value, count, name) {
    if (!value || value.length !== count || !Array.from(value).every(Number.isFinite))
        throw new TypeError(`${name} needs ${count} finite entries`);
}
function positive(value, name) {
    if (!Number.isFinite(value) || value <= 0) throw new RangeError(`${name} must be positive and finite`);
}
function fieldGeometry(field) {
    vector(field?.sdfOrigin, 3, 'SDF origin'); vector(field?.sdfDimensions, 3, 'SDF brick dimensions');
    positive(field.voxelSize, 'voxel size');
    if (field.sdfDimensions.some(v => !Number.isSafeInteger(v) || v < 1))
        throw new RangeError('SDF brick dimensions must be positive safe integers');
    if (!Number.isSafeInteger(field.brickSize) || field.brickSize < 2 ||
        field.sdfDimensions.some(v => !Number.isSafeInteger(v * field.brickSize)))
        throw new RangeError('SDF grid extents must be safe integers');
}
function rowWorkspace(n, index) {
    return { index, side: index === 0 ? 'lower' : 'upper', source: 'sparse-sdf-cell-polynomial',
        derivativeScope: 'fixed-cell-polynomial-limit-or-proved-local-continuation', enabled: false, inOriginalCell: false,
        cell: new Float64Array(3), lower: new Float64Array(3), upper: new Float64Array(3),
        corners: new Float64Array(8), fraction: new Float64Array(3), dofs: new Int32Array(n),
        gap: NaN, signedDistance: NaN, gradientNorm: NaN,
        pointGapGradient: new Float64Array(3), pointGapHessian: new Float64Array(9), normal: new Float64Array(3),
        pointNormalDerivative: new Float64Array(9), gapJacobian: new Float64Array(n),
        normalForceColumn: new Float64Array(n), forceColumn: new Float64Array(n), normalDerivative: new Float64Array(n * n) };
}
export function createCompositeWallSdfBranchesWorkspace(pointCount = 2) {
    if (pointCount !== 1 && pointCount !== 2) throw new RangeError('One point or one two-endpoint sampled capsule is required');
    const n = 3 * pointCount;
    return { pointCount, dofCount: n, supported: false, reason: 'not-evaluated', classification: 'unresolved',
        rows: [rowWorkspace(n, 0), rowWorkspace(n, 1)], positions: Array.from({ length: pointCount }, () => new Float64Array(3)),
        point: new Float64Array(3), weights: new Float64Array(pointCount), grid: new Float64Array(3),
        face: { axis: -1, gridIndex: NaN, coordinate: NaN },
        domain: { lower: new Float64Array(3), upper: new Float64Array(3), outerBoundaryIncluded: false },
        jumpCorners: new Float64Array(4), jumpLowerBound: NaN, jumpUpperBound: NaN, continuity: false,
        onSeam: false, selectedRow: -1, sampleFraction: NaN, sampleCount: 0, radius: NaN, branchSign: 0,
        rawContact: { signedGap: NaN, signedDistance: NaN, source: null, segmentT: NaN, capsuleSampleCount: 0,
            inward: { values: new Float64Array(3) } },
        derivativeScope: 'one-proved-min-seam-fixed-inside-sign-and-sampled-point', queryCount: 0, certified: false };
}
function unavailable(out, reason, classification = 'unresolved') {
    out.supported = false; out.reason = reason; out.classification = classification; out.certified = false;
    for (const row of out.rows) {
        row.enabled = false; row.gap = row.signedDistance = row.gradientNorm = NaN;
        for (const key of ['pointGapGradient', 'pointGapHessian', 'normal', 'pointNormalDerivative',
            'gapJacobian', 'normalForceColumn', 'forceColumn', 'normalDerivative']) row[key].fill(NaN);
    }
    return out;
}
function readCorners(field, row, localInside = false) {
    const size = field.brickSize, dimensions = field.sdfDimensions;
    let insideCount=0;
    for (let k = 0; k < 8; k++) {
        const xyz = [row.cell[0] + (k & 1), row.cell[1] + ((k >> 1) & 1), row.cell[2] + (k >> 2)];
        const brick = xyz.map(v => Math.floor(v / size));
        if (xyz.some((v, i) => v < 0 || brick[i] >= dimensions[i])) return 'missing-sdf-corner';
        const index = field.sdfBrickLookup[brick[0] + dimensions[0] * (brick[1] + dimensions[1] * brick[2])];
        if (!Number.isSafeInteger(index) || index < 0 || index === 0xffff) return 'missing-sdf-corner';
        const at = index * size ** 3 + xyz[0] - brick[0] * size + size * (xyz[1] - brick[1] * size + size * (xyz[2] - brick[2] * size));
        if (!Number.isSafeInteger(at) || at < 0) return 'unresolved-sdf-storage-index';
        const encoded = field.sdfDistances[at];
        if (!Number.isFinite(encoded) || encoded < 0) return 'invalid-unsigned-sdf-corner';
        row.corners[k] = encoded * field.sdfQuantization;
        if (!Number.isFinite(row.corners[k])) return 'invalid-unsigned-sdf-corner';
        // Uniform inside occupancy proves this provider sign branch over both
        // cells. Mixed occupancy, missing bits and exterior sign corrections
        // need their own proof; a positive value at one point is insufficient.
        const byte = Math.floor(at / 8);
        if (!field.sdfInsideBits || byte >= field.sdfInsideBits.length) return 'missing-inside-sign-proof';
        if ((field.sdfInsideBits[byte] & (1 << (at % 8))) !== 0) insideCount++;
    }
    // The provider consults the packed lumen predicate for mixed occupancy
    // only. A uniform exterior cell cannot borrow this local proof.
    if(insideCount!==8&&!(insideCount>0&&localInside))return 'nonuniform-or-exterior-sign-branch';
    return null;
}
function polynomial(row, grid, h, radius) {
    const c = row.corners, f = row.fraction;
    for (let i = 0; i < 3; i++) f[i] = grid[i] - row.cell[i];
    const [x, y, z] = f;
    const x00 = c[0] + (c[1] - c[0]) * x, x10 = c[2] + (c[3] - c[2]) * x;
    const x01 = c[4] + (c[5] - c[4]) * x, x11 = c[6] + (c[7] - c[6]) * x;
    const y0 = x00 + (x10 - x00) * y, y1 = x01 + (x11 - x01) * y;
    row.signedDistance = y0 + (y1 - y0) * z; row.gap = row.signedDistance - radius;
    const G = row.pointGapGradient, H = row.pointGapHessian;
    G[0] = (((c[1] - c[0]) * (1 - y) + (c[3] - c[2]) * y) * (1 - z) +
        ((c[5] - c[4]) * (1 - y) + (c[7] - c[6]) * y) * z) / h;
    G[1] = (((c[2] - c[0]) * (1 - x) + (c[3] - c[1]) * x) * (1 - z) +
        ((c[6] - c[4]) * (1 - x) + (c[7] - c[5]) * x) * z) / h;
    G[2] = (y1 - y0) / h;
    H.fill(0);
    H[1] = H[3] = ((c[3] - c[2] - c[1] + c[0]) * (1 - z) + (c[7] - c[6] - c[5] + c[4]) * z) / h / h;
    H[2] = H[6] = ((c[5] - c[4] - c[1] + c[0]) * (1 - y) + (c[7] - c[6] - c[3] + c[2]) * y) / h / h;
    H[5] = H[7] = ((c[6] - c[4] - c[2] + c[0]) * (1 - x) + (c[7] - c[5] - c[3] + c[1]) * x) / h / h;
    const m = Math.hypot(...G); row.gradientNorm = m;
    if (!(Math.sqrt(dot(G, G)) > NORMAL_EPSILON) || !Number.isFinite(m)) return false;
    for (let i = 0; i < 3; i++) row.normal[i] = G[i] / m;
    for (let j = 0; j < 3; j++) {
        let along = 0;
        for (let k = 0; k < 3; k++) along += row.normal[k] * H[3 * k + j];
        for (let i = 0; i < 3; i++) row.pointNormalDerivative[3 * i + j] = (H[3 * i + j] - row.normal[i] * along) / m;
    }
    return Number.isFinite(row.gap) && [G, H, row.pointNormalDerivative].every(v => v.every(Number.isFinite));
}

/** Evaluate BOTH exact cell polynomials in a proved two-cell chart. At the
 * face these are exact one-sided limits, not perturbed queries. Away from it,
 * the nonselected polynomial is only a locally valid auxiliary constraint;
 * final certification forbids a loaded wrong-domain branch. `contact` is the
 * already selected ORIGINAL query, never a synthesized normal/extra query.
 */
export function evaluateCompositeWallSdfBranches({ field, face, positions, radius, contact, dofs, domainBox }, out) {
    unavailable(out, 'not-evaluated'); out.localInsideProof = null; out.continuity = false; out.onSeam = false; out.selectedRow = -1;
    out.jumpLowerBound = out.jumpUpperBound = NaN; out.branchSign = 0; out.queryCount = 0;
    fieldGeometry(field); positive(field.sdfQuantization, 'distance quantization');
    if (!Number.isInteger(field.brickSize) || field.brickSize < 2 || !field.sdfBrickLookup || !field.sdfDistances)
        throw new TypeError('Sparse-SDF brick storage is required');
    if (!face || !Number.isInteger(face.axis) || face.axis < 0 || face.axis > 2 || !Number.isSafeInteger(face.gridIndex) || face.gridIndex < 0)
        throw new RangeError('A face needs axis 0..2 and a nonnegative safe integer gridIndex');
    if (!positions || positions.length !== out.pointCount) throw new RangeError('Point support must match the workspace');
    positions.forEach(p => vector(p, 3, 'position'));
    if (!Number.isFinite(radius) || radius < 0) throw new RangeError('Radius must be finite and nonnegative');
    if (!contact || !Number.isFinite(contact.signedGap) || !Number.isFinite(contact.signedDistance))
        throw new TypeError('A finite original contact is required');
    vector(contact.inward?.values, 3, 'original contact normal');
    const raw = out.rawContact;
    raw.signedGap = contact.signedGap; raw.signedDistance = contact.signedDistance; raw.source = contact.source;
    raw.segmentT = contact.segmentT; raw.capsuleSampleCount = contact.capsuleSampleCount; raw.inward.values.set(contact.inward.values);
    if (contact.source !== 'sparse-sdf') return unavailable(out, `unsupported-source:${contact.source}`);
    if (!(contact.signedDistance > 0)) return unavailable(out, 'unresolved-or-exterior-sign-branch');
    const t = out.pointCount === 1 ? 0 : contact.segmentT;
    const count = out.pointCount === 1 ? 0 : contact.capsuleSampleCount;
    if (!Number.isFinite(t) || t < 0 || t > 1 || (out.pointCount === 2 &&
        (!Number.isSafeInteger(count) || count < 1 || !close(t * count, Math.round(t * count)))))
        return unavailable(out, 'unrecognized-capsule-sampling-branch');
    if (dofs !== undefined && (dofs.length !== out.dofCount || !Array.from(dofs).every(v => Number.isSafeInteger(v) && v >= 0 && v <= 0x7fffffff) || new Set(dofs).size !== dofs.length))
        throw new RangeError('Dofs must be distinct nonnegative Int32 indices matching the point support');
    out.sampleFraction = t; out.sampleCount = count; out.radius = radius; out.weights[0] = out.pointCount === 1 ? 1 : 1 - t;
    if (out.pointCount === 2) out.weights[1] = t;
    positions.forEach((p, i) => out.positions[i].set(p));
    for (let i = 0; i < 3; i++) {
        out.point[i] = t === 0 ? positions[0][i] : t === 1 ? positions[1][i] : positions[0][i] + (positions[1][i] - positions[0][i]) * t;
        out.grid[i] = (out.point[i] - field.sdfOrigin[i]) / field.voxelSize;
    }
    if(typeof field.sdfGridCoordinates==='function')out.grid.set(field.sdfGridCoordinates(out.point,contact));
    for(let i=0;i<3;i++){
        if (!Number.isFinite(out.grid[i]) || !Number.isSafeInteger(Math.floor(out.grid[i]))) return unavailable(out, 'unresolved-grid-coordinate');
    }
    const axis = face.axis, index = face.gridIndex, tangentAxes = [0, 1, 2].filter(i => i !== axis);
    out.face.axis = axis; out.face.gridIndex = index; out.face.coordinate = typeof field.sdfFaceCoordinate==='function'?field.sdfFaceCoordinate(axis,index):field.sdfOrigin[axis] + index * field.voxelSize;
    if (!Number.isFinite(out.face.coordinate)) return unavailable(out, 'unresolved-grid-coordinate');
    if (tangentAxes.some(i => Number.isInteger(out.grid[i]))) return unavailable(out, 'multi-axis-cell-intersection');
    if (!(out.grid[axis] > index - 1 && out.grid[axis] < index + 1)) return unavailable(out, 'outside-adjacent-cell-domain');
    out.onSeam = out.grid[axis] === index;
    out.selectedRow = out.grid[axis] < index ? 0 : 1;
    let localInside=false;out.localInsideProof=null;
    if(domainBox!==undefined) {
        vector(domainBox.lower,3,'Local seam lower bound');vector(domainBox.upper,3,'Local seam upper bound');
        if(domainBox.lower.some((v,i)=>!(v<domainBox.upper[i])))throw new RangeError('Local seam bounds must increase');
        const center=domainBox.lower.map((v,i)=>(v+domainBox.upper[i])/2),ballRadius=Math.hypot(...domainBox.upper.map((v,i)=>(v-domainBox.lower[i])/2));
        if(typeof field.certifyInsideBallCoordinates==='function') {
            out.localInsideProof=field.certifyInsideBallCoordinates(...center,ballRadius);
            localInside=out.localInsideProof?.supported===true;
        }
    }
    for (const row of out.rows) {
        for (let i = 0; i < 3; i++) {
            row.cell[i] = i === axis ? index - 1 + row.index : Math.floor(out.grid[i]);
            row.lower[i] = field.sdfOrigin[i] + row.cell[i] * field.voxelSize; row.upper[i] = row.lower[i] + field.voxelSize;
        }
        row.inOriginalCell = out.onSeam || row.index === out.selectedRow;
        const reason = readCorners(field, row, localInside); if (reason) return unavailable(out, reason);
    }
    const [left, right] = out.rows, axisBit = 1 << axis;
    out.continuity = true;
    for (let k = 0; k < 4; k++) {
        const low = ((k & 1) << tangentAxes[0]) | (((k >> 1) & 1) << tangentAxes[1]), high = low | axisBit;
        if (left.corners[high] !== right.corners[low]) out.continuity = false;
        out.jumpCorners[k] = ((left.corners[high] - left.corners[low]) - (right.corners[high] - right.corners[low])) / field.voxelSize;
    }
    if (!out.continuity) return unavailable(out, 'discontinuous-shared-face');
    for (let i = 0; i < 3; i++) {
        out.domain.lower[i] = left.lower[i]; out.domain.upper[i] = right.upper[i];
    }
    if (domainBox !== undefined) {
        vector(domainBox.lower, 3, 'Local seam lower bound'); vector(domainBox.upper, 3, 'Local seam upper bound');
        if (domainBox.lower.some((v,i)=>v<out.domain.lower[i]||domainBox.upper[i]>out.domain.upper[i]||!(v<domainBox.upper[i])) ||
            !(domainBox.lower[axis]<out.face.coordinate&&domainBox.upper[axis]>out.face.coordinate))
            throw new RangeError('Local seam domain must lie within the adjacent cells and straddle their face');
        if (out.point.some((v,i)=>v<=domainBox.lower[i]||v>=domainBox.upper[i]))return unavailable(out,'outside-local-seam-domain');
        // The derivative jump is bilinear on the shared face. Its extrema
        // on an axis-aligned subrectangle occur at that rectangle's corners.
        const original=out.jumpCorners.slice();
        for(let k=0;k<4;k++) {
            const fractions=tangentAxes.map((a,j)=>((k&(1<<j)?domainBox.upper[a]:domainBox.lower[a])-left.lower[a])/field.voxelSize);
            const [u,v]=fractions;
            out.jumpCorners[k]=(1-v)*(original[0]+u*(original[1]-original[0]))+v*(original[2]+u*(original[3]-original[2]));
        }
        out.domain.lower.set(domainBox.lower);out.domain.upper.set(domainBox.upper);
    }
    out.jumpLowerBound = Math.min(...out.jumpCorners); out.jumpUpperBound = Math.max(...out.jumpCorners);
    if (out.jumpCorners.every(v => v === 0)) return unavailable(out, 'smooth-seam-needs-one-branch', 'smooth');
    if (out.jumpUpperBound < 0) return unavailable(out, 'max-union-needs-different-contact-law', 'max-union');
    if (!(out.jumpLowerBound > 0)) return unavailable(out, 'unresolved-min-max-order', 'unresolved-order');
    out.branchSign = 1;
    for (const row of out.rows) if (!polynomial(row, out.grid, field.voxelSize, radius))
        return unavailable(out, 'zero-fallback-or-nonfinite-branch-gradient');
    const selected = out.rows[out.selectedRow];
    if (!close(raw.signedDistance, selected.signedDistance) || !close(raw.signedGap, selected.gap) ||
        !close(raw.signedGap, raw.signedDistance - radius) || !close(raw.signedGap, Math.min(left.gap, right.gap)))
        return unavailable(out, 'original-contact-does-not-match-cell-chart');
    if (!close(dot(raw.inward.values, raw.inward.values), 1) || !selected.normal.every((v, i) => close(v, raw.inward.values[i])))
        return unavailable(out, 'original-normal-does-not-match-selected-cell');
    if (left.normal.every((v, i) => close(v, right.normal[i])))
        return unavailable(out, 'coincident-normal-rays-needs-one-force', 'coincident-normal-rays');
    if (left.normal.every((v, i) => close(v, -right.normal[i])))
        return unavailable(out, 'opposite-dependent-normal-rays', 'dependent-normal-rays');
    const n = out.dofCount;
    for (const row of out.rows) {
        row.enabled = true;
        for (let i = 0; i < n; i++) {
            const w = out.weights[Math.floor(i / 3)], a = i % 3;
            row.dofs[i] = dofs === undefined ? i : dofs[i];
            row.gapJacobian[i] = w * row.pointGapGradient[a];
            row.normalForceColumn[i] = w * row.normal[a]; row.forceColumn[i] = -row.normalForceColumn[i];
            for (let j = 0; j < n; j++) row.normalDerivative[n * i + j] =
                w * out.weights[Math.floor(j / 3)] * row.pointNormalDerivative[3 * a + j % 3];
        }
    }
    out.supported = true; out.reason = null; out.classification = 'min-intersection';
    return out;
}

/** Contact-only certificate. Mechanical equilibrium is the caller's original
 * Chain/inertia/length residual. No caller tolerances are inferred or relaxed.
 * A loaded adjacent continuation is forbidden away from the exact grid tie;
 * open rows may be eliminated only after this original certificate is kept.
 */
export function measureCompositeWallSdfBranches(out, { forces, penalty, gapTolerance, forceTolerance, workTolerance, about = [0, 0, 0] }) {
    vector(forces, 2, 'two physical normal forces'); vector(about, 3, 'moment origin');
    for (const [name, value] of Object.entries({ penalty, gapTolerance, forceTolerance, workTolerance })) positive(value, name);
    if (!out.supported) throw new RangeError(`SDF seam rows unavailable: ${out.reason}`);
    const proof = { scope: 'discrete-contact-cone-only', classification: out.classification,
        source: out.rawContact.source, onSeam: out.onSeam, selectedRow: out.selectedRow,
        originalGapMatches: close(out.rawContact.signedGap, Math.min(...out.rows.map(row => row.gap))),
        converged: false, domainAdmissible: true, tieAdmissible: true,
        originalGap: out.rawContact.signedGap, maximumPenetration: Math.max(0, -out.rawContact.signedGap),
        maximumNegativeForce: 0, maximumProjectedResidual: 0, maximumComplementarity: 0,
        resultant: new Float64Array(3), moment: new Float64Array(3),
        physicalGradient: new Float64Array(out.dofCount), nodalForces: new Float64Array(out.dofCount),
        forces: Float64Array.from(forces), branchGaps: new Float64Array(2), forceMagnitude: 0, scalarForceSum: forces[0] + forces[1] };
    for (const row of out.rows) {
        const Fn = forces[row.index]; proof.branchGaps[row.index] = row.gap;
        proof.maximumPenetration = Math.max(proof.maximumPenetration, -row.gap);
        proof.maximumNegativeForce = Math.max(proof.maximumNegativeForce, -Fn);
        proof.maximumProjectedResidual = Math.max(proof.maximumProjectedResidual, Math.abs(Fn - Math.max(0, Fn - penalty * row.gap)));
        proof.maximumComplementarity = Math.max(proof.maximumComplementarity, Math.abs(Fn * row.gap));
        if (Fn !== 0 && !row.inOriginalCell) { proof.domainAdmissible = false; proof.tieAdmissible = false; }
        for (let i = 0; i < 3; i++) proof.resultant[i] += Fn * row.normal[i];
        for (let i = 0; i < out.dofCount; i++) {
            proof.physicalGradient[i] += Fn * row.forceColumn[i]; proof.nodalForces[i] += Fn * row.normalForceColumn[i];
        }
    }
    const arm = out.point.map((v, i) => v - about[i]), F = proof.resultant;
    proof.moment.set([arm[1] * F[2] - arm[2] * F[1], arm[2] * F[0] - arm[0] * F[2], arm[0] * F[1] - arm[1] * F[0]]);
    proof.forceMagnitude = Math.hypot(...F);
    if (![proof.maximumPenetration, proof.maximumNegativeForce, proof.maximumProjectedResidual,
        proof.maximumComplementarity, proof.forceMagnitude, proof.scalarForceSum,
        ...proof.resultant, ...proof.moment, ...proof.physicalGradient].every(Number.isFinite))
        throw new RangeError('Nonfinite original cone force or certificate');
    proof.converged = proof.originalGapMatches && proof.domainAdmissible && proof.tieAdmissible && proof.maximumNegativeForce === 0 &&
        proof.maximumPenetration <= gapTolerance && proof.maximumProjectedResidual <= forceTolerance && proof.maximumComplementarity <= workTolerance;
    return proof;
}

/** First cell-face event for a KNOWN affine sampled point. Starting on any
 * face returns alpha=0 explicitly; the caller must consume that event rather
 * than repeatedly calling with the identical start. No nudge or substep is
 * performed. Multiple simultaneous axes are reported but unsupported here.
 */
export function findCompositeWallSdfSeamCrossing({ field, position, delta }) {
    fieldGeometry(field); vector(position, 3, 'position'); vector(delta, 3, 'trial displacement');
    const grid = position.map((v, i) => (v - field.sdfOrigin[i]) / field.voxelSize);
    if (grid.some(v => !Number.isFinite(v) || !Number.isSafeInteger(Math.floor(v)))) throw new RangeError('Unresolved grid coordinate');
    const extents = field.sdfDimensions.map(v => v * field.brickSize - 1);
    if (grid.some((v, i) => v < 0 || v > extents[i])) throw new RangeError('Trial sample is outside the SDF grid bounds');
    const startAxes = grid.map((v, axis) => Number.isInteger(v) ? axis : -1).filter(v => v >= 0);
    let events;
    if (startAxes.length) events = startAxes.map(axis => ({ axis, gridIndex: grid[axis], alpha: 0, direction: Math.sign(delta[axis]) }));
    else {
        events = [];
        for (let axis = 0; axis < 3; axis++) if (delta[axis] !== 0) {
            const direction = Math.sign(delta[axis]), gridIndex = direction > 0 ? Math.floor(grid[axis]) + 1 : Math.floor(grid[axis]);
            const alpha = (gridIndex - grid[axis]) * field.voxelSize / delta[axis];
            if (alpha >= 0 && alpha <= 1) events.push({ axis, gridIndex, alpha, direction });
        }
        if (!events.length) return { hit: false, supported: true, reason: null, alpha: null, events: [] };
        const alpha = Math.min(...events.map(event => event.alpha));
        // Near-coincident events cannot safely be ordered at machine precision.
        // Conservatively report the unresolved cluster, never skip one face.
        events = events.filter(event => close(event.alpha, alpha));
    }
    const alpha = Math.min(...events.map(event => event.alpha));
    const point = position.map((v, i) => v + alpha * delta[i]);
    for (const event of events) {
        event.coordinate = field.sdfOrigin[event.axis] + event.gridIndex * field.voxelSize;
        const lower = grid.map(Math.floor), upper = [...lower];
        lower[event.axis] = event.gridIndex - 1; upper[event.axis] = event.gridIndex;
        event.lowerCell = lower; event.upperCell = upper;
    }
    const hasBothCells = events.every(event => event.gridIndex > 0 && event.gridIndex < extents[event.axis]);
    return { hit: true, supported: events.length === 1 && hasBothCells,
        reason: events.length > 1 ? 'multi-axis-or-unresolved-simultaneous-crossing' : !hasBothCells ? 'missing-adjacent-grid-domain' : null,
        alpha, point, startsOnFace: startAxes.length > 0, events };
}

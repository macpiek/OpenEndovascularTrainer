const EPSILON = 1e-12;
const XYZ = ['x', 'y', 'z'];

function finite(value, label) {
    if (!Number.isFinite(value)) throw new TypeError(`${label} must be finite`);
    return value;
}

function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

function cross(a, b, out) {
    const x = a[1] * b[2] - a[2] * b[1], y = a[2] * b[0] - a[0] * b[2];
    const z = a[0] * b[1] - a[1] * b[0];
    out[0] = x; out[1] = y; out[2] = z;
}

function readPoint(body, node, out) {
    if (!Number.isInteger(node) || node < 0 || node >= body.count) throw new RangeError('Invalid contact node');
    for (let i = 0; i < 3; i++) out[i] = finite(body[XYZ[i]][node], 'contact position');
}

function storage(record) {
    if (record._normalRowScratch) return record._normalRowScratch;
    const scratch = { gradients: [], diagnostics: { normalMomentResidual: new Float64Array(3) } };
    for (const key of ['inner0', 'inner1', 'outer0', 'outer1', 'point', 'axis', 'offset',
        'radial', 'normal', 'innerGradient', 'axisGradient', 'wireDirection', 'moment']) scratch[key] = new Float64Array(3);
    record._normalRowScratch = scratch;
    return scratch;
}

function append(rows, count, side, node, axis, value) {
    if (value === 0) return count;
    const entry = rows[count] ??= Object.seal({ side: 0, dof: 0, value: 0 });
    entry.side = side; entry.dof = node * 6 + axis; entry.value = value;
    return count + 1;
}

/** Exact endpoint Jacobian for the CURRENT effective rounded-lip gap.
 * Writes/returns record.normalGradients:[{side,dof,value}], borrowed until next
 * build. Only diagnostics/cache/normalGradients are mutated; gap, normal,
 * geometry, positions and multipliers are unchanged.
 *
 * Fillet: g=hypot(z+f,rho-(clearance+f))-f. The outer shaft axis is computed
 * from its endpoints in world, so its derivative belongs to those endpoints,
 * NOT an extra orientation-frame gradient on top. This gives the missing
 * normal moment without changing the existing collision function.
 *
 * Five fixed quadrature samples hold innerT fixed. The sixth current-world
 * sample is the moving z=-f side boundary; it needs implicit innerT derivatives.
 * normalInnerParameterMode:'fixed'|'portal-side-boundary' explicitly overrides
 * inference from current-world quadrature values and the boundary coordinate.
 *
 * includeRim:true additionally differentiates the implicit crossing coordinate
 * of distal-rim. Default false leaves that separate migration opt-in. A
 * sliding-rim nearest-point distance already has the default stencil gradient
 * on a fixed closest segment (including a fixed clamped endpoint), so no custom
 * rows are added. Caller must rebuild after contact branch/topology changes.
 */
export function buildKirchhoffContactNormalGradients(constraint, record, { includeRim = false } = {}) {
    record.normalGradients = null;
    if (record.kind === 'side' && record.portalSideGradients) return record.normalGradients = record.portalSideGradients;
    const s = storage(record), diagnostics = s.diagnostics;
    record.normalGradientDiagnostics = diagnostics;
    diagnostics.kind = record.kind; diagnostics.supported = false;
    diagnostics.normalMomentResidual.fill(0);
    if (record.kind !== 'distal-fillet' && !(includeRim && record.kind === 'distal-rim')) {
        diagnostics.reason = record.kind === 'sliding-rim' ? 'default-closest-point-gradient' : 'default-gradient';
        return null;
    }
    const inner = constraint.innerBody, outer = constraint.outerBody;
    const innerSegment = record._innerSegmentIndex ?? record.manifoldContact?.innerSegmentIndex;
    const outerSegment = record._outerSegmentIndex ?? record.manifoldContact?.outerSegmentIndex;
    if (record._innerNodeIndices || record._outerNodeIndices) throw new RangeError('Portal normal gradients require the current linear endpoint records');
    readPoint(inner, innerSegment, s.inner0); readPoint(inner, innerSegment + 1, s.inner1);
    readPoint(outer, outerSegment, s.outer0); readPoint(outer, outerSegment + 1, s.outer1);
    const t = finite(record.innerT, 'innerT');
    if (t < 0 || t > 1 || record.outerT !== 1) throw new RangeError('Portal normal record must use a valid wire sample and the outer tip');
    for (let i = 0; i < 3; i++) {
        s.axis[i] = s.outer1[i] - s.outer0[i];
        s.wireDirection[i] = s.inner1[i] - s.inner0[i];
        s.point[i] = s.inner0[i] + t * s.wireDirection[i];
        s.offset[i] = s.point[i] - s.outer1[i];
    }
    const length = Math.hypot(...s.axis);
    if (length <= EPSILON) throw new RangeError('Outer tip segment is degenerate');
    for (let i = 0; i < 3; i++) s.axis[i] /= length;
    const z = dot(s.offset, s.axis);
    for (let i = 0; i < 3; i++) s.radial[i] = s.offset[i] - z * s.axis[i];
    const rho = Math.hypot(...s.radial);
    const radialEpsilon = record.normalRadialEpsilon ?? EPSILON;
    if (!Number.isFinite(radialEpsilon) || radialEpsilon < 0) throw new RangeError('Invalid collector radial epsilon');
    let directional = false;
    if (rho > radialEpsilon) for (let i = 0; i < 3; i++) s.radial[i] /= rho;
    else {
        // The old collector chooses an azimuth at its radial cusp. Preserve
        // that selected directional branch, but do not call it differentiable.
        const axialNormal = record.normal[0] * s.axis[0] + record.normal[1] * s.axis[1] + record.normal[2] * s.axis[2];
        for (let i = 0; i < 3; i++) s.radial[i] = record.normal[i] - axialNormal * s.axis[i];
        const magnitude = Math.hypot(...s.radial);
        if (magnitude <= EPSILON) throw new RangeError('Degenerate portal azimuth requires a selected normal direction');
        for (let i = 0; i < 3; i++) s.radial[i] /= magnitude;
        directional = true;
    }
    let boundaryMode = record.kind === 'distal-rim';
    if (record.kind === 'distal-fillet') {
        const f = finite(constraint.portalFilletRadius, 'portalFilletRadius');
        if (f <= 0) throw new RangeError('A fillet record requires positive effective fillet radius');
        const radius = Math.max(finite(inner.nodeRadius?.[innerSegment] ?? inner.radius, 'wire radius'),
            finite(inner.nodeRadius?.[innerSegment + 1] ?? inner.radius, 'wire radius'));
        const clearance = Math.max(0, finite(constraint.innerRadius, 'lumen radius') - radius);
        const major = clearance + f, u = z + f, v = rho - major, distance = Math.hypot(u, v);
        if (distance <= EPSILON) throw new RangeError('The fillet signed-distance gradient is undefined at its circle center');
        for (let i = 0; i < 3; i++) {
            s.normal[i] = -(s.axis[i] * u + s.radial[i] * v) / distance;
            s.innerGradient[i] = -s.normal[i];
            s.axisGradient[i] = ((f * rho + major * z) / distance) * s.radial[i];
        }
        const explicitMode = record.normalInnerParameterMode;
        if (explicitMode != null && explicitMode !== 'fixed' && explicitMode !== 'portal-side-boundary') throw new RangeError('Unknown normalInnerParameterMode');
        const fixedQuadrature = t === 0 || t === 0.25 || t === 0.5 || t === 0.75 || t === 1;
        boundaryMode = explicitMode === 'portal-side-boundary' ||
            (explicitMode == null && !fixedQuadrature && Math.abs(z + f) < 1e-8);
        diagnostics.parameterMode = boundaryMode ? (explicitMode ? 'portal-side-boundary' : 'inferred-portal-side-boundary') : 'fixed';
        diagnostics.reconstructedGap = distance - f;
    } else {
        s.normal.set(s.radial);
        for (let i = 0; i < 3; i++) { s.innerGradient[i] = -s.normal[i]; s.axisGradient[i] = 0; }
        diagnostics.parameterMode = 'portal-plane';
        diagnostics.reconstructedGap = finite(record.clearance, 'rim clearance') - rho;
    }
    const normalMismatch = Math.hypot(s.normal[0] - record.normal[0], s.normal[1] - record.normal[1], s.normal[2] - record.normal[2]);
    if (normalMismatch > 1e-7) {
        const error = new Error('Contact normal geometry changed; rebuild the collector before normal gradients');
        error.geometry = { normalMismatch, rho, radialEpsilon, kind: record.kind, innerSegment, outerSegment,
            innerT: t, normal: Array.from(record.normal), reconstructedNormal: Array.from(s.normal),
            inner0: Array.from(s.inner0), inner1: Array.from(s.inner1), outer0: Array.from(s.outer0), outer1: Array.from(s.outer1),
            gap: record.gap, reconstructedGap: diagnostics.reconstructedGap, filletRadius: constraint.portalFilletRadius };
        throw error;
    }
    if (boundaryMode) {
        const denominator = dot(s.wireDirection, s.axis);
        if (Math.abs(denominator) <= EPSILON) throw new RangeError('Implicit portal crossing is degenerate');
        const coefficient = dot(s.normal, s.wireDirection) / denominator;
        for (let i = 0; i < 3; i++) {
            s.innerGradient[i] += coefficient * s.axis[i];
            s.axisGradient[i] += coefficient * rho * s.radial[i];
        }
    }
    let count = 0;
    for (let axis = 0; axis < 3; axis++) {
        count = append(s.gradients, count, 0, innerSegment, axis, (1 - t) * s.innerGradient[axis]);
        count = append(s.gradients, count, 0, innerSegment + 1, axis, t * s.innerGradient[axis]);
        count = append(s.gradients, count, 1, outerSegment, axis, -s.axisGradient[axis] / length);
        count = append(s.gradients, count, 1, outerSegment + 1, axis, -s.innerGradient[axis] + s.axisGradient[axis] / length);
    }
    s.gradients.length = count;
    // Every gradient acts on endpoint translations, so verify the complete
    // orbital moment directly (no synthetic counter-moment metadata).
    for (const entry of s.gradients) {
        const body = entry.side === 0 ? inner : outer, node = Math.floor(entry.dof / 6), axis = entry.dof % 6;
        s.moment.fill(0); s.moment[axis] = entry.value;
        for (let i = 0; i < 3; i++) s.point[i] = body[XYZ[i]][node];
        cross(s.point, s.moment, s.offset);
        for (let i = 0; i < 3; i++) diagnostics.normalMomentResidual[i] += s.offset[i];
    }
    diagnostics.supported = true; diagnostics.reason = null; diagnostics.directionalAtAxis = directional;
    diagnostics.normalMismatch = normalMismatch;
    diagnostics.innerGradientMagnitude = Math.hypot(...s.innerGradient);
    record.normalGradients = s.gradients;
    return s.gradients;
}

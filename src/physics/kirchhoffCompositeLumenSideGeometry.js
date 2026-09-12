import { DEFAULT_LUMEN_QUADRATURE } from './kirchhoffLumenContact.js';
import { materialSegmentContactId } from './kirchhoffContactManifold.js';

// Match the existing detector's point-to-segment and radial-normal branches.
const DETECTOR_EPSILON = 1e-12;
const PORTAL_TOLERANCE = 1e-9;
const ROUND_OFF = 128 * Number.EPSILON;
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const close = (a, b) => Math.abs(a - b) <= ROUND_OFF * Math.max(1, Math.abs(a), Math.abs(b));
function nonnegative(value, name) {
    if (!Number.isFinite(value) || value < 0) throw new RangeError(`${name} must be finite and nonnegative`);
    return value;
}
function readVector(value, target, name) {
    if (value == null) throw new TypeError(`${name} is required`);
    for (let i = 0; i < 3; i++) {
        const v = Array.isArray(value) || ArrayBuffer.isView(value) ? value[i] : value[['x', 'y', 'z'][i]];
        if (!Number.isFinite(v)) throw new TypeError(`${name} needs three finite components`);
        target[i] = v;
    }
}
function weights(value, name) {
    if (!value || value.length !== 2 || !Array.from(value).every(Number.isFinite)) throw new TypeError(`${name} needs two finite entries`);
}
function rawWorkspace() {
    return { id: null, kind: null, feature: null, gap: NaN, clearance: NaN, radialDistance: NaN,
        innerT: NaN, outerT: NaN, normal: new Float64Array(3),
        innerWeights: new Float64Array(2), outerWeights: new Float64Array(2) };
}

/** One selected lumen-side row in physical coordinates:
 * [wire0.xyz, wire1.xyz, catheter0.xyz, catheter1.xyz]. Every numeric buffer is
 * owned and reused. Output is a branch derivative, never a winner/CCD proof.
 */
export function createCompositeLumenSideGeometryWorkspace() {
    return { dofCount: 12, dofs: Int32Array.from({ length: 12 }, (_, i) => i),
        supported: false, reason: 'not-evaluated', certified: false, queryCount: 0, hessianValid: false, witnessJacobianValid: false,
        derivativeScope: 'fixed-inner-quadrature-strict-interior-outer-side', selectionCertified: false,
        branchSignature: null, gap: NaN, clearance: NaN, radialDistance: NaN, innerT: NaN, outerT: NaN,
        positions: Array.from({ length: 4 }, () => new Float64Array(3)),
        innerPoint: new Float64Array(3), outerPoint: new Float64Array(3),
        outerDirection: new Float64Array(3), offsetFromOuterStart: new Float64Array(3), radial: new Float64Array(3),
        normal: new Float64Array(3), innerWeights: new Float64Array(2), outerWeights: new Float64Array(2),
        outerTGradient: new Float64Array(12), normalJacobian: new Float64Array(36),
        gapJacobian: new Float64Array(12), normalForceColumn: new Float64Array(12), forceColumn: new Float64Array(12),
        normalDerivative: new Float64Array(144), gapHessian: new Float64Array(144), rawContact: rawWorkspace() };
}
const geometryArrays = ['innerPoint', 'outerPoint', 'outerDirection', 'offsetFromOuterStart', 'radial', 'normal',
    'innerWeights', 'outerWeights', 'outerTGradient', 'normalJacobian', 'gapJacobian', 'normalForceColumn',
    'forceColumn', 'normalDerivative', 'gapHessian'];
const witnessArrays=geometryArrays.filter(key=>!['normalDerivative','gapHessian'].includes(key));
const gradientArrays=witnessArrays.filter(key=>!['outerTGradient','normalJacobian'].includes(key));
function unsupported(out, reason) {
    out.hessianValid = out.witnessJacobianValid = false;
    out.supported = false; out.reason = reason; out.certified = false; out.selectionCertified = false;
    out.branchSignature = null;
    out.gap = out.clearance = out.radialDistance = out.innerT = out.outerT = NaN;
    for (const key of geometryArrays) out[key].fill(NaN);
    out.positions.forEach(p => p.fill(NaN));
    return out;
}
function resetRaw(raw) {
    raw.id = raw.kind = raw.feature = null;
    raw.gap = raw.clearance = raw.radialDistance = raw.innerT = raw.outerT = NaN;
    raw.normal.fill(NaN); raw.innerWeights.fill(NaN); raw.outerWeights.fill(NaN);
}

/** Differentiate an ALREADY selected original detector record. `input` is the
 * same geometry/options object passed to evaluateKirchhoffLumenSegmentContact.
 * This helper performs zero queries, manifold updates or quadrature searches.
 * The caller owns changes of winner/sample/feature between runtime evaluations.
 *
 * g=clearance-|p-(C+t*(D-C))|, p=(1-s)*A+s*B, t=((p-C).(D-C))/|D-C|².
 * s is frozen; t and its weights are differentiated. The detector's normal
 * points outward from catheter axis to wire. Physical B points inward on the
 * wire and outward on the catheter: B=G^T, forceColumn=-B, DB=Hgap.
 * order:'gradient' keeps the exact validated gap and G/B; 'witness' also
 * computes d(normal)/dq and d(outerT)/dq for the surface friction operator.
 * Only 'full' computes DB/Hgap. Unavailable arrays remain NaN and their
 * validity flags are false, including after a previous full evaluation.
 */
export function differentiateCompositeLumenSideContact({ input, contact }, out, {order='full'}={}) {
    unsupported(out, 'not-evaluated'); resetRaw(out.rawContact); out.queryCount = 0;
    if(!['full','witness','gradient'].includes(order))throw new RangeError('Side geometry order must be full, witness or gradient');
    const full=order==='full',witness=order!=='gradient';
    if (!input || typeof input !== 'object') throw new TypeError('Original lumen detector input is required');
    if (!contact || typeof contact !== 'object') return unsupported(out, 'missing-original-contact');
    const raw = out.rawContact;
    raw.id = contact.id; raw.kind = contact.kind; raw.feature = contact.feature;
    if (contact.kind !== 'side') return unsupported(out, `unsupported-feature:${contact.kind}`);
    for (const key of ['gap', 'clearance', 'radialDistance', 'innerT', 'outerT']) {
        if (!Number.isFinite(contact[key])) return unsupported(out, `invalid-original-${key}`);
        raw[key] = contact[key];
    }
    // Read into local scratch first: any malformed input throws while every
    // public differential/value array remains invalid from the reset above.
    const points = [input.innerStart, input.innerEnd, input.outerStart, input.outerEnd].map((p, i) => {
        const v = [0, 0, 0]; readVector(p, v, ['innerStart', 'innerEnd', 'outerStart', 'outerEnd'][i]); return v;
    });
    readVector(contact.normal, raw.normal, 'original normal');
    weights(contact.innerWeights, 'original inner weights'); weights(contact.outerWeights, 'original outer weights');
    raw.innerWeights.set(contact.innerWeights); raw.outerWeights.set(contact.outerWeights);
    const lumenRadius = nonnegative(input.lumenRadius, 'lumenRadius'), innerRadius = nonnegative(input.innerRadius, 'innerRadius');
    const filletRadius = nonnegative(input.portalFilletRadius === undefined ? 0 : input.portalFilletRadius, 'portalFilletRadius');
    nonnegative(input.activationDistance === undefined ? 0 : input.activationDistance, 'activationDistance');
    const quadrature = input.quadrature === undefined ? DEFAULT_LUMEN_QUADRATURE : input.quadrature;
    if ((!Array.isArray(quadrature) && !ArrayBuffer.isView(quadrature)) || !quadrature.length ||
        !Array.from(quadrature).every(v => Number.isFinite(v) && v >= 0 && v <= 1))
        throw new RangeError('Original quadrature must contain coordinates in [0,1]');
    const featurePrefix = input.featurePrefix === undefined ? 'lumen' : input.featurePrefix;
    if (typeof featurePrefix !== 'string' || !featurePrefix.length) throw new TypeError('A nonempty original feature prefix is required');
    const feature = `${featurePrefix}:side`;
    if (raw.feature !== feature || raw.id !== materialSegmentContactId(input.innerMaterialSegmentId, input.outerMaterialSegmentId, feature))
        return unsupported(out, 'original-material-or-feature-mismatch');
    const s = raw.innerT;
    if (s < 0 || s > 1 || !Array.from(quadrature).some(v => v === s)) return unsupported(out, 'unrecognized-inner-quadrature-sample');
    const endpointPolicy=input.endpointDerivative==='clamped-one-sided';
    if (!(raw.outerT > 0 && raw.outerT < 1)&&!(endpointPolicy&&(raw.outerT===0||raw.outerT===1))) return unsupported(out, 'outer-endpoint-or-clamped-projection');
    if (!(raw.radialDistance > DETECTOR_EPSILON)) return unsupported(out, 'zero-or-fallback-radial-normal');
    const [A, W, C, D] = points;
    const p = A.map((v, i) => v + (W[i] - v) * s), d = D.map((v, i) => v - C[i]), v = p.map((x, i) => x - C[i]);
    const lengthSquared = dot(d, d);
    // pointToSegment uses its squared-length <= EPSILON fallback, even when
    // the outer segment passes the detector's separate length > EPSILON test.
    if (!(lengthSquared > DETECTOR_EPSILON) || !Number.isFinite(lengthSquared)) return unsupported(out, 'degenerate-detector-projection-branch');
    const rawT = dot(v, d) / lengthSquared,t=Math.max(0,Math.min(1,rawT)),clamped=rawT<=0||rawT>=1;
    if (!(t > 0 && t < 1)&&!(endpointPolicy&&rawT>=-PORTAL_TOLERANCE&&rawT<=1+PORTAL_TOLERANCE)) return unsupported(out, 'outer-endpoint-or-clamped-projection');
    const q = C.map((x, i) => x + d[i] * t), radial = p.map((x, i) => x - q[i]);
    const distance = Math.hypot(...radial), clearance = Math.max(0, lumenRadius - innerRadius), gap = clearance - distance;
    if (!(distance > DETECTOR_EPSILON) || !Number.isFinite(distance)) return unsupported(out, 'zero-or-fallback-radial-normal');
    const inverseDistance = 1 / distance, n = radial.map(x => x * inverseDistance);
    if (input.openDistal) {
        const outerLength = Math.hypot(...d), axis = d.map(x => x / outerLength);
        const distalAxial = dot(p.map((x, i) => x - D[i]), axis);
        if (distalAxial >= -Math.max(PORTAL_TOLERANCE, filletRadius)) return unsupported(out, 'distal-portal-or-fillet-owned-sample');
    }
    if (![t, distance, clearance, gap].every(Number.isFinite) || !close(raw.outerT, t) ||
        !close(raw.radialDistance, distance) || !close(raw.clearance, clearance) || !close(raw.gap, gap))
        return unsupported(out, 'original-contact-does-not-match-current-geometry');
    if (!close(dot(raw.normal, raw.normal), 1) || !n.every((x, i) => close(x, raw.normal[i])))
        return unsupported(out, 'original-normal-does-not-match-radial-direction');
    const iw = [1 - s, s], ow = [1 - t, t], coefficients = [-iw[0], -iw[1], ow[0], ow[1]];
    if (!iw.every((x, i) => close(x, raw.innerWeights[i])) || !ow.every((x, i) => close(x, raw.outerWeights[i])))
        return unsupported(out, 'original-contact-weights-mismatch');
    if (contact.gradients !== undefined) {
        const gradients = [...(contact.gradients?.inner ?? []), ...(contact.gradients?.outer ?? [])];
        if (gradients.length !== 4 || gradients.some((g, block) => !g || g.length !== 3 ||
            !Array.from(g).every((x, i) => Number.isFinite(x) && close(x, coefficients[block] * n[i]))))
            return unsupported(out, 'original-contact-gradients-mismatch');
    }
    out.gap = gap; out.clearance = clearance; out.radialDistance = distance; out.innerT = s; out.outerT = t;
    points.forEach((point, i) => out.positions[i].set(point));
    out.innerPoint.set(p); out.outerPoint.set(q); out.outerDirection.set(d); out.offsetFromOuterStart.set(v);
    out.radial.set(radial); out.normal.set(n); out.innerWeights.set(iw); out.outerWeights.set(ow);
    for (let i = 0; i < 12; i++) {
        const B = coefficients[Math.floor(i / 3)] * n[i % 3];
        out.gapJacobian[i] = B; out.normalForceColumn[i] = B; out.forceColumn[i] = -B;
    }
    if(witness)for (let j = 0; j < 12; j++) {
        const block = Math.floor(j / 3), axis = j % 3;
        const dp = block < 2 ? iw[block] : 0, dC = block === 2 ? 1 : 0, dd = block === 3 ? 1 : block === 2 ? -1 : 0;
        // At t==0/1 choose the clamped member of the point/segment
        // generalized derivative. The original detector still owns the
        // endpoint tolerance and rejects points beyond its side support.
        const dt = clamped?0:(d[axis] * (dp - dC) + (v[axis] - 2 * t * d[axis]) * dd) / lengthSquared;
        out.outerTGradient[j] = dt;
        const dr = [0, 1, 2].map(i => (i === axis ? dp - dC - t * dd : 0) - d[i] * dt);
        const radialPart = dot(n, dr);
        for (let i = 0; i < 3; i++) out.normalJacobian[12 * i + j] = (dr[i] - n[i] * radialPart) / distance;
        if(full)for (let i = 0; i < 12; i++) {
            const body = Math.floor(i / 3), a = i % 3;
            const dWeight = body === 2 ? -dt : body === 3 ? dt : 0;
            const value = coefficients[body] * out.normalJacobian[12 * a + j] + dWeight * n[a];
            out.normalDerivative[12 * i + j] = value; out.gapHessian[12 * i + j] = value;
        }
    }
    const required=full?geometryArrays:witness?witnessArrays:gradientArrays;
    if ([...required.map(key => out[key]), ...out.positions].some(array => !array.every(Number.isFinite)))
        return unsupported(out, 'nonfinite-side-differential');
    out.hessianValid=full;out.witnessJacobianValid=witness;
    out.supported = true; out.reason = null;
    out.branchSignature = `${raw.id}:innerT=${s}:${clamped?'clamped-one-sided-outer-endpoint':'strict-outer-side'}`;
    return out;
}

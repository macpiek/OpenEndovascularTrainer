import { DEFAULT_LUMEN_QUADRATURE } from './kirchhoffLumenContact.js';
import { materialSegmentContactId } from './kirchhoffContactManifold.js';

const N = 12, EPSILON = 1e-12, ROUND_OFF = 128 * Number.EPSILON;
const names = ['innerStart', 'innerEnd', 'outerStart', 'outerEnd'];
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
const sub = (a, b) => a.map((v, i) => v - b[i]);
const close = (a, b) => Math.abs(a - b) <= ROUND_OFF * Math.max(1, Math.abs(a), Math.abs(b));
const arenas = new WeakMap(), sources = new WeakMap();

// Reused second-order forward arena. No finite differences or detector calls
// occur in production. Hessians retain both halves for the subsequent DB.
function createArena() {
    const stride = 1 + N + N * N, data = new Float64Array(256 * stride);
    let used = 0;
    const constant = value => {
        const r = used; used += stride;
        if (used > data.length) throw new RangeError('Lumen tip derivative arena exceeded');
        data.fill(0, r, used); data[r] = value; return r;
    };
    const unary = (a, value, first, second) => {
        const r = constant(value);
        for (let i = 0; i < N; i++) data[r + 1 + i] = first * data[a + 1 + i];
        for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) {
            const k = 1 + N + N * i + j;
            data[r + k] = first * data[a + k] + second * data[a + 1 + i] * data[a + 1 + j];
        }
        return r;
    };
    const binary = (a, b, value, da, db, mixed = 0) => {
        const r = constant(value);
        for (let i = 0; i < N; i++) data[r + 1 + i] = da * data[a + 1 + i] + db * data[b + 1 + i];
        for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) {
            const k = 1 + N + N * i + j;
            data[r + k] = da * data[a + k] + db * data[b + k] + mixed *
                (data[a + 1 + i] * data[b + 1 + j] + data[b + 1 + i] * data[a + 1 + j]);
        }
        return r;
    };
    return { data, reset() { used = 0; }, constant,
        variable(value, i) { const r = constant(value); data[r + 1 + i] = 1; return r; },
        add: (a, b) => binary(a, b, data[a] + data[b], 1, 1),
        sub: (a, b) => binary(a, b, data[a] - data[b], 1, -1),
        mul: (a, b) => binary(a, b, data[a] * data[b], data[b], data[a], 1),
        scale: (a, b) => unary(a, data[a] * b, b, 0),
        reciprocal: a => unary(a, 1 / data[a], -1 / data[a] ** 2, 2 / data[a] ** 3),
        sqrt(a) { const v = Math.sqrt(data[a]); return unary(a, v, .5 / v, -.25 / v ** 3); }
    };
}
function readVector(v, label) {
    const result = ['x', 'y', 'z'].map((key, i) => Array.isArray(v) || ArrayBuffer.isView(v) ? v[i] : v?.[key]);
    if (!result.every(Number.isFinite)) throw new TypeError(`${label} requires three finite components`);
    return result;
}
function nonnegative(value, label) {
    if (!Number.isFinite(value) || value < 0) throw new RangeError(`${label} must be finite and nonnegative`);
    return value;
}
const vectorSizes = {
    innerPoint: 3, outerPoint: 3, axisPoint: 3, axis: 3, radial: 3, normal: 3, physicalNormal: 3,
    innerWeights: 2, outerWeights: 2, innerTGradient: N, normalJacobian: 3 * N,
    physicalNormalJacobian: 3 * N, forceScaleGradient: N,
    gapJacobian: N, normalForceColumn: N, forceColumn: N, normalDerivative: N * N, gapHessian: N * N
};
const scalarNames = ['gap', 'clearance', 'radialDistance', 'innerT', 'outerT', 'axial', 'forceScale'];
function invalid(out, reason) {
    out.supported = false; out.reason = reason; out.certified = out.selectionCertified = false;
    out.branchSignature = out.derivativeScope = null;
    for (const name of scalarNames) out[name] = NaN;
    for (const name of Object.keys(vectorSizes)) out[name].fill(NaN);
    out.positions.forEach(p => p.fill(NaN));
    return out;
}

/** Physical ordering [wire0.xyz, wire1.xyz, catheter0.xyz, catheter1.xyz].
 * Numeric output buffers are owned/reused. A selected smooth branch operator
 * is not a contact-selection, continuous-collision or timestep certificate.
 */
export function createCompositeLumenTipGeometryWorkspace() {
    const out = { dofCount: N, dofs: Int32Array.from({ length: N }, (_, i) => i),
        pointOrder: Object.freeze([...names]), positions: names.map(() => new Float64Array(3)),
        rawContact: null, queryCount: 0 };
    for (const [name, size] of Object.entries(vectorSizes)) out[name] = new Float64Array(size);
    arenas.set(out, createArena());
    return invalid(out, 'not-evaluated');
}

// Differentiate exactly the selected original scalar formula, including the
// catheter axis rotation and, for rim, the moving plane-crossing fraction.
function derivatives(arena, points, kind, s, f, clearance,clampedRim=false) {
    arena.reset();
    const a = arena, plus = (u, v) => u.map((x, i) => a.add(x, v[i]));
    const minus = (u, v) => u.map((x, i) => a.sub(x, v[i]));
    const times = (u, t) => u.map(x => a.mul(x, t));
    const product = (u, v) => u.reduce((sum, x, i) => a.add(sum, a.mul(x, v[i])), a.constant(0));
    const length = v => a.sqrt(product(v, v));
    const [A, W, C, D] = points.map((p, block) => p.map((v, i) => a.variable(v, 3 * block + i)));
    const d = minus(D, C), e = times(d, a.reciprocal(length(d))), w = minus(W, A);
    let t = a.constant(s);
    if (kind === 'distal-rim'&&!clampedRim) {
        const za = product(minus(A, D), e), zw = product(minus(W, D), e);
        t = a.mul(a.scale(za, -1), a.reciprocal(a.sub(zw, za)));
    }
    const p = plus(A, times(w, t)), x = minus(p, D), z = product(x, e);
    const radial = minus(x, times(e, z)), r = length(radial), n = times(radial, a.reciprocal(r));
    let gap, normal = n;
    if (kind === 'distal-fillet') {
        const ca = a.add(z, a.constant(f)), cr = a.sub(r, a.constant(clearance + f));
        const h = a.sqrt(a.add(a.mul(ca, ca), a.mul(cr, cr))), inverse = a.reciprocal(h);
        gap = a.sub(h, a.constant(f));
        normal = times(plus(times(e, ca), times(n, cr)), a.scale(inverse, -1));
    } else gap = a.sub(a.constant(clearance + f), r);
    return { gap, t, normal };
}

/** `input` and `contact` are the caller's original detector arguments/record.
 * No query, manifold update, sample search or clamp is performed here.
 * Fresh raw records are revalidated against the full current geometry. Once
 * accepted, a raw record's geometry/options/data must remain unchanged; a
 * caller must obtain a fresh record after motion. This binds on first use,
 * not at detector creation (the unchanged detector has no source stamp).
 *
 * Fillet: g=hypot(z+f,r-(clearance+f))-f, fixed quadrature s. B=G, DB=Hgap.
 * Rim: g=clearance+f-r at strict moving plane crossing s. Let v=G_A+G_W,
 * m=|v|. B=G/m gives unit wire resultant perpendicular to its tangent.
 * DB=Hgap/m-G*(Dm)^T/m², generally nonsymmetric. Thus physical virtual
 * work is Fn*B.dx=(Fn/m)*dg; forceColumn=-B is the residual convention.
 */
export function differentiateCompositeLumenTipContact({ input, contact }, out) {
    const arena = arenas.get(out);
    if (!arena) throw new TypeError('Use a prepared lumen tip geometry workspace');
    invalid(out, 'not-evaluated'); out.rawContact = null; out.queryCount = 0;
    if (!input || typeof input !== 'object') throw new TypeError('Original lumen detector input is required');
    if (!contact || typeof contact !== 'object') return invalid(out, 'missing-original-contact');
    const kind = contact.kind;
    if (!['distal-fillet', 'distal-rim'].includes(kind)) return invalid(out, `unsupported-feature:${kind}`);
    const points = names.map(name => readVector(input[name], name));
    const lumen = nonnegative(input.lumenRadius, 'lumenRadius'), inner = nonnegative(input.innerRadius, 'innerRadius');
    const f = nonnegative(input.portalFilletRadius === undefined ? 0 : input.portalFilletRadius, 'portalFilletRadius');
    const activation = nonnegative(input.activationDistance === undefined ? 0 : input.activationDistance, 'activationDistance');
    const quadrature = input.quadrature === undefined ? DEFAULT_LUMEN_QUADRATURE : input.quadrature;
    if ((!Array.isArray(quadrature) && !ArrayBuffer.isView(quadrature)) || !quadrature.length ||
        !Array.from(quadrature).every(v => Number.isFinite(v) && v >= 0 && v <= 1))
        throw new RangeError('Original quadrature must contain coordinates in [0,1]');
    const prefix = input.featurePrefix === undefined ? 'lumen' : input.featurePrefix;
    if (typeof prefix !== 'string' || !prefix.length) throw new TypeError('A nonempty original feature prefix is required');
    const feature = `${prefix}:${kind}`, id = materialSegmentContactId(input.innerMaterialSegmentId, input.outerMaterialSegmentId, feature);
    if (contact.id !== id || contact.feature !== feature) return invalid(out, 'original-material-or-feature-mismatch');
    if (!input.openDistal) return invalid(out, 'closed-distal-portal');
    const raw = { id: contact.id, kind, feature: contact.feature };
    for (const key of ['gap', 'clearance', 'radialDistance', 'innerT', 'outerT']) {
        if (!Number.isFinite(contact[key])) return invalid(out, `invalid-original-${key}`);
        raw[key] = contact[key];
    }
    raw.normal = readVector(contact.normal, 'original normal');
    for (const key of ['innerWeights', 'outerWeights']) {
        if (contact[key]?.length !== 2 || !Array.from(contact[key]).every(Number.isFinite))
            return invalid(out, 'invalid-original-weights');
        raw[key] = Array.from(contact[key]);
    }
    if (contact.gradients !== undefined) {
        const gradients = [...(contact.gradients?.inner ?? []), ...(contact.gradients?.outer ?? [])];
        if (gradients.length !== 4 || gradients.some(v => v?.length !== 3 || !Array.from(v).every(Number.isFinite)))
            return invalid(out, 'invalid-original-gradients');
        raw.gradients = gradients.map(v => Array.from(v));
    }
    if (contact.active !== undefined && contact.active !== (raw.gap < 0)) return invalid(out, 'original-active-mismatch');
    if (contact.violation !== undefined && !close(contact.violation, Math.max(0, -raw.gap))) return invalid(out, 'original-violation-mismatch');
    raw.active = contact.active; raw.violation = contact.violation;
    // Exact snapshot catches even value-consistent edits (e.g. a rigid world
    // translation with a stale raw record). External manifold history is not
    // geometry provenance and is intentionally neither copied nor inspected.
    const stamp = JSON.stringify({ points, lumen, inner, f, activation, openDistal: input.openDistal,
        quadrature: Array.from(quadrature), id, raw });
    if (sources.has(contact) && sources.get(contact) !== stamp) return invalid(out, 'original-source-mutated');
    out.rawContact = raw;
    const [A, W, C, D] = points, d = sub(D, C), length = Math.hypot(...d), clearance = Math.max(0, lumen - inner);
    if (!(length > EPSILON) || !Number.isFinite(length)) return invalid(out, 'degenerate-outer-axis');
    const e = d.map(v => v / length), w = sub(W, A);
    let s = raw.innerT,clampedRim=false;const endpointPolicy=input.endpointDerivative==='clamped-one-sided';
    if (kind === 'distal-fillet') {
        if (!(f > EPSILON)) return invalid(out, 'missing-smooth-fillet');
        if (s < 0 || s > 1 || !Array.from(quadrature).some(v => v === s)) return invalid(out, 'unrecognized-inner-quadrature-sample');
    } else {
        const za = dot(sub(A, D), e), zw = dot(sub(W, D), e), delta = zw - za;
        if (!(Math.abs(delta) > EPSILON)) return invalid(out, 'degenerate-rim-crossing');
        s = -za / delta;
        if (!(s > 0 && s < 1) || !(Math.min(za, zw) < 0 && Math.max(za, zw) > 0)) {
            if(!endpointPolicy||Math.min(za,zw)>1e-9||Math.max(za,zw)<-1e-9)return invalid(out, 'rim-endpoint-or-clamped-crossing');
            s=Math.max(0,Math.min(1,s));clampedRim=true;
        }
        if (!(raw.gap <= activation)) return invalid(out, 'original-rim-outside-activation');
    }
    const p = A.map((v, i) => v + s * w[i]), x = sub(p, D), z = dot(x, e);
    const radial = x.map((v, i) => v - z * e[i]), r = Math.hypot(...radial);
    if (!(r > EPSILON) || !Number.isFinite(r)) return invalid(out, 'zero-or-fallback-radial-normal');
    const n = radial.map(v => v / r), major = clearance + f;
    let gap = major - r, expectedNormal = n, rawClearance = major;
    if (kind === 'distal-fillet') {
        if (!(z > -f && z < f && r < major)&&!(endpointPolicy&&z>=-f&&z<=f&&r<=major)) return invalid(out, 'fillet-feature-boundary-or-outside');
        const ca = z + f, cr = r - major, h = Math.hypot(ca, cr);
        if (!(h > EPSILON)) return invalid(out, 'degenerate-fillet-circle');
        gap = h - f; rawClearance = f;
        expectedNormal = e.map((v, i) => -(v * ca / h + n[i] * cr / h));
    }
    if (![gap, r, s, rawClearance].every(Number.isFinite) || !close(raw.gap, gap) || !close(raw.radialDistance, r) ||
        !close(raw.innerT, s) || raw.outerT !== 1 || !close(raw.clearance, rawClearance))
        return invalid(out, 'original-contact-does-not-match-current-geometry');
    if (!expectedNormal.every((v, i) => close(v, raw.normal[i])) || !close(dot(raw.normal, raw.normal), 1))
        return invalid(out, 'original-normal-mismatch');
    const iw = [1 - s, s], ow = [0, 1], coefficients = [-iw[0], -iw[1], 0, 1];
    if (!iw.every((v, i) => close(v, raw.innerWeights[i])) || !ow.every((v, i) => v === raw.outerWeights[i]))
        return invalid(out, 'original-contact-weights-mismatch');
    // Validate old detector gradients only as raw data; they are NOT the
    // physical reactions returned below (fillet omits the axis lever pair).
    if (raw.gradients?.some((g, block) => g.some((v, i) => !close(v, coefficients[block] * raw.normal[i]))))
        return invalid(out, 'original-contact-gradients-mismatch');
    const jets = derivatives(arena, points, kind, s, f, clearance,clampedRim), data = arena.data;
    if (!close(data[jets.gap], raw.gap)) return invalid(out, 'nonfinite-or-inconsistent-tip-differential');
    const G = out.gapJacobian, H = out.gapHessian;
    for (let i = 0; i < N; i++) {
        G[i] = data[jets.gap + 1 + i]; out.innerTGradient[i] = data[jets.t + 1 + i];
        for (let j = 0; j < N; j++) H[N * i + j] = data[jets.gap + 1 + N + N * i + j];
    }
    const v = [0, 1, 2].map(i => G[i] + G[3 + i]);
    const m = kind === 'distal-rim' ? Math.hypot(...v) : 1;
    if (!(m > EPSILON) || !Number.isFinite(m)) return invalid(out, 'degenerate-normal-force-scale');
    out.forceScaleGradient.fill(0);
    if (kind === 'distal-rim') for (let j = 0; j < N; j++)
        out.forceScaleGradient[j] = v.reduce((sum, value, i) => sum + value * (H[N * i + j] + H[N * (3 + i) + j]), 0) / m;
    for (let i = 0; i < N; i++) {
        out.normalForceColumn[i] = G[i] / m; out.forceColumn[i] = -out.normalForceColumn[i];
        for (let j = 0; j < N; j++) out.normalDerivative[N * i + j] =
            H[N * i + j] / m - G[i] * out.forceScaleGradient[j] / (m * m);
    }
    for (let i = 0; i < 3; i++) {
        out.physicalNormal[i] = v[i] / m;
        for (let j = 0; j < N; j++) {
            out.normalJacobian[N * i + j] = data[jets.normal[i] + 1 + j];
            out.physicalNormalJacobian[N * i + j] = out.normalDerivative[N * i + j] + out.normalDerivative[N * (3 + i) + j];
        }
    }
    out.gap = raw.gap; out.clearance = raw.clearance; out.radialDistance = raw.radialDistance;
    out.innerT = s; out.outerT = 1; out.axial = z; out.forceScale = m;
    points.forEach((v, i) => out.positions[i].set(v));
    out.innerPoint.set(p); out.outerPoint.set(D); out.axisPoint.set(D.map((v, i) => v + z * e[i]));
    out.axis.set(e); out.radial.set(radial); out.normal.set(raw.normal); out.innerWeights.set(iw); out.outerWeights.set(ow);
    if (Object.keys(vectorSizes).some(key => !out[key].every(Number.isFinite)) || !scalarNames.every(key => Number.isFinite(out[key])))
        return invalid(out, 'nonfinite-tip-differential');
    out.derivativeScope = kind === 'distal-fillet' ? 'fixed-inner-quadrature-smooth-distal-torus' : 'strict-interior-moving-distal-plane-crossing';
    out.branchSignature = `${id}:${kind === 'distal-fillet' ? `innerT=${s}:smooth-torus` : 'strict-moving-crossing'}`;
    out.supported = true; out.reason = null;
    sources.set(contact, stamp);
    return out;
}

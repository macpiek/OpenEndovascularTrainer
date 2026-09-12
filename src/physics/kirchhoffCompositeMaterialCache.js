import { compileCompositeMaterial } from './kirchhoffCompositeElement.js';
import { compositeToolFromTipProfile } from './kirchhoffCompositeTopology.js';
import { KIRCHHOFF_MATERIAL_PROFILES } from './kirchhoffMaterialProfile.js';
import { GUIDEWIRE_TIP_CORE_LENGTH_MM, GUIDEWIRE_SOFT_TIP_LENGTH_MM } from './guidewireMaterialProfile.js';
import { BERENSTEIN_STRAIGHT_TIP_LENGTH_MM, BERENSTEIN_CURVATURE_TRANSITION_MM,
    BERENSTEIN_TIP_SHAPE_LENGTH_MM } from './catheterMaterialProfile.js';

const GAUSS6 = [
    [-.932469514203152, .1713244923791704], [-.6612093864662645, .3607615730481386],
    [-.2386191860831969, .4679139345726910], [.2386191860831969, .4679139345726910],
    [.6612093864662645, .3607615730481386], [.932469514203152, .1713244923791704]
];
const roundoffRelative = 512 * Number.EPSILON;
const frozenMaterial = raw => {
    const m = compileCompositeMaterial(raw);
    if (m.energyOffset < 0) throw new RangeError('material energyOffset must be nonnegative');
    return Object.freeze({ stiffness: Object.freeze(Array.from(m.stiffness)),
        intrinsic: Object.freeze(Array.from(m.intrinsic)), energyOffset: m.energyOffset });
};
const label = (tool, x) => x === tool.end ? tool.materialInterval[1] : x === tool.start ? tool.materialInterval[0]
    : tool.materialInterval[1] + tool.dsDx * (x - tool.insertion);
const dotK = (k, v) => v[0] * (k[0] * v[0] + k[1] * v[1] + k[2] * v[2]) +
    v[1] * (k[3] * v[0] + k[4] * v[1] + k[5] * v[2]) +
    v[2] * (k[6] * v[0] + k[7] * v[1] + k[8] * v[2]);
const finite = (value, name) => {
    if (!Number.isFinite(value)) throw new TypeError(`${name} must be finite`);
    return value;
};

/** Immutable profile adapters are created ONCE, outside a feed loop. Only the
 * exact built-in object identity supplies proven constant/polynomial spans.
 * An id, matching samples or a caller's mutable function never proves this.
 * Each adapter owns a bounded exact-support cache. Unknown profiles use Mesh's
 * original adaptive quadrature on every call. No global profile cache grows.
 */
export function createKirchhoffCompositeMaterialCache({ maximumEntriesPerProfile = 512 } = {}) {
    if (!Number.isInteger(maximumEntriesPerProfile) || maximumEntriesPerProfile < 0)
        throw new RangeError('maximumEntriesPerProfile must be a nonnegative integer');
    let metadata = new WeakMap();
    const statistics = { constantCells: 0, polynomialCells: 0, adaptiveProfileCells: 0, cacheHits: 0, fallbackCells: 0,
        freshEvaluations: 0, evictions: 0, entries: 0 };
    function storeRecord(material, record) {
        metadata.set(material, { ...record, cells: new Map(), revision: Object.freeze({}) });
    }
    function profileTool({ profile, tipBreakpoints = [], ...tool }) {
        const length = tool.materialInterval[1] - tool.materialInterval[0];
        const sTip = tool.materialInterval[1];
        let kind = null, distances = [], constants = [];
        if (profile === KIRCHHOFF_MATERIAL_PROFILES.glidewire) {
            kind = 'glidewire'; distances = [GUIDEWIRE_TIP_CORE_LENGTH_MM, GUIDEWIRE_SOFT_TIP_LENGTH_MM];
            constants = [[0, GUIDEWIRE_TIP_CORE_LENGTH_MM], [GUIDEWIRE_SOFT_TIP_LENGTH_MM, length]];
        } else if (profile === KIRCHHOFF_MATERIAL_PROFILES.berenstein) {
            kind = 'berenstein-polynomial-5';
            const a = BERENSTEIN_STRAIGHT_TIP_LENGTH_MM, b = BERENSTEIN_TIP_SHAPE_LENGTH_MM,
                ramp = BERENSTEIN_CURVATURE_TRANSITION_MM;
            distances = [a, a + ramp, b - ramp, b];
            constants = [[0, a], [a + ramp, b - ramp], [b, length]];
        }
        const result = compositeToolFromTipProfile({ ...tool, profile,
            tipBreakpoints: [...new Set([...tipBreakpoints, ...distances.filter(d => d <= length)])] });
        if (kind) storeRecord(result.material, { kind, profile,
            // These intervals follow the branches of the immutable built-in
            // samplers, including endpoint continuity. They are not inferred.
            spans: constants.filter(([a, b]) => b > a && a < length).map(([a, b]) => ({
                start: sTip - Math.min(length, b), end: sTip - a,
                material: frozenMaterial(profile.sample(a))
            })), breaks: distances.filter(d => d > 0 && d < length).map(d => sTip - d).sort((a, b) => a - b),
            sTip });
        return result;
    }

    /** Explicit piecewise-constant metadata becomes a private copied sampler.
     * Every jump is returned for topology insertion. This also supports full
     * anisotropic tensors and nonzero intrinsic/mismatch density without any
     * assertion about an external mutable sampler's constancy.
     */
    function constantTool({ spans, ...tool }) {
        const [a, b] = tool.materialInterval;
        const copied = spans.map(span => ({ start: finite(span.start, 'span start'),
            end: finite(span.end, 'span end'), material: frozenMaterial(span.material) }));
        if (!copied.length || copied[0].start !== a || copied.at(-1).end !== b ||
            copied.some((s, i) => s.end <= s.start || i && s.start !== copied[i - 1].end))
            throw new RangeError('Constant spans must exactly partition materialInterval');
        const material = (s, context) => {
            const span = copied.find(span => s >= span.start && s <= span.end &&
                !(s === span.start && s !== a && context?.side === 'left') &&
                !(s === span.end && s !== b && context?.side !== 'left'));
            if (!span) throw new RangeError('Material label lies outside declared constant spans');
            const m = span.material;
            return { stiffness: [m.stiffness.slice(0, 3), m.stiffness.slice(3, 6), m.stiffness.slice(6, 9)],
                intrinsic: m.intrinsic, energyOffset: m.energyOffset };
        };
        storeRecord(material, { kind: 'constant', spans: copied, breaks: copied.slice(1).map(s => s.start) });
        return { ...tool, material, materialBreakpoints: [...new Set([
            ...(tool.materialBreakpoints ?? []), ...copied.slice(1).map(s => s.start)])] };
    }

    function integrator(topology, id, start, end, nominalLength, dsDx, options, fallback) {
        const input = topology.tools.find(tool => tool.id === id), record = metadata.get(input?.material);
        if (!record) {
            statistics.fallbackCells++;
            const cell = fallback(topology, id, start, end, nominalLength, dsDx, options);
            statistics.freshEvaluations += cell.quadrature.evaluations;
            return cell;
        }
        const pieces = [];
        for (let sectionIndex = 0; sectionIndex < topology.sections.length; sectionIndex++) {
            const section = topology.sections[sectionIndex], a = Math.max(start, section.start), b = Math.min(end, section.end);
            if (a >= b) continue;
            const source = section.tools.find(tool => tool.id === id);
            if (!source) continue;
            if (source.material !== input.material || source.dsDx !== dsDx)
                throw new RangeError('Cached material needs one immutable profile and dsDx over its support');
            pieces.push({ start: a, end: b, sectionIndex });
        }
        if (!pieces.length || pieces[0].start !== start || pieces.at(-1).end !== end ||
            pieces.some((p, i) => i && p.start !== pieces[i - 1].end))
            throw new RangeError(`Material support for ${id} is not continuously covered`);
        const sStart = label(input, start), sEnd = label(input, end);
        const span = record.spans.find(span => sStart >= span.start && sEnd <= span.end);
        // Below the conservative arithmetic floor use the original algorithm,
        // which retains its own explicit convergence/failure contract.
        const toleranceAllowsClosedRule = options.relativeTolerance >= roundoffRelative;
        const constant = span && toleranceAllowsClosedRule;
        // A constant span already owns ONE compiled material: no per-feed
        // support cache entry or large string key is needed. Nonconstant
        // identity/revision lives in record; encode every mapping, explicit
        // jump, quadrature split and error-budget input without rounding.
        const key = constant ? null : [id, start, end, nominalLength, dsDx, input.insertion, ...input.materialInterval,
            ...input.materialBreakpoints, '|', ...pieces.flatMap(p => [p.start, p.end]), '|',
            options.absoluteTolerance, options.relativeTolerance, options.maxDepth].join(',');
        const hit = key === null ? null : record.cells.get(key);
        if (hit) {
            // Preserve frequently reused transition supports during long feed
            // sequences while bounding history from moving boundary cells.
            record.cells.delete(key); record.cells.set(key, hit);
            statistics.cacheHits++;
            return { ...hit, pieces, quadrature: { ...hit.quadrature, evaluations: 0, cacheHit: true } };
        }
        let cell;
        if (constant) {
            const ratio = (end - start) / nominalLength, m = span.material;
            const material = ratio === 1 ? m : frozenMaterial({
                stiffness: [m.stiffness.slice(0, 3), m.stiffness.slice(3, 6), m.stiffness.slice(6, 9)].map(row => row.map(v => v * ratio)),
                intrinsic: m.intrinsic, energyOffset: m.energyOffset * ratio });
            const materialLength = nominalLength * dsDx;
            cell = { material, id, start, end, nominalLength, materialLength,
                energyAtZeroStrain: materialLength * (.5 * dotK(material.stiffness, material.intrinsic) + material.energyOffset),
                integratedMaterialLength: dsDx * (end - start), pieces,
                quadrature: { evaluations: 0, leaves: pieces.length, maximumDepth: 0, maximumEstimatedComponentError: 0,
                    converged: true, certified: false, rule: 'declared-constant', exactForDeclaredProfile: true,
                    roundoffRelative } };
            statistics.constantCells++;
        } else if (record.kind === 'berenstein-polynomial-5' && toleranceAllowsClosedRule) {
            cell = polynomialCell(record, input, id, start, end, nominalLength, dsDx, pieces);
            statistics.polynomialCells++; statistics.freshEvaluations += cell.quadrature.evaluations;
        } else if (record.kind === 'glidewire') {
            cell = glidewireCell(record, input, id, start, end, nominalLength, dsDx, pieces, options);
            statistics.adaptiveProfileCells++; statistics.freshEvaluations += cell.quadrature.evaluations;
        } else {
            statistics.fallbackCells++;
            cell = fallback(topology, id, start, end, nominalLength, dsDx, options);
            statistics.freshEvaluations += cell.quadrature.evaluations;
            // Do not expose a mutable typed view into the private support cache.
            cell.material = Object.freeze({ stiffness: Object.freeze(Array.from(cell.material.stiffness)),
                intrinsic: Object.freeze(Array.from(cell.material.intrinsic)), energyOffset: cell.material.energyOffset });
        }
        if (key !== null && maximumEntriesPerProfile) {
            if (record.cells.size >= maximumEntriesPerProfile) {
                record.cells.delete(record.cells.keys().next().value); statistics.evictions++; statistics.entries--;
            }
            record.cells.set(key, { ...cell, pieces: null, quadrature: { ...cell.quadrature } }); statistics.entries++;
        }
        return cell;
    }
    return { profileTool, constantTool, integrator, statistics,
        clear() { metadata = new WeakMap(); Object.keys(statistics).forEach(k => { statistics[k] = 0; }); } };
}

function glidewireCell(record, input, id, start, end, nominalLength, dsDx, pieces, options) {
    // Same adaptive Simpson error estimate and positive Boole accepted value
    // as Mesh.integrateMaterial. Known zero off-diagonal/intrinsic components
    // do not need arrays, recompilation, or a second identically-zero integral.
    const statistics = { evaluations: 0, leaves: 0, maximumDepth: 0, maximumEstimatedComponentError: 0,
        converged: true, certified: false, rule: 'profile-adaptive-simpson-positive-boole' };
    const integral = [0, 0, 0];
    for (const piece of pieces) {
        const cache = new Map();
        const sample = x => {
            if (!cache.has(x)) {
                const m = record.profile.sample(record.sTip - label(input, x));
                cache.set(x, [m.EI1 * dsDx, m.EI2 * dsDx, m.GJ * dsDx]); statistics.evaluations++;
            }
            return cache.get(x);
        };
        const integrate = (a, b, depth = 0) => {
            const middle = a + (b - a) / 2, q1 = a + (middle - a) / 2, q3 = middle + (b - middle) / 2;
            if (!(a < q1 && q1 < middle && middle < q3 && q3 < b))
                throw new RangeError('Material quadrature interval needs representable interior samples');
            const p = [sample(a), sample(q1), sample(middle), sample(q3), sample(b)], h = b - a;
            const fine = [0, 1, 2].map(i => h * (p[0][i] + 4 * p[1][i] + 2 * p[2][i] + 4 * p[3][i] + p[4][i]) / 12);
            const coarse = [0, 1, 2].map(i => h * (p[0][i] + 4 * p[2][i] + p[4][i]) / 6);
            const errors = fine.map((v, i) => Math.abs(v - coarse[i]) / 15);
            if (errors.some((error, i) => error > options.absoluteTolerance * h / (end - start) +
                options.relativeTolerance * Math.max(Math.abs(fine[i]), Math.abs(coarse[i])))) {
                if (depth >= options.maxDepth) throw new RangeError(`Material quadrature did not converge on [${a},${b}]`);
                const left = integrate(a, middle, depth + 1), right = integrate(middle, b, depth + 1);
                return left.map((v, i) => v + right[i]);
            }
            statistics.leaves++; statistics.maximumDepth = Math.max(statistics.maximumDepth, depth);
            statistics.maximumEstimatedComponentError = Math.max(statistics.maximumEstimatedComponentError, ...errors);
            return [0, 1, 2].map(i => h * (7 * p[0][i] + 32 * p[1][i] + 12 * p[2][i] + 32 * p[3][i] + 7 * p[4][i]) / 90);
        };
        const values = integrate(piece.start, piece.end);
        for (let i = 0; i < 3; i++) integral[i] += values[i];
    }
    const materialLength = nominalLength * dsDx, material = frozenMaterial({
        EI1: integral[0] / materialLength, EI2: integral[1] / materialLength, GJ: integral[2] / materialLength });
    return { material, id, start, end, nominalLength, materialLength, energyAtZeroStrain: 0,
        integratedMaterialLength: dsDx * (end - start), pieces, quadrature: statistics };
}

function polynomialCell(record, input, id, start, end, nominalLength, dsDx, pieces) {
    // Berenstein: constant diagonal K, piecewise degree-5 intrinsic curvature;
    // K*kappa and the positive mismatch polynomial have degrees 5 and 10.
    // Six positive Gauss points integrate both exactly in real arithmetic.
    const points = new Set([start, end]);
    for (const p of pieces) { points.add(p.start); points.add(p.end); }
    for (const s of record.breaks) {
        const x = input.insertion + (s - input.materialInterval[1]) / dsDx;
        if (x > start && x < end) points.add(x);
    }
    const bounds = [...points].sort((a, b) => a - b), samples = [], anchorRaw = record.profile.sample(record.sTip - label(input, start));
    const anchor = anchorRaw.kappa01, weightTotal = dsDx * (end - start), materialLength = dsDx * nominalLength;
    let relativeIntegral = 0;
    for (let i = 0; i + 1 < bounds.length; i++) {
        const a = bounds[i], b = bounds[i + 1], midpoint = a + (b - a) / 2, half = (b - a) / 2;
        for (const [point, gaussWeight] of GAUSS6) {
            const x = midpoint + half * point;
            if (!(x > a && x < b)) throw new RangeError('Polynomial material quadrature needs representable interior samples');
            const m = record.profile.sample(record.sTip - label(input, x)), weight = gaussWeight * half * dsDx;
            relativeIntegral += weight * (m.kappa01 - anchor); samples.push([weight, m.kappa01]);
        }
    }
    const intrinsic = [anchor + relativeIntegral / weightTotal, 0, 0];
    let mismatch = 0;
    for (const [weight, curvature] of samples) mismatch += weight * .5 * anchorRaw.EI1 * (curvature - intrinsic[0]) ** 2;
    const ratio = weightTotal / materialLength;
    const material = frozenMaterial({ EI1: anchorRaw.EI1 * ratio, EI2: anchorRaw.EI2 * ratio, GJ: anchorRaw.GJ * ratio,
        intrinsic, energyOffset: mismatch / materialLength });
    return { material, id, start, end, nominalLength, materialLength,
        energyAtZeroStrain: materialLength * (.5 * dotK(material.stiffness, material.intrinsic) + material.energyOffset),
        integratedMaterialLength: weightTotal, pieces,
        quadrature: { evaluations: samples.length + 1, leaves: bounds.length - 1, maximumDepth: 0,
            maximumEstimatedComponentError: 0, converged: true, certified: false,
            rule: 'declared-polynomial-gauss-6', exactForDeclaredProfile: true, roundoffRelative } };
}

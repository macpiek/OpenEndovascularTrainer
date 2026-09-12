/** Coverage, error-controlled spatial sampling and conservative reduction gates. */

function finite(value, name) {
    if (!Number.isFinite(value)) throw new TypeError(`${name} must be finite`);
    return value;
}

function nonnegative(value, name) {
    finite(value, name);
    if (value < 0) throw new RangeError(`${name} must be nonnegative`);
    return value;
}

function interval(value, name) {
    if (!value || value.length !== 2) throw new TypeError(`${name} needs two endpoints`);
    const a = finite(value[0], `${name}[0]`);
    const b = finite(value[1], `${name}[1]`);
    if (!(b > a)) throw new RangeError(`${name} must have positive length`);
    return [a, b];
}

function tolerance(value, name) {
    if (!value) throw new TypeError(`Explicit ${name} tolerance is required`);
    const absolute = nonnegative(value.absolute ?? 0, `${name}.absolute`);
    const relative = nonnegative(value.relative ?? 0, `${name}.relative`);
    return { absolute, relative };
}

const ratio = (error, scale, tol) => {
    const allowed = tol.absolute + tol.relative * scale;
    return allowed > 0 ? error / allowed : error === 0 ? 0 : Infinity;
};
const norm = a => Math.hypot(...a);
const distance = (a, b) => norm(a.map((v, i) => v - b[i]));

function materialMap(tool, start, end) {
    return {
        id: tool.id, sStart: tool.offset + tool.scale * start,
        sEnd: tool.offset + tool.scale * end,
        sMid: tool.offset + tool.scale * (start + end) / 2, dsDx: tool.scale
    };
}

/**
 * Exact breakpoints for affine maps s_i(x) = offset_i + scale_i*x, scale_i > 0.
 * Moving tips and material interfaces stay discontinuous, never blended.
 * Additional boundaries may include contact changes or geometric corners.
 */
export function partitionBundleCoverage({ interval: domain, tools, boundaries = [] }) {
    const [start, end] = interval(domain, 'interval');
    if (!Array.isArray(tools)) throw new TypeError('tools must be an array');
    const ids = new Set();
    const points = new Set([start, end]);
    const addBoundary = point => {
        finite(point, 'boundary');
        if (point > start && point < end) points.add(point);
    };
    boundaries.forEach(addBoundary);
    const normalized = tools.map(tool => {
        if (typeof tool.id !== 'string' || !tool.id || ids.has(tool.id)) {
            throw new TypeError('Tool ids must be unique nonempty strings');
        }
        ids.add(tool.id);
        const [a, b] = interval(tool.interval, `${tool.id}.interval`);
        const offset = finite(tool.materialCoordinate?.offset ?? 0, 'material offset');
        const scale = finite(tool.materialCoordinate?.scale ?? 1, 'material scale');
        if (!(scale > 0)) throw new RangeError('Material scale must be positive');
        addBoundary(a);
        addBoundary(b);
        for (const s of tool.materialBreakpoints ?? []) {
            finite(s, 'material breakpoint');
            const x = (s - offset) / scale;
            if (x > a && x < b) addBoundary(x);
        }
        return { id: tool.id, start: a, end: b, offset, scale };
    });
    const sorted = [...points].sort((a, b) => a - b);
    return sorted.slice(1).map((end, i) => {
        const start = sorted[i];
        const midpoint = start + (end - start) / 2;
        const active = normalized.filter(tool => midpoint >= tool.start && midpoint < tool.end);
        return {
            start, end, toolIds: active.map(tool => tool.id), tools: active,
            materials: active.map(tool => materialMap(tool, start, end))
        };
    });
}

function sampleVector(value, name) {
    const result = typeof value === 'number' ? [value] : value;
    if (!result || !Number.isInteger(result.length) || !result.length) {
        throw new TypeError(`${name} must be a finite scalar or nonempty vector`);
    }
    return Array.from(result, (entry, i) => finite(entry, `${name}[${i}]`));
}

/**
 * Linear-field interpolation error at quarter points + embedded Simpson energy
 * quadrature. Unknown narrow features can alias these probes: supply mandatory
 * boundaries and estimateInterval() with certified:true and a conservative
 * normalizedError bound for a certified mesh. Resource limits never imply success.
 * sample(x, segment) must choose one-sided data at segment endpoints.
 */
export function buildAdaptiveBundleMesh({
    interval: domain, tools, boundaries = [], sample, tolerances,
    estimateInterval = null, reductionEvidence = null, stateKey = null,
    minLength = 1e-6, maxDepth = 24, maxElements = 16384
}) {
    const [domainStart, domainEnd] = interval(domain, 'interval');
    if (typeof sample !== 'function') throw new TypeError('sample must be a function');
    if (estimateInterval !== null && typeof estimateInterval !== 'function') {
        throw new TypeError('estimateInterval must be a function');
    }
    if (reductionEvidence !== null && typeof reductionEvidence !== 'function') {
        throw new TypeError('reductionEvidence must be a function');
    }
    if (!(finite(minLength, 'minLength') > 0)) throw new RangeError('minLength must be positive');
    if (!Number.isInteger(maxDepth) || maxDepth < 0) throw new RangeError('maxDepth must be a nonnegative integer');
    if (!Number.isInteger(maxElements) || maxElements < 1) throw new RangeError('maxElements must be positive');
    const energyTolerance = tolerance(tolerances?.energy, 'energy');
    const fieldTolerances = Object.fromEntries(Object.entries(tolerances?.fields ?? {})
        .map(([key, value]) => [key, tolerance(value, key)]));
    const fieldKeys = Object.keys(fieldTolerances);
    if (!fieldKeys.length) throw new TypeError('At least one interpolation field and tolerance is required');
    const segments = partitionBundleCoverage({ interval: domain, tools, boundaries });
    if (segments.length > maxElements) throw new RangeError('Mandatory interfaces exceed maxElements');
    let evaluationCount = 0;
    const stack = segments.map(segment => ({
        start: segment.start, end: segment.end, segment, depth: 0, cache: new Map()
    })).reverse();
    const elements = [];
    while (stack.length) {
        const item = stack.pop();
        const { start, end, segment, depth, cache } = item;
        const h = end - start;
        const read = x => {
            if (!cache.has(x)) {
                const supplied = sample(x, segment);
                const fields = supplied?.fields;
                if (!fields || Object.keys(fields).length !== fieldKeys.length
                    || Object.keys(fields).some(key => !(key in fieldTolerances))) {
                    throw new TypeError('sample.fields must exactly match tolerances.fields');
                }
                cache.set(x, {
                    x, energyDensity: finite(supplied.energyDensity, 'energyDensity'),
                    fields: Object.fromEntries(fieldKeys.map(key => [key, sampleVector(fields[key], key)]))
                });
                evaluationCount++;
            }
            return cache.get(x);
        };
        const samples = [start, start + h / 4, start + h / 2, start + 3 * h / 4, end].map(read);
        const f = samples.map(v => v.energyDensity);
        const coarseEnergy = h * (f[0] + 4 * f[2] + f[4]) / 6;
        const energy = h * (f[0] + 4 * f[1] + 2 * f[2] + 4 * f[3] + f[4]) / 12;
        const energyError = Math.abs(energy - coarseEnergy) / 15;
        const absoluteBudget = energyTolerance.absolute * h / (domainEnd - domainStart);
        const errors = { energy: energyError };
        const ratios = { energy: ratio(energyError, Math.abs(energy), {
            absolute: absoluteBudget, relative: energyTolerance.relative
        }) };
        for (const key of fieldKeys) {
            const values = samples.map(v => v.fields[key]);
            if (values.some(v => v.length !== values[0].length)) {
                throw new TypeError(`${key} dimension must be constant within each segment`);
            }
            let error = 0;
            for (let j = 1; j < 4; j++) {
                const fraction = j / 4;
                const interpolated = values[0].map((v, i) => (1 - fraction) * v + fraction * values[4][i]);
                error = Math.max(error, distance(values[j], interpolated));
            }
            errors[key] = error;
            ratios[key] = ratio(error, Math.max(...values.map(norm)), fieldTolerances[key]);
        }
        let normalizedError = Math.max(...Object.values(ratios));
        const bound = estimateInterval?.({ start, end, segment, samples, errors, ratios });
        if (bound) {
            nonnegative(bound.normalizedError, 'estimateInterval.normalizedError');
            normalizedError = Math.max(normalizedError, bound.normalizedError);
        }
        const reasons = [];
        const midpoint = start + h / 2;
        if (normalizedError > 1) {
            if (depth >= maxDepth) reasons.push('max-depth');
            if (h / 2 < minLength) reasons.push('min-length');
            if (midpoint === start || midpoint === end) reasons.push('floating-point-resolution');
            if (elements.length + stack.length + 2 > maxElements) reasons.push('max-elements');
            if (!reasons.length) {
                stack.push({ ...item, start: midpoint, depth: depth + 1 });
                stack.push({ ...item, end: midpoint, depth: depth + 1 });
                continue;
            }
        }
        const converged = normalizedError <= 1;
        const element = {
            start, end, toolIds: [...segment.toolIds],
            materials: segment.tools.map(tool => materialMap(tool, start, end)),
            energy, errors, ratios, normalizedError, converged,
            certified: converged && bound?.certified === true,
            unresolvedReasons: reasons,
            quadrature: samples.map((entry, i) => ({
                x: entry.x, weight: h * [1, 4, 2, 4, 1][i] / 12,
                materials: segment.tools.map(tool => ({
                    id: tool.id, s: tool.offset + tool.scale * entry.x, dsDx: tool.scale
                }))
            }))
        };
        const evidence = converged ? reductionEvidence?.({ ...element, segment, samples }) : null;
        const reduction = evidence ? assessBundleReduction({ ...evidence, stateKey }) : {
            admitted: false, reasons: [converged ? 'missing-reduction-evidence' : 'discretization-error']
        };
        element.representation = reduction.admitted && segment.toolIds.length > 1 ? 'common-axis' : 'full';
        element.fullDofReasons = element.representation === 'full'
            ? segment.toolIds.length < 2 ? ['not-overlapping', ...reduction.reasons] : [...reduction.reasons] : [];
        element.reduction = reduction;
        elements.push(element);
    }
    return {
        nodes: [domainStart, ...elements.map(element => element.end)], elements, evaluationCount,
        energy: elements.reduce((sum, element) => sum + element.energy, 0),
        converged: elements.every(element => element.converged),
        certified: elements.every(element => element.certified),
        unresolved: elements.filter(element => !element.converged),
        maxNormalizedError: Math.max(0, ...elements.map(element => element.normalizedError))
    };
}

const REDUCTION_METRICS = ['energy', 'position', 'force', 'moment', 'twist', 'gap'];
const STATE_FIELDS = { position: 'positions', force: 'forces', moment: 'moments', twist: 'twists', gap: 'gaps' };

/**
 * Compare full/reduced reconstructions at the same named validation probes.
 * errorBounds are caller-supplied absolute UPPER bounds on errors between
 * probes / in eliminated modes, from an independently validated estimator.
 * They supplement measured errors; they cannot override failed comparisons.
 * Keys invalidate evidence after any geometry/material/coverage/load change.
 * Without bounds, comparisons are diagnostic only and reduction stays disabled.
 * Active/uncertain contact stays full; resolving its multipliers is solver work.
 */
export function assessBundleReduction({ stateKey, full, reduced, tolerances, certificate = null, contactMargin = 0 }) {
    nonnegative(contactMargin, 'contactMargin');
    const reasons = [];
    const reject = reason => { if (!reasons.includes(reason)) reasons.push(reason); };
    if (typeof stateKey !== 'string' || !stateKey) reject('missing-current-state-key');
    if (!full || !reduced || full.stateKey !== stateKey || reduced.stateKey !== stateKey) {
        reject('state-or-coverage-changed');
    }
    if (!Array.isArray(full?.probeIds) || !full.probeIds.length
        || new Set(full.probeIds).size !== full.probeIds.length
        || !Array.isArray(reduced?.probeIds) || full.probeIds.length !== reduced.probeIds.length
        || full.probeIds.some((id, i) => id !== reduced.probeIds[i])) reject('probe-mismatch');
    if (!certificate) reject('missing-error-bounds');
    else if (certificate.stateKey !== full?.stateKey) reject('stale-error-bounds');
    const measurements = {};
    const errors = {};
    const ratios = {};
    for (const metric of REDUCTION_METRICS) {
        const tol = tolerance(tolerances?.[metric], metric);
        const bound = certificate?.errorBounds?.[metric];
        if (certificate && (!Number.isFinite(bound) || bound < 0)) reject(`missing-${metric}-bound`);
        let measured;
        let scale;
        if (metric === 'energy') {
            if (!Number.isFinite(full?.energy) || !Number.isFinite(reduced?.energy)) {
                reject('missing-energy-evidence');
                continue;
            }
            measured = Math.abs(full.energy - reduced.energy);
            scale = Math.max(Math.abs(full.energy), Math.abs(reduced.energy));
        } else {
            const field = STATE_FIELDS[metric];
            const a = full?.[field];
            const b = reduced?.[field];
            if (!Array.isArray(a) || !Array.isArray(b) || a.length !== full?.probeIds?.length || a.length !== b.length) {
                reject(`missing-${metric}-evidence`);
                continue;
            }
            measured = 0;
            scale = 0;
            for (let i = 0; i < a.length; i++) {
                const u = sampleVector(a[i], `full.${field}`);
                const v = sampleVector(b[i], `reduced.${field}`);
                if (u.length !== v.length) throw new TypeError(`${metric} dimensions must match`);
                if (metric === 'gap' && (u.length !== 1 || v.length !== 1)) throw new TypeError('gaps must be scalars');
                measured = Math.max(measured, distance(u, v));
                // Position error tolerances must not depend on world translation.
                scale = metric === 'position'
                    ? Math.max(scale, distance(u, sampleVector(a[0], 'position origin')),
                        distance(v, sampleVector(b[0], 'position origin')))
                    : Math.max(scale, norm(u), norm(v));
                if (metric === 'gap' && Math.min(u[0], v[0]) <= contactMargin + (bound ?? Infinity)) {
                    reject('active-or-uncertain-contact');
                }
            }
        }
        measurements[metric] = measured;
        // A bound estimates the unobserved remainder, so add it conservatively.
        errors[metric] = measured + (Number.isFinite(bound) && bound >= 0 ? bound : 0);
        ratios[metric] = ratio(errors[metric], scale, tol);
        if (ratios[metric] > 1) reject(`${metric}-error`);
    }
    return { admitted: reasons.length === 0, reasons, measurements, errors, ratios };
}

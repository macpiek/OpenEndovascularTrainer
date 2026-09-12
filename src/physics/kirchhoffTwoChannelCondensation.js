import { createKirchhoffLinearKernel } from './kirchhoffLinearKernel.js';

// One bounded reusable numerical arena. Published Schur/recovery arrays are
// separately owned; no returned result borrows factor/work storage. Nested
// calls get a temporary arena instead of overwriting an in-flight factor.
let reusableArena=null;
const capacity=n=>2**Math.ceil(Math.log2(Math.max(1,n)));
function acquireArena(entries,n){
    let arena=reusableArena;
    if(arena&&!arena.busy&&arena.factor.length>=entries&&arena.work.length>=n){
        arena.busy=true;return {arena,reused:true};
    }
    const factorCapacity=capacity(Math.max(entries,arena&&!arena.busy?arena.factor.length:0)),nodeCapacity=capacity(Math.max(n,arena&&!arena.busy?arena.work.length:0)),kernel=createKirchhoffLinearKernel(factorCapacity*8+nodeCapacity*12+128);
    arena={kernel,factor:kernel.alloc(Float64Array,factorCapacity),work:kernel.alloc(Float64Array,nodeCapacity),
        starts:kernel.alloc(Int32Array,nodeCapacity),busy:true};
    if(!reusableArena||!reusableArena.busy)reusableArena=arena;
    return {arena,reused:false};
}

const bilateral = (lo, hi) => lo === -Infinity && hi === Infinity;
const fallback = (reason, detail = {}) => ({ status: 'fallback', reason, ...detail });

/** Exact Schur condensation of a borrowed native symmetric-band assembly.
 * Channel metadata has the dense adapter's ORIGINAL-row indexing/contract.
 * Only bilateral physical-pose / bias-motion pairs with identical alpha are
 * eliminated. All inequalities and friction rows remain in the Schur system.
 *
 * The equality block is [Ae, Gee; 0, Ae], Gee=Ae-diag(alpha). One unshifted,
 * diagonally scaled Wasm Cholesky factor of Ae serves both channels. Clamped
 * or near-null pivots and failed original-equation checks cause an explicit
 * fallback, never an approximate change of compliance.
 *
 * No full expanded 2N-by-2N matrix is allocated. Output arrays, metadata and
 * recover()'s private response columns survive subsequent native assemblies.
 * recover() returns owned increments in ORIGINAL native row order plus the
 * dense adapter's sorted/interleaved fullIncrement. It does not apply forces,
 * update hints, solve retained inequalities, or replace the full J*dq gate.
 */
export function condenseKirchhoffTwoChannelSystem(native, channels, options = {}) {
    const n = native.count, band = native.band;
    if (!Number.isInteger(n) || n < 0 || !Number.isInteger(band) || band < 1 ||
        native.matrix.length < n * band || native.order.length !== n || native.rows.length < n ||
        ['rhs', 'lower', 'upper'].some(key => native[key].length < n)) throw new RangeError('Invalid native band system');
    const specification = typeof channels === 'function' ? channels(native) : channels;
    if (!Array.isArray(specification) || specification.length !== n) throw new RangeError('An explicit channel is required for every native row');
    const descriptors = specification.map(item => {
        if (!['pose', 'physical-motion'].includes(item?.physical) || !Object.hasOwn(item, 'bias'))
            throw new RangeError('Explicit physical and bias channel membership required');
        const b = item.bias;
        if (b !== null && (!['pose', 'bias-motion'].includes(b?.channel) ||
            ![b.strain, b.alpha, b.lambda].every(Number.isFinite) || b.alpha < 0 ||
            typeof b.lower !== 'number' || typeof b.upper !== 'number' ||
            Number.isNaN(b.lower) || Number.isNaN(b.upper) || b.lower > b.upper)) throw new RangeError('Invalid bias row');
        return { physical: item.physical, bias: b === null ? null : { ...b } };
    });
    const order = Int32Array.from(native.order), seen = new Uint8Array(n), grouped = new Uint8Array(n);
    const nativeGroups = native.groups ?? [];
    for (let i = 0; i < n; i++) {
        const original = native.order[i], row = native.rows[original];
        if (!Number.isInteger(original) || original < 0 || original >= n || seen[original]++) throw new RangeError('Invalid native row permutation');
        if (!row || !Number.isFinite(row.alpha) || row.alpha < 0 || !Number.isFinite(native.rhs[i]) ||
            typeof native.lower[i] !== 'number' || typeof native.upper[i] !== 'number' ||
            Number.isNaN(native.lower[i]) || Number.isNaN(native.upper[i]) || native.lower[i] > native.upper[i])
            throw new RangeError('Invalid native row');
    }
    for (const group of nativeGroups) {
        if (!group.rows || group.rows.length !== 2) throw new RangeError('Invalid friction group');
        for (const sorted of group.rows) {
            if (!Number.isInteger(sorted) || sorted < 0 || sorted >= n || grouped[sorted]++) throw new RangeError('Invalid or overlapping friction group');
            const d = descriptors[order[sorted]];
            if (d.physical !== 'physical-motion' || d.bias !== null) return fallback('unsupported-friction-channel', { originalRow: order[sorted] });
        }
        if (group.normalRow != null && (!Number.isInteger(group.normalRow) || group.normalRow < 0 || group.normalRow >= n))
            throw new RangeError('Invalid friction normal row');
    }
    const valueAt = (i, j) => {
        const hi = Math.max(i, j), lo = Math.min(i, j);
        return hi - lo < band ? native.matrix[hi * band + hi - lo] : 0;
    };
    const equality = [], equalityMap = new Int32Array(n).fill(-1), zeroEqualityRows = [];
    for (let sorted = 0; sorted < n; sorted++) {
        const original = order[sorted], d = descriptors[original], row = native.rows[original];
        const candidate = !grouped[sorted] && d.physical === 'pose' && bilateral(native.lower[sorted], native.upper[sorted]);
        if (candidate) {
            if (d.bias?.channel !== 'bias-motion' || !bilateral(d.bias.lower, d.bias.upper))
                return fallback('unsupported-equality-pair', { originalRow: original });
            if (d.bias.alpha !== row.alpha) return fallback('unequal-bias-alpha', { originalRow: original, physicalAlpha: row.alpha, biasAlpha: d.bias.alpha });
            if (valueAt(sorted, sorted) === 0) {
                if (row.alpha !== 0) return fallback('singular-equality-block', { originalRow: original });
                for (let j = Math.max(0, sorted - band + 1); j < Math.min(n, sorted + band); j++)
                    if (valueAt(sorted, j) !== 0) return fallback('singular-equality-block', { originalRow: original });
                // Preserve BOTH rows and their exact RHS in the retained
                // system, including nonzero roundoff/immovable residuals.
                // They are neither invertible equalities nor discarded rows.
                zeroEqualityRows.push(original); continue;
            }
            equalityMap[sorted] = equality.length; equality.push(sorted);
        } else if (row.kind === 'material' || (d.bias?.channel === 'bias-motion' &&
            !(Number.isFinite(native.lower[sorted]) && native.lower[sorted] === native.upper[sorted] &&
                Number.isFinite(d.bias.lower) && d.bias.lower === d.bias.upper))) {
            return fallback('unsupported-equality-pair', { originalRow: original });
        }
    }
    const ne = equality.length;
    if (!ne) return fallback('no-paired-equalities');
    const gram = (i, j) => valueAt(i, j) - (i === j ? native.rows[order[i]].alpha : 0);
    let equalityBand = 1;
    for (let i = 0; i < ne; i++) for (let j = Math.max(0, equality[i] - band + 1); j < equality[i]; j++)
        if (equalityMap[j] >= 0 && valueAt(equality[i], j) !== 0) equalityBand = Math.max(equalityBand, i - equalityMap[j] + 1);
    const ae = new Float64Array(ne * equalityBand), alphaE = new Float64Array(ne), scales = new Float64Array(ne);
    const {arena,reused}=acquireArena(ae.length,ne),kernel=arena.kernel;
    const factor=arena.factor.subarray(0,ae.length),work=arena.work.subarray(0,ne),starts=arena.starts.subarray(0,ne);
    factor.fill(0);
    try {
    for (let i = 0; i < ne; i++) {
        const diagonal = valueAt(equality[i], equality[i]);
        if (!(diagonal > 0) || !Number.isFinite(diagonal)) return fallback('singular-equality-block', { originalRow: order[equality[i]] });
        scales[i] = 1 / Math.sqrt(diagonal); alphaE[i] = native.rows[order[equality[i]]].alpha;
    }
    for (let i = 0; i < ne; i++) {
        starts[i] = i;
        for (let j = Math.max(0, i - equalityBand + 1); j <= i; j++) {
            const a = valueAt(equality[i], equality[j]);
            if (!Number.isFinite(a)) throw new RangeError('Non-finite native matrix');
            const index = i * equalityBand + i - j;
            ae[index] = a; factor[index] = a * scales[i] * scales[j];
            if (a !== 0) starts[i] = Math.min(starts[i], j);
        }
    }
    const skyline = options.factorization !== 'band' && typeof kernel.factorSkyline === 'function';
    if (skyline) kernel.factorSkyline(factor.byteOffset, ne, equalityBand, starts.byteOffset);
    else kernel.factorBand(factor.byteOffset, ne, equalityBand, -1, 1);
    let minimumScaledPivot = Infinity;
    for (let i = 0; i < ne; i++) {
        const pivot = factor[i * equalityBand] ** 2;
        minimumScaledPivot = Math.min(minimumScaledPivot, pivot);
        // Existing kernels clamp pivots at 1e-12. Never use that clamp as a
        // substitute for an invertible original equality matrix.
        if (!(pivot > 32e-12) || !Number.isFinite(pivot)) return fallback('unsafe-equality-factor', {
            originalRow: order[equality[i]], minimumScaledPivot, factorizations: 1 });
    }
    const diagnostics = { equalityCount: ne, eliminatedRows: 2 * ne, equalityBand, equalityMatrixEntries: ae.length,
        factorEntries: factor.length, kernelArenaReused:reused, kernelArenaFactorCapacity:arena.factor.length,
        kernelArenaNodeCapacity:arena.work.length, factorizations: 1, factorization: skyline ? 'wasm-skyline' : 'wasm-band',
        minimumScaledPivot, maximumEqualityBackwardError: 0, equalitySolves: 0, zeroEqualityResponses: 0,
        reusedEqualityResponses: 0, fullExpandedMatrixEntries: 0,
        retainedZeroEqualityOriginalRows: zeroEqualityRows.slice() };
    function multiply(input, out, absolute = false, gramOnly = false) {
        out.fill(0);
        for (let i = 0; i < ne; i++) {
            const diagonal = ae[i * equalityBand] - (gramOnly ? alphaE[i] : 0);
            out[i] += absolute ? Math.abs(diagonal * input[i]) : diagonal * input[i];
            for (let j = Math.max(0, i - equalityBand + 1); j < i; j++) {
                const a = ae[i * equalityBand + i - j];
                out[i] += absolute ? Math.abs(a * input[j]) : a * input[j];
                out[j] += absolute ? Math.abs(a * input[i]) : a * input[i];
            }
        }
    }
    const product = new Float64Array(ne), magnitude = new Float64Array(ne), gBeta = new Float64Array(ne);
    function solveAe(rhs) {
        // Ae has already passed the unshifted factor/pivot checks. Its response
        // to an exactly zero load is zero; no factor solve or residual scan is
        // needed. This is an algebraic case, not a tolerance-based load cutoff.
        if (rhs.every(value => value === 0)) {
            diagnostics.zeroEqualityResponses++;
            return new Float64Array(ne);
        }
        for (let i = 0; i < ne; i++) work[i] = rhs[i] * scales[i];
        if (skyline) kernel.solveSkyline(factor.byteOffset, work.byteOffset, ne, equalityBand, starts.byteOffset);
        else kernel.solveBand(factor.byteOffset, work.byteOffset, ne, equalityBand);
        const result = Float64Array.from(work, (v, i) => v * scales[i]);
        multiply(result, product); multiply(result, magnitude, true);
        for (let i = 0; i < ne; i++) {
            const error = Math.abs(product[i] - rhs[i]) / Math.max(Number.MIN_VALUE, Math.abs(rhs[i]) + magnitude[i]);
            diagnostics.maximumEqualityBackwardError = Math.max(diagnostics.maximumEqualityBackwardError, error);
            if (!Number.isFinite(result[i]) || !Number.isFinite(error) || error > 1e-10) throw new Error('equality-solve-residual');
        }
        diagnostics.equalitySolves++;
        return result;
    }
    function solvePair(rp, rb, knownBiasResponse = null) {
        const b = knownBiasResponse ?? solveAe(rb);
        if (knownBiasResponse) diagnostics.reusedEqualityResponses++;
        if (b.every(value => value === 0)) return [solveAe(rp), b];
        multiply(b, gBeta, false, true);
        return [solveAe(Float64Array.from(rp, (v, i) => v - gBeta[i])), b];
    }
    const retained = [], physicalExpanded = new Int32Array(n), biasExpanded = new Int32Array(n).fill(-1);
    const physicalRetained = new Int32Array(n).fill(-1), biasRetained = new Int32Array(n).fill(-1);
    let expandedCount = 0;
    for (let sorted = 0; sorted < n; sorted++) {
        const original = order[sorted], d = descriptors[original], nr = native.rows[original];
        physicalExpanded[sorted] = expandedCount++;
        if (equalityMap[sorted] < 0) {
            physicalRetained[original] = retained.length;
            retained.push({ sorted, original, phase: 'physical', channel: d.physical, alpha: nr.alpha,
                rhs: native.rhs[sorted], lower: native.lower[sorted], upper: native.upper[sorted] });
        }
        if (d.bias) {
            biasExpanded[sorted] = expandedCount++;
            if (equalityMap[sorted] < 0) {
                const b = d.bias; biasRetained[original] = retained.length;
                retained.push({ sorted, original, phase: 'bias', channel: b.channel, alpha: b.alpha,
                    rhs: -b.strain - b.alpha * b.lambda, lower: b.lower - b.lambda, upper: b.upper - b.lambda });
            }
        }
    }
    const groups = [];
    for (const group of nativeGroups) {
        const rows = Array.from(group.rows, i => physicalRetained[order[i]]);
        const normalRow = group.normalRow == null ? undefined : physicalRetained[order[group.normalRow]];
        if (rows.some(i => i < 0) || normalRow === -1) return fallback('eliminated-friction-member');
        groups.push({ ...group, rows, normalRow, lambda: Array.from(group.lambda), radii: Array.from(group.radii),
            ...(group.originalRows ? { originalRows: Array.from(group.originalRows) } : {}), ...(group.mu ? { mu: Array.from(group.mu) } : {}) });
    }
    const k = retained.length, couplings = retained.map(row => {
        const out = [];
        for (let j = Math.max(0, row.sorted - band + 1); j < Math.min(n, row.sorted + band); j++) {
            const e = equalityMap[j], g = e < 0 ? 0 : gram(row.sorted, j);
            if (g) out.push(e, g);
        }
        return out;
    });
    const couplingDot = (row, entries, p, b) => {
        const readP = row.phase === 'physical' || row.channel === 'pose', readB = row.phase === 'bias' || row.channel === 'pose';
        let sum = 0;
        for (let i = 0; i < entries.length; i += 2) sum += entries[i + 1] *
            ((readP ? p[entries[i]] : 0) + (readB ? b[entries[i]] : 0));
        return sum;
    };
    const matrix = new Float64Array(k * k), rhs = new Float64Array(k);
    const responsesP = new Float64Array(k * ne), responsesB = new Float64Array(k * ne);
    let freeP, freeB;
    try {
        [freeP, freeB] = solvePair(Float64Array.from(equality, i => native.rhs[i]), Float64Array.from(equality, i => {
            const b = descriptors[order[i]].bias; return -b.strain - b.alpha * b.lambda;
        }));
        for (let i = 0; i < k; i++) rhs[i] = retained[i].rhs - couplingDot(retained[i], couplings[i], freeP, freeB);
        const rp = new Float64Array(ne), rb = new Float64Array(ne);
        const physicalColumns = new Int32Array(n).fill(-1);
        for (let j = 0; j < k; j++) {
            rp.fill(0); rb.fill(0);
            for (let i = 0; i < couplings[j].length; i += 2) {
                const e = couplings[j][i], g = couplings[j][i + 1]; rp[e] = g;
                if (retained[j].phase === 'bias') rb[e] = g;
            }
            // Both columns of one native row have the same equality coupling
            // g. The physical column has (p,b)=(Ae^-1 g,0); that p is exactly
            // the bias column's b. Reuse only within this frozen assembly,
            // keeping all response columns owned for subsequent recovery.
            const prior = physicalColumns[retained[j].sorted];
            const knownBiasResponse = retained[j].phase === 'bias' && prior >= 0
                ? responsesP.subarray(prior * ne, (prior + 1) * ne) : null;
            const [p, b] = solvePair(rp, rb, knownBiasResponse);
            responsesP.set(p, j * ne); responsesB.set(b, j * ne);
            if (retained[j].phase === 'physical') physicalColumns[retained[j].sorted] = j;
            for (let i = 0; i < k; i++) {
                const row = retained[i], col = retained[j];
                const a = row.phase === col.phase || row.channel === 'pose' ? gram(row.sorted, col.sorted) : 0;
                matrix[i * k + j] = a + (i === j ? row.alpha : 0) - couplingDot(row, couplings[i], p, b);
            }
        }
        if (!matrix.every(Number.isFinite) || !rhs.every(Number.isFinite)) throw new Error('non-finite-schur');
    } catch (error) {
        if (['equality-solve-residual', 'non-finite-schur'].includes(error.message)) return fallback(error.message, { diagnostics });
        throw error;
    }
    diagnostics.retainedRows = k; diagnostics.schurEntries = k * k; diagnostics.responseEntries = 2 * k * ne;
    const equalityOriginal = Int32Array.from(equality, i => order[i]);
    function recover(increment) {
        if (increment.length !== k || !increment.every(Number.isFinite)) throw new RangeError('Finite retained increment required');
        const ep = freeP.slice(), eb = freeB.slice(), physicalIncrement = new Float64Array(n), biasIncrement = new Float64Array(n);
        for (let j = 0; j < k; j++) {
            const v = increment[j], row = retained[j];
            (row.phase === 'physical' ? physicalIncrement : biasIncrement)[row.original] = v;
            for (let i = 0; i < ne; i++) { ep[i] -= responsesP[j * ne + i] * v; eb[i] -= responsesB[j * ne + i] * v; }
        }
        for (let i = 0; i < ne; i++) { physicalIncrement[equalityOriginal[i]] = ep[i]; biasIncrement[equalityOriginal[i]] = eb[i]; }
        const fullIncrement = new Float64Array(expandedCount);
        for (let i = 0; i < n; i++) {
            fullIncrement[physicalExpanded[i]] = physicalIncrement[order[i]];
            if (biasExpanded[i] >= 0) fullIncrement[biasExpanded[i]] = biasIncrement[order[i]];
        }
        return { physicalIncrement, biasIncrement, fullIncrement };
    }
    return { status: 'condensed', matrixFormat: 'row-major', count: k, band: k, matrix, rhs,
        lower: Float64Array.from(retained, r => r.lower), upper: Float64Array.from(retained, r => r.upper), groups,
        nativeCount: n, expandedCount, equalityOriginalRows: equalityOriginal.slice(),
        retainedRows: retained.map(r => ({ ...r })), originalToRetainedPhysical: physicalRetained.slice(),
        originalToRetainedBias: biasRetained.slice(), diagnostics, recover };
    } finally { arena.busy=false; }
}

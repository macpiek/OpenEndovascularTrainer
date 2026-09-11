import { createKirchhoffLinearKernel } from './kirchhoffLinearKernel.js';
import { solveCoupledBandQP } from './kirchhoffCoupledLinearSolver.js';
import { solveCoupledFrictionQP, measureCoupledFrictionKKT } from './kirchhoffCoupledFrictionSolver.js';
import { solveCoupledLoadQP, measureCoupledLoadKKT } from './kirchhoffCoupledLoadSolver.js';
import { solveSeededCoulombNewton } from './kirchhoffCoulombNewtonSolver.js';

const valueAt = (matrix, band, i, j) => {
    if (i < j) [i, j] = [j, i];
    return i - j < band ? matrix[i * band + i - j] : 0;
};
function grownCapacity(required, current = 0) {
    let capacity = Math.max(16, current);
    while (capacity < required) capacity *= 2;
    return capacity;
}

/** Exact material elimination with a growing contact working
 * set. Equality responses are computed only for contacts used by the solve.
 * Every excluded inequality is checked in the ORIGINAL full matrix after
 * reconstruction and any violated row is added before success is possible.
 * There is no contact-distance cutoff, removed physical DOF or reduced
 * stiffness. Contact responses share factor reads in batches of four.
 */
export function solveActiveCondensedCoupledQP(matrix, rhs, lower, upper, count, band, groups = [], options = {}) {
    // Explicit experimental route: keep every original material/contact row
    // in the local dual operator, including zero-load friction groups. This
    // bypasses global equality elimination and its dense contact Schur matrix.
    // The existing fixed-load seed and simultaneous original KKT stay intact.
    if (options.coulombStructure === 'full-band') {
        const result = solveSeededCoulombNewton(matrix, rhs, lower, upper, count, band, groups,
            { ...options, coulombLinearSolver: 'band-lu' });
        result.diagnostics.coulombStructure = 'full-band';
        result.diagnostics.originalCount = count;
        // The fixed-load seed can itself certify the solution and return
        // workspace views. Preserve this entry point's owned-output contract.
        return { ...result, increment: Float64Array.from(result.increment), residual: Float64Array.from(result.residual),
            free: Uint8Array.from(result.free), lower: Float64Array.from(result.lower), upper: Float64Array.from(result.upper) };
    }
    const began = performance.now();
    let schurMs=0, contactSolveMs=0, reconstructionMs=0, seedMs=0;
    const tolerance = options.tolerance ?? 1e-8;
    const workspace = options.condensedWorkspace ?? ((options.workspace ??= {}).activeCondensed ??= {});
    if (!workspace.grouped || workspace.grouped.length < count) {
        const capacity = grownCapacity(count, workspace.grouped?.length);
        workspace.grouped = new Uint8Array(capacity); workspace.equalityMap = new Int32Array(capacity);
        workspace.base = new Float64Array(capacity); workspace.adjusted = new Float64Array(capacity);
        workspace.localMap = new Int32Array(capacity);
    }
    const grouped = workspace.grouped, equalityMap = workspace.equalityMap;
    const base = workspace.base.subarray(0, count), adjusted = workspace.adjusted.subarray(0, count);
    const dynamicRows = workspace.dynamicRows ??= new Map(), selected = workspace.selected ??= new Set();
    const equality = workspace.equality ??= [], candidates = workspace.candidates ??= [];
    grouped.fill(0, 0, count); equalityMap.fill(-1, 0, count); base.fill(0);
    dynamicRows.clear(); selected.clear(); equality.length = candidates.length = 0;
    for (const group of groups) {
        for (const row of group.rows) grouped[row] = 1;
        if (group.normalRow != null) {
            grouped[group.normalRow] = 1;
            group.rows.forEach((row, axis) => dynamicRows.set(row, { group, axis }));
        }
    }
    // Public result arrays remain owned snapshots; only internal arithmetic
    // storage is reused. No multiplier or response survives as a warm start.
    const residual = new Float64Array(count), x = new Float64Array(count);
    for (let i = 0; i < count; i++) {
        if (!grouped[i] && lower[i] === -Infinity && upper[i] === Infinity && matrix[i * band] > 0) {
            equalityMap[i] = equality.length; equality.push(i);
        } else {
            const member = dynamicRows.get(i);
            base[i] = member ? -member.group.lambda[member.axis] : Math.max(lower[i], Math.min(upper[i], 0));
            if (!member) {
                if (lower[i] !== upper[i]) candidates.push(i);
                if (base[i] > lower[i] && base[i] < upper[i] || options.initialFree?.[i]) selected.add(i);
            }
        }
    }
    function includeGroups() {
        for (const group of groups) if (group.normalRow == null || selected.has(group.normalRow) ||
            group.normalLambda + base[group.normalRow] > 0) {
            group.rows.forEach(row => selected.add(row));
            if (group.normalRow != null) selected.add(group.normalRow);
        }
    }
    includeGroups();
    const ne = equality.length;
    let equalityBand = 1;
    for (let i = 0; i < ne; i++) for (let j = Math.max(0, equality[i] - band + 1); j < equality[i]; j++)
        if (equalityMap[j] >= 0 && valueAt(matrix, band, equality[i], j) !== 0)
            equalityBand = Math.max(equalityBand, i - equalityMap[j] + 1);
    const equalityEntries = ne * equalityBand;
    if (!workspace.kernel || workspace.countCapacity < count || workspace.equalityCapacity < ne ||
        workspace.matrixCapacity < matrix.length || workspace.factorCapacity < equalityEntries) {
        workspace.countCapacity = grownCapacity(count, workspace.countCapacity);
        workspace.equalityCapacity = grownCapacity(ne, workspace.equalityCapacity);
        workspace.matrixCapacity = grownCapacity(matrix.length, workspace.matrixCapacity);
        workspace.factorCapacity = grownCapacity(equalityEntries, workspace.factorCapacity);
        const kernel = workspace.kernel = createKirchhoffLinearKernel(8 * (workspace.matrixCapacity + workspace.factorCapacity) +
            32 * workspace.countCapacity + 64 * workspace.equalityCapacity + 128);
        workspace.fullMatrix = kernel.alloc(Float64Array, workspace.matrixCapacity);
        for (const key of ['fullRhs', 'fullX', 'fullResidual']) workspace[key] = kernel.alloc(Float64Array, workspace.countCapacity);
        workspace.fullStarts = kernel.alloc(Int32Array, workspace.countCapacity);
        workspace.factor = kernel.alloc(Float64Array, workspace.factorCapacity);
        workspace.starts = kernel.alloc(Int32Array, workspace.equalityCapacity);
        for (const key of ['scales', 'free', 'solveRhs']) workspace[key] = kernel.alloc(Float64Array, workspace.equalityCapacity);
        workspace.batchRhs = kernel.alloc(Float64Array, workspace.equalityCapacity * 4);
    }
    const { kernel, factor, starts, fullMatrix, fullRhs, fullX, fullResidual, fullStarts } = workspace;
    const scales = workspace.scales.subarray(0, ne), free = workspace.free.subarray(0, ne), solveRhs = workspace.solveRhs.subarray(0, ne);
    fullMatrix.set(matrix); fullRhs.set(rhs);
    kernel.findBandStarts(fullMatrix.byteOffset, fullStarts.byteOffset, count, band);
    const residualOf = input => {
        fullX.set(input);
        kernel.residualBandProfile(fullMatrix.byteOffset, fullRhs.byteOffset, fullX.byteOffset, fullResidual.byteOffset, count, band, fullStarts.byteOffset);
        residual.set(fullResidual.subarray(0, count));
    };
    residualOf(base); adjusted.set(residual);
    for (let i = 0; i < ne; i++) scales[i] = 1 / Math.sqrt(matrix[equality[i] * band]);
    for (let i = 0; i < ne; i++) {
        starts[i] = i; free[i] = adjusted[equality[i]] * scales[i];
        for (let j = Math.max(0, i - equalityBand + 1); j <= i; j++) {
            const a = valueAt(matrix, band, equality[i], equality[j]) * scales[i] * scales[j];
            factor[i * equalityBand + i - j] = a;
            if (a !== 0) starts[i] = Math.min(starts[i], j);
        }
    }
    if (ne) {
        kernel.factorSkyline(factor.byteOffset, ne, equalityBand, starts.byteOffset);
        kernel.solveSkyline(factor.byteOffset, free.byteOffset, ne, equalityBand, starts.byteOffset);
        for (let i = 0; i < ne; i++) free[i] *= scales[i];
    }
    const responses = workspace.responses ??= new Map(), couplings = workspace.couplings ??= new Map();
    const responsePool = workspace.responsePool ??= [], couplingPool = workspace.couplingPool ??= [];
    responses.clear(); couplings.clear();
    let responseKernelCalls = 0, responseBatches = 0;
    function response(row) {
        if (responses.has(row)) return responses.get(row);
        const slot = responses.size, coupling = couplingPool[slot] ??= [];
        solveRhs.fill(0); coupling.length = 0;
        for (let j = Math.max(0, row - band + 1); j < Math.min(count, row + band); j++) {
            const e = equalityMap[j], a = valueAt(matrix, band, row, j);
            if (e >= 0 && a !== 0) { coupling.push(e, a); solveRhs[e] = a * scales[e]; }
        }
        if (ne) { kernel.solveSkyline(factor.byteOffset, solveRhs.byteOffset, ne, equalityBand, starts.byteOffset); responseKernelCalls++; }
        if (!responsePool[slot] || responsePool[slot].length < ne) responsePool[slot] = new Float64Array(workspace.equalityCapacity);
        const result = responsePool[slot];
        for (let i = 0; i < ne; i++) result[i] = solveRhs[i] * scales[i];
        couplings.set(row, coupling); responses.set(row, result); return result;
    }
    function prepareResponses(rows) {
        if (options.batchEqualityResponses === false || !ne) {
            for (const row of rows) response(row);
            return;
        }
        const pending = workspace.pendingResponses ??= [];
        pending.length = 0;
        for (const row of rows) if (!responses.has(row)) pending.push(row);
        const batch = workspace.batchRhs;
        let index = 0;
        for (; index + 4 <= pending.length; index += 4) {
            batch.fill(0, 0, ne * 4);
            const firstSlot = responses.size;
            for (let axis = 0; axis < 4; axis++) {
                const row = pending[index + axis], slot = firstSlot + axis;
                const coupling = couplingPool[slot] ??= [];
                coupling.length = 0;
                for (let j = Math.max(0, row - band + 1); j < Math.min(count, row + band); j++) {
                    const e = equalityMap[j], a = valueAt(matrix, band, row, j);
                    if (e >= 0 && a !== 0) {
                        coupling.push(e, a); batch[e * 4 + axis] = a * scales[e];
                    }
                }
                couplings.set(row, coupling);
            }
            kernel.solveSkyline4(factor.byteOffset, batch.byteOffset, ne, equalityBand, starts.byteOffset);
            responseKernelCalls++; responseBatches++;
            for (let axis = 0; axis < 4; axis++) {
                const slot = firstSlot + axis;
                if (!responsePool[slot] || responsePool[slot].length < ne)
                    responsePool[slot] = new Float64Array(workspace.equalityCapacity);
                const result = responsePool[slot];
                for (let i = 0; i < ne; i++) result[i] = batch[i * 4 + axis] * scales[i];
                responses.set(pending[index + axis], result);
            }
        }
        for (; index < pending.length; index++) response(pending[index]);
    }
    if (!workspace.equalitySolution || workspace.equalitySolution.length < ne) workspace.equalitySolution = new Float64Array(workspace.equalityCapacity);
    const equalitySolution = workspace.equalitySolution;
    const reconstruct = (rows, delta) => {
        x.set(base);
        equalitySolution.set(free);
        for (let i = 0; i < rows.length; i++) {
            const d = delta[i], r = response(rows[i]); x[rows[i]] += d;
            // Accumulate in contiguous material coordinates, then scatter
            // once. The column order and every arithmetic operation remain
            // identical; no response outside the contact region is omitted.
            for (let j = 0; j < ne; j++) equalitySolution[j] -= r[j] * d;
        }
        for (let i = 0; i < ne; i++) x[equality[i]] = equalitySolution[i];
        residualOf(x);
    };
    reconstruct([], []);
    function addViolated() {
        let added = 0;
        for (const row of candidates) if (!selected.has(row)) {
            const r = residual[row], v = x[row] <= lower[row] ? Math.max(0, r) :
                x[row] >= upper[row] ? Math.max(0, -r) : Math.abs(r);
            if (v > tolerance) { selected.add(row); added++; }
        }
        includeGroups(); return added;
    }
    addViolated();
    const dynamic = groups.some(group => group.normalRow != null);
    let result, rows, finalGroups, expansions = 0, factors = ne ? 1 : 0, iterations = 0, kkt;
    // A growing contact set changes the selected rows, not this frozen
    // operator. Preserve old Schur entries only within this solve; next
    // geometry/material assembly always starts with an empty cache.
    let previousCount = 0, schurEvaluations = 0, schurReuses = 0;
    const previousMap = workspace.previousSchurMap ??= new Map();
    previousMap.clear();
    const setupMs=performance.now()-began;
    const limit = options.maxCondensedExpansions ?? 8;
    for (; expansions < limit; expansions++) {
        const schurStarted=performance.now();
        rows = [...selected].sort((a, b) => a - b);
        const nr = rows.length, map = workspace.localMap;
        map.fill(-1, 0, count);
        rows.forEach((row, i) => { map[row] = i; });
        prepareResponses(rows);
        if (!workspace.schurMatrix || workspace.schurMatrix.length < nr * nr) workspace.schurMatrix = new Float64Array(grownCapacity(nr * nr, workspace.schurMatrix?.length));
        if (!workspace.localRhs || workspace.localRhs.length < nr) {
            const capacity = grownCapacity(nr, workspace.localRhs?.length);
            for (const key of ['localRhs', 'localLower', 'localUpper']) workspace[key] = new Float64Array(capacity);
            workspace.localFree = new Uint8Array(capacity);
        }
        const A = workspace.schurMatrix.subarray(0, nr * nr), b = workspace.localRhs.subarray(0, nr);
        const oldIndices = previousCount ? rows.map(row => previousMap.get(row)) : null;
        for (let i = 0; i < nr; i++) {
            const c = couplings.get(rows[i]); let r = adjusted[rows[i]];
            for (let k = 0; k < c.length; k += 2) r -= c[k + 1] * free[c[k]];
            b[i] = r;
            for (let j = 0; j <= i; j++) {
                const oldI = oldIndices?.[i], oldJ = oldIndices?.[j];
                if (oldI !== undefined && oldJ !== undefined) {
                    A[i * nr + i - j] = workspace.previousSchur[oldI * previousCount + oldI - oldJ];
                    schurReuses++;
                    continue;
                }
                let a = valueAt(matrix, band, rows[i], rows[j]); const column = responses.get(rows[j]);
                for (let k = 0; k < c.length; k += 2) a -= c[k + 1] * column[c[k]];
                A[i * nr + i - j] = a;
                schurEvaluations++;
            }
        }
        const lo = workspace.localLower.subarray(0, nr), hi = workspace.localUpper.subarray(0, nr), initialFree = workspace.localFree.subarray(0, nr);
        for (let i = 0; i < nr; i++) {
            lo[i] = lower[rows[i]] - base[rows[i]]; hi[i] = upper[rows[i]] - base[rows[i]];
            initialFree[i] = options.initialFree?.[rows[i]] ?? 0;
        }
        const localGroups = groups.filter(group => group.normalRow == null || selected.has(group.normalRow)).map(group => ({ ...group, rows: Array.from(group.rows, row => map[row]),
            lambda: Array.from(group.rows, (row, i) => group.lambda[i] + base[row]),
            normalRow: group.normalRow == null ? undefined : map[group.normalRow],
            normalLambda: group.normalRow == null ? group.normalLambda : group.normalLambda + base[group.normalRow] }));
        if (!workspace.diagonalRoundoff || workspace.diagonalRoundoff.length < nr)
            workspace.diagonalRoundoff = new Float64Array(grownCapacity(nr));
        const gramDiagonalRoundoff = workspace.diagonalRoundoff.subarray(0, nr);
        for (let i = 0; i < nr; i++) {
            const row = rows[i], c = couplings.get(row), column = responses.get(row);
            let magnitude = Math.abs(valueAt(matrix, band, row, row));
            for (let k = 0; k < c.length; k += 2) magnitude += Math.abs(c[k + 1] * column[c[k]]);
            // Include the eliminated solve and the cancellation in the Schur
            // diagonal. This bounds numerical scaling only, never edits A.
            gramDiagonalRoundoff[i] = 32 * Number.EPSILON * Math.max(1, ne + c.length) * magnitude;
        }
        const localOptions = { ...options, initialFree, gramDiagonalRoundoff };
        schurMs+=performance.now()-schurStarted;
        const contactStarted=performance.now();
        result = dynamic && options.simultaneousCoulomb ? solveSeededCoulombNewton(A, b, lo, hi, nr, nr, localGroups, localOptions)
            : dynamic ? solveCoupledLoadQP(A, b, lo, hi, nr, nr, localGroups, localOptions) : localGroups.length
            ? solveCoupledFrictionQP(A, b, lo, hi, nr, nr, localGroups, localOptions)
            : solveCoupledBandQP(A, b, lo, hi, nr, nr, localOptions);
        contactSolveMs+=performance.now()-contactStarted;
        seedMs+=result.diagnostics.seedMs??0;
        const reconstructionStarted=performance.now();
        factors += result.diagnostics.factorizations; iterations += result.diagnostics.iterations;
        reconstruct(rows, result.increment);
        // Re-map final loads to the original force coordinates for the final
        // certificate. Geometry/world will additionally check the actual J dx.
        finalGroups = groups.map(group => ({ ...group, radii: group.normalRow == null ? group.radii :
            group.mu.map(mu => mu * Math.max(0, group.normalLambda + x[group.normalRow])) }));
        kkt = dynamic ? measureCoupledLoadKKT(residual, x, lower, upper, finalGroups)
            : measureCoupledFrictionKKT(residual, x, lower, upper, finalGroups);
        reconstructionMs+=performance.now()-reconstructionStarted;
        if (kkt.maximumResidual <= tolerance || !result.diagnostics.converged || !addViolated()) break;
        if (expansions + 1 < limit) {
            if (!workspace.previousSchur || workspace.previousSchur.length < A.length)
                workspace.previousSchur = new Float64Array(grownCapacity(A.length, workspace.previousSchur?.length));
            workspace.previousSchur.set(A);
            previousCount = nr;
            previousMap.clear();
            rows.forEach((row, i) => previousMap.set(row, i));
        }
    }
    const fullFree = new Uint8Array(count);
    equality.forEach(row => { fullFree[row] = 1; });
    rows.forEach((row, i) => { fullFree[row] = result.free[i]; });
    const converged = kkt.maximumResidual <= tolerance;
    const finalLower = Float64Array.from(lower), finalUpper = Float64Array.from(upper);
    for (const group of finalGroups) if (group.radii.some(radius => radius === 0)) group.rows.forEach((row, axis) => {
        finalLower[row] = -group.radii[axis] - group.lambda[axis]; finalUpper[row] = group.radii[axis] - group.lambda[axis];
    });
    return { increment: x, residual, free: fullFree, lower: finalLower, upper: finalUpper,
        allGroups: finalGroups, groups: finalGroups.filter(group => group.radii.every(radius => radius > 0)), diagnostics: { ...result.diagnostics,
        converged, status: converged ? 'converged' : !result.diagnostics.converged ? result.diagnostics.status : 'condensed-full-residual',
        maximumResidual: kkt.maximumResidual, frictionResidual: kkt.groupResidual, factorizations: factors, iterations,
        condensedCosts:{setupMs,schurMs,contactSolveMs,reconstructionMs,seedMs},
        originalCount: count, equalityCount: ne, equalityBand, retainedCount: rows.length,
        responseColumns: responses.size, responseKernelCalls, responseBatches, schurEvaluations, schurReuses, expansions: expansions + 1 } };
}

/** Exact static condensation for a LOCAL dense symmetric positive definite
 * block. No diagonal shifts or dropped couplings. All outputs own their data.
 */
export function condenseKirchhoffAxialBlock({ matrix, rhs, count, retainedIndices }) {
    if (!Number.isSafeInteger(count) || count < 0 || !Number.isSafeInteger(count * count))
        throw new RangeError('Invalid block count');
    if (!matrix || matrix.length !== count * count || !Array.from(matrix).every(Number.isFinite)
        || !rhs || rhs.length !== count || !Array.from(rhs).every(Number.isFinite))
        throw new TypeError('Matrix and rhs must have finite entries and exact dimensions');
    if (!retainedIndices || typeof retainedIndices[Symbol.iterator] !== 'function')
        throw new TypeError('Explicit retainedIndices are required');
    const retained = Array.from(retainedIndices);
    if (retained.some(i => !Number.isInteger(i) || i < 0 || i >= count) || new Set(retained).size !== retained.length)
        throw new RangeError('Retained indices must be unique and within the block');
    const H = Float64Array.from(matrix), b = Float64Array.from(rhs);
    for (let i = 0; i < count; i++) for (let j = 0; j < i; j++) {
        if (H[i * count + j] !== H[j * count + i]) throw new RangeError('Matrix must be symmetric');
    }
    const keep = new Set(retained), eliminated = Array.from({ length: count }, (_, i) => i).filter(i => !keep.has(i));
    const ne = eliminated.length, nr = retained.length;
    // Translational offsets have only their own mass term when adaptation
    // constraints are hard. Detect exact independence; even tiny couplings
    // must retain the general factorization.
    let diagonalOnly = true;
    for (let i = 0; i < ne; i++) for (let j = 0; j < i; j++)
        if (H[eliminated[i] * count + eliminated[j]] !== 0) diagonalOnly = false;
    const scales = new Float64Array(ne), L = diagonalOnly ? null : new Float64Array(ne * ne);
    for (let i = 0; i < ne; i++) {
        const diagonal = H[eliminated[i] * count + eliminated[i]];
        if (!(diagonal > 0)) throw new RangeError('Eliminated block must be positive definite');
        scales[i] = Math.sqrt(diagonal);
    }
    if (!diagonalOnly) for (let i = 0; i < ne; i++) for (let j = 0; j <= i; j++) {
        let value = (H[eliminated[i] * count + eliminated[j]] / scales[i]) / scales[j];
        for (let k = 0; k < j; k++) value -= L[i * ne + k] * L[j * ne + k];
        if (i === j) {
            if (!(value > 0) || !Number.isFinite(value)) throw new RangeError('Eliminated block is singular or not positive definite');
            L[i * ne + j] = Math.sqrt(value);
        } else L[i * ne + j] = value / L[j * ne + j];
    }
    const solve = input => {
        if (!input || input.length !== ne || !Array.from(input).every(Number.isFinite))
            throw new TypeError('Eliminated rhs must have finite entries and exact dimensions');
        const x = new Float64Array(ne);
        if (diagonalOnly) {
            for (let i = 0; i < ne; i++) x[i] = input[i] / H[eliminated[i] * count + eliminated[i]];
            if (!x.every(Number.isFinite)) throw new RangeError('Eliminated solve overflow');
            return x;
        }
        for (let i = 0; i < ne; i++) {
            let value = input[i] / scales[i];
            for (let j = 0; j < i; j++) value -= L[i * ne + j] * x[j];
            x[i] = value / L[i * ne + i];
        }
        for (let i = ne - 1; i >= 0; i--) {
            let value = x[i];
            for (let j = i + 1; j < ne; j++) value -= L[j * ne + i] * x[j];
            x[i] = value / L[i * ne + i];
        }
        for (let i = 0; i < ne; i++) x[i] /= scales[i];
        if (!x.every(Number.isFinite)) throw new RangeError('Eliminated solve overflow');
        return x;
    };
    const free = solve(eliminated.map(i => b[i]));
    const responses = retained.map(j => solve(eliminated.map(i => H[i * count + j])));
    const schur = new Float64Array(nr * nr), condensedRhs = new Float64Array(nr);
    for (let i = 0; i < nr; i++) {
        let r = b[retained[i]];
        for (let k = 0; k < ne; k++) r -= H[retained[i] * count + eliminated[k]] * free[k];
        condensedRhs[i] = r;
        for (let j = 0; j <= i; j++) {
            let value = H[retained[i] * count + retained[j]];
            for (let k = 0; k < ne; k++) value -= H[retained[i] * count + eliminated[k]] * responses[j][k];
            schur[i * nr + j] = schur[j * nr + i] = value;
        }
    }
    if (!schur.every(Number.isFinite) || !condensedRhs.every(Number.isFinite)) throw new RangeError('Condensation overflow');
    const recover = retainedSolution => {
        if (!retainedSolution || retainedSolution.length !== nr || !Array.from(retainedSolution).every(Number.isFinite))
            throw new TypeError('Retained solution must have finite entries and exact dimensions');
        const x = new Float64Array(count);
        retained.forEach((node, i) => { x[node] = retainedSolution[i]; });
        eliminated.forEach((node, i) => {
            let value = free[i];
            for (let j = 0; j < nr; j++) value -= responses[j][i] * retainedSolution[j];
            x[node] = value;
        });
        if (!x.every(Number.isFinite)) throw new RangeError('Recovery overflow');
        return x;
    };
    const certify = solution => {
        if (!solution || solution.length !== count || !Array.from(solution).every(Number.isFinite))
            throw new TypeError('Full solution must have finite entries and exact dimensions');
        const residual = new Float64Array(count);
        let maximumResidual = 0, normalizedResidual = 0;
        for (let i = 0; i < count; i++) {
            let sum = -b[i], scale = Math.abs(b[i]);
            for (let j = 0; j < count; j++) {
                const term = H[i * count + j] * solution[j]; sum += term; scale += Math.abs(term);
            }
            residual[i] = sum;
            maximumResidual = Math.max(maximumResidual, Math.abs(sum));
            const relative = scale > 0 ? Math.abs(sum) / scale : sum === 0 ? 0 : Infinity;
            normalizedResidual = !Number.isFinite(sum) || !Number.isFinite(scale) ? Infinity : Math.max(normalizedResidual, relative);
        }
        return { residual, maximumResidual, normalizedResidual };
    };
    return { matrix: schur, rhs: condensedRhs, count: nr, retainedIndices: [...retained], eliminatedIndices: [...eliminated],
        recover, certify, solveEliminated: solve,
        diagnostics: { eliminatedSolver: diagonalOnly ? 'diagonal' : 'cholesky', factorEntries: L?.length ?? 0 } };
}

import { createKirchhoffLinearKernel } from './kirchhoffLinearKernel.js';

/** Validate storage without allocating or deriving another row layout. */
export function validateCoulombGeneralBandMatrix(matrix,count) {
    if(!Number.isSafeInteger(count)||count<0||!(matrix?.values instanceof Float64Array)||
        !['starts','ends','offsets'].every(key=>matrix[key] instanceof Int32Array&&matrix[key].length===count)||
        !matrix.values.every(Number.isFinite))throw new RangeError('general-band requires finite values and complete Int32 row metadata');
    for(let i=0;i<count;i++){
        const first=matrix.starts[i],last=matrix.ends[i],offset=matrix.offsets[i];
        if(first<0||first>i||last<i||last>=count||offset+first<0||offset+last>=matrix.values.length)
            throw new RangeError('general-band row bounds must contain the diagonal and valid storage');
    }
}

/** Exact row envelopes of the full Coulomb Jacobian. Each friction pair
 * mixes two mobility rows and its normal column; no transpose coupling is
 * introduced. Exact nonzeros, including arbitrarily small ones, are kept.
 */
export function createCoulombBandLayout(matrix, count, band, groups, matrixFormat='symmetric-band') {
    if(matrixFormat==='general-band')validateCoulombGeneralBandMatrix(matrix,count);
    const starts = Int32Array.from({ length: count }, (_, i) => i), ends = starts.slice();
    if(matrixFormat==='general-band') {
        for(let i=0;i<count;i++){
            const first=matrix.starts[i],last=matrix.ends[i],offset=matrix.offsets[i];
            for(let j=first;j<=last;j++)if(matrix.values[offset+j]!==0){starts[i]=Math.min(starts[i],j);ends[i]=Math.max(ends[i],j);}
        }
        for(const group of groups)if(group.rows?.length!==2||group.rows[0]===group.rows[1]||
            !Array.from(group.rows).every(i=>Number.isInteger(i)&&i>=0&&i<count)||
            group.normalRow!=null&&(!Number.isInteger(group.normalRow)||group.normalRow<0||group.normalRow>=count))
            throw new RangeError('general-band friction rows need valid distinct indices');
    } else if(matrixFormat==='row-major') {
        for(let i=0;i<count;i++)for(let j=0;j<count;j++)if(matrix[i*count+j]!==0){starts[i]=Math.min(starts[i],j);ends[i]=Math.max(ends[i],j);}
    } else if(matrixFormat==='symmetric-band')for (let i = 0; i < count; i++) for (let j = Math.max(0, i - band + 1); j < i; j++) {
        if (matrix[i * band + i - j] !== 0) { starts[i] = Math.min(starts[i], j); ends[j] = Math.max(ends[j], i); }
    } else throw new RangeError('Unknown Coulomb band matrix format');
    for (const group of groups) {
        const [i, j] = group.rows, normal = group.normalRow ?? i;
        const first = Math.min(starts[i], starts[j], i, j, normal), last = Math.max(ends[i], ends[j], i, j, normal);
        starts[i] = starts[j] = first; ends[i] = ends[j] = last;
    }
    const offsets = new Int32Array(count);
    let entries = 0, kl = 0, ku = 0;
    for (let i = 0; i < count; i++) {
        offsets[i] = entries - starts[i]; entries += ends[i] - starts[i] + 1;
        kl = Math.max(kl, i - starts[i]); ku = Math.max(ku, ends[i] - i);
    }
    return { starts, ends, offsets, entries, kl, ku };
}

/** General band Gaussian elimination with partial row pivoting. The factor
 * reserves all KL additional upper fill diagonals caused by row interchanges.
 * The kernel eliminates the RHS alongside the matrix (one Newton RHS), so
 * it never needs to swap previously stored L entries outside this band.
 * No positive-definiteness assumption or diagonal pivot replacement is used.
 */
export function createCoulombBandLU(layout, count) {
    const { starts, ends, offsets, kl, ku } = layout, stride = 2 * kl + ku + 1;
    const kernel = createKirchhoffLinearKernel(8 * count * stride + 16 * count + 128);
    const factor = kernel.alloc(Float64Array, count * stride), rhs = kernel.alloc(Float64Array, count);
    const right = kernel.alloc(Int32Array, count);
    const diagnostics = { linearSolver: 'band-lu', jacobianEntries: layout.entries,
        factorEntries: factor.length, lowerBandwidth: kl, upperBandwidth: ku, rowSwaps: 0,
        maximumLinearBackwardError: 0, maximumPivotGrowth: 0, linearResidualFailures: 0 };
    return { diagnostics,
        solve(J, F, scales, shift, direction) {
            factor.fill(0); right.set(ends);
            let originalMaximum = 0;
            for (let i = 0; i < count; i++) {
                rhs[i] = -F[i] * scales[i];
                const target = i * stride + kl - i, source = offsets[i];
                for (let j = starts[i]; j <= ends[i]; j++) factor[target + j] = J[source + j] * scales[i] * scales[j];
                factor[target + i] += shift;
                for (let j = starts[i]; j <= ends[i]; j++) originalMaximum = Math.max(originalMaximum, Math.abs(factor[target + j]));
            }
            const swaps = kernel.solveGeneralBandLU(factor.byteOffset, rhs.byteOffset, right.byteOffset, count, kl, ku);
            if (swaps < 0) return false;
            diagnostics.rowSwaps += swaps;
            let residualNorm = 0, matrixNorm = 0, rhsNorm = 0, solutionNorm = 0, factorMaximum = 0;
            for (let i = 0; i < count; i++) {
                let error = F[i] * scales[i], rowNorm = 0;
                for (let j = starts[i]; j <= ends[i]; j++) {
                    const a = J[offsets[i] + j] * scales[i] * scales[j] + (i === j ? shift : 0);
                    error += a * rhs[j]; rowNorm += Math.abs(a);
                }
                residualNorm = Math.max(residualNorm, Math.abs(error)); matrixNorm = Math.max(matrixNorm, rowNorm);
                rhsNorm = Math.max(rhsNorm, Math.abs(F[i] * scales[i])); solutionNorm = Math.max(solutionNorm, Math.abs(rhs[i]));
                const base = i * stride + kl - i;
                for (let j = i; j <= right[i]; j++) factorMaximum = Math.max(factorMaximum, Math.abs(factor[base + j]));
            }
            const backwardError = residualNorm / Math.max(Number.MIN_VALUE, matrixNorm * solutionNorm + rhsNorm);
            diagnostics.maximumLinearBackwardError = Math.max(diagnostics.maximumLinearBackwardError, backwardError);
            diagnostics.maximumPivotGrowth = Math.max(diagnostics.maximumPivotGrowth, factorMaximum / Math.max(Number.MIN_VALUE, originalMaximum));
            // Numerical backward error certifies this shifted Newton system,
            // independently of physical KKT. Partial pivoting bounds each
            // multiplier but cannot guarantee small element growth for every
            // nonsymmetric matrix. An unreliable solve uses the existing
            // gradient fallback; it never changes a physical equation or tol.
            if (!Number.isFinite(backwardError) || backwardError > 64 * Math.max(1, count) * Number.EPSILON) {
                diagnostics.linearResidualFailures++; return false;
            }
            for (let i = 0; i < count; i++) direction[i] = rhs[i];
            return direction.every(Number.isFinite);
        }
    };
}

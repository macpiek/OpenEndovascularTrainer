import { deleteKirchhoffCholeskyRow } from './kirchhoffCholeskyDelete.js';
import { createKirchhoffLinearKernel } from './kirchhoffLinearKernel.js';
import { fillKirchhoffGramMobilities } from './kirchhoffGramScaling.js';

function grownCapacity(required, current = 0) {
    let capacity = Math.max(16, current);
    while (capacity < required) capacity *= 2;
    return capacity;
}

/** Banded convex bound QP in the dual Gram matrix, never indefinite KKT.
 * A tiny diagonal term is a NUMERICAL proximal preconditioner. Every residual
 * and stopping test uses the original unregularized equations. Repeated solves
 * refine the same QP, so neither physical compliance nor force is softened.
 * Redundant rows remain inequalities and can enter/leave the working set.
 * Failure to meet the original KKT test is explicitly returned, never applied.
 */
export function solveCoupledBandQP(matrix, rhs, lower, upper, count, band, options = {}) {
    const tolerance = options.tolerance ?? 1e-8;
    const numericalShift = options.numericalShift ?? 1e-12;
    const limit = options.maxActiveSetIterations ?? Math.max(32, count * 2);
    const refinementLimit = options.maxRefinementIterations ?? 16;
    const skyline = options.factorization !== 'band';
    if (!(tolerance > 0) || !(numericalShift >= 0)) throw new RangeError('Invalid coupled solver tolerances');
    const workspace = options.workspace ?? {};
    if (!workspace.kernel || workspace.capacity < count || workspace.entries < matrix.length) {
        const previousFree = workspace.free;
        workspace.capacity = grownCapacity(count, workspace.capacity);
        workspace.entries = grownCapacity(matrix.length, workspace.entries);
        workspace.kernel = createKirchhoffLinearKernel(workspace.entries * 16 + workspace.capacity * 96 + 128);
        for (const key of ['factor', 'scaled']) workspace[key] = workspace.kernel.alloc(Float64Array, workspace.entries);
        for (const key of ['scale', 'x', 'lo', 'hi', 'residual', 'direction', 'b', 'solveRhs']) workspace[key] = workspace.kernel.alloc(Float64Array, workspace.capacity);
        workspace.free = workspace.kernel.alloc(Uint8Array, workspace.capacity);
        if (previousFree) workspace.free.set(previousFree);
        workspace.freeRows = workspace.kernel.alloc(Int32Array, workspace.capacity);
        workspace.rowMap = workspace.kernel.alloc(Int32Array, workspace.capacity);
        workspace.factorStarts = workspace.kernel.alloc(Int32Array, workspace.capacity);
        workspace.sourceStarts = workspace.kernel.alloc(Int32Array, workspace.capacity);
    }
    if(options.choleskyDeletes&&band===count&&(!workspace.deleteRows||workspace.deleteRows.length<count)){
        workspace.deleteRows=new Int32Array(workspace.capacity);
        workspace.deleteFactor=new Float64Array(workspace.capacity*workspace.capacity);
        workspace.deleteScratch={};
    }
    const { kernel, scale, x, lo, hi, free, residual, factor, direction, scaled, b, solveRhs, freeRows, rowMap, factorStarts, sourceStarts } = workspace;
    fillKirchhoffGramMobilities(matrix, count, band, options.gramDiagonalRoundoff, scale);
    for (let i = 0; i < count; i++) {
        scale[i] = 1 / Math.sqrt(scale[i]);
        lo[i] = lower[i] / scale[i]; hi[i] = upper[i] / scale[i];
        x[i] = Math.max(lo[i], Math.min(hi[i], 0));
        b[i] = rhs[i] * scale[i];
        if (matrix[i * band] === 0) {
            if (b[i] < 0 && Number.isFinite(lo[i])) x[i] = lo[i];
            if (b[i] > 0 && Number.isFinite(hi[i])) x[i] = hi[i];
        }
        free[i] = Number(matrix[i * band] > 0 && lo[i] !== hi[i] && (lo[i] === -Infinity && hi[i] === Infinity || x[i] > lo[i] && x[i] < hi[i] ||
            options.warmStart !== false && (options.initialFree ? options.initialFree[i] : free[i])));
    }
    scaled.set(matrix);
    kernel.scaleBandProfile(scaled.byteOffset, scale.byteOffset, sourceStarts.byteOffset, count, band);
    const multiplyResidual = () => {
        kernel.multiplyBandProfile(scaled.byteOffset, x.byteOffset, residual.byteOffset, count, band, scale.byteOffset, 0, sourceStarts.byteOffset);
        for (let i = 0; i < count; i++) residual[i] = b[i] - residual[i];
    };
    let iterations = 0, nearNullPivots = 0, status = 'iteration-limit', factorizations = 0;
    let maximumResidual = Infinity, factorValid = false, refinements = 0, refinementsOnFactor = 0, factorCount = 0, factorBand = 1;
    let factorProfileEntries = 0, factorUpdates=0, factorFromDelete=false;
    for (; iterations < limit; iterations++) {
        multiplyResidual();
        maximumResidual = 0;
        let freeResidual = 0, movableResidual = 0;
        for (let i = 0; i < count; i++) if (free[i]) freeResidual = Math.max(freeResidual, Math.abs(residual[i] / scale[i]));
        for (let i = 0; i < count; i++) {
            const r = residual[i] / scale[i];
            const violation = lo[i] === hi[i] ? 0 : x[i] <= lo[i] ? Math.max(0, r)
                : x[i] >= hi[i] ? Math.max(0, -r) : Math.abs(r);
            maximumResidual = Math.max(maximumResidual, violation);
            if (matrix[i * band] > 0) movableResidual = Math.max(movableResidual, violation);
            // Finish the current equality subproblem before releasing new
            // bounds. This avoids cycling on redundant active inequalities.
            if (matrix[i * band] > 0 && freeResidual <= tolerance && !free[i] && violation > tolerance && lo[i] !== hi[i]) { free[i] = 1; factorValid = false; }
        }
        if (maximumResidual <= tolerance) { status = 'converged'; break; }
        if (movableResidual <= tolerance) { status = 'immovable-constraints'; break; }
        if (!factorValid) {
            // Factor only the free principal matrix. Bound rows retain their
            // current multipliers in the FULL residual above; compression is
            // algebraic working-set elimination, never inequality removal.
            let updated=false;
            if(options.choleskyDeletes&&band===count&&factorCount>1){
                const next=workspace.deleteRows;let nextCount=0;
                for(let i=0;i<count;i++)if(free[i])next[nextCount++]=i;
                if(nextCount===factorCount-1){
                    let removed=-1,j=0,compatible=true;
                    for(let i=0;i<factorCount;i++){
                        if(j<nextCount&&freeRows[i]===next[j])j++;
                        else if(removed<0)removed=i;
                        else {compatible=false;break;}
                    }
                    if(compatible&&j===nextCount&&removed>=0)
                        updated=deleteKirchhoffCholeskyRow(factor,factorCount,factorBand,removed,
                            workspace.deleteFactor,workspace.deleteScratch);
                    if(updated){
                        factor.set(workspace.deleteFactor.subarray(0,nextCount*nextCount));
                        factorBand=nextCount;factorStarts.fill(0,0,nextCount);factorUpdates++;
                    }
                }
            }
            factorCount = 0; rowMap.fill(-1, 0, count);
            for (let i = 0; i < count; i++) if (free[i]) { rowMap[i] = factorCount; freeRows[factorCount++] = i; }
            if(!updated){
                // All other working-set changes use the original factorization.
                factorBand = kernel.compressFreeBand(scaled.byteOffset, factor.byteOffset, freeRows.byteOffset, rowMap.byteOffset,
                    sourceStarts.byteOffset, factorStarts.byteOffset, factorCount, band, numericalShift);
                if (skyline) kernel.factorSkyline(factor.byteOffset, factorCount, factorBand, factorStarts.byteOffset);
                else kernel.factorBand(factor.byteOffset, factorCount, factorBand, -1, 0);
                factorizations++;
            }
            factorValid = true; factorFromDelete=updated; refinementsOnFactor = 0;
            let small = 0; factorProfileEntries = 0;
            for (let i = 0; i < factorCount; i++) {
                if (factor[i * factorBand] ** 2 < Math.max(1e-12, numericalShift) * 32) small++;
                factorProfileEntries += i - factorStarts[i] + 1;
            }
            nearNullPivots = Math.max(nearNullPivots, small);
        }
        if (refinementsOnFactor++ >= refinementLimit) {
            if(factorFromDelete){factorValid=false;factorFromDelete=false;continue;}
            status = 'refinement-limit'; break;
        }
        if (refinementsOnFactor > 1) refinements++;
        for (let i = 0; i < factorCount; i++) solveRhs[i] = residual[freeRows[i]];
        if (skyline) kernel.solveSkyline(factor.byteOffset, solveRhs.byteOffset, factorCount, factorBand, factorStarts.byteOffset);
        else kernel.solveBand(factor.byteOffset, solveRhs.byteOffset, factorCount, factorBand);
        direction.fill(0, 0, count);
        for (let i = 0; i < factorCount; i++) direction[freeRows[i]] = solveRhs[i];
        let step = 1;
        for (let i = 0; i < count; i++) {
            if (direction[i] < 0 && Number.isFinite(lo[i])) step = Math.min(step, (lo[i] - x[i]) / direction[i]);
            if (direction[i] > 0 && Number.isFinite(hi[i])) step = Math.min(step, (hi[i] - x[i]) / direction[i]);
        }
        if (!Number.isFinite(step)) { status = 'non-finite-direction'; break; }
        step = Math.max(0, step);
        for (let i = 0; i < count; i++) {
            const lowerStep = direction[i] < 0 ? (lo[i] - x[i]) / direction[i] : Infinity;
            const upperStep = direction[i] > 0 ? (hi[i] - x[i]) / direction[i] : Infinity;
            x[i] = Math.max(lo[i], Math.min(hi[i], x[i] + step * direction[i]));
            // Identify the hit by its line parameter, not an absolute force
            // cutoff. This preserves arbitrarily small valid multipliers.
            if (lowerStep <= step * (1 + 8 * Number.EPSILON)) { x[i] = lo[i]; free[i] = 0; factorValid = false; }
            if (upperStep <= step * (1 + 8 * Number.EPSILON)) { x[i] = hi[i]; free[i] = 0; factorValid = false; }
        }
    }
    multiplyResidual();
    maximumResidual = 0;
    for (let i = 0; i < count; i++) {
        x[i] = x[i] === lo[i] ? lower[i] : x[i] === hi[i] ? upper[i] : x[i] * scale[i];
        residual[i] /= scale[i];
        const violation = lower[i] === upper[i] ? 0 : x[i] <= lower[i] ? Math.max(0, residual[i])
            : x[i] >= upper[i] ? Math.max(0, -residual[i]) : Math.abs(residual[i]);
        maximumResidual = Math.max(maximumResidual, violation);
    }
    if (!Number.isFinite(maximumResidual)) status = 'non-finite-direction';
    else if (maximumResidual <= tolerance) status = 'converged';
    return { increment: x.subarray(0, count), residual: residual.subarray(0, count), free: free.subarray(0, count), diagnostics: { status, converged: status === 'converged', iterations,
        factorizations, factorUpdates, refinements, nearNullPivots, numericalShift, maximumResidual, rowCount: count, band, matrixEntries: count * band, factorRowCount: factorCount, factorBand, factorEntries: factorCount * factorBand,
        factorization: skyline ? 'skyline' : 'band', factorProfileEntries } };
}

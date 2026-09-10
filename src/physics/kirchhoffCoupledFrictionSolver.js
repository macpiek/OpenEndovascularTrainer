import { createKirchhoffLinearKernel } from './kirchhoffLinearKernel.js';
import { solveCoupledBandQP } from './kirchhoffCoupledLinearSolver.js';

function grownCapacity(required, current = 0) {
    let capacity = Math.max(16, current);
    while (capacity < required) capacity *= 2;
    return capacity;
}

/** Euclidean projection onto a fixed-load ellipse. */
export function projectCoupledEllipse(u, v, a, b, out = [0, 0]) {
    if (a === 0 || b === 0) { out[0] = Math.max(-a, Math.min(a, u)); out[1] = Math.max(-b, Math.min(b, v)); return out; }
    if ((u / a) ** 2 + (v / b) ** 2 <= 1) { out[0] = u; out[1] = v; return out; }
    let lo = 0, hi = Math.hypot(a * u, b * v);
    for (let i = 0; i < 64; i++) {
        const s = (lo + hi) * 0.5;
        const x = a * u / (a * a + s), y = b * v / (b * b + s);
        if (x * x + y * y > 1) lo = s; else hi = s;
    }
    out[0] = a * a * u / (a * a + hi); out[1] = b * b * v / (b * b + hi);
    return out;
}

/** KKT residual of boxes plus disjoint fixed-load disks/ellipses, in original
 * equation units. A boundary residual parallel to the OUTWARD ellipse normal
 * is legitimate sliding; inward or tangential residuals are not stationary. */
export function measureCoupledFrictionKKT(residual, x, lower, upper, groups, out = {}) {
    const grouped = out.grouped?.length === x.length ? out.grouped : new Uint8Array(x.length);
    grouped.fill(0);
    let maximum = 0, groupResidual = 0;
    for (const group of groups) {
        const [i, j] = group.rows, [a, b] = group.radii;
        grouped[i] = grouped[j] = 1;
        const u = x[i] + group.lambda[0], v = x[j] + group.lambda[1];
        const norm = Math.hypot(u / a, v / b);
        let error;
        if (norm < 1 - 1e-10) error = Math.max(Math.abs(residual[i]), Math.abs(residual[j]));
        else {
            const nu = u / (a * a), nv = v / (b * b), length = Math.hypot(nu, nv);
            const nx = nu / length, ny = nv / length;
            const outward = residual[i] * nx + residual[j] * ny;
            error = Math.max(Math.abs(-ny * residual[i] + nx * residual[j]), Math.max(0, -outward));
        }
        if (norm > 1 + 1e-9) error = Infinity;
        groupResidual = Math.max(groupResidual, error);
    }
    for (let i = 0; i < x.length; i++) if (!grouped[i]) {
        const r = residual[i];
        const error = lower[i] === upper[i] ? 0 : x[i] <= lower[i] ? Math.max(0, r) : x[i] >= upper[i] ? Math.max(0, -r) : Math.abs(r);
        maximum = Math.max(maximum, error);
    }
    out.grouped = grouped; out.groupResidual = groupResidual;
    out.maximumResidual = Math.max(maximum, groupResidual); out.boxResidual = maximum;
    return out;
}

/** Full convex QP with fixed-load friction ellipses. Active ellipse pairs use
 * their one-dimensional force tangent; the removed radial variable is exactly
 * the active force-domain boundary, NOT a rod dof reduction. The positive
 * Lagrange curvature gives an SPD banded reduced Newton system. All material,
 * normal and friction increments are still reconstructed simultaneously.
 * No radius derivative with respect to normal load is included (non-associated).
 */
export function solveCoupledFrictionQP(matrix, rhs, lower, upper, count, band, groups, options = {}) {
    const tolerance = options.tolerance ?? 1e-8, limit = options.maxFrictionIterations ?? 80;
    const workspace = options.frictionWorkspace ?? {};
    if (!workspace.kernel || workspace.count < count || workspace.entries < matrix.length) {
        workspace.count = grownCapacity(count, workspace.count);
        workspace.entries = grownCapacity(matrix.length, workspace.entries);
        workspace.kernel = createKirchhoffLinearKernel(workspace.entries * 8 + workspace.count * 64 + 128);
        workspace.matrix = workspace.kernel.alloc(Float64Array, workspace.entries);
        for (const key of ['x', 'product', 'residual', 'trial', 'trialProduct']) workspace[key] = workspace.kernel.alloc(Float64Array, workspace.count);
        workspace.sourceStarts = workspace.kernel.alloc(Int32Array, workspace.count);
    }
    const { kernel, x, product, residual, trial, trialProduct, sourceStarts } = workspace;
    workspace.matrix.set(matrix);
    kernel.findBandStarts(workspace.matrix.byteOffset, sourceStarts.byteOffset, count, band);
    const multiply = (input, output) => kernel.multiplyBandProfile(workspace.matrix.byteOffset, input.byteOffset, output.byteOffset, count, band, input.byteOffset, 0, sourceStarts.byteOffset);
    if (!workspace.rowGroup || workspace.rowGroup.length < count) {
        for (const key of ['rowGroup', 'map']) workspace[key] = new Int32Array(workspace.count);
        for (const key of ['coefficients', 'reducedRhs', 'reducedLower', 'reducedUpper', 'curvature', 'direction']) workspace[key] = new Float64Array(workspace.count);
        for (const key of ['fullFree', 'reducedFree', 'grouped']) workspace[key] = new Uint8Array(workspace.count);
    }
    const { rowGroup, map, coefficients, reducedRhs, reducedLower, reducedUpper, curvature, direction, fullFree, reducedFree } = workspace;
    const states = workspace.states ??= [], statePool = workspace.statePool ??= [];
    const kkt = workspace.kkt ??= {}, projection = workspace.projection ??= [0, 0];
    const xView = x.subarray(0, count), residualView = residual.subarray(0, count), trialView = trial.subarray(0, count);
    kkt.grouped = workspace.grouped.subarray(0, count);
    rowGroup.fill(-1, 0, count);
    states.length = groups.length;
    for (let g = 0; g < groups.length; g++) {
        const source = groups[g], state = statePool[g] ??= { tangent: [0, 0] };
        // Refresh every force-domain identity/value, even when storage and
        // group count stay unchanged after contact permutation or reload.
        state.rows = source.rows; state.radii = source.radii; state.lambda = source.lambda;
        state.active = false; state.activationPending = false; state.angle = 0; state.speed = 1; state.eta = 0; state.variable = -1;
        states[g] = state;
        const [i, j] = state.rows; rowGroup[i] = rowGroup[j] = g;
    }
    for (let i = 0; i < count; i++) x[i] = Math.max(lower[i], Math.min(upper[i], 0));
    for (const group of states) {
        const [i, j] = group.rows, [a, b] = group.radii;
        const p = projectCoupledEllipse(group.lambda[0], group.lambda[1], a, b, projection);
        x[i] = p[0] - group.lambda[0]; x[j] = p[1] - group.lambda[1];
    }
    // Hints live in the fixed original row space. A sliding ellipse removes
    // one reduced row, so reduced indices cannot be retained across mappings.
    for (let i = 0; i < count; i++) fullFree[i] = options.initialFree?.[i] ?? 0;
    const innerOptions = { ...options, initialFree: reducedFree,
        workspace: workspace.reducedQP ??= {}, tolerance: tolerance * 0.25 };
    let iterations = 0, factorizations = 0, factorUpdates = 0, nearNullPivots = 0, innerIterations = 0, status = 'friction-iteration-limit';
    let lastObjective = Infinity, backtracks = 0, gradientFallbacks = 0, boundaryActivations = 0;
    for (; iterations < limit; iterations++) {
        multiply(x, product);
        let objective = 0;
        for (let i = 0; i < count; i++) { residual[i] = rhs[i] - product[i]; objective += x[i] * (0.5 * product[i] - rhs[i]); }
        measureCoupledFrictionKKT(residualView, xView, lower, upper, groups, kkt);
        if (kkt.maximumResidual <= tolerance) { status = 'converged'; break; }
        for (const group of states) {
            const [i, j] = group.rows, [a, b] = group.radii;
            const u = x[i] + group.lambda[0], v = x[j] + group.lambda[1];
            group.angle = Math.atan2(v / b, u / a);
            const nu = u / (a * a), nv = v / (b * b), n2 = nu * nu + nv * nv;
            group.eta = n2 ? Math.max(0, (nu * residual[i] + nv * residual[j]) / n2) : 0;
            const norm = Math.hypot(u / a, v / b);
            group.active = norm >= 1 - 1e-10 && (group.activationPending || nu * residual[i] + nv * residual[j] >= 0);
            group.activationPending = false;
            const tu = -a * Math.sin(group.angle), tv = b * Math.cos(group.angle);
            group.speed = Math.hypot(tu, tv);
            group.tangent[0] = tu / group.speed; group.tangent[1] = tv / group.speed;
        }
        let reducedCount = 0;
        map.fill(-1, 0, count); curvature.fill(0, 0, count); reducedRhs.fill(0, 0, count);
        for (let i = 0; i < count; i++) {
            if (map[i] >= 0) continue;
            const group = rowGroup[i] >= 0 ? states[rowGroup[i]] : null;
            const k = reducedCount++;
            if (group?.active) {
                const [u, v] = group.rows;
                map[u] = map[v] = k; coefficients[u] = group.tangent[0]; coefficients[v] = group.tangent[1];
                curvature[k] = group.eta / (group.speed * group.speed);
                reducedLower[k] = -0.5 * group.speed; reducedUpper[k] = 0.5 * group.speed;
                group.variable = k;
            } else {
                map[i] = k; coefficients[i] = 1;
                reducedLower[k] = lower[i] - x[i]; reducedUpper[k] = upper[i] - x[i];
                if (group) {
                    // Necessary component bounds keep an interior Newton
                    // target within the ellipse's enclosing box. The ACTUAL
                    // accepted force remains on/in the ellipse via the exact
                    // boundary intersection/retraction and ellipse KKT below.
                    // This avoids solving a remote, physically inadmissible
                    // friction load merely to cut its step almost to zero.
                    const axis = i === group.rows[0] ? 0 : 1;
                    reducedLower[k] = Math.max(reducedLower[k], -group.radii[axis] - group.lambda[axis] - x[i]);
                    reducedUpper[k] = Math.min(reducedUpper[k], group.radii[axis] - group.lambda[axis] - x[i]);
                }
            }
        }
        let reducedBand = 1;
        for (let i = 0; i < count; i++) {
            reducedRhs[map[i]] += coefficients[i] * residual[i];
            for (let j = sourceStarts[i]; j < i; j++) if (matrix[i * band + i - j]) reducedBand = Math.max(reducedBand, Math.abs(map[i] - map[j]) + 1);
        }
        const size = reducedCount * reducedBand;
        if (!workspace.reducedMatrix || workspace.reducedMatrix.length < size) workspace.reducedMatrix = new Float64Array(grownCapacity(size, workspace.reducedMatrix?.length));
        const reducedMatrix = workspace.reducedMatrix.subarray(0, size);
        reducedMatrix.fill(0);
        for (let i = 0; i < reducedCount; i++) reducedMatrix[i * reducedBand] = curvature[i];
        for (let i = 0; i < count; i++) for (let j = sourceStarts[i]; j <= i; j++) {
            const a = map[i], b = map[j], value = matrix[i * band + i - j] * coefficients[i] * coefficients[j];
            const row = Math.max(a, b), col = Math.min(a, b);
            reducedMatrix[row * reducedBand + row - col] += value * (a === b && i !== j ? 2 : 1);
        }
        reducedFree.fill(0, 0, reducedCount);
        for (let i = 0; i < count; i++) if (rowGroup[i] < 0) reducedFree[map[i]] = fullFree[i];
        const stepResult = solveCoupledBandQP(reducedMatrix, reducedRhs, reducedLower, reducedUpper, reducedCount, reducedBand, innerOptions);
        for (let i = 0; i < count; i++) if (rowGroup[i] < 0) fullFree[i] = stepResult.free[map[i]];
        factorizations += stepResult.diagnostics.factorizations; factorUpdates += stepResult.diagnostics.factorUpdates ?? 0; innerIterations += stepResult.diagnostics.iterations;
        nearNullPivots = Math.max(nearNullPivots, stepResult.diagnostics.nearNullPivots);
        for (let i = 0; i < count; i++) direction[i] = coefficients[i] * stepResult.increment[map[i]];
        let step = 1, slope = 0, activateBoundary = false;
        for (let i = 0; i < count; i++) slope -= residual[i] * direction[i];
        // Exact first intersection for every currently interior ellipse.
        for (const group of states) if (!group.active) {
            const [i, j] = group.rows, [a, b] = group.radii;
            const u = (x[i] + group.lambda[0]) / a, v = (x[j] + group.lambda[1]) / b;
            const du = direction[i] / a, dv = direction[j] / b;
            const aa = du * du + dv * dv, bb = u * du + v * dv, cc = u * u + v * v - 1;
            if (aa > 0 && (u + du) ** 2 + (v + dv) ** 2 > 1) {
                const root = (-bb + Math.sqrt(Math.max(0, bb * bb - aa * cc))) / aa;
                step = Math.min(step, Math.max(0, root));
                if (root < 1e-12 && Math.hypot(u, v) >= 1 - 1e-10) {
                    group.activationPending = true; activateBoundary = true; boundaryActivations++;
                }
            }
        }
        const innerResolved = stepResult.diagnostics.converged || stepResult.diagnostics.status === 'immovable-constraints' && stepResult.diagnostics.maximumResidual <= tolerance;
        let fallback = !(slope < 0) || step < 1e-12 || !innerResolved;
        options.debugFrictionIteration?.({ iteration: iterations, slope, step, fallback, innerResolved,
            matrix: reducedMatrix, rhs: reducedRhs.subarray(0, reducedCount), lower: reducedLower.subarray(0, reducedCount),
            upper: reducedUpper.subarray(0, reducedCount), count: reducedCount, band: reducedBand,
            map: map.subarray(0, count), coefficients: coefficients.subarray(0, count), result: stepResult,
            activateBoundary, states, x: xView, residual: residualView, direction: direction.subarray(0, count) });
        if (activateBoundary && innerResolved) {
            // A local inward gradient does not guarantee an inward coupled
            // Newton direction. Add any blocking boundary to the working set
            // and solve again at the SAME feasible force. Taking a projected
            // gradient step here can repeatedly release/reactivate that ellipse.
            for (const group of states) if (group.active) group.activationPending = true;
            continue;
        }
        if (fallback) {
            // Safeguard for a released boundary whose Newton direction points
            // outside: projected gradient is feasible and has descent. This
            // is one full-matrix product, never whole-rod mobility per contact.
            gradientFallbacks++;
            let norm = 0;
            for (let i = 0; i < count; i++) norm = Math.max(norm, matrix[i * band]);
            const rho = 1 / Math.max(norm, 1e-30);
            for (let i = 0; i < count; i++) direction[i] = Math.max(lower[i], Math.min(upper[i], x[i] + rho * residual[i]));
            for (const group of states) {
                const [i, j] = group.rows;
                const p = projectCoupledEllipse(direction[i] + group.lambda[0], direction[j] + group.lambda[1], ...group.radii, projection);
                direction[i] = p[0] - group.lambda[0]; direction[j] = p[1] - group.lambda[1];
            }
            slope = 0;
            for (let i = 0; i < count; i++) { direction[i] -= x[i]; slope -= residual[i] * direction[i]; }
            step = 1;
        }
        let accepted = false;
        for (let search = 0; search < 30; search++) {
            for (let i = 0; i < count; i++) trial[i] = Math.max(lower[i], Math.min(upper[i], x[i] + step * direction[i]));
            if (!fallback) for (const group of states) if (group.active) {
                const [i, j] = group.rows, angle = group.angle + step * stepResult.increment[group.variable] / group.speed;
                trial[i] = group.radii[0] * Math.cos(angle) - group.lambda[0];
                trial[j] = group.radii[1] * Math.sin(angle) - group.lambda[1];
            }
            multiply(trial, trialProduct);
            let value = 0;
            for (let i = 0; i < count; i++) value += trial[i] * (0.5 * trialProduct[i] - rhs[i]);
            if (Number.isFinite(value) && value <= objective + 1e-4 * step * slope + 64 * Number.EPSILON * Math.max(1e-300, Math.abs(objective), Math.abs(value))) {
                x.set(trialView); lastObjective = value; accepted = true; break;
            }
            step *= 0.5; backtracks++;
        }
        if (!accepted) { status = 'friction-line-search'; break; }
    }
    multiply(x, product);
    for (let i = 0; i < count; i++) residual[i] = rhs[i] - product[i];
    measureCoupledFrictionKKT(residualView, xView, lower, upper, groups, kkt);
    if (kkt.maximumResidual <= tolerance) status = 'converged';
    return { increment: xView, residual: residualView, free: fullFree.subarray(0, count), diagnostics: {
        status, converged: status === 'converged', iterations, factorizations, factorUpdates, innerIterations, nearNullPivots,
        maximumResidual: kkt.maximumResidual, frictionResidual: kkt.groupResidual, groupCount: groups.length,
        rowCount: count, band, matrixEntries: count * band, backtracks, gradientFallbacks, boundaryActivations, objective: lastObjective,
    } };
}

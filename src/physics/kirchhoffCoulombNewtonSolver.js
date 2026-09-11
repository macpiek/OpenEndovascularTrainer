import { projectCoupledEllipse } from './kirchhoffCoupledFrictionSolver.js';
import { measureCoupledLoadKKT, solveCoupledLoadQP } from './kirchhoffCoupledLoadSolver.js';
import { createCoulombBandLayout, createCoulombBandLU } from './kirchhoffCoulombBandLU.js';
import { createCoulombSectionLU } from './kirchhoffCoulombSectionLU.js';
import { fillKirchhoffGramMobilities } from './kirchhoffGramScaling.js';

/** One fixed-load QP supplies a mechanically informed starting point, then
 * the joint Newton equations close normal load and friction simultaneously.
 * This is local to this frozen system, never a force guess across timesteps.
 */
export function solveSeededCoulombNewton(matrix, rhs, lower, upper, count, band, groups, options = {}) {
    if (options.matrixFormat != null && options.matrixFormat !== 'symmetric-band')
        throw new RangeError('The fixed-load QP seed requires symmetric-band input; use solveCoulombNewton for row-major matrices');
    const seedLimit=options.maxSeedFrictionIterations;
    if(seedLimit!==undefined&&(!Number.isInteger(seedLimit)||seedLimit<1))
        throw new RangeError('Positive fixed-load seed iteration limit required');
    const seedActiveLimit=options.maxSeedActiveSetIterations;
    if(seedActiveLimit!==undefined&&(!Number.isInteger(seedActiveLimit)||seedActiveLimit<1))
        throw new RangeError('Positive fixed-load seed active-set iteration limit required');
    if (options.tryNewtonBeforeSeed) {
        const probeLimit = options.maxUnseededNewtonIterations ?? 8;
        if (!Number.isInteger(probeLimit) || probeLimit < 1)
            throw new RangeError('Positive unseeded Newton iteration limit required');
        const probeStarted = performance.now();
        const probe = solveCoulombNewton(matrix, rhs, lower, upper, count, band, groups,
            { ...options, maxCoulombNewtonIterations: Math.min(probeLimit, options.maxCoulombNewtonIterations ?? 60) });
        const probeMs = performance.now() - probeStarted;
        if (probe.diagnostics.converged) {
            Object.assign(probe.diagnostics, {unseededNewtonAccepted:true, unseededNewtonMs:probeMs, seedMs:0});
            return probe;
        }
        // The unsuccessful numerical iterate is never applied or used to
        // initialize fallback. Preserve the original fully certified route.
        const fallback = solveSeededCoulombNewton(matrix, rhs, lower, upper, count, band, groups,
            { ...options, tryNewtonBeforeSeed:false });
        for (const key of ['factorizations','factorUpdates','iterations','backtracks','gradientFallbacks','rowSwaps','linearResidualFailures'])
            if (key in probe.diagnostics || key in fallback.diagnostics)
                fallback.diagnostics[key] = (fallback.diagnostics[key] ?? 0) + (probe.diagnostics[key] ?? 0);
        Object.assign(fallback.diagnostics, {unseededNewtonAccepted:false, unseededNewtonMs:probeMs});
        return fallback;
    }
    const seedStarted=performance.now();
    const seed = solveCoupledLoadQP(matrix, rhs, lower, upper, count, band, groups,
        { ...options, maxNormalLoadIterations: 1,
            ...(seedLimit===undefined?{}:{maxFrictionIterations:seedLimit}),
            ...(seedActiveLimit===undefined?{}:{maxActiveSetIterations:seedActiveLimit}) });
    const seedMs=performance.now()-seedStarted;
    seed.diagnostics.seedMs=seedMs;
    if (seed.diagnostics.converged) return seed;
    let result = solveCoulombNewton(matrix, rhs, lower, upper, count, band, groups,
        { ...options, normalMap: options.normalMap ?? 'projection', initialIncrement: seed.increment });
    const attempts = [seed, result];
    if (!result.diagnostics.converged && !options.normalMap) {
        const projection = result;
        result = solveCoulombNewton(matrix, rhs, lower, upper, count, band, groups,
            { ...options, normalMap: 'fischer-burmeister', initialIncrement: projection.increment });
        attempts.push(result);
        result.diagnostics.normalMapStart = 'projection';
        if (!result.diagnostics.converged) {
            // A stalled complementarity branch is not necessarily a useful
            // starting point for the other map. Retry the same frozen system
            // from its original fixed-load seed. Neither failed iterate is
            // applied to a body, and all attempts use the same final KKT.
            result = solveCoulombNewton(matrix, rhs, lower, upper, count, band, groups,
                { ...options, normalMap: 'fischer-burmeister', initialIncrement: seed.increment });
            attempts.push(result);
            result.diagnostics.normalMapStart = 'seed';
        }
        result.diagnostics.normalMapFallback = true;
    }
    for (const key of ['factorizations', 'factorUpdates', 'iterations', 'backtracks', 'gradientFallbacks'])
        result.diagnostics[key] = attempts.reduce((sum, attempt) => sum + (attempt.diagnostics[key] ?? 0), 0);
    for (const key of ['rowSwaps', 'linearResidualFailures']) if (attempts.some(attempt => key in attempt.diagnostics))
        result.diagnostics[key] = attempts.reduce((sum, attempt) => sum + (attempt.diagnostics[key] ?? 0), 0);
    for (const key of ['maximumLinearBackwardError', 'maximumPivotGrowth']) if (attempts.some(attempt => key in attempt.diagnostics))
        result.diagnostics[key] = Math.max(...attempts.map(attempt => attempt.diagnostics[key] ?? 0));
    result.diagnostics.seedMs=seedMs;
    result.diagnostics.seedFactorizations = seed.diagnostics.factorizations;
    result.diagnostics.normalLoadIterations = 1;
    if (!result.diagnostics.converged && options.retryWithoutHints !== false && options.warmStart !== false) {
        // Redundant hard contacts can give the fixed-load QP several force
        // distributions. A retained active-set hint may seed a bad Coulomb
        // branch even though the unhinted solve converges. Discard only these
        // numerical hints and retry once; retained physical forces, equations
        // and acceptance tolerances remain identical.
        const retry = solveSeededCoulombNewton(matrix, rhs, lower, upper, count, band, groups,
            { ...options, initialFree: undefined, warmStart: false, retryWithoutHints: false });
        for (const key of ['factorizations', 'factorUpdates', 'iterations', 'backtracks', 'gradientFallbacks', 'normalLoadIterations'])
            retry.diagnostics[key] = (retry.diagnostics[key] ?? 0) + (result.diagnostics[key] ?? 0);
        for (const key of ['rowSwaps', 'linearResidualFailures']) if (key in retry.diagnostics || key in result.diagnostics)
            retry.diagnostics[key] = (retry.diagnostics[key] ?? 0) + (result.diagnostics[key] ?? 0);
        for (const key of ['maximumLinearBackwardError', 'maximumPivotGrowth']) if (key in retry.diagnostics || key in result.diagnostics)
            retry.diagnostics[key] = Math.max(retry.diagnostics[key] ?? 0, result.diagnostics[key] ?? 0);
        retry.diagnostics.seedMs=(retry.diagnostics.seedMs??0)+seedMs;
        retry.diagnostics.coldSeedRetry = true;
        retry.diagnostics.seedFactorizations = (retry.diagnostics.seedFactorizations ?? 0) + seed.diagnostics.factorizations;
        return retry;
    }
    return result;
}

/** Projection and its derivatives with respect to the input force and a
 * homothetic ellipse's normal load. The derivative of the load acts ONLY in
 * the friction law; the normal equilibrium is unchanged (no dilation).
 */
export function projectLoadEllipseDerivative(u, v, load, mu, out = {}) {
    const p = out.value ??= [0, 0], D = out.input ??= new Float64Array(4), d = out.load ??= [0, 0];
    D.fill(0); d[0] = d[1] = 0;
    const a = load * mu[0], b = load * mu[1];
    projectCoupledEllipse(u, v, a, b, p);
    if (load === 0) {
        const length = Math.hypot(mu[0] * u, mu[1] * v);
        if (length > 0) { d[0] = mu[0] * mu[0] * u / length; d[1] = mu[1] * mu[1] * v / length; }
    } else if (a === 0 || b === 0) {
        if (a > 0) { if (Math.abs(u) < a) D[0] = 1; else d[0] = Math.sign(u) * mu[0]; }
        if (b > 0) { if (Math.abs(v) < b) D[3] = 1; else d[1] = Math.sign(v) * mu[1]; }
    } else if (Math.hypot(u / a, v / b) <= 1) { D[0] = D[3] = 1; }
    else {
        // At the boundary p.(y-p)=eta because p.D^-2.p=1.
        const eta = Math.max(0, p[0] * (u - p[0]) + p[1] * (v - p[1]));
        const da = a * a + eta, db = b * b + eta, qa = p[0] / da, qb = p[1] / db;
        const denominator = p[0] * qa / (a * a) + p[1] * qb / (b * b);
        D[0] = a * a / da - qa * qa / denominator;
        D[1] = D[2] = -qa * qb / denominator;
        D[3] = b * b / db - qb * qb / denominator;
        d[0] = (p[0] - D[0] * u - D[1] * v) / load;
        d[1] = (p[1] - D[2] * u - D[3] * v) / load;
    }
    return out;
}

function solveDenseLU(A, b, n) {
    for (let k = 0; k < n; k++) {
        let pivot = k;
        for (let i = k + 1; i < n; i++) if (Math.abs(A[i * n + k]) > Math.abs(A[pivot * n + k])) pivot = i;
        if (!(Math.abs(A[pivot * n + k]) > 1e-15)) return false;
        if (pivot !== k) {
            for (let j = k; j < n; j++) { const t = A[k * n + j]; A[k * n + j] = A[pivot * n + j]; A[pivot * n + j] = t; }
            const t = b[k]; b[k] = b[pivot]; b[pivot] = t;
        }
        for (let i = k + 1; i < n; i++) {
            const q = A[i * n + k] / A[k * n + k];
            for (let j = k + 1; j < n; j++) A[i * n + j] -= q * A[k * n + j];
            b[i] -= q * b[k];
        }
    }
    for (let i = n - 1; i >= 0; i--) {
        for (let j = i + 1; j < n; j++) b[i] -= A[i * n + j] * b[j];
        b[i] /= A[i * n + i];
    }
    return b.every(Number.isFinite);
}

/** Simultaneous non-associated Coulomb root solve in the retained contact
 * system. The natural-map equations use exact box/ellipse projections. A
 * nonsymmetric Newton Jacobian includes the force-domain radius derivative,
 * without adding its transpose to the normal equilibrium. Trial forces are
 * numerical iterates only; success requires the ORIGINAL full load KKT and
 * cone feasibility. No trial can be applied to bodies without that check.
 * matrixFormat defaults to 'symmetric-band'. Explicit 'row-major' accepts
 * count*count finite entries of a generally nonsymmetric operator, ignores
 * band. It uses dense LU by default, or exact directed row envelopes with
 * coulombLinearSolver:'band-lu', without an SPD fixed-load QP seed.
 */
export function solveCoulombNewton(matrix, rhs, lower, upper, count, band, groups, options = {}) {
    const n = count, tolerance = options.tolerance ?? 1e-8, limit = options.maxCoulombNewtonIterations ?? 60;
    const matrixFormat = options.matrixFormat ?? 'symmetric-band', rowMajor = matrixFormat === 'row-major', compact = matrixFormat === 'general-band', general = rowMajor || compact;
    if (!general && matrixFormat !== 'symmetric-band') throw new RangeError('Unknown Coulomb matrixFormat');
    if (rowMajor && (!Number.isInteger(n) || n < 0 || matrix.length !== n * n || !matrix.every(Number.isFinite)))
        throw new RangeError('row-major requires count*count finite matrix entries');
    const layout = compact || options.localSections || options.coulombLinearSolver === 'band-lu' ? createCoulombBandLayout(matrix, n, band, groups, matrixFormat) : null;
    const starts = layout?.starts ?? new Int32Array(n), ends = layout?.ends ?? new Int32Array(n).fill(n - 1);
    const offsets = layout?.offsets ?? Int32Array.from({ length: n }, (_, i) => i * n);
    const bandLU = layout ? options.localSections ? createCoulombSectionLU(layout, n, options.localSections, options.sectionWorkspace)
        : createCoulombBandLU(layout, n) : null, entries = layout?.entries ?? n * n;
    const A = new Float64Array(entries), x = new Float64Array(n), residual = new Float64Array(n), F = new Float64Array(n);
    const J = new Float64Array(entries), factor = bandLU ? null : new Float64Array(n * n), direction = new Float64Array(n), scales = new Float64Array(n);
    const trial = new Float64Array(n), trialF = new Float64Array(n), trialResidual = new Float64Array(n), scratch = {};
    const finalGroups = groups.map(group => ({ ...group, radii: group.radii ? [...group.radii] : [0, 0] }));
    if (general) for (const group of finalGroups) {
        group.rows = [...group.rows]; group.lambda = [...group.lambda];
        if (group.mu) group.mu = [...group.mu];
    }
    const frictionRows = new Uint8Array(n);
    const mobilities = new Float64Array(n), zeroRows = general ? new Uint8Array(n) : null;
    if (!general) fillKirchhoffGramMobilities(matrix, n, band, options.gramDiagonalRoundoff, mobilities);
    const initialIncrement = options.initialIncrement?.every(Number.isFinite) ? options.initialIncrement : undefined;
    for (const group of groups) for (const row of group.rows) frictionRows[row] = 1;
    for (let i = 0; i < n; i++) {
        x[i] = Math.max(lower[i], Math.min(upper[i], initialIncrement?.[i] ?? 0));
        if (general) {
            let maximum = 0;
            for (let j = starts[i]; j <= ends[i]; j++) {
                const value = compact ? (j>=matrix.starts[i]&&j<=matrix.ends[i]?matrix.values[matrix.offsets[i]+j]:0) : matrix[i * n + j]; A[offsets[i] + j] = value;
                maximum = Math.max(maximum, Math.abs(value));
            }
            // Projection steps require a positive scale, even when a general
            // operator has a zero or negative diagonal. This scales only the
            // root equations; the original operator and KKT remain intact.
            mobilities[i] = Math.abs(A[offsets[i] + i]) || maximum || 1;
            zeroRows[i] = Number(maximum === 0);
            scales[i] = 1 / Math.sqrt(mobilities[i]);
        } else {
            scales[i] = 1 / Math.sqrt(mobilities[i]);
            for (let j = Math.max(0, i - band + 1); j <= i; j++) {
                const value = matrix[i * band + i - j];
                if (!layout || value !== 0) A[offsets[i] + j] = A[offsets[j] + i] = value;
            }
        }
    }
    const kkt = {};
    function feasible(input, out, recoverBounds = false) {
        let recovered = 0;
        for (let i = 0; i < n; i++) {
            out[i] = Math.max(lower[i], Math.min(upper[i], input[i]));
            const mobility = mobilities[i];
            if (recoverBounds && !frictionRows[i] && mobility > 0) {
                // Use the exact natural-map branch, not a small-force cutoff.
                // FB cancellation can leave positive force at a separated
                // contact. Move that force to the actual active bound, then
                // recompute every coupled reaction and its friction cone.
                // Mixed hard-contact rows have zero diagonal. Use the same
                // positive root scale as evaluate(), including for these rows.
                const target = input[i] + residual[i] / mobility;
                if (target <= lower[i] && out[i] !== lower[i]) { out[i] = lower[i]; recovered++; }
                else if (target >= upper[i] && out[i] !== upper[i]) { out[i] = upper[i]; recovered++; }
            }
        }
        for (const group of groups) {
            const load = group.normalRow == null ? 1 : Math.max(0, group.normalLambda + out[group.normalRow]);
            const radii = group.normalRow == null ? group.radii : group.mu, [i, j] = group.rows;
            const p = projectCoupledEllipse(out[i] + group.lambda[0], out[j] + group.lambda[1], load * radii[0], load * radii[1], scratch.value ??= [0, 0]);
            out[i] = p[0] - group.lambda[0]; out[j] = p[1] - group.lambda[1];
        }
        return recovered;
    }
    function evaluate(input, f, r, jacobian) {
        let merit = 0;
        if (jacobian) jacobian.fill(0);
        for (let i = 0; i < n; i++) {
            let v = rhs[i]; for (let j = starts[i]; j <= ends[i]; j++) v -= A[offsets[i] + j] * input[j];
            r[i] = v;
            const rho = 1 / mobilities[i], target = input[i] + rho * v;
            let da, db;
            if (options.normalMap !== 'projection' && (Number.isFinite(lower[i]) && upper[i] === Infinity || lower[i] === -Infinity && Number.isFinite(upper[i]))) {
                // Fischer-Burmeister complementarity has a continuously
                // differentiable squared merit away from the double-zero.
                // It avoids a min-map line search stalling on a switching
                // plane while the trial normal force is still negative.
                const sign = Number.isFinite(lower[i]) ? 1 : -1;
                const a = sign * (input[i] - (sign > 0 ? lower[i] : upper[i])) / rho, b = -sign * v;
                const length = Math.hypot(a, b);
                f[i] = sign * (a + b - length);
                da = length ? 1 - a / length : 1 - Math.SQRT1_2;
                db = length ? 1 - b / length : 1 - Math.SQRT1_2;
            } else {
                const free = target > lower[i] && target < upper[i];
                f[i] = (input[i] - Math.max(lower[i], Math.min(upper[i], target))) / rho;
                da = free ? 0 : 1; db = free ? 1 : 0;
            }
            if (jacobian) for (let j = starts[i]; j <= ends[i]; j++) jacobian[offsets[i] + j] = db * A[offsets[i] + j] + da * Number(i === j) / rho;
            // A zero mobility equality cannot be repaired by any force. Its
            // full KKT still participates in acceptance; a tolerated residual
            // must not make an otherwise solvable Newton matrix singular.
            if ((general ? zeroRows[i] : A[offsets[i] + i] === 0) && Math.abs(v) <= tolerance) {
                f[i] = 0; if (jacobian) jacobian[offsets[i] + i] = 1;
            }
        }
        for (let g = 0; g < groups.length; g++) {
            const group = groups[g], [i, j] = group.rows;
            const dynamic = group.normalRow != null;
            const rawLoad = dynamic ? group.normalLambda + input[group.normalRow] : 1, load = Math.max(0, rawLoad);
            const mu = dynamic ? group.mu : group.radii, rho = 1 / Math.max(mobilities[i], mobilities[j]);
            const u = input[i] + group.lambda[0], v = input[j] + group.lambda[1];
            const p = projectLoadEllipseDerivative(u + rho * r[i], v + rho * r[j], load, mu, scratch);
            finalGroups[g].radii[0] = load * mu[0]; finalGroups[g].radii[1] = load * mu[1];
            f[i] = (u - p.value[0]) / rho; f[j] = (v - p.value[1]) / rho;
            if (jacobian) for (let k = starts[i]; k <= ends[i]; k++) {
                const du = Number(k === i) - rho * A[offsets[i] + k], dv = Number(k === j) - rho * A[offsets[j] + k];
                const dn = dynamic && rawLoad >= 0 && k === group.normalRow ? 1 : 0;
                jacobian[offsets[i] + k] = (Number(k === i) - p.input[0] * du - p.input[1] * dv - p.load[0] * dn) / rho;
                jacobian[offsets[j] + k] = (Number(k === j) - p.input[2] * du - p.input[3] * dv - p.load[1] * dn) / rho;
            }
        }
        for (let i = 0; i < n; i++) merit += (f[i] * scales[i]) ** 2;
        return .5 * merit;
    }
    let iterations = 0, factorizations = 0, backtracks = 0, gradientFallbacks = 0, boundRecoveries = 0, status = 'coulomb-newton-limit';
    function certifyCandidate() {
        feasible(x, trial); evaluate(trial, trialF, trialResidual);
        measureCoupledLoadKKT(trialResidual, trial, lower, upper, finalGroups, kkt);
        if (kkt.maximumResidual <= tolerance && (!options.acceptCandidate || options.acceptCandidate(trial))) return true;
        const recovered = feasible(x, trial, true);
        if (recovered) {
            evaluate(trial, trialF, trialResidual);
            measureCoupledLoadKKT(trialResidual, trial, lower, upper, finalGroups, kkt);
            if (kkt.maximumResidual <= tolerance && (!options.acceptCandidate || options.acceptCandidate(trial))) { boundRecoveries += recovered; return true; }
        }
        return false;
    }
    for (; iterations < limit; iterations++) {
        const merit = evaluate(x, F, residual, J);
        // Newton iterates may lie outside a force cone. Certify a feasible
        // candidate, including its complete original response, rather than
        // projecting every search step and invalidating the Newton direction.
        if (certifyCandidate()) { x.set(trial); status = 'converged'; break; }
        if (!bandLU) for (let i = 0; i < n; i++) {
            direction[i] = -F[i] * scales[i];
            for (let j = 0; j < n; j++) factor[i * n + j] = J[i * n + j] * scales[i] * scales[j];
            factor[i * n + i] += options.numericalShift ?? 1e-8;
        }
        factorizations++;
        let solved = bandLU ? bandLU.solve(J, F, scales, options.numericalShift ?? 1e-8, direction) : solveDenseLU(factor, direction, n), slope = 0;
        if (solved) {
            for (let i = 0; i < n; i++) direction[i] *= scales[i];
            for (let i = 0; i < n; i++) {
                let v = 0; for (let j = starts[i]; j <= ends[i]; j++) v += J[offsets[i] + j] * direction[j];
                slope += F[i] * scales[i] ** 2 * v;
            }
        }
        if (!solved || !(slope < 0)) {
            gradientFallbacks++; slope = 0;
            direction.fill(0);
            for (let i = 0; i < n; i++) for (let j = starts[i]; j <= ends[i]; j++)
                direction[j] += F[i] * scales[i] ** 2 * J[offsets[i] + j];
            for (let j = 0; j < n; j++) { const v = direction[j]; direction[j] = -v * scales[j] ** 2; slope -= v * v * scales[j] ** 2; }
        }
        options.debugCoulombDirection?.({ iteration: iterations, x, F, J, direction, evaluate, layout });
        let step = 1, accepted = false;
        for (let search = 0; search < 30; search++) {
            for (let i = 0; i < n; i++) trial[i] = x[i] + step * direction[i];
            const next = evaluate(trial, trialF, trialResidual);
            if (Number.isFinite(next) && next <= merit + 1e-4 * step * slope) { x.set(trial); accepted = true; break; }
            step *= .5; backtracks++;
        }
        options.debugCoulombIteration?.({ iteration: iterations, merit, slope, step, accepted, kkt: { ...kkt } });
        if (!accepted) { status = 'coulomb-line-search'; break; }
    }
    evaluate(x, F, residual);
    if (certifyCandidate()) status = 'converged';
    else if (status === 'converged') status = 'coulomb-final-certificate';
    x.set(trial); residual.set(trialResidual);
    const free = Uint8Array.from(x, (v, i) => Number(v > lower[i] && v < upper[i]));
    const lo = Float64Array.from(lower), hi = Float64Array.from(upper);
    for (const group of finalGroups) if (group.radii.some(radius => radius === 0)) group.rows.forEach((row, axis) => {
        lo[row] = -group.radii[axis] - group.lambda[axis]; hi[row] = group.radii[axis] - group.lambda[axis];
    });
    const result = { increment: x, residual, free, lower: lo, upper: hi, allGroups: finalGroups,
        groups: finalGroups.filter(group => group.radii.every(radius => radius > 0)), diagnostics: {
            status, converged: status === 'converged', iterations, factorizations, backtracks, gradientFallbacks, boundRecoveries,
            discardedNonfiniteInitialIncrement: options.initialIncrement != null && initialIncrement == null,
            maximumResidual: kkt.maximumResidual, frictionResidual: kkt.groupResidual, coneViolation: kkt.coneViolation,
            rowCount: n, band, groupCount: groups.length, normalLoadIterations: 0, method: 'simultaneous-coulomb-newton',
            normalMap: options.normalMap ?? 'fischer-burmeister', ...(general ? { matrixFormat, linearSolver: bandLU?'band-lu':'dense-lu' } : {}),
            ...(bandLU?.diagnostics) } };
    options.debugCoulombResult?.({ matrix, rhs, lower, upper, count, band, groups, result,
        initialIncrement: options.initialIncrement, initialFree: options.initialFree });
    return result;
}

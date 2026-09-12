import { solveCoupledBandQP } from './kirchhoffCoupledLinearSolver.js';
import { solveCoupledFrictionQP } from './kirchhoffCoupledFrictionSolver.js';
import { evaluateKirchhoffContinuousFrictionKKT } from './kirchhoffContinuousFrictionKKT.js';

/** Final-load KKT in original displacement units, continuous at stick/slide.
 * Cone feasibility is separate; a tiny interior force error must not switch
 * to requiring zero slip before its actual maximum-dissipation error is read.
 */
export function measureCoupledLoadKKT(residual, increment, lower, upper, groups, out = {}) {
    const mask = out.mask?.length === increment.length ? out.mask : out.mask = new Uint8Array(increment.length);
    const lambda = out.lambda ??= new Float64Array(2), slip = out.slip ??= new Float64Array(2);
    mask.fill(0); out.maximumResidual = out.groupResidual = out.coneViolation = 0;
    for (const group of groups) {
        for (let axis = 0; axis < 2; axis++) {
            const row = group.rows[axis]; mask[row] = 1;
            lambda[axis] = increment[row] + group.lambda[axis]; slip[axis] = -residual[row];
        }
        const r = evaluateKirchhoffContinuousFrictionKKT(lambda, slip, 1, group.radii, out.local ??= {});
        out.groupResidual = Math.max(out.groupResidual, r.residualMm);
        out.coneViolation = Math.max(out.coneViolation, r.coneViolation);
    }
    for (let i = 0; i < increment.length; i++) if (!mask[i]) {
        const r = residual[i], x = increment[i];
        out.maximumResidual = Math.max(out.maximumResidual, lower[i] === upper[i] ? 0 : x <= lower[i] ? Math.max(0, r) :
            x >= upper[i] ? Math.max(0, -r) : Math.abs(r));
    }
    out.maximumResidual = Math.max(out.maximumResidual, out.groupResidual);
    if (out.coneViolation > 1e-9) out.maximumResidual = Infinity;
    return out;
}

/** Non-associated Coulomb closure at ONE frozen material/contact Jacobian.
 * The normal load is an unknown in this same system. Success requires the QP
 * and the friction cone evaluated at its solved normal force to agree. This
 * never differentiates a cone radius into the normal equilibrium equation
 * (which would introduce associated dilation), and never clips forces without
 * recomputing their complete generalized response. It is a local fixed-point
 * iteration; failure is explicit and must not be applied by the world.
 *
 * groups: {rows:[u,v], lambda:[oldU,oldV], mu:[muU,muV],
 * normalRow, normalLambda}, or a fixed {rows,lambda,radii} group.
 * Bounds are on increments, normalLambda is the old TOTAL normal force.
 */
export function solveCoupledLoadQP(matrix, rhs, lower, upper, count, band, groups, options = {}) {
    const workspace = options.loadWorkspace ??= {};
    const dynamic = groups.filter(group => Number.isInteger(group.normalRow));
    if (!workspace.lower || workspace.lower.length < count) {
        workspace.lower = new Float64Array(count); workspace.upper = new Float64Array(count);
    }
    const lo = workspace.lower.subarray(0, count), hi = workspace.upper.subarray(0, count);
    const loads = dynamic.map(group => group.normalLambda), finalLoads = new Float64Array(dynamic.length);
    const definitions = groups.map(group => ({ ...group, radii: group.radii ? [...group.radii] : [0, 0] }));
    const activeGroups = [], finalGroups = [];
    const maxIterations = options.maxNormalLoadIterations ?? 24;
    if (!Number.isInteger(maxIterations) || maxIterations < 1) throw new RangeError('Positive normal-load iteration limit required');
    const tolerance = options.tolerance ?? 1e-8;
    const innerOptions = { ...options, tolerance };
    let solved, totalFactors = 0, totalUpdates = 0, totalIterations = 0, loadIterations = 0, loadError = Infinity, certified = false;
    const kkt = workspace.kkt ??= {};
    function configure(values, output) {
        lo.set(lower); hi.set(upper); output.length = 0;
        let index = 0;
        for (let i = 0; i < groups.length; i++) {
            const source = groups[i], group = definitions[i];
            if (Number.isInteger(source.normalRow)) {
                const load = values[index++];
                group.radii[0] = load * source.mu[0]; group.radii[1] = load * source.mu[1];
            }
            if (group.radii[0] === 0 || group.radii[1] === 0) {
                for (let axis = 0; axis < 2; axis++) {
                    lo[group.rows[axis]] = -group.radii[axis] - group.lambda[axis];
                    hi[group.rows[axis]] = group.radii[axis] - group.lambda[axis];
                }
            } else output.push(group);
        }
    }
    for (; loadIterations < maxIterations; loadIterations++) {
        configure(loads, activeGroups);
        const debugInitial = options.debugLoadIteration ? {
            initialFree: innerOptions.initialFree ? Array.from(innerOptions.initialFree) : undefined
        } : null;
        solved = activeGroups.length ? solveCoupledFrictionQP(matrix, rhs, lo, hi, count, band, activeGroups, innerOptions)
            : solveCoupledBandQP(matrix, rhs, lo, hi, count, band, innerOptions);
        options.debugLoadIteration?.({ matrix, rhs, lower: lo, upper: hi, count, band, groups: activeGroups,
            sourceGroups: groups, sourceLower: lower, sourceUpper: upper, iteration: loadIterations,
            loads, result: solved, options: { tolerance: innerOptions.tolerance, numericalShift: options.numericalShift, ...debugInitial } });
        totalFactors += solved.diagnostics.factorizations; totalUpdates += solved.diagnostics.factorUpdates ?? 0;
        totalIterations += solved.diagnostics.iterations;
        kkt.maximumResidual = Infinity;
        if (!solved.increment.every(Number.isFinite) || !solved.residual.every(Number.isFinite)) break;
        loadError = 0;
        for (let i = 0; i < dynamic.length; i++) {
            const group = dynamic[i];
            finalLoads[i] = Math.max(0, group.normalLambda + solved.increment[group.normalRow]);
            loadError = Math.max(loadError, Math.abs(finalLoads[i] - loads[i]) /
                Math.max(Number.MIN_VALUE, finalLoads[i], loads[i]));
        }
        // Evaluate the ORIGINAL normal/material equations and the final cone.
        // The QP's residual is rhs-A*increment in original row units.
        configure(finalLoads, finalGroups);
        measureCoupledLoadKKT(solved.residual, solved.increment, lo, hi, definitions, kkt);
        let boundsValid = true;
        for (let i = 0; i < count; i++) if (solved.increment[i] < lo[i] || solved.increment[i] > hi[i]) boundsValid = false;
        // The inner QP target can be tighter after refinement. Its iteration
        // cap is not the physical certificate: accept only if ALL
        // final-load equations satisfy the original requested tolerance.
        if (boundsValid && kkt.maximumResidual <= tolerance) { certified = true; break; }
        if (!solved.diagnostics.converged) break;
        // Radius iteration cannot improve a stationary fixed-load QP whose
        // own stopping error dominates the continuous dissipation residual.
        if (loadError < 1e-8 && kkt.coneViolation <= 1e-9 && kkt.maximumResidual > tolerance)
            innerOptions.tolerance *= .25;
        for (let i = 0; i < loads.length; i++) loads[i] = finalLoads[i];
        // Working-set hints keep the exact row identities of this frozen
        // system. Forces still start from zero: redundant hard contacts must
        // not accumulate a drifting null-space force through load iterations.
        if (options.reuseLoadHints) innerOptions.initialFree = solved.free;
    }
    const converged = certified;
    return { ...solved, groups: finalGroups, allGroups: definitions, lower: lo, upper: hi, diagnostics: {
        ...solved?.diagnostics, converged, status: converged ? 'converged' : solved?.diagnostics.converged ? 'normal-load-iteration-limit' : solved?.diagnostics.status,
        factorizations: totalFactors, factorUpdates: totalUpdates, iterations: totalIterations, normalLoadIterations: Math.min(loadIterations + 1, maxIterations),
        normalLoadRelativeError: loadError, maximumResidual: Math.max(solved?.diagnostics.maximumResidual ?? Infinity, kkt.maximumResidual ?? Infinity)
    } };
}

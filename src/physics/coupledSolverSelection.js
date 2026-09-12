// Selection only: the numerical kernel is supplied by the caller so an
// external-source replay uses exactly that source tree's implementation.
const variants = Object.freeze({
    reference: null,
    'composite-joint': null,
    joint: Object.freeze({ activeCondensation: false, simultaneousCoulomb: false }),
    'joint-active-coulomb': Object.freeze({ activeCondensation: true, simultaneousCoulomb: true }),
    'joint-wall-witnesses': Object.freeze({ activeCondensation: true, simultaneousCoulomb: true }),
    'joint-axial-sections': Object.freeze({ sectionSpan: 32, sectionScope: 'all' }),
    'joint-two-channel': Object.freeze({ activeCondensation: true, simultaneousCoulomb: true }),
    'joint-full-band': Object.freeze({ activeCondensation: true, simultaneousCoulomb: true, coulombStructure: 'full-band' })
});

// Application bookmarks from the experimental rebuild must restore the
// established position-history mechanics, not merely an older experiment.
// Split motion remains available only through an explicit experimental flag.
export function resolveAppCoupledSolver(search = '') {
    const params = new URLSearchParams(search);
    const requested = params.get('coupledSolver');
    if (params.get('experimentalSplitMotion') === '1' && requested === 'joint-two-channel')
        return requested;
    return !requested || requested === 'composite-joint' || requested === 'joint-two-channel'
        ? 'joint-active-coulomb' : requested;
}

export function createCoupledSolverSelection(id = 'reference', kernel = {}) {
    if (!Object.hasOwn(variants, id)) throw new RangeError(`Unknown coupled solver: ${id}`);
    const options = variants[id];
    const twoChannel = id === 'joint-two-channel';
    const composite = id === 'composite-joint';
    const solveKernel = id === 'joint-axial-sections' ? kernel.solveAxial : kernel.solve;
    if (composite && (typeof kernel.wholeStepSystem?.step !== 'function' || typeof kernel.wholeStepSystem?.reset !== 'function'))
        throw new TypeError(`${id} requires its whole-step system`);
    if (options && (typeof solveKernel !== 'function' || typeof kernel.apply !== 'function')) {
        throw new TypeError(`${id} requires a coupled solve/apply kernel`);
    }
    if (twoChannel && typeof kernel.solveTwoChannel !== 'function')
        throw new TypeError(`${id} requires the two-channel kernel`);
    let solveCalls, returnedSolves, activeCondensedResults, simultaneousCoulombNewtonResults;
    let nonconvergedResults, fullBandResults, bandLUNewtonResults, twoChannelResults, last, lastFailure;
    let axialTotals;
    function resetDiagnostics() {
        solveCalls = returnedSolves = activeCondensedResults = simultaneousCoulombNewtonResults = nonconvergedResults = 0;
        fullBandResults = bandLUNewtonResults = twoChannelResults = 0;
        last = null;
        lastFailure = null;
        axialTotals = { calls: 0, axialAssemblyMs: 0, axialNewtonMs: 0, axialRecoveryMs: 0,
            sectionAssemblyMs: 0, boundaryPreparationMs: 0, boundarySolveMs: 0, sectionRecoveryMs: 0,
            sectionTopologyMs: 0,localPackingMs: 0,localFactorMs: 0,localResponsesMs: 0,localReactionsMs: 0,
            localResponseSolves: 0, localResponseReuses: 0 };
    }
    resetDiagnostics();
    function run(solve, constraint, dt, runtimeOptions = {}) {
            solveCalls++;
            // Preserve tolerances, boundary/contact rows, workspaces and all
            // other world options; only the named variant options are fixed.
            const result = solve(constraint, dt, { ...runtimeOptions, ...options });
            last = result.diagnostics;
            returnedSolves++;
            activeCondensedResults += Number(Number.isInteger(last.originalCount) && Number.isInteger(last.equalityCount));
            simultaneousCoulombNewtonResults += Number(last.method === 'simultaneous-coulomb-newton');
            fullBandResults += Number(last.coulombStructure === 'full-band');
            bandLUNewtonResults += Number(last.linearSolver === 'band-lu');
            twoChannelResults += Number(last.channels === 2);
            nonconvergedResults += Number(!last.converged);
            if (last.axialReduction) {
                axialTotals.calls++;
                for (const key of Object.keys(axialTotals)) if (key !== 'calls') axialTotals[key] += last[key] ?? 0;
            }
            if (!last.converged) lastFailure = { status: last.status, maximumResidual: last.maximumResidual,
                worstScalarConstraint: last.worstScalarConstraint ? { ...last.worstScalarConstraint } : null };
            return result;
    }
    const coupledSystem = options ? Object.freeze({
        independentComponents: id === 'joint-active-coulomb' || id === 'joint-wall-witnesses',
        physicalTrialState: id === 'joint-active-coulomb' || id === 'joint-wall-witnesses',
        earlyTrialRejection: id === 'joint-active-coulomb',
        ...(id === 'joint-wall-witnesses' ? {wallWitnesses:true} : {}),
        solve(constraint, dt, runtimeOptions) { return run(solveKernel, constraint, dt, runtimeOptions); },
        ...(twoChannel ? { solveTwoChannel(constraint, dt, runtimeOptions) {
            return run(kernel.solveTwoChannel, constraint, dt, runtimeOptions);
        } } : {}),
        apply: kernel.apply
    }) : null;
    return Object.freeze({
        id, coupledSystem, resetDiagnostics, wholeStepSystem: composite ? kernel.wholeStepSystem : null,
        jointMotionMode: twoChannel ? 'split-physical-bias' : 'position-history',
        biasMaterialMode: twoChannel ? 'coupled-compliance' : null,
        getReport(world) {
            return {
                id, options: options ? { ...options } : null,
                jointMotionMode: world?.jointMotionMode ?? null,
                biasMaterialMode: twoChannel ? 'coupled-compliance' : null,
                installed: world ? composite ? world.wholeStepSystem === kernel.wholeStepSystem : world.coupledSystem === coupledSystem : null,
                ...(composite ? {wholeStep: kernel.wholeStepSystem.diagnostics ?? null} : {}),
                lastStepSolver: world?.lastCoupledSolver ?? null,
                independentComponents: coupledSystem?.independentComponents ?? false,
                // Counts since reset, including failed linear proposals.
                // Newton results exclude a converged fixed-load seed and are
                // not a count of every internal Newton attempt/iteration.
                solveCalls, returnedSolves, activeCondensedResults,
                lastFailure,
                axialTotals: { ...axialTotals },
                simultaneousCoulombNewtonResults, fullBandResults, bandLUNewtonResults, twoChannelResults, nonconvergedResults,
                lastResult: last ? {
                    converged: last.converged, status: last.status,
                    method: last.method ?? null, basis: last.basis ?? null,
                    coulombStructure: last.coulombStructure ?? null, linearSolver: last.linearSolver ?? null,
                    originalCount: last.originalCount ?? null,
                    equalityCount: last.equalityCount ?? null,
                    retainedCount: last.retainedCount ?? null,
                    axialReduction: last.axialReduction ?? false,
                    globalRowCount: last.globalRowCount ?? null,
                    localRowCount: last.localRowCount ?? null,
                    axialTimings: last.axialReduction ? {
                        assemblyMs: last.axialAssemblyMs, newtonMs: last.axialNewtonMs, recoveryMs: last.axialRecoveryMs,
                        sectionAssemblyMs: last.sectionAssemblyMs, boundaryPreparationMs: last.boundaryPreparationMs,
                        boundarySolveMs: last.boundarySolveMs, sectionRecoveryMs: last.sectionRecoveryMs,
                        localResponseSolves: last.localResponseSolves, localResponseReuses: last.localResponseReuses
                    } : null,
                    channels: last.channels ?? null,
                    fullRowCount: last.fullRowCount ?? null,
                    condensation: last.condensation ?? null,
                    maximumResidual: last.maximumResidual ?? null
                } : null
            };
        }
    });
}

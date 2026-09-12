// Disabled inter-tool friction contributes no rows, history or convergence
// condition. Normal containment still belongs to the ordinary coupled solve.
export const FRICTIONLESS_LUMEN_RESIDUAL = Object.freeze({
    maximumResidual: 0, maximumFeasibilityResidual: 0, maximumStationarityResidual: 0,
    maximumDisplacementResidualMm: 0, maximumConeViolation: 0,
    maximumNormalMomentResidual: 0, contactCount: 0,
    residualUnits: 'multiplier', inverseMobility: 1, reusedEvaluation: false,
    _batch: Object.freeze({ entries: Object.freeze([]), rows: Object.freeze([]) })
});

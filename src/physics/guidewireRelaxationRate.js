export const MIN_GUIDEWIRE_RELAXATION_RATE = 1;
export const MAX_GUIDEWIRE_RELAXATION_RATE = 50;
export const DEFAULT_GUIDEWIRE_RELAXATION_RATE = 1;

// The regular world solve already performs six constitutive sweeps. Extra
// guidewire-only sweeps accelerate convergence toward the same static
// equilibrium without changing EI, tip compliance, wall friction or rest
// curvature. The same pass count is used during manipulation and at rest.
// Iterative residuals contract geometrically, so achieving a multiplicative
// reduction in visible settling time requires a logarithmic—not linear—growth
// in pass count. This keeps the 50x end of the control computationally usable.
const BASE_GUIDEWIRE_RELAXATION_PASSES = 6;

export function clampGuidewireRelaxationRate(value) {
    const finiteValue = Number.isFinite(value)
        ? value
        : DEFAULT_GUIDEWIRE_RELAXATION_RATE;
    return Math.max(
        MIN_GUIDEWIRE_RELAXATION_RATE,
        Math.min(MAX_GUIDEWIRE_RELAXATION_RATE, finiteValue)
    );
}

export function guidewireRelaxationPasses(rate) {
    const clampedRate = clampGuidewireRelaxationRate(rate);
    return Math.round(
        Math.log2(clampedRate / DEFAULT_GUIDEWIRE_RELAXATION_RATE) *
            BASE_GUIDEWIRE_RELAXATION_PASSES
    );
}

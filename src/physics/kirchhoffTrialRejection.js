// boundary/tolerance is a lower bound on the final max-merit. Both acceptance
// routes must fail: merit decrease AND final physical convergence. Equality,
// nonfinite values and disabled/zero tolerances deliberately use a full measure.
export function boundaryRejectsKirchhoffTrial(boundary, tolerance, meritThreshold) {
    return Number.isFinite(boundary) && Number.isFinite(tolerance) && tolerance > 0 &&
        Number.isFinite(meritThreshold) && boundary > tolerance &&
        boundary / tolerance > meritThreshold;
}

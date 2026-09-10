export const CATHETER_PHYSICS_SPACING_MM = 5;
// The existing nodal mass was calibrated on the former 4 mm grid.
const MASS_REFERENCE_SPACING_MM = 4;

export function catheterPhysicsNodeCount(maximumInsertionMm, proximalVisibleMm, spacing = CATHETER_PHYSICS_SPACING_MM) {
    if (![maximumInsertionMm, proximalVisibleMm, spacing].every(Number.isFinite) ||
        maximumInsertionMm < 0 || proximalVisibleMm < 0 || spacing <= 0) {
        throw new RangeError('Catheter lengths must be finite and nonnegative, with positive spacing');
    }
    // Retain the visible proximal reservoir even at maximum insertion, and
    // at least one extra feed segment required by the sheath transport.
    return Math.ceil((maximumInsertionMm + Math.max(proximalVisibleMm, spacing)) / spacing) + 1;
}

export function catheterNodeMass(referenceMass, spacing = CATHETER_PHYSICS_SPACING_MM) {
    if (!Number.isFinite(referenceMass) || referenceMass <= 0 || !Number.isFinite(spacing) || spacing <= 0)
        throw new RangeError('Catheter mass and spacing must be positive and finite');
    return referenceMass * spacing / MASS_REFERENCE_SPACING_MM;
}

import { kirchhoffComponentBodies } from './kirchhoffComponentBodies.js';
import { assembleKirchhoffDirect } from './kirchhoffDirectSolver.js';

/** Evaluate the nonlinear constitutive equations at the APPLIED state.
 * A small linear-system residual alone does not certify the finite quaternion
 * update. Keep millimetres (adaptation) and radians (bend/twist) separate.
 * Uses the same constitutive equations and refreshed material multipliers.
 */
export function measureKirchhoffCoupledMaterialResidual(constraint, dt, out = {}) {
    out.adaptationMm = 0;
    out.bendTwistRad = 0;
    out.worstAdaptationSide = out.worstAdaptationSegment = -1;
    const bodies = kirchhoffComponentBodies(constraint);
    for (let side = 0; side < bodies.length; side++) {
        const state = assembleKirchhoffDirect(bodies[side], dt);
        if (!state) continue;
        for (let row = 0; row < state.rowCount; row += 6) {
            const value = k => state.strain[row + k] + state.alpha[row + k] * state.lambda[row + k];
            const adaptation = Math.hypot(value(3), value(4), value(5));
            const bendTwist = Math.hypot(value(0), value(1), value(2));
            if (adaptation > out.adaptationMm) {
                out.adaptationMm = adaptation;
                out.worstAdaptationSide = side;
                out.worstAdaptationSegment = state.start + row / 6;
            }
            out.bendTwistRad = Math.max(out.bendTwistRad, bendTwist);
        }
    }
    return out;
}

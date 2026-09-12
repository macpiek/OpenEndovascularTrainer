# Cause of Invalid friction data

Confirmed by the current anatomy run at zero-based schedule step 796, with 220 mm wire insertion and approximately 85.3667 mm catheter insertion. A standalone replay of the captured 61-row condensed contact system reproduces the same exception without anatomy or rendering.

## First invalid operation

1. `kirchhoffActiveCondensedSolver.js` forms the Schur complement by subtracting the eliminated material response from the contact Gram matrix. For local row 4 (ordered source row 291), the original diagonal is 0.3438774849832733. The 22 subtraction terms sum to approximately 0.34387748498327336. Sequential subtraction produces **-6.508048707864232e-18**. The original matrix, condensed matrix and RHS are all finite; the affected row is nearly zero (its other entries are around 1e-17).
2. `kirchhoffCoupledLinearSolver.js` computes `1 / Math.sqrt(matrix[i * band] || 1)`. A negative nonzero diagonal passes the `|| 1` fallback. Its square root is NaN. Bound scaling and the fixed-load seed then contain NaNs.
3. `solveSeededCoulombNewton` passes the nonconverged seed into Newton without rejecting nonfinite seed values. The symmetric-band Newton scaling repeats the same unsafe square root.
4. At Newton iteration 0, certification receives tangential force [NaN, 0] and displacement [NaN, NaN]. `evaluateKirchhoffContinuousFrictionKKT` detects this and throws `Invalid friction data`. It is a downstream symptom of roundoff and unchecked scaling, not a failure first produced by Newton LU.

The cancellation occurs at floating-point roundoff scale: approximately 1.9e-17 relative to the original diagonal. It reflects a nearly eliminated contact direction. A physically negative stiffness is not established by this residual.

## Reproduction

Run `node scripts/physics/reproduce-friction-roundoff.mjs`. Input is saved in `reports/friction-roundoff-reproducer-2026-09-10.json`, including the original condensation terms. The script reports the negative diagonal and reproduces the invalid friction values.

## Repair

Added shared positive numerical scaling for the bound QP and symmetric Coulomb Newton system. Schur rows carry a cancellation estimate based on the original diagonal and eliminated response terms. Near-zero diagonals within this estimate use a finite reference preconditioner; the source matrix is never clipped or regularized by this change. Tangent reduction transforms these estimates with its row mapping. Materially negative diagonals outside the estimate are rejected explicitly.

Newton discards a nonfinite initial numerical guess. Existing physical contact reactions remain unchanged, and acceptance still evaluates the original matrix, full reconstructed system, force bounds and nonlinear contact conditions. No collision, friction coefficient, timestep or tolerance change was made.

The captured 61-row case now converges. A regression test recomputes its residual independently from the original matrix and checks the final Coulomb certificate. Further tests check tiny positive mobilities, rejection of genuinely negative/nonfinite diagonals, and a nonfinite seed. Temporary diagnostic instrumentation was removed; the exception retains structured offending values.

Runtime verification is recorded in `friction-roundoff-fix-validation-2026-09-10.json`.

Final validation: 1080/1080 short-coupled steps completed, 220 mm wire and 208 mm catheter insertion, no exception and all coordinates finite. Eleven steps did not achieve nonlinear convergence; maximum measured penetration was 0.008694 mm. This fixes the NaN crash, not all deep-insertion convergence or performance issues. The 35 targeted tests and build passed; the broader coupled suite retained its seven known failures.

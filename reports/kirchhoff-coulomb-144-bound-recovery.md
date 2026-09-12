# 144.2 mm Coulomb convergence: recover the actual contact bound

The adjacent patch applies cleanly to the current root and changes only
`kirchhoffCoulombNewtonSolver.js`, its existing tests, and adds
`kirchhoffCoulombBoundRecovery.test.js`. Also copy the binary fixture
`tests/fixtures/kirchhoff-coulomb-144-bound-recovery.json.gz`.
No world, geometry, material kernel, tolerances, or KKT rules changed.

## Reproduced failure

The supplied retained problem has 284 rows and 81 friction groups. Direct
Fischer–Burmeister Newton from its captured initial increment reproduces
the failed force vector exactly: 57 iterations, 58 factors, 328 backtracks,
then line-search failure. Its merit is only 1.2653e-32, yet full KKT is
7.3414235e-4, above the unchanged tolerance 2e-4.

Row 105 explains the discrepancy: normal force 7.9884e-20, separating
residual -7.3414235e-4, but FB map value exactly zero. In `a+b-hypot(a,b)`
the tiny force contribution cancels. Feasibility clipping preserves the
positive force because it already lies inside its interval. The original
KKT test correctly requires zero residual for that positive force and
rejects it. More Newton iterations cannot usefully improve this merit.

Rationalizing the FB expression was tested separately and still stalled;
that arithmetic experiment is not included in the patch.

## Corrective step

The existing feasible candidate is checked first. If it fails, non-friction
box rows with positive mobility use their existing natural-map target
`x + (rhs - A*x)/Aii` to identify a lower or upper active bound. Only rows
whose target reaches that bound are moved there. There is no force-magnitude
cutoff, softened contact, radius-transpose reaction, or changed classification
inside the KKT test.

This changes the actual candidate force vector. All dependent friction
cones are then rebuilt from its new normal loads, the complete original
matrix response is recalculated, and the unchanged full KKT/feasibility
certificate is required. A failed candidate does not replace the ongoing
Newton iterate. Final output is certified again; it cannot retain a stale
success status if that last check fails. `boundRecoveries` reports the number
of bound corrections in a successfully certified candidate.

## Results

| Same direct captured initial state | Before | After |
| --- | ---: | ---: |
| Status | line-search failure | converged |
| Factors | 58 | 2 |
| Backtracks | 328 | 1 |
| Original full KKT | 7.3414e-4 | 1.2765e-4 |
| Friction KKT | 4.9073e-11 | 8.5360e-5 |
| Cone violation | 2.22e-16 | 2.22e-16 |

The candidate corrects three bounds. Row 105's force is now exactly zero,
with separating residual -9.0809e-4; that inactive contact is consistent.
Every other equation and cone is checked after the force correction.

The full seeded entry point also converges on the fixture: 20 factors, one
normal-load seed, full KKT 1.4835e-4, and zero cone violation. It does not
need the cold-hint retry. Existing map fallback logic is unchanged; no new
arbitrary retry or higher iteration limit was added.

46 combined condensed/Coulomb/load/KKT tests pass, including the previous
1,999-row byte-hash regression. New tests independently reconstruct the full
284-row residual, check all force bounds and solved-load cones, preserve a
legitimate 1e-20 loaded contact, recover both lower/upper bounds without a
force cutoff, refresh an unloaded dependent friction cone, and deliberately
reject a bound correction that violates another coupled equation.

Two earlier expectations were updated: the switching-plane test now
explicitly requests its asserted 1e-5 accuracy, and the 105.2 mm hinted case
now verifies successful bound recovery without a cold retry. Its former
requirement that the first attempt fail is obsolete. Runtime tolerance is
unchanged at 2e-4.

No timing or FPS conclusion is drawn. The root browser smoke test overlapped
some diagnostic probes; the evidence here is the reproducible step count
and full physical certificate, not wall-clock time. No scene replay ran here.

## Handoff

Source/case baseline: `/tmp/oet-coulomb-144-source-baseline`.
Frozen candidate: `/tmp/oet-coulomb-144-bound-recovery-frozen`.
The adjacent JSON records diagnostics, the stalled merit sequence and hashes.

```sh
node --test tests/kirchhoffActiveCondensedSolver.test.js tests/kirchhoffActiveCondensedWorkspace.test.js tests/kirchhoffCoulombBoundRecovery.test.js tests/kirchhoffCoulombNewtonSolver.test.js tests/kirchhoffCoupledLoadSolver.test.js tests/kirchhoffContinuousFrictionKKT.test.js
```

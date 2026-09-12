# Captured friction solve: bounded Newton targets and stable hints

Frozen candidate changes only `kirchhoffCoupledFrictionSolver.js` and
`kirchhoffCoupledLinearSolver.js`. Apply the adjacent `.patch` to the previous
geometric-capacity/typed-row-index candidate, or copy those two files. No world,
geometry, tolerance, physical compliance, material, or force-radius changes.

The captured real solve has 1,681 rows, band 97, and three positive ellipses
among 337 additional rows. Original input snapshots were preserved before
profiling at `/tmp/oet-cone-linear-baseline-20260906.json` and
`/tmp/oet-cone-raw2-baseline-20260906.json`. Pristine prior source is at
`/tmp/oet-cone-source-baseline-20260906`.

## Changes and mechanics

- Free-set hints remain in original full row identity and are mapped afresh
  into every reduced cone subproblem. A sliding ellipse removes one reduced
  row, so reusing its positional mask was invalid. The box solver now returns
  its borrowed `free` mask; it is used only as a boolean working-set hint.
- For an interior ellipse, the Newton *target* is restricted by the necessary
  component bounds `-radius <= oldLambda + x + step <= radius`. These contain
  every ellipse-feasible force. They avoid computing a distant target whose
  accepted step then shrinks to almost zero. Exact ellipse intersection,
  active ellipse retraction, outward-normal KKT, original matrix residual,
  and objective line search still decide the accepted solution. This does
  not replace Coulomb ellipses with rectangular friction domains.

## Evidence

| Frozen case | Prior | Candidate |
| --- | ---: | ---: |
| Cone iterations | 3 | 5 |
| Factorizations | 51 | 11 |
| Original/reconstructed KKT residual | 9.02092e-6 | 6.31892e-6 |
| Objective | -0.000384372846409153 | -0.000384372846420296 |
| Post-JIT median solve, ms | 106.58 | 30.43 |
| Post-JIT mean solve, ms | 111.59 | 35.48 |
| First process trial, ms | 160.32 | 82.93 |

Times use Node v24.6.0, 16 trials in each separate process, baseline followed
by candidate, each with fresh workspace and identical saved hints. The first
four trials are excluded from post-JIT statistics. The adjacent JSON retains
all trial times, diagnostics, source hashes, and comparison results. This is
a frozen linear case, not a whole-world replay or a frame-rate claim.

Reassembling the raw bodies and reconstructing their actual physical
corrections gives maximum differences 2.0842e-8 (inner), 2.3718e-8 (outer),
and 4.8314e-8 (force increment). Normal residual improves from 8.86e-13 to
1.17e-13. The fixed-boundary material residual floor remains 4.19848e-6.
Trust scale is 1. No gradient fallbacks or backtracks occur in either solve.

A separate instrumented warmed trial attributes roughly 10.76 ms to the
11 WASM factorizations, 4.96 ms to free-matrix assembly, 0.94 ms to solves,
and 5.57 ms to cone assembly/residual work. First target reaches an exact
ellipse at step 0.7071 instead of 9.69e-7. Factor counts by cone iteration
are 5, 3, 1, 1, 1. Instrumentation stays in `/tmp`, outside the candidate.

## Validation and reproduction

60 tests pass: coupled system, friction dense/manufactured oracles, bundle
runtime, capacity growth, and the new captured-runtime regression. The
existing fresh Uint32 row-index regression also passes, preserving fractional
previous forces and total-force bounds. The captured fixture is compressed
losslessly; the test checks force agreement, objective, original residual,
exact ellipse feasibility and <=16 factors, with no wall-clock assertion.

```sh
node --test tests/kirchhoffCoupledSystem.test.js tests/kirchhoffCoupledFrictionSolver.test.js tests/kirchhoffCoupledFrictionRuntime.test.js tests/kirchhoffBundleRuntime.test.js tests/kirchhoffCoupledCapacity.test.js
node scripts/physics/profile-coupled-linear-snapshot.mjs /tmp/oet-cone-linear-baseline-20260906.json /absolute/path/to/kirchhoffCoupledFrictionSolver.js 16
node scripts/physics/check-coupled-snapshot.mjs /tmp/oet-cone-raw2-baseline-20260906.json 0.0002 /absolute/path/to/kirchhoffCoupledSystem.js
```

The raw snapshot checker now uses cone-aware KKT for grouped rows; an outward
sliding residual is not reported as an equality violation. It also reports
owned force/correction vectors for independent comparison. No batch PDAS,
factor cache, or physical force warm start is included in this candidate.

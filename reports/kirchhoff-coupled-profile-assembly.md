# Exact profile assembly after skyline

Frozen candidate follows the skyline handoff. Its JS diff is
`kirchhoff-coupled-profile-assembly.patch`; apply that narrow diff if the
parent has newer friction changes. Required files:

- `src/physics/kirchhoffCoupledLinearSolver.js`
- `src/physics/kirchhoffCoupledFrictionSolver.js`
- `scripts/physics/kirchhoffLinearKernel.wat`
- `src/physics/kirchhoffLinearKernelBytes.js`
- `tests/kirchhoffSkylineKernel.test.js`

The WAT adds `scaleBandProfile`, `findBandStarts`, and `compressFreeBand`.
Solo factor/solve exports and all physical equations remain unchanged.

The box solver copies the current matrix into its existing WASM workspace,
equilibrates it with exactly the previous multiplication order, and records
each row's first exact nonzero. Each working-set change copies the free
principal matrix directly in WASM, traversing only that source profile. It
discovers the compressed skyline and keeps every internal entry/fill position.
No coefficients are thresholded. Bound rows remain in full original residuals.

The cone solver discovers the exact profile once per current full QP, then
uses it for both reduced-band discovery and congruence assembly in JS. It
retains existing exact ellipse rules and workspace growth. Profiles are
rebuilt on every solve, so changing contact geometry cannot leave stale
structural zeros. There is no topology cache or physical force warm start.

## Validation

81 tests pass: all 79 from the skyline handoff plus two tests verifying exact
WASM scale/compression against a dense principal matrix for changing masks,
and reused cone workspaces whose matrices gain/remove distant nonzeros.
The tests retain the 1e-30 leading coefficient, direct/solo coverage, geometric
capacity checks, typed fractional previous forces, and full captured fixture.

On the original frozen 1,681-row/band-97 snapshot, all force increments and
both raw-body physical corrections are **bit-identical** to the skyline
candidate. It still takes 11 factors/five cone iterations, with no backtracks
or gradient fallbacks. KKT and reconstruction residual remain 6.31891866e-6;
fixed-boundary material residual is 4.19848268e-6 and trust scale is one.

## Timing

Two bounded timing comparisons place the new median at 8.17–8.88 ms, versus
16.02–18.22 ms for skyline. The final controlled sequential comparison uses
40 fresh-workspace trials per process, original saved hints, Node v24.6.0,
and excludes the first 16 trials for JIT warmup:

| Frozen linear solve | Skyline | Profile assembly |
| --- | ---: | ---: |
| Median, ms | 16.018 | 8.879 |
| Mean, ms | 16.132 | 11.043 |
| Maximum retained trial, ms | 17.689 | 26.448 |
| First trial, ms | 75.764 | 35.743 |

The adjacent JSON preserves every trial time, diagnostics and file hashes.
The candidate had timing outliers; no wall-clock test threshold is used.
These are isolated linear solves, not full-world timing or a frame-rate claim.

Frozen source snapshots:
`/tmp/oet-cone-frozen-skyline-20260906` (prior) and
`/tmp/oet-cone-frozen-profile-assembly-20260906` (candidate).

```sh
OET_WABT_PATH=/tmp/oet-wasm-tools/node_modules/wabt node scripts/physics/build-linear-kernel.mjs
node --test tests/kirchhoffSkylineKernel.test.js tests/kirchhoffCoupledSystem.test.js tests/kirchhoffCoupledFrictionSolver.test.js tests/kirchhoffCoupledFrictionRuntime.test.js tests/kirchhoffBundleRuntime.test.js tests/kirchhoffCoupledCapacity.test.js tests/kirchhoffDirectSolver.test.js tests/kirchhoffDirectOptimization.test.js tests/kirchhoffDirectCoupling.test.js
```

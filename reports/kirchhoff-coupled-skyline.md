# Exact skyline Cholesky candidate

Apply after the frozen 11-factor friction-target/hint candidate. Copy only:

- `src/physics/kirchhoffCoupledLinearSolver.js` (or apply adjacent linear patch)
- `scripts/physics/kirchhoffLinearKernel.wat`
- `src/physics/kirchhoffLinearKernelBytes.js` (generated from that WAT)
- `tests/kirchhoffSkylineKernel.test.js`

The original `factorBand`, `solveBand` and solo paths remain unchanged. The
coupled box solver defaults to new `factorSkyline`/`solveSkyline` exports;
`factorization:'band'` retains the previous factor/solve path for comparison.
There are no physical equation, force bound, tolerance or compliance changes.

For each compressed free principal row, assembly records its first exactly
nonzero column. Cholesky retains all internal zeros and fill inside that
skyline. Its product loop begins at the later of the two row starts.
Forward substitution and backward scatter traverse the same profile.
There is no numerical zero threshold. A 1e-30 leading coefficient is covered
by the tests. The same numerical pivot floor and original-system iterative
refinement/KKT checks are preserved.

## Captured-case validation

The same 1,681-row/band-97 snapshot still takes 11 factors and five cone
iterations with no fallback/backtrack. Original and physical reconstruction
KKT residual remain 6.31891866e-6, and fixed-boundary material residual remains
4.19848268e-6. Maximum physical correction difference versus the frozen band
candidate is 4.134e-18 (inner) and 2.223e-18 (outer). Dual increment difference
is 3.989e-9 in the redundant force representation; objective difference is
at floating-point rounding scale. Trust scale remains one.

Sequential Node v24.6.0 processes ran 40 trials each with fresh workspaces and
identical saved hints, excluding the first 16 for JIT warmup:

| Linear solve | Band | Skyline |
| --- | ---: | ---: |
| Warm median, ms | 30.687 | 18.219 |
| Warm mean, ms | 31.553 | 18.427 |
| First trial, ms | 86.686 | 112.848 |

The separate JSON retains all times and source hashes. Cold startup was not
improved. These measurements are one frozen linear system, not frame times.

The final factor has 1,359 rows and maximum band 53, but mean skyline width
8.15, median 7, and 95th percentile 11. It has 11,078 profile entries instead
of 72,027 band entries, and 47,106 inner products instead of 1,824,472.
A warmed instrumented solve attributes 0.462 ms total to all 11 factors,
0.201 ms to triangular solves, and 5.048 ms to JS free-matrix assembly.
The previous factor total was about 10.76 ms. Assembly and cone work are now
the main remaining costs; instrumentation is confined to `/tmp`.

79 tests pass: the previous 60, five skyline tests, and 14 direct/solo tests.
Tests compare irregular-profile factors/solutions against the unchanged
band exports and independently multiply the original dense equations;
they also cover tiny leading coefficients, rank-deficient pivot flooring,
compressed bound activity and the original full-rod fixture.

```sh
OET_WABT_PATH=/tmp/oet-wasm-tools/node_modules/wabt node scripts/physics/build-linear-kernel.mjs
node --test tests/kirchhoffSkylineKernel.test.js tests/kirchhoffCoupledSystem.test.js tests/kirchhoffCoupledFrictionSolver.test.js tests/kirchhoffCoupledFrictionRuntime.test.js tests/kirchhoffBundleRuntime.test.js tests/kirchhoffCoupledCapacity.test.js tests/kirchhoffDirectSolver.test.js tests/kirchhoffDirectOptimization.test.js tests/kirchhoffDirectCoupling.test.js
```

Frozen prior and candidate modules are preserved at
`/tmp/oet-cone-frozen-11factor-20260906` and
`/tmp/oet-cone-frozen-skyline-20260906`.

## Bounded experiments excluded from this candidate

`kirchhoffCoupledBatchSolver.js` and its 28 small tests are a separate block
principal-pivot prototype, not imported by runtime. Simultaneous PDAS
classification released redundant normal constraints while a target was
infeasible, requiring the feasible fallback (13 factors total). Separating
batch primal fixes from dual releases avoids that failure but still takes
11 factors (6,2,1,1,1) on the captured case, so it is not promoted.

The transformed/original Gram comparison found no extra transformed-only
nonzeros here (15,441 versus 15,443 original, band 97 in both). Direct original
Gram assembly with common-relative recovery preserved the physical direction
to 2.63e-18 but did not reduce factors. That experiment remains in `/tmp`.

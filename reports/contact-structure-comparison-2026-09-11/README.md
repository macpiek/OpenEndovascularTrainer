# Contact structure comparison — 2026-09-11

## Result and decision

The full-band route is substantially faster on the frozen 400 mm system,
slightly faster at 200 mm, and slower at 100 mm. **Do not globally replace
the app default on this evidence.** The measurements support a future
selection of the band route for large contact systems, subject to a complete
nonlinear feed/hold/withdrawal replay. This change adds a reproducible
comparison harness and retained inputs; production physics and the default
`joint-active-coulomb` selection are unchanged.

Two independent batches ran in opposite variant order, each with six warmup
solves and 15 measured solves per variant and depth. All 252 results passed
the original-equation certificate. Reusing workspaces changed no multiplier
within either variant. The maximum cross-variant position-correction
difference was 1.68e-7 mm and rotation-correction difference 1.66e-7 rad.
These are frozen-step corrections, not a full-trajectory error bound.

| Catheter insertion | Original rows / band | Retained dense rows | Condensed median, ms | Full band median, ms | Full band result |
| --- | --- | --- | --- | --- | --- |
| 100 mm | 1243 / 128 | 63 | 3.6 | 6.0 | slower |
| 200 mm | 1627 / 132 | 176 | 17.5 | 15.6 | modest gain |
| 400 mm | 2370 / 133 | 400 | 102.7 | 35.4 | about 2.9× faster |

Rounded pooled medians; exact results are in `summary.json`. The 400 mm
gain also held separately in each batch (2.93× and 2.86×). At 200 mm it was
only 1.18× and 1.06×, so the crossover is not established precisely. A
depth-only threshold would be unjustified: contact activation also depends
on vessel shape, rotation, load, and the particular nonlinear trial.

The structural advantage is visible in the matrix sizes: the full band
stays almost constant while the retained dense contact problem grows from
63 to 400 rows. At 400 mm the final condensed replay spent approximately
90 ms in the contact solve itself, versus roughly 10 ms forming the Schur
matrix. The band route avoids that dense elimination/Newton path while
retaining all original constraints.

This does **not** establish 60 FPS: approximately 35 ms for a single linear
proposal already exceeds the 8.33 ms budget of an entire 120 Hz physics
step. Geometry, nonlinear trial measurement, rollback and repeated proposals
add further work. The preparation run reached 400 mm in 2561 steps and
recorded three nonlinear closure failures; see `capture-summary.json`.

## Scope and reproduction

This experiment compares the current active-condensed Coulomb solver against
the existing `full-band` route on the **same frozen equations**. It measures
the complete linear solve (including seed, elimination where applicable,
Newton attempts and multiplier recovery), not just one LU factorization.
It does not measure browser FPS or the complete nonlinear physics step.

Source revision: `f0e9e6b1793dd49553ff182977732976830f9792`.
Environment: Node 24.6.0, macOS arm64.

```sh
node scripts/physics/compare-contact-structures.mjs capture
node scripts/physics/compare-contact-structures.mjs compare
```

The capture uses the runtime anatomy fixture, the position-history model,
120 Hz stepping and the shared tool runtime policy. Wire insertion is 600 mm;
catheter targets are 100, 200 and 400 mm. Both use 5 mm spacing. Stiffness
multipliers match the app defaults: wire 39/30.7 and catheter 58.1/87
(shaft/tip). Tool transport advances through the anatomy; positions are not
initialized directly at the target insertion.

Each file contains the first joint linearization at its requested depth,
including the full original band matrix, right-hand side, bounds, friction
groups, active-set hint, original physical Jacobian columns, inverse weights
and compliance. Inputs are Float64. Capture runs use the condensed solver;
the replayed alternatives cannot alter the input trajectory. The preparation
counter records nonlinear closure failures; these snapshots must not be
mistaken for a fully certified nonlinear trajectory.

After capture ends, solvers run sequentially in isolated workers. Each has
its own reusable workspaces and the same original active-set hint on every
repetition. The first timing includes cold allocation/JIT work; the reported
warm median excludes the first six solves. Defaults: 21 solves and a
30-second hard limit per worker, configurable through
`OET_STRUCTURE_REPETITIONS`, `OET_STRUCTURE_WARMUP_RUNS` and
`OET_STRUCTURE_TIMEOUT_MS`. `OET_STRUCTURE_REVERSE_ORDER=1` reverses solver
order. Checks run outside the timed solve.

Every repetition independently rebuilds the original `J W J^T + alpha`
operator, checks bounds and the final-load Coulomb conditions. The last
result also reconstructs each body's physical correction as `W J^T Δlambda`
and evaluates the original residual using those corrections. No contact
cutoff, lower stiffness, looser tolerance or changed timestep is used.

`comparison.json` (reverse-order batch) and `comparison-forward.json` store
raw timings, all certificates, diagnostics,
correction/multiplier differences and SHA-256 hashes of uncompressed inputs.
`depth-*.json.gz` retain the replayable equations. A frozen solve passing at
one depth does not prove that all subsequent nonlinear trials will pass or
that the full-band route is faster throughout a complete insertion.
`comparison-initial.json` retains the initial shorter seven-solve screen;
it is excluded from the pooled statistics above.

## Validation

`node --test tests/kirchhoffFullBandCoulomb.test.js tests/coupledSolverSelection.test.js`
passed 18/18 tests. The comparison script also passed `node --check`.

# Crossover performance scenarios

The optimization goal was stopped at the user's request on 2026-10-18. Sustained
60 accepted common physics steps per second throughout the moving crossover has
**not** been demonstrated. Rendering FPS and stationary throughput are separate
measurements.

## Reproducible inputs

- `001-demonstration.json` describes the user's initial manual demonstration.
  Its checkpoints are observations, not a complete input recording.
- `001-stage-05-replay.json.gz` captures the long-insertion state used by focused
  contact and worker regressions. The contact-tail, long-hold and temporal-step-99
  fixtures preserve difficult solver states used by the automated tests.
- `003-crossover-full-user-inputs.json.gz` is the complete user recording: 7,507
  accepted steps at 1/60 s, including 3,645 steps with tool movement and five
  operator-marked stages. Its gzip SHA-256 is
  `bf2067e132ee933e052f39c4a68ba8d1f7e0c2fada445cedc86177a70324dbac`.
  The final insertions are 970.933333 mm for the guidewire and 934.866667 mm for
  the catheter.
- `001-checkpoint-result.json` and `001-continuation-result.json` support the
  historical browser replay in `tests/crossoverReplay.browser.html`.

Run `npm run dev`, then open `tests/crossoverAlternativeSolvers.browser.html`
to compare solvers using the full recording. `npm run benchmark:crossover` and
`npm run benchmark:crossover:throughput` use the captured long-insertion state;
they do not measure the complete maneuver. Automated regressions run through
`npm run test:physics:shared-axis`.

## Application defaults and experimental scope

The application defaults to `shared-axis` Kirchhoff with a 5 mm base mesh,
`fastNewton` and `predictiveNewton` enabled, and `modifiedNewton` and
`pruneWitnesses` disabled. Explicit URL parameters override these defaults.
The opt-in manual PD crossover preset uses the fine mesh, 16 local/global
iterations, geometric closure, cached geometry and scratch rotation buffers.
It does not certify Newton force equilibrium.

The worker remains an opt-in crossover benchmark prototype. Later experimental
changes described by reports 025–030 were tested in an isolated source tree;
their reports and patches do not mean those changes are enabled in this
checkout. In particular, the Schur experiment did not improve long-stage cost,
and the final deferred-projection-basis experiment had only targeted regression
tests before work stopped, with no completed scenario benchmark.

## Recorded results and limitations

- `003-full-scenario-validation.json`: baseline moving throughput 32.73 Hz,
  long-stage moving throughput 22.63 Hz; worker confirmations about 39.4–39.8 Hz
  and 28.0–28.6 Hz respectively, with identical recorded poses in those runs.
- `020-relaxed-mesh-validation.json`: experimental full-app moving throughput
  46.89 Hz and long-stage moving throughput 38.44 Hz. This used an adaptive mesh
  and relaxed force tolerance; it is not the reference accuracy configuration.
- `027-projective-full-app-validation.json`: complete PD application replay,
  26.52 Hz during movement and 14.25 Hz during long insertion, with another
  simulator active on the host. This is not an isolated performance comparison.
- `029-schur-validation.json`: both complete paired trajectories passed coarse
  route checks, but the candidate's long-stage CPU cost was slightly worse.

Shape-difference bounds evolved during the investigation: 0.1 mm, then 1 mm,
then functional crossover acceptance at the user's request. Historical reports
retain the policy that applied when they were generated. The current research
policy is in `004-optimization-policy.json`. Coarse route checks alone do not
establish realistic motion or absence of jitter throughout the maneuver.

Validation summaries, source manifests, served-source audits and research
patches are committed as historical evidence. The approximately 1.3 GB of raw
generated trajectories, CPU profiles, logs, screenshots and source archives
remain local and are ignored here. References to those local files or temporary
workspaces in historical reports are not available in a fresh checkout. Re-run
the relevant harness to generate new raw results; do not treat a summary as a
fresh validation of the current source tree.

The pre-commit checks from 2026-10-18 are recorded in
`031-commit-validation.json`. The build passes, but the full test inventory is
not green; the report distinguishes the corrected memory-test contract from
failures reproduced at the parent revision.

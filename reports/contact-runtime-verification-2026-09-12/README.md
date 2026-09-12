# Runtime contact-solver verification — 2026-09-12

This is a paired, headless replay of the actual actuator/world adapter and
transformed anatomy. Both trajectories receive the same commands and evolve
independently. It is not a browser FPS test. Production solver selection and
physics parameters are unchanged.

## Decision

**Do not enable automatic band selection in the app yet.** Unconditional
catheter band solving has a reproducible nonlinear-closure regression. A
separate delayed-switch trial also fails the conservative motion-parity
screen. The earlier 2.9× gain remains a valid frozen-linear-system result;
it has not been verified as a full-runtime gain with preserved behavior.

## Delayed switch above 300 mm

The two fixtures used the current solver through 300 mm, with identical
positions and frames. They completed hold/short withdrawal at 100 and 200 mm.
There was one common nonlinear closure failure during preparation, reproduced
in both fixtures before any band activation; it is not a band regression.

After band activation, the first logged parity violation was at 301.6 mm:
0.0185 mm position difference and 0.00341 rad frame difference. By 319.8 mm,
the log had recorded maxima of **0.1591 mm** and **0.1421 rad**. Logged steps
in this feed phase remained finite and had no nonlinear closure failures.
Thus this trial establishes loss of agreement with the reference trajectory,
not an additional nonlinear failure or a proof that both converged shapes
are physically impossible. The parity screen uses 0.001 mm / 0.001 rad
thresholds and is intentionally conservative.

The owned benchmark was interrupted after the observed parity-gate failure.
**400 mm hold and withdrawal were not reached.** Individual deep-step timing
buffers had not been flushed, so no deep runtime speedup is inferred from
combined elapsed times or from the earlier frozen-matrix timings.

See `deep-switch/stopped-on-parity.json` for the explicit stop/coverage record,
`deep-switch/progress.jsonl` for the retained observations, and
`deep-switch/runtime.json` for the completed preparation phases. Physics
source hashes were unchanged during the experiment. This does not establish
a production crossover threshold: 300 mm was a diagnostic activation point.

## Confirmed failure with band solving throughout catheter insertion

With **identical guidewire preparation at 600 mm**, the band route completes
catheter insertion to 100 mm but fails nonlinear closure on hold step 26
(global step 1894, about 0.217 s after release). The current condensed solver
converges on that same command. A second run reproduces the identical failing
step, final pose fingerprints and position/orientation differences.

| Phase | Current solver mean full-step time | Band route mean full-step time |
| --- | ---: | ---: |
| Catheter feed 0–100 mm, 231 steps | 23.2 / 23.8 ms | 35.4 / 35.6 ms |
| First 26 hold steps at 100 mm | 79.4 / 79.8 ms | 206.1 / 210.7 ms |

The two values are independent runs. Timings include actuation, synchronization
and the entire world step. Variant execution order alternates each step;
the solves do not run concurrently. The second run's short unit-test process
finished during wire preparation, before the catheter timing phases.

The failure is **not a failed LU factorization**:

- Every linear proposal in both controlled runs reported convergence.
- At failure, the band's final original/reconstructed linear residual is
  1.3678e-4; the linear solve accepts it at the requested tolerance.
- The final band LU backward error is 3.15e-17, with zero linear residual failures.
- The outer iteration nevertheless retains a normal-contact residual of
  0.001573 mm and a friction displacement residual of 0.001996 mm, both above
  the 0.001 mm runtime requirement.
- Trial correction reduces the normal-contact term but increases the friction
  merit term from approximately 1.55 to 5.17. The line search reports eight
  unsuccessful trials and restores the preceding state.
- The current solver completes the same command with normal residual
  2.23e-5 mm and friction displacement residual 1.83e-4 mm.

The trajectories no longer reproduce the same shape: maximum node separation
at the failed step is 0.0902 mm; the maximum during the controlled feed is
0.1120 mm. This contrasts with the tiny correction differences measured for
one frozen linearization in the earlier report. The trace establishes outer
contact/friction convergence failure, but does not by itself identify which
earlier contact/history transition first causes trajectory divergence.

Artifacts:

- `common-wire/runtime.json`: first controlled run.
- `common-wire-repeat/runtime.json`: repeat with explicit acceptance gates.
- Each directory has `failure-poses.json.gz` with owned positions, frames and velocities.
- Root `runtime.json`: preliminary unconditional band run, including wire
  preparation, which failed already at 90.567 mm. It is not the controlled
  comparison above; the two prepared wires differed slightly in that run.

The controlled test stops on the first additional band-only closure failure.
It therefore does **not** validate withdrawal or 200/400 mm operation with
band solving enabled throughout insertion.

## Protocol and reproduction

```sh
node scripts/physics/verify-contact-structure-runtime.mjs OUTPUT_DIRECTORY
OET_BAND_FROM_MM=300 node scripts/physics/verify-contact-structure-runtime.mjs OUTPUT_DIRECTORY
OET_BAND_FROM_MM=300 OET_STOP_ON_PARITY=1 node scripts/physics/verify-contact-structure-runtime.mjs OUTPUT_DIRECTORY
```

Both variants use 120 Hz, the shared tool runtime policy, existing material
and friction defaults, 5 mm tool spacing, independent components, physical
trial snapshots and early trial rejection. The experimental app bookmark's
different outer-policy flags are not used to confound this comparison.

Planned sequence: wire to 600 mm; catheter to 100 mm, hold 30 steps, withdraw
5 mm; feed to 200 mm, hold 30 steps, withdraw 5 mm; feed to 400 mm, hold 30
steps, withdraw 20 mm. The default enables band solving only after catheter
insertion exceeds 0.5 mm. `OET_FULL_BAND_DURING_WIRE=1` reproduces the initial
unconditional routing. `OET_BAND_FROM_MM` is an experimental verification
switch, **not a production threshold recommendation**.
`OET_STOP_ON_PARITY=1` now stops and saves the full failure record automatically
on the first parity-gate violation; the older deep run was stopped externally
after observing that gate. Periodic `runtime-partial.json` files now also
retain unfinished phase timing buffers during longer tests.
The new automatic stop was exercised in `automatic-parity-stop/runtime.json`:
it returned exit code 2 at the first motion-parity violation, retained the
failure poses and marked later phases unexecuted. Both closures at that stop
were converged; the reason is correctly recorded as a parity-screen failure.

Reports retain source hashes, per-step times, closure/linear counters,
accepted-trial traces at failure, checkpoints and explicit coverage. A
failed acceptance returns exit code 2 in the current script. Pose-parity
gates conservatively compare accumulated differences against existing
runtime tolerances; they are not a physical error-bound theorem.

World `condensed*`, Schur and seed split timing counters are specific to the
condensed route; their zero values in the full-band route do not mean that
full-band seeding is free. Whole solve/step times are measured for both.

The focused linear-solver/selection suite still passes 18/18 tests. This is
consistent with a runtime failure despite individually certified linear
solves, and does not override the failed runtime acceptance.

# Frictionless sliding between catheter and guidewire

The application now creates its lumen containment with `surfaceFrictionEnabled: false` and its external tool contact with `friction: 0`. Wall and sheath contact, normal containment, reciprocal bending response and material properties remain enabled. This intentionally changes the physical assumption: axial and circumferential sliding have no inter-tool friction resistance. Normal forces and bending can still oppose motion geometrically.

The normal-only path:

- Omits the lumen tangential row batch, cone groups, multiplier commits, surface-motion/KKT evaluation and friction merit calculation. Cone correction cannot trigger from lumen friction.
- Does not allocate lumen friction evaluation, gradient, cache or correction workspaces in the runtime. Disabled residuals use one immutable zero contribution.
- Maintains normal contact identity, normal direction and normal loads without constructing or reprojecting the tangent basis/history. The common manifold schema retains small zero-valued tangent fields for compatibility.
- Skips external tool tangential rows before allocating their contact history when their coefficient is zero. Vessel wall friction is separate.

The reusable containment API retains friction support by default for reference/other callers. The application explicitly selects the frictionless mode at construction; this is not a hot-switch API. The existing test fixture keeps its legacy reference default and accepts `interToolFriction: false`. The profiling script selects the application's frictionless mode by default; `OET_INTER_TOOL_FRICTION=1` reproduces the old model.

## Measurement

Four sequential Node real-anatomy replays in ABBA order, each 2733 steps. Tools separately, wire at 309 mm with catheter feed/hold/withdraw/refeed to 100 mm, then simultaneous feed to catheter 120 mm, wire withdrawal and independent rotations. Controls, anatomy, stiffness, discretization, timesteps and tolerances are the same across variants. Physical trajectories may differ because friction has been removed. Desktop/browser activity remained present, so this is not an isolated machine or rendered-FPS benchmark.

| Phase, mean step cost across two runs | With friction | Without friction | Reduction |
|---|---:|---:|---:|
| Catheter feed to 100 mm | 69.71 ms | 24.59 ms | 64.7% |
| Hold at 100 mm | 79.60 ms | 23.49 ms | 70.5% |
| Refeed to 100 mm | 79.64 ms | 20.65 ms | 74.1% |
| Both tools advancing, catheter to 120 mm | 127.74 ms | 31.80 ms | 75.1% |

The earlier exploratory frictionless feed measured 15.86 ms. Absolute timing varies substantially with machine activity; compare the paired runs and work counts. This does not demonstrate 60 rendered FPS or real-time 120 Hz physics.

## Verification and remaining convergence failures

- 91 focused tests pass and the production build passes (existing Vite bundle-size warning only).
- New tests verify zero tangential-history reads during normal remapping, retained normal forces, no friction rows/workspaces, reciprocal transverse response, independent axial/rotational motion against the full-row zero-coefficient reference (1e-5 tolerance), and normal-contact rollback.
- All 2733 states in each frictionless replay contain finite poses, zero lumen friction rows and exactly zero tangential/twist multipliers.
- Both frictionless runs have identical physical hashes and iteration/rejection decisions at every step.
- Catheter feed/hold/withdraw/refeed through 100 mm has no reported failures in the frictionless replay. The old reference has one failure during catheter withdrawal.
- The deeper simultaneous-feed phase has **five nonlinear closure failures in both models**, at different later step indices. Removing friction does not fix normal/wall convergence. The frictionless replay's recorded maximum lumen/wall penetrations over that stress phase are 0.405/0.082 mm; these are recorded attempted-step diagnostics, including failed attempts, not a certificate of an accepted collision-free trajectory. The reference records 0.672/0.056 mm. No claim of equivalent stress trajectories is made.
- Subsequent independent rotations report no frictionless failures; the reference reports one failure in each rotation phase.
- The simulator source served by `http://127.0.0.1:5173/src/simulator.js` was checked and includes the new mode. The user's live controls/trajectory were not reset for benchmarking.

Reproduce:

```sh
OET_INTER_TOOL_FRICTION=1 OET_VERIFY_WITHDRAWAL=1 OET_VERIFY_BOTH=1 node scripts/physics/profile-overlap-309.mjs /tmp/with-friction.json
OET_INTER_TOOL_FRICTION=0 OET_VERIFY_WITHDRAWAL=1 OET_VERIFY_BOTH=1 node scripts/physics/profile-overlap-309.mjs /tmp/without-friction.json
node --test tests/kirchhoffFrictionlessLumen.test.js
```

`comparison.json` contains per-phase timings, work counts, failures, geometric diagnostics and source hashes. Compressed trajectories preserve all four runs; test/build logs are included.

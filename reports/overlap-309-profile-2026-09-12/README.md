# Guidewire 309 mm / catheter 100 mm runtime profile

Reproduction of depths observed in the live UI, with Glidewire/Berenstein and shaft/tip scales 39/30.7 and 58.1/87. The existing browser trajectory was not reset. This is a Node replay against the same anatomy and default joint-active-coulomb policies, not an export of the exact browser contact history or a rendered FPS measurement. The user subsequently changed the live wire depth to 152 mm while profiling was running.

Run: `node scripts/physics/profile-overlap-309.mjs /tmp/oet-overlap-309.json`. For the sampling repeat add Node `--cpu-prof`. No production physics changes.

Feed statistics cover the final 47 steps of each depth interval (about 20 mm); hold covers the first 60 steps after insertion stops. Physics timestep 1/120 s. There is no settling hold between wire preparation and catheter feed. The second execution included a sampling profiler, so it is a corroborating run, not a controlled timing A/B comparison. All recorded nonlinear closures converged in both runs.

| Case | Baseline mean full step | Repeat mean full step |
|---|---:|---:|
| Wire alone approaching 309 mm | 1.94 ms | 1.73 ms |
| Catheter alone approaching 100 mm | 1.39 ms | 1.43 ms |
| Catheter over wire approaching 100 mm | 75.92 ms | 65.45 ms |
| First 60 hold steps at 309/100 mm | 100.51 ms | 71.04 ms |

Baseline overlap feed mean cost per step:

- Nonlinear residual measurement: 29.91 ms (39%).
- Trial state capture: 20.12 ms (27%).
- Coupled system solve including internal assembly: 15.69 ms (21%). Its nested costs include contact solve 3.68 ms, Schur 2.28 ms, condensed setup 2.41 ms and seed 2.08 ms; these are not additional top-level costs.
- Boundary/contact/friction row assembly outside the solve: 7.40 ms (10%).
- Correction apply: 1.57 ms; restore: 0.43 ms.
- Separately timed initial/final narrow phase: 0.135 ms. Geometry/contact work also occurs within assembly and residual measurement, so this is not all collision-related work.

Overlap needs 6.49 outer solves, 6.70 trial evaluations, 12.19 full residual measurements and 5.49 snapshots per step. Backtracks average only 0.21: ordinary repeated nonlinear passes dominate, not a cascade of rejected steps. Snapshot object count averages 9373 (the counter is the per-step maximum, not newly allocated objects); typed bytes average 788172, excluding JS object memory. Compared with 2.64 passes for wire and 1.85 for catheter alone, this multiplies both the work and its per-pass size.

CPU sampling confirms expensive refresh/visit/captureValues in kirchhoffCoupledTrialState, friction stencil/surface preparation, containment geometry reconstruction, and coupled assembly. Dense LU is not the largest hotspot in this configuration. The CPU profile includes fixture/anatomy startup: exclude those stacks when interpreting runtime costs.

At the final overlap feed sample there are 63 active wire nodes and 22 catheter nodes, 504 DOFs and 494 material equality rows. The basis is common-relative with 21 paired nodes: pairing is a coordinate transformation, not removal of relative DOFs. `assembleKirchhoffCoupledSystem` still assembles each body's material equations and appends contact and friction equations. Thus the present joint solve is not yet a single centerline with only locally changed material properties.

Relevant code: src/physics/kirchhoffCoupledSystem.js:48, src/physics/endovascularPhysicsWorld.js:2690, :2860, :3041, src/physics/kirchhoffCoupledTrialState.js:99 and :284, src/physics/kirchhoffCoupledFrictionRows.js:189.

Recommended next work: replace generic traversal of the trial graph with an audited compact mutable-state representation; avoid recomputing identical geometry and friction kinematics within one unchanged candidate state. Reuse must invalidate after pose, contact-feature or normal-load changes and preserve rollback/contact history. These target the measured dominant cost without changing physical behavior. Reducing to one mechanical axis is a separate modeling change requiring preservation of sliding, twist, tip emergence and wall interactions.

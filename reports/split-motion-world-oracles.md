# Independent World oracles for split physical/bias motion

Test file: `tests/kirchhoffSplitMotionWorld.test.js` (8 tests). Runtime is loaded from `OET_SPLIT_MOTION_SOURCE_ROOT`; without that variable it uses the repository containing the test. No source files are changed or copied into root. No test has a skip, todo, expected-failure, compatibility fallback, or alternate success condition.

## Execution contract

Agreed with model task `01a075f0-f24d-7321-a7b9-f87a0f48f327`: `World.getStats().jointMotion` reports the **executed** path with `mode`, `physicalDt`, `physicalPasses`, `biasPasses`, `historyCommits`, `rejectedTrials`, `rollbackCount`, `certified`, `physicalKKTResidualMm`, `physicalConeViolation`, `reactionUnits:'xpbd-multiplier'`, `impulseScale:1/physicalDt`, and `contacts`. Each contact reports `kind`, `normalPhysical`, `normalBias`, `tangentPhysical:[t1,t2]`, and `mu:[mu1,mu2]`. The implementation can additionally report `limitations`.

The tests require actual calls to the real coupled solve/apply, both phase counters positive, an accepted physical step with exactly one history commit, independent World convergence, physical KKT within the existing World tolerance, and cone violation at most `1e-9`. A constructor that merely stores or ignores the new option cannot pass. Phase reactions are diagnostics, not the sole oracle: load cases independently compare them to measured wire momentum changes.

`debugJointTrial` supplies `state.motionPhase`. A real physical trial at pass 1/trial 0 is rejected through the existing merit gate (`settled=false`, `merit=Infinity`), and the final body/history state is compared with a control run. Both runs keep the first converged physical candidate open for one additional iteration. This deliberately tests an extra candidate after convergence; it does not claim coverage of large nonlinear backtracking trajectories or topology changes.

## Fixtures and independent expectations

Each fixture contains a three-node free wire and a five-node, statically supported catheter. They use native World, material rods, coupled assembly, solve, apply, contact refresh, and velocity reconstruction. All damping coefficients are 1. Legacy projection retention coefficients are 0, so retained physical motion cannot be supplied by an incidental legacy retention setting. The catheter is a fixed external support; momentum conservation of the two-body free system is therefore not asserted.

| Test | Geometry oracle | Physical oracle |
| --- | --- | --- |
| Initial wall overlap, zero input | Plane `y <= 0`, wire radius .5, initial center y=-.25; correction exceeds .2 mm; final segments retain their lengths | Every v and omega is zero; positive bias reaction; zero physical normal/friction reaction; kinetic energy includes material spin |
| Nested bias-only contact and axial slip | Straight cylindrical lumen, centerline clearance .5, initial radial offset .75 | With mu=.3 and vx=4, physical normal and friction budgets are zero; axial motion/energy survives |
| Nested incoming normal velocity | Initial radial offset .5 and vy=1, vx=4 | Positive physical normal and friction impulses; sum(lambda_n)/dt equals lost wire normal momentum; each physical ellipse and total friction budget are respected; kinetic energy cannot increase |
| Nested normal force | Same contact, Fy=120 per unit-mass node, dt=1/120, vx=4 | Predictor vy=1 as above; catches classification based only on stored incoming velocity instead of force-integrated physical motion |
| Away/tangent motion and wire-only spin during wall bias | Initial overlap as above | vx=3, vy=-2, omega_x=2 are retained; material frames equal the independent Three.js world-axis rotation; catheter remains stationary |
| Free torsional prestrain | Straight, unconstrained wire; two free segment inertias I=1; relative twist theta=.2; C=.02 | Analytic implicit update: delta=theta*dt^2/(C+2*dt^2), omega_0=delta/dt, omega_1=-delta/dt; remaining twist theta-2delta; nonzero recoil and nonincreasing kinetic plus elastic energy |
| Hard controls and history over two physical steps | Exact axial targets and material-frame target; fixed catheter; wall tangent placement | Controlled v=2 and omega=1; no geometric drift; one history commit per physical dt, including the second step |
| Rejected physical trial | Same native torsional fixture, one real rejected candidate, actual rollback count | Accepted pose, v, omega, previous positions/frames, and energy match the clean run; one history commit |

The affine collision adapter calculates signed gap directly as `-y-radius`; endpoint convexity makes the segment geometry check exact for this plane. The cylindrical oracle uses radial distance to the straight supported catheter and checks that all witness coordinates stay within its finite span. These checks do not call production contact-gradient, reaction-ledger, or KKT helpers.

## Tolerances and scope

World physical acceptance tolerances are unchanged. Small analytic fixtures additionally require position/length error at most `2e-6` mm, velocity error at most `5e-4` mm/s, and angular velocity error at most `2e-5` rad/s. The position and velocity bounds account for Float32 storage near coordinates of order 1 mm and reconstruction over 1/120 s. Zero physical reactions use `1e-10` in XPBD multiplier units. These bounds are set before running the candidate implementation.

Energy includes both translation and rotation using material-frame inertias and an independent quaternion transform. For pure torsion the elastic energy is theta^2/(2C); no production energy diagnostic supplies the expected value. The normal load fixture compares summed normal reaction with measured momentum; per-contact ellipse checks independently use the reported **physical** load, never physical plus bias.

This bounded suite does not certify long curved vessels, distal portal transitions, broad material parameter sweeps, general finite-rotation momentum conservation, nonlinear rejection with large corrections, or runtime performance. The separate native ownership force/moment proofs remain relevant and are not duplicated here.

## Current validation

- `node --check tests/kirchhoffSplitMotionWorld.test.js`: passed.
- One anti-vacuity run against legacy root `/Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer`: **0 passed, 1 failed, 0 skipped, 0 todo**, at the explicit requirement for diagnostics from the executed split-motion path. The test body took about 15 ms. This is a failing implementation result, not an expected-failure success.
- First frozen candidate batch, source `/Users/macpiek/.codex/worktrees/8996/OpenEndovascularTrainer`: **5 passed, 3 failed, 0 skipped, 0 todo**, total 168.6 ms. All eight tests were unchanged. Passed: initial wall bias; away/tangent/wire spin; analytic torsion recoil/energy; hard controls over two physical steps; rejection/rollback/history. All three nested tests threw `TypeError: bank.contact[key].set is not a function` in `finishKirchhoffSplitBias`, `src/physics/kirchhoffSplitMotion.js:243`, before reaching their physical oracles. Their physics is therefore not yet certified. Failure-first was sent to model/root; no implementation or test tolerance was changed.
- Second frozen candidate batch, after the model task fixed mixed Array/TypedArray restoration and normalized the diagnostic `kind:'lumen'`: **8 passed, 0 failed, 0 skipped, 0 todo**, total 317.5 ms. Independently rerun against 8996 with the identical test file and tolerances. All nested cases now reach and pass the physical reaction, momentum, cone, geometry, and energy checks. The 53 JavaScript files under `src/physics` have identical hashes before and after the run. Full output: `reports/split-motion-world-checkpoint2-tests.txt`; source identity, test SHA-256, command, and timestamps: `reports/split-motion-world-checkpoint2-source.json`. This certifies only the bounded cases above; it does not establish integrated root acceptance or broader runtime coverage.

Mechanics checks actually reached in checkpoint 2: both executed phase counters and real solve/apply calls; physical timestep and multiplier-to-impulse conversion; one history commit; `certified` and World closure convergence; physical KKT and cone gates; independent final raw plane/cylinder gaps and segment lengths; physical linear/angular velocity and kinetic energy; separate bias/physical reactions; per-contact Coulomb ellipses; incoming-velocity and force-driven normal momentum balance; analytic compliant torsional recoil and elastic-plus-kinetic energy; exact prescribed position/orientation; and an actual rejected physical trial with rollback and final history comparison. No nested case remains unconverged, so no additional failure-only raw-gap/velocity probe was run. The passing output records assertions, not a numerical trace of each diagnostic field.

Run the unchanged suite against a frozen tree or the integrated root:

```sh
OET_SPLIT_MOTION_SOURCE_ROOT=/absolute/path/to/frozen/OpenEndovascularTrainer node --test --test-concurrency=1 tests/kirchhoffSplitMotionWorld.test.js
```

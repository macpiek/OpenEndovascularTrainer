# Independent positive-friction wall tests

Final test file: `tests/kirchhoffSplitWallWorld.test.js`, exactly three native World tests. The earlier eight tests are unchanged. The new tests load the selected runtime with `OET_SPLIT_MOTION_SOURCE_ROOT` and never change production code. Both wall coefficients are explicitly equal: `mu_static = mu_kinetic = .3`. Unequal coefficients remain explicitly unsupported and are outside this suite.

## Final result

On a reconstructed copy of the exact announced 8996 freeze: **3 PASS, 0 FAIL, 0 skipped, 0 todo**, total 117.1 ms. The test observations below are from the completed native Node test output, not estimated values. `reports/split-wall-world-exact-freeze-tests.txt` contains the full output; `reports/split-wall-world-exact-freeze-source.json` records the source/test hashes and command. `reports/split-wall-world-final-observations.json` summarizes the measurements. No scene or benchmark was run.

Source provenance: after the initial wall batch, the model began a separately assigned optional bias-mode subtask and changed `kirchhoffSplitMotion.js` and `kirchhoffCoupledSystem.js`. A later hash capture therefore could not establish the source identity of the preceding passing batch. To resolve that ambiguity, an isolated runtime copy was overlaid with the immutable `/tmp/oet-split-final-8996/after` files; all 53 physics JavaScript hashes were required to match the first announced freeze before execution. The final three tests then passed on that exact copy, with the 53 hashes unchanged before/after the run. No source file in the model or root workspace was changed by this validation.

| Case | Actual path | Raw final gap (both nodes), mm | Physical result |
| --- | --- | --- | --- |
| Normal approach plus axial slip | physical 1, bias 1, three physical wall-friction groups, history commit 1 | 0 | vx=3.6999964714 from incoming 4; abs(vy)<=3.34e-9; omega_z=9.99999e-9; KKT=1.1438848271e-9 mm; cone violation 0 |
| Initial .25 mm overlap plus axial slip | physical 1, bias 2, three physical wall-friction groups, history commit 1 | 0 | vx=3.9999961853; vy=omega=0; positive bias normal reaction, exactly zero physical normal and tangent reactions |
| Next dt after removing pressure and prescribing separation | physical 1, bias 1, three physical wall-friction groups, history commit 1 | .01666665077 | vx=2.999997139, vy=-1.999998093; abs(omega_z)=1.12e-9; physical contact reaction list empty; KKT/cone zero |

All cases report the executed `split-physical-bias` mode, `certified:true`, no limitations, and zero swept witnesses. The last condition is required because the prototype does not include swept contact reactions in its public contact totals. Both capsule (`wall:0:0`) and endpoint (`wall-point:0:0`, `wall-point:0:1`) contacts actually enter the native `split-wall-friction` row/group path. Merely reporting the requested mode cannot pass.

## Impulse and moment oracle

The loaded case starts at the plane with two unit-mass nodes, radius .5 mm, incoming vx=4 and vy=1, dt=1/120. Public physical normal and tangential multipliers are converted to impulse by division by dt. Their sums independently match the measured loss of wire momentum. Each contact satisfies the physical-load Coulomb disk; the bias reaction is never included in that budget.

For each physical solve/apply, the observer saves native witness indices and interpolation parameters but does not read the emitted gradients or invoke the production surface/torque/ledger helpers to construct an expected reaction. The affine plane independently supplies normal -Y and tangent axes +X/+Z. The contact point is the interpolated rod center plus the known radial lever +Y*.5. Cross products then give the external force and moment.

The complete applied generalized response, including material terms, is separately converted from displacement/local rotation to impulse using the body's mass and material inertia. Three.js maps the rotational impulse to world coordinates. This must equal the independently constructed surface wrench. The test would detect a centerline-only tangential force that omitted its radial torque.

Final loaded observation:

- Full-response and independently expected impulse: `[-.5999994219161056, -1.9999980909847022, 0]`.
- Full-response angular impulse about the fixed world origin: `[0,0,-3.361663385225447]`.
- Independent surface angular impulse: `[0,0,-3.3616633852254463]`.
- Nonzero radial contribution: `[0,0,.2999997109580528]`.

These instantaneous generalized-impulse checks are distinct from finite-step angular-momentum conservation. The suite checks final physical linear velocity, geometric gap, and load release, but does not claim general finite-rotation conservation from the per-apply moment identity.

## Initial diagnostic failure and fixture correction

The first version used a 1-mm edge at x=[-.5,.5]. That batch was **2 PASS, 1 FAIL**, retained in `reports/split-wall-world-checkpoint1-tests.txt` and its source manifest. The loaded case passed physical KKT, cone, and linear impulse checks but missed a `1e-9` moment threshold by approximately `8.94e-9`.

An added read-only observation showed the pre-apply edge was `[1.0000000298023224,0,0]`: adding vx*dt to Float32 positions with different exponents had introduced `2.98023224e-8` mm of material strain. The resulting unconverged material torque contaminated the intended wall-only identity. The discrepancy was approximately this length error times the .2999997 radial contribution. The next physical iteration had exact edge length 1 and matched the moment identity to below `1e-18`.

The final fixture uses a .5-mm edge at x=[1.25,1.75]. Both coordinates stay within the same Float32 exponent interval during these small steps, so uniform prediction preserves the exact .5-mm edge. This is a correction to the analytic fixture, not an engine fix or a loosened threshold. Mu=.3, incoming velocities, dt, all production source, and **all assertion tolerances remain unchanged**. The first failure is not relabeled as a passing run, and this suite does not certify the material torque identity away from the exact-constraint state.

## Run

```sh
OET_SPLIT_MOTION_SOURCE_ROOT=/absolute/path/to/frozen/OpenEndovascularTrainer node --test --test-concurrency=1 tests/kirchhoffSplitWallWorld.test.js
```

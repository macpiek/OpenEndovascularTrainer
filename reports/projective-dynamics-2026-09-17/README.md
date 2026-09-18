# Experimental Projective Dynamics rod

Select **Debug → Solver fizyki → Projective Dynamics — pręt podatny, siatka adaptacyjna**
and apply the restart, or open `?coupledSolver=shared-axis-projective&solverDebug=1`.
The default remains the reference Kirchhoff solver. Both Kirchhoff meshes remain
selectable. PD uses the adaptive mesh controls and accepted-node overlay; Modified
Newton is disabled for this variant.

## Model and algorithm

This implements an experimental Cosserat penalty rod using the local/global
principle of [Bouaziz et al., Projective Dynamics (2014)](https://www.projectivedynamics.org/).
It is an alternative constitutive model, not an exact reformulation of the
existing inextensible Kirchhoff/KKT system.

The generalized vector variables are one shared centerline plus three directors
per edge **for each material**. Intrinsic rotations and bend/twist stiffnesses come
from the existing profiles; wire and catheter retain independent proximal spins.
Each global solve is three right-hand sides of the same scalar SPD band system.
A private Cholesky factor is reused within a substep, rebuilt for new contact
projectors, and discarded on feed/remesh, a new timestep or cancellation. There
is no fallback to Newton.

The terms are:

- inertial prediction for positions and chordal director inertia;
- local projection of each edge onto its rest-length sphere;
- a quadratic stretch/shear penalty linking the centerline tangent to the third director;
- quadratic adjacent-director differences with the integrated intrinsic rotation;
- local proper-rotation SO(3) projection of each director triad using a symmetric
  quaternion eigensolve (including reflections and 180-degree rotations);
- local linearized contact/bend-limit projections with a constant positive penalty.

The stretch/shear scale is 2e6; short material cells retain at least a 5 mm-cell
shear penalty. Without this floor, short moving pigtail tips developed excessive
shear. This regularization deliberately sacrifices mesh-invariant shear stiffness
for this prototype. Directional bend/twist weights match the profile's small-angle
rigidities when compatible; this is a chordal energy at larger rotations.

After solving, directors are orthonormalized and aligned to the tangent for the
existing native publication/remeshing contract. Their pre-alignment angular
mismatch is checked before accepting the step. Generalized director strain is
therefore not retained as an independent physical history across timesteps.

Wall discovery uses the existing sheath and vessel capsule samples. Trials that
cross the signed vessel surface are shortened. Friction is an approximate
positional Coulomb return including axial feed, bounded by the penalty normal
correction. It does **not** preserve the reference solver's persistent sticking
history or surface-spin friction. No force/KKT/friction residual certificate is
reported; the shared quality recorder uses `null` for that certificate.

There are at most 16 local/global iterations per substep. Small iterate change
(0.002, with director changes scaled by edge length) can stop earlier after at
least eight iterations. A step reaching the budget may still be published if
finite and within the geometric limits: 3% relative length error, 0.1 mm wall
penetration, 0.1 unit-director/tangent distance (about 5.7 degrees), and the native
bend limit plus 2 degrees. The UI explicitly distinguishes iteration-budget
acceptance from iterate-change convergence. Neither means the original Newton
force certificate was satisfied. Failed quality checks trigger the existing
1/2/4/8 substep retry; the complete requested step is published atomically.

## Measurements

Results and commands are recorded in the JSON summaries alongside this document.
The short comparison uses the same input commands, material profiles, timestep
and adaptive mesh budgets; it compares two different physical approximations.
The resulting meshes and contact sets can differ. A single sequential Node pair
is not a browser FPS claim or a confidence interval.

Short comparison: **838 / 838 accepted steps**, wire 300 mm and catheter 240 mm,
then simultaneous feed, rotation and withdrawal. Both runs used the current UI
stiffnesses (wire 9.6/6.8, catheter 40.65/66.8), UI masses/radii and dt = 1/60 s.
The PD run preceded the reference; no other Node benchmark/test/build ran during
this pair. Vite and the local preview remained open. There is no randomized order
or repeated-trial uncertainty estimate.

| Phase | Kirchhoff mean ms | PD mean ms | Change in time |
| --- | ---: | ---: | ---: |
| Wire feed | 18.10 | 13.22 | −27.0% |
| Catheter feed | 31.76 | 33.41 | +5.2% |
| Simultaneous feed | 25.67 | 33.98 | +32.4% |
| Rotation | 27.64 | 33.10 | +19.8% |
| Withdrawal | 32.19 | 31.77 | −1.3% |

Whole trajectory: 20.547 s → 19.623 s (**4.5% less CPU time**). This small overall
change is not convincing evidence of a general speedup: combined motion and
rotation are slower. The largest tool-tip deviation between the independent
trajectories is **21.61 mm**. PD's maximum relative segment error is **1.142%**,
maximum sampled wall penetration **0.0501 mm**, and director/tangent mismatch
0.0394. **569 / 838 steps hit the local/global iteration budget** rather than
iterate-change convergence. Geometric acceptance does not bound trajectory error.

Pigtail validation: **1096 accepted steps**, wire 300 mm, catheter 200 mm, the same
combined/rotation/withdrawal sequence, then wire withdrawal to 150 mm. All states
finite; maximum segment error **1.152%**, sampled penetration **0.0556 mm** and
frame mismatch **0.0721**. This is a functional check; other development activity
ran concurrently, so its timings are not a speed comparison.

The short pair predates only a correction to clear stale *published contact
counts* on released contacts; that correction does not change the physical
solution, contact discovery, stored reactions or local/global solve. The pigtail
run additionally predates reporting the complete default PD parameters. Hashes
in each summary identify the actual measured source.

## Verification

The focused selection/provider/PD/quality/UI suite passes **54/54** tests,
including SPD equation residuals, proper-rotation projections, free rest shape,
rotation covariance, proximal twist propagation, contact correction/release,
atomic cancellation and rejection, exact material endpoints after remeshing,
and a stiff pigtail with moving sub-millimetre tip cells. Build and generated API
documentation checks pass.

The browser smoke selected PD through Debug, confirmed enabled adaptive controls,
a disabled Modified Newton checkbox, the generalized DOF count and live PD
quality counters. A motion preset reached 562 accepted steps without console
errors, then the scene was reset with PD still selected. This smoke was not a
completed browser performance acceptance workload.

The complete shared-axis suite reports **277 tests: 274 passed, 2 failed, 1 skipped**.
The two failures are the previously verified baseline failures in
`kirchhoffSharedAxisAnatomyRegression.test.js` (terminal witness discovery) and
`kirchhoffSharedAxisLiveWallAnatomy.test.js` (expected fallback count). No additional
failure appeared. See `shared-axis-tests.log`, `focused-tests.log` and `build.log`.

Final long validation: **1663 accepted steps**, wire and catheter to 600 mm,
then simultaneous feed, rotation and withdrawal. All states finite, no rejected
complete step. Maximum segment error **2.793%**, sampled wall penetration
**0.0656 mm**, director/tangent mismatch **0.0457**.
Tests, build and browser smoke ran concurrently; this is a functional validation,
not a paired speed comparison. See `full-validation.json`.

## Reproduction

```sh
SHARED_AXIS_PROJECTIVE=1 SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_WIRE_MM=300 SHARED_AXIS_CATHETER_MM=240 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/pd-short
SHARED_AXIS_ADAPTIVE_MESH=1 SHARED_AXIS_LIVE_WALL_NORMAL=1 SHARED_AXIS_WIRE_MM=300 SHARED_AXIS_CATHETER_MM=240 node scripts/physics/profile-shared-axis-dynamic.mjs /tmp/kirchhoff-short
node scripts/physics/compare-projective-dynamics.mjs /tmp/kirchhoff-short/profile.json /tmp/pd-short/profile.json /tmp/pd-comparison.json
```

Use `SHARED_AXIS_PD_ITERATIONS` to change the iteration budget in the profiling
script. The application uses the exported default of 16. For the long validation,
set both insertion targets to 600. For the pigtail validation, set wire 300,
catheter 200, `SHARED_AXIS_CATHETER_TYPE=pigtail` and
`SHARED_AXIS_WIRE_WITHDRAW_TO_MM=150`.

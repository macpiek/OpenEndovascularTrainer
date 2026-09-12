# Optional World physical/bias prototype — frozen handoff

Status: bounded experimental JOINT implementation. The default remains `jointMotionMode: 'position-history'`; opt in with `'split-physical-bias'`. This checkpoint is suitable for integration and further mechanical closure work, not a claim of full runtime acceptance. No rest shape, material compliance, tolerances, retention profile, discretization or solver backend default was changed.

## Implemented contract

One physical timestep captures immutable start poses and postforce physical velocity before CCD. A physical JOINT solve updates physical linear velocity and world angular velocity using the same common-scale correction applied to the pose; the local angular increment is rotated by its pre-apply material frame. Hard orientation targets prescribe angular motion from the step-start frame. Existing configured damping runs once on incoming physical motion.

Physical contact rows use positive initial clearance plus `dt J vphysical`. Surface strain uses weighted physical point velocity including `omega x lever`; the existing surface-force gradients remain unchanged. Physical wall friction uses the same surface law against a stationary zero-mobility witness and couples its normal unknown into the simultaneous Coulomb solve through `groups[].normalRow`. Endpoint sphere witnesses supplement existing capsule-interior wall rows. Sweep planes are retained instead of feeding a positional TOI clamp into physical velocity.

A separate bias JOINT solve repairs actual geometry with zero physical-friction rows and distinct multiplier banks. Material RHS offsets preserve the physical phase strain reference while original native J/W/compliance/rest data remain unchanged. Bias pose changes never increment physical velocity. Bias elastic energy change and hard strain are reported explicitly.

At finish, physical multiplier banks are restored and actual geometric material equations, current contact/friction equations, controls, folds and outward contact velocities are checked again. A successful first phase cannot certify a later elastic deformation: `physicalPhaseMaterialResidual` and `finalMaterialResidual` are distinct. Recycled manifold objects must preserve membership, id, both material labels and feature before their physical bank is restored. A mismatch leaves stored old values in `_splitMotion.bank.contacts`, does not transfer Fn/Ft, and forces explicit uncertified status. It is not a mechanically reconciled release.

Published body velocities come from the physical channel; legacy pose-derived velocities and contact projection-retention corrections are bypassed for the split pair. Accepted history is recorded exactly once per accepted dt. A failed certificate has `historyCommits: 0`.

## Frozen verification

- Original independent native World oracle, unchanged from 0827: **8/8 PASS** after both final guards. Covers stationary wall overlap, nested bias-only positive friction, true incoming normal load and Coulomb budget, force equivalence, away motion/material spin, analytic free twist recoil, hard controls over two dt and forced trial rollback. Log `/tmp/oet-split-independent-final-8996.txt`.
- Default SurfaceFriction, ExternalFrictionRows, CoupledSystem suite: **48/48 PASS**, log `/tmp/oet-split-default-final-8996.txt`.
- New `tests/kirchhoffSplitMotionCertification.test.js`: **3/3 PASS**. Stationary bias remains zero physical motion on next dt. Tilted stress-free rod receiving nonuniform wall bias is rejected using fresh material equations. A contact with a stable runtime id but changed material labels cannot inherit a prior physical reaction or commit history.
- Reviewer independent rechecks and three positive-wall-friction tests are coordinated separately by root; this handoff does not claim their pending outcome.
- No scenes, replays, full benchmarks, commits or root edits were performed for this checkpoint.

Reproduce local certification tests:

```sh
node --test --test-concurrency=1 tests/kirchhoffSplitMotionCertification.test.js
```

Independent original oracle against this source tree:

```sh
OET_SPLIT_MOTION_SOURCE_ROOT=/Users/macpiek/.codex/worktrees/8996/OpenEndovascularTrainer node --test --test-concurrency=1 /Users/macpiek/.codex/worktrees/0827/OpenEndovascularTrainer/tests/kirchhoffSplitMotionWorld.test.js
```

## Explicitly open results

The translated-mouth lifecycle proof (`reports/probe-translated-split-world.mjs`, output `/tmp/oet-translated-split-final-8996.txt`) now has physical Fn=0 and physical Ft=[0,0] on every sampled contact; geometric bias no longer creates the old spurious Coulomb force. Nevertheless its current witnesses require additional physical closure after bias. The final result is **certified=false, historyCommits=0**, physical KKT displacement residual **0.0016996188682287892 mm**, maximum outward contact velocity **0.20395426418745471 mm/s**, cone violation 0 and bias elastic energy change **1.2561989880728693e-6**. Both phaseAccepted flags are true; they are not the final certificate. No outer physical/bias reclosure loop has been added.

The review's tilted stress-free 3-node wire (`y=-.5+.2*(i-1)`, radius .5) exposes a much stronger issue if the phase material residual were reused: bias changes elastic energy by about .15582 and leaves about .07895 rad actual material residual. The final guard now rejects it. Mechanical redistribution of that bias remains future work.

Other prototype limits:

- Changing contact material identity during bias is detected and rejected, without impulse undo/reconciliation. The old physical bank remains available for diagnosis.
- Unequal wall static/kinetic coefficients and sheath tangent-plane history are explicitly marked unverified, so cannot certify.
- Wall friction keys currently identify capsule segment or endpoint, not a fixed material coordinate. A reviewer component proof moved a capsule foot `wallT:0→1` while retaining Ft on the same segment key. This is not a full World false-pass proof, but wall witness/history ownership needs closure before general runtime acceptance.
- CCD sweep reactions participate in the solve but are not included in public `contacts` reaction samples; `sweptWitnesses` counts them. Public summed wall reactions are therefore incomplete when sweeps are present. Nonlinear/new-witness CCD behavior needs further independent validation.
- This World step is not an all-or-nothing outer transaction: final failure does not roll back the entire published pose/velocity timestep. `historyCommits:0` means the accepted-history snapshot was not committed, not that live World/body state was restored. Root must decide failed-step handling before using this as a default runtime path.
- General moving zero-mass kinematic supports, migration and long/scaled runtime behavior have not been certified. Physical translation prediction currently uses zero velocity for inverseMass=0 nodes.

## Patch boundary

The handoff patch is built against a freshly captured root tree, contains exactly eight physics source paths, this new test and this report, and preserves root's `includeSystem.originalGroups` audit addition and sealed three-field surface gradients. It does not touch Direct, TrialState, ContactNormalRows, FoldRows, OrientationRows or any discretization/benchmark/profile file. The JSON manifest records root-before and 8996-after SHA256 per path. The patch is checked against a temporary copy of those exact root files; root itself is untouched.

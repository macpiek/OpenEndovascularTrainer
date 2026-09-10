# TwoChannelRows narrow handoff — 8996

Frozen patch SHA-256: dbb4dccd27e9281aba2f9566c9b37141e5d999edd2a4ad9ec84f67698ae14827
Only three NEW files: helper + dedicated unit/runtime mapping tests + native System integration test. Existing Motion, SplitMotion, World, System, anatomy and replay sources were not edited. No benchmark, anatomy replay, commit or root-worktree mutation was performed.

## API and call order

- beginKirchhoffTwoChannelRows(joint, world), once immediately after beginKirchhoffTwoChannelMotion. Allocates real independent beta banks in s.twoChannel.rows.bank and exposes the same object as s.biasBank. s.bank contains actual physical tool/body arrays and a joints object; no phase swaps or reset of physical lambda, velocity or history.
- prepareKirchhoffTwoChannelRows(joint, additionalRows=[], groups=[]): {ready, issues, version, channels(native)}. Must run before each solve, with all original additional rows and all friction groups. Unsupported loaded history sets ready=false; factory throws before a solve can discard that force. Targets and equations live in snapshot-owned state; they never borrow pooled row identity. A previously applied batch must be committed before preparing another.
- commitKirchhoffTwoChannelRows(joint, result): immediately after applyKirchhoffTwoChannelPhysicalMotion, total geometry/physical commits, and commitKirchhoffTwoChannelBiasMaterial. Requires exactly one new physical+material Motion receipt since prepare and the identical common scale. Applies betaContactIncrement/biasAdditionalIncrement once. It prevalidates every target/bank and records actual scaled sheath bias history. It does not clamp away negative normal forces. Material beta remains owned by Motion.
- measureKirchhoffTwoChannelRows(joint, freshBoundaryRows=[], orientationBatch=null): {finite, supported, normalResidualMm, controlResidualMm, orientationResidualRad, missingLoadedRows, issues}. missingLoadedRows is an ARRAY. The default null orientation batch computes fresh target-frame log differences directly from current qG/qP and current compliant orientation controls, without changing any solve batch. Explicit orientation batch/rows is optional. Physical and complete phases support measurement; prepare/commit require physical phase.

Fresh geometry must already be recollected and raw _splitActualGap/_splitActualStrain staged. Normal residual is absolute gap+alpha*beta when loaded, or penetration-only when beta=0. Positional and orientation controls use vector norms. Bias material uses the existing Motion native C(qG)-C(qP) evaluator. Physical material/control rows read pose, bias material/control read bias-motion; fold reads pose only; all friction is physical-motion only; physical normals read physical-motion and bias normals pose. Finite alpha is retained.

## Ownership / supported lifecycle

Contacts use actual manifold contact owner+stable id/material labels/feature/stencil; wall/point/sweep use actual body+side+node; sheath uses actual sheath owner, not pooled per-side storage; tools use actual tool+lambda index (side may be absent). Controls use actual body+node/component or body orientation vector with controlled segment in its identity. Bank entries retain copied local gradients AND frozenWorldGradients (old frame already applied). Material coordinates, capsule wallT, lumen weights and tool tA/tB prevent transfer of a loaded reaction to a different material foot. Missing loaded rows or changed loaded identity remain explicitly unsupported, with the old beta and old world wrench preserved. No lifecycle force is cleared by geometry refresh.

s.biasBank shape: bodies[{wallLambda,orientationControlLambda}], joints{_coupledBoundaries:{controls,sheaths}}, contacts[{contact,normalLambda}], tools[{tool,lambdas}], pointWalls:[Map(node->{lambda}),Map], sweeps:[Map(node->{lambda}),Map]. s.twoChannel.rows owns maps and pending targets, all restored by existing CoupledTrialState. Existing readonly stats/certificate readers can access this real bank shape.

## Validation

OET_TWO_CHANNEL_SYSTEM_SOURCE_ROOT=/Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer node --test --test-concurrency=1 tests/kirchhoffTwoChannelRows.test.js tests/kirchhoffTwoChannelRowsSystem.test.js tests/kirchhoffTwoChannelMotion.test.js tests/kirchhoffTwoChannelMotionSystem.test.js

28/28 PASS: 15 new Rows tests, 13 existing Motion tests. Full output: tests.txt. Read-only root dependency hashes are in manifest.json. After integration the tests use the default repository-relative System import.

Native analytic wall/material oracle, two actual consecutive solves: qG.x=10, qP.x=8.367552280433378, physical v=195.9009933480502, betaWall=163.2447719566548, physical wall lambda=0. Second solve retains beta and adds approximately zero physical/bias direction. Both native material equations and fresh normal KKT close. Additional tests cover pooled rows/reordering, all channel classes, untyped grouped friction with zero physical load, scaled one-time commit, missing loaded banks, material-foot changes, target-frame orientation logarithms, fresh completed-phase measurements, old world angular wrench retention, atomic stale/invalid commits and exact snapshot restoration including sheath history.

## Remaining root lifecycle work (not implemented by this patch)

Root reported a real World block when a loaded capsule foot changes after a small correction. This patch preserves and extends that explicit guard; it does not claim the World lifecycle is complete.

A future release implementation must retire the old entry to a separate owned bank without losing old beta. Add an explicit full-response frozen old-J row with physical total multiplier fixed at zero and bias total multiplier fixed at zero (therefore bias increment=-oldBeta). Convert stored frozenWorldGradients angular coordinates into the CURRENT qG local frame. New identity gets its own zero-initialized canonical beta slot alongside the retired reaction. Apply physical+bias responses and retired/canonical beta updates with the exact same scale; fractional steps leave a retained old beta. Certification must measure the remaining generalized release response in mm/rad and must not misreport it as a new static contact or silently drop it using a force epsilon. This is a separate bounded task after integration.

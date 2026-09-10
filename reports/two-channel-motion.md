# Two-channel motion handoff

Scope: one NEW production helper, two NEW test files. World, SplitMotion and TwoChannelSystem are not edited by this task.

Canonical state is joint._splitMotion.twoChannel. It owns separate Float64 physicalPose[side] (x/y/z/orientationX/Y/Z/W), native-local materialLambda[side] for beta_M, active-range layout, dt/step and once-only application receipts. The weak result identity cache contains only numeric ids; every mechanically relevant receipt is snapshot-owned and rolls back. No result matrices are retained by receipts.

API:
- beginKirchhoffTwoChannelMotion(joint): call once immediately after beginSplit/prediction and before geometric repair/CCD. A repeated call in the same split step rejects rather than resetting physical history.
- applyKirchhoffTwoChannelPhysicalMotion(joint,result): call BEFORE applying total pose to qG. It publishes only dqP/dt through the existing split physical hook, preserving the physical sheath journal. qP translation receives scaled dqP; local angular dqP is rotated by the CURRENT qG frame and LEFT-applied to qP. It does not apply qG, physical material lambdas or bias normals. Unconverged/duplicate applications reject before mutation.
- commitKirchhoffTwoChannelBiasMaterial(joint,result): beta_M += scale*result.bias[side].lambda, native LOCAL material row indexing, once at the same scale as physical application.
- measureKirchhoffTwoChannelMaterial(joint,out={}): physicalPoseStrain:[C(qP)], geometryStrain:[C(qG)], biasStrain:[C(qG)-C(qP)], alpha:[native alpha], physicalLambda:[physical lambda], biasLambda:[beta_M], physicalResidual:{adaptationMm,bendTwistRad}, biasResidual:{adaptationMm,bendTwistRad}, bodies:[owned row arrays and start/end/count], top-level maxima and finite. Native physical material residual is C(qG)+alpha*lambda; bias residual is C(qG)-C(qP)+alpha*beta_M. Compliance is not hardened.

The native constitutive assembly temporarily receives the selected pose references. Its hard-frame target side effect is neutralized by temporarily setting the target to that pose's own frame while retaining the hard mobility mask; refs and original target are restored in finally. After qP assembly, native qG assembly restores the current geometry scratch. Outputs own their arrays.

13/13 tests PASS: separate predicted pose, cumulative passes, bias-only, physical impulse once, noncommuting qG-local angular transport, common fractional scale, both nonlinear strain equations, hard control read isolation, failed native qP assembly finally, rollback bytes/references/receipts, nonzero activeStart local indexing, physical sheath journal without bias normal, rejection of unconverged direction, and actual frozen TwoChannelSystem solve/application. The native channels factory proves its original qG scratch arrays are byte-equal before and after the qP read.

Native oracle: qP.x=8.367552280923107, qG.x=10, vP=195.9009934068178, beta_M=-161.61232418762322, alpha=1/99. Final physical adaptation residual=5.021028731677646e-12 mm; bias adaptation residual=9.992007221626409e-12 mm. The motion test checks velocity against the actual converged native response divided by dt; analytic force comparisons use displacement-equation units to avoid confusing solver error with multiplier/stiffness scaling.

Limits owned by root: World integration, total pose and physical multiplier application, geometric/physical contact banks and release, folds, control channel equations and the transported quaternion difference for orientation controls. Active-range/topology changes during a step reject. Native redundant adaptation-row identity changes between qP and qG reject instead of subtracting unlike equations. No anatomy replay, adaptive stepping, or performance claim is included.

Worker validation command (root owns the new solver source):
OET_TWO_CHANNEL_SYSTEM_SOURCE_ROOT=/Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer node --test --test-concurrency=1 tests/kirchhoffTwoChannelMotion.test.js tests/kirchhoffTwoChannelMotionSystem.test.js
After root imports all files, the environment override is unnecessary.

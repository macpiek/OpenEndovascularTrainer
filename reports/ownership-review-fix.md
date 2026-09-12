# Ownership review corrections

P1 and P2 are corrected in this worktree. No root edits, solver/Newton edits, tolerance/friction changes, scene runs, or replay.

## Mechanical contract

Normal rows freeze their unit world force and world moment about origin (0,0,0) in the World collector. External friction rows freeze the same data immediately after surface-row assembly. Both occur before the simultaneous apply. Commit accumulates only the actual common-scale multiplier increment. It never converts a local moment using the already changed pose.

The owned per-node ledger stores F_world and M_origin. Release reconstructs F_world and tau_local = R_current^T (M_origin - p_current cross F_world). Each active terminal node has only translation; any transported terminal couple is added to the last active material frame of that rod. At unchanged positions/frames this returns the original generalized action (up to roundoff); after independent changes it preserves each rod's original spatial wrench. This transports mechanical action, not the exact inverse of interleaved finite SO(3) pose updates.

A fixed carrier 1 -> 0 remains in the full joint solve. Its retained wrench, normal-only projection ledger, Fn and both Ft components receive the same release fraction. Partial release is reconstructed again at the next linearization. Rejected trials restore the owned spatial ledger, frozen normal snapshots, multipliers and frames. The reaction commit API now requires captureKirchhoffToolReaction(joint, gradients), not a raw gradient array; malformed legacy input throws before mutation.

P2: a failed branch certificate produces segment=-1, distance=Infinity, violation=0. It cannot provide a sliding aperture or an external-contact exemption. The existing external collector retains the returning-loop row. Successful certification and withdrawn-tip guard are preserved. The reference path avoids the new Set/closure allocation.

## Checks

79 small tests: 78 pass, 1 visible nonlinear integration regression. Log: /tmp/oet-ownership-review-tests-8996.txt.

The independent Three.js point-force proof from validator 0827 was copied to tests/kirchhoffOwnershipProof.test.js with only the load API adapted. Its four original proofs pass; two additional independent/nonrigid endpoint-translation cases also pass (6/6). Maximum net moment in these cases is below 4e-17; scales 1 and 1/4 both checked. No terminal angular DOF is emitted.

The real World lifecycle tests independently freeze each applied load using Three.js before the actual joint apply, then compare the committed spatial ledger. At every release and retry they compare each rod's world force/moment, reconstruct full W J^T deltaLambda, verify the actual common-scale translation and local-right quaternion update, and inspect Fn/Ft/ledger release. One- and two-load cases, disabled/window changes, independent spin plus nonrigid node translation, and forced reject/retry are covered. Certified and uncertified returning-loop fixtures both retain a positive real external reaction in the first joint solve.

## Open integration regression (not hidden)

The original translated-mouth full-convergence assertion remains enabled and fails. Its external action is fully released and accepted at pass 2: Fn=Ft=ledger=0, pending release=0, material adaptation=.0009496218 mm. At pass 3, with no release row, the lumen surface-friction direction fails line-search descent (merit 60.4696944 -> 61.6894301 at scale 1/128; all eight trials fail). Its friction contact set changes 7 -> 9 -> 8, and the accepted friction KKT displacement is .1638338 mm. The ownership fix changes the physically applied moments and thus exposes this continuation failure. No claim of overall scene acceptance is made. Trace: /tmp/oet-translated-release-trace.txt. Root must resolve this before declaring the lifecycle integration green.

## Handoff

Patch /tmp/oet-ownership-review-8996.patch is relative to the previously delivered ownership snapshot in /tmp/oet-ownership-review-baseline-8996. Six changed/new files: World, Ownership, CoupledBoundaryRows, ExternalFrictionRows, ToolContactLifecycle.test, OwnershipProof.test. The earlier ToolContactOwnership.test is unchanged. Do not copy other worktree baseline differences. No edits to the root-owned TrialState, ContactNormalRows, FoldRows, OrientationRows or SurfaceFriction files.

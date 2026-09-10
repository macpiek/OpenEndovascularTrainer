# Wall differential rows handoff

Exactly five files: new differential helper/test, minimal WallContacts/WallEnvelope integration, and the authorized one-line Envelope fixture source rename to `analytic-plane`. No root TimeStep/Mixed or frozen WallGeometry change.

Every row owns persistent global `dofs[6]`, true `gapJacobian[6]`, signed `forceColumn=-B`, and unscaled `normalDerivative=DB[36]`. Physical residual uses `Fn*forceColumn`; the mixed geometric block is `-Fn*normalDerivative`. G and B are independent for sparse-SDF. Endpoint-envelope rows use the frozen point contract and scatter its 3 DOFs into the appropriate 6-DOF block.

Collectors preserve the original raw query and perform no additional query. `rawContact.segmentT`/`querySegmentT` retains the provider fraction even for degenerate endpoint probes; `row.t` is separately the endpoint/capsule scatter fraction.

Refresh accepts optional `lambdas`. `requireCompositeWallDifferentialRows(workspace,lambdas)` is mandatory before using/omitting rows and is already invoked by canonicalization and physical measurement. Unsupported active/loaded rows reject. Only unsupported `Fn===0 && gap>0` rows get `inactiveForSolve:true,deltaNormalForce:0`; their derivatives remain NaN. All original gaps remain in measurement; the helper does not remove original rows.

Endpoint duplicates compare source/g/G/B/DB after exact global scatter, alongside existing owner/radius/normal/witness checks. Interior reduction requires exact source/g/G/B/DB equality to its weighted endpoint combination, including cross-endpoint DB blocks. Unknown derivatives cannot establish dependence. Physical forces are transferred with original weights and never rescaled by gradient norm.

Physical-normal AL now throws clearly for sparse-SDF before touching the chain; analytic-plane AL remains covered. The original physical-force measurement uses -Fn*B, not -Fn*G.

10 new tests, 31/31 combined PASS. Includes frozen anatomy P1, original query count/raw data, G != B and nonsymmetric DB, independent endpoint FD/scatter, all exact canonicalization distinctions, real nonlinear flat-gap/unit-normal counterexample, unknown open/active handling, physical Fn and total force/torque preservation, and persistent buffer reuse. Test/dependency hashes and base hashes are in manifest.json. `git apply --check` passes against root901c. No profiling or FPS claim.

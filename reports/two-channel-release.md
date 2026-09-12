# Two-channel bias release — narrow frozen handoff 8996

Patch: two-channel-release.patch
SHA-256: 97c860d167ec8d119bcee755175c65d8c2b6edf40100b970e6056b53783c563b

Applies to the imported initial Rows helper SHA178c42da21f6653dad57d6e1ea32e00277ec92408aa52e42ccd6f1869596003f. Exactly four files: modified Rows + its test, NEW Release helper + dedicated test. No World, SplitMotion, Motion, System, Condensation, actuation, anatomy, or root-owned source was edited. No commits. Complete before/after files and hashes are included.

## Mechanics

1. Missing contact or changed semantic owner/id/feature/material segment: the WHOLE old canonical beta transfers into an independent full-release bank retaining old WORLD J. The new semantic normal starts at beta0. Existing pending difference reaction also remains independently owned and released; it is never absorbed into a new semantic contact.
2. Continuous J/material-foot changes with the SAME semantic contact and unchanged law/compliance use the approved exact representation:

   beta*Jold + priorDifference = beta*Jnew + newDifference.

   Canonical beta is retained. A unit carrier stores the full WORLD wrench difference. This bookkeeping changes no pose or velocity. No force/J epsilon is used.
3. Each semantic entry reuses at most one attached difference carrier. The anchor of its full owned world reaction remains fixed BETWEEN actual applications, and difference = anchor - beta*Jnew. Thus successive transports telescope; J0->J1->J2->J0 with no intervening application produces exactly zero difference and reuses the same entry. After an actual common-scale update the anchor is rebuilt from canonical plus remaining difference. Fractional remainder is preserved when the carrier is re-expressed for a later transport.
4. Every retained row is appended AFTER all original additional/friction rows, preserving offsets and physical friction loads. Physical total lambda is fixed0. Bias total carrier/beta is fixed0, forcing the full negative remaining force response through the native coupled system. Stored WORLD torque gradients are transported to CURRENT qG local coordinates at each assembly. No force is clipped, no material alpha is hardened, and no body/history state is reset.
5. Commit validates the exact full negative release increment, physical increment0, the original common Motion scale and once-only receipt, then updates canonical and retained banks together. Snapshot-owned historyVersion rejects an old frozen result after an intervening force re-expression. Trial and whole-step rollback retain original bank/entry/anchor references and bytes.
6. Material labels, owner geometry, mobility, control history and missing applied-force history remain explicit unsupported gates. Negative normal bias is still a hard failure. Sheath reaction history journals the ACTUAL world difference magnitude, not its dimensionless unit carrier, and keeps evidence of the earlier applied reaction after retirement.

## Existing API, added result fields

beginKirchhoffTwoChannelRows(joint, world)
prepareKirchhoffTwoChannelRows(joint, additionalRows=[], groups=[])
commitKirchhoffTwoChannelRows(joint, result)
measureKirchhoffTwoChannelRows(joint, freshBoundaryRows=[], orientationBatch=null)

Call order is unchanged. prepare may append owned trailing release rows; on reuse it replaces only its own trailing rows. Fresh measure may re-express force history into canonical/retired banks, preserving the total world wrench, pose and velocity. Ordinary solve targets never rely on pooled row identity.

measure adds releasePositionMm, releaseAngleRad, releaseCorrection:[Float64Array,Float64Array], and pendingReleases ARRAY. The correction is the SUM of exact remaining negative release responses under current native mobility; vector norms are taken after summation, so genuine cancellation is represented. Full releases report mode:'full' and beta. Difference releases report mode:'difference' and carrier; the carrier is not a normal force. No multiplier threshold decides whether a nonzero reaction exists.

World should gate and normalize the two release residuals using its EXISTING mm/rad tolerances. Pending list need not be empty when the exact generalized response is within those tolerances. Root has already wired these fields into gate/merit and retains scalar/list diagnostics without copying the full correction arrays every dt.

## Validation

39/39 module tests PASS (tests.txt): 15 Rows, 1 RowsSystem, 10 Release, 13 existing Motion. Native tests use the independently owned root System via OET_TWO_CHANNEL_SYSTEM_SOURCE_ROOT; no System dependency was copied into the worker checkout. The root dependency hashes observed at freeze are in manifest.json.

Command:
OET_TWO_CHANNEL_SYSTEM_SOURCE_ROOT=/Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer node --test --test-concurrency=1 tests/kirchhoffTwoChannelRows.test.js tests/kirchhoffTwoChannelRowsSystem.test.js tests/kirchhoffTwoChannelRelease.test.js tests/kirchhoffTwoChannelMotion.test.js tests/kirchhoffTwoChannelMotionSystem.test.js

5/5 native World tests PASS on a read-only temporary root-source snapshot with ONLY the final Rows/Release overlaid (world-tests.txt). Tests cover both root-requested wall accuracy levels .001mm and2e-6mm, independent nested bias-only axial sliding, physical normal/friction momentum balance, and whole-step rollback/retry. Root independently updated its wall test to ask World for the desired accuracy; the worker changed neither that test nor World tolerances.

Temporary snapshot: /var/folders/mr/zt5btlpd0snc4yk0z99bgs300000gn/T/oet-two-channel-release-world-k6w01x6n
Its complete JS/test/package hashes are in preview-source-manifest.json and were rechecked after execution. This snapshot predates root's independent tighter-linear retry and later release-row condensation changes; it is intentionally NOT an instruction to replace root-owned sources.

Native release tests prove common fractional scale, fixed full negative response, native force balance, anisotropic rotated angular wrench, new semantic beta0 beside old force, continuous force-preserving transport, exact closed-cycle telescoping, bounded carrier reuse, fractional retransport, rollback identity/bytes, stiffness-independent response units, exact cancellation, and tiny multipliers with large mobility. Global negative and missing-history cases remain failures.

## Bounded runtime probe and remaining separate root work

runtime-probe.json / runtime-probe.txt contain the unchanged root short synthetic-vessel insertion probe, run on the same frozen temporary snapshot. sourceStable=true, no exception, final Rows/Release hashes verified against this patch.

The probe makes seven accepted eligible coupled timesteps and reaches prepared catheter7.3666667mm / step49. Its later failure is the separate root physical-cone/merit issue: cone1.996846656e-6 >1e-9, line-search merit0.02900250327664501 unchanged over8 trials. Every Rows channel is supported; missingLoadedRows=[]; pending releasePosition9.069102e-6mm and releaseAngle0, with only two difference carriers (normal and sheath). The previous first-eligible loaded-foot guard is removed through actual force-preserving mechanics. This probe is not a whole-anatomy, browser-performance, or complete end-to-end insertion claim.

Root is independently addressing the cone/tighter-linear retry and exact retained release-row condensation. Those changes are outside this patch and must not be overwritten by the temporary validation snapshot.

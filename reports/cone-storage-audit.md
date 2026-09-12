# Frozen / committed / fresh cone audit

The specified 7.3667 mm failure is **lumen anisotropic-basis drift, not Float32 normal storage**. The exact failing contact uses a JavaScript Number normal multiplier and Float64 tangent multipliers. Frozen and committed cones are feasible; rebuilding geometry changes the physical ellipse basis and creates the measured 8.013592633915323e-8 violation.

A separate native storage test confirms that Float32 wall/tool normal banks can create a cone violation around 4.94e-8. Widening those banks is a concrete independent storage fix, but cannot fix this preview failure: the entire traced replay contains zero wall and zero external-friction entries.

No root or supplied preview source was edited. The only runtime edits were in an isolated copy: removal of the failed `coneMerit` experiment, and two optional observational callbacks around physical apply/commit. The existing `debugJointTrial` callback supplied fresh measurements. **The complete final state, accepted count, result, error, and diagnostics—including all candidate pose/motion arrays—are identical to `/tmp/oet-two-channel-world-before-cone-merit.json`.** The supplied preview World hash is unchanged. See `cone-storage-source.json`.

## Exact reproduction

Runtime source: `/var/folders/mr/zt5btlpd0snc4yk0z99bgs300000gn/T/oet-two-channel-world-preview-zizk0c2d`, with the older Rows a07 guard, intentionally used only for this numerical diagnosis. Isolated copy: path in `cone-storage-source.json`.

The original short command sequence ran once under numeric instrumentation: 33 guidewire advances, then catheter advance until rejection. Result: wire 12.100000000000001 mm, catheter 7.36666666666667 mm, 16 accepted catheter steps, 49 executed steps. Fingerprints match `9b0f6550` / `a9fc34a4`. The 43 recorded nonlinear trials contain 208 lumen entries, no external entries, and no wall entries. Every lumen normal before commit equals its committed and fresh value exactly.

Critical retained trial: world step49, pass4, trial0, scale1, contact `lumen:sliding-rim|runtime:199:7`, feature `sliding-rim`.

| Quantity | Frozen | Committed | Fresh |
|---|---:|---:|---:|
| Fn | 0.01945943318110135 | identical | identical |
| Ft U in current surface basis | 0.0002918556405175611 | 0.00029185564051756107 | 0.0002918556347299928 |
| Ft V in current surface basis | 0.0000018298618880681098 | 0.0000018298618880681233 | 0.0000018307843483009498 |
| Cone violation | 0 | 0 | 8.013592633915323e-8 |

The normal increment is zero; μU=.015 and μV=.006. A hypothetical Float32 write rounds this particular Fn **up** to 0.01945943385362625, so its isolated normal-storage cone remains feasible.

## Native minimal replay and physical interpretation

`kirchhoffLumenConeBasisDrift.test.js` uses the native `KirchhoffContactManifold.remapContact`, native `buildKirchhoffSurfaceFriction`, and native KKT evaluator, with a minimal two-node Float64 geometry fixture. It reproduces the 8.01359e-8 drift with unchanged Fn and without a World solve or any Float32 value. Both tests PASS.

An independent calculation reconstructs the committed world tangent force from its old surface basis, then projects that force into the fresh physical surface basis. It produces a cone violation of 8.013592589506402e-8, differing from the full native replay by only 4.44e-16. Thus this is not unexplained rounding inside the nonlinear solver.

Measured geometry changes in the critical trial:

- Normal-vector change norm: 0.0009789830488693744.
- Contact-point shift: 0.0004541422478146994 mm.
- Surface U/V changes: 0.000010764940278176601 / 0.0009789340529641369.
- Inner endpoint weights change from [0.9467554485216363, 0.053244551478363704] to [0.9467553693308197, 0.0532446306691803].
- Projected world tangent-force multiplier changes by 1.2117621019612403e-9; this equals the discarded component along the fresh normal within roundoff.
- The norm of the changed combined Fn*n + Ft resultant is 1.905045585659063e-5 in multiplier units. This resultant comparison precedes stencil/moment redistribution and is not a full generalized-wrench error estimate.

Euclidean tangent-force length decreases under the new-plane projection, but the **anisotropic** ellipse norm increases. With the explicit diagnostic counterfactual μU=μV=.015, both states remain inside the disk. No runtime coefficient was changed.

The manifold stores a force in a tangent basis and reprojects it as the contact normal changes. The surface builder then measures that world force in the current physical anisotropic U/V axes. This cannot preserve an anisotropic ellipse automatically. Merely storing the same two component numbers in the new axes would rotate the world force; clamping them would alter the force. Either action requires its corresponding physical generalized impulse/moment update and history accounting. Increasing precision alone does not supply that update.

The later backtracking trace explains the final stall without guessing: pass5 full scale has fresh cone 1.0384754900449877e-7. Half scale reduces it to 4.0067964057755034e-8, but the other merit is exactly unchanged at 0.021006992884139354; successive smaller scales approach the prior 8.01359e-8 violation. None is accepted. This report does not propose another global merit scale or claim that a storage change resolves this geometric/history issue. Current root's stronger loaded-foot guard is a separate, earlier gate.

## Independent normal-storage defect and precise fix

Current root allocations are `body.wallLambda = new Float32Array(segmentCount)` and `addToolContact(...).lambdas = new Float32Array(pairCount)` in `endovascularPhysicsWorld.js` (approximately lines483 and1299). The legacy containment `lambdas` allocation is separate; the audited native lumen uses manifold Number multipliers. Native external contacts store tangent multipliers in Float64Array and read Fn directly from the tool array. Native wall friction similarly reads `body.wallLambda`.

A minimal native allocation plus `applyKirchhoffCoupledBoundaryMultipliers` replay proves:

- Solved positive multiplier: 0.016447099738834046.
- Value written to either current Float32 normal bank: 0.016447098925709724.
- At a fixed valid boundary Ft=μ*Fn, the native fresh cone becomes approximately 4.9438769e-8 (>1e-9), solely because Fn shrank.
- The tool's applied normal-wrench journal retains the original Float64 impulse, so it also differs from its Float32 normal bank.
- Replacing only the **owned test object's** normal bank with Float64Array preserves the solved value exactly, its native cone, and its applied-wrench journal. No λ clipping, Fn ceiling, velocity adjustment, solver-tolerance change, or second impulse is involved.

`kirchhoffNormalMultiplierPrecision.test.js` contains two intended native regression assertions, currently **FAIL**, plus two owned Float64 counterfactual controls, **PASS**. These failures are deliberate evidence of the current production allocation defect. Root owns the source fix.

The concrete storage correction is to keep physical wall/tool normal banks in Float64 from allocation through their physical/bias banks, snapshot/rollback, commit, and release paths. Invariant: the final Fn used to define the cone is the same Float64 `oldFn + scale*deltaFn` whose delta was applied to physical motion and recorded in the wrench journal. Changing storage precision must not recompute or apply an impulse. Existing TwoChannel bias tool banks are already Float64; physical banks retain the original tool arrays. No widening of coordinates or global merit change is implied by this correction.

## Handoff and checks

- `tests/kirchhoffLumenConeBasisDrift.test.js` + `tests/fixtures/kirchhoff-cone-transport-storage.json`: 2/2 PASS native minimal diagnosis.
- `tests/kirchhoffNormalMultiplierPrecision.test.js`: 2 expected native FAIL / 2 Float64 counterfactual PASS; ready to turn green after root's allocation change.
- `reports/cone-storage-basis-tests.txt`, `cone-storage-precision-tests.txt`: exact logs.
- `reports/cone-storage-worst-fixture.json`: owned frozen/committed/fresh critical record.
- `reports/cone-storage-replay.json`: complete 43-trial numeric journal and unchanged final diagnostics.
- `reports/cone-storage-conclusions.json`: compact stage table, batch counts, and baseline equality.
- `reports/probe-cone-storage.mjs`: probe used; source provenance and hashes in `cone-storage-source.json`.

Both dedicated test files support `OET_CONE_STORAGE_SOURCE_ROOT` for isolated runtime selection. No additional runtime replay is necessary to establish the classification; further source changes should be validated by their owner against the unchanged full physical/force-history invariants.

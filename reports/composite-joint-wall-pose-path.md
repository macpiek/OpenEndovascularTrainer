# Prepared pose path in stationary WallSurface

Completed only the delegated WallSurface source, its tests, and new reports in worktree 0827. Root 901c, normal rows, friction manager, Step, World, PhysicalColumnPullback and reservoir/history modules were not edited. No commits or resets.

## API

```js
const ws = createCompositeJointWallSurfaceWorkspace({ surfacePosePath });
evaluateCompositeJointWallSurface({ current, tool, wall, dt, configuration, order: 'full' }, ws);
```

`surfacePosePath` must be an owned prepared path, not a structural clone. Its `configurationColumns` fixes the N-vector order. `tool.id`, actual `edge`, `edgeId`, and type-preserving `materialSegmentId` must identify its target. `dt` must equal the prepared dt. Current coordinates, affine labels/rates, own endpoint positions and own angle are verified against the prepared maps and supplied configuration. Optional reference/previousAngle/materialPath are checked if present. `previousPositions` is never read in this branch; the immutable accepted pose and explicit external pose are authoritative.

The caller supplies the current original wall row with its matching field, differential and endpoints, as before. The same fresh current wall witness/frame is used at both times. `tool.trace` selects the physical endpoint trace; optional `tool.materialPath.previousTrace` is forwarded. The existing provider rejects unsupported material transport (including entering labels without an explicit reservoir) and WallSurface propagates `requiredTransport` while revoking all result validity and clearing numeric buffers.

Outputs: `slipJacobian` is row-major 2×N; `forceMap` is N×2; `DforceMap` is (N×2)×N; `currentQueryJacobian` is 10×N with coordinate, point xyz, normal xyz, tangent xyz rows. `configurationDofs=N`, frozen `configurationColumns` and `currentTools` feed PhysicalColumnPullback directly. `physicalDofCount=7` describes the current contact edge, not the expanded matrix width. `motion` exposes the reservoir provider result (including its 19 query columns); `physicalForce` is its local seven-coordinate force result. Width must come from `configurationDofs` in the path branch.

Both current and previous query derivatives are summed for the same-current-witness policy before contraction with the current 10×N query Jacobian. The coordinate query is counted once; the other nine coordinates are counted at both times. Additional actual node/spin columns retain G even when their B rows vanish. B and DB use the expanded force packing, not a seven-column truncation. The current geometry derivative is scattered by physical identity, supporting reordered columns.

Without a path, the old same-edge API, workspace shape, values, buffers and output metadata remain unchanged. `full`/`value` produce bit-identical values/B; value invalidates all derivative buffers/flags. Path-owned buffers and immutable column metadata cannot be replaced. Failure clears outputs and allows exact retry.

## Validation

125/125 tests PASS: WallSurface 22 (11 original + 11 new), ReservoirSurface 17, PhysicalColumnPullback 12, WallFrictionRows 18, WallRows 23, SurfaceMotion 33. Tests used an isolated copy of current root dependencies; all 36 captured source/test hashes still matched root after testing. No World/Step integration claim is made here.

The real proximal nodal endpoint with dsDt=-0.3 produces finite increment [0.03, 0] at dt=0.1 for the unit-label/unit-length case. An own full 2π spin is preserved. Tests retain actual one-sided endpoint fraction despite a different raw degenerate capsule fraction, use explicit external pose history, and reject a same-edge history lacking that pose.

Every N=11 column was checked against central finite differences of values/B and current geometry after original wall re-detection:

| Source | max G error | max DB error | max query error | extra wall queries |
| --- | ---: | ---: | ---: | ---: |
| Plane | 1.281e-10 | 1.105e-10 | 6.665e-11 | 0 |
| Smooth SDF | 3.489e-10 | 2.537e-10 | 1.770e-10 | 0 |
| Smooth BVH | 1.586e-10 | 2.830e-10 | 1.187e-10 | 0 |

The previous-query contribution is nonzero (maximum magnitude 0.967–2.142), so these fixtures detect dropping it. Tests also cover the transported own-reference basis, reordered N=7/11 columns, zero-B/nonzero-G extra physical support, direct mapper composition and virtual work, source/preparation immutability, poisoned/absent previousPositions, and 46 full/value invalid-input/retry cases. The simultaneous proximal feed/bending/spin virtual-power error decreases from 9.591e-6 to 9.595e-8 as dt decreases from 1e-3 to 1e-5.

A separate baseline A/B probe compared complete workspaces and results using deep strict equality in 72 cases (plane/SDF/BVH × two bases × capsule/proximal envelope × three own spins × full/value). Every case was exactly equal to baseline source `dae80edb3b75cf39e0d696317ecdab6762cf7ebb0e48482895617a8728952be8`. The original anatomy SDF test also remains PASS.

## Files and reproduction

Candidate source SHA256: `ff026caa2b6da4e7008793f67707adb3651e3407883f204c8e7e8f68d7b9e366`

Candidate test SHA256: `4d75ae1f8c4b1d00b8975794d6f33f693a2688d90ba1207f40c0f44a5605692d`

The source JSON records every captured dependency hash, baseline directory and isolated runtime. Reproduction used `/opt/homebrew/bin/node --test` with the six test files listed in that JSON. The A/B probe accepts the isolated runtime path and original baseline WallSurface source path as arguments. The frozen payload contains only the owned source, test and new reports; dependencies were not modified or bundled as replacement files.

Inherited support remains compatible proximal reservoir entry and supported smooth original wall branches; the provider explicitly rejects unsupported internal-hinge transport. This change adds no wall queries and makes no new tangent-basis, pressure-discretization or contact-selection policy.

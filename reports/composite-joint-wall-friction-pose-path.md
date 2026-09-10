# Wall friction manager consuming prepared own pose paths

Implemented only `kirchhoffCompositeJointWallFrictionRows.js`, its tests and new reports in worktree 0827. No edits to WallSurface, reservoir/history, PhysicalColumnPullback, normal rows, Step, World, or root 901c. No commits/resets. This is manager-level integration; it makes no whole-Step, World, UI or FPS claim.

## Explicit API and ownership

```js
import { getCompositeJointWallFrictionEdgeId } from './kirchhoffCompositeJointWallFrictionRows.js';
// Use this exact ID for accepted edges in the owned pose history:
const edgeId = getCompositeJointWallFrictionEdgeId(materialSegmentId);
wall.surfacePosePaths = [{ owner: toolId, edge: actualEdge, path: preparedSurfacePosePath }];
```

The helper preserves the existing no-path convention: string `17` becomes `string:2:17`; number 17 becomes `number:17`; bigint 17n becomes `bigint:17`. Original materialSegmentId retains its primitive type separately. The underlying pose-history API currently accepts string/number material identities; legacy bigint handling is unchanged. Accepted path edge IDs must be unique as required by that API.

`surfacePosePaths` is an optional explicit list, not a callback. Each entry must address a declared actual wall owner/edge with a current normal surface record. Duplicate/unowned/wrong-target entries and structural clones of a path reject. The target ID and materialSegmentId must match the wall declaration exactly, and path dt must equal the manager/prepared dt. Every accepted edge uses the helper's ID convention. Where that edge has a wall declaration for the same tool, its material ID must match that declaration. Extra accepted support has its explicit semantic ID in the prepared contract and subsequent friction history.

Every accepted endpoint, reference frame and unwrapped angle is compared exactly to the incoming own state, including adjacent physical support that has zero current force. Accepted-to-accepted hinge lifts must equal `state.tools[].referenceTwists[left.edge]`; reservoir hinge lifts remain part of the explicit external contract. Accepted/current label maps and endpoint rates are checked against prepared inertia (with only arithmetic-roundoff allowance), and existing accepted friction map continuity remains authoritative. The manager never invents/extrapolates an external pose or substitutes a same-named path for the incoming accepted body pose.

Declared edges use WallSurface prepared with that path and PhysicalColumnPullback. Configuration is packed from current toolPositions/candidate.angles by every declared physical column identity. All additional G dependencies remain present even if B is zero. Physical loads scatter every `tools[].nodes/nodalForces` and `tools[].edges/spinTorques` entry. Undeclared records retain the old seven-coordinate path and scalar torque handling, including mixed managers.

## History and lifetime

The friction signature retains its original shape without paths (also for an empty path list). With paths it adds semantically encoded reservoir identity, target/edge/node/column support, source roles, coordinates, hinge topology and explicit linear binding support/weights. It excludes path object identity, dt, current/old numeric poses/angles/maps and binding offsets, which can evolve under the same explicit physical policy. Changed support/binding policy/reservoir identity rejects without Fn/Ft transfer or disabling friction. Column reordering between steps is conservatively treated as changed support packing; fixed reordered packing is fully supported.

Within one prepared step the list and its exact owned paths remain fixed. Replacing a path, including with a new path with the same names, revokes current evaluation/commit. Each manager owns only its current declared path arenas. Reusable scratch stores its original single legacy surface arena and equation workspace and never stores a path/history cache. Keeping an old manager handle naturally keeps that caller-owned manager alive, but its generation lease becomes stale after workspace reuse. Committed history stores no path objects or pose-history arrays.

Normal query provenance, original pressure ownership, source/branch guards, signed Fn/Ft trial behavior, exact dual offsets and private certification gates remain intact. Preparation/evaluation/commit perform zero additional wall queries. Late failures clear every row/load and leave caller residual arrays and trial Fn/Ft intact; retry restores exact results. Commit freshness includes all accepted support node positions and angles, including zero-B support.

## Validation

135/135 PASS: manager 28 (18 original + 10 new), WallSurface 22, ReservoirSurface 16, PhysicalColumnPullback 12, WallRows 24, SurfaceMotion 33. The isolated runtime used all 36 current root source/test hashes; no dependency drift was found after testing.

Positive proximal catheter feed dsDt=-0.3 at dt=0.02 produces [0.006,0] slip at both original nodal sites for stationary unit-metric geometry, with actual resisting Coulomb tractions and negative work. The same request without a prepared path still rejects entering labels with `surface-material-transport-required`; an owned same-edge path lacking external pose rejects too.

Full finite differences perturb all 11 expanded common columns under simultaneous feed, bending and own spins, redetecting original wall geometry:

| Source | max equation-J error | max mechanical-H error |
| --- | ---: | ---: |
| Plane | 7.600e-11 | 2.535e-10 |
| Smooth SDF | 7.158e-11 | 6.299e-10 |
| Smooth BVH | 7.013e-11 | 4.042e-10 |

Tests explicitly observe nonzero equation G influence at an extra physical node with zero force column. N=7 prepared entry also passes. A relative wire owner checks all 20 common/relative equation columns (11 common + 9 relative) with reordered physical packing. Mixed catheter-path/wire-no-path rows retain the correct global Fn/other-Ft cross offsets and each tool's own torque scatter.

Two consecutive manager steps use dt=0.02 then 0.013, updated own positions/angles/maps, a freshly prepared path and penalty 5 then 50. Their signatures and incoming Ft history remain compatible without transfer; a stale same-name pose is rejected. Twelve successive reused-workspace steps retain one legacy arena, fixed support/history size and stale old leases. Full/value values/loads are bit-identical; late second-site source failure, extra node/spin changes and path replacement revoke acceptance and permit exact retry after restoration.

A separate deep-strict A/B probe checks 36 no-path cases: 24 capsule combinations (one/two tools, one/two records, rates 0/0.2, penalties 5/50/500) and 12 nodal combinations (one/two tools, both shared-node traces, three penalties). Complete equations, matrices, certificates, residuals, physical loads, signatures, accepted history and diagnostics match baseline `cdcbfa956202f8779b47bda0267c20eb50d2c9682afc893032a197cc99cfa59b` exactly through full/value/full evaluations. Half the cases explicitly pass an empty path list.

## Source and reproduction

Source SHA256: `61b7143fb917eb04101b6167960d24e4e451454976aa5a1db840e17adade2812`

Test SHA256: `71c778e6ed279be51e4188bd36a9e60ac3149ab7954494946967d146779fc1ba`

The source JSON records the isolated runtime, baseline and dependency hashes. Run `/opt/homebrew/bin/node --test` on its six listed tests. The A/B probe takes ISOLATED_RUNTIME and BASELINE_MANAGER arguments. The frozen bundle contains only this owned manager, test and new reports/patch; no dependency replacements.

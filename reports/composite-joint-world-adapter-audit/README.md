# Read-only audit: Joint World adapter and whole-step hook

One concrete coverage defect was found, reported, fixed by root, and independently retested. No production source/test was edited by this audit worker. The existing six adapter tests pass in the immutable baseline snapshot; two additional narrow physical-history/publication controls also pass. No other critical issue was reproduced within the explicitly transitional fixed-chart scope.

## Closed finding: original World anatomy could receive an empty certificate

The former `guardWorldConstraints` checked only identity of `options.wall.field` and `world.contactField`. A valid `wall-normal` problem with the SAME original field but `contactOwners.edges=[{edge:0,wall:null},{edge:1,wall:null}]` passed that guard. The original Joint dt then had no wall rows or queries and reported a vacuously converged wall certificate. Full World advancement returned `accepted:true`, incremented both step counters, and published the bodies while the wire centerline was 0.341129874 mm outside the explicit y<=0 wall, even before including its positive radius.

This was a World source-coverage error, not a defect in the declared low-level empty inequality set. Merely matching a collision field or accepting a caller-provided owner list does not prove that the original World exposed surfaces/radii/friction were represented. A `wall.mode:'none'` record did NOT bypass the low-level Step and was not used for the finding.

Root fixed the transitional adapter to reject EVERY non-null World collision field with `joint-world-wall-adapter-required` until a complete source mapping exists, consistently with its current sheath/containment/tool-contact guards. An isolated fixed-adapter snapshot with identical prior dependencies now returns `accepted:false`, 0 executed steps, both counters 0, and retains the pending 1/120 s. Body arrays/views are untouched. This resolves the observed false acceptance without declaring the original World anatomy integrated.

- Baseline adapter SHA: `fa368fbb56c4bd8db19f16718b8c71331f539bb694750476d1177d1d3d35d81e`.
- Fixed adapter SHA: `6126a8b855e1df39bcb02fed0525ca1027778835adac4d1cc79ac8fce23c7dde`.
- Baseline witness: `node /tmp/oet-joint-world-adapter-audit/probe.mjs`.
- Root-fix witness: `node /tmp/oet-joint-world-adapter-audit/fixed-probe.mjs`.
- Full raw normal certificate and before/after counters: `evidence.json` / `fixed-evidence.json`.

## Ownership and history controls

The existing **6/6 tests PASS**, 289.534 ms, on the frozen baseline. Read-only inspection confirms that the adapter keeps Float64 Joint state, independent reference frames/angles and accepted material velocities, prepares numeric commands once, and does not reconstruct subsequent physics from the coarse Float32 body views. The whole-step hook increments and consumes dt only after valid success; rejection retains the system identity and prepared input. The adapter owns body/physical-state rollback; World does not mix in old predictor/damping passes.

An independent loaded-publication control accepts dt1 with Fn=0.375638637448 and nonzero Ft, then injects the existing second-body publication fault at dt2. All body numeric bytes are restored, each prior `jointStateView` object is restored by identity, internal accepted state and both clocks remain at dt1, and the prepared dt remains queued. A direct retry produces exactly the same state as direct production JointTimeStep from the old accepted state. This extends the original first-publication fault test to existing nonzero physical histories and previous views.

A separate two-step control initializes wire +4π and catheter −6π, then applies their separate small prescribed rotations. Their complete own unwrapped angles remain in internal state and published full views after both dt: wire `[12.566870614359173, 12.568870619092014]`, catheter `[-18.849722588205424, -18.849722588277505]`. Quaternion display conversion does not become the next physical spin history.

The complete union geometry in `jointStateView` and coarse-body-view scope remain explicit. This audit does not certify remapping, unsupported source adapters, material angular inertia, app activation or 60 FPS. The initialize/prepareStep callbacks remain subject to their documented synchronous, read-only/immutable-provider contract.

## Frozen handoff

`/tmp/oet-joint-world-adapter-audit/manifest.json` freezes the original 83 source/test dependencies, fixture export wrapper, original six-test output, baseline/fixed adapter versions and independent probes/evidence. Runtime Node/npm packages use the worktree node_modules link. The old failing snapshot is preserved rather than overwritten by the root fix. Parent can independently rerun the original witness and its fixed counterpart.

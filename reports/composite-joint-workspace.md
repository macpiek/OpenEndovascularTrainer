# Reusable symbolic joint workspace

This patch removes repeated symbolic preparation in `JointAssembly` and `JointTimeStep` while retaining the existing no-contact full-motion equations, tolerances and accepted-only physical state. No other production module changed. The captured pre-patch baseline already includes the root's gradient-only length tangent optimization; its gain is not attributed to workspace reuse.

## API and lifetime

```js
const workspace = createCompositeJointTimeStepWorkspace({
  layout, coordinates, modes, relativeToolId: 'wire',
  elementBackend: 'wasm-exact', rowCacheCapacity: 2,
});
const result = advanceCompositeJointTimeStep(state, {
  ...preparedInput, workspace,
});
```

The handle freezes chart coordinates, canonical topology, ordered mode nodes/complete bases and backend. It lazily owns one symbolic Cluster/CSR, scatter plans, element/operator scratch, ToolLengths and an LRU of 1–4 Direction row patterns. Row identity includes material/index, anchor, ordered common/relative supports and units. Target/load/multiplier values and the current fixed mask are **not** cached as physical data. Changing which edge length rows are algebraically redundant creates the corresponding pattern; eviction bounds retained matrix/factor storage. Reusing a pattern reuses storage only: every actual direction reconstructs and factors the current original matrix.

The low-level equivalent is `createCompositeJointAssemblyWorkspace({layout,coordinates,modes,relativeToolId,elementBackend,physicalInertia:true})`, passed as `createCompositeJointAssembly(args,{workspace})`. Explicit `physicalInertia:false` supports an elastic-only symbolic scope. Every prepare owns fresh reference frames, winding, sampled material tensors/intrinsic strain/energy offset, inertia maps, density, old geometry and current-label old velocities. No constancy is inferred from samples or a provider identity. The per-dt physical inertia coefficients are compiled again; this patch does not modify MaterialInertia.

A later prepare, including a failed prepare, invalidates earlier borrowed assembly handles and their operator/Hessian validity flags. Calling an earlier handle then throws. `invalidateCompositeJointAssemblyWorkspace(handle)` explicitly ends such a lease. Nested preparation is rejected as busy. The timestep wrapper releases busy and invalidates its lease on acceptance, rejection and preparation error; a valid retry starts from its supplied state/history.

Physical multipliers are restored from **STATE**, including nonzero length and wire support forces. Only the existing explicit gauge for a length edge with two prescribed physical endpoints sets its redundant multiplier to zero. No physical history, current reaction or accepted geometry is used as a workspace seed. Accepted states and returned reactions/momentum records own their arrays independently of reusable scratch. `diagnostics.workspaceReused` selects the reusable execution path; cumulative `workspace.diagnostics` build/hit counters distinguish its first construction from later reuse.

Within the prepared step, the same physical force-decoding arrays and line-search backups are reused, with current values written on every evaluation/direction. This also reduces per-iteration allocations on the cold path. Omitting the workspace retains the same cold construction path as an explicit oracle; the benchmark separates this shared scratch improvement from symbolic reuse.

## Correctness

**69/69 affected tests PASS**, including six new controls in the owned Assembly/TimeStep tests, with the original independent Element finite differences, full matrix pullbacks, physical force balances and strict length/torsion gates unchanged.

New controls require exact cold/reuse equality of complete E/g/H, kinetic/momentum data, state, original certificates, reaction forces and iteration/factor counts after changing physical frames, winding, material stiffness/intrinsic/offset, density, dt, maps, old velocities, loads and position/spin targets. Provider invocation is repeated for every current material hinge. Exact common-geometry sharing changes on/off with the **current** independent reference frames, never a cached previous decision.

Additional cases exercise nonzero STATE multiplier histories, bounded LRU eviction across different redundant-length patterns, accepted result ownership, stale handles, changed chart/basis, nested calls, malformed inertia, first/deformed-trial rejection and late evaluation-budget rejection. Recovery and deterministic retry match the cold reference. Current ToolLengths/RelativeDirection gradient-tangent guards remain active; no stale stress tangent enters a solve.

## Controlled paired measurement

Protocol: the same 33-node synthetic full-overlap fixture and load as the root's earlier warm probe; dt=1/120 s, no contacts, distinct wire/catheter geometry and dsDx, unchanged original tolerances. Fifteen interleaved warmup **pairs** per arm precede sixteen measured pairs per arm. A pair contains a first loaded dt and a continued physical dt using the first dt's accepted material velocities at unchanged labels. Arm order rotates each round. The baseline is immutable `/tmp/oet-composite-joint-workspace-before`, captured after gradientLength was integrated. The final bundle contains that baseline plus the exact self-contained probe.

All **96 measured dt accepted**. Every state, original certificate and per-tool momentum/support balance was bit-identical across the three arms. Input hash: `2bf1290a25c0ab8a6cbe5e77ff4a37f0d97515e26fd469b2fa7dbcfc24637f20`. Per-state hashes and all slow samples remain in `composite-joint-workspace-probe.json`. First dt used 2 directions/6 evaluations; continued dt used 1 direction/4 evaluations, identically in all arms. Every direction still used fresh current tangent/matrix and LU.

| Arm / physical step | Prepare median ms | Iteration median ms | Whole median ms | Whole P95 ms |
| --- | ---: | ---: | ---: | ---: |
| Frozen before / first | 6.179 | 10.593 | 17.249 | 34.226 |
| Frozen before / continued | 6.684 | 6.851 | 14.778 | 23.636 |
| Current cold / first | 6.510 | 8.475 | 15.242 | 24.758 |
| Current cold / continued | 4.975 | 6.095 | 13.294 | 24.892 |
| Reuse / first | 2.033 | 8.583 | 11.289 | 21.058 |
| Reuse / continued | 2.156 | 4.524 | 8.176 | 18.860 |

Preparation is approximately 67–68% lower than the frozen before version. Whole median dt is about 35%/45% lower than before, and 26%/38% lower than the current cold control. Cumulative counters for 62 reusable calls show one Cluster build, one Length build, one Direction build and 61 Direction hits; the 126 local scatter plans were compiled once. No discarded/failed step was counted as executed.

This short Node measurement has visible host/JIT/GC noise and is not an anatomy/profile, browser or FPS certificate. Both the median and tail remain above the intended ≤4 ms average / ≤6 ms P95 full-step goals. Remaining measured work includes current inertia/material/frame preparation, physical operator/scatter evaluation and original band solve/certificate work. Contact integration and later controlled reduction remain separate tasks.

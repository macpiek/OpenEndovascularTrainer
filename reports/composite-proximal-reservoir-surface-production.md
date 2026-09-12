# Production own pose history and compatible proximal reservoir entry

Implemented as two NEW production modules, a NEW test and this report in 8996. Existing SurfaceMotion, WallSurface, LumenSurface, Pullback and root files are unchanged. The accepted domain is same-edge material transport and compatible proximal reservoir entry. Interior hinge crossings remain explicitly unsupported; this does not complete the wider hinge/contact reconstruction objective.

## Exports and preparation

`kirchhoffCompositeJointSurfacePoseHistory.js` exports:

```js
createCompositeJointSurfacePoseHistory({
  toolId, reservoirIdentity, nodes, edges, hinges
});
prepareCompositeJointSurfacePosePath({
  history, targetEdgeId, dt, currentMaps,
  configurationColumns, nodeBindings, angleBindings, reservoirIdentity
});
```

The module also has a frozen, provenance-checked `readCompositeJointSurfacePosePath` reader used by the evaluator module. Consumer code should use the creation/preparation API.

History records are:

```js
nodes: [{ id, position: [x,y,z], node /* actual physical node index */ }]
edges: [{
  edgeId, materialSegmentId, source: 'accepted' | 'reservoir',
  edge, // actual layout edge for accepted records; forbidden for reservoir
  nodeIds: [leftId,rightId], coordinates: [x0,x1], labels: [s0,s1],
  reference: { tangent, director }, angle // accepted own unwrapped value
}]
hinges: [{ leftEdgeId, rightEdgeId, referenceTwist }]
```

An external-only reservoir node omits `node`; a shared proximal endpoint uses the SAME node ID and physical node index as the original body. Accepted edges require endpoint indices `edge,edge+1`. A reservoir must be the first span and have a stable string/number `reservoirIdentity`, matched during preparation. Node IDs and materialSegmentIds preserve primitive type; e.g. `42` and `"42"` remain distinct.

Histories own deep-frozen copies of positions, frames, angles, labels and directed reference lifts. Adjacent records must share one node identity and continuous coordinates/material labels. Duplicate nodes/edges, stale frames/lifts, invalid spans and unreferenced external nodes reject. An accepted frame must match its own old endpoints. No velocity-only history is interpreted as pose history and no coordinate is extrapolated.

`currentMaps` supplies `{edgeId,labels:[s0,s1],dsDt?}` for every declared pose edge. The old/current maps explicitly define linear time interpolation. If a scalar or endpoint-pair `dsDt` is also supplied, it must agree with that path to arithmetic roundoff. Snapshot scope is explicit: preparation does not silently select, remove or interpolate source edges. A caller may prepare its local accepted/reservoir stencil; a parent mapping can enforce its supported physical bandwidth.

## Physical columns and external dependency contract

Columns are ordered unique records:

```js
{ kind: 'position', toolId, node, component: 0 | 1 | 2 }
{ kind: 'angle', toolId, edge }
```

They identify actual nodes/angles of this tool, not free external reservoir coordinates. Every declared accepted physical node must have all three physical columns and every accepted edge its own angle column. The current actual endpoint/spin bindings MUST be identity mappings with zero offset, including degrees of freedom later fixed by Dirichlet conditions. Surface preparation never removes body force or reaction columns.

Bindings are explicit affine contracts:

```js
nodeBindings: [{
  nodeId, offset: [x,y,z],
  terms: [{ column: configurationColumnIndex, weights: [dx,dy,dz] }]
}]
angleBindings: [{
  edgeId, offset: unwrappedAngle,
  terms: [{ column: configurationColumnIndex, weight }]
}]
```

For an external pose only, empty terms mean a prescribed current value. Nonempty terms mean `currentValue=offset+J*configuration` with the stated CONSTANT Jacobian; no callback derivative is guessed. This supports an external endpoint attached to declared actual rod columns. There is one binding per unique node and one per edge angle. The shared proximal node therefore cannot be bound twice. External-only nodes cannot be mislabeled with an actual free-body node index. Prescribed external parameters are contracted out; all body columns remain available for reactions.

## Workspace, evaluation and output packing

`kirchhoffCompositeJointReservoirSurface.js` exports:

```js
const out = createCompositeJointReservoirSurfaceWorkspace(path);
evaluateCompositeJointReservoirSurface({
  configuration, // values in path.configurationColumns order
  query: { coordinate, trace, previousTrace },
  finiteGeometry: {
    kind: 'explicit-affine-side-queries',
    current: {point,normal,tangent}, previous: {point,normal,tangent}
  },
  order: 'full' | 'value' // full is default
}, out);
```

`trace` and `previousTrace` select right/left material traces at current and accepted endpoints. A stationary material label at a junction needs matching explicit traces. Endpoints of an isolated same-edge history obey the same trace requirements. A stationary wall's same current point/frame at both time endpoints, and the associated query Jacobian sum, remain the caller's explicit geometry policy. This lower-level own-body provider does not query or replace wall/lumen geometry.

Let N be the number of declared physical columns and Q=19. Query columns are the current own coordinate, current point/normal/tangent xyz, then previous point/normal/tangent xyz. Raw query vectors are normalized/orthogonalized inside AD.

| Output | Shape and meaning |
|---|---|
| `increment`, `relativeIncrement` | 2 tangential components; 3 contact-coordinate components |
| `configurationJacobian`, aliases `G`/`slipJacobian` | 2*N, `[component,configurationColumn]` |
| `queryJacobian` | 2*19, `[component,queryColumn]` |
| `forceMap` | N*2, `[configurationColumn,component]`; body force is **F=B*Ft** |
| `DforceMap` | N*2*N, `[configurationColumn,component,derivativeColumn]` |
| `forceMapQueryDerivative` | N*2*19, same entry order plus query column |
| `physicalForceMap` | Original current-edge B, 7*2 |
| `physicalDforceMap` | Original current-edge DB, 7*2*7 |
| `physicalForceMapQueryDerivative` | Original current-edge query DB, 7*2*10 |
| `physicalForce` | Owned original ForceMap result with its own dimensions and flags |
| `configurationColumns`, `queryColumns` | Immutable contracted output column identities |
| `physicalConfigurationColumns`, `physicalQueryColumns` | Current physical 7/10 column identities |
| `currentTools`, alias `tools` | `[{id,edge,edgeId,materialSegmentId,nodeIds,nodes:[edge,edge+1]}]` |
| `events`, `segments`, `jumps` | Owned path diagnostics, identities, label, time, direction and event derivatives |

Current physical B/DB is computed by the unchanged ForceMap provider. Its rows and derivatives are embedded/contracted through the explicit physical mapping; the finite G is independently differentiated and never substituted for B. Extra actual columns can have nonzero G and zero current B. Previous query columns of B are zero; finite G still includes their complete chain rule.

Validity is separate: `incrementValid`, `configurationJacobianValid`, `queryJacobianValid`, `GValid`, `forceMapValid`, `DforceMapValid`, `forceMapQueryDerivativeValid`, `physicalForceMapValid`, `physicalDforceMapValid`, and `physicalForceMapQueryDerivativeValid`. `operatorReady` requires full derivatives. Value mode returns bit-identical increment/B using scalar-only finite algebra; absent derivative buffers contain NaN and their flags are false. The original `physicalForce` object's absent derivatives are null, following its existing API.

Outputs are reusable scratch. Numeric buffers and column identities retain their references; metadata/buffer references cannot be replaced. Every evaluation first invalidates all outputs. Any error leaves all numerical buffers NaN, every operator flag false, `physicalForce=null`, and an owned reason/transport record. Candidate inputs and preparation sources are not mutated. A valid retry recomputes the current bound poses and restores valid results. The provider does not infer instantaneous velocities or prescribed feed/wall power and does not issue a contact certificate.

## Transport domain and derivatives

The evaluator traces the current own label through exact linear-map boundary event times. Each smooth piece uses the existing explicit short tangent swing, own unwrapped spin and symmetric mean-lever finite rule. The compatible external/own junction is a separate event. At tau=1, the outgoing trace and zero-duration target piece remain in AD; deleting them would lose the one-sided query derivative.

Compatibility requires shared accepted/current junction position through the same node binding, matching accepted tangent, and zero accepted **unwrapped physical orientation mismatch** `thetaOwn−thetaReservoir+referenceTwist` to frame tolerance. Independent gauges are allowed when angles and the directed reference lift change consistently. A 2*pi accepted spatial mismatch is not discarded as an equivalent physical orientation. Nonzero CURRENT temporal turns are retained.

Current external/own bending and spin are explicit supplied pose dependencies, not frozen. For bounded changes tending to their compatible accepted poses as dt tends to zero, the verified limit is the current physical B applied to actual rates plus prescribed material feed. Sharp accepted reservoir mismatch or any internal hinge crossing returns `surface-material-transport-required` with owned metadata; it is not zeroed, smoothed or treated as another compatible edge.

The finite AD first differentiates all supplied pose and query arguments, then contracts the exact constant dependency Jacobian. There is no runtime finite difference, approximate derivative, extra material advection, common-frame assumption, stiffness modification or hidden force-column truncation.

## Verification and integration handoff

**16/16 tests PASS.** The standalone test imports only the two new modules and the frozen existing SurfaceMotion module.

- Actual proximal `dsDt=-0.3`: slip [0.03,0] at dt=0.1, [0.003,0] at dt=0.01 and [0.00003,0] at dt=0.0001. The old point is inside the explicit external span. Reverse feed traces old own material and returns [-0.03,0].
- Moving/bending own endpoints and spin: rate error decreases from 1.6211e-5 to 1.6210e-7 as dt decreases from 1e-3 to 1e-5; corresponding power error decreases from 1.5169e-7 to 1.5306e-9.
- A full own 2*pi turn contributes 1.25663706144 circumferential slip for radius/lever 0.2.
- Expanded attachment to two actual adjacent edges: N=11, with nonzero upstream G and zero upstream current B. Actual body-force/virtual-power packing is preserved.
- Every one of 30 configuration/query columns passes finite differences: maximum G error 3.57e-11, maximum DB error 8.79e-11. Terminal one-sided foot G error is 7.45e-8 at h=1e-6.
- Same-edge values/G match the frozen finite operator at interior and endpoint traces. Prescribed/attached/expanded-support value/full equality, gauge and observer/common finite motion, typed identity preservation, 30 full/value evaluation rejection cases, preparation provenance failures and retry behavior pass.

Root can import the two new modules and new test, then use `configurationColumns` for its generalized G/B/DB pullback. The existing current physical 7-column map remains separately accessible. The caller must preserve the original current contact geometry and query chain, and supply a genuine compatible external pose/dependency contract. Interior hinge smoothing and a consistent surface/contact reconstruction remain open work.

Frozen bundle: `/tmp/oet-proximal-reservoir-surface-production-final-8996`. Its patch contains only the two NEW production modules, NEW test and this report. Existing SurfaceMotion SHA-256 remains `545c1b1b3fe25e6e045bf0745e1ed4c4a45d801640a4f03860083224320af92d`.

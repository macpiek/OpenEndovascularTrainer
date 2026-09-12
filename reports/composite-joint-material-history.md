# Own accepted material velocity history for joint feed preparation

`kirchhoffCompositeJointMaterialHistory.js` is a bounded translational history helper. It does not alter JointTimeStep, geometry, topology, material frames, contact state or history. It does not add remeshing or certify a fed physical dt.

```js
const history = createCompositeJointMaterialHistory({
  materialVelocities: acceptedState.materialVelocities,
  reservoir: ({toolId, s, trace}) => explicitAffineRecordOrNull
});
const v = history.sample('wire', currentMaterialLabel, {trace: 'right'});
const prepared = history.prepare({coordinates, inertiaEdges, maxPieces: 10000});
```

The helper validates and privately owns each accepted tool's records `{id,sStart,sEnd,velocities:[v0,v1],interpretation:'physical-material-velocity'}`. Spans are sorted by material label, independently of mesh edge numbering, common position, overlap/exposure or the other tool. Duplicate/overlapping histories reject. Unknown regions remain unknown. Labels and field ownership are exact; no physical tolerance snaps boundaries, fills gaps or extrapolates history. At a shared boundary, a discontinuity requires explicit `trace:'left'` or `'right'`. A boundary with equal values permits an unqualified point query but remains a quadrature boundary.

The current material map supplies `sStart`, positive `dsDx` and scalar or two-endpoint `dsDt`. The query uses its **current label directly**. The helper never shifts that label by dsDt, dt or a feed distance; this avoids double advection. It samples only physical translational velocity. Angular velocity, material spin and frame spin remain explicitly unknown/null.

Every old affine-field boundary crossed inside a current edge becomes `requiredCuts:{edge,toolId,s,x,reason}`. Output per tool contains `pieces` with original-edge fractions, coordinates, one-sided old endpoint velocities and two Gauss points per piece. Each Gauss point owns its label, old material velocity and coordinate/material quadrature weights. All source boundaries are preserved, including a continuous value with a changed derivative; there is no inference that equal samples imply one global affine field.

These are cuts of the **integration domain**, not added geometric nodes. Fractions refer to the original current edge, so a future integrator must retain its original geometric basis while integrating the pieces. Multiple tools may supply different cuts at the same or different coordinates. They remain independently identified; a shared integration loop may take their union.

A whole-edge `oldMaterialVelocities` pair is returned only when the entire requested interval belongs to one old affine field. Otherwise it is absent and `compatibleWithAffineEdgeOperator:false`; the top-level result has `requiresSubdivision:true`. The current two-point MaterialInertia/JointTimeStep API must consume piecewise integration in a later patch, or reject that preparation. Passing just two endpoint values across those cuts is not an equivalent integration. This helper supplies neither mass nor previous geometry; the caller retains those separate physical inputs.

Missing history, a gap, or newly entering material requires an explicit external reservoir provider. The provider must return an affine record for the requested physical tool and one-sided label, not a default vector or zero. It may return null when unknown, which produces `missing-material-history`. Accepted history always owns its known interval, even if a supplied reservoir span extends across it. Provider records are copied and never cached across successful or failed preparations. The caller must keep the provider deterministic for identical retries; this helper does not turn a mutable external process into accepted physical history.

All output arrays are independently owned. Source mutations, output mutations and failed preparation cannot alter the private accepted snapshot. A bounded piece budget also rejects without partial history updates.

## Validation

Nine tests pass, including:

- Opposite feeds, nonunit dsDx and nonuniform current mesh, with independent wire/catheter labels and no second advection.
- Crossed source edges, explicit left/right discontinuous traces, continuous endpoint values with changed slope, and every required integration cut.
- Independent overlap/exposure, reservoir ownership, missing data, ambiguous/invalid records and no extrapolation.
- Source/output mutation isolation, failed piece-budget preparation and deterministic retry.
- Two actual accepted JointTimeStep dt on a small fixed-topology fixture with different tool geometries/metrics and independent free rigid velocities. Each next unchanged-map endpoint exactly matches its own prior accepted physical velocity history. No history is borrowed from the other tool.

An independent discontinuity control has v=1 on material [0,1] and v=3 on [1,3]. The exact momentum integral is7 and kinetic integral is9.5. Piecewise Gauss2 reproduces both. Two whole-edge Gauss points give kinetic7.5; replacing the history with one affine endpoint field is also incorrect. This demonstrates why returning cuts is required before general feed can use the current inertia API.

The helper passes syntax validation. Test output is `reports/composite-joint-material-history-tests.txt`. The frozen bundle at `/tmp/oet-composite-joint-material-history-final-v2` includes helper/test/report and immutable dependencies for the actual-dt control; source hashes are recorded in its manifest and `reports/composite-joint-material-history-source.json`. No package script was changed.

Packaging note: the first immutable bundle omitted the installed Three.js dependency newly imported by the root wall integration. The v2 bundle includes the exact local Three.js0.160.1 ESM build and license; helper/test source is unchanged. The first manifest is retained as packaging history.

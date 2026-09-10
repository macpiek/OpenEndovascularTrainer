# Own piecewise surface material transport: executable design and limits

The negative-label-rate proximal advancement case can be closed with an explicit reservoir **pose** history and an endpoint transport event. A velocity-only reservoir cannot supply that pose. An interior sharp DER hinge needs an additional orientation-path convention and has a separate mathematical limitation: its piecewise-constant director jump can produce finite slip as dt tends to zero. Consequently, tracing an old label into a neighboring edge does not, by itself, justify enabling the existing smooth Coulomb operator at that jump.

This task adds only a standalone test/probe and this report in 8996. Production SurfaceMotion remains SHA-256 `545c1b1b3fe25e6e045bf0745e1ed4c4a45d801640a4f03860083224320af92d`; no root or contact implementation has been changed. The executable proposal is in [kirchhoffCompositePiecewiseSurfacePath.probe.test.js](/Users/macpiek/.codex/worktrees/8996/OpenEndovascularTrainer/tests/kirchhoffCompositePiecewiseSurfacePath.probe.test.js:52). It uses its own first-order dual algebra and the existing finite/physical-B functions only as external controls. **11/11 probe tests pass.**

## Data actually present, and data still required

| Data | Current source/state | Required action for a pose-history provider |
|---|---|---|
| Accepted own physical positions | `state.toolPositions`, independently checked against `prepared.previousPositions` by JointTimeStep | Snapshot by physical tool and stable edge/node identity; retain original affine geometry. |
| Accepted own directors/tangents | `state.tools[id].reference[edge]` | Copy each actual edge's frame and verify it belongs to that edge's accepted endpoints. |
| Accepted unwrapped own angles | `state.angles.get(id)[edge]` | Preserve independently for every visited edge. |
| Accepted spatial reference winding | `referenceTwists[vertex-1]`, transported and unwrapped in `commitFrames` | Use as a directed hinge-reference lift, verify its phase against the two accepted frames, and keep the physical branch explicit. |
| Candidate own geometry and angle | Candidate geometry and candidate angle maps | Needed on **all visited edges** for the proposed physical space-time reconstruction, not only the current contact edge. |
| Accepted material-label spans | Committed `materialVelocities` include per-edge `sStart/sEnd`; friction history retains `currentMaps` only for previously touched contact edges | Preserve the entire own pose/map history, including edges not previously in contact. Derive an old map from current map minus dt times endpoint label rates only under the already-declared linear-map contract, then verify against accepted spans when present. |
| Current maps and label rates | `prepared.inertiaEdges[].tools[].materialMap` | Keep maps fixed preparation inputs and retain each tool's own label measure. Never advect a sampled label twice. |
| External material history | `JointMaterialHistory` reservoir supplies `id,sStart,sEnd,velocities,interpretation` | Extend through a separate explicit pose provider: covered label interval, endpoint positions at both times, accepted frame, unwrapped old/current spin, junction connection/lift, ownership and motion/derivative contract. |

Relevant inspected sources: [JointMaterialHistory](/Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer/src/physics/kirchhoffCompositeJointMaterialHistory.js:50), [JointTimeStep state](/Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer/src/physics/kirchhoffCompositeJointTimeStep.js:90), [accepted velocity commit](/Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer/src/physics/kirchhoffCompositeJointTimeStep.js:189), and [friction map preparation](/Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer/src/physics/kirchhoffCompositeJointWallFrictionRows.js:92). MaterialHistory explicitly returns `includesAngularHistory:false`; its quadrature splitting is a useful label-ownership pattern, not a surface orientation reconstruction.

The older [quadratic-hinge SurfaceMotion](/Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer/src/physics/kirchhoffCompositeSurfaceMotion.js:82) does state a continuous reconstruction and spatial spin connection, but it changes the affine centerline/contact field. It cannot be silently substituted for the actual affine wall/lumen surface.

## Concrete API and reconstruction choice

The executable `piecewisePath(input)` accepts:

```js
{
  dt, toolId,
  nodes: { old: [[x,y,z], ...], current: [[x,y,z], ...] },
  edges: [{
    id, toolId, source: 'accepted' | 'reservoir',
    poseProvided: true, // required for a reservoir record
    nodes: [i,j], coordinates: [x0,x1],
    oldLabels: [s0,s1], newLabels: [s0New,s1New],
    frame: { tangent, director }, oldAngle, angle
  }],
  hinges: [{ left: edgeId, right: edgeId, referenceTwist }],
  query: { edgeId, coordinate, trace, previousTrace },
  geometry: {
    previous: { point, normal, tangent },
    current: { point, normal, tangent }
  }
}
```

Every edge belongs to the same actual tool. Adjacent spans must share their physical node and continuous old/new material labels. Missing coverage rejects; no point or frame is extrapolated. Both current and accepted endpoint traces are explicit when a label lies on a junction. A reservoir is an actual supplied external affine pose span, with a declared connection to the modeled rod, not a fabricated copy of the proximal edge.

A production preparation/evaluation split can wrap precisely this contract:

```js
const history = createOwnSurfacePoseHistory({ acceptedState, acceptedMaps, reservoirPose });
const path = history.prepare({ toolId, targetEdgeId, currentMaps, query, dt,
  reconstruction: 'piecewise-own-affine-space-time',
  hingeRule: 'short-swing-own-unwrapped-spin',
  leverRule: 'symmetric-mean-per-piece', queryFrames });
const result = path.evaluate({ candidateOwnGeometry, candidateOwnAngles, order });
// result: increment, G, configurationColumns, queryColumns,
// label, pieces, events, branchMargins and validity/provenance
```

These production names are a proposal, not newly exported functions. The test already evaluates the complete specified algebra and all its derivatives.

The reconstruction is **explicitly a discrete extension of the existing finite mean-lever rule**. It is not a claim to recover an unknown continuous contact trajectory from endpoint states. Normalize the two query frames as C=(n,t×n,t). Express old/current own endpoint positions relative to their respective query origins in C. Linearly interpolate those endpoint coordinates with normalized time τ. The intermediate affine tangent is the normalized interpolated chord. Carry each own old reference to that tangent by its own shortest time-PT, and add the explicitly computed endpoint reference/contact-frame phase proportionally to τ. Interpolate that edge's own unwrapped angle in τ. This is objective under common motion of body and query; it does not impose a common material frame on different edges or tools.

Each smooth path piece uses its endpoint tangent swing and own relative spin with the existing symmetric mean-lever formula. At a crossed junction, the two traces meet at the same physical center but can have different orientations. Apply a **separate** shortest tangent swing plus the directed unwrapped physical twist `thetaRight−thetaLeft+referenceTwist`. Reverse traversal reverses the direction/lift. A reference-gauge change must change each own angle and the directed reference lift consistently; changing a physical winding by 2π must remain visible.

The accepted reference lift selects a valid continuous short temporal chart; an antiparallel tangent, an unresolved phase, a lift branch boundary, simultaneous events or changed path ownership must reject/reprepare. Long temporal reference paths require further explicit chart/phase anchors. The bounded probe does not claim production validation, caching or all-event lifecycle coverage.

## Label tracing, finite slip and G

Compute the current label once, `s*=s_new,target(x_current)`. On any visited edge,

`f_e(τ) = (s*−S_e,left(τ))/(S_e,right(τ)−S_e,left(τ))`,

where each boundary label is linearly interpolated between the supplied accepted and current maps. Every possible crossing time is explicit:

`τ_h = (s*−S_h,old)/(S_h,new−S_h,old)`.

Sort valid events and choose the unique own edge covering the label between events. Do not substitute the current edge index for this search. Stationary labels exactly at a hinge require matching accepted/current traces. An event at τ=0 depends on the accepted trace; an event at τ=1 depends on the requested outgoing/incoming target trace. A zero-duration segment at either endpoint can have a nonzero one-sided derivative and must be retained.

For each path piece in contact coordinates, let `b` be the shortest tangent swing vector and `alpha` its own unwrapped relative spin. With `k=acos(c)/sqrt(1−c²)`, `c=t_a·t_b`,

`Tbar=(t_a+t_b)/(k(1+c))`, `Omega=b+alpha*Tbar`,

`Delta = (c_b−c_a) + Omega × (−(c_a+c_b)/2)`.

The parallel limit uses the analytic removable-singularity series, as does the existing finite operator. At a zero-length hinge, `c_a=c_b` and the second term still contributes. Sum all smooth and jump pieces, then project onto the axial and circumferential C components. For two tools, construct each independent own path and subtract its result in the same objective contact query coordinates.

The exact derivative of this **stated discrete rule** is

`DDelta = D(c_b−c_a) + DOmega × meanLever + Omega × DmeanLever`.

All `Dτ_h`, fractions, intermediate nodes/frames, unwrapped angle interpolation, moving foot and both query-frame derivatives enter. For frozen preparation maps, `Dτ_h=Ds*/(S_h,new−S_h,old)`; differentiating map controls would also require their numerator and denominator terms. There is no runtime finite differencing in the prototype. Finite differences are only its independent test oracle.

The no-crossing specialization agrees with the existing finite rule in both values and all mapped G columns. It also satisfies `slip/dt → Bᵀ qdot + prescribedFeed` in the existing packing convention, and the corresponding traction-power limit. Here B remains the independent instantaneous physical force map; G is the finite slip derivative and cannot replace B.

A physical space-time path through two adjacent own edges gives G on **9 node-coordinate components plus 2 independent spins**, even when the current contact's B uses only its own 7 physical columns. The probe witnesses nonzero derivatives on the upstream edge's current nodes. Thus production must publish a separate path support/column map and scatter G over that support. Existing 7/14-only contact pullbacks must not truncate it. If external pose parameters are prescribed, their G columns are contracted with their explicit zero or supplied dependency Jacobian; an attached moving reservoir endpoint is not silently treated as fixed. Changing to an accepted-spatial-then-temporal splitting could alter support, but would be a different declared finite reconstruction requiring its own proof.

## Independent witnesses and the sharp-hinge obstruction

For a static 90° bend, q=[(-1,0,0),(0,0,0),(0,1,0)], old/current material-map offset 0/−0.1, current foot 1.05, radius/lever r=0.2, and own angle difference beta=0.5:

- The same label is 0.95 and crosses at τ=0.5.
- Axial/circumferential slip is **[0.364159265359, 0.063661977237]**.
- Reverse feed with reversed maps traces the same label and gives the exact negative to roundoff.
- The hinge contribution alone is `[r*pi/2, 2*r*beta/pi]` in these two components. Adding 2π to the spatial winding adds **0.8** circumferential slip.
- All 30 pose/query columns match finite differences to **8.72e−11**. Deliberately freezing event times changes G by **1.30079**.
- Independent gauge changes, observer rotation/translation, and common finite geometry/query motion preserve the expected result.

At fixed unit feed, choose the current foot `1+dt/2` so the sharp hinge is crossed for every dt. The finite hinge contribution does not shrink:

| dt | Axial slip | Axial slip/dt |
|---:|---:|---:|
| 0.1 | 0.364159265359 | 3.64159265359 |
| 0.01 | 0.319159265359 | 31.9159265359 |
| 0.001 | 0.314659265359 | 314.659265359 |

The circumferential jump remains 0.063661977237. A bounded instantaneous B on the outgoing affine edge cannot reproduce this as an ordinary finite pointwise velocity. This is a sharp-frame discontinuity, not a missing interpolation coefficient. General hinge Coulomb requires either an explicitly resolved finite-extent physical surface/frame reconstruction and consistent contact/force map, or a separately defined junction/measure-valued work law. The present test defines the jump and exposes its work; it does not certify that the existing smooth point-contact friction law can consume it.

## Real reservoir advancement result

For a supplied straight external pose span ending at the proximal node, with its accepted orientation compatible with the first own edge and `dsDt=−0.3`:

- The old label is −0.03 for dt=0.1 and is found inside the actual supplied reservoir interval.
- Material reaches the proximal outgoing right trace at τ=1. The explicit compatible terminal jump has zero value, while its derivative and the zero-duration target piece remain active.
- Slip is **[0.03,0]**, then [0.003,0] at dt=0.01 and [0.00003,0] at dt=0.0001: rate 0.3 to roundoff.
- Nonzero candidate bending and distinct reservoir/own spin rates retain the B/feed power limit: rate error 1.5961e−5 at dt=1e−3 falls to 1.5960e−7 at dt=1e−5.
- An additional full own 2π turn adds **1.25663706144** circumferential slip for r=0.2. Angles are not frozen.
- Reverse feed samples accepted own material and gives [−0.03,0] without needing an external old pose.
- All 30 terminal pose/query columns pass their derivative checks. Maximum error 1.90e−6 is from the explicitly one-sided endpoint-foot finite difference at h=1e−6; the smooth columns use central differences with 3e−8 tolerance.

A reservoir with a finite orientation mismatch at the accepted junction recreates the sharp-jump obstruction. Merely supplying any external orientation does not guarantee a regular B limit. Missing reservoir pose, missing own orientation, stale frames/lifts, absent label coverage and unknown hinge paths reject in the probe.

## Proposed implementation boundary

First implement an owned surface-pose history, exact label/event preparation, and the **compatible proximal reservoir-entry case** with its explicit current/accepted pose and derivative contract. This closes real advancement while retaining the existing same-edge finite path and physical force map on their valid domains. Keep interior sharp-hinge jumps reported as separate transport events until their physical contact/work convention is approved. An event's existence is useful output and must not be hidden by zeroing its rotation.

This is ready for a design decision; there is no proposed automatic guard removal or production patch. The general executable path and its sharp-hinge witness make the remaining physical choice reviewable. Full frozen test source, source audit snapshots, log, patch containing only this new test/report, and checksums are in `/tmp/oet-piecewise-surface-path-design-final-8996`.

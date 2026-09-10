# One actual affine tool against stationary wall material

`kirchhoffCompositeJointWallSurface.js` is a local physical7 surface provider.
It composes a selected original wall contact/differential with the existing
finite slip and physical B/DB operators. It performs **zero new wall queries**,
changes no normal gap or reaction, and owns no accepted history or timestep.
Only this new source, its new test and this report set are changed by this task.

## API

```js
const scratch = createCompositeJointWallSurfaceWorkspace();
const result = evaluateCompositeJointWallSurface({
  current: {
    field,                    // original selected provider
    row,                      // ordinary collector row; JointWallRows record.raw
    positions: [ownA, ownB],   // actual physical endpoints from that query
    plane: {normal, offset},  // required only for analytic-plane
    localFaceIndices          // optional known incident BVH faces; no search
  },
  tool,                       // own SurfaceMotion geometry/frame/map history
  dt,
  wall: {
    motion: 'stationary-material',
    source: 'sparse-sdf',     // explicitly prepared expected source
    tangentBasis: 'projected-own-tangent'
  },
  order: 'full'               // default; 'value' supplies increment and B
}, scratch);
```

Supported source values are `analytic-plane`, `sparse-sdf` and
`sparse-sdf-bvh`. A smooth ordinary row is required; `current.seam` rejects.
The caller still owns source/field revision, winner/locality, original-query
freshness and accepted chart identity. This helper revalidates the selected
record and differential, but does not claim a global winner certificate or
prove that an unreported provider object/revision has not changed.

The tool supplies its own `id`, `edgeId`, layout `edge`, increasing
`coordinates`, current and previous physical endpoints, accepted reference
frame, own current and previous unwrapped angle, current affine material map
and `materialPath:{kind:'linear-affine-maps',previousEdgeId,previousMap}`.
`tool.positions` must exactly equal `current.positions`; owner/edge/radius
must match the row. Optional `tool.coordinate` must equal the derived own
coordinate. `materialMap.dsDtEnds:[rate0,rate1]` is interpolated at the original
selected fraction; an additional scalar `dsDt` must agree. The existing
old/current map consistency checks are preserved. No default zero feed,
angular rate, old geometry or old material map is supplied.

Ordinary capsule fraction is the original `row.t == rawContact.segmentT`.
An envelope `proximal`/`distal` row uses its **physical** `row.t == 0/1`, not
the sample fraction of its degenerate point query. Own endpoint and old
material-label endpoint traces remain explicit. Crossing a real material
hinge, undefined prior frames, unknown rotation path and changed source
reject; no cap/portal/fillet, seam or history extension is invented.

`result.tools=[{id,edgeId,edge}]` binds the existing joint pullback. Physical
order is `[own q0.xyz, own q1.xyz, own theta]`:

| Field | Storage |
| --- | --- |
| `increment` | 2 axial/circumferential finite increments |
| `slipJacobian` | 2×7, component then physical coordinate |
| `forceMap` | 7×2, physical coordinate then traction component |
| `DforceMap` | 7×2×7, physical coordinate/component/coordinate |
| `currentQueryJacobian` | 10×7, own coordinate, point3, normal3, tangent3 |

`incrementValid` and `forceMapValid` are true for full/value success.
`operatorReady`, `slipJacobianValid`, `DforceMapValid` are true only for full.
Value derivative buffers contain NaNs. Pass only B and its true flag into a
force-only pullback; do not pass invalid NaN arrays as supplied maps. Any
failure revokes all flags and fills all public numerical buffers with NaNs.
The same heavy one-tool finite/B scratch and normal differential workspaces
are reused across contacts and dt. Small validation/vector scratch remains;
this is not an allocation-free or full-step performance claim. Published
buffers are overwritten on reuse: copy accepted data outside this provider.

## Stationary material-point rule and its justification

Let x be the current selected axis point, n the original inward unit normal,
d the signed distance and P the original `raw.closestPoint`. The provider
requires the original policy `P=x-d*n`. For a plane P is its true projection;
for BVH P is the original triangle-feature foot. For sparse SDF it is the
existing provider's **virtual normal projection**, which need not lie exactly
on the zero isosurface. This distinction is exposed in `identity.wallWitness`.
At zero gap d=radius. Off contact P remains an explicit common-force-point
extension. Original normal force has identical torque at x and P because
P−x is parallel to n, so this policy preserves its discrete force/wrench.
It is not a claim of an exact cylinder–wall intersection.

Select the **current** wall point P and current wall tangent frame C as the
wall material point for this evaluation. A stationary wall has that same P
and C at the previous time. Therefore BOTH finite geometry query blocks
receive identical P,n,t values. No old nearest-point query is constructed.
The tool's previous center is independently sampled at its CURRENT material
label through its own accepted affine map/history.

For translation with unchanged orientation, the finite center difference is
`C^T[(q_current-P)-(q_previous_at_current_label-P)]`.
P cancels and tangential translation remains. Replacing the second P with a
previous nearest point would instead cancel sliding along a plane. The
regression produces `[0.3,0.4]` for a translation of that amount, not zero.

In the dt→0 limit the resulting surface rate is
`v_material + omega × (P-q)`, projected into C, exactly the existing
instantaneous wall-relative rate for explicitly zero wall velocity. The
finite operator retains its declared shortest tangent transport, both own
accepted frame/angle values and own unwrapped winding. Its mean-lever/angular
increment is a discrete rule, not an exact integral of arbitrary unknown
finite motion. This provider does not infer multi-turn bending paths or
transport across a hinge.

The default tangent is `a=T-(T·n)n`, using the actual normalized own chord T.
At normal incidence it rejects, requesting an explicit alternative. The
supported alternative `projected-own-reference-director` first time-parallel
transports the own accepted director onto current T, then projects it onto
the wall tangent plane. Its complete derivative is included. No arbitrary
world axis or hidden branch switch is used. Wall friction is isotropic; the
explicit-basis test verifies nonzero finite-slip magnitude and physical-rate
power invariance under a changed own reference gauge.

## Complete local chain rule

The wall point derivative is `DP=Dx-n*Dd-d*Dn`, where `Dd` is the ORIGINAL
signed-distance derivative, not the unit normal force column. For sparse
SDF these generally differ. The original exact branch differentiator obtains
Dn with no query, including normalization of the trilinear gradient. For
BVH it differentiates the actual face/edge/vertex feature without a search.
The supplied original G, signed normal column and DB are revalidated. All
sample-count, face, normal, gap, radius, source and own-foot metadata agree.

The analytic projected tangent derivative includes both DT and Dn; the
explicit director alternative includes time-PT differentiation. The original
selected fraction is fixed on this declared branch, so its own-coordinate
derivative is zero. The current physical geometry may change all other
queries. With Q the composed current query Jacobian:

`G = G_configuration + (G_current_query + G_previous_geometry_query) Q`.

The previous geometry block contains the SAME current wall material point
and frame, hence its chain derivative must be included too. The one own-foot
query appears only once. The previous own material center remains traced by
the finite evaluator; it is not frozen to the old geometric foot.

`DB = DB_configuration + DB_query Q`.

The physical map B is still the independent instantaneous virtual-power map,
not transposed G. Its endpoint bending couples, time-PT chart connection and
own spin torque are retained. The joint pullback test checks actual loads.

## Validation (frozen dependency SurfaceMotion 545c1b1b...)

11/11 new tests pass, and 68/68 affected SurfaceMotion/LumenSurface/Pullback/
WallSurface tests pass. Source and test syntax checks pass. No tolerance was
changed. FD step is 1e-6 and original absolute gate is 3e-8 for G/DB.

| Original wall branch | maximum G FD error | maximum DB FD error |
| --- | ---: | ---: |
| Analytic plane | 2.8433e-11 | 1.1045e-10 |
| Quantized sparse-SDF interior sample | 6.7176e-11 | 1.6938e-10 |
| Actual MeshBVH strict edge feature | 7.4885e-11 | 2.3880e-10 |

Simultaneous material feed, bending and spin at dt=[1e-3,1e-4,1e-5] produces
rate errors `[3.704660e-4,3.704545e-5,3.704551e-6]`, and virtual-power errors
`[2.223810e-4,2.223878e-5,2.223892e-6]`: the declared finite rule approaches
its independent instantaneous rate with first-order convergence.

The actual Aorta collision asset produces a supported original sparse-SDF
record; disabling both field query functions after selection still permits
composition. Separate BVH control disables the actual BVH nearest query too.
Other controls cover ±2π winding, independent feed and nonunit/affine maps,
normal-force and friction wrench, explicit basis/gauge, envelope endpoint
provenance, input ownership, full/value bit parity and full/value/invalid/full
reuse. Invalid full AND value paths preserve all geometry/history gates.

This is a frozen local operator handoff. It does not implement Coulomb state,
contact activation, nonlinear wall-friction dt, accepted history, runtime
adapter or UI. It establishes no FPS or whole-step latency result.

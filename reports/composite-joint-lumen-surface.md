# Strict-side actual-geometry surface composition

The new `kirchhoffCompositeJointLumenSurface.js` composes the existing original
side-contact record and its analytical geometry derivatives with the frozen
finite surface increment and instantaneous B/DB. It produces a complete local
**physical14** operator suitable for `JointSurfacePullback`. There are zero
extra detector queries, manifold changes, winner searches or history commits.
Only this new module, its new test and reports were written.

## API and owned scratch

```js
const workspace = createCompositeJointLumenSurfaceWorkspace();
const result = evaluateCompositeJointLumenSurface({
  current:  { input: originalCurrentInput,  contact: originalCurrentSide },
  previous: { input: originalPreviousInput, contact: originalPreviousSide },
  tools: [innerTool, outerTool],
  dt
}, workspace);
```

Each tool uses the existing own physical-frame/material-map surface API and
also provides `materialSegmentId` matching its respective detector input.
`edgeId` is the own physical provider edge identity. Optional `edge` is the
actual layout edge index, passed unchanged to the joint pullback. Both own
current and previous endpoint arrays must **exactly** equal the corresponding
original detector endpoints. Own coordinate spans must increase.

The binder derives current coordinates from the fixed original inner fraction
`s` and the moving original outer fraction `t`. An optional supplied
`tool.coordinate` must agree with that derived coordinate. The material maps
and accepted physical frames are not replaced by geometry data.

`materialMap.dsDtEnds:[rate0,rate1]` is optional. When present, it is
interpolated at the **derived current own fraction**. An additional scalar
`dsDt` must agree; it cannot silently remain stale while the outer foot moves.
Without endpoint rates, the existing scalar API remains. The old/current
affine maps must still satisfy the existing declared linear map-history
contract. Their fixed slopes and endpoint rates are frozen parameters during
the geometry derivative. This supports unequal maps, independent feeds and
query-dependent affine label rates without double advection.

The output buffers are reusable read-only scratch, overwritten or revoked by
the next call on the same workspace. One heavy workspace can be reused across
all samples and time steps. No arena is allocated per contact by the binder.
The finite/force evaluators return their own numeric response data, which the
binder exposes as diagnostics; the binder's published maps have independent
preallocated buffers. Copy an accepted result before reusing scratch.

The standard pullback fields are:

| Output | Layout |
| --- | --- |
| `increment` | two finite axial/circumferential increments |
| `slipJacobian` | 2×14, component by physical configuration |
| `forceMap` | 14×2, physical DOF by tangential traction component |
| `DforceMap` | 14×2×14, physical DOF, component, configuration |
| `tools` | ordered `{id,edgeId,edge?}` |
| Validity | `operatorReady`, `incrementValid`, `slipJacobianValid`, `forceMapValid`, `DforceMapValid` |

Physical order is `[inner q0.xyz,q1.xyz,theta, outer q0.xyz,q1.xyz,theta]`.
`currentQueryJacobian` is the composed current-query 11×14 map, retained for
inspection. On any unsupported branch, stale record, malformed data or late
finite-history failure, the call throws **after invalidating every flag and
filling every published numerical map with NaN**. Retry resets both geometry
workspaces and arenas. Inputs and accepted histories are never mutated.

## The explicit virtual common-point policy

For each original side record, let p be its fixed-s inner axis point, q its
projected outer axis point, n its original outward radial normal, and ti the
normalized actual inner chord. Define

`innerSurface = p + ri * normalize(n−(n·ti)ti)`,

`outerSurface = q + ro * n`,

`point = (innerSurface+outerSurface)/2`.

The raw contact tangent is the actual outer chord. This preserves the chosen
original detector normal and gap. Each individual witness lies on its own
actual cylindrical cross-section, but their midpoint is generally **not** an
exact intersection of the cylinders. Both tools use the same virtual point
for their physical force levers, which preserves action/reaction and physical
wrench. The response explicitly publishes `virtualCommonPoint:true`,
`exactCylinderIntersection:false`, both witnesses, their separation vector
and its norm in current and previous geometry.

This distinction matters even at original gap zero. For a 0.3-radian inner
tilt, ri=0.16, ro=0.5 and original radial distance 0.34, the two witnesses are
separated by `0.32*sin(0.15)=0.04782020239`; their midpoint does not lie on
both cylinders. The test preserves this witness and the original gap. The
module does not silently change the detector to an exact cylinder model.

The old virtual query is evaluated from the immutable **previous original
geometry at the same declared s**. This is distinct from the old centers of
the current material labels. Those centers are correctly traced through each
tool's own old map by the existing finite rule. In particular, the old outer
material foot varies when the current outer geometric foot varies; it is not
replaced by the old detector's t. Under feed, the old virtual point need not
be on those traced material sections. This is the declared common-point
extension of the finite rule, not a claimed exact surface-particle path.

Both current and previous records must be supported strict side contacts,
with the same material identities, radii, contact/feature identity and inner
sample. The inner and outer fractions are strictly interior. Missing old
contact data, radial fallback, endpoint/portal/fillet feature ownership,
undefined projection of n onto the inner cross-section, material crossing a
real hinge and incompatible own reference history all reject explicitly.

## Full derivative composition

`SideGeometry` validates both supplied original records against their exact
inputs without running detection. Its `outerTGradient` and `normalJacobian`
provide the moving projection and normal derivatives. The binder analytically
differentiates the normalized inner chord, projected inner radial direction,
both surface witnesses, their midpoint and the raw outer chord. The current
query derivative includes:

- Inner foot: fixed s, zero derivative.
- Outer own coordinate: own coordinate-span length times `D t`.
- Common point: half the sum of both differentiated surface witnesses.
- Normal: the original side normal derivative.
- Raw contact tangent: outer endpoint difference.

Each of these derivatives maps the physical positional entries into the
14-dimensional order with both spin columns retained as explicit zeros.
The current query map is Q. All current feet and geometry contributions are
then composed as

`G = G_config + G_currentQuery * Q`,

`DB = DB_config + DB_currentQuery * Q`.

The previous query geometry is immutable in a trial, so its direct derivative
is zero. The finite evaluator's current-foot columns already include tracing
the current material label into its own old edge; those columns are included
in the composition above. B remains the independent physical instantaneous
rate transpose. No finite G is substituted for B, and no additional geometry
force is added by differentiating a contact search as though it were a material
velocity.

## Verification

**9/9 new tests and 60/60 affected tests pass; syntax passes.** The new tests
use original detector records with independently moving nonparallel axes,
different own coordinate spans and dsDx, independent feed and unwrapped spin.
They check all 14 columns of G and all 14×2×14 entries of DB against finite
differences of the original detector plus the surface composition. Separate
controls cover common finite rigid motion, full winding, virtual power,
correct physical wrench including the time-PT spin connection, actual
JointSurfacePullback/load compatibility, previous/current witness identity,
frozen affine endpoint-rate input, stale-input revocation and identical retry.

The deterministic raw probe reports:

| Check | Result |
| --- | ---: |
| Full composed G FD error, h=10⁻⁶ | 4.200×10⁻⁹ |
| Full composed DB FD error, h=10⁻⁶ | 1.426×10⁻¹⁰ |
| B difference from existing instantaneous operator | 5.552×10⁻¹⁷ |
| Virtual-power error | 1.111×10⁻¹⁶ |
| Common finite R=0.8 rad, maximum slip | 7.826×10⁻¹⁶ |
| Extra detector/manifold calls | 0 |

With moving actual queries and independent feed, the finite-rule limit at
dt=10⁻³, 10⁻⁴, 10⁻⁵ gives max rate errors
`2.48524e−6, 2.43663e−7, 2.43186e−8` and power errors
`2.60487e−6, 2.59689e−7, 2.55720e−8`. The expected O(dt) convergence is visible;
there is no residual mismatch hidden by an acceptance tolerance. Existing
physical gates and dependencies were not edited.

```sh
node --check src/physics/kirchhoffCompositeJointLumenSurface.js
node --test tests/kirchhoffCompositeJointLumenSurface.test.js tests/kirchhoffCompositeJointSurfaceMotion.test.js tests/kirchhoffCompositeLumenSideGeometry.test.js tests/kirchhoffCompositeJointSurfacePullback.test.js
```

The immutable bundle is `/tmp/oet-composite-joint-lumen-surface-final`, with
the new files, frozen transitive dependencies, all four tests, deterministic
probe, reports and a SHA-256 manifest. Prior SurfaceMotion freezes remain
unchanged. This operator is ready to feed the parent's contact/friction row
manager. It does not implement a Coulomb state transition, accept a nonlinear
time step, provide unsupported hinge/tip histories, or establish runtime FPS.

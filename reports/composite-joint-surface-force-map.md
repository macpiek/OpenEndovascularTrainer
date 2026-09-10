# Physical surface force map and its derivative

`kirchhoffCompositeJointSurfaceMotion.js` adds
`createCompositeJointSurfaceForceMapWorkspace(T)` and
`evaluateCompositeJointSurfaceForceMap(input, workspace)`. This supplies the
instantaneous physical force map **B**, together with its configuration and
current-query derivative, for one actual affine tool against a wall or two
independent actual affine tools. It does not require endpoint rates, angular
rates, wall velocity, `dt`, material maps or previous theta just to obtain B.
These quantities are not replaced with zeros: total velocity and prescribed
feed/wall power are explicitly marked unknown in the response.

Only the owned SurfaceMotion module, its existing test file and new reports
changed. The previous finite/nonparallel/instantaneous source is retained as
an unchanged byte prefix; all previous 22 test bodies remain unchanged. No
solver, constitutive equation, time step, detector, pullback or history module
changed. This is a local derivative handoff, not accepted full-step friction or
a performance/FPS claim.

## Minimal API and storage

```js
evaluateCompositeJointSurfaceForceMap({
  tools: [{
    id, edgeId, coordinates: [x0, x1], coordinate, trace,
    positions: [q0, q1], previousPositions: [oldQ0, oldQ1],
    reference: { tangent: oldT, director: oldD }, angle: theta
  }, /* optional second tool */],
  forceGeometry: {
    kind: 'explicit-affine-side-query', point, normal, tangent
  }
}, workspace);
```

The accepted reference must belong to that tool's own previous physical edge.
Current foot coordinates must be inside that edge, with explicit one-sided
traces at its endpoints. Missing or degenerate geometry, stale accepted frames,
nonfinite derived lengths and ambiguous time-PT charts reject. The current
unwrapped angle is finite; the original current director convention is used.

The caller provides one objective current query. Its raw tangent and normal
are normalized and orthogonalized **inside AD**, consistently with the finite
increment operator. Geometry/provenance and the actual branch are caller
owned. The derivative includes the whole query; the caller must chain moving
feet, point, normal and tangent to its local actual geometry.

Let `n=7T`, `p=T+9`. The response contains:

| Buffer | Shape and ordering |
| --- | --- |
| `forceMap` | `[n,2]`: generalized DOF, axial/circumferential component |
| `configurationDerivative` | `[n,2,n]`: DOF, component, configuration column |
| `queryDerivative` | `[n,2,p]`: DOF, component, query column |
| `derivative` | `[n,2,n+p]`, same order, all columns |

Configuration columns are per tool `[q0.xyz,q1.xyz,theta]`. Query columns are
the `T` own current coordinates, then current `point.xyz, normal.xyz,
tangent.xyz`. Hence two tools have 14 configuration and 11 query columns;
one tool has 7 and 10. The response includes the orthonormal query axes,
normal, point, own centers/levers/tangents and each own `omegaMap` (3×7,
angular component by local generalized rate). Returned buffers are owned and
survive workspace reuse and failed/retried evaluation.

`forceMap` is the **transpose of the existing instantaneous rate Jacobian**,
not the finite-slip configuration Jacobian. Multiplying it by a tangential
traction `[f_axial,f_circumferential]` returns its work-conjugate generalized
forces. The first tool receives the positive physical traction and the second
the negative traction. For one tool the wall receives the opposite force,
but its prescribed velocity/power is separate from this unknown-free B.

## Exact local construction

The current material directors are evaluated once from the actual affine
chord, own accepted time-PT reference and own theta. A second-order local AD
arena supplies each director `d`, its first derivative `d_j` and its second
derivative `d_jk`. Then, directly,

`omega_j = 1/2 Σ d × d_j`,

`D_k omega_j = 1/2 Σ (d_k × d_j + d × d_jk)`.

An additional first-order arena carries these expressions and the entire
current query. For generalized rate j, its material-center rate coefficient
is `(1-f)e_j` or `f e_j` for the appropriate endpoint positional DOF and zero
for theta. Its surface rate coefficient is

`v_j = centerRate_j + omega_j × (point−center)`.

Project this onto the two current query tangents and apply the tool sign to
get row j of B. The derivative includes the center interpolation weights,
lever, angular coefficients and the normalized current basis. There are no
numeric differences and no 7T repeated full unit-rate evaluations in runtime.
The two-tool witness uses 212 second-order arena nodes and 1,355 first-order
nodes; one tool uses 106 and 705. Both arenas are reusable.

Feed is prescribed separately in the existing fixed-chart kinematics, so it
does not enter the Jacobian with respect to these generalized rates. Thus B
can be evaluated without asking for feed maps or a velocity history. This
does not discard feed from the total material slip or power elsewhere.

## Spatial moment in a time-PT coordinate chart

The net-force and moment checks use physical virtual motion, rather than
assuming generalized q-forces and physical spatial endpoint forces coincide.
For a current tangent different from its accepted tangent, a q variation at
fixed theta also changes the time-PT reference's physical axial angle.

Write `a_j=t·omega_j` for the six positional rate columns. For a rigid angular
velocity Ω and endpoint rates `qDot=V+Ω×q`, the correct scalar spin rate is

`thetaDot = Ω·t − Σ a_j qDot_j`.

Consequently, the physical positional forces used to decode a spatial wrench
are `Fphysical_j=Fgeneralized_j−Qtheta*a_j`, and the own axial torque is
`Qtheta*t`. This is a change of coordinates for the virtual-power proof, not
an alteration of B. It gives

`Σ Fphysical = appliedForce`,

`Σ q×Fphysical + Qtheta*t = point×appliedForce`.

The regression retains bending-induced endpoint force couples and the spin
torque. It also checks the rigid-motion power directly with the correct
`thetaDot`. Naively using `Σq×Fgeneralized+Qtheta*t` misses the reference-chart
connection: the finite bent witness shows moment errors 0.0003905 and
0.0025423 for the two tools, whereas the correct physical wrench errors are
about 10⁻¹⁶. The instantaneous original operator independently verifies the
same rigid-rate zero relative slip after its prescribed feed term is removed.

## Validation

**27/27 owned tests and 45/45 affected tests pass.** Five new tests cover:

- One/two-tool B parity with the actual existing instantaneous rate transpose;
  the same B and derivative with all unneeded rate/map/time fields absent.
- All 17 or 25 configuration/query derivative columns against independent
  central differences, full buffer indexing, source ownership and exact retry
  after a failed degenerate evaluation.
- Virtual power, per-tool total force, physical spatial moment, nonzero
  bending endpoint couples and own spin torque on nonparallel bent axes.
- The full actual strict-side projection chain rule, including the outer foot,
  radial normal, tangent and physical contact point. A negative witness that
  freezes query arguments produces an incorrect force derivative.
- Independent frame gauges, endpoint traces, stale accepted frames, missing
  queries, parallel normal/tangent degeneracy and nonfinite chord arithmetic.

The immutable deterministic probe records:

| Check | One tool | Two tools |
| --- | ---: | ---: |
| Max B difference from original | 2.776×10⁻¹⁷ | 2.776×10⁻¹⁷ |
| Max configuration-D error, h=10⁻⁶ | 7.502×10⁻¹¹ | 1.170×10⁻¹⁰ |
| Max query-D error, h=10⁻⁶ | 7.639×10⁻¹¹ | 1.140×10⁻¹⁰ |
| Virtual-power error | 0 | 2.221×10⁻¹⁶ |

Per-tool net force, correct physical moment and rigid-power errors are at
most 1.111×10⁻¹⁶. The actual strict-side chain-rule force-derivative error is
1.513×10⁻¹⁰; freezing the query incorrectly changes that derivative by 0.08039.
No existing test or physical acceptance threshold was weakened.

```sh
node --check src/physics/kirchhoffCompositeJointSurfaceMotion.js
node --test tests/kirchhoffCompositeJointSurfaceMotion.test.js tests/kirchhoffCompositeSurfaceMotion.test.js tests/kirchhoffCompositeSurfaceContinuity.test.js
```

The immutable bundle is `/tmp/oet-composite-joint-surface-force-map-final`,
with source, tests/dependencies, deterministic probe, reports and a SHA-256
manifest. The preceding nonparallel finite bundle is preserved unchanged.
The caller can now assemble `D(B*f)=DB*f+B*Df` with the supplied geometry
chain rule, while retaining the separate finite slip derivative for its
constitutive Newton row. This patch does not assemble or solve those rows.

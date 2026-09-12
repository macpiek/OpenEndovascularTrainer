# Explicit endpoint wall pressure: executable prototype

The min-capsule contact is an unsuitable persistent owner of Coulomb pressure
and traction: its selected fraction can jump on an arbitrarily small tilt of
a flat edge. Moving the old Fn/Ft to another sample is generally not an exact
transformation of the local slip, dissipation or torque. The independent
`composite-lumen-friction-gauge-audit/README.md` proves this obstruction.
Root's actual two-tool reproduction remains in
`composite-joint-flat-wall-friction-failure.mjs/json` (149 evaluations,
125 invalid trials); this prototype does not replace or claim to solve that
full JointTimeStep reproduction yet.

## Model decision

`JOINT_WALL_PRESSURE_SCHEME = 'nodal-endpoints-one-sided-surface'` is an
explicit PHYSICAL discretization selected **from initialization**, not a
reduction of pressure at the current capsule minimum. Each owned material
node/radius has exactly one integrated normal force Fn in N and one Ft pair
in N. Both incident endpoint samples at a shared material node refer to this
one pressure site. There is never an independent capsule Fn, duplicate node
Fn, or old gauge to transfer. A node force must not be applied or given a
second friction capacity μFn for each incident edge.

The site includes a mandatory incident `edge` and its material `trace`:
`right` at that edge's proximal endpoint, `left` at its distal endpoint.
This edge supplies the actual current/previous axis, accepted reference frame,
unwrapped spin and affine material labels. The choice is fixed independently
of load, slip, feed direction, current nearest sample or a temporary equality
of adjacent frames. It is serialized as part of the pressure law/history.

This is an explicit one-sided DER surface law. It is not an interpolation or
sum of two inconsistent edge spins at a node. Radius discontinuities require
a separately declared physical surface and reject in this prototype. Changing
a loaded site or importing a history from another pressure scheme rejects;
there is no post-hoc pressure choice made to match old tests.

All three original WallEnvelope inequalities (proximal, distal, capsule) for
EVERY owned edge are freshly measured and included in the certificate. For a
plane, endpoint feasibility implies capsule feasibility, but the original
capsule remains measured independently. On a curved wall the capsule may
penetrate with both endpoints outside; that condition remains a failure.
The executable quantized-SDF control has strictly open endpoints and original
interior capsule gap **−0.16875 mm**, and is correctly not certified.

## Bounded API

```js
const candidate = createCompositeJointWallPressurePrototype({
  layout, modes, relativeToolId, coordinates, contactOwners,
  sites: [{owner, node, edge, trace}, ...],
  edgeTools, // own accepted frames/positions/angles/maps; copied on preparation
  wall: {motion:'stationary-material', source, tangentBasis},
  dt, mu, normalScale, frictionScale, tolerances, history
});
const result = candidate.refresh({
  envelope, positionsByTool, anglesByTool,
  field, plane, localFaceIndices, order:'full' // or 'value'
});
```

`edgeTools` use the existing WallSurface own-history API. Provider `edgeId`
is explicitly a nonempty string in this prototype. A production manager must
use its existing typed encoder for primitive `materialSegmentId` values,
including bigint; this module does not coerce those identities to strings.

`forces` contains `[Fn,FtU,FtV]` per declared site in its frozen order.
`rows` contains the corresponding three local equations with the existing
RelativeDirection common/rho support contract. Friction row multiplier blocks
include their normal DOF and the other tangential DOF. Row indices here are
LOCAL, starting at zero. A production manager must add its explicit global
row offsets rather than directly concatenate them after unrelated rows.

Each refresh calls the already selected original endpoint's WallSurface
without additional detection, preserving current G/B/DB and the stationary
material-point rule. A separate pullback transports the original normal
G/physical B/DB through the same full physical coordinate map. The assembled
mechanical residual has `−Fn*Bn−Bt*Ft`; geometric terms are `−Fn*DBn−Ft*DBt`.
The physical B is not replaced by the finite-slip G. The normal and Coulomb
scales are numerical equation scales, not spring stiffness or regularization.
Private signed Fn trials are permitted by the existing exact NCP extension;
physical proof still requires literal Fn≥0, all original gaps, complementarity
and the original Coulomb cone/work/slip gates.

The prototype returns common/relative contact residuals, all original gap
records and per-site proofs. Full/value residuals are bit-identical. Value
invalidates H/J and the full direction guard rejects them. Failed evaluation
revokes published residuals/rows while retaining private force values; it
does not commit a state. `copyCandidateHistory()` is marked `accepted:false`.
Only a caller-explicit accepted history with the exact scheme/signature can
seed another preparation; the caller remains responsible for full mechanics,
physical time/history acceptance and old-map continuity.

The module owns copies of each prepared tool history, sites, coefficients,
wall settings and imported force history. Heavy WallSurface/equation scratch
is shared sequentially; per-site symbolic pullbacks are prepared once. It has
no global original-query generation guard, manager lifecycle, concurrent
ownership contract, nonlinear loop, standalone timestep or runtime adapter.
Those belong in the production existing managers/JointTimeStep, not another
integrator. WallEnvelope already caches shared endpoint queries: N endpoint
plus E original capsule queries for a contiguous owned chain. This increases
geometry sampling relative to capsule-only; this report makes no latency or
FPS claim.

## Shared-node rank and spin witness

At one shared node, the left distal and right proximal original normal columns
are identical after global scatter. At positive Fn, zero slip and Ft=0 (stick),
the Coulomb derivative with respect to Fn is zero. Therefore a mixed system
that retains BOTH pressure variables has the exact nonzero null direction
`δFn_left=+1, δFn_right=−1`, all other increments zero. Its force equation
columns cancel and every geometry/constitutive row annihilates the direction.
This remains true with distinct own edge reference gauges and independent
spin DOFs. No numerical rank threshold, pivot floor or SVD is involved.

The same physical node with radius .8 and independently known own spin
increments +.1 and −.2 rad has one-sided circumferential slips **−.08** and
**+.16 mm**, respectively. The exact normal-column equality does not make
the two surface laws equal. Both traces are supported by the existing surface
operator; this new pressure law explicitly selects one, and the corresponding
history signature differs. It never sums both spins or averages their slip.

## Executed common-band nonlinear witness

The test uses the existing exact Chain, full consistent material inertia,
separate ToolLengths and RelativeDirection operators, not a new production
solver. It has one actual three-node flat rod, coordinates [0,2,4], radius .8,
EI1=2, EI2=3, GJ=1, mass per material length .13, dt=1/120, dsDx=1 and dsDt=0.
These are declared synthetic controls, not measured guidewire properties.
Both original rest lengths are 2. Old material velocity is [.02,0,0]. Loads
are [.1,−.4,0] N per node; own spin boundary rates are [.03,−.02] rad/s.
The incoming physical geometry is flat; the first private Newton guess has
a ±1e−7 mm tilt to exercise the original sample switch.

One fresh unshifted band solve contains all common DOFs, length multipliers
and the three normal/Coulomb contact blocks: **22 unknowns**, local half-band
21 for this small three-node fixture. It converges in **6 directions** with
zero unsupported/invalid trials. The original first capsule changes t=0→1
in the first direction, without changing any pressure or traction identity.
Final residuals: maximum free mechanical force **2.895e−13 N**, maximum row
**4.441e−16 mm**; all original gaps are zero and all Coulomb gates pass.

Final nodal Fn in N:

`[.388423484331688, .3840637228881094, .42751279278020277]`

Final Ft in N:

```
[-.028941538337456954, +.025905894897980265]
[-.032528978526698685, -.020418496232492755]
[-.03625299479834822,  -.022658160738643886]
```

The independent affine material momentum balance is

```
momentumRate = [.202276974355582, 0, -.0171701885391684]
appliedLoad  = [.3, -1.2, 0]
contactLoad  = [-.0977230256439899, 1.2, -.0171701885391684]
```

Every site's physical endpoint force/couple plus own spin torque has moment
`P×Ft_world` at the original wall force point. The time-PT spin connection is
removed when converting generalized endpoint loads to spatial wrench.
Independent instantaneous surface rates verify virtual work to 2e−13, and
finite Coulomb work is negative with the original maximum-dissipation gate.
The full physical/contact/dual Jacobian matches re-query central FD across all
20 physical/contact columns (step1e−6, gate2e−7); B and geometry are refreshed on every trial.

7/7 owned tests, 65/65 affected pressure/WallSurface/RelativeDirection/Friction
tests and source/test syntax checks pass. Raw nonlinear trace and test output
are in `composite-joint-wall-pressure-prototype-tests.txt`.

## Required next production integration and limitation

Use a new explicit WallRows mode with one physical record/site and slotCount1;
consume the existing original WallEnvelope with its normal manager freshness
checks. Preserve the old capsule/envelope laws unchanged. Friction manager
must use endpoint records, typed semantic IDs, global row offsets, accepted
map/frame/traction histories and fixed site signatures. Keep every original
gap in the final JointTimeStep proof, including capsule gaps without pressure
DOFs. Root then needs the actual two-tool flat two-dt JointTimeStep regression.

A concrete separate limitation remains: with nonzero feed, a current endpoint
material label may lie beyond this selected edge's previous label interval.
Existing SurfaceMotion correctly returns `surface-material-transport-required`.
Do not switch site.edge/trace with the feed direction or fabricate the missing
orientation. Supporting that case requires its own accepted orientation/path
history through a real hinge or tip. This prototype removes selected-minimum
pressure ownership jumps; it does not claim that hinge transport, arbitrary
curved-wall contact adaptation or the complete simulator is finished.

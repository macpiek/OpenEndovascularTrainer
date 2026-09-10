# Same-time composite state refinement

`refineCompositeState` transfers an existing composite state onto a refinement of its old mesh. It retains every old coordinate, both endpoints, all supplied exact boundaries, the same material maps, time and step. It never advances feed or integrates a physical timestep. A rejected transfer returns the original state object and commits no history.

The new code is limited to `src/physics/kirchhoffCompositeStateTransfer.js`, its test file, and this report. TimeStep, Chain, Mesh, Kinematics and contact modules are unchanged.

## Input and ownership

```js
const result = refineCompositeState(state, {
    coordinates,        // strictly increasing; contains every old node exactly
    boundaries,         // explicit exact boundary coordinates already in the old mesh
    targetTools,        // explicit newly integrated materialAt + dsDx, or constant compiled material
    inertiaEdges,       // prepared per-edge material maps, density and OLD MATERIAL velocities
    contactHistory,    // explicit persistent point records; [] when absent
    tolerances,        // all absolute criteria below are required
    elementBackend: 'wasm',
});
```

The accepted state owns its geometry, frames, spins, winding arrays, multipliers, velocity arrays and point-history payloads. Target material samples are copied into a cache for the new hinge supports. Material providers are read-only inputs and must be pure. Passing an old cached mesh provider on different supports produces an explicit failure; the module never falls back to an old cell's constitutive sample.

Recognized source-state fields are `data`, `layout`, `lengthMultipliers`, `time`, `step`, `materialVelocities`, `torsionMode` and `contactHistory`. Other history requires an explicit adapter and causes rejection, so an old edge-indexed buffer cannot silently survive with incorrect ownership. When `state.contactHistory` exists, pass that same accepted history list.

## Geometry, orientations and boundary convention

Each child restricts its parent edge's affine position field. No old node is removed or moved. Reference frames are copied from the parent, without a new spatial Bishop-frame capture. Each tool's edge spin is copied independently to every child. Original hinge winding anchors remain at their original coordinates; an inserted collinear hinge has zero reference jump. The report compares physical material directors and cumulative unwrapped phase, including the evaluated original reference winding.

This preserves the declared piecewise-constant edge orientation field, including its one-sided jumps at old hinges. It does not infer a smooth twist gradient from edge spin samples. The returned `orientation` and `sampleCompositeTransferredMaterial` make that field explicit at material labels.

`prolongation` contains sparse rows mapping original position/spin increments into the new DOFs. It is used for comparing virtual work and is not an automatic future boundary-condition policy. `oldNodeToNewNode` identifies retained position nodes. `spinDofMap` gives each original spin's `childDofs`, `proximalDof` and `distalDof`. A prescribed entire old edge field covers all children; a proximal handle convention selects the proximal child. The caller must choose the physical support it intends.

## Material velocity and angular fields

Every input tool uses Kinematics' existing `massPerMaterialLength`, affine `materialMap:{sStart,dsDx,dsDt}` and two `oldMaterialVelocities`. Density and `dsDx` are constant on each old edge; `dsDt` and physical old velocity are scalar/linear endpoint fields. Child fields are their restrictions in the same material labels. One-sided velocity jumps at original nodes remain distinct. Per-tool mass, momentum and kinetic energy are compared by Gauss-2 integration, exact for these declared fields; tests independently use analytic endpoint integrals.

Optional `angularKinematics` contains `thetaDt`, `thetaDx` and either `frameSpin:{dt,dx}` or the frame's material-path spin. Each scalar field is constant or linear through endpoint pairs. Optional `oldAngularVelocities` is an independent full 3-D endpoint field. Transfer preserves these declared inputs and does not invent their values from a same-time mesh change. Unknown angular data remains `null`.

The sample helper returns `kinematicsInput` for the existing Kinematics function. Its axial spin includes `thetaDt + u*thetaDx + frameSpin.dt + u*frameSpin.dx`, with `u=-dsDt/dsDx`; a supplied material-path frame spin is used directly. The caller supplies its actual translational derivatives to Kinematics. A known scalar axial spin does not establish a full bending angular velocity. Angular derivative fields remain attached to the accepted velocity history so another refinement can verify and preserve them; scalar endpoints alone cannot describe an arbitrary quadratic material-path spin.

## Length multipliers and contact history

Signed length multipliers represent axial tension. The same multiplier is copied onto every child; it is not divided by the number of children or their lengths. Independent comparisons check the original sum of `lambda*g` and the original force pulled back through the position prolongation. Tests also compare arbitrary virtual work.

Persistent contact records have `id`, `edge`, `fraction`, `owner`, `s`, `worldForce`, `worldCouple` and an owned opaque payload. A record is remapped once to its child edge. A foot exactly at an internal split belongs to the right child, except the original edge's distal endpoint, which stays on that edge's last child. Material label, world force, world couple and history are retained. Normal force is a point value and is never duplicated or split as a density.

`worldCouple` is the complete couple about the centerline foot, including any surface offset. Force pullback, foot location and world moment are checked. Rebuilding a contact's rotational Jacobian and adapting persistent records to the wall collector's slots remain caller responsibilities; a transverse couple is never silently projected onto the scalar tool spin. This transfer does not recertify contact geometry or complementarity.

## Error gates

Every absolute tolerance must be finite and nonnegative. There are no hidden tolerance floors or relative acceptance defaults.

| Tolerance | Comparison and units |
| --- | --- |
| `position` | Difference between old/new affine position fields and contact feet; length |
| `rotation` | Material-director angular discrepancy and cumulative unwrapped phase discrepancy; radians |
| `materialLabel` | Preserved map labels, continuity and accepted history labels; material length |
| `velocity` | Restricted and accepted old material velocities; length/time |
| `angularVelocity` | Full angular velocity, time derivatives, frame time spin and material-path axial spin; radians/time |
| `rotationGradient` | Explicit theta/frame spatial derivatives in the same x coordinate; radians/length |
| `mass` | Per-tool integrated mass |
| `momentum` | Per-tool vector momentum difference |
| `kineticEnergy` | Per-tool translational kinetic-energy difference |
| `energy` | Absolute independently assembled DER elastic-energy change |
| `force` | Maximum position-force norm of coarse versus pulled-back fine elastic gradient; inserted-node elastic detail forces; length/contact force pullback errors |
| `torque` | Independent spin-gradient changes, fine spin detail modes and point-wrench moment errors |
| `work` | Original length-multiplier work difference |

Independent Chain assemblies compare `E_old(q)` with `E_new(Pq)` and `g_old` with `P^T*g_new`. The additional inserted-position forces and complementary child-spin torque modes prevent a small coarse pullback alone from hiding large new elastic modes. Child-spin detail torque subtracts the length-weighted aggregate torque on its parent edge.

These are discretization-change measurements and sampled transfer-error estimates. They are not a bound on an arbitrary smooth underlying curve and do not claim exact DER energy after subdivision. Even when the affine polygon and all directors are preserved, the hinge support/quadrature changes. A curved mesh or nonuniform preform can therefore fail the caller's energy/force tolerance. Acceptance is a state-transfer decision, not proof of equilibrium on the new mesh. No kinetic-energy correction, artificial projection or force renormalization is used to make a transfer pass.

## Validation

The new suite passes 14/14 tests. Combined with frozen Chain, Kinematics, Length and Mesh dependencies, 62/62 pass. Coverage includes independent analytic mass/momentum/kinetic integrals, finite differences of `E_new(Pq)-E_old(q)` on every original DOF, independent material directors with large spin/winding, opposite tools, signed tension and virtual work, persistent contact wrenches, partial tool ownership, all exact boundaries, rollback and repeated same-time refinements with known angular history.

A steel-J/pigtail case rebuilds the actual Mesh dual-cell material providers and compares target energy with the independent JavaScript Chain backend. Both that case and a curved polygon reject under tight error tolerances. Test fixtures with relaxed elastic-error tolerances permit inspection of the measured change; they do not certify those tolerances for an application.

Coarsening, moving tips/material boundaries, physical history advection, angular inertia, contact solve, timestep integration and runtime/FPS optimization are outside this stage.

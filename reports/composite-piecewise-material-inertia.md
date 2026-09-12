# Exact piecewise material inertia on one unchanged physical element

`kirchhoffCompositePiecewiseMaterialInertia.js` adds a prepared factory compatible with the current material-inertia response. No Assembly, Step, Kinematics, MaterialInertia or MaterialHistory source was changed.

```js
const operator = createCompositePiecewiseMaterialInertiaEdge({
  coordinates: [x0, x1], previousPositions: [oldQ0, oldQ1], dt,
  tool: {
    id, massPerMaterialLength,
    materialMap: {sStart, dsDx, dsDt},
    oldVelocityPieces: [
      {fractions: [0, a], oldMaterialVelocities: [v0, vLeft]},
      {fractions: [a, 1], oldMaterialVelocities: [vRight, v1]}
    ]
  }
});
const response = operator.evaluate([currentQ0, currentQ1], {order: 'full'});
```

The `pieces` returned by MaterialHistory.prepare can be passed as `oldVelocityPieces`. Fractions must form an ordered literal covering of [0,1], with no gaps, overlaps or zero-width intervals. Each history piece owns two finite three-dimensional endpoint velocities; jumps at adjacent endpoints are allowed and preserved. All physical inputs are explicit: positive dt, constant positive density and dsDx, and scalar or affine endpoint dsDt. A simultaneous whole-edge `oldMaterialVelocities` pair is rejected as an ambiguous second history contract. There is no clamp, extrapolation, density default or second label advection.

Each history subinterval compiles the existing `createCompositeMaterialInertiaEdge` once. Its previous/current virtual endpoints are affine interpolations of the **original** two physical endpoints. Its dsDt is restricted from the original affine field, and its start material label is shifted by the original map. The fixed 2x2 endpoint interpolation T pulls force and full consistent Hessian back to the original six positional DOFs:

`E = sum E_piece`, `g = sum Tᵀg_piece`, `H = sum TᵀH_piece T`.

This partitions quadrature only. It adds no geometry nodes, material path, spins, constraints or nonlinear unknowns. The full physical convective and endpoint cross terms remain. Angular inertia is explicitly absent. The constructor prepares and sums H once; evaluate reuses compiled suboperators and scratch endpoints, requesting only their gradient evaluation. It never constructs new operators or factors in a trial. A full evaluation restores the original prepared H; gradient-only marks it invalid and leaves its bytes unused. Invalid trials invalidate the Hessian flag, and a subsequent valid retry recovers the unchanged prepared response.

The output uses the existing `energy`, `kineticEnergy`, `oldKineticEnergy`, `gradient`, `kineticGradient`, `hessian`, `momentumIncrement` and `tools[0]` mass/momentum contract. Physical momentum vectors add directly. All original quadrature sample diagnostics are retained; samples additionally carry `pieceIndex` and `localFraction`. Their `fraction` and velocity Jacobian `coefficients` refer to the **original** edge, not to additional unknowns. Map labels/rates, weights, old velocities, current velocities and increments remain explicit. Outputs are borrowed reusable buffers; source geometry, maps and old history are privately copied during preparation.

## Validation

Seven new tests pass; new plus original MaterialInertia tests are **13/13 PASS**, and the new helper passes syntax validation. Recorded output: `reports/composite-piecewise-material-inertia-tests.txt`.

- The independent discontinuity witness has old v=1 on material [0,1] and v=3 on [1,3], with current v=0, density1 and unchanged endpoints. Exact increment energy/old kinetic energy9.5 and old momentum7 are recovered on one six-DOF edge. Two whole-edge Gauss points instead give7.5. Input pieces come directly from MaterialHistory.prepare.
- An independent whole-original-edge polynomial integrator uses Simpson separately on each old history piece. It matches E, kinetic/old kinetic energies, both gradients, full6x6 H, mass, current/old momentum and momentum increment for nonunit dsDx, nonzero convection and affine dsDt.
- Independent energy/gradient finite differences recover every force row and all36 Hessian entries; the matrix remains symmetric to floating arithmetic precision and retains off-diagonal consistent inertia.
- One-piece scalar fields, arrays and original sample diagnostic fields are **bit-identical** to the existing MaterialInertia factory.
- Frame translations, rigid material boosts and rigid physical velocities preserve the expected velocity increments and mass/impulse relations.
- Prepared source ownership, output reuse, gradient/full validity, poisoned stale Hessian, invalid trial and retry, and malformed/missing physical inputs are covered.

No benchmark or full nonlinear fed-dt claim is made by this local operator. Root integration still needs to route piecewise histories through the factory and preserve their prepared values in Step. Accepted-history creation remains owned by the existing timestep. Frozen source/test/report and dependencies are in `/tmp/oet-composite-piecewise-material-inertia-final/manifest.json`; exact hashes are in `reports/composite-piecewise-material-inertia-source.json`.

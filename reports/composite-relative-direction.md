# One common / relative / constraint banded direction

Implemented only `src/physics/kirchhoffCompositeRelativeDirection.js` and its new test file. Existing Chain, TimeStep, MixedDirection, RelativeCluster and wall modules are unchanged by this task. This is an original linear-equation direction, not a nonlinear accepted time step or a clearance/contact certificate.

## Operator and API

`createCompositeRelativeDirectionWorkspace(layout, cluster, rowDefinitions = [])` freezes the index structure, mode nodes/bases, Hessian kind and local row supports. It does **not** cache values of the relative Hessian or common/relative coupling.

Each row definition is:

```js
{
  anchorNode: 4,
  commonDofs: Int32Array.from([layout.positions[4] + 1]),
  relativeDofs: Int32Array.from([0, 1]),
  unit: 'mm'
}
```

The ordered local support is the common indices followed by the relative indices. Support and anchor must fit within two edges, the existing wire hinge stencil. Common position/spin DOFs, both relative coordinates of each mode, and local dual rows are interleaved by node. The full nonsymmetric row band retains the required pivot fill using `createCoulombBandLU`. For a bounded number of local modes/rows per node, bandwidth stays bounded as the chain grows.

```js
solveCompositeRelativeDirection(workspace, chain, {
  cluster,              // CURRENT values, required on every direction
  commonResidual,      // original common physical stationarity
  relativeResidual,    // original relative physical stationarity
  fixed,               // zero/one mask of common coordinates
  rows: [{
    residual,
    jacobian,          // derivative of the original row equation
    forceColumn,       // signed physical derivative d(mechanical F)/d(lambda)
    multiplierDerivative: 0,
    geometricTangent,  // optional full local d(mechanical F)/d(primal)
    tolerance          // REQUIRED in that row definition's explicit unit
  }],
  tolerances: {force, torque},
  maxCorrections: 1    // allowed 0..2; each solve/factorization is reported
});
```

It assembles `[H_common, C; C^T, H_rho_rho]`, plus the caller's local dual rows and geometric tangent, exactly once. The input common Chain already contains the common wire contribution including its full physical inertia. `cluster.common` is diagnostic and is never read or added to this matrix. `additionalInertia` is likewise not added again. The supplied common/relative residuals already contain loads and current reactions; this solver never subtracts or adds multiplier loads a second time.

The geometry Jacobian and signed physical force column are separate mandatory arrays. An arbitrary prepared local row can therefore preserve a physical force convention distinct from the geometry derivative. Geometric tangents can be nonsymmetric and indefinite. The caller supplies valid branch values, reactions and derivatives; this module does not choose a disk normal at zero offset, infer active contacts, clamp a force, or reconstruct nonlinear geometry. Common and relative GN/exact Hessian kinds must agree.

Every direction freshly assembles and factors the original unshifted matrix. Optional residual corrections refactor the same matrix, without a pivot floor or regularization. A prescribed common coordinate has exactly zero increment. An undetermined multiplier row with no free geometric derivative and zero multiplier derivative holds its increment at zero; its original row incompatibility still fails the certificate. Both the pre-boundary original matrix and residual are retained for measurement. The band storage remains linear; no dense common response columns, per-disk elimination or global Schur complement exist.

The result returns common, relative and multiplier increments; original mixed linear residuals; original row residuals; separate common positional force, transverse relative force and spin torque residuals; per-row residual/tolerance/unit/pass; and counts of every factorization/backsolve. `tolerances.force` is the maximum absolute force-component gate for both common positions and orthonormal relative modes. Each constraint row has its own unchanged physical gate rather than a single dimensionless threshold.

Fixed-coordinate diagnostics are `fixedStationarity = F_fixed + A_fixed,* delta` and `fixedReactionIncrement = +A_fixed,* delta`. Both follow the physical force **applied by the boundary to the chain**: with `F = inertia gradient + elastic gradient - applied force - wall force`, the boundary force at a fixed coordinate is `R = +F_fixed`, and its increment is `+A_fixed,*delta`. The opposite sign describes the force of the chain on the handle. Typed output buffers belong to the workspace and are overwritten by its next solve. `certified:false` and `nonlinearStepAccepted:false` explicitly limit scope to a direction even when its original linear equations converge.

## Independent checks

The new tests use a curved three-dimensional two-material chain with anisotropy, intrinsic bend/twist, independent spins, unequal `dsDx`, and full physical convective edge inertia. Its constitutive/mass inputs are explicit manufactured controls, not measured Glidewire/Berenstein parameters. The dense test oracle independently assembles the natural ordering `[all common, all rho, all dual]` and uses ordinary partial-pivot Gaussian elimination. It does not use the production permutation, band scatter, LU, scaling or residual code. Every original matrix entry, increment and fixed reaction is compared.

The witness file `reports/composite-relative-direction-witness.json` records:

| Case | Unknowns | Half-bandwidth | Max increment error vs dense | Original force error | Original torque error |
| --- | ---: | ---: | ---: | ---: | ---: |
| GN, 11 nodes / 3 modes / 2 mixed rows | 61 | 21 | 3.77e-15 | 6.10e-15 | 3.02e-16 |
| Exact, same prepared state/support | 61 | 21 | 3.11e-15 | 1.30e-14 | 5.05e-16 |

Both require one fresh factorization/backsolve and pass force `1e-9`, torque `1e-10`, and the separate row gates `1e-11 mm` / `3e-12 N s`. These rows deliberately exercise different equation units; the second is a manufactured algebraic control, not a newly proposed physical constraint law.

Negative checks confirm that poisoning unused common diagnostic bytes does not affect the solution, intentionally adding that common block again changes the solution, and removing neighbor/second-neighbor relative blocks gives a wrong direction. In the cross-mode witness, omission changes the increment by `0.08118` and leaves original relative force residual `13.0253`. It is therefore insufficient to solve each mode independently.

Further checks cover fresh H/C values in a reused workspace; changed CSR topology/basis rejection; separate signed force columns; fixed coordinates mixed with relative/spin support; full nonsymmetric/indefinite geometric tangent; release rows with an explicit dual derivative; incompatible fully prescribed constraints; stale/nonfinite operators; singular rejection; and unattainable original tolerances. The latter remains `converged:false` after exactly three allowed unshifted factorizations, with no altered threshold.

The reaction sign is independently verified using two uniform linear elements with fixed outer nodes and an axial load `+3.2` on the middle node, starting from zero velocity. Each element contains the complete consistent translational mass of the two materials (densities `2.4` and `4.1`, length `2`); `dt = 0.25`. Integrating the resulting piecewise linear velocity field directly gives total momentum rate `4.8`. The force of each boundary on the chain is `+0.8`, so `momentumRate = appliedLoad + leftReaction + rightReaction = 3.2 + 0.8 + 0.8`. The impulse is `1.2`. Reversing the reaction signs predicts `1.6` momentum rate and fails that balance. Adding an existing external load `+0.3` at the prescribed left endpoint leaves its reaction increment `+0.8` unchanged but changes its total boundary force to `+0.5`; the balance remains `4.8 = 3.2 + 0.3 + 0.5 + 0.8`. This uses a physical velocity integral, not a dense matrix oracle with an assumed sign. The earlier handoff had the force-on-handle sign under the force-on-chain name; the revised bundle corrects that result and its documentation without changing the solved direction.

| Nodes | Modes | Common / rho / dual unknowns | Total | Half-bandwidth | Matrix entries | LU factor entries |
| --- | ---: | --- | ---: | ---: | ---: | ---: |
| 25 | 21 | 123 / 42 / 21 | 186 | 22 | 7,864 | 12,462 |
| 65 | 61 | 323 / 122 / 61 | 506 | 22 | 22,264 | 33,902 |
| 201 | 197 | 1,003 / 394 / 197 | 1,594 | 22 | 71,224 | 106,798 |

All three storage controls pass their original equations with one factorization each. At 201 nodes the maximum common/relative force errors are `1.46e-14` / `9.71e-15`, torque `4.14e-14`, and row residual `9.29e-18`. Two matrix buffers retain original and boundary-conditioned equations; both have the same linear storage bound. No FPS or full-step runtime claim follows from these algebraic controls.

## Verification and handoff

New RelativeDirection tests: **11/11 PASS**. Combined RelativeDirection + RelativeCluster + existing MixedDirection: **28/28 PASS**. Syntax check passes. The revised frozen bundle and dependency/source hashes are in `/tmp/oet-composite-relative-direction-reaction-sign-final/manifest.json`; its `tests.txt` contains the final combined test output. The prior `/tmp/oet-composite-relative-direction-final/manifest.json` is preserved unchanged as handoff history. No runtime integration or change to `npm test` is included in this revision.

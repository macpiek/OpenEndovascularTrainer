# Shared catheter–guidewire contact block

All active lumen normal contacts are assembled on the same frozen geometry.
Their interpolation stencils contribute to one unilateral system, including
smooth side quadrature, the fractional entering cell and the sliding mouth.
An impulse changes both tools with opposite gradients and their respective
inverse masses. No contact is selected as a representative or discarded.

The first contact block of each step includes both complete rods' constrained
mobility, accumulating all contact impulses before each rod response. Later
contact sweeps use the less expensive local mobility. The runtime uses block
Gauss–Seidel: the complete direct Kirchhoff solves of
both tools alternate with the simultaneous normal contact block. Axial and
circumferential Coulomb friction and torsional friction are projected after the
normal block with the updated load bounds. This is **not one monolithic
factorization of material, normal, tangential and twist equations**. Subsequent local contact subproblems also receive global material response
through the outer block iteration.

## Numerical implementation

For contact Jacobian C, inverse generalized mass W, physical compliance alpha,
and accumulated normal multiplier lambda, the local increment solves

    (C W C^T + alpha + rho D) delta = -gap - alpha lambda
    lambda + delta >= 0

with the complementary inequality on unloaded constraints. D is the diagonal
of the unshifted operator and rho is 0.001. The shift damps only the increment;
it is absent from the final physical contact residual. Projected conjugate
gradients use a banded Cholesky preconditioner restricted to the current active
set. A feasible projected step may release several bounds together, but must
reduce the quadratic objective. Otherwise the solver uses the first-bound step.
The previous active set is a hint only: no force is retained across physical
steps. New or released contacts still pass the same complementarity checks.
Predictor contact blocks use a looser linear tolerance (0.02 mm at the default
settings); final closure blocks use 0.0002 mm and retain the 0.001 mm physical
acceptance gate. Buffers are retained between sweeps. No dense inverse is formed.

Whole-rod force propagation remains in every outer material iteration. The
existing contact-coordinate motion criterion ignores distant free-wire motion
and admissible sliding, while checking constrained normal and sticking motion.
Both rods' complete length errors are still checked. The outer acceptance
limits remain 0.001 mm contact residual and 0.2% relative segment-length error;
the physical timestep remains 1/120 s. Spatial sampling, EI/GJ, friction and
radii are unchanged. No simulation time is dropped to improve frame rate.

The scalar per-contact full-rod response path was removed from the world.
Float64 banded factorization and triangular solves now use a small embedded
WebAssembly kernel with fixed-capacity workspaces. Both rod and contact solves
use it. Geometry coefficients and conservative cubic bounds are rebuilt once
per frozen contact sweep and once per read-only residual scan.

## Kernel source

The human-readable source is `scripts/physics/kirchhoffLinearKernel.wat`.
The generated `src/physics/kirchhoffLinearKernelBytes.js` is committed as source
input to Vite; ordinary development/builds need no compiler or network fetch.
To regenerate with the optional build-time compiler:

```sh
npm install --prefix /tmp/oet-wasm-tools --no-save --ignore-scripts wabt@1.0.37
OET_WABT_PATH=/tmp/oet-wasm-tools/node_modules/wabt/index.js node scripts/physics/build-linear-kernel.mjs
```

## Validation

- Dense active-set enumeration checks the whole unilateral system, including
  load release and unequal tool masses, for both local and material-constrained
  mobility.
- Aggregate full-rod mobility agrees with the sum of independent sparse loads.
- Masked and unmasked WebAssembly band solves agree with independent dense
  Gaussian elimination for multiple bandwidths.
- Frozen-geometry search matches the independent exhaustive spline reference
  on 1600 queries, and cached/uncached results agree exactly across changing
  straight, curved, folded and coincident geometries.
- Contact convergence tests verify reciprocal momentum and a reaction 80 mm
  away from the short overlap, including a one-sweep inner contact budget.
- Full over-wire feeding regressions pass for Berenstein, pigtail and SIM1.

The analytical two-rod containment fixture explicitly requests a contact
stopping tolerance tighter than its existing 0.00002 mm assertion. This avoids
relying on the previous solver's incidental over-solving; its geometric and
momentum assertions were not relaxed. The application retains its original
0.001 mm stopping tolerance.

Optimization, direct rod mechanics, all three full over-wire regressions,
18 coupled material/control combinations, length closure, containment and world
integration pass. Production build and generated API documentation checks pass.
The full `npm test` still stops at the pre-existing guidewire wall-to-momentum
regression: 1.230252692423265 mm/s against a 1 mm/s bound. The value is unchanged
from the deep-insertion baseline; the full suite is not green.

## Browser measurement (2026-09-06)

Debug → **Cewnik 10–60 cm**, one visible Codex in-app browser tab, fluoroscopy,
no concurrent Node tests. Parameters: Glidewire shaft/tip 10/4.55, Berenstein
25/5, both relaxation rates 1. Baseline and new run execute the identical 8553
steps (71.275 seconds of physics). Both tools' fingerprints match exactly at
the end of all three uncoupled preparation phases. Coupled trajectories are
not bit-identical. These are single full replays on the same host.

| Depth | Feed FPS before → after | Held FPS before → after | Held step before → after | Held outer passes before → after |
| --- | --- | --- | --- | --- |
| 100 mm | 59.7 → 60.0 | 58.9 → 60.0 | 15.42 → 6.19 ms | 20.16 → 6.26 |
| 200 mm | 24.3 → 60.0 | 17.5 → 60.0 | 54.12 → 8.79 ms | 62.38 → 5.97 |
| 400 mm | 13.9 → 59.6 | 11.0 → 59.9 | 87.06 → 12.51 ms | 46.03 → 4.02 |
| 600 mm | 8.4 → 32.6 | 6.5 → 45.3 | 150.01 → 20.11 ms | 59.75 → 4.77 |


At 600 mm, rotation gives 50.9 / 52.2 FPS (previously 7.62 / 7.45), and the final
settling interval gives 48.3 FPS. All **4866 coupled steps converge**. Final
contact count is 479. At held 600 mm, mean physics cost falls by 86.6%; the
outer solve drops from 59.75 to 4.77 passes. Per-phase source fields are in
[the browser JSON](catheter-contact-block-browser.json); the original baseline
remains in [its own report](deep-catheter-performance-browser.json).

The full replay takes 478.71 → 121.07 seconds. Overall mean FPS is 14.45 → 53.30,
and 1% low is 5.05 → 22.92 FPS. **This does not yet meet the 60 FPS target at
600 mm**, especially while feeding. Simulation backlog remains 49.79 seconds
(previously 407.44), with zero dropped steps. Approximately 60 FPS at 200–400 mm
also does not imply real-time 120 Hz physics: backlog grows from 4.15 seconds
before catheter feed to 18.48 seconds after the held 400 mm phase.

During coupled phases, maximum relative segment error stays below the 0.2%
acceptance bound, and physical contact residual stays below 0.001 mm (values
round to those limits in the selected JSON). Raw lumen penetration reaches
0.07295 mm during fractional-cell entry and 0.00243 mm while held; these raw
geometric values include the existing partial-overlap compliance and must not
be confused with the compliant solver residual. Maximum coupled vessel-wall
penetration is 0.14081 mm. The 4.37438 mm all-run wall maximum and the initial
full-wire convergence failures occur before catheter feed and match the
unchanged preparation fingerprints. This change does not fix that separate
wire-only transient.

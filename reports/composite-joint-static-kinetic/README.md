# Static and kinetic wall friction in the common step

The common Joint step now accepts `law: coulomb-static-kinetic` with separate static and kinetic tangent-axis pairs, either common or per physical wall owner. The World source adapter reads both original body coefficients; the guidewire kinetic alias does not replace its static coefficient. The simulator still selects the earlier solver.

A sample entering at rest first solves the common mechanical/contact system with its static cone. Only a converged whole-step root showing resolved slip can trigger breakaway. All necessary branch changes are batched, then the same prepared dt is solved with kinetic coefficients, preserving inertia, material history, force transfer and cumulative numerical budgets. A failed numerical iteration never counts as physical breakaway. Each sample can demote once per dt. No numerical-penalty threshold or friction blending is introduced.

Accepted zero slip, or strict cone-interior feasibility within the original physical residual, permits the next same-material contact to try static friction. An unresolved small slip at the cone boundary rejects the dt. Disabled tangent axes remain free. Equal static/kinetic coefficients reproduce the original Coulomb states, reactions and solver counts.

## Validation

`npm run test:physics:composite`: **779/779 PASS**, 16.391 s. `npm run build`: PASS, 1.47 s, with the existing bundle-size warning. Logs and source hashes accompany this report. This does not claim the entire historical `npm test` or browser performance passed.

- Whole-step holding between kinetic/static limits, breakaway, continued kinetic sliding, stopping and regained static holding range.
- Same physical root at numerical penalties 5/50/500; late direction/evaluation/query rejection preserves the accepted state and retry matches cold execution.
- Wall static/kinetic and catheter-lumen Coulomb share one two-tool solve, with independent material histories and force balances. The sliding state agrees with a separately configured kinetic-Coulomb solve. A second dt and atomic retry pass.
- Actual World source values 0.006/0.002 are retained. Initial translation/angular motion cannot be silently treated as rest, and a pending source-rest mutation invalidates its preparation.

## Remaining scope

Mode history is attached to an exact material label. A changed label must provide explicit incoming physical tangential surface velocity; it cannot inherit the old spatial sample's stop. The World adapter currently derives this input only for an initial exact-rest state, checking prepared old material velocities and actual body angular velocities. General initially moving sources, feed across labels and angular/surface history transport remain unfinished. Finite-step slip mode history is not a complete angular-velocity model.

Full anatomy with loaded friction, sheath/portal/tip/external source laws, moving material profiles, active-range/mesh transfer, error-controlled relative-coordinate reduction, app integration and deep/max insertion performance remain open. There is no new 60 FPS result. The active goal remains unfinished.

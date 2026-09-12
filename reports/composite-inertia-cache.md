# Compiled affine material inertia

The new `src/physics/kirchhoffCompositeInertiaCache.js` compiles the original Kinematics operator once for a prepared, fixed-layout dt. It does not modify Kinematics, TimeStep, Chain, material profiles, timestep acceptance, or history commits.

`createCompositeInertiaCache({layout, coordinates, previousPositions, dt, inertiaEdges})` calls original Gauss-2 assembly and scatter during compilation, freezing independent maps, mass densities, and old physical material velocities into owned packed arrays. `compiled.append(positions, chain)` runs after elastic assembly and before constraints. It adds the original complete convective inertia Hessian, with its off-diagonal entries and original accumulation order, plus current energy and gradient. There is no mass lumping, diagonal replacement, angular inertia, material history update, remesh, or implicit admission.

For each sample, the changing velocity is affine about the previous accepted positions. The update is algebraically `a0*dq0+a1*dq1`, evaluated as `(N0/dt)*dq0+(N1/dt)*dq1+(u/L)*(dq1-dq0)`. Grouping the convective difference avoids subtracting two large opposite transport terms under rigid motion. The original `a0/a1` coefficients provide the gradient and exact Hessian. Incremental energy is the positive sum `0.5*weight*(incrementOrigin+shift)^2`; it never subtracts large kinetic energies. Kinetic energy is independently summed from physical material velocities. Every trial keeps separate wire/catheter contributions on their one shared position field.

The layout is borrowed read-only and must retain identity; the maps, coordinates, mass values, prior positions and prior material velocities are copied during compilation. Returned diagnostic arrays and the result object are reused. The only changing physical input in the hot loop is current positions. Compilation validates material ownership and complete band storage through the original scatter. Trial output finite guards remain enabled. A new dt, maps, history, or topology requires a new compilation. This module performs no acceptance or state transfer.

## Verification

42/42 tests passed: the existing Kinematics, Chain and numerical-refinement suites plus 8 new cache tests. The new tests compare the original Kinematics energy, full gradient, physical velocities, kinetic gradient/energy, and every full-band Hessian entry for a 3D chain with wire-only, overlap and catheter-only sections, independent unequal `dsDx`, linearly varying opposite `dsDt`, and different old material velocities. The complete Hessian is byte identical. Independent finite differences retain the original `2e-8` tolerance for energy/kinetic gradients and all Hessian rows; response parity uses `1e-11`.

Additional controls cover rigid frame transformation and velocity boost, zero velocity, steady opposite feed, negative convective off-diagonal terms, a genuine inertial null mode without floors, large transport velocity with a small positive incremental energy, poisoned/reused output buffers, frozen prepared inputs, unchanged spin rows, ownership and nonfinite failures. Source and benchmark syntax checks passed.

## Short paired performance probe

`scripts/physics/benchmark-composite-inertia-cache.mjs` uses actual nominal Glidewire/Berenstein profiles, wire318mm/catheter310mm, a fixed 3D common centerline with every exact material/tip boundary, and 65 or 128 nodes. Body model mass conventions are wire `1/5` and catheter `1.4/4` model units/mm, not measured kg. Both materials have explicit opposite affine map rates. There is no feed/remesh simulation. Both arms reuse workspaces, alternate execution order, and receive identical prepared and trial hashes. There are four warmups and eight measured pairs per mode. Whole operator measurements include elastic assembly, full consistent inertia scatter, factorization/backsolve and the original residual certificate at `5e-10`; they do not include nonlinear length/contact iterations or timestep acceptance.

Frozen final run (`/tmp/oet-composite-inertia-cache-final.json`, copied to `reports/composite-inertia-cache.json`):

| Nodes | Original inertia median | Compiled append median | Original elastic+inertia+direction | With compiled append | Warm compile median | Cold compile |
|---|---:|---:|---:|---:|---:|---:|
| 65 | 0.623ms | 0.141ms | 1.773ms | 0.964ms | 1.045ms | 4.603ms |
| 128 | 0.927ms | 0.160ms | 2.426ms | 1.371ms | 1.080ms | 7.317ms |

Inertia append is 4.40×/5.78× faster; the measured elastic+inertia+direction operator is 1.84×/1.77× faster. All 16 measured direction pairs converged in both arms. The maximum full-gradient difference is `8.527e-14`, direction difference `3.904e-18`, and original residual `1.080e-12`. All 32 measured matrix pairs are byte identical, prepared inputs remain unchanged, and before/after source hashes match.

Compilation is an additional once-per-dt cost: this optimization amortizes after roughly 2–3 inertia evaluations in the measured warm cases. A timestep using only one evaluation may become slower; the first cold compile can exceed the desired total frame budget. The probe intentionally records this cost and its allocations instead of treating it as free. Three short runs showed substantial host/JIT noise: inertia speedup ranged4.4–6.9× at65 nodes and5.3–5.8× at128 nodes. These data establish a repeatable reduction in the diagnosed repeated-inertia bottleneck, not a full-dt or 60FPS certificate. Full-step integration and warm/first-dt budgets still require the parent solver's benchmark.

Frozen handoff: `/tmp/oet-composite-inertia-cache-final/manifest.json`. Original Kinematics and the parent solver sources were not edited.

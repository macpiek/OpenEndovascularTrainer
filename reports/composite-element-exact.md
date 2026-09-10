# Exact elastic Hessian candidate

`kirchhoffCompositeElementExact.js` is an explicit alternative backend. Existing JS and Fast/GN sources and generators are unchanged. Chain adds only `elementBackend:'wasm-exact'`; the default remains `'wasm'` (GN), with `'javascript'` retained as the independent energy/gradient oracle.

The full Hessian is `H = sum_tools L*(Jᵀ K J + sum_a moment_a * Hessian(strain_a))`, including every spatial-position and independent material-spin derivative of time-transported directors, Darboux curvature and anchored reference twist. Anisotropic bending/torsion cross terms, `dsDx`, intrinsic strain, energyOffset and winding semantics are unchanged. Winding is constant only within its selected continuous branch. The energy and exact gradient match the original oracle. The Hessian is symmetric and can be indefinite; there is no PSD clamp, diagonal floor, shift, or runtime finite difference. It belongs in the general mixed band LU. Existing Cholesky may correctly reject it.

The separate build generator differentiates the fixed first-derivative DAG at build time, reuses common expressions and removes identically zero derivatives. The generated kernel has 605245bytes, 27865/32708 symbolic nodes for one/two tools, and 810/947 live WASM locals after build-time last-use coloring. A first implementation retained a local for every DAG node; coloring shortened the kernel from690502bytes and greatly reduced its cold execution/tiering overhead. Values, derivatives and complete Hessian pass the same tests after coloring. This does not eliminate first-use/JIT variability.

## Verification

34/34 tests passed across new Exact (8), existing Fast, Chain and ChainRefinement suites. Syntax checks passed for wrapper, Chain and generator.

Every full local Hessian entry is compared against independent central differences of the ORIGINAL JS exact gradient at the unchanged `2e-8` tolerance, for one/two materials in 3D with unequal material scales, linear-independent spins, full anisotropic bending/torsion coupling, nonzero intrinsic strain and multiple winding anchors. The full mixed-section Chain band is independently checked the same way. Tests also cover bitwise Hessian symmetry, correct rigid transformation of all q/spin blocks, translational null directions, independent material spins, continuous atan2 branch crossing under an explicit anchor, zero-moment reduction to GN, constant energy offsets, immutable inputs/reused outputs and finite/degenerate-geometry guards.

A simple prestressed straight element has a certified direction with `dᵀHd=-0.25`; GN is nonnegative on that direction. Independent energy differences confirm the negative curvature. The backend does not conceal this physical prestress term.

## Immutable experiment

Final snapshot: `/tmp/oet-composite-exact-snapshot-iBYhkf/manifest.json`.
Final results: `reports/composite-element-exact.json`, copied from `/tmp/oet-composite-element-exact-final.json`.
All executed source bytes are sealed in the snapshot; before/capture and after-run hashes match. No root source changed during this run.

The assembly microfixture has65 nodes/323DOFs/band13, one shared 3D centerline, two independently spinning declared constant materials, and identical inputs in each alternating pair. It includes every elastic element and full band scatter, but no inertia or direction. Six initial warmups, eight measured pairs and a further eight pairs after the timestep experiment retain cold/tiering and later warm measurements separately:

| Assembly65 phase | Existing GN median | Exact median |
|---|---:|---:|
| Initial measured series | see JSON | 2.784ms |
| After timestep experiment | 0.807ms | 1.850ms |

Exact assembly is more expensive than GN; its benefit depends on reducing nonlinear iterations. The later warm exact samples range1.249–8.111ms, showing host/JIT noise and occasional slow samples. First workspace construction is also recorded (GN1.331ms/Exact3.179ms). The initial pre-coloring snapshot remains under `/tmp/oet-composite-element-exact-initial.json` and records large cold/tiering costs. A separate run overlapping the parent's full composite test suite suffered pathological multi-second timings in both unchanged GN and Exact; it is retained under `/tmp/oet-composite-element-exact-colored.json` and is not used to claim a speedup.

The complete-step comparison uses actual nominal Glidewire/Berenstein profiles at wire318/catheter310mm, the original exact material/tip boundaries, Body model mass-density conventions, 120Hz dt, and the current mixed length/wall LU. It covers contact-free and explicit frictionless analytic-plane controls, each with two paired repeats and an initial plus consecutive loaded step. There is no anatomical curve transfer, remesh or feed. Both arms always receive the SAME prepared state and input hash. The accepted GN result supplies the common next state for both arms, avoiding different trajectories in the consecutive comparison. Prepared input construction is outside the timed `advanceCompositeTimeStep` call; all allocation, assembly, direction, line search, certification and commit inside that call are timed.

Tolerances are unchanged from the original TimeStep/Wall tests: force1e−7, torque1e−8, length1e−8, linear5e−10 contact-free and1e−10 wall; wall gap1e−8, force/work1e−7. Budgets are unchanged. Every result and certificate is retained; failed attempts would remain failures with zero committed time/history. No attempt in this paired experiment was rejected.

| Scenario at310mm catheter | GN directions, initial/consecutive | Exact directions | GN assemblies | Exact assemblies | GN median full call | Exact median full call |
|---|---:|---:|---:|---:|---:|---:|
| Contact-free | 10 / 17 | 4 / 4 | 13 / 19 | 6 / 6 | 64.302ms | 33.344ms |
| Analytic frictionless plane | 12 / 10 | 4 / 4 | 14 / 12 | 6 / 6 | 55.773ms | 36.607ms |

All16 full calls were accepted:8 per backend. Every accepted call committed one dt and one history update; every original state stayed unchanged. The maximum paired accepted position difference is1.007e−11mm and active material-angle difference1.673e−10rad. All8 pairs have identical prepared/original hashes. The direct mixed solver requires only4 factorizations/backsolves per exact step in these cases.

This is evidence that the omitted strain curvature caused many extra iterations and that exact H reduces total cost in these bounded examples. It does NOT meet mean4ms/P956ms,120Hz/60FPS, or establish performance under anatomy, sliding friction or changing topology. Persistent TimeStep/Mixed workspaces and the remaining full-step costs are still parent integration work. The exact backend remains opt-in.

Frozen handoff manifest: `/tmp/oet-composite-element-exact-final/manifest.json`.

# Reusable common-chain timestep workspace

The solver now accepts an explicit `workspace` created by `createCompositeTimeStepWorkspace(layout, {elementBackend})`. It reuses the Chain/length/inertia-oracle/contact buffers and a bounded LRU of mixed-system storage (default two layouts, maximum four). Numerical matrices and factors are always rebuilt. Geometry, material maps, prior velocities and physical multipliers are not cached between prepared steps. A changed topology/backend requires another workspace.

The handle is opaque; nested calls are rejected and the busy flag is released on every success, rejection and thrown argument error. Accepted output owns its positions, spins, frames, multipliers, diagnostics and layout. Rejected trials restore the base wall multipliers before applying another line-search scale, including multipliers redistributed by exact contact-row canonicalization.

## Verification

213/213 integrated composite tests pass. Nine new workspace tests compare all physical state and certificate values exactly against fresh scratch for GN and Exact, contact-free and analytic-wall cases; cover consecutive steps, rejected-step retry, output ownership, invalid/reentrant calls and bounded storage under changing wall rows. The Exact and SurfaceMotion handoffs were imported only after their frozen SHA256 manifests matched. SurfaceMotion remains an isolated operator with its explicit reconstruction limitations.

## Paired full-step probe

The benchmark uses actual Glidewire/Berenstein profiles at wire318mm and catheter9/160/310mm, fixed topology, independent prepared material inertia and unchanged strict tolerances. Each arm receives identical state/inputs; outputs, original certificates and every iteration/backsolve count match exactly. One cold pair plus three warmup steps precede eight measured consecutive steps. Workspace creation is timed separately and is not free. There is no feed/remesh, finite-clearance or friction model and no browser FPS claim.

| Backend | Wall | Catheter mm | Fresh median ms | Reused median ms | Saved |
|---|---|---:|---:|---:|---:|
| wasm | contact-free | 9 | 3.177 | 2.909 | 8.4% |
| wasm | contact-free | 160 | 4.072 | 3.566 | 12.4% |
| wasm | contact-free | 310 | 8.650 | 8.303 | 4.0% |
| wasm | analytic-plane | 9 | 6.151 | 5.441 | 11.5% |
| wasm | analytic-plane | 160 | rejected step 3 | same rejection | — |
| wasm | analytic-plane | 310 | 25.571 | 24.909 | 2.6% |
| wasm-exact | contact-free | 9 | 3.746 | 3.287 | 12.3% |
| wasm-exact | contact-free | 160 | 4.786 | 4.345 | 9.2% |
| wasm-exact | contact-free | 310 | 6.684 | 6.059 | 9.4% |
| wasm-exact | analytic-plane | 9 | 6.239 | 5.566 | 10.8% |
| wasm-exact | analytic-plane | 160 | 8.455 | 8.007 | 5.3% |
| wasm-exact | analytic-plane | 310 | 8.907 | 8.460 | 5.0% |

**Open convergence failure:** GN on the analytic wall with catheter160mm rejects the third physical step after the unchanged32-direction budget. Force residual1.704e-4 and torque5.211e-6 exceed their gates even though wall and length pass. Both fresh and reusable arms reject identically, with no state/time commit. This is recorded, not dropped from the dataset. Exact accepts all12 consecutive steps in each of its six scenarios with unchanged gates.

Reuse is a measurable secondary improvement; most excess cost is still nonlinear iteration and repeated constitutive assembly. The 8.46ms warm Exact median at the deep analytic-wall case still exceeds the goal, and the eight samples are not a robust P95 study. The lower absolute times than earlier isolated probes also reflect JIT/host variation; only paired arms from this run support the reuse speedup.

Source hashes before and after match for the entire relative import graph. Raw samples, failure, construction costs, exact input/output hashes and original certificates are in [composite-timestep-workspace.json](composite-timestep-workspace.json). Reproduce with `node scripts/physics/benchmark-composite-timestep-workspace.mjs`.

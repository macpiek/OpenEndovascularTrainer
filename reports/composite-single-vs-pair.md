# Identical-grid single-wire versus composite comparison

The user’s expectation is supported at the operator level: one shared axis with the additional catheter material has a moderate cost increase. The excessive full-step penalty comes mainly from more nonlinear evaluations/directions, rather than duplicating the entire rod system. The current new TimeStep also has a slow single-wire baseline and cannot be equated with the existing browser’s observed60FPS.

Executed dependency graph: `/tmp/oet-composite-exact-snapshot-iBYhkf/manifest.json`, the frozen TimeStep before the parent’s persistent-workspace changes. All files still match the sealed manifest. No TimeStep, Chain, Mixed, material, or existing benchmark source was edited. New harness/helper hashes were stable during the run.

## Fairness and scope

For each grid originally prepared for wire318mm/catheter9,160,310mm, the single-wire fixture is formed by removing ONLY the catheter material/spin/mass arm. Every spatial node, exact material/tip boundary, coordinate, reference frame and initial position is retained. The wire material provider, angles, winding history, independent material maps/rates, old material velocities and mass remain identical. Positional loads and prescribed positions, wire spin boundary,120Hz dt, strict original tolerances and budgets match. The catheter arm necessarily adds its own explicit spin boundary. Normalized common inputs are asserted identical and hashed. Both arms start from zero length multipliers.

Each of the four variants (single/pair × GN/Exact) repeats the SAME original straight physical input three times. The first call and two subsequent warm calls are recorded separately. These are JIT/cache warm repeats, not a settled physical trajectory or a sequence with artificially shared accepted histories. All36 full calls accepted and committed exactly one dt/history; every original input/state stayed unchanged. No failure is counted as a completed dt.

The isolated operator benchmark reuses its elastic, compiled inertia, length and mixed-direction workspaces, with three warmups and six measured alternating samples. It measures elastic assembly, full consistent inertia, original length rows and one unshifted mixed LU direction separately. All72 measured directions converged. A direction is not a timestep certificate. The complete TimeStep calls include their own setup, material freezing, inertia compilation, repeated assembly, line search, original certification and commit. Fixture/profile/mesh construction is separately reported.

Nominal real Glidewire/Berenstein profiles and existing Body mass conventions are used: wire1/5 and catheter1.4/4 model units/mm, not measured kg. Force1e−7, torque1e−8, length1e−8 and linear5e−10 are unchanged. There is no anatomy, lumen/clearance, sliding friction, feed, remesh, state transfer, browser rendering or FPS certification.

## Structural cost

| Grid from catheter depth | Spatial nodes, both arms | Independent spins single→pair | Primal unknowns single→pair | Mixed unknowns single→pair | Primal band | Mixed half-band |
|---|---:|---:|---:|---:|---:|---:|
| 9mm | 67 | 66→69 | 267→270 | 333→336 | 11→13 | 13→15 |
| 160mm | 69 | 68→103 | 275→310 | 343→378 | 11→13 | 13→15 |
| 310mm | 68 | 67→132 | 271→336 | 338→403 | 11→13 | 13→15 |

At310mm, adding65 catheter spin unknowns raises the primal size24% and mixed size19%; the shared position field remains204DOFs in both arms. The fixed rectangular band widens when overlap first appears. That storage/factor overhead is a property of this implementation’s global band, not a physical requirement to redo a second complete centerline.

## Measured repeated operator cost

All times below are medians in milliseconds. Operator includes elastic, inertia and original length rows; direction is a separate mixed matrix/factor/backsolve operation.

| Depth | Backend | Elastic single→pair | Operator single→pair | Pair/single operator | Direction single→pair |
|---|---|---:|---:|---:|---:|
| 9mm | GN | 0.659→0.972 | 0.802→1.072 | 1.34× | 0.680→0.792 |
| 9mm | Exact | 1.277→1.554 | 1.383→1.658 | 1.20× | 0.728→0.895 |
| 160mm | GN | 0.332→0.431 | 0.397→0.515 | 1.30× | 0.536→0.588 |
| 160mm | Exact | 0.908→1.101 | 1.043→1.196 | 1.15× | 0.542→0.699 |
| 310mm | GN | 0.533→0.835 | 0.639→0.954 | 1.49× | 0.572→0.810 |
| 310mm | Exact | 1.524→1.925 | 1.634→2.070 | 1.27× | 0.563→0.785 |

The additional material costs only1.30–1.49× for the GN operator and1.15–1.27× for Exact in this run. At310mm the GN mixed direction rises1.42×. These modest ratios support the unified model’s structure. Extra profile contributions, independent spins and increased local bandwidth remain real costs; an order-of-magnitude operator penalty is not inherent here.

## Complete prepared timestep calls

| Depth | Backend | Directions single→pair | Assemblies single→pair | First full call single→pair | Warm full call single→pair |
|---|---|---:|---:|---:|---:|
| 9mm | GN | 1→2 | 3→4 | 46.365→33.895ms | 13.171→14.146ms |
| 9mm | Exact | 1→2 | 3→4 | 34.272→24.219ms | 14.857→17.414ms |
| 160mm | GN | 1→4 | 3→6 | 7.086→37.035ms | 4.485→15.762ms |
| 160mm | Exact | 1→3 | 3→5 | 9.393→21.505ms | 12.088→18.701ms |
| 310mm | GN | 1→10 | 3→13 | 10.116→27.762ms | 10.989→28.513ms |
| 310mm | Exact | 1→4 | 3→6 | 12.998→22.491ms | 8.551→25.320ms |

The single wire needs one Newton direction in every case. At310mm the coupled GN model requires ten; Exact reduces this to four, at a greater per-assembly cost. The real catheter profile has nonzero intrinsic curvature, so its initial straight pose carries prestress absent from the straight wire. Some additional nonlinear work is physical; the10→4 reduction at identical tolerances shows that much of the GN work is the missing strain-curvature tangent.

Warm full calls use only two samples; large JIT/host variance is visible in the JSON and the9mm cases run first in the process. They do not establish P95 or a steady runtime frame rate. In particular, a warm single-wire Exact value below GN at310mm is noise, not evidence that Exact is intrinsically cheaper: the repeated operator measurement shows the opposite.

## Separate setup control

A second short probe uses the same inputs and tolerances with an explicit `evaluations:0` budget. Every one of its36 calls MUST reject before any evaluation, direction, backsolve, dt or history commit; all assertions passed. It measures TimeStep preparation/freezing/compilation plus the failure return, not an accepted step and not an exact subtraction-based partition of a differently timed call. It excludes mixed-workspace creation, which occurs after the initial evaluation.

| Depth | Backend | Warm setup-control single→pair |
|---|---|---:|
| 9mm | GN | 1.270→0.857ms |
| 9mm | Exact | 0.868→2.453ms |
| 160mm | GN | 0.606→0.753ms |
| 160mm | Exact | 0.686→0.758ms |
| 310mm | GN | 0.720→0.872ms |
| 310mm | Exact | 0.638→1.694ms |

At310mm, the GN preparation control rises about0.72→0.87ms. Preparation alone therefore does not explain a28.5ms coupled step. Repeated assembly/linear directions are the main observed amplification; remaining TimeStep certification, state copies and material/geometry validation still add overhead. The operator workspace setup, profile/mesh construction, and all raw cold/warm values are also retained in the data.

One exact optimization opportunity follows from these mechanics: when every local moment is EXACTLY zero, the complete Hessian equals GN, so the correction DAG may be skipped without approximating anything. The current Exact kernel still evaluates it. This is especially relevant to initially straight unstressed wire/shaft elements. This report does not implement or select a numerical threshold for that branch.

## Existing-runtime reference

The existing immutable-style baseline `reports/composite-baseline.json` contains a before-coupling anatomical scene with the wire318mm and a short catheter already present (active-node counts65/16). Its reference full-step mean is8.675ms across three accepted attempts, before the first coupled step becomes29.682ms. It is not an isolated wire-only measurement: preparation, actuation, anatomy and the old solver differ from this benchmark. No directly comparable existing-runtime wire-only timing was identified, and no new anatomical run was performed.

The appropriate conclusion is that the user’s moderate-overhead expectation holds for one solve of the shared operator. The current full runtime is not yet a60FPS implementation: iteration multiplication and general TimeStep overhead must be reduced, and the single-wire path must itself be measured under the eventual runtime conditions.

Files: `scripts/physics/benchmark-composite-single-vs-pair.mjs`, `scripts/physics/helpers/composite-single-vs-pair.js`, `scripts/physics/probe-composite-single-vs-pair-setup.mjs`. Syntax and all embedded consistency/acceptance assertions passed. Frozen handoff: `/tmp/oet-composite-single-vs-pair-final/manifest.json`.

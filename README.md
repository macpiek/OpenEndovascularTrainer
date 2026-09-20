# Open Endovascular Trainer

Open Endovascular Trainer is a browser-based endovascular training prototype built with Three.js. It lets a user practice guidewire and pigtail catheter manipulation inside a modeled aorta, head-and-neck circulation, and bilateral upper- and lower-limb arterial tree, switch between debug and fluoroscopy-style views, move a virtual C-arm, and inject contrast to observe wash-in and washout.

> This project is a simulation prototype for education, research, and interaction design. It is not a medical device and must not be used for clinical decision-making.

## Current Features

- Real-time WebGL simulator with a full-screen Three.js scene.
- Imported aortic, cerebral, iliac, and bilateral upper- and lower-limb arterial anatomy in
  `res/Aorta_plain.stl`, plus the skeleton from `res/skeleton.obj`.
- Shared Kirchhoff model for the guidewire and catheter with independent material feed and spin, segment-length preservation, wall and lumen contact, Coulomb friction, and resistance feedback.
- Precompiled sparse signed-distance collision field backed by a MeshBVH validator; production startup does not generate the centerline or collision field.
- Pigtail and Berenstein catheter shapes rendered with instanced segments instead of rebuilding `TubeGeometry` every frame.
- Introducer sheath positioned in the iliac branch, with retraction limits that keep the wire inside the sheath.
- Fluoroscopy rendering mode with persistence, pulse rate, noise, scatter, collimation, bone visibility, edge enhancement, brightness, contrast, and auto exposure controls.
- Hybrid contrast injection model with conservative pulsatile 1D transport on
  the complete STL centerline tree and a local deterministic 3D plume at the
  selected sheath or catheter outlet. A cell-resolved lumen mesh fills the
  vessel cross-section continuously, while the unmixed outlet jet uses
  volume-conserving directional streaks.
- C-arm controls for LAO/RAO, CRA/CAU, roll, table-plane movement, height, readouts, and a miniature C-arm preview.
- Patient monitor and procedure readouts for inserted length, pigtail length, contrast dose, kV, mA, FPS, and memory use.
- Remotion composition for creating a short promotional video from the project assets.

## Quick Start

Install dependencies and run the local dev server:

```bash
npm install
npm run dev
```

Open the URL printed by Vite, usually:

```text
http://localhost:5173
```

Node.js 20 or newer is recommended for the Vite 7 toolchain used by this project.

## Controls

The simulator starts in fluoroscopy mode.

| Action | Control |
| --- | --- |
| Advance guidewire | `W` or `ArrowUp` |
| Retract guidewire | `S` or `ArrowDown` |
| Advance pigtail catheter | `D` or the `Advance` button |
| Withdraw pigtail catheter | `A` or the `Withdraw` button |
| Rotate pigtail catheter left/right | `Q` / `E` or the rotate buttons |
| Inject contrast through the selected sheath/catheter source | `I`, `C`, or the `Inject` button |
| Stop active injection | `Stop Injection` |
| Toggle debug/fluoroscopy view | `Debug` / `Fluoroscopy` button |

The Injection panel exposes an explicit sheath/catheter source, volume, and
rate. Duration is calculated from volume/rate. The other panels expose runtime
sliders for guidewire stiffness, smoothing, wall friction, image quality,
contrast display, and C-arm position.

Use `npm run test:contrast` for the quantitative hybrid-model checks. The
clinician review scenarios and acceptance scorecard are in
[`reports/contrast-clinical-validation.md`](reports/contrast-clinical-validation.md).

## Vessel Geometry

The vessel centerline metadata is generated deterministically. Branch length and angle offset use fixed defaults (140 units and 0 radians) and only change when explicitly provided to `generateVessel`. A short introducer sheath extends from the distal left branch with a 30 degree tilt against the vessel wall toward +Z.

The visible vessel, tool-wall contacts, and contrast-flow tree are driven by
the imported STL aorta and `res/Aorta_plain.collision.bin`. The procedural
vessel data is retained only for control and tool-path metadata.

The STL continues both external iliac arteries through the common and
superficial femoral, deep femoral, popliteal, anterior tibial, posterior
tibial, tibioperoneal trunk, fibular, dorsalis pedis, medial and lateral
plantar, deep plantar, and transverse plantar arch arteries. Smaller branches
include the circumflex and perforating femoral, genicular, and anterior tibial
recurrent arteries. Four plantar metatarsal branches divide into proper plantar
digital arteries reaching all five toes. These are hollow, tapered vessel walls
joined to the original lumen rather than overlaid visual tubes. The
repeatable `npm run anatomy:extend-lower-limbs` generator starts from the
original STL and rebuilds this Boolean extension before the collision asset is
regenerated.

Both subclavian endpoints continue through the axillary and brachial arteries.
Each upper limb includes the deep brachial, radial, ulnar, common/anterior
interosseous, superficial and deep palmar arch, and four common palmar digital
branches. It also includes the thoracoacromial, lateral thoracic,
subscapular/thoracodorsal, circumflex scapular and humeral, collateral,
recurrent, and posterior interosseous branches. The common digital branches
divide into proper digital arteries, with separate princeps pollicis and thumb
digital arteries. Their paths are aligned anterior-medial to the humerus, on the
appropriate sides of the paired forearm bones, and anterior to the hand
skeleton. They share the same hollow Boolean wall, collision field, and
contrast-flow tree as the pre-existing vessels.

The shipped subclavian arteries have a subsequent bilateral correction relative
to the loaded clavicles and first ribs. Their arches turn medially and posteriorly
before descending toward the axillae. The rendered STL and collision asset share
this correction. This is a model-based anatomical approximation; a single AP
angiogram cannot determine a patient-specific 3D course.

The reproducible offline correction is `scripts/align-subclavian-arteries.mjs`.
It requires Python with NumPy (`PYTHON` selects the interpreter) and accepts
distinct input/output resource directories. It checks the input STL hash against
`scripts/anatomy/subclavian-landmarks.json`; the source revision is recorded there.
It locally subdivides the surface, transports both subclavian regions and outlet
landmarks, and reconstructs hollow extension junctions and small shoulder branches.
After this step, regenerate `Aorta_plain.collision.bin` with `npm run collision:build`
in the prepared project, and install the STL, collision binary and both JSON
manifests together. `npm run test:anatomy` and `node --test tests/aortaOutletClosures.test.js` check
the shared assets, the subclavian clearance and the preserved terminal walls.
Running the older `anatomy:rebuild` generator alone restores the earlier course;
it does not apply this later correction or the outlet closures.

At the unchanged superior endpoints of the source model, the STL also
continues both common carotid and vertebral arteries. It includes the external
and internal carotids, vertebrobasilar system, complete Circle of Willis, and
distal ACA, MCA, and PCA branches. The extensions follow the original terminal
centerlines while the pre-existing aorta and lower-limb geometry retain their
original positions.

The STL centerline is extracted offline as one acyclic medial tree. Each lumen cross-section is thinned to a topological medial axis, the resulting 3D graph is reduced with a clearance-weighted TEASAR pass, and every final edge is checked against the STL wall BVH. The centerline is used only for broad-phase lookup and branch identity; it never pulls a simulated tool toward the vessel axis.

## Development Scripts

Pełna dokumentacja architektury i indeks funkcji są dostępne jako statyczna
strona w [`docs/index.html`](docs/index.html). Indeks można odświeżyć poleceniem
`npm run docs:generate`; workflow aktualizuje go także automatycznie po merge.

```bash
npm run dev          # start Vite development server
npm run build        # build the browser app
npm run anatomy:rebuild # rebuild the hollow limb, neck, and cerebral arteries
npm run collision:build # regenerate the versioned centerline and sparse SDF asset
npm run benchmark:collision # write direct Kirchhoff/contact timing reports to reports/
npm run benchmark:browser:chrome # run the foreground Chrome acceptance workload
npm run benchmark:browser:safari # run the same workload through Safari WebDriver
npm run centerline:diagnostics # export centerline metrics and orthogonal projections
npm run test:contrast # run hybrid contrast conservation/performance tests
npm run preview      # preview the production build
npm test             # run simulator syntax checks and regression tests
npm run video:studio # open the Remotion studio
npm run video:still  # render a still frame to out/endovascular-trainer-frame.png
npm run video:render # render the promotional video to out/endovascular-trainer-ad.mp4
```

For a simple static server without Vite, the helper script can serve the repository root:

```bash
./scripts/run-browser.sh
```

## Project Layout

```text
index.html                  Main simulator shell and controls
style.css                   Simulator UI styling
src/simulator.js            Main scene, physics loop, rendering passes, and integration
src/physics/endovascularPhysicsWorld.js Shared XPBD rod/contact world
src/physics/collision/         Packed collision asset and VesselContactField
src/physics/rodState.js        Material-node storage shared with rendering
src/physics/guidewireTransport.js Prescribed inlet feed and contact diagnostics
src/pigtailCatheter.js      Pigtail catheter behavior and mesh generation
src/contrast/               Hybrid 1D/3D contrast transport and volume renderer
src/vesselGeometry.js       Vessel centerline, sheath, flow, and branch metadata
src/aortaModel.js           STL loading and vessel collision setup
src/aortaPreprocess.js      Offline/source lumen preprocessing helpers
src/lowerLimbArteries.js    Bilateral lower-limb path and attachment definitions
src/upperLimbArteries.js    Bilateral upper-limb path and attachment definitions
src/headArteries.js         Carotid, vertebrobasilar, and cerebral path definitions
src/boneModel.js            Skeleton asset loading
src/carmControls.js         C-arm movement controls
src/ui/                    UI widgets, monitor, and C-arm preview
res/                        Aorta STL and skeleton OBJ assets
tests/                      Physics and solver regression tests
video/                      Remotion video composition
out/                        Generated preview frame and video
```

## Collision And Physics

The default application uses `shared-axis` with a reference 5 mm mechanical grid.
The **Debug → Solver fizyki** selector also offers the experimental
`shared-axis-adaptive` variant and older solvers. Apply the selection with
**Zmień solver i zresetuj scenę**; switching reloads the scene. The adaptive
variant merges quiet shaft regions up to 20 mm, preserves fine tip/curvature/contact
regions, and uses a 0.15 mm local shape simplification threshold. This threshold
is not a bound on accumulated trajectory error. It also relaxes the nonlinear
force/torque residual to `1e-4` and geometric residual to `1e-3`; the reference
retains `1e-6` / `1e-5`. Actual tool endpoints can create shorter cells.
In the debug view, **Węzły pręta** draws accepted mechanical nodes (cyan wire,
orange catheter); the panel reports the shared node count, DOFs and segment lengths.
The experimental mode is also available at `?coupledSolver=shared-axis-adaptive`.

Debug also offers **Newton: ponowne użycie macierzy i faktoryzacji** for both
shared-axis grids. It is opt-in (`modifiedNewton=1`), applied with the solver
restart button. The experiment retains the full KKT matrix and pivoted LU for
at most two additional directions after a small, productive Newton step.
Changed active contacts/supports, poor progress, rejected directions or newly
discovered contacts refresh the tangent. Current nonlinear residuals and
contact/friction acceptance remain mandatory; a failed experimental timestep
retries the original method. The Debug counter reports accepted frozen-matrix
attempts and full-matrix fallbacks. This is an approximation, not exact reuse
of a current Jacobian, and its speed benefit depends on the motion.
See `reports/modified-newton-2026-09-17/` for comparisons and limitations.

Debug additionally offers **Projective Dynamics — pręt podatny, siatka adaptacyjna**
(`?coupledSolver=shared-axis-projective&solverDebug=1`). This opt-in experiment
uses local SO(3)/length/contact projections and a reusable SPD band Cholesky
factor, with 16 local/global iterations per substep. The shared centerline and
independent material frames preserve both tool profiles and proximal rotation.
It is a different, approximate Cosserat penalty model: finite stretch/shear,
chordal bend/twist, penalty wall contact and a positional Coulomb approximation.
It does not claim the reference solver's force/friction certificate. Debug shows
iterations, factorizations, geometric errors and whether the iteration budget
was exhausted. Accepted steps must stay within 3% segment length error, 0.1 mm
wall penetration, a 0.1 director/tangent mismatch and the bend limit + 2 degrees.
The adaptive mesh sliders also work in PD. Modified Newton is unavailable for PD.
See [the PD experiment report](reports/projective-dynamics-2026-09-17/README.md)
for measurements and limitations. The default solver remains `shared-axis`.

Historical composite integration notes below describe an earlier rollout:
the application then used the previous `joint-two-channel` solver while
an insertion regression in the new model is being repaired. The experimental
`?coupledSolver=composite-joint` model solves both tools in one common/relative
system with independent material feed, spin, stiffness and contact reactions.
It currently stalls near 131.27 mm in the captured anatomy replay and is not
ready to replace the default solver.
The world uses a fixed 1/120 s step. Numerical work can continue across render
frames, and a timestep is consumed only after acceptance. The on-screen solver
status distinguishes accepted physical steps from rendering FPS. Performance
optimization and the 60 FPS acceptance workload remain a subsequent phase. The
manufactured curvature lives in material frames, while the introducer boundary
controls feeding. Vessel, sheath and tool contacts remain unilateral constraints.
See [the UI integration status and known convergence limits](reports/composite-joint-ui-integration.md)
before treating this development version as a completed simulator.

Earlier solver variants remain available for reproducing previous measurements.
The earlier simultaneous material/contact solver is available at
`?coupledSolver=joint-active-coulomb`. It enables exact active material elimination
and simultaneous Coulomb Newton; `?coupledSolver=reference` selects the previous direct implementation. Debug benchmark
reports identify the selected variant and count its actual solves. This variant
does not yet meet the real-time target: inspect both FPS and physics backlog.
The additional `?coupledSolver=joint-full-band` variant retains the full local
dual operator and uses nonsymmetric band LU for Coulomb Newton. Frozen comparisons
do not show a consistent speed advantage, so it is also an explicit experiment.
See [current integration and validation](reports/coupled-rebuild-progress.md).

For a deterministic Node replay of that variant, use:

```bash
node scripts/benchmark-coupled-rebuild.mjs --solver joint-active-coulomb --deep
npm run test:physics:coupled
```

`scripts/physics/compare-coupled-timesteps.mjs --hz 60,120 --prepare-hz 120`
compares timestep choices after the same guidewire preparation. It is a CPU
diagnostic, not a browser FPS or scheduler acceptance test.

An additional World prototype separates physical motion from geometric bias:
`jointMotionMode: 'split-physical-bias'` with a joint solver installed. It is
available in the timestep diagnostic through `--motion-mode split-physical-bias`;
add `--bias-material-mode preserve-strain` for the strain-preserving variant.
The browser default is unchanged. `configureKirchhoffSplitBias(joint,
{ materialMode: 'preserve-strain' })` additionally preserves the physical
material strain during geometric repair. Small analytic tests pass, but new
contacts after bias, wall witness migration and sheath history still require
closure before general runtime use. See the [prototype handoff](reports/split-physical-bias-handoff.md)
and [strain preservation comparison](reports/split-bias-preserve-strain.md).

For an eligible split joint, `world.stepFixed()` returns an acceptance result.
A rejected step restores the mechanical state and preserves elapsed time;
`world.advance()` retries its prepared inputs without invoking the input callback
again. Independent guidewire preparation remains available before the pair is
eligible. The bias phase first evaluates the existing nonlinear accuracy gates
and skips its linear solve when they already pass. These changes do not establish
the deep-insertion performance target. See [step transactions](reports/split-step-transaction.md).

Run `npm run test:guidewire:mechanics` and `npm run test:catheter:mechanics` for
analytical bending, unloading, torsion and catheter-over-wire regressions.
See [the removal and validation report](reports/direct-only-solver.md) for the
scope of the cleanup and remaining acceptance failures inherited from the direct solver.

See [the contact-block report](reports/catheter-contact-block.md) for the coupled
solve, numerical kernel and deep-insertion performance measurements.

Regenerate the collision asset whenever `Aorta_plain.stl`, its transform, or the offline centerline/SDF pipeline changes. To reproduce all arterial extensions from the original model, run the anatomy generator first:

```bash
npm run anatomy:rebuild
npm run collision:build
npm test
npm run build
```

See `reports/collision-system.md` for the contact API, asset layout, benchmark results, and acceptance status.
For the foreground ten-minute browser workload, open the `Debug` tab, select `Start 10 min`, and leave the simulator in the foreground until the acceptance report appears. The automated Chrome and Safari commands use the same deterministic workload and a two-cycle warmup; Safari WebDriver additionally requires `Allow remote automation` in Safari's Developer settings.
Treat the long browser workload as a regression gate for major physics/rendering changes and releases. During active solver development, use the deterministic unit/regression suite plus a short browser smoke run instead of tuning isolated frame-time outliers.

## Tests and benchmarks

```bash
npm test
npm run test:guidewire:mechanics
npm run test:catheter:mechanics
npm run benchmark:kirchhoff
npm run benchmark:collision
```

### Dwa dostępy udowe

Przyciski **Prawa koszulka** i **Lewa koszulka** w zakładce **Tool selection**
wybierają aktywny dostęp.
Każdy dostęp ma osobny prowadnik, cewnik, ustawienia sztywności i historię solvera.
Klawisze oraz przyciski sterują aktywnym zestawem. Zmiana dostępu kończy bieżący
krok, zatrzymuje automatyczne wycofywanie i podawanie kontrastu; drugi zestaw
pozostaje widoczny i jest nadal symulowany, z zerowym poleceniem wsuwania/obrotu.
Oba solvery mają niezależne zegary, kroki w toku i punkty odtwarzania po odrzuceniu.
Wskaźnik fizyki pokazuje osobno Hz prawego (P) i lewego (L) dostępu. Kontrast jest następnie podawany z wybranej koszulki lub jej cewnika.
W debug aktywna koszulka jest żółta. Kolizje między narzędziami należącymi do
różnych dostępów nie są obecnie rozwiązywane; sprzężenie prowadnik–cewnik działa
osobno dla każdego dostępu.

Testy przełączania i położenia koszulek: `node --test tests/femoralAccess.test.js`.

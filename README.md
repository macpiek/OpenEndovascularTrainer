# Open Endovascular Trainer

Open Endovascular Trainer is a browser-based endovascular training prototype built with Three.js. It lets a user practice guidewire and pigtail catheter manipulation inside a modeled aortoiliac vessel, switch between debug and fluoroscopy-style views, move a virtual C-arm, and inject contrast to observe wash-in and washout.

> This project is a simulation prototype for education, research, and interaction design. It is not a medical device and must not be used for clinical decision-making.

## Current Features

- Real-time WebGL simulator with a full-screen Three.js scene.
- Imported aorta and skeleton assets from `res/Aorta_plain.stl` and `res/skeleton.obj`.
- Shared XPBD world for the guidewire and catheter with segment-length preservation, bending/rest-shape constraints, wall contact, Coulomb friction, and resistance feedback.
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

The STL centerline is extracted offline as one acyclic medial tree. Each lumen cross-section is thinned to a topological medial axis, the resulting 3D graph is reduced with a clearance-weighted TEASAR pass, and every final edge is checked against the STL wall BVH. The centerline is used only for broad-phase lookup and branch identity; it never pulls a simulated tool toward the vessel axis.

## Development Scripts

Pełna dokumentacja architektury i indeks funkcji są dostępne jako statyczna
strona w [`docs/index.html`](docs/index.html). Indeks można odświeżyć poleceniem
`npm run docs:generate`; workflow aktualizuje go także automatycznie po merge.

```bash
npm run dev          # start Vite development server
npm run build        # build the browser app
npm run collision:build # regenerate the versioned centerline and sparse SDF asset
npm run benchmark:collision # write legacy/XPBD timing reports to reports/
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
src/physics/elasticRod.js      Legacy elastic rod physics model
src/physics/guidewireSolver.js Guidewire path and collision solver
src/pigtailCatheter.js      Pigtail catheter behavior and mesh generation
src/contrast/               Hybrid 1D/3D contrast transport and volume renderer
src/vesselGeometry.js       Vessel centerline, sheath, flow, and branch metadata
src/aortaModel.js           STL loading and vessel collision setup
src/aortaPreprocess.js      Offline/source lumen preprocessing helpers
src/boneModel.js            Skeleton asset loading
src/carmControls.js         C-arm movement controls
src/ui/                    UI widgets, monitor, and C-arm preview
res/                        Aorta STL and skeleton OBJ assets
tests/                      Physics and solver regression tests
tools/legacy/               Standalone legacy simulation demos
video/                      Remotion video composition
out/                        Generated preview frame and video
```

## Collision And Physics

The default mode is `xpbd-contact-v1`; append `?physics=legacy` to compare the previous path. The shared world runs at 120 Hz with at most two substeps per rendered frame. It solves the analytic sheath lumen, rod length and bending/rest shape, guidewire-in-catheter containment, external tool contact, vessel wall contact, and friction in a fixed order.

Regenerate the collision asset whenever `Aorta_plain.stl`, its transform, or the offline centerline/SDF pipeline changes:

```bash
npm run collision:build
npm test
npm run build
```

See `reports/collision-system.md` for the contact API, asset layout, benchmark results, and acceptance status.
For the foreground ten-minute browser workload, open the `Debug` tab, select `Start 10 min`, and leave the simulator in the foreground until the acceptance report appears. The automated Chrome and Safari commands use the same deterministic workload and a two-cycle warmup; Safari WebDriver additionally requires `Allow remote automation` in Safari's Developer settings.
Treat the long browser workload as a regression gate for major physics/rendering changes and releases. During active solver development, use the deterministic unit/regression suite plus a short browser smoke run instead of tuning isolated frame-time outliers.

## Legacy Physics Notes

The guidewire is modeled as an `ElasticRod` with position-based constraints. Each segment is kept near its rest length, bending behavior is approximated by curvature and shape constraints, and wall contact applies tangential friction. Runtime tuning hooks are exported from `src/physics/elasticRod.js`:

```js
import {
  setBendingStiffness,
  setSmoothingIterations,
  setWallFriction
} from './src/physics/elasticRod.js';

setBendingStiffness(0.8);
setSmoothingIterations(1);
setWallFriction(0.006, 0.002);
```

Lower friction values reduce sticking when the wire slides along the vessel wall. Higher bending stiffness makes the wire straighten more aggressively after release or withdrawal.

## Test and Demo Scripts

The primary test command checks simulator syntax and runs the regression tests:

```bash
npm test
```

Legacy guidewire experiments produce JSON-style logs:

```bash
node tools/legacy/elasticRod/straightening.js
node tools/legacy/elasticRod/wallBend.js
node tools/legacy/elasticRod/branchCollision.js
node tools/legacy/elasticRod/remoteSegmentInterference.js
```

For an isolated browser visualization of the rod model, open:

```text
tools/legacy/elasticRod/visualize.html
```

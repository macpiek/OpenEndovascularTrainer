# Open Endovascular Trainer

Open Endovascular Trainer is a browser-based endovascular training prototype built with Three.js. It lets a user practice guidewire and pigtail catheter manipulation inside a modeled aortoiliac vessel, switch between debug and fluoroscopy-style views, move a virtual C-arm, and inject contrast to observe wash-in and washout.

> This project is a simulation prototype for education, research, and interaction design. It is not a medical device and must not be used for clinical decision-making.

## Current Features

- Real-time WebGL simulator with a full-screen Three.js scene.
- Imported aorta and skeleton assets from `res/Aorta_plain.stl` and `res/skeleton.obj`.
- Position-based guidewire physics with segment-length preservation, bending stiffness, wall contact, friction, and resistance feedback.
- Guidewire solver and preprocessed aorta lumen field for more robust wall collision behavior.
- Pigtail catheter model that advances over the guidewire and rotates inside the vessel.
- Introducer sheath positioned in the iliac branch, with retraction limits that keep the wire inside the sheath.
- Fluoroscopy rendering mode with persistence, pulse rate, noise, scatter, collimation, bone visibility, edge enhancement, brightness, contrast, and auto exposure controls.
- Contrast injection model with adjustable volume, rate, and duration.
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
| Inject contrast in fluoroscopy mode | `C` or the `Inject` button |
| Stop active injection | `Stop Injection` |
| Toggle debug/fluoroscopy view | `Debug` / `Fluoroscopy` button |

The control panels also expose runtime sliders for guidewire stiffness, smoothing, wall friction, injection parameters, image quality, contrast display, and C-arm position.

## Vessel Geometry

The vessel centerline metadata is generated deterministically. Branch length and angle offset use fixed defaults (140 units and 0 radians) and only change when explicitly provided to `generateVessel`. A short introducer sheath extends from the distal left branch with a 30 degree tilt against the vessel wall toward +Z.

The visible vessel and guidewire wall collisions are driven by the imported STL aorta model and its preprocessed lumen field; the procedural vessel data is kept for flow, controls, and tool path metadata.

The STL centerline is extracted as one acyclic medial tree. Each lumen cross-section is thinned to a topological medial axis, the resulting 3D graph is reduced with a clearance-weighted TEASAR pass, and every final edge is checked against the STL wall BVH. A final optimization moves short path neighborhoods away from the nearest wall while limiting added curvature. This preserves connected distal branches without the parallel paths and wall shortcuts produced by merging independent axial centerlines.

## Development Scripts

```bash
npm run dev          # start Vite development server
npm run build        # build the browser app
npm run centerline:diagnostics # export centerline metrics and orthogonal projections
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
src/physics/elasticRod.js      Elastic rod physics model
src/physics/guidewireSolver.js Guidewire path and collision solver
src/pigtailCatheter.js      Pigtail catheter behavior and mesh generation
src/contrastFlowAgent.js    Centerline contrast transport model
src/vesselGeometry.js       Vessel centerline, sheath, flow, and branch metadata
src/aortaModel.js           STL loading and vessel collision setup
src/aortaPreprocess.js      Lumen field preprocessing for the aorta model
src/boneModel.js            Skeleton asset loading
src/carmControls.js         C-arm movement controls
src/ui/                    UI widgets, monitor, and C-arm preview
res/                        Aorta STL and skeleton OBJ assets
tests/                      Physics and solver regression tests
tools/legacy/               Standalone legacy simulation demos
video/                      Remotion video composition
out/                        Generated preview frame and video
```

## Physics Notes

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

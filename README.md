# Open Endovascular Trainer

This prototype demonstrates a basic browser-based simulator for guiding a stiff wire through a branched vessel. The vessel consists of a main tube with smoothly joined side branches modeled with quadratic curves. The guidewire is now simulated with a position‑based dynamics solver that preserves segment length, adds bending stiffness, and applies tangential friction when it contacts the vessel wall, producing more realistic motion and preventing artificial shortening. The visual style mimics fluoroscopy by using a monochrome palette and persistent trail.

## Usage

Run the app with a local dev server so ES modules and assets resolve correctly:

```bash
npm install
npm run dev
```

Then open the URL shown by Vite (typically `http://localhost:5173`). Use `W`/`S` or the up/down arrow keys to advance or retract the guidewire through the introducer sheath positioned at the distal end of the left branch. Retraction stops when the wire's tip reaches the sheath entrance to keep it within the sheath. The sheath enters this branch at a 30° angle against the vessel wall, tilting toward the anterior (+Z) direction so the wire can pass from outside the body into the vessel lumen.


Click the **Fluoroscopy** button to hide the debug vessel surfaces and display only the fluoroscopy view. Click again to return to the debug view.

## Vessel Geometry


The vessel centerline metadata is generated deterministically. Branch length and angle offset use fixed defaults (140 units and 0 radians) and only change when explicitly provided to `generateVessel`. A short introducer sheath extends from the distal left branch with a 30° tilt against the vessel wall toward +Z.

The visible vessel and guidewire wall collisions are driven by the imported STL aorta model and its preprocessed lumen field; the procedural vessel data is kept for flow, controls, and tool path metadata.


## Tuning wall friction

The guidewire uses a simple Coulomb model when it collides with the vessel wall. Static and kinetic friction coefficients can be adjusted at runtime to control how easily the wire slides and straightens after withdrawal. Lower defaults are already applied to minimise sticking, but you can tweak them further:

```js
import { setWallFriction } from './src/physics/elasticRod.js';

// lower values reduce sticking on the vessel wall
setWallFriction(0.05, 0.02);
```

Providing smaller coefficients allows the wire to shed kinks more readily when pulled back through a branch.

## Elastic Rod Constraints

The `ElasticRod` physics used for the guidewire keeps each segment at a fixed
rest length and approximates bending moments by pulling interior nodes toward
the midpoint of their neighbours. Positions are integrated with a semi-implicit
Euler step followed by constraint projection and a small velocity damping
factor. This simple model ignores shear and torsion and is stable for time
steps of roughly `0.01` seconds or smaller.

Curvature for each node is computed from neighbouring positions and a
straightening force proportional to the node's `bendingStiffness` is applied.
After constraints are solved an optional Laplacian smoothing pass can further
relax sharp bends. Default values for bending stiffness and the number of
smoothing iterations may be configured via the `setBendingStiffness` and
`setSmoothingIterations` functions exported from `src/physics/elasticRod.js`.

## Simulation logging and tests

`ElasticRod` accepts an optional `logger` callback. When provided, the callback
is invoked after each `step` with the current iteration count, average
curvature and total rod length:

```js
const rod = new ElasticRod(10, 1, {
  logger: data => console.log(data)
});
```

Automated regression tests live in `tests/` and can be run with:

```sh
npm test
```

Legacy demonstration scripts exercising the rod model live in `tools/legacy/elasticRod`:

* `straightening.js` – rod straightening after release
* `wallBend.js` – bending while sliding along a vessel wall
* `branchCollision.js` – collision at a vessel bifurcation

Run them with Node to produce JSON logs describing the simulation state:

```sh
node tools/legacy/elasticRod/straightening.js
node tools/legacy/elasticRod/wallBend.js
node tools/legacy/elasticRod/branchCollision.js
```

For a quick visual check, open `tools/legacy/elasticRod/visualize.html` in a modern
browser. It uses Three.js to display the rod evolving in isolation.

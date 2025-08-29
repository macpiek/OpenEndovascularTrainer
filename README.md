# Open Endovascular Trainer

This prototype demonstrates a basic browser-based simulator for guiding a stiff wire through a branched vessel. The vessel consists of a main tube with smoothly joined side branches modeled with quadratic curves. The guidewire is now simulated with a position‑based dynamics solver that preserves segment length, adds bending stiffness, and applies tangential friction when it contacts the vessel wall, producing more realistic motion and preventing artificial shortening. The visual style mimics fluoroscopy by using a monochrome palette and persistent trail.

## Usage

Open `index.html` in a modern browser. Use `W`/`S` or the up/down arrow keys to advance or retract the guidewire through the introducer sheath positioned in the left branch. The sheath exits this branch with a fixed 30° anterior (+Z) angulation, and the push distance now allows the wire to be fully inserted if desired.

Click the **Fluoroscopy** button to hide the vessel and display only the guidewire. Click again to return to the wireframe view.

## Vessel Geometry

The vessel is generated deterministically. Branch length and angle offset use fixed defaults (140 units and 0 radians) and only change when explicitly provided to `generateVessel`. The introducer sheath extends from the left branch with a 30° anterior (+Z) tilt.

## Tuning wall friction

The guidewire uses a simple Coulomb model when it collides with the vessel wall. Static and kinetic friction coefficients and the amount of normal damping can be adjusted at runtime to control how easily the wire slides and straightens after withdrawal. Lower defaults are already applied to minimise sticking, but you can tweak them further:

```js
import { setWallFriction, setNormalDamping } from './physics/guidewire.js';

// lower values reduce sticking on the vessel wall
setWallFriction(0.05, 0.02);
setNormalDamping(0.2);
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
`setSmoothingIterations` functions exported from `physics/elasticRod.js`.

## Simulation logging and tests

`ElasticRod` accepts an optional `logger` callback. When provided, the callback
is invoked after each `step` with the current iteration count, average
curvature and total rod length:

```js
const rod = new ElasticRod(10, 1, {
  logger: data => console.log(data)
});
```

Example scripts exercising the rod model live in `tests/elasticRod`:

* `straightening.js` – rod straightening after release
* `wallBend.js` – bending while sliding along a vessel wall
* `branchCollision.js` – collision at a vessel bifurcation

Run them with Node to produce JSON logs describing the simulation state:

```sh
node tests/elasticRod/straightening.js
node tests/elasticRod/wallBend.js
node tests/elasticRod/branchCollision.js
```

For a quick visual check, open `tests/elasticRod/visualize.html` in a modern
browser. It uses Three.js to display the rod evolving in isolation.

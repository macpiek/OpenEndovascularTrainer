# Open Endovascular Trainer

This prototype demonstrates a basic browser-based simulator for guiding a stiff wire through a branched vessel. The vessel consists of a main tube with smoothly joined side branches modeled with quadratic curves. The guidewire is now simulated with a position‑based dynamics solver that preserves segment length, adds bending stiffness, and applies tangential friction when it contacts the vessel wall, producing more realistic motion and preventing artificial shortening. The visual style mimics fluoroscopy by using a monochrome palette and persistent trail.

## Usage

Open `index.html` in a modern browser. The guidewire begins at the outer end of the introducer sheath. Use `W`/`S` or the up/down arrow keys to advance or retract the guidewire through the sheath and into the vessel. The sheath exits this branch with a fixed 30° anterior (+Z) angulation, and the push distance now allows the wire to be fully inserted if desired.

Click the **Fluoroscopy** button to hide the vessel and display only the guidewire. Click again to return to the wireframe view.

## Vessel Geometry

The vessel is generated deterministically. Branch length and angle offset use fixed defaults (140 units and 0 radians) and only change when explicitly provided to `generateVessel`. The introducer sheath extends from the left branch with a 30° anterior (+Z) tilt.

# Open Endovascular Trainer

This prototype demonstrates a basic browser-based simulator for guiding a stiff wire through a branched vessel. The vessel consists of a main tube with smoothly joined side branches modeled with quadratic curves. The guidewire is now simulated with a position‑based dynamics solver that preserves segment length, adds bending stiffness, and applies tangential friction when it contacts the vessel wall, producing more realistic motion and preventing artificial shortening. The visual style mimics fluoroscopy by using a monochrome palette and persistent trail.

## Usage

Open `index.html` in a modern browser. Use `W` and `S` to advance or retract the guidewire through the introducer sheath positioned in the left branch. The push distance now allows the wire to be fully inserted if desired.

# Open Endovascular Trainer

This prototype demonstrates a basic browser-based simulator for guiding a stiff wire through a branched vessel. The vessel consists of a main tube with smoothly joined side branches modeled with quadratic curves, and the guidewire uses a 3D Verlet rope with midpoint bending constraints that keep the rod smoothly curved instead of visibly segmented. Contact with the vessel wall dissipates velocity to emulate friction, so the wire advances more like a real steel guide without lateral steering. The visual style mimics fluoroscopy by using a monochrome palette and persistent trail.

## Usage

Open `index.html` in a modern browser. Use `W` and `S` to advance or retract the guidewire through the introducer sheath positioned in the left branch. The push distance is limited so a short segment of wire always remains outside.

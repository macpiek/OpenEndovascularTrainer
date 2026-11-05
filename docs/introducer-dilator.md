# Removable sheath dilator

Each access has its own matching dilator for the selected 7, 45 or 90 cm
sheath, in sizes 6F–24F. The default state is withdrawn.

Starting sheath insertion from zero with **H** (or its insert button) automatically
advances the matching dilator first, then moves the seated pair. The tapered tip
enters gradually at the commanded feed speed; releasing H stops either stage.
The wire must still lead the tip by 10 mm. With an existing catheter in place,
sheath exchange remains coreless. **G** withdraws the pair and then its remaining
25 mm tip completely, allowing a new selection without a separate V action.
Independent B/V control remains available.

- Hold **B** or the insert button to advance over the wire; **V** withdraws
  the dilator independently. Advance requires the wire tip at least 10 mm ahead.
- Full seating places the tapered tip 25 mm beyond the sheath and couples
  the hubs. **G/H** then move the sheath and dilator together. Withdrawing
  the dilator releases the coupling.
- The catheter and dilator cannot be inserted into the same access lumen
  simultaneously. Withdraw the dilator before catheter insertion or sheath
  selection. The guidewire can stay in place.

The native shared-axis solver has a fourth material slot for the removable
core. Its positive bending/torsional stiffness contributes along the physical
overlap, including the shaft; removing it removes that support. Its hollow
polymer profile and collision envelope taper smoothly to the distal tip.
The displayed debug/fluoroscopic mesh uses the accepted mechanical path and
the same radius profile. Failed or pending steps do not publish a trial path.

Generic model dimensions: shaft radius = sheath lumen radius minus 0.04 mm;
wire lumen radius = 0.47 mm; distal outer radius = 0.52 mm. Stiffness follows
the hollow section's fourth moment and is in existing simulation units.
This is not a calibrated manufacturer-specific dilator. Historical crossover
benchmark URLs retain their recorded material topology and disable the core.

Tests: `tests/introducerDilator.test.js` and
`tests/introducerDilatorAnatomy.test.js` cover dimensions, taper, mechanical
support, independent motion, seating, exclusions, rollback, replay and both
femoral access routes on the actual renal anatomy.

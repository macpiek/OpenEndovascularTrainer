# Compliant balloon catheter

Select **Tool selection → Catheter → Balon podatny · Reliant 46**. Replace the
withdrawn sheath with at least 12F, remove its dilator, advance the wire, and use
D/A to place the balloon catheter over it. The complete balloon section must
clear the sheath before **Napełnij** becomes available. Choose a target in ml;
**Opróżnij balon** removes fluid gradually. Space pauses inflation and deflation
along with the rest of simulation. Both access sites keep independent balloons.
While inflated, feed and rotation of the coaxial tools are held; deflate before
repositioning. Exchange remains possible only with the catheter withdrawn.

## Source dimensions

[Medtronic Reliant product specifications](https://www.medtronic.com/en-us/healthcare-professionals/products/cardiovascular/aortic/accessories/reliant-stent-graft-balloon-catheter.html):
8F shaft, 100 cm working length, 12F introducer compatibility, guidewire up to
0.038 inch, compliant 10–46 mm balloon. Free inflation guide:
3/9/19/41/60 ml corresponds to 10/20/30/40/46 mm.
The simulator's existing 0.035 inch guidewires fit this catheter.

## Model and limits

The straight shaft participates in the existing shared-axis Kirchhoff mechanics,
with its actual 8F diameter and a separate tunable polymer rigidity preset.
The rigidity is not a measured manufacturer material property. Selecting a fully
withdrawn 8F catheter beside a 6F sheath must not reject every timestep: the
external access boundary accommodates it, while insertion is explicitly blocked
until a compatible sheath is selected. A selected smaller sheath is also refused
while a balloon catheter remains inserted.

Inflation now participates in the same accepted mechanical transaction as the
wire, catheter, sheath and vessel contact. A nested radial envelope conserves
free volume and matches the published guide points. Every cross-section grows
monotonically; shoulders no longer retract as the balloon fills. The envelope
occupies a fixed 72 mm section, 8 mm behind the tip. These axial dimensions are
simulation presets, not manufacturer measurements.

The outer envelope discovers contacts with the actual anatomical triangles.
A nonlinear membrane spring and a nonlinear wall foundation act in series under
the same normal reaction. Compression cannot pass through the 8F shaft. The
reaction changes the shared catheter/wire axis; inflation also increases the
bending rigidity of the balloon section to represent membrane tension. Both
mechanisms can straighten or deflect the tools. The membrane uses frictionless
normal contact: applying rigid-shaft friction at the uncompressed free radius
would introduce spurious torque. Ordinary shaft friction remains unchanged.

Accepted wall displacement is published through the same deformation field as
the vessel and contrast. It is bounded to keep the axis inside the reference
lumen. The displayed membrane follows the accepted axis and radial wall hits,
including that displacement. This is a first-order radial reconstruction, not
an independent mesh that pushes through the wall. The actual contained volume
feeds the next inflation step. Requested volume may exceed the local capacity;
the UI reports contained volume and wall limitation. A reduced pressure/strain
limit bounds free-envelope overdrive to the contained volume plus the larger of
0.35 ml or 35% of contained volume, also capped at the 60 ml guide endpoint.
This pump limit is a simulation preset, not a clinical pressure calibration.
The membrane tension contribution is independent of catheter stiffness sliders.
Inflation uses continuation substeps while retaining the force certificates. Deflation reduces that drive at simulation
time; Space freezes it. Rejected steps roll back inflation together with the rods.

This is a reduced mechanical model with tunable, uncalibrated stiffness and
compliance, not a pressure-calibrated membrane FEM. It does not yet reshape
stent grafts or occlude the hydraulic network, and does not predict clinical
pressure, injury, molding outcome or distal hemodynamics.

Verification: `node --test tests/balloonMechanics.test.js tests/compliantBalloon.test.js`
plus anatomical contact, shared-axis, catheter/sheath and vessel regressions.
Tests cover axis deflection, opposing wall loads, deflation recovery, monotonic
free growth, volume feedback and rollback. The browser preview at
`/tests/compliantBalloonPreview.html` remains an isolated geometry/sampling demo;
the main simulator runs the coupled mechanical model.

## Fluoroscopy mask regression

`balloonProjectionMaterial.js` writes **positive RGB** into the device projection
target. The display shader derives attenuation from RGB and does not use alpha
as its signal; a black transparent material made a filled balloon invisible.
`/tests/balloonProjection.browser.html` reproduces that failure on the GPU,
then tests the production material with the production display-mask decoder,
including disappearance after deflation and reappearance after reinflation.

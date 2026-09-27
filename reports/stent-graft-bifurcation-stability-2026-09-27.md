# Progressive bifurcation release stability — 2026-09-27

## Observed session

Right access, guidewire 418 mm, Endurant IIs 23/14 × 103 mm, delivery 295 mm,
sheath withdrawal 65.4 mm (51%), fixation retained. Physics was blocked.
The archived exception originated in `kirchhoffSharedAxisWallFriction.makeRecord`:
`Cannot read properties of undefined (reading '56')`.

## Causes and changes

1. The uncovered delivery core uses a contact-radius view of the catheter.
   Friction looked up that object with `materials.indexOf`, returned -1 and
   indexed a nonexistent reference frame. Resolve the mechanical owner by ID;
   retain the 0.8 mm core radius and the actual shaft's orientation/friction.
2. While fixation was retained, each rod displacement rewrote the packed
   positions of partially released cloth. Cloth contact moved the rod, which
   moved the contacting cloth on the next step. Exposed rows now retain their
   release position and orientation; covered rows follow the delivery path.
   Both must be retained: parallel-transporting a new frame across the moving
   cover boundary still rotated already exposed rings.
3. Commanded translation and roll transport retained release frames with the
   assembled graft until detachment. Re-covering permits the cloth to follow
   the shaft again. A separate dirty flag refreshes geometry without erasing
   which rows have been exposed.

The captured crown potential and ipsilateral fabric contacts remain active.
This is a stabilization of the existing reduced deployment model, not a full
finite-element metal/fabric constitutive solver.

## Validation

- 73 focused tests pass (friction, release stability, capture, partial threading,
  rotation, displacement, withdrawal, limb release, ring kinematics, continuity).
  The six release-stability cases run for both 23 mm and 28 mm main bodies.
- Both new held-release/shaft-oscillation regressions fail on the previous code
  and pass on the corrected code.
- Build passes (existing large-bundle warning).
- Browser integration on aneurysm anatomy: wire ~419 mm, delivery ~296 mm,
  continuous sheath withdrawal 0–100 mm with crown retained. Passed through
  65 mm and bifurcation with no terminal block. Four recovered numerical
  substep failures were recorded; this is not a claim of zero Newton retries
  or 60 Hz under graft contact.

The unconstrained diagnostic fixture starting abruptly at 50 mm release is a
stress case, not the browser trajectory. Its baseline failed at 64.2 mm and
corrected version at 89.6 mm. No claim is made that this removes all possible
contact convergence failures.

- A second browser run with the 28 mm main body completed 136 mm cover travel,
  fixation release and 30 additional committed steps. UI confirmed 100% and
  implant detached. Numerical substep retries still occurred.

- The same full cycle also passes with the reported **23 / 14 × 103 mm**
  main body: guidewire ~418 mm, delivery ~296 mm, cover 136 mm, fixation
  detached, 30 further committed steps, no terminal block. UI confirms 100%.
  The browser harness defaults to 23 mm; `?diameter=28` selects the other case.

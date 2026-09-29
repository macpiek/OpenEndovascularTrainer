# Continuous stent-graft release — 2026-09-29

The release solver now starts from the preceding committed cloth pose, relaxes toward the constrained release shape, and limits the displacement of the assembly with a common continuation parameter. This runs before scaffold construction and collision snapshots, so visible and contact geometry agree. Branch roots remain bound to the shared sewn outlet; captured roots retain their fixed metal-arm length. The implant completes deployment only after the final motion-limited transition. Stationary settled poses remain cached.

## Measurements

Endurant IIs 23 mm fixture, 60 Hz, sheath withdrawal 12 mm/s (0.2 mm per step), including the bifurcation:

| Maximum displacement per step | Before | After |
| --- | ---: | ---: |
| Stent wire | 7.416 mm | 0.746 mm |
| Visible sewn cloth | 7.379 mm | 0.400 mm |

The cloth continuation speed is 24 mm/s. This is a reduced relaxation model, not a calibrated dynamic nitinol material model. The numbers are from the reproducible fixture, not a replay of the earlier user scene. Suprarenal crown release remains a separate mechanism.

## Validation

78 tests passed: release continuity on both accesses, final release, pause and recapture, fabric length, sewn assembly and topology, scaffold wire length, capture, rotation, interaction, expansion, wall fit on plain and aneurysmal anatomy, and update caching. Production build passed (existing bundle-size warning). Browser preview completed 1000 steps and showed the same measured maximum displacements.

Preview: `tests/stentGraftReleaseContinuity.browser.html`.
Screenshot: `screenshots/stent-graft-release-continuity-2026-09-29.png`.

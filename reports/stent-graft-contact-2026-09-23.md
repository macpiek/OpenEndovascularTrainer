# Stent-graft contact and dimensions — 2026-09-23

## Captured session

Read the local failure archive without resetting the scene: 439 cumulative records.
26 new records from 2026-09-22 21:45–21:52 UTC: 13 recovered and 13 terminal.
The latest terminal report was left access, wire 274.27 mm / catheter 219.40 mm.
It reported 94 Newton iterations, 451 factorizations and about 1222 ms solver CPU.
Replaying it without graft contact converged in 4 iterations: cloth contact is the
immediate cause. This is different from the older 250.8 mm guidewire snapshot,
whose axis was already crossing the vessel between sampled contact sites.

## Changes

- Full spatial finite-radius vessel/segment queries in the real-time solver.
  Sampled-field clearance cannot suppress this check.
- 0.15 mm fabric quadrature for non-owning tools; the owning-lumen recovery guide
  retains its existing contact quadrature. Continuous fabric crossing checks stay on.
- Bounded local indentation for exterior wires/catheters at vessel-apposed fabric,
  with a restoring spring. Reject/reduce an indentation that would cut another
  portion of the same rod. Both access views share committed indentations.
- Smooth complementary branch roots and varying radii replace cylinder shelves.
  Gate lengths 74/84 mm, gate diameters 12/14 mm, crotch widths and leg tapers
  follow the catalogue. Main covered lengths/proximal/distal combinations were
  already consistent with the IFU.

## Sources and limits

[Endurant II/IIs IFU, tables 2–3](https://www.accessdata.fda.gov/cdrh_docs/pdf10/P100021S063D.pdf)
confirms body sizes. [Medtronic Aortic Product Catalogue, pp. 15–18](https://www.medtronic.com/content/dam/medtronic-wide/public/western-europe/products/cardiac-vascular/cardiovascular/aortic-stent-grafts/aortic-product-catalogue.pdf)
provides dimensions A–F. Crotch shape, compliance, stent rings and nose are reduced
procedural approximations; this is not manufacturer CAD or calibrated nitinol FEM.
The separate limb remains an 80 mm training model, not an IFU catalogue replica.
The contrast model still uses ideal sealing and does not model the new local gutter.

## Replay results

Replays keep their old implanted surface: new nominal bifurcation geometry is not
substituted into archived states. Wall time below is Node CPU/wall replay time,
not browser Hz and not directly comparable with archived browser CPU timing.
Each replay has a 15 s diagnostic budget; timeout is not a solver convergence result.

| Time UTC | Old iterations / factors | New status | New iterations / factors | Replay ms |
|---|---:|---|---:|---:|
| 21:50:39.912 | 123 / 540 | line-search | 180 / 1654 | 7819 |
| 21:50:51.063 | 653 / 2715 | line-search | 171 / 1338 | 6189 |
| 21:51:04.512 | 69 / 373 | converged | 33 / 150 | 728 |
| 21:51:11.775 | 125 / 386 | diagnostic-time-limit | None / None | 15078 |
| 21:51:16.465 | 155 / 446 | line-search | 210 / 2332 | 8208 |
| 21:51:19.303 | 96 / 423 | converged | 13 / 298 | 373 |
| 21:51:22.113 | 95 / 544 | converged | 12 / 228 | 328 |
| 21:51:25.596 | 96 / 423 | converged | 13 / 298 | 387 |
| 21:51:32.919 | 104 / 326 | converged | 15 / 260 | 338 |
| 21:51:36.916 | 140 / 615 | converged | 11 / 218 | 291 |
| 21:51:39.764 | 94 / 451 | converged | 16 / 318 | 402 |
| 21:51:50.679 | 104 / 326 | converged | 15 / 260 | 395 |
| 21:51:54.961 | 94 / 451 | converged | 16 / 318 | 547 |

The latest captured failure also passes 24 successive steps including 2 mm of
withdrawal. The legacy vessel-crossing snapshot is rejected atomically, preserving
its original state; it is not silently moved through the wall.

## Verification

- 71 stent-graft tests passed before the final bounded-indentation guard; targeted
  affected tests rerun after it (including apposition, archive replay, access wakeup).
- 52 focused physics/archive tests passed.
- Browser preview: four release stages, Endurant II 166 mm, 32/20 mm, PASS;
  inspected the bifurcation and gate visually.
- Production build passed (existing large-chunk warning).
- Baseline failures outside this change: old anatomy replay mesh identity and an
  outdated sampled-crossing fixture also fail without these changes; a fixed-step
  test VM lacks a stentGraftSystem stub. No product workaround was added for these.

## Applied workspace verification

Applied changes to the main workspace and rebuilt successfully. After the user-authorized
reload, the aneurysm anatomy renders, both tools are at 0 cm and both physics
loops report 60 Hz at rest. The persistent failure archive is still available
with 488 records (the replay analysis above used the earlier 439-record capture).
The final five apposition/session regression tests passed after the last edits.

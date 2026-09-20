# Predictor feasibility projection — rejected

Private sequential wall/length projection after velocity prediction, bounded to .5 mm correction, no change to inertia reference, reactions, velocities or friction history; all final physical certificates unchanged. Tested 2/4/8 sweeps in five alternating warm replays. Step 777 improved from 279 to 12 LU, but withdrawal step 4895 worsened from 114 to 326/341/134. Full cycle therefore used four sweeps only during advancing feed.

Full ordinary-mesh cycle completed 5757 steps, 28739 LU / 16554 Newton versus 28832 / 16468 reference. Mean Node step worsened 12.647 -> 13.154 ms, max 229.723 -> 400.721 ms. Shape RMS 1.60–2.94 mm, largest difference 36.22 mm. No performance gain.

With contactMaxSpacing=10, tipRefinementAhead=40, the candidate failed catheter insertion at 605.80 mm, unsupported-direction / crossed vessel surface. That variant additionally skips applying a zero correction to avoid unnecessary frame reorientation.

13 focused tests passed including cancellation, immutable physics, undefined wall-normal rejection and direction gating. Reverted prototype and tests; never enabled in the app. Next: prevent geometrically unsafe chord coarsening before remeshing, rather than correcting a predicted state after that merge.

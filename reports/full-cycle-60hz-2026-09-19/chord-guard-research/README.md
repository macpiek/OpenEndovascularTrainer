# Collision-aware mechanical coarsening — research, reverted

Before merging nodes, veto the replacement capsule if it approaches any BVH triangle within radius minus 1e-6 mm. Shapecast checks both finite triangle/segment distance and interior crossings, handles world origin and clips the sheath interval. This only vetoes coarsening; it is not an inside classifier.

With contactMaxSpacing=10 and tipRefinementAhead=40, the full 5757-step cycle completed (the earlier unguarded setting failed). Some withdrawal states used ~125 nodes instead of ~150. However LU increased 28832 -> 39660 and Newton 16468 -> 17815. Node mean 12.647 -> 14.426 ms, P95 23.443 ms, max 384.865 ms. Trajectory RMS 1.06–4.46 mm, maximum 44.62 mm; physical certificates remained within limits.

A more aggressive variant (contact20/max30/shape.3/arc.005, same guard) failed catheter insertion at 614.47 mm: unsupported-direction / crossed vessel surface.

14 targeted tests passed, including the case where endpoint and triangle-edge distances miss an interior crossing and a shape-budget-approved chord cuts through a finite wall patch. The guard improves one coarse configuration's robustness but does not produce a speed gain. Prototype and tests reverted; implementation retained as patch for future discretization work. No app defaults changed.

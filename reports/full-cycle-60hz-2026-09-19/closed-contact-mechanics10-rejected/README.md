# Rejected coarse contact mechanics on the closed anatomy

Only the mesh budget changes: contactMaxSpacing 10 mm, shapeTolerance 0.3 mm, maxArcLoss 0.01, maxTurn 0.4 rad. Remaining reference settings, 2 mm discrete contact sampling, exact axis guard, material profiles, feed rates and physical residual thresholds stay unchanged. No app default was changed.

All 5757 steps completed. Mean 36.738 ms versus reference 33.915; P95 108.137 versus 93.964; max 3546.183 versus 1612.864. Movement LU 131064 versus 87269. The wire-in mean improves 51.521 to 39.076 ms, but catheter-in worsens 42.360 to 51.633 ms and catheter-out 29.245 to 40.001 ms. A smaller linear system did not compensate for extra nonlinear work.

96 saved shapes: maximum trajectory difference 369.25 mm; catheter-in RMS 139.11 mm. Zero exact axis intersections and zero far-outside/beyond-bounds lumen warnings in 44,880 samples. Continuous capsule audit still finds overlap up to 0.40984 mm (reference 0.10964 mm). Snapshot audits do not prove swept containment. Finite states and small discrete contact residuals alone are insufficient physical evidence.

This budget is rejected. Keep the previous default mesh. Node timing excludes UI/rendering and does not establish 60 Hz.

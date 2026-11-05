# Balloon runtime performance

The production changes preserve the mechanical mesh, native material laws,
friction, force/length tolerances, time integration and inflation subdivisions.

- Coverage and outer-material intervals are reused only inside a private
  timestep. Feed creates a new cache. Cancellation discards it; custom observers
  or geometry evaluators retain the uncached path.
- Consecutive exact BVH queries at the same capsule endpoint reuse the finite
  triangle result. A different hint must preserve the winning face/tie order;
  a new BVH invalidates the record. No coordinate rounding is used.
- Balloon rays stop at the free membrane radius plus the existing 0.03 mm
  clearance. An exact empty-ball query skips radial rays only when the entire
  ring is clear. Contacting rings retain all 32 directions and 81 axial rings.
  This projects an already accepted mechanical axis; it does not replace the
  solver's inside/crossing certificate. Contained-volume feedback is unchanged.
- Vessel deformation reuses sparse spatial buckets instead of allocating
  4096 sets on every publication. Every contact patch is retained.

## Reproduce

```sh
node scripts/benchmark-balloon-runtime.mjs /tmp/balloon-runtime.json
node --test tests/balloonRuntimeOptimization.test.js tests/scalarMeshBvhNeighborhood.test.js
```

The benchmark runs reference/optimized warmups, then optimized/reference
measurements on the same captured iliac bend. It asserts identical accepted
positions, frames, velocities, reactions and wall patches. It separately compares
every balloon surface vertex and its volume, including deformed-wall projection.
Reported step times cover the entire requested timestep, not just its final
substep. Browser rendering, flow and UI are excluded from solver timing.

Recorded Node 24 results in `reports/balloon-runtime-performance/paired.json`:

| Work | Reference mean | Optimized mean |
| --- | ---: | ---: |
| Complete loaded balloon step (8-step stress continuation) | 83.52 ms | 77.98 ms |
| Balloon surface / contained-volume sampling | 7.50 ms | 3.24 ms |

This is a 6.6% solver reduction and a 56.8% surface-sampling reduction on this
machine. **The loaded stress case does not meet the 16.67 ms / 60 Hz budget.**
These numbers are CPU measurements, not browser FPS. No quality setting or
physical rate has been lowered to present a 60 Hz result.

Larger continuation increments, contact condensation and analytic elimination
were explored but not enabled: their long replay trajectories differed from the
reference. They require separate mechanical validation before production use.

The additional Lunderquist wire/catheter/sheath replay also preserved positions,
frames and velocities exactly. After warmup its median complete-step time was
11.58 ms without coverage reuse versus 10.94 ms with reuse; 17/20 steps met the
60 Hz CPU budget in each run. Difficult outlet contact pivots still produced
outliers (optimized p95 113.25 ms), so this is not a sustained 60 Hz guarantee.
See `reports/balloon-runtime-performance/triaxial.json`.

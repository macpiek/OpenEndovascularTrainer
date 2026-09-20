# Closed-anatomy reference: full-cycle CPU profile

Command: `CPU_PROFILE=1 node scripts/physics/profile-shared-axis-full-cycle.mjs /tmp/oet-closed-cpu-profile-default`.

All 5757 movement steps completed. No compliance, condensation or stagnation-residual override was enabled. The saved shapes are byte-identical to `/tmp/oet-axis-no-early-full/shapes.json`, SHA-256 `f16483a72982ec1f17990aeff05357fbf5c487c81959d1f34e58be30dca6b5bb`.

The inspector samples the loop after anatomy loading. It includes capture/serialization bookkeeping between steps and profiling overhead, so this is hotspot evidence, not a new uninstrumented performance result or a 60 Hz claim. Inclusive percentages overlap and must not be added.

| Operation | Inclusive sampled CPU | Self sampled CPU |
|---|---:|---:|
| Complete physical advance | 90.12% | — |
| Linear direction solver | 40.21% | — |
| Nonlinear assembly | 34.97% | 2.70% |
| Constraint-row assembly | 25.42% | 5.29% |
| Vessel discovery sampling | 16.15% | 0.40% |
| Certified inside capsule queries | 10.54% | 0.10% |
| Active-basis preparation | 9.95% | 5.47% |
| Band-LU solve wrapper | 9.07% | 0.56% |
| Scalar BVH closest-point queries | 8.21% | 3.92% |
| Prepared matrix assembly | 5.33% | 3.72% |
| Material tangent assembly | 4.55% | 0.98% |
| Linear workspace construction | 4.37% | 4.03% |

The leading anonymous WASM kernel (`wasm-function[4]`, 5.56% self) is called by the band-LU solve wrapper, confirmed from its sampled parent stacks. It is not the rod material kernel.

Mesh-identity hashing accounts for 4.92% of samples, but belongs to slow-state debug capture outside the measured physical advance. Optimizing that cost would not reduce the reported physics-step duration. Garbage collection accounts for another 2.80% of sampled time.

The next promising targets are exact contact-query reuse and active-basis/workspace preparation, rather than further material-kernel work. Any geometry shortcut must preserve nearest-feature discovery, sign/containment checks and invalidation when the mesh or sample grid changes. The current retained empty-ball certificates already skip clear segments; savings must cover the expensive near-wall case as well.

Raw profile, summarized hotspots, source hashes, complete steps, snapshots and slow incoming cases are archived alongside this report. The goal of constant 60 Hz remains unachieved.

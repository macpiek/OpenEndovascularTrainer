# Composite material refresh cache — frozen handoff

The present bottleneck was constitutive compilation, not the new band solve. The frozen candidate removes generic deep-copy/compile work inside profile quadrature and avoids recomputing unchanged material supports. It changes neither the Element energy nor Chain assembly/solve.

## Measured result

Six paired AB/BA measurements after three warmups per insertion; immutable profile adapters constructed once; identical topology, fixed optional coordinates, exact mandatory boundaries, synthetic 3D centerline, independent material maps/spins, and identical per-layout workspace policy. Pose/input hashes match in every pair. Source hashes stable. Units ms, medians:

| Catheter mm (wire318) | Mesh/material oracle → cached | Full topology+Mesh+workspace+assembly+solve | Speedup |
| --- | --- | --- | --- |
| 9 | 12.559 → 1.127 | 13.761 → 2.158 | 6.38× |
| 160 | 15.251 → 1.434 | 15.923 → 2.058 | 7.74× |
| 310 | 24.406 → 1.423 | 26.918 → 2.530 | 10.64× |

All18 measured pairs converged in both arms. Median fresh evaluations fall916→10,1936→26,2143→93. The second pass visits already seen retract supports; per-row data distinguishes hits/fresh evaluations. The first forward measurements are also retained individually. Cache sizes after the whole short sequence are19/31/52 supports, bounded at512 per adapter; constant spans keep just one immutable compiled material and do not allocate support keys. No dt/history commit, dynamics/contact/anatomy, state-transfer or FPS claim is made.

The cold warmup at catheter310.017mm has mandatory profile boundaries0.017mm apart. Cached direction residual1.80328698e-7 fails the unchanged1e-7 tolerance; oracle residual6.73142796e-8 passes. Paired normalized E/g/H differences are2.71e-14/3.41e-13/1.42e-13 and increment difference1.42e-9. This arithmetic-sensitive case is explicitly FAILED, never counted as an executed step; the full warmup ledger is retained. A runtime integration still needs mesh quality/conditioning handling for nearly coincident mandatory boundaries. We did not relax the solver tolerance or edit Chain.

Across all measured+warmup pairs maximum normalized differences: E2.78e-14,g3.41e-13,H1.46e-11,increment1.42e-9. The test suite also compares full operators under unequal dsDx and retained multi-turn winding, using an explicitly identical diagonal1000 to condition those large-load parity fixtures; the performance benchmark retains diagonal3.

## Contract and implementation

- MaterialCache.profileTool creates the existing distally increasing-label adapter once. Fast constitutive metadata is enabled ONLY by exact immutable built-in object identity. A familiar id or equal samples does not authorize it. Unknown/mutable profiles always fall back to Mesh's original adaptive integration without support reuse.
- Glidewire known constant core/shaft follow exported24/50mm boundaries. Its varying EI/GJ transition uses the SAME adaptive Simpson error criterion and accepted positive Boole formula as Mesh, with the same spatial subintervals and sampled component values. Only the zero components, repeated deep copies, and repeated SPD compilation are removed. Tests verify bit-identical transition tensors, evaluation count, maximum depth and estimated error.
- Berenstein's immutable constant K and piecewise degree5 intrinsic profile use positive six-point Gauss quadrature split at all exact8/10/16/18mm interfaces. Both weighted intrinsic and positive mismatch energy are polynomial of degree≤10, so the rule has zero mathematical truncation error. It retains the caller's tolerance and falls back below a conservative floating arithmetic floor; it does not claim certified floating-point bounds. Mesh itself also reports certified:false.
- Explicit copied constantTool spans support full anisotropic bend/twist tensors and positive energyOffset. Declared jumps remain topology boundaries. General mixed-tensor supports retain original positive quadrature and anchored mismatch, avoiding cancellation of large intrinsic offsets. Returned compiled arrays are frozen, so clients cannot poison the cache.
- Cache identity/revision is private per immutable sampler. Exact support keys include id, spatial support and every quadrature split, nominalLength, dsDx, insertion/material interval, declared breakpoints and all tolerance/maxDepth options. Keys are never rounded. Reused supports retain immutable materials but regenerate current section indices. LRU bounds per profile preserve hot transition cells. clear() drops the adapter registry and statistics; previous adapters then safely use the oracle until recreated.
- Parent-approved Mesh change is ONLY optional materialIntegrator dispatch at the two hinge/boundary call sites. Callback receives the seven original arguments plus the original integrator as fallback; null retains the untouched default path.
- MeshUpdate material-only refresh preserves the same accepted positions, coordinates, reference frames, independent spins, winding arrays, layout, cell objects and map objects. It compiles a validated draft before committing; failures leave accepted state unchanged. It updates each tool's independent material labels/dsDx and discards stale clearance approvals. It does not advect material velocity/spin history or perform state transfer.
- Missing exact tip/material/sheath boundaries or changed tool ownership/domain yield candidate-needs-state-transfer with proposed exact coordinates and NO accepted-state mutation. A moving distal boundary in every primary benchmark remains this candidate. The current task does not silently claim a persistent runtime remesh. True fixed-coordinate material refresh is tested separately on retained real profile shaft spans.
- Nominal midpoint-to-midpoint Voronoi weights and separately reported unassembled boundary half-cells are preserved, including floating arithmetic length differences; no endpoint weight extension or stiffness blending is introduced.

## Validation and integration

67/67 tests passed across Topology, Element, Chain, FastElement, Mesh and the9 new MaterialCache/MeshUpdate cases. New cases cover advance/retract real profiles at9/160/310mm, unequal maps, full E/g/H/direction, independent midpoint energy integration, anisotropic jumps/mismatch/offsets, mutable-profile fallback, bounded cache/error-budget invalidation, persistent history object identity, moving-boundary candidates and atomic failed refresh. Final syntax and git diff --check passed. No browser or server action; no production app build or runtime integration by this worker.

Run: node --test --test-concurrency=1 tests/kirchhoffCompositeMaterialCache.test.js tests/kirchhoffCompositeMesh.test.js
Benchmark: node scripts/physics/benchmark-composite-material-cache.mjs /tmp/oet-composite-material-cache.json

Root may register the new test and integrate the optional cache after review. Root must supply validated remeshing/material velocity/spin/frame history transfer and physical contact admission before treating moving-boundary candidates as accepted dynamics.

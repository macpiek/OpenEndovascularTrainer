The 6.114016522 mm witness is an incorrectly selected catheter endpoint, not evidence of a phantom initial overlap created by a rotating tangent plane. The frozen implementation adds a vessel point constraint even though that body's vessel capsule range is empty. This is a P1 contact ownership/domain defect in `appendKirchhoffSplitPointWalls`.

At step 3698, catheter insertion 5.2 mm, the body has `activeStart=0`, `activeEnd=17`, `collisionStartSegment=17`, `collisionEndSegment=16`, and `sheathMaterialEndNode=17`. `PigtailCatheter.syncXpbdBody` explicitly sets the vessel collision range and sheath material endpoint. `setCollisionRange` represents an empty range as `end=start−1`. Existing World capsule preparation returns for this condition. The new point loop instead computes `start=17`, `end=min(17,16+1)=17`, then executes `node<=end` once. There is no eligible vessel capsule for this point to supplement.

The exact certificate witness is `split-point-wall`, side 1, catheter node 17, material coordinate 5.2 mm, radius 0.8333333134651184 mm. It has three translational gradients at DOFs 102–104, zero physical normal multiplier, zero bias multiplier, and zero compliance. It is an endpoint sphere; no capsule foot or stencil migration is involved in this row.

| Quantity | Exact saved/control value |
|---|---:|
| Time-start position | [−88.52568054199219, −444.8899230957031, 27.56562614440918] mm |
| Candidate position | [−88.42401123046875, −444.47894287109375, 27.473155975341797] mm |
| Position displacement magnitude | 0.4333498887653861 mm |
| Actual time-start sphere gap | −7.934613513920906 mm |
| Time-start gap via degenerate capsule/BVH | −6.329191000640638 mm |
| Actual current sphere/BVH gap | −6.114016522283371 mm |
| Time-start point evaluated on current tangent plane | −6.328683424014381 mm |
| Current Jn | [0.0048236909309131775, 0.30700509541760534, −0.9516956464087825] |
| Current physical velocity | [12.20376205444336, 49.32834243774414, −11.097631454467773] mm/s |
| dt·Jn·v, dt=1/120 s | +0.2147040599491928 mm |
| Jn·(x−x_start) | +0.2146669017310101 mm |
| Unilateral history violation for λ=0 | 0 mm |

The physical row is `max(0,startGap)+dt·Jn·v = +0.2147040599491928 mm`. Because λ is zero, it must satisfy a nonnegative inequality; it is not a loaded equality with a residual of 0.214704 mm. The certificate correctly contributes `max(0,−historyGap)=0` for this row. The saved probe ledger calls the signed expression `historyResidual`; its unilateral violation must be computed using the multiplier, as the verification script does. Across all 95 observations of this endpoint the multiplier remains zero. The physical-end pose and certificate pose are identical, so the 6.114 mm is already present before bias. Bias then sees the full negative geometric gap and fails before accepting a correction.

The hypothesis “actual start gap is nonnegative but tangent extrapolation invents a negative start gap” is false for this worst witness. Both independently queried time-start paths report an outside point. The sphere query initially uses `centerline-estimate` (conservative, branch 1176, face −1); the current query uses `sparse-sdf-bvh` (branch 1176, face 38644). A degenerate capsule forces the BVH validation path at the exact same start point and also finds face 38644, with gap −6.329191001 mm. Thus the approximately 1.6 mm difference between the initial sphere estimate and current-plane extrapolation is principally a query-source difference here. Sphere normals differ by only 1.26737°. The current-plane start differs from the start BVH gap by about 0.000508 mm. Neither result supports a six-millimeter newly created penetration.

For completeness, independently querying adjacent capsule 16→17 gives current gap −7.412152319 mm, foot t=0, face 38611. This capsule is outside the declared vessel collision domain too. It must not be interpreted as a missed eligible wall contact. The small 3.71582e−5 mm difference between dt·Jn·v and Jn·position displacement is recorded, but cannot account for this failure.

The mechanical contract to repair is: endpoint sphere supplements must be drawn from a nonempty, clipped set of eligible vessel capsules and respect the same material ownership as that set. An empty segment range or an empty intersection with active segments must emit no point rows. Preserve both endpoints of actual eligible capsules; changing the inclusive loop alone would lose their final endpoint. This finding does not justify lowering the geometry certificate gate or changing μ/CCD. The proposed geometric normal law with a known actual initial overlap may deserve a separate audit, but this witness does not establish that it is needed to cure the observed 6.114 mm failure.

Validation used one initial replay whose optional post-solve callback failed on `node+1` at the final active endpoint. Its standard report and candidate pose survived. The parent explicitly authorized one corrected replay on the same frozen runtime. The corrected callback was first checked separately with final node 17 and an adjacent capsule 16→17; it persists the passive ledger before optional queries. No additional anatomy dynamics runs were made. Both replays have exactly equal joint-motion diagnostics/candidate snapshots and final fingerprints: physical 30 passes, 2213 factors, bias 1 pass, history 0, raw 6.114016522283371 mm. The independent unequal-static/kinetic-friction guard remains separate.

Runtime source provenance: `/tmp/oet-raw-wall-audit-7keu604y`, copied from root `/901c`. World SHA-256 is `e993cec61125fe003e58d3bd081b24b15ec91e158351427319b2216900522f62`; original SplitMotion SHA-256 is `0584fa9cf1c8b54eb7dfd156916f9370c5d33775e4a45a392f6caf52d50bb3db`. Of the supplied report's 56 hashes, 55 match. The sole World difference adds physical/bias failure snapshots; no equations change. Passive hooks modified only this temporary copy's SplitMotion and harness. The fixture records all 124 frozen source hashes, the two instrumented hashes, and both anatomy asset hashes. Query-only verification checked them successfully.

Artifacts in this reports directory:

- `split-anatomy-wall-worst-fixture.json`: exact start/current row, body ownership bounds, adjacent capsule, 95-row trajectory and full provenance.
- `query-split-anatomy-wall-snapshot.mjs`: executable geometry/algebra replay and direct characterization of the empty-range production selector. Runs no World timesteps.
- `split-anatomy-wall-verification.json`: PASS result, fresh sphere and BVH queries, exact algebra and the spurious row emitted by the frozen production function.
- `split-anatomy-wall-first-replay.json` and `split-anatomy-wall-corrected-replay.json`: preserved full standard reports.
- `probe-split-anatomy-wall.mjs`: corrected full-replay instrumentation, intended for a pristine frozen copy matching its manifest.
- `split-anatomy-wall-callback-check.txt`: endpoint callback preflight result.

Recheck the saved witness without another dynamics replay:

```sh
node /Users/macpiek/.codex/worktrees/0827/OpenEndovascularTrainer/reports/query-split-anatomy-wall-snapshot.mjs
```

The 47 MB full passive ledger is retained at `/tmp/oet-raw-wall-audit-7keu604y/corrected-audit.json`. No root/model source, gate, coefficient, or runtime behavior was edited by this audit. The remaining geometry after a selector fix has not been dynamically replayed here; this report does not claim the fix alone will certify the anatomy step.

Parent follow-up: the main task reports that root now intersects the capsule interval `[max(activeStart,collisionStart), min(activeEnd−1,collisionEnd)]`, skips an empty intersection, and only then includes endpoints through `lastSegment+1`. It reports three coverage regressions passing (native World empty range outside the vessel, both endpoints of one eligible capsule, disjoint intervals sharing a node), plus the three wall oracles. These post-fix checks were performed by the main task, not rerun by this audit. The fixture deliberately preserves the pre-fix defect for verification. The ghost-initial-gap hypothesis is explicitly rejected for this witness; no broader normal-law conclusion follows from it.

# Continuous vessel-contact performance — 2026-09-23

Three archived steps without a stent graft, three repetitions per version. Values are median Node step times, not live browser physics Hz. Both versions use full finite-radius collision discovery and identical requested force/length tolerances (1e-4 / 1e-3). Historical mesh-specific clearance certificates are discarded and rebuilt. Baseline is the code deployed before this optimization.

| Fixture | Before ms | After ms | Speedup | Factorizations | Full assemblies |
|---|---:|---:|---:|---:|---:|
| anatomy-berenstein-feed-138.67-live-cycle.json | 748.1 | 443.7 | 1.69× | 499 → 219 | 58 → 26 |
| anatomy-pigtail-wire-withdraw-200.27-incoming.json | 1066.9 | 304.8 | 3.50× | 1437 → 331 | 148 → 48 |
| anatomy-wire-569.80-poor-prediction.json.gz | 534.8 | 96.8 | 5.53× | 489 → 31 | 71 → 12 |

## Changes

- Bounded exact-query cache and conservative whole-segment clearance certificates for static vessel geometry. Reuse accounts for both endpoint displacements, the current radius and geometry revisions. Deforming graft geometry remains uncached.
- Existing sampled clearance can skip a full segment query only after subtracting the maximum distance to the nearest sample (half the spatial grid interval).
- Non-crossing finite-radius overlaps are collected across the rod before restarting assembly. Centerline crossings still stop immediately.
- Continuous contact discovery uses 10% of the requested length tolerance, capped at 0.0001 mm (0.1 micrometre), instead of endlessly adding near-identical constraints for sub-budget penetration. This changes discovery precision; it is not a bit-identical trajectory optimization.

## Verification

- 52 focused tests passed: cache invalidation and accumulated motion, capsule/crossing detection, transactional state updates, apposition, and 24-step graft withdrawal replay.
- Independent, uncached exact segment/triangle checks on every accepted exposed tool interval: no centerline crossings; maximum capsule overlap below 0.0001 mm in all benchmark cases.
- The legacy already-crossed graft snapshot still fails atomically. Its first reported failure can change because contact publication is batched; the regression checks the pre-existing real crossing and unchanged rejected state.
- Production build passed, with the existing large-chunk warning.
- A broader archived certified-sample test still has an old vessel mesh identity mismatch; checked against the unchanged baseline. No weakening of replay mesh validation.

Run `npm run benchmark:continuous-contacts` to reproduce the optimized measurements. These difficult isolated steps do not establish 60 Hz during continuous user interaction.

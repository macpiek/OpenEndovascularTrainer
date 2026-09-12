# Exact closed time-PT physical force map

Implemented only in the ForceMap workspace/evaluator of `kirchhoffCompositeJointSurfaceMotion.js`, its corresponding test, and this report. The original instantaneous-rate and finite-increment paths before `createCompositeJointSurfaceForceMapWorkspace` are byte-identical to the frozen root source. No root edits or commits.

## Derivation and retained physical semantics

For old/current unit tangents a,t, with c=a·t and v=a×t, the shortest time-PT rotation is R=I+[v]+[v]²/(1+c). Its spatial angular differential is

`omega_j = t × delta_t_j − t ((a × t) · delta_t_j)/(1+a·t)`

where `delta_t_j = (I−t tᵀ) delta_chord_j/L`. Current own-theta contributes the column `t`. Endpoint 0/1 chord variations have opposite signs, so the three endpoint-1 columns are computed once and negated for endpoint 0. This retains the axial spin connection of the chosen time-PT chart. For an independent sign witness a=z and t=Rz(phi)Ry(beta)z, R=Rz(phi)Ry(beta)Rz(−phi) gives omega_phi=z−t and omega_phi·t=cos(beta)−1: the connection sign is negative.

The full physical field is `centerRate_j + omega_j × (queryPoint−center)`. Projecting on the normalized/orthogonalized current query axes and applying inner/outer signs produces the original physical 7/14 packing. First-order AD differentiates the explicit formula and all current configuration, own-foot, common-point, normal and tangent arguments. No numerical differences, repeated unit-rate full evaluation, stiffness change or approximate transport enters runtime.

The circular affine surface cancels reference-director gauge and current angle from the physical map. Both remain validated, together with previous endpoints, ownership, frames, current geometry, support coordinates/traces, finite inputs and the original antiparallel chart guard. Full/value output shape and validity flags are unchanged. Returned arrays remain owned. New full/value B and tool outputs are bit-identical to each other; agreement with the old trigonometric triad path is to floating-point roundoff, rather than bit identity. The old unit-frame acceptance tolerance remains unchanged.

Only ForceMap scratch fields `second` and `firstOnly` are removed. Public full/value evaluator functions, dimensions and returned data are preserved. In the paired two-tool benchmark, derivative arena storage falls from 4,587,520 to 884,736 bytes; first-order output nodes fall from 1,355 to 841, and all 212 second-order triad nodes disappear. Other providers' workspaces are unchanged.

## Verification

**57/57 PASS** in the frozen dependency snapshot: SurfaceMotion 33 (29 existing plus 4 new), JointLumenSurface 11, JointSurfacePullback 13. The existing tests include every configuration/query-column finite difference, moving detector chain rules, physical wrench with axial-connection removal, own spins, material feeds, value/full equality and recovery after rejection.

The full original ForceMap triad/Hessian evaluator and arena helpers are frozen inside the corresponding test as an independent oracle, identified by the original source SHA. Across 48 deterministic one/two-tool fixtures, with arbitrary nonparallel frames, parallel/near-parallel cases, a near-antiparallel valid chart, independent gauges and unwrapped windings:

- Maximum |new B − old B|: 2.0961e−13.
- Maximum |new DB − old DB|: 2.3874e−12.
- Maximum |new omega − old omega|: 1.8799e−13.
- Dense independent shortest-arc quaternion triad kinematics, 168 physical columns: maximum omega discrepancy 2.3814e−9 and physical power discrepancy 2.7960e−9 with a central perturbation of 1e−6.
- 26 malformed cases in both full/value modes preserve exception type, message, code and transport metadata, with successful workspace recovery; both valid endpoint traces also match the oracle.
- Every own-theta DB column is exactly zero; independent gauge/angle changes preserve the new B/DB bit-for-bit.

Run the frozen tests with:

`node --test --test-concurrency=1 tests/kirchhoffCompositeJointSurfaceMotion.test.js tests/kirchhoffCompositeJointLumenSurface.test.js tests/kirchhoffCompositeJointSurfacePullback.test.js`

## Paired complete-call benchmark

Node v24.6.0, darwin/arm64, Apple M3. Each case warms each variant for 1,000 calls, then runs 21 paired blocks of 250 calls; the old/new execution order alternates each block. The same input fixtures and reused workspaces are used by both implementations. The measured region includes validation, full B/DB construction and owned return-array allocation; no forced GC. Value calls are separately labeled. All outputs are compared before timing. Full raw block timings, fixtures and runnable script are in the frozen bundle.

| Tools | Fixture | Order | Old median ms/call | New median ms/call | Speedup |
|---|---|---|---:|---:|---:|
| 1 | mild-current-root-fixture | full | 0.151249 | 0.076715 | 1.97× |
| 1 | mild-current-root-fixture | value | 0.029074 | 0.017379 | 1.67× |
| 1 | varied-nonparallel | full | 0.080844 | 0.049070 | 1.65× |
| 1 | varied-nonparallel | value | 0.028418 | 0.017255 | 1.65× |
| 2 | mild-current-root-fixture | full | 0.258621 | 0.112986 | 2.29× |
| 2 | mild-current-root-fixture | value | 0.056924 | 0.030858 | 1.84× |
| 2 | varied-nonparallel | full | 0.260911 | 0.112747 | 2.31× |
| 2 | varied-nonparallel | value | 0.057611 | 0.031116 | 1.85× |

The two-tool FULL calls are approximately 2.3× faster in this run (about 56–57% reduction). These are provider microbenchmarks; they do not measure a complete Step, browser frame time or the parent's first-P95 spike. Absolute timings depend on JIT and host load; paired comparisons apply within each fixture/order row.

## Frozen handoff

Bundle: `/tmp/oet-surface-force-map-closed-omega-final-8996`.

Base source SHA-256: `ec36b610557e1f47938be9f107bde40710a46f9dbce0a8dcb5bc368510b48c9f`.
Base test SHA-256: `7a3b3c41086c41a0801449c88e86fab4e807a75c908741083f853fb677fb03e6`.
New source SHA-256: `545c1b1b3fe25e6e045bf0745e1ed4c4a45d801640a4f03860083224320af92d`.
New test SHA-256: `667905134964e16342ef6815c0793aed6e4d19ee46b522d4697452073afceefc`.

`patch.diff` contains only the source, corresponding test and this report. `manifest.json` and `SHA256SUMS` identify the full frozen dependency/evidence payload. Apply against the specified root source/test base; no root mutation has been performed by this task.

## Root: pełny mały krok po integracji

[Kontrolowane porównanie całego dt](composite-joint-closed-omega-dt-benchmark.json) obejmuje przygotowanie historii materiałowej, wszystkie oceny, solve i commit. To 3 węzły, dwa narzędzia, jedna oryginalna próbka tarcia światła, mu=[.015,.006], dt=1/120. Każdy etap ma 40 par rozgrzewki i 100 naprzemiennych par pomiarowych; wariant bazowy zachowuje stary ForceMap z triadą/Hessianem, reszta operatorów i krok są wspólne.

| Etap | Mediana stare → nowe [ms] | Średnia stare → nowe [ms] | P95 stare → nowe [ms] |
|---|---:|---:|---:|
| Pierwszy obciążony dt | 2.100 → 1.713 | 2.196 → 1.790 | 2.740 → 2.256 |
| Kolejny obciążony dt | 2.141 → 1.814 | 2.277 → 1.872 | 3.016 → 2.364 |

Wszystkie pary zachowują dwa kierunki, sześć ocen (2 pełne i 4 gradientowe), te same długości prób i akceptację oryginalnych testów mechaniki. Maksymalna różnica liczby w stanach, historiach, siłach i bilansach to 3.388e−21. Bieżący wynik nie jest pomiarem World ani pełnej anatomii. Nie wyjaśnia również wcześniejszych skoków P95 w odrębnym pomiarze włączania trybu value; poprzedni wynik pozostaje zachowany. Nie wolno sumować przyspieszeń z różnych przebiegów.

Skrypt i metadane bazy: [benchmark](composite-joint-closed-omega-dt-benchmark.mjs), [hash źródeł](composite-joint-closed-omega-dt-baseline-source.json); rekonstrukcja bazy w `/tmp/oet-prepare-closed-omega-dt-benchmark.py`. Root zweryfikował wszystkie hashe frozen bundle i zastosował tylko deklarowane source/test/report.

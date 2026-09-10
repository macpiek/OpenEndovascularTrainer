# Sparse two-channel release measurement

The change is confined to `measureKirchhoffBiasReleases` in `src/physics/kirchhoffTwoChannelRelease.js`. Local reaction norms visit only nodes touched by the already-sorted gradients. A single six-double scratch vector replaces two whole-rod vectors per release. The two returned full correction vectors, final combined norm scan, gradient helper, history guards, units, output shape, ownership and snapshots remain unchanged.

Every gradient still updates the global correction immediately in the original order. Duplicate DOFs accumulate in the same order and Float64 precision. Per-release norms are evaluated after each touched node is complete; omitted untouched nodes contain exactly zero and cannot change a maximum. The old full typed array's ignored out-of-range writes are preserved. No force or Jacobian epsilon is introduced.

## Measured cost

Local function microbenchmark: two rods of 200 nodes each, 400 sparse retained reactions, activeStart 7/11, rotated frames, anisotropic inertia, duplicate DOFs and inactive/end-node samples. This measures this function's cost; it does not establish whole-step or browser FPS.

| Metric per call | Frozen baseline | Sparse measurement |
| --- | ---: | ---: |
| Median time | 14.252369 ms | 10.897010 ms |
| Minimum time | 11.003652 ms | 5.679294 ms |
| Maximum time | 83.714525 ms | 17.794069 ms |
| Float64 allocations | 802 full vectors | 2 full vectors + 1 six-double vector |
| Float64 bytes allocated | 7,699,200 | 19,248 |

Ratio of median times: **1.308x**, or **23.54% less time**. Float64 allocation falls **99.75%**. All nine paired rounds favored the sparse version; times varied substantially on the shared host, as the raw table shows. Other JavaScript object allocations in the unchanged gradient conversion are included in timings but not in the Float64 byte counter.

Host: Apple M3, darwin arm64, Node v24.6.0. The process warmed both variants with 24 interleaved call pairs, then measured nine AB/BA rounds of 20 calls per variant. Explicit GC occurred outside each timed interval; allocation and any incidental GC inside a call are included. Allocation instrumentation and byte comparisons ran outside timing. Timings are evidence, not a flaky speed threshold in the regression test.

| Round | First variant | Baseline ms/call | Sparse ms/call |
| --- | --- | ---: | ---: |
| 1 | baseline | 12.827260 | 10.897010 |
| 2 | optimized | 15.082702 | 10.854837 |
| 3 | baseline | 13.116719 | 8.516033 |
| 4 | optimized | 13.327758 | 7.766288 |
| 5 | baseline | 14.252369 | 11.158827 |
| 6 | optimized | 24.433088 | 11.478023 |
| 7 | baseline | 83.714525 | 12.344402 |
| 8 | optimized | 31.685383 | 17.794069 |
| 9 | baseline | 11.003652 | 5.679294 |

## Exact A/B checks

The checked-in reference is the literal baseline measurement plus unchanged private mobility/norm helpers from Release SHA-256 `8b38077b5d2451d70251fd976966d6ca268ee2b5a92086b3bf900d1638d124c6`. Unchanged public gradient/history helpers are shared, isolating the one optimized function.

Four dedicated regression tests compare the whole logical output graph, every number's IEEE754 bytes, both full backing buffers, types/property order/array holes/shared references, and the entire input/retired state. Input object and array identities are also checked directly. Cases cover 16/21, 200/200, 201/320 and 503/601 node layouts; up to 400 sparse releases; nonzero active starts; rotated frames; anisotropic/hard/zero mobility; duplicate DOFs with order-sensitive large values; exact opposing-wrench cancellation; zero, empty, invalid-history and nonfinite cases. The benchmark repeats byte parity after JIT warmup and verifies source stability.

The canonical byte encoder excludes only V8's internal packed-versus-holey allocation representation. Raw `v8.serialize` can encode those differently after warmup despite identical array properties and values. The encoder retains all logical structure and exact numeric/buffer bytes, including NaN and signed zero; it does not round numbers through JSON.

Validation: **4/4 dedicated parity/allocation tests + 10/10 existing Release regressions PASS**, and **5/5 native World tests PASS** on a fresh temporary root-source snapshot with only the optimized Release file overlaid. No World, Rows, System, Condensation, solver tolerance or retired-state mechanics was changed. This bounded task did not run an insertion probe, anatomy replay or mesh work.

Reproduce from an integrated checkout:

```sh
node --test --test-concurrency=1 tests/kirchhoffTwoChannelReleaseMeasure.test.js tests/kirchhoffTwoChannelRelease.test.js
node --expose-gc scripts/physics/measure-two-channel-release-perf.mjs /tmp/oet-release-measure-perf.json
```

During worker verification the independently owned root System was read using `OET_TWO_CHANNEL_SYSTEM_SOURCE_ROOT`. The frozen handoff bundle contains raw timing JSON, test logs, source hashes and the World snapshot manifest.

# Compact physical contact trial snapshots

The default position-history runtime now captures pooled contact metadata directly in reusable value arrays and small contact vectors byte-exactly in a reusable byte bank. Normal Jacobian scratch and clipped portal geometry are omitted because containment collection/Jacobian construction rebuilds them before the next candidate measurement. Unused partitioned-projection scratch is also omitted. Contact identities, manifold maps, normal/tangential multipliers, history, body state, and unknown fields outside the explicitly derived workspaces are retained. Custom descriptors fall back to the complete graph; compact capture requires the existing fixed-descriptor runtime contract (`reusePropertyLayout`). Split-motion and complete transaction snapshots retain their prior implementation.

No changes to forces, contact geometry, friction coefficients, tolerances, timestep, iteration limits, or solver selection.

## Reproduction

```
OET_VERIFY_WITHDRAWAL=1 OET_COMPACT_CONTACT_TRIAL=0 node scripts/physics/profile-overlap-309.mjs /tmp/before.json
OET_VERIFY_WITHDRAWAL=1 node scripts/physics/profile-overlap-309.mjs /tmp/after.json
```

Node runtime, same anatomy/default tool settings, wire 309 mm, catheter 100 mm, 120 Hz. Statistics describe the final 47 feed/withdraw steps in each interval and the first 60 hold steps. This is a replay, not a recording of the live browser state or an FPS measurement. Timing excludes hashing and report serialization. Runs are sequential and subject to JIT/system-load variability.

## Results

| Phase | State capture before → after | Whole step before → after |
|---|---:|---:|
| Catheter feed to 100 mm over wire | 13.45 → 8.54 ms (36.6% less) | 55.60 → 50.40 ms (9.4% less) |
| First 60 hold steps | 15.94 → 12.07 ms | 61.38 → 64.52 ms |
| Withdrawal to 80 mm | 13.68 → 8.16 ms | 46.85 → 40.34 ms |
| Refeed to 100 mm | 16.71 → 13.22 ms | 59.35 → 64.19 ms |

Capture improves in all overlap phases. Whole-step results remain variable; this is not an across-the-board runtime/FPS gain. Residual/solve costs still dominate much of the remaining runtime. The 120 Hz budget is not met.

For feed to 100 mm, average per-step maximum saved owners drops from 9373 to 3649. This count includes compact metadata/vector owners, not just the smaller remaining generic graph. Typed snapshot bytes drop from approximately 788 kB to 715 kB; JS metadata and allocated buffer capacity are not included in that byte counter.

## Verification

- 2578 steps per implementation across wire-only, catheter-only, coupled feed/hold/withdraw/refeed.
- Zero pose fingerprint differences.
- Zero SHA-256 differences over every body typed array plus scalar/typed-array contact-manifold fields. These include material forces, wall-history arrays, velocities and orientations; this is not merely visual pose comparison.
- Identical outer passes, trial/backtrack/factorization counts, rejection and failure decisions at every step.
- One existing nonlinear closure failure during overlap withdrawal occurs identically in both implementations; feed, hold and refeed have no failures. This optimization does not repair that separate convergence problem.
- 51 focused tests pass: rollback of repeated rejected trials, aliases, metadata/pool growth, signed zero, history, descriptor fallback, complete/compact switching, rebuilt fillet/portal geometry, friction and early rejection.
- Production build passes.

Raw runs are compressed beside `comparison.json`. A prior independent timing pair measured feed capture 15.22 → 9.78 ms and full step 60.58 → 53.11 ms; timings should not be interpreted as an exact browser speedup guarantee.

# Exact reuse of paired material responses

The condensed assembly now skips exactly zero equality loads and shares the
response of a native row between its physical and bias columns. These changes
do not alter the matrix, RHS, contact bounds, compliance, reaction ownership,
or full generalized-response certificate.

For one retained native row with equality coupling `g`, the physical column
has `p=Ae^-1*g, b=0`. Its paired bias column has `b=Ae^-1*g`. That response is
already available in owned column storage. The remaining bias-column physical
response still solves the original `Ae*p=g-Gee*b` and retains its independent
backward-error check. Reuse is confined to one frozen assembly. Exactly zero
loads use the exact zero solution only after the unshifted equality factor has
passed its pivot checks; there is no load cutoff or approximate force removal.

On the saved 155-native-row / 298-expanded-row system, actual equality solves
decrease from 126 to 59. Forty-two responses are exactly zero and twenty-five
reuse the identical native-row load. Every Schur entry, RHS, bound, group and
recovered physical/bias multiplier is exactly equal in the paired replay.
Both variants pass the original full-system KKT at 0.000108614831 mm against
the unchanged 0.0002 mm threshold; cone violation is zero.

The [benchmark](two-channel-response-reuse.json) alternates A/B order after
warm-up and records source hashes, every timing pair and the full certificate.
It measures condensation assembly alone. Neither these timings nor the
earlier 298-to-62 reduction establish a whole-timestep or browser FPS result.

Fixed physical-zero / bias-release pairs also remain in the retained Schur
system. They previously triggered a conservative unsupported-channel fallback
to the full dense block. Both fixed bounds, the prescribed negative bias
increment and all cross-channel material reactions are preserved. An independent
dense elimination and arbitrary-direction reconstruction verify this case in
`tests/kirchhoffTwoChannelCondensation.test.js`.

Reproduction:

```sh
node --test tests/kirchhoffTwoChannelCondensation.test.js tests/kirchhoffTwoChannelSystem.test.js tests/kirchhoffTwoChannelFallbackAccounting.test.js
node scripts/physics/benchmark-two-channel-condensation.mjs --baseline /tmp/oet-condensation-response-baseline/kirchhoffTwoChannelCondensation.js --output reports/two-channel-response-reuse.json
```

The preserved baseline module SHA-256 is
`991cfa60084b8e9954190ad1fe12841388ddc03e5dc634ea35afcd0f1296076a`.
The [patch](two-channel-response-reuse.patch) records the change from that
baseline; it can reconstruct the baseline in an isolated checkout for future
A/B runs. The baseline and candidate use identical kernel sources, recorded
in the benchmark manifest.

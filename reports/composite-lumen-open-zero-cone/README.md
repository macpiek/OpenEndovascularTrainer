# Exact elimination of unloaded open lumen friction increments

`JointLumenFrictionRows.refresh` avoids the full surface-motion derivative operator only after a fresh original strict-side normal query proves g>0, inactive normal, literal Fn=Ft=0, zero normal residual/Jacobian and positive normal multiplier derivative. The normal linear equation then gives δFn=0; the two Coulomb equations give δFt=0. Their isolated equations remain in the system. Known-zero mechanical increments do not require evaluating B/DB or slip.

The certificate explicitly records `slip:null`, `slipRequired:false`, zero cone/work/equation residual and a named algebraic proof. It does not invent a physical zero slip. Positive Fn, active Fn=0, g=0, and any nonzero traction, including a subnormal, use the full original operator. Conservative same-edge/history/chart eligibility only chooses the optimization; outside it, the original surface operator and its rejection rules remain. Previous-history preparation and current normal queries are retained. Full/value reuse, activation/release/reactivation, unsupported transport and failures are tested.

Fourteen manager tests pass in the root full suite. The full-step verification independently checks the original physical gap and exact accepted Fn/Ft when slip is unnecessary; it still invokes the original friction measurement when slip is computed.

`probe.mjs` and `probe.json` compare immutable baseline/candidate stages. Root verified that the only differing production source in those stages is this manager and that the candidate hash equals the root source. Nine consecutive full-step A/B cases (open and loaded/release cases, cold/reuse and rejection/retry) have exactly equal physical states, loads, reactions and balances. Query/evaluation/direction counts are retained. Their individual cold timings are not a performance benchmark.

The bounded refresh microbenchmark has 17 nodes, 32 open original normal samples and 64 friction rows, six paired warmups and 18 alternating paired repeats. Full normal+friction refresh median is 11.052→0.585 ms; gradient refresh median 3.911→0.535 ms. The original normal query remains in both measurements. This measures an all-open contact operator on Node, not a full World timestep, loaded contact cost, anatomy or FPS. It cannot establish the requested 60 FPS / 120 Hz target.

Frozen baseline: `/tmp/oet-lumen-open-zero-cone-baseline/stage`; candidate: `/tmp/oet-lumen-open-zero-cone-candidate`. Reproduce with `node reports/composite-lumen-open-zero-cone/probe.mjs BASELINE_STAGE CANDIDATE_STAGE output.json`.

# Current retained solver CPU profile

Full 5757-step cycle with CPU_PROFILE=1 after reverting spatial activation batching and initial contact release. Full trajectory comparison against wasm-lu-final: all 96 snapshots exactly identical, same 28832 factorizations / 16468 Newton iterations. CPU profiling has overhead; do not treat these wall timings as browser physics Hz.

Sampling: constraint-row assembly inclusive 27.68% (self 7.88%); active-set iteration inclusive 21.09%; material tangent inclusive 9.87%; scalar BVH nearest-triangle query inclusive 8.05%; contact discovery sampler inclusive 14.96%. Inclusive percentages overlap and must not be added. In particular contact discovery is already within constraint-row assembly. Retained per-face witness geometry itself is only 1.67% inclusive, so accelerating just that kernel has limited potential.

The original profile remains /tmp/oet-full-cycle-current-wasm-profile/cycle.cpuprofile. Summarized samples, source hashes and full-cycle quality are retained here.

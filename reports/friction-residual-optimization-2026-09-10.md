# Lumen friction residual evaluation

Implemented residual-only surface kinematics and direct 2×2 mobility evaluation. Assembly still builds complete gradient rows. Each residual call rereads current geometry, material pose history and reactions; there is no cross-trial geometry cache. Diagnostic force/moment outputs and acceptance tolerances are preserved.

## Measurement

CPU-only, no renderer. The reference is the pre-change `kirchhoffCoupledFrictionRows.js`, saved before editing; its full-row builder remains the reference for the optimized measurement. Isolated benchmark: 80 synthetic side contacts, reused output buffers, full residual diagnostics plus line-search merit, 100 warm-up calls per variant, nine alternating-order batches of 150 evaluations.

- Median evaluation: **0.5714 → 0.5055 ms**, about **11.5% lower**.
- Residual and merit agree between variants.
- This does not measure browser FPS or deep-insertion performance.

Paired runtime comparison used the transformed aortic anatomy, 120 Hz, current 5 mm catheter grid and current user stiffness defaults (catheter 58.1/87, wire 39/30.7). The protocol inserted the wire to 220 mm, then fed the catheter. Both variants applied the same runtime policy and alternated execution order each step.

- **796 completed steps** with zero position/orientation differences and identical convergence decisions.
- Both variants threw **Invalid friction data** at schedule step 796 (zero-based), approximately 85.37 mm catheter feed. This is a pre-existing limit of this run, not a successful full-depth validation.
- In the recorded run, mean complete-step CPU time was 3.410 → 3.252 ms, and constraint measurement 1.191 → 1.068 ms across all 796 completed steps (including wire-only steps). Whole-runtime timing varied between runs; no stable FPS gain is claimed.

## Validation

- 33 focused surface/friction/runtime tests passed.
- Coupled suite: 765 passed, the same seven known failures (default-source audit, two Coulomb matrix-format tests, two mouth transitions, two experimental two-channel refinement tests).
- Added direct parity coverage for every current lumen feature kind, load/coefficient changes, changing normals, repeated interpolation nodes, prescribed/active-range masks, pose history, physical velocity mode, restoration and disappearing contacts.
- Production build passed, output outside tracked dist.

Detailed timings and runtime diagnostics: `friction-residual-optimization-2026-09-10.json`.

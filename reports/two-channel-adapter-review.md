Bounded read-only review of the initial `kirchhoffTwoChannelSystem.js` adapter: no actionable assembly, row-mapping, recovery or reconstructed-KKT defect found in the reviewed version. No World or anatomy execution, native integration test, or completed row-major kernel acceptance is claimed.

Reviewed adapter SHA-256: `c38df7abd719a10bbdc9a4755b767971f52c9c0e27a1529a4b5ab70113fcc458`. The snapshot and source-before/after hashes are recorded in `two-channel-adapter-review-source.json`.

The explicit channel array is consumed by original native row index, while native sorted order drives interleaving. Recovery converts native column entries from sorted to original indices before reading the two increments. Material offsets and contact/additional slices remain in original ordering. This is consistent on both bodies, including absent bias rows.

The Gram helper removes physical alpha only from the native diagonal. The physical and bias diagonals then add their own alpha; bias RHS and increment bounds correctly use the bias strain, bias total multiplier and bias alpha. Cross-channel response follows the row residual channel, so physical pose controls see both corrections and physical motion rows see only dqPhysical. Bias-motion controls/material see only dqBias; bias geometry sees the total correction. This matches the affine contract and intentionally produces a nonsymmetric matrix.

Friction member and normal indices are remapped to physical rows. Copies of lambda/radii/mu avoid mutating the native group vectors. Correction recovery uses native individual-coordinate J and W; this is valid even when native assembly computed the equivalent common-relative Gram. The independent KKT reconstruction uses recovered physical/bias generalized corrections with the same row-channel rules and owning compliance. It is not merely a repeat of dense matrix multiplication.

The result proposes one common scale bounding each channel and their sum, returns physical material increments for the existing pose application path, and separately returns bias material/contact/additional increments. The caller still owns the nonlinear joint acceptance, application and bank accumulation. The adapter rejects assembly while the old split bias banks are installed.

Two inherited/API details to keep explicit during integration, not correctness findings:

- This adapter does not update `_jointActive` or `additionalRows[].activeHint`; unlike the native solve it currently performs no hint update, regardless of `updateActiveHints`. Hint omission changes warm-start behavior, not the equations.
- “Without applying either channel” does not mean strictly mutation-free. Native assembly owns borrowed scratch and enforces an already prescribed hard orientation target in `assembleKirchhoffDirect`, just as the existing native assembly does. Returned correction and multiplier arrays are owned; `includeSystem.native` retains the native borrowed-assembly lifetime.

Fold rows with physical `pose` and no bias row are algebraically supported: their residual receives dqPhysical+dqBias, and only their physical reaction is solved. Whether that is the intended physical fold law remains a World/model integration decision.

The row-major Newton implementation was still pending during this review. Its format support, final-load return structures and full solve behavior must be covered by the solver worker/native adapter tests; they are not certified by this source review.

# Wall and lumen normals in the same JointTimeStep

`kirchhoffCompositeJointTimeStep.js` now accepts `wall:'none'` (default) or the explicit `wall-normal` configuration documented in [JointWallRows](composite-joint-wall-rows.md), alongside `contacts:'none'` or the existing `lumen-normal` configuration. Both physical material axes, inertia, elastic forces, length constraints, supports and declared normal contacts enter one band solve. The new solver still does not drive World or the application.

The final wall row adapter was imported from `/tmp/oet-composite-joint-wall-rows-final-8996`, manifest SHA256 `6b235b766a3b9c93b4505ebd099135e6d15cf1dff66c5f33c59d7ea85bb9f388`. Its original source SHA256 was `42bce9955c3bee4e3db1b157ac981f775b7785fdc73a815f1705a5f329b1d653`. The subsequent [endpoint-envelope extension](composite-joint-wall-envelope.md) is now integrated from `/tmp/oet-composite-joint-wall-envelope-final-8996`; the current adapter SHA256 is `39d315a17a682bfdcfbf2e7c4f1c8304a171462220f2a0e3c6ea273f99c6a0b4`. This report covers root's whole-step integration; the worker's sixteen operator tests alone are not a timestep result.

## Physical ownership and transaction

Each wall query uses its actual outer owner's axis. Wire motion inside the overlap is reconstructed from common plus relative coordinates; vessel load is not also applied to the inner wire. Lumen loads remain equal and opposite on the two axes. Committed `contactForces` are the sum of wall and lumen nodal forces, and each separate material balance checks momentum change against applied, boundary and contact forces.

The step owns `wallContactState` alongside the lumen state. Nonzero accepted wall reactions cannot be dropped with `wall:'none'`. Each attempted line-search trial restores both wall Fn and all chart/source/sample/duplicate metadata from a checkpoint, then queries its actual new geometry. Rejected trials and failed steps cannot overwrite the incoming physical state or advance time. Query budgets count actual attempts, including failed work, and are shared between both contact types.

SDF chart discovery uses the physical displacement of the relevant owner. A newly admitted two-branch chart triggers rebuilding of the original row structure, a fresh material/contact evaluation and a new joint direction at the same geometry. It does not consume a physical timestep. Bounded row workspace reuse retains storage only; coefficients and factors are current.

## Newton globalization

The final wall acceptance gates are unchanged: original gap, NCP residual in mm, literal Fn >= 0, complementarity work |Fn*g| in Nmm and the original SDF domain test. Force, torque, length, boundary and original linear gates also remain unchanged.

The line-search merit uses the actual mechanical/NCP equations plus gap and sign violations. The product Fn*g remains a mandatory acceptance gate and stays in the full diagnostic merit, but is not an additional Newton equation. Including its square in the globalization merit can reject a useful step when Fn starts at zero: both solved equations improve while their redundant product initially grows. The separate `lineSearchMerit` resolves the real P1 fixture without changing physical tolerances or normal stiffness. `forcePerLength` remains a numerical NCP scale, not penalty stiffness.

## Validation

`tests/kirchhoffCompositeJointWallTimeStep.test.js`: seven tests pass, including successful physical steps, query-budget rollback and explicit rejection of an unsupported loaded sample switch in capsule mode.

- A tilted catheter contacts an analytic plane while the independently moving wire retains its own motion and zero wall load.
- The next dt consumes accepted velocities and wall history. Cold/reused workspaces agree; late rejection, retry and release preserve owned data.
- Query-budget rejection counts performed work and leaves the input unchanged.
- Wire pressure, lumen reactions and outer catheter wall support act simultaneously in one accepted dt. Separate momentum balances pass, while total internal lumen force cancels exactly.
- The real Aorta P1 fixture admits both SDF reactions, accepts its first and second dt at 1/120 s, and passes original queries, per-material balances and rejection/retry. The other material remains physically independent. This is a small actual-anatomy contact fixture, not a full inserted-device replay.
- In the default capsule mode, a loaded sample change from midpoint to endpoint still rejects and rolls back explicitly.
- With `wall.contactMode:'envelope'`, the same flat-contact scenario now accepts contact, a second loaded dt and a third release dt at 1/120 s. Endpoint reactions retain the load when the original capsule sample changes. The original capsule queries and all their gap/NCP/work checks remain; only exactly dependent force rows are transferred to the endpoint gauge. Independent total momentum predicts the summed endpoint load. Cold/reused workspaces agree, late rejection leaves history unchanged, and retry matches the uninterrupted result. Loaded envelope history cannot silently become capsule history.

With the final wall adapter, the combined operator and whole-step subset passes 16/16. The no-wall candidate reuses the frozen lumen probe: all six loaded/released physical positions and Fn match the unreduced oracle exactly; the no-contact state, certificate and balances are bit-identical to the earlier frozen no-contact operator. Raw parity: `/tmp/oet-composite-joint-wall-no-wall-parity.json`; raw 16-test output: `/tmp/oet-composite-joint-wall-final-tests.txt`.

The final envelope operator plus whole-step subset passes **23/23** (`/tmp/oet-composite-joint-wall-envelope-integrated-tests.txt`). The full composite suite passes **446/446** (`/tmp/oet-composite-joint-envelope-full-suite.txt`). The earlier no-wall parity result above remains a separate frozen comparison. Envelope mode currently handles smooth branches; active SDF seams in that mode and general independent loaded-sample transfer remain unsupported.

This scope excludes friction, portal/rim/fillet contact, general source/sample transfer, continuous containment/CCD, moving topology, adaptation and application integration. No FPS or full-device timing result is claimed.

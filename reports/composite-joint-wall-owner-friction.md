# Separate wall friction coefficients for exposed tools

The production Joint wall friction manager now accepts either the existing common `mu:[axis0,axis1]` or an explicit `muByOwner:[{owner,mu}]` declaration. Each exposed physical owner must appear exactly once. Coefficients are finite and nonnegative, copied at preparation, applied to that owner's original Fn/Ft rows and stored as owned history. Reordering the declaration is immaterial; changing its law invalidates prepared or accepted history. The old common declaration and signature remain compatible.

This closes coefficient routing for distinct guidewire and catheter surfaces. The pair still describes two tangent axes of one Coulomb law. It does **not** implement separate static and kinetic coefficients. In the app, guidewire `wallFriction` is a kinetic alias while `wallStaticFriction` is independently configured; those values must not be passed as the two axes. The simulator has not been switched to this solver.

## Validation

- Four manager tests cover different owner tractions/work and physical load scatter, finite differences across common/relative/spin and normal-force columns, history/workspace reuse, copied coefficients, and invalid or mutated declarations.
- Two whole-step tests exercise both exposed surfaces in one common/relative system, independent feed/spin, free mechanical unknowns, distinct radii and coefficients. Each owner's force balance, Coulomb traction and dissipation, original gap and rod lengths are checked. Two successive dt retain loaded history; late evaluation/query budget failures preserve state and retry exactly.
- Changing numerical penalty through 5/50/500 preserves the physical root. Equal per-owner coefficients reproduce the existing common-law state, forces, tractions and evaluation count exactly. All original wall query accounting remains unchanged.
- This two-owner wall fixture intentionally isolates exposed-surface laws with no lumen contact. Existing separate lumen-plus-wall whole-step tests remain in the full suite. It is not an anatomy replay or a test of full insertion lifecycle.

`npm run test:physics:composite`: **758/758 PASS**, 16.825 s. Build PASS, 1.89 s. Logs and source hashes are in `composite-joint-wall-owner-friction/`. This validates correctness of this addition, not 60 FPS or the full `npm test`. The app's source constraints, unequal static/kinetic law, moving material and surface history, adaptive reduction and deep/max insertion browser measurements remain unfinished.

Implemented `src/physics/kirchhoffTwoChannelCondensation.js`. Its narrow API is:

```js
const reduced = condenseKirchhoffTwoChannelSystem(native, channels, options);
if (reduced.status === 'fallback') {
  // Run the existing dense reference path; reason identifies the unsupported case.
} else {
  // Solve reduced.matrix/rhs/lower/upper/groups in row-major format.
  const recovered = reduced.recover(retainedIncrement);
  // recovered.physicalIncrement and biasIncrement: ORIGINAL native row order.
  // recovered.fullIncrement: dense adapter's sorted/interleaved row order.
  // Root performs the independent full J*dq/KKT check before application.
}
```

`channels` accepts the root adapter's explicit original-row array or its native-assembly callback. The module eliminates only paired bilateral physical-pose/bias-motion rows with identical alpha. Friction members must be physical-motion with no bias copy. Physical normals, bias geometry, bounded folds and every other inequality stay in the retained system. Unsupported pairs, unequal equality alpha, a missing usable equality block, clamped/unsafe pivots and failed original-Ae solve checks produce explicit fallback statuses. No coefficient, compliance, bound, friction law or stopping threshold is changed.

For the eliminated block `[Ae,Gee;0,Ae]`, `Gee=Ae−diag(alpha)`, it factors diagonally scaled Ae once with the existing Wasm skyline kernel, or the existing band kernel when skyline is unavailable. It solves beta first and then lambda from `rPhysical−Gee*beta`. No diagonal shift is added. Because the existing kernels internally floor pivots at 1e−12, the adapter rejects scaled pivots at or below 32e−12 and checks every Ae solve against the unmodified band matrix (componentwise backward error at most 1e−10).

The nonsymmetric Schur matrix is formed from exact cross-channel response columns. Retained bounds remain increment bounds, and friction group/normal indices map exclusively to retained physical rows. Recovery uses owned particular solutions and response columns; it never rereads native borrowed scratch. Each recovered result owns its arrays. Native input arrays, contact history and active hints are not mutated.

Structurally zero equalities are retained, not inverted or discarded. The supplied runtime has original rows 3 and 5 of this form; row 3 has physical RHS 3.102116925779441e−6. Both channel rows and their original RHS remain in the Schur system, so the original KKT measures the same residual as the dense reference. There is no RHS zeroing, tolerance-based rank removal or hidden relaxation.

Eight tests pass with both the local legacy Wasm band kernel and a frozen copy of root's current Wasm skyline/kernel implementation:

- Native pinned compliant tip: dense and condensed Coulomb solutions recover the same physical and bias increments.
- Native hard material, hard pose control, normal, friction and bounded fold rows: every Schur entry/RHS agrees with an independent dense elimination, in individual and common-relative bases, with nontrivial native row permutation.
- The two recovered generalized response channels preserve the original full residual identity for arbitrary retained directions.
- Schur arrays, groups and recovery remain unchanged after native scratch reuse and another condensation. Mutating a previously returned increment does not corrupt later recovery.
- Unsupported equality pairs and unequal alpha return explicit fallback without editing input.
- Dependent nonzero equalities reject the kernel pivot floor rather than treating it as compliance.
- An immovable row retains its nonzero RHS and complete row identity.
- The supplied 155-row frozen runtime solves after condensation and satisfies the original full 298-row final-load KKT, cone and bounds.

The first test run exposed an assertion-helper error comparing infinite bounds with a finite-number tolerance check. Bounds were changed to exact array equality. Numerical tolerances and implementation equations were not loosened.

On the provided frozen runtime input, native 155 rows expand to 298 two-channel rows. The module eliminates 118 equality pairs and retains 62 rows, including both copies of the two structurally zero equalities. Storage is:

| Item | Entries |
|---|---:|
| Full reference matrix | 88,804 |
| Retained dense Schur | 3,844 |
| Ae band, bandwidth 15 | 1,770 |
| Shared factor | 1,770 |
| Both channel response columns | 14,632 |
| Full expanded matrix allocated by condensation | 0 |

The minimum scaled Ae pivot is 0.0421247744; maximum original-Ae backward error with skyline is 6.36e−16. Condensation uses one equality factor; retained Newton uses two factors versus three for the dense reference. The recovered full-system KKT residual is 0.000108614831 mm, versus 0.000010829510 mm for the dense solution; both pass the unchanged 0.0002 mm tolerance, with zero cone and bound violations. Maximum physical multiplier difference from the saved dense oracle is 0.000135975238, and bias difference is 0.000254123039. These are separately converged numerical solutions, not bitwise-identical multiplier vectors.

One cold and three warmed paired solves used exactly that saved linear system and the same root Newton kernel `5318eef39e4c2031179930ffd5e71aa414665a309f2cfee809edd6b9d2c8f1e5`:

| Timing | Dense solve | Condensation + solve + recovery |
|---|---:|---:|
| First/cold | 95.40 ms | 18.71 ms |
| Median of three warmed pairs | 53.16 ms | 12.66 ms |

The warmed ratio is 4.20× on this frozen case. The dense timing excludes loading/assembling its already supplied matrix, while condensed timing includes Schur construction and recovery. Both exclude the separately measured full reference KKT check. The run count is small, runtime/JIT contention is visible, and these figures are not an anatomy, World-step or FPS benchmark. No anatomy was rerun.

Deliverable freeze:

- Module SHA-256: `6612f923bca9b9f68ce371673b26c8911a8685fb722438c633ebcfb94943d377`.
- Test SHA-256: `3d69f69c394e74d7e1d92e7e38e442968b262e93e90c2cd3fe12254170624e54`.
- Fixture `tests/fixtures/kirchhoff-two-channel-condensation.json.gz`: `5bbe3bd70be517767e7cae983e555e8f57740379b605d8bdc4cc5a82ee350c56`.

Reproduce in a root checkout containing the dense adapter and row-major Newton kernel:

```sh
node --test tests/kirchhoffTwoChannelCondensation.test.js
node reports/probe-two-channel-condensation.mjs
```

For this worker's preserved frozen runtime, the commands used were:

```sh
node --test /tmp/oet-two-channel-condensation-o0k_egvj/tests/kirchhoffTwoChannelCondensation.test.js
node /Users/macpiek/.codex/worktrees/0827/OpenEndovascularTrainer/reports/probe-two-channel-condensation.mjs --source-root /tmp/oet-two-channel-condensation-o0k_egvj
```

Detailed results: `two-channel-condensation-tests.txt`, `two-channel-condensation-frozen-comparison.json`, and `two-channel-condensation-source.json`. The manifest verifies frozen native source stability and records the module, test, fixture and replay hashes. Root owns integration into TwoChannelSystem and World; this task changed only the new condensation module, its tests/fixture and reports. Full original J*dq reconstruction remains the final integration gate.

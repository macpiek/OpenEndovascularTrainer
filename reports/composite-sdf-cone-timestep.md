# Frozen CompositeTimeStep SDF cone integration

The original free-three-node P1 fixture now completes its whole 1/120 s timestep with two positive physical reactions at the exact x=65 min seam. The next physical timestep carries the accepted old material velocities, leaves the seam, releases the left reaction exactly to zero, and retains a positive right reaction. The original gap, force, torque, length, complementarity and nonnegative-force gates remain in force.

Exactly four owned files are in `sdf-timestep.patch`: TimeStep, its SDF integration test, MixedDirection, and its unit test. No changes to WallGeometry, SdfBranches, DifferentialRows, WallContacts, the provider, the anatomy asset, or root sources were made. The frozen imported SdfBranches module is a dependency, not part of this patch. All source before/after hashes and dependency hashes are in `manifest.json`.

## Measured result

| | First P1 dt | Second physical dt |
|---|---:|---:|
| Directions | 7 | 7 |
| Original queries | 14 | 18 |
| Maximum nodal force residual | 4.19466e-13 | 1.56950e-12 |
| Maximum torque residual | 2.81967e-18 | 1.12014e-16 |
| Maximum length residual | 1.57652e-14 | 1.88544e-13 |
| Left Fn | 2.19247787048963 | 0 |
| Right Fn | 0.59800742385073 | 0.58017223573999 |
| q1.x | 65 exactly | 65.01337321578661 |

These are solver counters from the bounded witness, not a performance benchmark. First-dt query count equals its material-evaluation count: discovering/rebuilding the seam block performs no extra provider query. `witness.mjs` reproduces both production steps and an interrupted signed trial; run `node witness.mjs /absolute/repository/root > reproduced.json`. The independent mechanical reassembly is in the SDF test, not inferred from reported diagnostics.

## Integration

The existing collector still queries every original capsule exactly once per refresh and retains raw source/gap/normal/sample data. A Newton direction's known affine sample path identifies an encountered face. Only a successful SdfBranches min/intersection proof admits the local two-force chart. The timestep rebuilds contact equations at the same current state using already captured query data. It neither moves the geometry to the face nor inserts a grid-plane wall. Exact tie is reached by solving the two physical contact equations.

Two physical force unknowns are interleaved with the same chain positions and length reactions in the existing local band. Ordinary collector rows are measured/canonicalized through a transaction-local view; the obsolete single-normal row for an admitted cone is replaced by the complete original-gap/cone proof. Its raw query is retained. Unknown derivatives on other active/loaded rows still reject.

The existing merit and Armijo condition are unchanged. Inside a proved chart, nonselected polynomial continuations and temporarily inadmissible reactions are auxiliary private nonlinear iterates. Final acceptance re-queries the original provider, reevaluates the chart, and requires physical nonnegativity, domain/tie admissibility, complementarity, original gap agreement, and fresh total mechanical equilibrium. No wrong-domain force can enter accepted state.

`wallContactState.sdfSeams` owns each face identity, edge/owner/radius/sample provenance, and its two physical scalar forces. The corresponding legacy scalar slot is zero; it is not a sum or norm of the two reactions. Records carry two actual normal vectors and physical forces; each scalar Fn remains its own force magnitude. Geometry, frames, velocities and both force histories commit together exactly once, or the original input state is returned unchanged.

## Minimal signed-iterate interface

MixedDirection accepts `row.allowSignedWallIterate === true` only for a definition explicitly carrying `sdfBranch:0|1`. Ordinary wall definitions reject that opt-in and retain their nonnegative-iterate guard. No NCP equation, derivative, matrix, scaling, pivot or linear residual proof was changed.

The SDF cone solver may use a signed PRIVATE NCP unknown so a releasing face can switch to its inactive equation. Clipping only the force part of that Newton step had pinned the second dt to the wall. At a full inactive step the exact equation sets Fn to canonical zero. Accepted force/history still requires Fn>=0 exactly. A deliberately interrupted second step contains negative private forces, fails the original cone certificate, and rolls back the entire dt; the test verifies that no negative reaction leaks into accepted state or later workspace reuse.

## Validation and limits

All 70 tests passed: existing timestep/gradient/workspace/wall/mixed suites, 8 SDF integration tests, 10 MixedDirection tests, and all 8 newly integrated BVH collector tests including complete capsule and envelope timesteps. The exact imported BVH dependencies tested were DifferentialRows `3e57d869cdc4deb403fe0802a5637f0699ad9ba40ea0c0dbe52fb05fb333a80f` and WallContacts `47b11dedba3a67c124e1d499f1c162c60439349980b2ec2878a58b2bc2c581f7`. Full and lazy second-dt states/certificates are identical. Input immutability, interrupted discovery/retry, owned accepted buffers and a changed original source in the fresh commit query are covered.

Automatic cone integration is bounded to mixed **capsule** contact and one proved adjacent-cell face for each selected sample. Smooth envelope and BVH modes retain their existing paths; generalized envelope cone canonicalization is not added. The SdfBranches restrictions remain: uniform inside sign proof, strict min ordering, two independent normals, no multi-axis intersections or unresolved source/normal branches. A retained chart's source, sample, ownership or domain change rejects instead of silently transferring traction; general chart-to-chart and duplicate-cone reaction transfer are outside this patch. The certificate remains discrete frictionless contact, not CCD, friction, remeshing, anatomical ridge validation or application-level acceptance. No app or performance benchmark was run.

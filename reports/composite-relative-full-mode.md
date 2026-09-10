# Variable-dimensional joint relative coordinates

Implemented in the owned RelativeCluster and RelativeDirection modules and their existing tests. The preceding read-only diagnosis is frozen at `/tmp/oet-composite-relative-length-diagnosis-final/manifest.json`. Existing Chain, TimeStep, Kinematics, wall and original Element kernels remain unchanged by this task.

`modes` now accepts either two or three orthonormal basis vectors per node. Each returned mode has `dimension`, an owned frozen basis, and contiguous `relativeDofs`. Their indices follow a prefix sum of mode dimensions; they are no longer `2*i`. For example dimensions `[2,3,2]` produce indices `[0,1]`, `[2,3,4]`, `[5,6]`. All common/relative/spin and cross-mode blocks retain their complete physical contributions.

Two vectors retain the original centered-chord transversality check and explicitly report `relativeRepresentation:'transverse-reduced'`. Three vectors form a complete fixed world basis and do not need to remain transverse to the current chord. Every three-coordinate mode must belong to an overlap node carrying both materials; a material ending at an adjacent edge still counts as present at its tip node. A bare single-material section does not gain a second physical axis. Mixed dimensions report `'mixed-full-and-transverse-reduced'`; all three-dimensional modes report `'full-rank-on-represented-nodes'`.

The full-rank statement concerns only the represented overlap nodes and the linear coordinate map `(q,rho) -> (q,q+B rho)`. This operator still evaluates at zero relative offset. It does not yet evaluate a nonlinear finite-offset state, supply both material length constraints, update separate geometry/frame histories, add endpoint modes, perform transfer, or certify a dt. Default GN and optional exact Hessians retain their prior meaning; no PSD clamp or approximation was added.

The new `createCompositeRelativeClusterStructure(args)` export compiles modes, affected hinge/inertia stencils, common/relative bands, CSR coupling indices and zeroed numeric scratch without calling a material provider, Element or Kinematics evaluator. Its output has `operatorReady:false` and `hessianValid:false`. The original `assembleCompositeRelativeCluster(args)` API remains compatible: it assembles those same blocks and marks both flags true after successful finite checks.

The public symbolic contract is:

- `modes`: node, dimension, frozen basis and contiguous relative indices;
- `common.dofs`, `common.band`: local common support and lower-band layout;
- `relative.dofCount`, `relative.band`: prefix-summed relative support;
- `coupling.commonDofs`, `rowOffsets`, `columns`: CSR support for H_qrho;
- `stencils`: each physical hinge/edge once, with vertex/edge, global common DOFs, local common indices and all touched relative DOFs.

This is sufficient for a nonlinear caller to compile its own constant-basis pullback without first evaluating a stale coincident-wire operator. Common diagnostic values and `additionalInertia` retain their previous non-additive roles.

RelativeDirection now freezes mode offsets/dimensions/bases and interleaves each mode's actual coordinate count with common DOFs and local duals. It accepts the unassembled structure when creating workspace, but rejects it at solve time until a fresh physical operator is provided. Subsequent solves still verify current structure and consume current H/C. Force/torque and each constraint's original unit/tolerance remain unchanged, as does the corrected positive force-on-chain reaction convention. No independent per-mode solve or global dense Schur matrix was introduced.

Verification: **42/42 PASS**, comprising 13 Cluster, 14 RelativeDirection, 7 prior RelativePatch and 8 existing MixedDirection tests. Seven tests are new in this generalization. Independent summed-wire-energy finite differences cover full 3D and mixed 2/3, axial coordinates, anisotropy, exact nonlinear strain derivatives, spin rows and complete physical convective inertia, at the existing `2e-8` comparison tolerance. Independent dense original systems cover GN and Exact for both 3D and mixed dimensions. Previous 2D, reaction sign, strict residual, singular rejection and double-counting regressions remain passing.

The geometric test shows why the axial coordinate matters: for straight catheter spacing `2 mm` and transverse wire offsets `[0,0.0405,0] mm`, the transverse-only model has more than `4e-4 mm` edge-length error. Axial increments `sqrt(2²-0.0405²)-2` restore both wire lengths to `2 mm` with a freely sliding distal wire endpoint. The two length Jacobians become independent when axial relative columns exist. This test demonstrates coordinate admissibility, not a nonlinear length solve.

The companion witness records full 3D local-band storage at 25/65/201 nodes, one local dual per mode, and original equation residuals. It uses explicit manufactured constitutive/inertia controls and makes no FPS claim. The revised frozen bundle is `/tmp/oet-composite-relative-full-mode-final/manifest.json`; all earlier frozen bundles remain unchanged.

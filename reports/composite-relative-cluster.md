# Joint transverse-mode cluster operator

`src/physics/kirchhoffCompositeRelativeCluster.js` assembles one UNCONDENSED operator for multiple wire transverse node modes with intersecting supports. It keeps one common centerline and the independent material spins. It evaluates the union of affected wire hinges exactly once, rather than assembling or condensing each disk independently.

The linearization is at zero relative offset: `x_wire = q + B*rho`, with frozen orthonormal world-space basis vectors. The exact pullbacks are `g_rho = Bᵀ*g_wire`, `H_rhorho = Bᵀ*H_wire*B`, and `H_qrho = H_wire*B`. Every common position and wire-spin row of each affected element is retained, including transported-frame/reference-twist and anisotropic bend/twist coupling. The default backend is the existing Fast/GN operator. Explicit `elementBackend:'wasm-exact'` uses the complete energy Hessian; `'javascript'` retains the independent GN oracle. No PSD clamp, pivot floor, normal force, disk solve, condensation, reconstruction, state transfer or CCD is added.

## Input and output

`assembleCompositeRelativeCluster({data, layout, modes, toolId:'wire', elementBackend:'wasm', inertia:null})` accepts the existing common Chain data/layout and a strictly node-sorted list `modes:[{node,basis:[b0,b1]}]`. Each node is interior and carries wire material on both incident edges. Its basis is finite, orthonormal and transverse to the centered COMMON chord; owned copies are returned. Basis vectors are fixed in this linearization and do not introduce extra derivatives. This is not an API for nonlinear relative-offset history or arbitrary changing bases.

Outputs:

- `common`: sorted GLOBAL Chain `dofs`, local gradient, and lower-band Hessian. Entry `(i,j)` uses `max(i,j)*band+abs(i-j)` within the band. These are the affected wire contributions already represented in common assembly: diagnostics only, NEVER add them again to Chain.
- `relative`: `dofCount=2*modes.length`, complete gradient and lower-band Hessian. Relative coordinates are ordered `[rho_0a,rho_0b,rho_1a,rho_1b,...]`. Shared-element cross-mode entries are retained.
- `coupling`: sparse CSR `rowOffsets`, `columns`, `values`. Row indices address `common.dofs`; column indices address relative coordinates. It contains the complete `H_qrho`, including wire spin rows.
- `stencils`: each affected hinge or inertia edge once, with its global/common-local DOFs and participating relative DOFs, for later sparse mixed scatter.
- `affectedHinges`, `affectedInertiaEdges`, `hessianKind`, `elasticEnergy`, `inertialEnergy`, and explicit `certified:false`/`condensed:false`.

There is no dense global matrix. Common and relative operators retain fixed local bandwidth and the coupling uses local CSR support. Twenty consecutive transverse node modes have40 relative coordinates but relative band6; each CSR common row has at most10 relative columns. Extending a distant wire from11 to201 nodes does not change the local cluster storage or response.

## Full additional physical inertia

Optional `inertia:{dt,previousPositions,inertiaEdges}` accepts the same prepared per-edge Kinematics input shape used by the common TimeStep. It evaluates each incident wire edge once and uses ONLY that edge’s wire material mass/map/old MATERIAL velocity. Catheter inertia remains on the common axis and is not spuriously assigned the wire-relative motion.

The original Kinematics operator supplies its full consistent/convective6x6 edge Hessian and gradient. The cluster applies the SAME `q+B*rho` pullback to all three blocks, including adjacent-node mode coupling; no diagonal mass approximation or free-standing relative penalty substitutes for the physical term. Aggregate `common`, `relative` and `coupling` include this inertia. `additionalInertia` reports the full physical contribution separately, including its base common energy/g/H, for diagnostics. It is a subset of the aggregate, NOT an extra block to add again. Existing common Chain inertia must not be double-counted.

## Verification

38/38 tests passed across new RelativeCluster(9), existing RelativePatch, Exact Element and Kinematics suites. New source syntax passed.

- One-mode GN energy, complete common g/H, relative g/H and all coupling rows agree with the existing RelativePatch at2e−12.
- Three neighboring modes atnodes4,5,6 evaluate exactly hinges3,4,5,6,7 once each; the material callback records that exact sequence.
- Joint GN blocks match independent contractions of the full affected common diagnostic matrix, including nonzero wire-spin coupling. JS and Fast/GN agree.
- Every Exact relative-H and common-relative entry is compared with independent finite differences of the ORIGINAL JS summed wire energy/gradient, evaluating all wire hinges rather than the cluster’s stencils. All columns and rows are checked independently; no mirrored result is used as a reference. Several simultaneous offsets also match a directional derivative.
- Additional physical inertia is independently scattered from full original Kinematics6x6 matrices into a dense TEST oracle, checking every base common entry. Total relative and cross-mode/common response also matches independent complete wire-plus-inertia finite differences. Changing the catheter mass does not change wire-relative response; no angular inertia appears in wire-spin rows.
- Finite-difference tolerances remain2e−8. Elastic checks useh2e−6; the convective inertia check usesh2e−5 to resolve small axial basis projections against nonzero momentum. Paired local energy differences are summed before division to avoid subtracting unrelated large prestress constants. No solver or admission tolerance is changed.
- Duplicate/non-interior/tangential modes, missing material ownership, invalid constitutive tensors and invalid physical inertia fail explicitly. Inputs stay unchanged and returned bases are owned/frozen.

## Cross-mode witness

For a straight11-node chain with spacing2, wire `EI1=2,EI2=7,GJ=3,dsDx=1.3`, three adjacent two-coordinate modes produce27 local common DOFs with common band11,6 relative coordinates with band6, and114 CSR coupling entries. Only5 distinct wire hinges contribute. The y-direction submatrix contains diagonal4.03846, neighbor−2.69231 and second-neighbor+0.673077 terms.

For the normalized simultaneous direction `[0.3,-0.2,0.7,0.1,-0.4,0.6]`, the joint gradient-response first component is−0.942308. Zeroing all cross-mode terms changes it to+1.211538, with maximum response error2.153846. This is an algebraic directional witness, not an admitted physical lumen offset or a contact force. The full matrix and responses are recorded in `reports/composite-relative-cluster-witness.json`.

This bounded operator is ready for the parent’s later joint constrained block. It supplies neither a solved rho/normal reaction nor clearance/reconstruction certification, runtime stepping or FPS evidence. TimeStep, Chain, Wall and other existing source modules were not edited.

Frozen handoff: `/tmp/oet-composite-relative-cluster-final/manifest.json`.

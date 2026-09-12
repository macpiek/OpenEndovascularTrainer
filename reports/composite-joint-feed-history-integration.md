# Material-history integration in JointTimeStep

The whole joint step now accepts either an old affine endpoint-velocity pair or `oldVelocityPieces:[{fractions:[a,b],oldMaterialVelocities:[vA,vB]}]` for each physical material edge. Supplying both is rejected. Pieces must cover [0,1] exactly, without gaps or overlap, and are copied during preparation.

[JointMaterialHistory](composite-joint-material-history.md) samples each tool's own accepted translational history at the current material labels. Its explicit reservoir supplies newly entering material. It preserves old field boundaries and one-sided traces, rather than extrapolating or shifting already current labels again.

[PiecewiseMaterialInertia](composite-piecewise-material-inertia.md) integrates those old fields exactly while keeping the same two geometric endpoints and six physical degrees of freedom. The JointAssembly selects this factory only for a piecewise record. Each subinterval uses the existing material-inertia operator, with an exact constant affine pullback into the parent edge; the Hessian is prepared once. The ordinary one-field path remains unchanged.

`tests/kirchhoffCompositeJointFeedHistoryTimeStep.test.js`: 2/2 pass. Opposite material feeds cross distinct old velocity boundaries and a declared external reservoir. An independent integral over the wire's current labels gives old momentum `.1*(1.25*1+2.75*3)=.95`, and its accepted mean material velocity is 2.375. The catheter independently retains -.75. Both original length/force gates and momentum balances pass with the unchanged three-node chart, then a second physical dt consumes the first step's actual accepted history. Cold/reuse, late rejection/retry, malformed coverage and ambiguous history checks pass without changing the input state.

Raw integration output: `/tmp/oet-composite-joint-feed-history-tests.txt`. The helper has 9 independent tests; the piecewise operator has 7, plus the 6 existing material-inertia controls.

This solves translational history sampling and integration on a fixed geometry chart. It does not yet transfer geometric nodes, rest metrics, frames/winding, boundary/contact reactions or material surfaces between changing topologies. Angular history is still explicitly unknown. It does not implement application feed controls or establish a frame budget.

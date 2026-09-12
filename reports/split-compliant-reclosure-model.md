Recommend a common nonsymmetric solve for physical and bias reactions, using the existing native row assembly. Alternating reclosure is mathematically possible for a correctly defined common system, but a small affine oracle needs 440 alternating cycles where the common block solves to 5.68e−14 residual. Merely repeating the current two phases, with renewed strain offsets or multiplier resets, does not solve that common system.

The parent supplied two separate infeasibility results for preserve-strain: full rank 1254/1254 with a required rim repair of 0.01632478 mm, and Farkas residual lower bounds 0.0005905154 and 0.0002259521, both above the stated 0.0002 solve target. These were not independently rerun here. The proposed model removes the artificial hardening of finite-compliance material rows. It preserves every native compliance, rest strain, EI, actual hard constraint and acceptance threshold. If the genuinely hard subset remains infeasible, the step must still fail.

Let h be the one physical timestep, M the generalized mass matrix, W its inverse on mobile DOFs, and A=physicalCompliance/h². Use XPBD displacement multipliers: physical impulse is lambda/h, and force is lambda/h². The native constitutive equation C(q)+A lambda=0 is essential: a change in final elastic strain requires a change in physical material reaction. This follows the force/compliance interpretation of [XPBD, equations 7–9](https://matthias-research.github.io/pages/publications/XPBD.pdf).

In one frozen tangent chart, define

```text
u = h (vPhysical − vPred)
q = qPred + u + b
```

qPred, vPred and the time-start history remain fixed throughout this timestep. b is the accumulated bias displacement, not the most recent correction. For rotations these are tangent increments with the existing quaternion retraction and frame transport; quaternion components must never be added, and finite rotations must not be assumed to commute.

Let E be the material Jacobian, N the normal Jacobian and T the physical surface-slip Jacobian. The proposed affine equations are

```text
M u = Eᵀ lambdaMaterial + Nᵀ p + Tᵀ t + physical control reactions
C(qPred) + E(u+b) + A lambdaMaterial = 0

M b = Eᵀ betaMaterial + Nᵀ betaNormal + bias control reactions
E b + A betaMaterial = 0

0 ≤ p ⟂ hPhysical(q,vPhysical) + A_normal p ≥ 0
0 ≤ betaNormal ⟂ g(q) + A_normal betaNormal ≥ 0
physical Coulomb law: ||t|| ≤ mu p, slip from vPhysical only
```

Here `hPhysical` retains the existing discrete normal-history law and must be recomputed at the final geometry. For an affine normal it is `max(0,g_start)+h*N*vPred+N*u`. Bias has no tangential friction row. Physical and bias normal active sets are independent. betaNormal never enters physical friction capacity, physical normal load, published velocity or impulse history.

The bias material equation is a finite penalty for the strain change induced by b; it is not `E*b=0`. At fixed physical pose qP=qPred+u, a nonlinear extension is `C(qP+b)−C(qP)+A*betaMaterial=0`. This is an auxiliary correction metric, not a change of the manufactured rest strain. The reference qP is derived from the current physical channel and immutable predictor, not recaptured from the last arbitrary phase endpoint. Native zero-compliance rows remain exact; after eliminating them, the following finite-compliance derivation applies in the remaining mobile space.

Writing `K=Eᵀ A⁻¹ E` and `H=M+K`, elimination of the two material banks gives

```text
H u + K b − Nᵀ p − Tᵀ t = −Eᵀ A⁻¹ C(qPred)
H b − Nᵀ betaNormal = 0
```

Thus changing b generally changes physical material reactions and u. “Bias does not update physical momentum” means beta is absent from the momentum equation. It cannot mean that vPhysical remains frozen despite a changed physical elastic force. Initial-overlap repair can change elastic potential energy; the bias contribution must be recorded as stabilization work, not relabeled as physical contact work. Independent position correction is known to permit potential-energy changes even when it does not directly change velocity, as discussed by [Catto in Solver2D](https://box2d.org/posts/2024/02/solver2d/). This proposal makes no energy-conservation or unconditional nonlinear stability claim.

The narrow native assembly adapter is the row-space version of these equations. Keep native gradients, W, compliance and the physical Coulomb solver. Own separate unknowns `[lambdaMaterial,p,t,physicalControls]` and `[betaMaterial,betaNormal,biasControls]`. For a frozen increment,

```text
dqPhysical = W * JPhysicalᵀ * dLambdaPhysical
dqBias     = W * JBiasᵀ     * dBeta
```

Use explicit residual-channel semantics:

| Residual reads | derivative against physical column | derivative against bias column |
|---|---|---|
| Final pose: physical material, position/orientation controls | Jrow W JPhysicalᵀ | Jrow W JBiasᵀ |
| Physical motion: physical normal and slip | Jrow W JPhysicalᵀ | 0 |
| Bias motion: bias material/control increment | 0 | Jrow W JBiasᵀ |
| Final geometry: bias normal | N W JPhysicalᵀ | N W JBiasᵀ |

Add compliance only to the owning multiplier diagonal. Before complementarity/friction handling the diagonal response blocks have the native form `J W Jᵀ+A`. Off-diagonal blocks are deliberately nonsymmetric. In particular, physical material sees bias while bias material does not see physical motion in the frozen affine model; bias geometry sees physical motion while the physical normal-motion row does not see bias. Do not symmetrize the resulting matrix. Physical pose controls belong with material-type pose residuals unless their DOFs are already eliminated from bias with exact `b_control=0`.

The zero cross derivatives in that table are exact only for frozen J. In the nonlinear model, derivatives of `J(q)*vPhysical` and `C(q)−C(q−b)` introduce extra cross terms. Omitting them is a quasi-Newton choice, not an identity. Refresh geometry and all channel residuals after every shared trial; use the existing final geometry, material, normal-history, control and Coulomb gates. A common line search must accept or roll back both channels and both reaction banks together.

One need not allocate a dense matrix with twice all rod DOFs. A response interface for the base block can reuse a factor of H:

```text
solveBase(fPhysical, fBias):
    b = solveH(fBias)
    u = solveH(fPhysical − K*b)
    return (u,b)
```

Use physical contact columns `(Jᵀ,0)` and bias normal columns `(0,Nᵀ)` to build the common contact Schur operator. In practice the direct row-space adapter may be narrower because native material rows and their hard constraints are already assembled. Native hard material/control rows must be kept in the existing KKT/nullspace machinery; never invert zero compliance. The shown H shortcut is a derivation and optional factor-reuse implementation, not an instruction to replace the existing native solver wholesale.

Implementation boundary for root/solver worker:

1. Add one frozen-system adapter with row channel flags and two owned reaction banks. Retain separate normal loads in physical and bias rows; attach all Coulomb capacity to physical p.
2. Start with the nonsymmetric common solve, reusing native assembly, current row identities and material coefficients. Include physical pose controls in cross-channel response.
3. Apply `dqPhysical/h` to the physical velocity channel and `dqPhysical+dqBias` to pose, once each. Accumulate b and both multiplier totals; scale both channel increments by the same accepted trial scale.
4. Recompute actual final C(q)+A*lambdaPhysical, geometry and physical history/slip residuals from that same trial. Do not certify separate phase endpoints.
5. Integrate external forces/damping once; keep predictors and start history fixed; do not restart beginStep, clear multipliers or perform another physical dt during reclosure. Commit history once only after all existing checks pass. The next predictor starts at accepted q using published vPhysical; no `(q_final−q_start)/h` reconstruction may include b.

Alternating methods are not intrinsically inconsistent. For affine frozen geometry, fixed contact active sets, positive M, finite positive compliance and no changing Coulomb branch, exact block Gauss–Seidel for the above common equations is contractive. With `R` the physical response constrained by its active normals and `P_B` the H-metric bias-contact response, its error map is `b_error_next=P_B*R*K*b_error`. In the H norm, its bound is `kappa=lambdaMax(H^(−1/2)*K*H^(−1/2))<1`. For one scalar mode, the factor is exactly `K/(M+K)`. This tends to one as stiffness dominates mass. Changing normals, material Jacobians, nonconvex geometry or Coulomb active sets invalidates that fixed-matrix convergence guarantee. A common linear block removes this particular cross-channel iteration bottleneck; it does not guarantee global nonlinear convergence or feasibility.

The executable independent oracle is `reports/probe-split-compliant-reclosure.mjs`; it imports no application code and advances no anatomy. A scalar elastic normal mode uses M=1, K=99, h=1/120, unchanged rest position `100*0.01632478/99`, and predictor `−200*0.01632478`. Its first physical solution has exactly −0.01632478 mm geometric gap. The one material row has full rank, so preserve-strain bias cannot repair it. Independent free axial slide and circumferential spin channels supplement this normal test.

| Method/check | Result |
|---|---:|
| One physical solve followed by compliant bias | material residual 0.016324780 mm |
| Correct alternating solve, 64 cycles | material residual 0.008666916 mm; geometry 0 |
| Alternating cycles to 0.0002 mm | 440 |
| Common block maximum algebraic residual | 5.68434e−14 |
| Common block u and b | 1.632478 mm each |
| Independent uneliminated native row adapter | agrees with eliminated H block |

The oracle additionally verifies zero normal/friction impulse for an unloaded free-mode overlap, with slide 3 and spin −2 unchanged at 60 and 120 Hz; correct physical linear/angular friction impulse balance with a loaded elastic contact; identical friction when betaNormal is multiplied by 1000 at fixed physical normal load; and nonzero physical recoil from free elastic prestrain. Both loads in the loaded test arise from solved equations, not manually zeroed velocities. All checks pass. This is an affine mechanics oracle, not native rod, rim-feature or full anatomy validation.

Run:

```sh
node /Users/macpiek/.codex/worktrees/0827/OpenEndovascularTrainer/reports/probe-split-compliant-reclosure.mjs
```

Results are saved in `split-compliant-reclosure-oracle.json` and the console summary in `split-compliant-reclosure-oracle.txt`. No root source, EI, rest strain, friction coefficient or acceptance threshold was edited in this task.

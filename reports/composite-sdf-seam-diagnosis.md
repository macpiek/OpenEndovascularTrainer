# Sparse-SDF P1 seam: bounded diagnosis and contact interface

The failed free-three-node P1 timestep has a numerically stable local equilibrium on x=65 with TWO positive normal reactions. Neither nearby single-cell continuation root is admissible. The current single-normal provider certificate cannot express this equilibrium. This is a contact-law issue at a nonsmooth level set, in addition to the observed line-search stall; smaller steps or larger budgets do not supply the missing force directions.

No repository source, query policy, asset, tolerance, or production solver was changed. `diagnose-seam.mjs` is a separate, read-only numerical experiment. It imports the exact Chain and compiled consistent inertia, assembles the same two length constraints, and solves tiny dense diagnostic systems. It never accepts a production timestep. `witness.json` contains all inputs, results, source SHA256 values and checks; `input-trace.json` is the root trace used for model parity.

## Reproduce and evidence

```sh
node diagnose-seam.mjs /absolute/path/to/OpenEndovascularTrainer input-trace.json > reproduced.json
```

Compare `sourceSHA256` and `externalSHA256` against the frozen witness before comparing results. The script verifies that its source files remain unchanged during execution. All 11 checks passed on the recorded source snapshot. Its first primal and dual Newton directions match the original trace to 1.74e-16 and 5.11e-15 respectively.

The fixture has three initially straight nodes, edge length 0.2397203545642866 mm, dt=1/120 s, EI1=EI2=GJ=0.001, mass per material length 0.01, zero old material velocities, fixed root spin, and one edge-0 capsule with radius 0.4445. The selected point remains its distal endpoint, t=1, sampleCount=1, source `sparse-sdf`.

| Diagnostic equations | q1.x | Physical reaction(s) | Admissibility |
|---|---:|---:|---|
| Left polynomial continued | 65.008935465666 | +1.680066659917 | Outside left cell; original gap -0.008687140291 |
| Right polynomial continued | 64.987037817507 | -0.631617844190 | Outside right cell and tensile; original gap -0.012504566992 |
| Both faces, gL=gR=0 | 65 | +2.192477870514, +0.598007423867 | Positive cone reaction at the seam |

The two-face solution is:

```
q0 = [64.97836989399583, -462.02966905032116, -79.55471756170732]
q1 = [65,                -461.81336005090407, -79.65575420693476]
q2 = [64.90596731869697, -461.60534501121300, -79.72892145713436]
length multipliers = [-0.38974649998033, +0.56835616232941]
spins = [0, approximately 0]
F = [1.0767936389, -1.6394534015, 0.5065181225], |F| = 2.0257967827
```

Cone mechanical force residual is 1.88e-11, torque 4.65e-15, length 2.01e-12, original query gap approximately -1.12e-14, projected wall residual at penalty 1e4 is 1.12e-10, and complementarity work is 2.46e-14. These use the original force/torque/length/gap tolerances. A single nonnegative force on the left normal leaves a best-fit force residual of 0.546898 at q1; on the right normal the best nonnegative force is zero and the residual is 2.025797. The endpoint force equations fix the two length multipliers, so they cannot repair this missing contact direction.

With eta_i=Fn_i/|G_i|, the symmetric constrained-energy Lagrangian is E+lambda*c-sum(eta_i*g_i). The four active constraint gradients are independent. Its Hessian restricted to the six-dimensional tangent space has eigenvalues approximately [0.00389562, 7.29030, 9.97071, 13.55049, 13.97146, 30.53107], using exact polynomial limits at the seam. Positive reactions and this positive tangent Hessian support a strict local minimum. This is numerical local evidence, not a proof that no distant single-branch equilibrium exists anywhere in the vessel.

The original stall is DIFFERENT: q1 approaches x=65 while gap=-0.00578143 and length error=0.00554985. That infeasible point is not a cone equilibrium. Its force residual jumps because the normalized gradient changes, whereas the scalar gap is continuous.

## Why this seam admits two inward normals

The adjacent cells are [433,60,48] and [434,60,48], voxel size 0.5, origin [-152,-492,-104]. Their shared face values agree exactly. For signed branch polynomials pL,pR and g_i=p_i-radius,

```
gL - gR = (x-65) * h(y,z)
```

The bilinear h is positive over this entire shared face: its four corner values are [0.80, 0.72, 1.16, 0.92]. At the solution h=0.976874568083. Thus the original piecewise scalar gap locally equals min(gL,gR), and admissibility is the INTERSECTION gL>=0 AND gR>=0. At their common zero the two positive inward normal rays generate the relevant reaction cone. Their dot product is -0.404509529; neither ray alone gives the required force.

## Proposed bounded interface for one proved face

This is an explicit interface extension, not permission to manufacture extra walls at all voxel boundaries.

1. **Inputs/provenance.** Supply the exact field storage, selected sample position and its fixed capsule weights/support, radius, raw original contact as a separate owned record, and one face identified by axis and adjacent integer cells. Keep sign, source, capsule sampling/winner, and material ownership explicit. The face evaluator performs no query and never labels a synthesized normal as `provider normal`.
2. **Admission proof.** Read the actual eight quantized corners of each cell. Require available finite corners, face adjacency, exact shared-face continuity, resolved matching sign, nonzero branch gradients, and a sample inside the closed common face with the other two coordinates strictly inside their cells. Prove the min/intersection relation on a stated tangential rectangle: h is bilinear, so its four corner extrema suffice for a strictly positive lower bound. If the sign/order proof only holds on a smaller rectangle, return that rectangle as the domain. An exact zero jump is smooth across the face; compatible coincident normal rays do not justify an extra independent reaction. Negative h gives max/union, whose boundary behavior is different; a generic positive combination is invalid there. Mixed-sign/unresolved h, multi-axis grid intersections, normal fallback, missing corners, or an unproved sign/source/winner change are separate unsupported cases.
3. **Rows and domains.** For a fixed sample p=sum(w_a*q_a), return for each face the exact one-sided gap g_i, G_i,a=w_a*grad(g_i), B_i,a=w_a*n_i, and DB_i,ab=w_a*w_b*(I-n_i*n_i^T)*H_i/|grad(g_i)|, with n_i=grad(g_i)/|grad(g_i)|. Also return H_i if needed for an energy-based restoration model. Use physical Fn_i>=0; the mechanical residual contribution is -sum(Fn_i*B_i), position tangent -sum(Fn_i*DB_i), and force column -B_i. G is NOT B and DB is generally nonsymmetric. Every row carries its cell, side, sample support, source proof and certified domain. The two cells are domains of the actual branch polynomials; their continuations outside those domains are only auxiliary local constraint functions where the proved min identity applies.
4. **Crossing event.** For a known affine trial sample p(alpha)=p+alpha*deltaP, compute the earliest grid-plane intersection in alpha in [0,1] from integer cell boundaries; retain the touched axes and the adjacent cells. Do not epsilon-nudge the sample or blindly accept repeated alpha=0 events. The caller must track which cell was exited/entered and consume a starting-face event explicitly. Simultaneous multi-axis hits require their own proof and are unsupported by this bounded interface. A capsule sampling/winner change invalidates this affine sample event and requires fresh original-query branch selection. This event partitions the PRIVATE nonlinear trial path; it is not a committed time substep or a timestep acceptance test.
5. **Global solve handling.** On a smooth branch, use that branch's usual local block. At an encountered proved min seam, let the same local block carry the two physical force unknowns and the two NCP rows; ordinary active-set behavior removes an open face. A restoration/filter or constrained-energy model using the continuous original gap can reach feasibility while the branch force residual is discontinuous. An interior cell seam at g<0 is an event to traverse or follow during restoration, never itself an accepted wall constraint x=constant. Away from the tie, a loaded extension of the wrong cell is not admissible; certify the original selected branch and require the other force to be zero. At g=0 with a proved tie, admit the cone. Do not relax final force, gap or complementarity tolerances to cross a discontinuity.
6. **Original final cone certificate.** Requery the unchanged provider. Check its original gap and source/sample provenance, independently evaluate the admitted cell polynomials, and verify the original gap equals the proved piecewise min in the stated domain. Check branch gaps, Fn_i>=0, each original NCP/complementarity equation, and tie/domain admissibility for every loaded branch. At a true seam, both branch limits must coincide with the original gap. Sum the physical forces and their moments about the SAME reference point and verify the full original mechanical equilibrium, including length forces, inertia and elastic forces. Keep the two scalar reactions and resultant vector; sum(Fn_i) is generally NOT |sum(Fn_i*n_i)|. Original gap alone does not certify the reaction cone. A legacy certificate that insists on one selected provider normal is unable to certify this solution and must explicitly reject it until this contact-law extension is integrated.

## Grid artifact versus a true distance ridge

A voxel face is a storage boundary. Usually it has no active contact, may have a smooth normal, or meets g=0 with only one loaded branch. It must not automatically become a physical plane wall. Here the trilinear interpolant has an actual corner in its radius-offset level set; both normals are physically consistent with THIS discrete geometric model.

That does not prove that the underlying vessel surface has a geometric corner or that its exact distance has a medial-axis ridge there. A genuine distance ridge comes from competing closest surface features and requires corresponding geometric provenance. A trilinear gradient jump can be solely an interpolation artifact. The proposed proof classifies the local piecewise polynomial level set and its feasible tangent geometry; it does not claim an anatomical ridge. Source changes such as sparse-SDF to BVH are a separate interface and were not modified or inferred here.

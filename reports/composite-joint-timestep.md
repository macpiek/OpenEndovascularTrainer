# Prepared nonlinear joint timestep — bounded full-motion reference

`kirchhoffCompositeJointTimeStep.js` integrates one fixed-topology physical dt in common positions q, three relative wire coordinates at **every** overlap node (including tips/endpoints), and separate material spins. It uses `JointAssembly`, both original `ToolLengths`, and **one** `RelativeDirection` band LU. There is no independent patch elimination or alternating rod solve.

This scope explicitly requires `contacts:'none'` and `torsionMode:'quasi-static'`. It does not implement vessel/lumen contact, friction, angular inertia, topology changes, remeshing, adaptive reduction or app integration. With contacts absent the two physical materials remain mechanically independent; the common/relative representation is a full change of variables, not an artificial bonded catheter/wire. An accepted result certifies the original discrete force, torque, separate lengths and prescribed positions in this bounded model, not the complete endovascular simulation.

## State and prepared input

`createCompositeJointTimeStepState({layout,coordinates,positions,relative,modes,angles:Map,tools,restLengths:Map,materialCoordinate:'reference-arclength',relativeToolId:'wire',lengthMultipliers?,time?,step?})` owns numeric geometry, bases, spins, each material's frames/winding/rest metric and multiplier/history arrays. Material provider functions remain borrowed immutable providers. Complete fixed bases can have any orientation. Missing overlap modes or reduced transverse modes are rejected.

`advanceCompositeJointTimeStep(state,{dt,torsionMode:'quasi-static',contacts:'none',inertia,boundaries,loads,tolerances?,budget?,elementBackend?,assemblyPolicy?})` returns an owned new state only on success. Explicit inertia contains `previousPositions:Map<id,points>` and one `inertiaEdges[e].tools` record per physical material. Every record retains its own density, affine material map and old physical velocity samples at the **current** labels. Previous geometry must equal the incoming state's physical tool geometry in the same chart. Input frame tangents must also belong to that material's physical incoming axis.

One scalar positive `tools[i].dsDx` declares the fixed affine **reference-arclength** measure of each tool. Every inertia edge slope must agree with it and `restLength[e] = (x[e+1]-x[e])*dsDx`. The check permits only 64 eps relative roundoff, independently of physical solver tolerances; incompatible mass/elastic/rest measures are rejected, never repaired. Material labels are continuous across edges. Callable/variable elastic slopes and arbitrary label metrics are explicitly unsupported. Prepared map time integration and resampling old material velocities at current labels remain caller responsibilities; missing histories are not silently replaced with the other axis or zero velocity.

Position BCs are `{toolId,node,value:[x,y,z]}`; spin BCs are `{toolId,edge,value}`. At least one explicit spin boundary is required for each quasi-static material. Loads are physical `{toolId,node,value:[fx,fy,fz]}` forces and `{toolId,edge,value}` torques. Common-axis position targets use exact q Dirichlet conditions. A wire target inside overlap uses a local original `q+B*rho-target` row with `forceColumn=-J`; its multiplier is the physical boundary force **on the wire**. It does not fix rho merely because the catheter is held.

If both physical endpoints of an edge have full prescribed targets, their target separation is independently checked against its original rest length. The redundant length dual is then omitted with the explicit gauge lambda=0. The final original length test still includes this edge. Prescribed-node support reactions absorb this choice of stress gauge. Other singular original Jacobians remain honest failures; no pivot floor, diagonal shift or regularization is introduced.

## Nonlinear acceptance and history

Default backend is Exact; explicit WASM GN and JavaScript GN are also tested. Each direction assembles current material tangents, relative/cross blocks and exact length stress tangents in one local band. Original residuals include loads and signed reactions once. Physical wire force is decoded from the full relative gradient using B; the other material force is common minus wire. The nonlinear force gate checks individual physical nodal vector norms, not just the summed common force.

Default final gates are force 1e-7 N, torque 1e-8 N mm, each material length 1e-8 mm and physical target distance 1e-9 mm. Linear force/torque gates are 5e-10 in their respective units and each linear constraint gate is 5e-11 mm. No tolerance was relaxed to make a test pass. Newton backtracking uses original residual merit; exact gradient-only trial/commit evaluations skip unused Hessians. Full Hessians are rebuilt before every actual direction. All evaluations, directions, factorizations and linear solves are counted against explicit budgets. A fresh original certificate is evaluated again before any commit.

Accepted frames time-transport each tool's **own** previous reference director to its own current tangent; winding is unwrapped to that same tool's preceding anchor. Accepted velocities use each own actual/previous geometry and frozen map, `v=(p-new - p-old)/dt - dsDt/dsDx * p-new,x`. No shared-axis velocity is substituted. Numeric state, time, multipliers and histories change only after all certificate/frame/velocity work succeeds. Budget, line-search or numerical failure returns the original object unchanged. Malformed prepared input throws before mutation. Repeated identical retries produce identical states/certificates (wall-clock timings naturally differ).

## Validation and a deliberately small cost probe

13 new tests pass; together with JointAssembly, MaterialInertia, ToolLengths, RelativeCluster and RelativeDirection: **62/62 PASS**. Controls include:

- Different finite physical axes and rest metrics (wire dsDx=1.02, catheter=1), own histories and exact physical mass.
- Independent free rigid velocities and per-tool momentum balance; no artificial catheter drag.
- Wire feed 0.002 mm and rotation 0.4 rad with every catheter node fixed. Wire support equals `mass*feed/dt²`; catheter reactions vanish within the original gates. All separate lengths remain valid.
- Pure through-chart feed counted once in the accepted physical velocity along the wire's own tangent.
- All positions of both tools prescribed, explicit redundant length-row removal, and independent support/impulse balances.
- Nonlinear bending with separate loads/supports, followed by another accepted dt using the first dt's accepted velocities at unchanged labels.
- Wire winding 4 pi and catheter winding -2 pi retained over two commits; rotated full bases give the same nonlinear physical response.
- Catheter tip inside a longer wire, including its full relative tip mode and separate inactive/active histories.
- First-trial, deformed-trial and late precommit rejection with rollback; deterministic retry; Exact/GN/JS and lazy/full policy parity.

`composite-joint-timestep-probe.json` records three paired first/continued dt runs for 5 and 33 nodes. It uses the explicitly declared **synthetic test** materials, dt=0.06 s, full overlap, zero map rate and the same small wire tip load. Continued steps use the accepted preceding material velocities; all constructors are rebuilt on both first and continued calls. Thus “warm” means a continued physical state/JIT, **not** a persistent compiled workspace. The embedded frozen probe script records the exact protocol.

All 12 attempts accepted, each with 2 directions / 6 evaluations. Common plus full relative plus length unknown counts were 46 and 326, and half-bandwidth was 27 in both sizes. Median timings (ms; tiny sample with host/JIT/GC noise, not a performance certificate):

| Nodes / step | Preparation | Iterations | Commit | Whole dt |
| --- | ---: | ---: | ---: | ---: |
| 5 first | 3.259 | 4.994 | 1.070 | 8.966 |
| 5 continued | 2.880 | 6.387 | 0.346 | 9.673 |
| 33 first | 12.956 | 21.444 | 0.399 | 33.811 |
| 33 continued | 11.530 | 28.278 | 0.730 | 52.000 |

Medians of separate columns do not sum to the median total. These numbers expose constructor/scatter/workspace overhead and the full iteration cost; they must not be substituted for realistic anatomy/profile measurements or a 60 FPS claim. This bounded change establishes the full-motion reference before contact integration and later measured persistent-workspace/reduction work.

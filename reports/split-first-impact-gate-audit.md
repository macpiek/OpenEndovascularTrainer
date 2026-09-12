# Discrete first-impact law versus terminal-velocity gate

**The incompatibility is reproduced on the requested stable root source.** A .01-mm positive-gap, frictionless first-impact case satisfies the implemented discrete contact law, closes the actual geometric gap, and has zero final contact KKT error. The additional closing-velocity gate nevertheless rejects certification and commits no history. This is an audit proof of a conflicting acceptance requirement, not a solver-acceptance PASS and not a proposal to remove the gate.

Source: `/Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer`, World SHA prefix `2d820149`, SplitMotion `53128abc`. All 53 physics JavaScript hashes are identical before/after the three-case audit. Full source hashes, raw observations, native solve rows, and phase diagnostics are in `reports/split-first-impact-gate-proof.json`. Reproduction script: `reports/probe-split-first-impact-gate.mjs`. No production source, tolerance, or solver option implementing a new contact law was changed.

## Analytic setup and equations

The lumen is the affine half-space y<=0, with inward normal -Y. A straight two-node rod has radius .5 mm, node mass 1, isotropic segment inertia 1, length .5 mm, zero initial material strain, no damping, no external force, and mu=0. Both nodes initially have closing velocity +2 mm/s. A parallel supported rod supplies the required native joint World component but never contacts the moving rod or wall. The timestep is 1/120 s. The small normal displacement does not invoke the swept/CCD path (`sweptWitnesses=0`).

Let `v_c` denote velocity toward the wall (+Y). The implemented row, in `kirchhoffSplitMotion.js:189`, is:

```
w = max(0, g_start) + dt Jv = g_start - dt v_c
lambda >= 0, w >= 0, lambda*w = 0
```

For a positive gap crossed by the free predictor, a positive reaction is necessary. The active row therefore imposes `w=0`, hence:

```
v_c = g_start / dt
I_normal = sum(m_i * (v_predictor_i - v_c_i))
```

For the nominal data, this is v_c=1.2 mm/s, impulse .8 per unit-mass node, and total normal impulse 1.6. The integrated displacement `dt*v_c=.01` reaches the wall exactly. The published velocity remains positive toward the wall.

The final gate, in `kirchhoffSplitMotion.js:312` through `:318`, selects contacts whose actual final gap is at most .001 mm and additionally requires:

```
maximumOutwardContactVelocity * dt <= .001 mm
```

At the active closed contact this requires `dt*v_c<=.001`, whereas the discrete active law requires `dt*v_c=g_start`. Consequently, **a positive initial gap greater than .001 mm is incompatible with the two exact conditions for this first-impact fixture**. The .01-mm example is also well outside tolerance ambiguity: reducing dt*v_c to .001 while retaining positive lambda would leave the declared physical row residual at least .009 mm, above the .001-mm physical gate tolerance.

## Observed .01-mm case

Float32 storage makes the actual starting gap `.009999990463256836` mm and the predictor velocity `2.0000016689300537` mm/s at each node. The independent expectation uses those stored values rather than hiding the rounding difference.

| Quantity | Native result |
| --- | --- |
| Raw final gaps | `[0, 0]` mm |
| Published closing velocities | `[1.1999988555908203, 1.1999988555908203]` mm/s |
| Independent `g_start-dt*v_c` | `[0,0]` mm |
| Physical normal multiplier sum | `.013333380123678634` |
| Normal impulse `sum(lambda)/dt` | `1.600005614841436` |
| Independently measured predictor-to-final momentum loss | `1.6000056266784668` |
| Final physical contact KKT / cone violation | `0 / 0` |
| Final material adaptation / bend-twist residual | `1.851857556206524e-11` mm / `0` rad |
| Physical/bias passes | `1 / 1` |
| Physical accepted / bias accepted / final residual settled | `true / true / true` |
| Bias normal multiplier / bias elastic energy change | `0 / 0` |
| Swept witnesses / unsupported limitations | `0 / []` |
| Closing-velocity gate residual | `.009999990463256836` mm |
| Certified / World closure converged | `false / false` |
| History commits | `0` |

The linear solves themselves converge: physical residual about `3.70e-11`, bias residual 0. Capsule and endpoint wall rows carry positive physical normal reactions; no friction force participates. The failure is localized to the extra closing-velocity condition, not geometry, material residual, Coulomb feasibility, numerical convergence, or a failed unsupported-path check.

Kinetic energy includes translation and the isotropic material spin. Nominal initial energy is 4; after Float32 prediction it is `4.000006675723`; final energy is `1.4399972534192784`. Energy decreases by `2.5600094223037217`. Thus this example does not create energy or a bounce away from the wall. It retains closing kinetic energy that the stricter terminal condition rejects.

## Controls

| Stored initial gap, mm | Final closing speed, mm/s | Gate residual, mm | Certified / commits |
| --- | --- | --- | --- |
| 0 | at most `1.12e-8` | `9.26e-11` | true / 1 |
| `.000500023365020752` | approximately `.06000281` | `.0005000234581530094` | true / 1 |
| `.009999990463256836` | `1.1999988555908203` | `.009999990463256836` | false / 0 |

All three have raw final gap zero and converged physical/bias rows. The small positive-gap control shows that the extra gate tolerates a small nonzero closing speed; it is not an exact zero-terminal-velocity condition.

## Interpretation for the next model decision

The implemented position-level discrete contact approximation uses one velocity over the complete timestep. At first impact that velocity also equals the average displacement needed to traverse the available gap. It need not equal the continuous trajectory's terminal velocity after a zero-restitution collision. Publishing it as the next physical velocity is the current discrete model; the extra gate asks for a stricter terminal result that this active equation does not provide.

For the corresponding continuous frictionless trajectory, nominal impact time is approximately .005 s: travel with incoming velocity 2 until reaching the plane, then stop for the rest of the timestep. The same final position is compatible with zero terminal velocity, but the trajectory and impulse accounting differ. With the observed current state, a subsequent terminal stop would require an **additional** total normal impulse `2.3999977111816406` and removal of the remaining approximately 1.4399972534 kinetic energy. In a coupled rod/contact system that impulse must be solved and applied consistently, including material response and any friction budget.

This audit does not choose between accepting the existing discrete first-impact approximation and implementing a separately defined terminal-impact/CCD treatment. It does establish that a numerical convergence fix cannot make these two current requirements compatible in the .01-mm case. Silently zeroing the published velocity would omit a real impulse; applying such an impulse while reevaluating the unchanged full-step complementarity row would violate that row. A terminal projection or CCD decision therefore needs its own coherent equations, position/velocity staging, reaction accounting, and history semantics. No automatic gate removal or replacement velocity is proposed here.

## Reproduce

```sh
node reports/probe-split-first-impact-gate.mjs
```

The script verifies the requested source identities and unchanged before/after hashes, records all cases, and labels the .01-mm native result as uncertified. Successful script completion means the mathematical proof and measured balances were checked; it does not relabel the failed native certification as an implementation PASS.

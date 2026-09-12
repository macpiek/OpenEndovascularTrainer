> Aktualizacja 7 września: poniższy zapis odrzucenia swobodnego P1 jest historyczną diagnozą. Zintegrowana poprawka dwóch reakcji na granicy komórek przechodzi pierwszy i drugi fizyczny dt z oryginalnymi kryteriami. Bieżący wynik i jawne ograniczenia: [composite-sdf-cone-timestep.md](composite-sdf-cone-timestep.md).

# Physical wall derivatives in the complete common-chain step

The fixed-topology mixed timestep now consumes the true provider gap Jacobian
G, the signed physical normal force column -B and the complete nonsymmetric
normal derivative -Fn DB. Raw capsule and endpoint queries remain unchanged.
Physical Fn keeps its original force units. Endpoint duplicates and interior
row reductions must preserve the complete globally scattered g/G/B/DB contract.
The frozen five-file collector handoff and dependencies were SHA256-verified
before and after import; its manifest is in
[the collector report](composite-wall-differential-rows-source.json).

Exactly open, unloaded wall rows (g > 0 and Fn === 0) are eliminated from the
linear matrix: the original inactive NCP equation gives deltaFn = 0. Every raw
gap still enters the full acceptance proof and is requeried on every trial.
Loaded open rows remain in the solve. Unknown active/loaded derivatives reject;
unknown open zero-force rows need no invented gradient. This is an algebraic
elimination with no geometric threshold. Persistent row DOFs/derivative buffers
replace temporary arrays during force measurement; the mixed workspace also
retains its position/spin classification.

## Positive controls

The original anatomy witness has g = -0.02 mm and gradient norm 0.41087864,
which must not rescale physical Fn. A three-node control holds X/Z and permits
Y translation, staying on the supported smooth polynomial branch. Both capsule
and envelope complete the strict physical step in **two directions**. The
capsule force residual is 9.13e-13, length residual zero and projected contact
residual 6.00e-11. Independent momentum integration includes the original wall
forces and prescribed boundary reactions. Detection options are unchanged.

The open unsupported-source control has the same positions and material
velocities as the contact-free calculation, with the same mixed dimension;
all original queries still execute. Adding a nonzero physical Fn to that same
unknown source rejects without changing accepted state or time. Physical-normal
AL on sparse-SDF explicitly rejects its inconsistent potential.

The integrated suite passes **272/272** tests. This includes five new timestep
tests and ten collector tests. One of the new timestep tests deliberately
checks rejection of the unresolved case below; it is not a successful physical
scenario and does not establish general anatomy convergence.

## Open defect: discontinuous cell normals

The fully free three-node version of the same witness still rejects after
17 directions and 215 evaluations. It approaches the SDF cell boundary at
X = 65 mm from alternating sides. The inward normal X component changes from
approximately +0.74 to -0.92 across that boundary. Gap is continuous, but the
physical force residual has a finite jump as the trial displacement tends to
zero. No unsupported-source or derivative error occurred in this trace.

The last accepted trial has force residual 0.4969, length residual 0.00555 mm
and gap -0.005781 mm. An arbitrarily small crossing raises the force residual
to about 1.459. The monotone original-residual line search consequently stalls.
The complete step is rejected, with no geometry, reaction, history or time
commit. Final tolerances were not relaxed.

This is an unresolved contact-model/globalization issue at a nonsmooth seam.
Local exact derivatives alone cannot certify a solution there. The diagnostic
task is checking whether a single branch has an equilibrium or multiple
admissible normals are required. The successful smooth-branch control must not
be presented as fixing this free-motion case. Selected branch data and the
whole-step outcomes are preserved in
[the integration witness](composite-sdf-timestep-integration.json).

BVH derivatives, generalized seam handling, coupled lumen modes/friction,
moving material boundaries, adaptive transfer and application integration
remain open. The new common-chain modules still do not drive the browser's
`joint-two-channel` solver. No 120 Hz / 60 FPS result is claimed.

# Wspólny solver — bieżąca integracja

Stan bieżący opisuje [raport postępu](coupled-rebuild-progress.md). Pełny
nieliniowy World jest już zintegrowany przez API `jointMotionMode:
'split-physical-bias'` i `configureKirchhoffSplitBias(joint,
{materialMode:'coupled-compliance'})`. Nie ma jeszcze domyślnego włączenia
tego wariantu w aplikacji ani wyniku 60 FPS.

Poniższy zapis zachowuje poprzednie dowody suchego rozwiązania. Informacje o
braku integracji World, 480 testach i otwartym błędzie trzech kroków tarcia
statycznego dotyczą wcześniejszego checkpointu. World jest podłączony,
błąd tarcia statycznego naprawiony; trwają integracja release kontaktów i
poprawa zbieżności przy dalszym wsuwaniu. Pliki `*-integration-pending.patch`
są historycznymi, już zastosowanymi patchami; nie należy ich ponownie nakładać.

## Checkpoint przed integracją World (historyczny)

This checkpoint implements the complete **frozen linear block**, exact paired
material condensation, and separate physical-pose/multiplier application for
two channels on the native two-rod Jacobians and mobility. The nonlinear World
timestep is still pending integration. This is not a successful FPS result.

`kirchhoffTwoChannelSystem.js` retains independent physical and bias multipliers.
Physical material and pose controls read the total pose correction. Physical
normal and friction equations read only physical motion. Bias material reads
only bias motion; geometric contact reads both. The cross blocks are generally
nonsymmetric. Friction cones reference only the physical normal row.

The adapter reconstructs both generalized responses separately and checks the
original equations through `J*dq`, independently of matrix multiplication. The
returned pose correction is their sum, while returned native material multiplier
increments are physical only. The imported `kirchhoffTwoChannelMotion.js` applies physical motion to a
separate pose and physical velocity, transports angular increments through the
current geometric frame, and commits only the bias material bank at the same
accepted scale. Geometric pose application, contact/control bias banks and
committing one physical timestep remain the World caller's responsibilities.

The [model derivation](split-compliant-reclosure-model.md) and independent affine
oracle explain why preserving every strain is infeasible at fixed supports.
Alternating compliant correction and physical equilibrium can solve the same
fixed equations, but a stiffness/mass ratio of 99 yields contraction 0.99 and
440 iterations to a .0002 tolerance in that oracle. The simultaneous block
solves that affine case directly without changing stiffness or tolerances.

## Validation

- Four native adapter tests pass: a pinned compliant rod with an independently
  calculated physical/bias response, pose-control cross terms, complete explicit
  row metadata, and ownership after another assembly.
- Fifteen [nonsymmetric Newton tests](kirchhoff-coulomb-nonsymmetric.md) pass.
  The existing symmetric-band path is unchanged; a separate 32-pair baseline
  comparison verifies byte-identical outputs and Newton iteration data.
- A separate [adapter review](two-channel-adapter-review.md) found no actionable
  mapping, compliance, recovery or reconstruction error. Native assembly still
  updates borrowed scratch and prescribes hard orientation controls.

## Actual short-insertion state

The [dry runtime probe](../scripts/physics/probe-two-channel-frozen-runtime.mjs)
captures the fifth physical pass at 7.3667 mm of catheter after preparing
12.1 mm of guidewire. The original preserve-strain phase later fails at this
state. The new joint frozen block includes 155 physical and 143 bias rows and
converges in three Newton factorizations, with original/reconstructed residual
about .00001083 against the unchanged .0002 target and proposed scale 1.

The candidate is **never applied**. A transaction restores the original runtime
after the dry solve; the final prepared pose fingerprints and step count match
the original failing replay. Source hashes are stable. The [raw result](two-channel-frozen-runtime.json)
and [frozen matrix input](two-channel-frozen-runtime.system.json) are retained.
The input encodes infinite bounds as the strings `Infinity` and `-Infinity`.

The integrated condensed solver now eliminates 118 paired equalities with one
unshifted band/skyline factorization, leaving 62 retained rows from 298. The
structurally immovable original rows 3 and 5 retain their exact nonzero RHS,
identities and both channels. All inequalities and friction members remain.
The successful path allocates 3,844 Schur entries and no expanded 88,804-entry
matrix. Recovery is checked through the complete native J*dq equations, and
friction cones use recovered physical normal loads exclusively. Unsafe or
unsupported equality elimination explicitly falls back to the dense reference;
its attempted factorization still counts as work.

A [new native dry probe](two-channel-condensed-runtime.json) passes the complete
298-row residual at .000108614831 against the unchanged .0002 tolerance, with
cone violation zero, one material factor and two Newton factors. It remains a
dry solve: the candidate is never applied, and the original runtime/fingerprints
are restored. Cold diagnostic times were 110.69 ms dense and 19.21 ms condensed
including adapter recovery/certification. The separate frozen-only worker
comparison measured a 4.20x median gain (53.16 to 12.66 ms, three warm pairs).
These scopes differ and neither is a complete timestep or browser FPS result.
The remaining cost still exceeds the target step budget.

The native motion+system oracle closes physical and bias material residuals
below 1e-12 mm while preserving alpha=1/99, with separate qP and qG and only
physical displacement entering velocity. Root also updates qP at a prescribed
hard frame so operator rotation is not counted as bias strain. Its regression
checks two prescriptions without a duplicate angular impulse. Whole-step and
trial snapshots retain qP, beta, and application receipts in place.

Independent review of the integrated adapter found a factor-accounting omission
on unsafe fallback; the native regression now passes after the attempted factor
is included. Mechanical recovery, physical cone mapping, zero-axis cones and
initial-increment mapping had no actionable findings in that bounded review.

The next dependent World hooks are prepared in
[two-channel-world-integration-pending.patch](two-channel-world-integration-pending.patch)
but are NOT applied. They await the delegated `kirchhoffTwoChannelRows.js` bias
contact/control bank implementation and its tests. There is no new selectable
World mode yet. The provisional row adapter guards disappearing or changed
loaded witnesses; implementing their mechanically consistent release remains
necessary for unrestricted tool motion.

## Wall friction and remaining work

World now captures incoming motion before force/damping prediction, retries a
static-to-kinetic transition from the same post-integration transaction, and
commits certified mode history once. The discarded static force does not remain
in the accepted motion; its computational cost remains in diagnostics.
The actual applied coefficient is reported in contact statistics.

The native World tests pass breakaway, existing slip, zero kinetic friction,
failed retry/input rollback and the equal-coefficient path. A three-step static
test remains explicitly failing: a distal zero-load witness loses its stop
certificate when redundant reactions redistribute. A bounded kinematic proof
handles a coincident capsule/point witness, but does not yet infer the distal
node's relation through hard material constraints. Velocities and tolerances
were not altered to hide this case.

The complete coupled suite at this checkpoint has **480 tests: 478 pass, two fail**
(15.77 s). The native fallback-accounting regression passes. Build, generated
documentation check and `git diff --check` pass.
Failures are this new static-history witness and the previously recorded
position-history translated-mouth lifecycle test. The default application
continues to use `reference`; no new deep-insertion or browser FPS acceptance
has been claimed. Nonlinear two-channel World integration, loaded witness lifecycle, adaptive
discretization and deep/max manipulation validation
remain required by the active goal.

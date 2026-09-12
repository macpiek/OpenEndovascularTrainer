# Fresh initial bias certificate

The optional split World now measures every existing nonlinear acceptance gate
before requesting the first bias correction. A state already within those gates
does not require a linear solve at the tighter inner tolerance. The physical
phase still runs, and its failure still rejects the entire timestep.

The precheck initializes the current fold rows and invokes the same fresh World
measurement used after a correction. It checks length, material adaptation and
bend/twist, fold geometry, orientation controls, contact and boundary residuals,
friction residuals and cones, and pending tool release. No tolerance, material
profile, physical velocity, force, or geometric rest configuration is changed.
`biasInitialStateSettled` and `biasInitialMerit` record the decision. A measured
phase counts as a phase evaluation; the actual solve/factorization counters and
trial hooks are not fabricated for an omitted correction.

Three regression cases cover the optimization:

- A stationary .0005-mm wall penetration passes the original .001-mm geometry
  gate with zero bias solves, unchanged pose and velocity, and one history commit.
- A .002-mm penetration fails the precheck, receives a bias solve, and closes the
  raw gap while retaining zero physical velocity.
- A rejected physical phase remains rejected even when initial bias is settled;
  World restores the original state and commits no physical timestep.

The independent wall oracle's path assertion now accepts either a real bias
trial or an explicit fresh initial certificate. All force, friction, momentum,
radius-moment, geometry, and accuracy assertions are unchanged.

## Runtime witness before the consistent normal certificate

[`split-first-coupled-precheck.json`](split-first-coupled-precheck.json) records
regular fixture commands on the synthetic vessel: first 33 wire steps to
12.1 mm, then 10 catheter commands to 4.333 mm. There are no injected pose or
force changes in the observation hooks. The first eligible coupled step has:

- five accepted physical solves;
- initial bias merit .844541, zero bias solves, and accepted bias;
- fresh physical KKT residual 1.4492e-6 mm, material adaptation 3.5720e-5 mm,
  bend/twist 1.1599e-10 rad, no unknown history features, zero bias energy change.

It still rejects certification because the old additional closing-velocity
condition conflicts with the discrete normal law. The independent proof is in
[`split-first-impact-gate-audit.md`](split-first-impact-gate-audit.md).
The transaction therefore leaves the World at 42 accepted steps and restores
the candidate. This report does not classify the step as accepted or establish
FPS. Its source hashes identify the exact pre-certificate checkpoint.

Validation at this checkpoint: 36 focused tests pass; the complete coupled suite
has 375 tests, 374 passing and the previously recorded translated-mouth
position-history lifecycle failure. No acceptance threshold was weakened.

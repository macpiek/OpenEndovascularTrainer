# Known-open prescribed nodal wall contact — frozen correction

The same full two-tool 120 Hz input previously rejected with `original-linear-equations` after 1 direction. It now accepts in 2 directions / 6 evaluations, with normal forces `[0, 0, 0]` and both tangential components exactly zero at every wall pressure site. This is a correctness fix, not a performance or FPS claim. World/UI integration is outside this patch.

## Algebra and freshness

The eligible site must be part of the existing explicit `nodal-endpoints` pressure law, carry an explicit full physical position boundary for its owner/node, and be exactly at that target in the current actual physical geometry. Its just-completed ORIGINAL query must give `gap > 0`. Under these conditions the original unilateral equations uniquely give `Fn=0`; the Coulomb disk of radius `mu*Fn` uniquely gives `Ft=0`. Previous signed/private force magnitude, a force threshold, and `hasFreeGeometry` are not eligibility conditions.

The timestep backs up the residual before wall assembly, substitutes these known algebraic unknowns, restores that residual, and reassembles normal mechanics/rows on the same verified geometry with `query:false`. Thus an old reaction is removed exactly once and no additional detector query is hidden. The original wall certificate, all endpoint/capsule inequalities, current physical Coulomb rows, forces and tangents remain freshly evaluated. The mandatory final evaluation performs an ORIGINAL query again before normal/friction history commit. Counters separately report `wallKnownOpenBacksubstitutions` and `wallKnownOpenRowRefreshes`; the physical fixture has two substitutions and one derivative-only row refresh.

A closed site (`gap == 0`) retains its accepted pressure gauge. A free physical position, capsule pressure coordinate, or not-yet-achieved dual physical BC is not changed by this rule. A wire overlap BC represented by dual rows qualifies only when the actual physical point equals its target exactly; this bounded correction does not reparameterize dual BCs, snap geometry, infer a future target gap from another position, or remedy a general dependent-constraint system. Unsupported original contact branches/provenance still reject. Existing contact-query and nonlinear budgets and all original gates are unchanged.

## Reproduction and evidence

- Existing real JointTimeStep fixture: three nodes, actual separate wire/catheter axes, own reference frames/spins, full own inertia/material histories, two independent rest metrics, wire feed −0.3 and catheter feed 0, both lumen and wall Coulomb, dt=1/120 s. The mass/stiffness values are the existing synthetic fixture, not measured device material.
- Begin with accepted loaded wall forces `[0.38615348816463735, 0.16255508872752597, 0]`. Prescribe all three catheter physical positions translated by y=−1e−4 mm, hold its own spins, retain independent wire mechanics. All six original endpoint/capsule gaps are 9.9999999999988987e-05 mm.
- Prepared input hash (same before/after): `2f9f99f8551f597cb9d3291be20e46a69d2420358b27ff3ba07f86ad4041024a`. Initial loaded accepted states before/after are bit-identical. Frozen old Step is the prior independently audited source, with all other dependencies held identical to the correction snapshot.
- Accepted original force residual 4.15e-12 N; maximum independent per-tool momentum balance residual 9.71e-14 N. All own original length/BC/normal/friction certificates pass.
- Normal NCP scales .1/1/10 and friction scales 5/50/500 produce bit-identical accepted state; these are numerical equation scales, not pressure stiffnesses.
- Closed held control keeps original Fn bit-identically and performs zero known-open substitutions. The independent zero cone at the third site stays literal zero. Only the two algebraically dependent prescribed catheter length rows are suppressed, and both original lengths are still checked.
- Cold/reused states match bit-identically. Early query-budget, zero-direction budget, late query-budget and late evaluation-budget failures return the original state and preserve all nonzero accepted histories; each retry matches the cold accepted state.
- Affected tests: **120/120 PASS**, 2.468 s; source syntax PASS. Root subsequently reported complete composite **656/656 PASS**, 11.986 s, build PASS, 1.77 s (root-run result, not an extra worker run).

## Ownership and frozen files

Only `src/physics/kirchhoffCompositeJointTimeStep.js` and three appended tests in `tests/kirchhoffCompositeJointFrictionTimeStep.test.js` changed. No package, wall manager, normal row, direction or UI edits.

- Step SHA-256: `68863abe08a15b06b0cd8037a135a669fc1e3e05f380e9c149d10aa1ef651c8e`
- Test SHA-256: `84c71f06dfcbe247754d4177c561ff76ddffeb12cee4a4884dd583335b0ee74c`
- Frozen bundle: `/tmp/oet-joint-known-open-dirichlet-fix`
- Reproduce: `node /tmp/oet-joint-known-open-dirichlet-fix/probe.mjs`
- `source.json` records 45 transitive source/test hashes. `manifest.json` additionally freezes the previous Step, generated fixture with test registration disabled, probe, and raw evidence. Runtime Node/bare npm packages are borrowed from the existing worktree through the snapshot node_modules link, not copied as a separate distribution. Source hash check after the probe: `True`.
- Previous baseline audit and manifest are preserved unchanged at `/tmp/oet-production-nodal-wall-audit`.

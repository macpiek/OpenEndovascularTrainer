# Actual World wall sources in the common Joint step

The production World bridge can now derive its wall constraints from actual body collision ranges, node radii and friction coefficients, using the original `World.contactField`. Enable this with `worldWall:{...}` in `createCompositeJointWorldAdapter`. The simulator selector is still unchanged; this is not yet the full application integration or a 60 FPS result.

## Source mapping and physical ownership

`prepareCompositeJointWorldWall` expands every original active, exposed body capsule into its actual union subedges. Every child retains the parent's maximum endpoint radius, matching the original source capsule envelope. Sleeping/contact-active flags cannot omit an inequality. Shielding follows the actual collision range instead of inferring it from the presence of another material. Complete ordered original-node bindings must cover all own Joint edges.

An interval can expose both tools, including near a catheter portal. The new `contactOwners.edges[].walls` form retains both physical surfaces, with separate normal forces, tractions, radii and spin mappings. Existing single `wall` declarations remain compatible. The normal collector still queries each owner's own geometry. Nodal pressure sites are unique per physical owner/node; capsule inequalities remain checked. A nodal radius discontinuity requires an explicit alternative surface discretization.

The source adapter defaults to explicit nodal endpoint pressure. Capsule pressure is available through `contactMode:'capsule'`. It reads `wallStaticFriction` and `wallKineticFriction`, not the guidewire's kinetic `wallFriction` alias. Equal static/kinetic coefficients form an isotropic Coulomb law per exposed tool. Unequal coefficients still reject explicitly until the two-branch law is implemented. They are never reinterpreted as tangent-axis coefficients.

`source:'original-field'` lets different samples use their original SDF or BVH operators. Every raw source, derivative and material witness is checked; this does not relax the existing per-sample history/provenance requirements. An analytic test field still requires its explicit plane.

World preparation privately binds complete numeric wall data, source field storage/version/query policy, body ranges, radii and friction to one prepared dt. Checks run before each attempt and before publication. A caller-provided field identity, empty ownership list or fabricated proof cannot authorize missing contacts. Accepted diagnostics expose capsule counts. `prepareStep` can pass genuine `worldWallSurfacePosePaths`; mutable path declarations are copied, while immutable prepared path handles retain their provenance. Caller wall laws cannot override the derived source law.

## Verified cases

- Actual 5/4-spaced bodies, differing source radii, partial shielding and both owners on a shared interval retain every original collision interval and its parent radius. Preparation issues no geometry queries.
- Both exposed tools pass complete World steps with their own coefficients and spins. Force balance, physical state and source-derived direct/World results agree. Second-dt budget failure and changed-radius retry preserve the accepted state and one pending dt; restoring the original radius gives the exact cold result.
- A source-derived proximal catheter feed with an explicit straight incoming-material pose passes two dt with loaded wall friction and a failed/retried second dt. The caller cannot change its already-prepared path declarations.
- The real `Aorta_plain.collision.bin` field, actual Float32 body positions/radii and the importer pass two World steps for a three-node catheter. Initial original capsule gap is `-0.01999905239790678` mm; final gaps are `-3.941291737419306e-15` and `-1.9524493133360465e-11` mm. The original physical certificates pass, with 7 directions/17 evaluations and then 6/21. This particular Aorta test has zero wall friction; it does not combine anatomy with the separate positive-feed test.
- Additional manager/whole-step tests retain two Fn/Ft owners on one chart edge, preserve distinct SDF/BVH surface laws from one field, and reject duplicate/missing/mutated source declarations.

Full composite suite: **768/768 PASS**, 16.391 s. Build PASS, 1.74 s. `source.json` records sources and logs. This is not a pass of the entire `npm test`; the previously documented unrelated old World test remains outside this suite.

## Remaining integration and performance work

The actual app still uses its earlier solver. Its unequal guidewire static/kinetic law, sheath/containment/portal/external-contact source mappings, internal-hinge surface transport, evolving material profiles and mesh/range transfer remain unfinished. A small real-anatomy normal-contact test is not full loaded-friction anatomy coverage. Fixed full relative coordinates still need error-controlled reduction. Deep/max insertion browser trials, 120 Hz without backlog and the mean ≤4 ms/P95 ≤6 ms budget remain unverified. The active goal is unchanged.

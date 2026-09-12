# Bounded zero-load kinematic stop certificate

This optional continuation changes only kirchhoffWallFrictionMode.js and its dedicated test relative to V2 ecf1cffa. World, SplitMotion, profiles, collision geometry and numerical gates are unchanged.

A zero-load contact can retain a stop certificate if its physical surface rate is exactly zero, or if both its tangent Jacobian functionals equal those of a CURRENT loaded stop witness on all free DOFs. Any unmatched prescribed DOF term must have exactly zero physical velocity. Equality is exact in coefficient space, not a comparison of observed small speeds. The source and target have the same body, normal, wall feature and plane; each keeps its own material identity. A dot-product roundoff bound derived from actual term magnitudes checks the evaluated rates. No normal/tangent multiplier is copied, and no physical velocity is changed.

The two tangential rows may be expressed in the source tangent basis. The proof records source key, numerical stop certificate, basis coefficients, zero free-J difference, zero prescribed-rate difference and per-axis roundoff bound. A merely small nonzero velocity without such support does not retain history.

22 new helper/wrench tests PASS, including an equivalent zero-load witness, a different mobile node at the same tiny speed (not certified), and truly free tiny motion (not certified).

EXPLICIT LIMIT: The unchanged root native 3-step static World witness still fails at step 3. In step 2, point1 Fn=0 now inherits a kinematic stop certificate from wall:0:1, with roundoff bound 8.688384636915769e-25 mm. Point2 Fn=0 has a distinct free translation DOF and remains unproved; its history is not retained. Step 3 contains four static and one sliding contact and is rejected as wall-friction-mode-ambiguous. Hard material kinematic relations would be needed to prove point2; no general row-space solver was added, and the fixture/assertions were not changed.

results/ contains exact isolated-overlay World witness and test logs plus source provenance. Root physics sources were stable when copied; only the mode helper differs in that overlay.

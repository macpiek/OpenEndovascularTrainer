# Continuous stent-graft Y surface — 2026-09-28

This supersedes the centroid/material-binding-only seam fix in `stent-graft-junction-2026-09-28.md`.

## Correction

Previously the three independently tessellated cloth meshes were still rendered separately. Limb mouth samples followed a trunk fan, including an axial overlap, but the polygon edges between samples did not coincide. This produced the visible ipsilateral slit and contralateral fabric overlap.

The new outlet is split into complementary polygons. Every boundary sample is inserted into the neighbouring strip and shares a single vertex ID. The trunk, both limbs and crotch form one indexed pair-of-pants surface; old cloth meshes remain solver data and are hidden. The mechanical mouth bindings use these exact boundary edges, without axial overlap. Topology is constructed once and its vertex positions are updated during release.

Partial collision faces come directly from this indexed mesh. The deployed body uses its sewn closed solid for volume queries and removes the actual cap triangles for tool contacts, preserving all three open ports. A connected extension is still combined with the main body by the existing solid union. Deliberately misplaced/degenerate outlet geometry retains finite material topology.

## Verification

- Final focused run: **35/35 passed** (topology, material attachment, metal length, interaction, anatomical Y seam).
- New topology checks: exactly two oppositely directed faces at every internal edge; only the three intended port boundaries; no duplicate indexed faces; Euler characteristic -1; unchanged connectivity through rotation/release/recapture.
- All four main-body models, right and left access; anatomical aneurysm tests from both sides.
- Rendered triangles and partial/deployed collision triangles agree. The deployed collision mesh has no artificial portal caps.
- Off-target main-body deployment and standalone limb deployment pass after fixing a degenerate-edge case found by the broader tests.
- Broader regression run: 66/68 passed, with two pre-existing saved-session failures (`Discovery replay requires the same vessel mesh`), reproduced in the unchanged main project.
- Broader legacy deployment tests have six pre-existing failures asserting exact equality to the unconstrained expansion target; the same six fail before this patch. They were not weakened or removed. The entire legacy suite is therefore not green.
- Production build passes (existing bundle-size warning only).
- Browser comparison uses the same three deployment fractions as the rejected screenshot: 55%, 70%, 100%. The slit and overlap are absent in this view. Screenshot: `screenshots/stent-graft-sewn-y-2026-09-28.png`.

This remains the existing reduced deployment model. Topological continuity is verified; this is not a claim of a complete finite-element cloth/contact model or validation of every possible self-contact configuration.

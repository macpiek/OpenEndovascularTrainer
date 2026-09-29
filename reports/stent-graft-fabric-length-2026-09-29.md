# Ipsilateral cloth length — 2026-09-29

The live scene was already reset when inspected (tools at zero); the previous exact pose was unavailable. The cause was reproduced independently during deployment and with an imposed long-leg axial deformation.

## Defects and correction
- Longitudinal cloth limits used the maximum of material row distance and already wall-fitted target vertex separation. A distorted target could therefore define extra material length.
- Release-center limits likewise accepted expanded/folded spatial distances instead of only the manufactured material coordinates.
- Assembly-level center constraints covered only the first six rows of the branches.

Rest fiber lengths now use immutable material coordinates and the nominal radius profile (including taper). Every section is constrained, with dyadic chord constraints to propagate tension along the entire leg in the existing 100-pass budget. The common sewn Y boundary is retained. Cloth may bend and fold; the rest dimensions do not change with fitted pose.

## Regression and validation
A 53 mm ipsilateral leg was stretched axially in a controlled test. The previous correction left 64.097 mm of centerline; the revised correction restored 52.814 mm. Test bounds: total length no more than nominal + 0.3 mm, local interval excess below 0.25 mm (the existing seam-test tolerance). These are numerical bounds, not a claim of an exact shell mechanics solver.

Two new tests fail on the prior code and pass on the correction: fitted-target rest-length invariance and recovery from axial stretching. Both-access release/rotation/recapture are covered as well.

55 tests passed: fabric metric, sewn assembly/anchors/topology, ring kinematics, expansion, contact interaction, sewn release and anatomical wall fitting. Production build passed with the pre-existing large-bundle warning. Browser preview checks 55%, 70% and full release. No claim is made that the separate previously observed suprarenal-crown exception is fixed.

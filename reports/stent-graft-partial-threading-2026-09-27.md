# Ipsilateral threading during partial deployment

The folded long limb now encloses the delivery axis; the short gate is packed beside it, with mirrored placement for the opposite access. Partial ipsilateral contact starts on actually exposed rows instead of waiting for full radial expansion.

Contact follows the displayed fabric geometry. After tip detachment, exposed fabric retains its release reference, while covered rows follow the moving shaft. Previously, moving the wire also moved already exposed fabric and destabilized its contact response.

The collision envelope distinguishes the full delivery cover from the exposed inner shaft (0.8 mm radius, matching its rendered tube). Within a transitioning ipsilateral branch, contact uses its actual triangles instead of also applying potentially conflicting mapped Boolean cut faces. Fully deployed cloth retains continuous crossing detection. Spatial bounds prevent the branch recovery force from acting on remote segments.

Unchanged geometry no longer invalidates collision snapshots just because a captured crown is evaluated again. Scaffold wires were moved 0.05 mm farther inside the fabric to retain separation with the corrected folded layout; wire thickness and metal arc length were not increased.

## Verification

- New right/left fixtures: 79.6 mm cover withdrawal, approximately 72% deployment; contact exists before the former threshold.
- Each access: 140 converged steps, including 8 mm delivery withdrawal and settling. The test checks reaction, ipsilateral lumen clearance away from the open portals, and geometry updates after every committed step.
- Geometry snapshots remain immutable; covered rows follow shaft motion, exposed rows do not follow the detached shaft.
- Selected suite: 87 tests covering deployment, mechanics, ring separation/length, threading, sliding, withdrawal, contact lifetime, fractional tips, vessel contact, sheath contact and friction. Two full-deployment CCD regressions found during development were corrected by restricting the partial-surface exception to transitioning branches. Final reruns of 23 threading/sliding tests and 7 withdrawal tests passed; the remaining selected tests passed in the broader run.
- Production build passed (existing bundle-size warning).
- Applied the 12 changed source/test/package files to the live workspace; browser reloaded with tools at zero and no console errors.

## Other test-suite limitations

The earlier full stentgraft run also encountered six archived replay cases whose saved vessel mesh differs from the current anatomy; their replay guard rejects reconstruction. Two anatomy deployment assertions (`every row must fully expand`) fail in both the unchanged live baseline and the staged version. These are outside this contact fix. This change does not claim that the entire historical suite is green or that the exact former live scene was replayed.

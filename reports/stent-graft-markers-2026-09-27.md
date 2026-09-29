# Stent wires and radiopaque markers — 2026-09-27

Implemented appearance changes, preserving mechanical/contact geometry:
- Scaffold visual radius 0.045 → 0.030; crown 0.055 → 0.035 world units.
- Wire opacity multiplier 0.55, applied in both debug and metal projection, combined with subpixel coverage. Ordinary alpha blending preserves continuous wires without alpha-to-coverage stippling.
- Debug fabric opacity 0.38 → 0.18. Marker material retains full contrast; connecting a limb no longer recolors its parent's gate marker as ordinary metal.

## Reference and marker layout

Compared Figure 1 (printed p. 4 / PDF p. 8) and sections 11.2.1, 11.2.10, 11.2.14 of the publicly available **Endurant II/IIs IFU, FDA P100021/S063 (2017)**:
https://www.accessdata.fda.gov/cdrh_docs/pdf10/P100021S063D.pdf

This is an identified older IFU, not a claim to have verified the latest regional revision.

- Proximal main body: three button markers plus the e-shaped orientation marker.
- Flow divider: one landmark, replacing the duplicated sets on the three procedural tube seams.
- Ipsilateral distal edge: one button for II; two for IIs.
- Contralateral gate: one asymmetric marker, replacing a continuous bright circumference and four extra buttons.
- Separate limb: two proximal and two distal buttons; one overlap marker about 25 mm distal to the proximal markers.

The same layout is used in the folded preview and deployed geometry. Markers follow fabric material coordinates. The e-marker is also present in the folded preview. Button dimensions and precise angular placement are rendering approximations: Figure 1 is explicitly not to scale.

## Validation

28/28 tests passed: stentGraftRotation, stentGraftNose, stentGraftDeployment. Includes shared packed/deployed marker layout and zero duplicate seam markers, continuous wire projection, bilateral deployment, capture/rotation and preview disposal. Existing visual radius/strip expectations updated to the requested thinner wires and compact buttons.

Production build passed (existing chunk-size warning). Browser checked four release stages of IIs 23/14 in debug and X-ray; final X-ray checked against main simulator server 5178. Screenshot: screenshots/stent-markers-2026-09-27.png.

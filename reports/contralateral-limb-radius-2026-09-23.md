# Contralateral limb insertion: delivery diameter correction

The captured left-access insertion (wire 387.2 mm, limb delivery 92.5 → 92.9167 mm) used a 3 mm radius / 18 Fr system for a short 16 mm limb. The original browser report recorded 839 Newton iterations, 162,819 factorizations and 121,998 ms solver CPU before recovery through subdivision. The following motion remained pending with the left physics rate at zero.

The Endurant II/IIs sizing sheet lists 14 Fr for short limbs with distal diameters 10–16 mm; longer limbs and 20–28 mm distal limbs use 16 Fr. Bodies up to 28 mm use 18 Fr; 32/36 mm bodies use 20 Fr. See [manufacturer sizing sheet, page 2](https://www.medtronic.com/content/dam/medtronic-wide/public/western-europe/products/cardiac-vascular/cardiovascular/aortic-stent-grafts/endurant-ii-sizing-sheet-print-en-gb.pdf). The simulator still has an approximate 80 mm limb length and optional 14 mm graft diameter, rather than a complete manufacturer catalogue.

The fix passes selected delivery radius to the catheter physics adapter, node radii, active sheath clearance and visual sheath marker. It preserves material stiffness, graft/vessel collisions, Newton tolerances and solver algorithms. Trial work-budget/subdivision changes were investigated but excluded: they shortened a failed solve without making continued insertion succeed.

## Replay validation

The exact archived pose and contact history were restored against the aneurysm collision asset, correcting the delivery radius to 14/6 mm. A fixed deployed graft surface remains present. Feeding at 25 mm/s for 120 steps of 1/60 s advances another 50 mm (92.5 → 142.5 mm):

- 120/120 steps converge on the first subdivision; no rejected steps.
- Median CPU wall duration in the Node replay: 46.88 ms; p95 130.00 ms; maximum 210.87 ms.
- Total 7.174 s, 598 Newton iterations and 2,288 factorizations.
- Maximum 99 factorizations per step, versus 162,819 in the captured original step.

These Node timings are not a browser Hz measurement and do not establish 60 Hz. The regression runs 24 successive steps, verifies penetration below 0.0001 mm and fewer than 256 factorizations per step. It retains all contacts. Additional tests verify selected-radius propagation and restoration of a normal catheter, catalogue size families, and the earlier graft withdrawal cases.

Validation: 14 targeted tests passed; Vite production build passed (existing bundle-size warning).

A second 120-step replay also updates the sheath clearance to radius + 0.2 mm, matching the application. All 120 steps converge; the regression includes this correction.

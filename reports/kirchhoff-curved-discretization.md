# Geometry screen for curved elements at 200 mm

This offline audit tests replacing two existing material edges with a cubic
Hermite position field. It does not alter the runtime mesh or approve mechanical
reduction. Input is the checked-in full 200 mm fixture; its raw SHA-256 and the
audit source hashes are in `kirchhoff-curved-discretization-audit.json`.

Endpoint unit directors are estimated by spherical interpolation/extrapolation
of the existing segment frames at their material centers. They would require a
different nodal representation in a real curved-element implementation. The
audit does not infer that existing angular DOFs can simply be deleted.

| Geometry screen, 0.001 mm | Guidewire, 201 nodes | Catheter, 72 nodes |
| --- | ---: | ---: |
| Straight replacement fits the removed vertex | 13 | 8 |
| Curved replacement fits the removed vertex | 121 | 25 |
| Curved replacement fits the whole fine polyline and speed error ≤0.002 | 22 | 14 |
| Nonoverlapping two-edge candidates meeting the preceding screen | 11 | 7 |

The large apparent gain at the vertices disappears when checking the whole
element. Median curved error against the fine polyline is approximately
0.02204 mm for the wire and 0.01826 mm for the catheter. At 0.00025 mm there
are only 6 and 4 nonoverlapping candidates. These counts are not approved node
removals: supports, material and contact boundaries, twist, forces, moment,
energy and eliminated-mode stability have not been certified.

The continuous geometric screen converts the difference between the cubic and
each fine edge to Bernstein form. The squared positional error has degree six;
the squared material speed has degree four. Subdivision bounds their ranges by
their coefficient convex hulls. Bounds apply continuously in exact arithmetic;
the implementation adds a conservative arithmetic noise guard but does not use
formal directed-rounding intervals. No physical certificate is asserted.

Three independent tests pass: a straight field with unequal material intervals,
a cubic with zero error at every old vertex but known interior error
1/(6 sqrt(3)), and a three-dimensional rigid transformation with a dense
cross-check. The analytic cubic also checks the known maximum stretch error.

Preserving the current discrete polyline is different from convergence toward
an unknown continuum solution. A curved continuum model might be more accurate
even when it differs from that polyline; establishing that requires a refined
mechanical reference and contact/work comparisons. This audit therefore rules
out a large, immediately justified coarsening under the current geometric
comparison. It does not rule out a future curved beam formulation.

Reproduce with:

```sh
node --test tests/curvedElementGeometry.test.js
node scripts/physics/audit-curved-discretization.mjs
```

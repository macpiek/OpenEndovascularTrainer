# Adjacent SurfaceMotion supports: continuity check

The frozen quadratic SurfaceMotion patches do **not** generally form a continuous centerline. Matching midpoint tangents and material frames does not fix this, and the material surface slip can also jump. This check concerns an **artificial support boundary inside the same edge**, with continuous material labels and no declared physical interface. It adds only a separate test file and this report; frozen SurfaceMotion mathematics/API and `contactCertified:false` are unchanged.

## Independent result

Let `Q_L` interpolate `(x0,q0),(x1,q1),(x2,q2)` and `Q_R` interpolate `(x1,q1),(x2,q2),(x3,q3)`. At their shared boundary `m=(x1+x2)/2`, independent Lagrange interpolation gives

```
Q_L(m) - Q_R(m)
  = q[x0,x1,x2,x3] * (x2-x1)^2 * (x3-x0) / 4
  = (-q0 + 3q1 - 3q2 + q3) / 8       // uniform grid
```

Here `q[...]` is the vector cubic divided difference. Both first derivatives equal `(q2-q1)/(x2-x1)`, so their normalized tangents agree. Both material-frame constructions reach the same accepted edge frame and the same shared edge angle; physical directors therefore agree. Their spatial derivatives generally do not.

| Quantity at the shared midpoint | Generic behavior |
| --- | --- |
| Position | Jump above; not C0 |
| `q_x` and normalized tangent | Matching one-sided values |
| Material directors/lever/axes | C0 when referring to the same shared edge data |
| `t_x`, `theta_x`, spatial frame connection | May jump; frame is generally not C1 |
| Time derivatives of tangent/frame | Matching shared-edge chart values |
| BE `q_t` | Jump equals `(positionJump_new-positionJump_old)/dt` |
| Full omega and surface slip | Generally discontinuous with material feed; BE translation can also cause a jump without feed |

For `x=[0,1,2,3]`, `q=(x,0.2x^3,0)`, the two positions at `x=1.5` differ by **0.15 mm**. With stationary geometry, `u=2 mm/s`, lever radius `0.2 mm` and `dt=1/120 s`, their wall-slip components along the common tangent differ by **−0.00135135135 mm**. Wire/catheter feeds of `2` and `−1 mm/s` give a relative-slip jump of **−0.00202702703 mm**.

The problem is not limited to geometric curvature. On a straight axis, edge angles `[0,0.2,0.8]` produce matching material directors at the interface but different `theta_x`; the same feed/radius produces a **0.00133333333 mm** slip jump. Those angle samples can come from the smooth quadratic field `theta(x)=0.2(x-0.5)^2` at edge centers. Conversely, a globally quadratic position field with constant material angle is a continuous control case.

This does **not** make global continuity of curvature or omega an unconditional physical requirement. Stiffness changes, an outlet or an applied point moment can produce valid one-sided curvature/twist-rate jumps. Those must be explicit physical boundaries, with traces and ownership preserved. A derivative/slip jump by itself is not evidence of an error at such a boundary. In these tests the boundary is an artificial midpoint seam within one edge and the cubic position example comes from a smooth curve. The position jump remains incompatible with the stated continuous common centerline.

## What can be bounded against the existing affine field

On either original edge `[a,b]`, let `A` be its affine interpolant, `h=b-a`, and `c=Q''` for the selected quadratic patch. Exactly,

```
Q(x)-A(x)   = c * (x-a)*(x-b)/2
|Q-A|       <= |c| h^2/8
|Q_x-A_x|   <= |c| h/2
|Q_t-A_t|   <= |c_dot_BE| h^2/8
|v_Q-v_A|   <= |c_dot_BE| h^2/8 + max|u| |c| h/2
```

The position envelope is attained at the midpoint, including on the half-edge used by a patch. These are algebraic bounds for the stated polynomial fields; machine-certified bounds additionally require outward rounding. They can support a geometric enclosure and a translational-error budget.

They **cannot certify the complete current slip map**. The affine DER tangent/director field has hinge jumps and no unique classical pointwise material angular derivative there. A small positional bound does not remove a discontinuous material path or define the missing angular transport. A position-only gate, or simply averaging the two slip values, is insufficient. Retaining the current quadratic patches in runtime would need explicit crossing/transport semantics and a full error contract, not just these bounds.

## Smallest practical continuous replacement to discuss

Within a region intended to have smooth material motion, continuous finite-radius slip with nonzero feed requires matching spatial derivatives of the unit tangent/material frame, alongside continuous translational velocity. C2 geometry with nonzero `q_x` is a simple sufficient condition for the tangent; it is not a necessary condition for every special parameterization or across a physical interface. Shared cubic Hermite positions/slopes repair position/tangent continuity but generally leave a curvature and advected-omega jump at an artificial seam.

A local interpolatory option inside such a region is **quintic Hermite per physical edge**, using each node's shared position, first derivative and second derivative. Compute derivative values once per node from a fixed neighboring three-node stencil; share them only where the boundary is artificial and smoothness is intended. At an explicit physical interface retain the required distinct one-sided second derivatives/frame rates rather than smoothing the jump. This keeps the original position nodes and introduces no independent position DOFs. Degree five is the minimum Hermite degree for independently prescribed value/first/second derivative at both endpoints. Each interior geometric edge then depends on four position nodes (12 translational DOFs).

Use a shared C1 interpolation of the unwrapped material angle in a continuous reference chart, with shared slopes at edge-angle samples. A cubic Hermite angle field can use four existing angle samples per tool without new scalar unknowns. Reference-chart conversion must remain differentiated. A straightforward conversion of four time-transported midpoint frames to one smooth chart can expand the union to seven position nodes plus four spins per tool: at most **29 local DOFs for two tools**, versus the current 13. This is a concrete stencil estimate, not a measured optimum or performance result.

The same geometry/velocity field must then drive inertia and contact evaluation. With fixed affine material maps, constant density and at-most-linear map rate, quintic shape-function velocity has degree at most five, so its squared translational inertial integrand needs Gauss-6 rather than Gauss-2 for exact polynomial integration. Accepted velocity history must retain that field rather than only endpoint values. A quintic Bézier deviation from the affine chord is bounded by the largest deviation of its Bernstein control points; this can retain an inflated capsule as an enclosure, but the current discrete capsule query alone does not certify the curved narrow phase.

No replacement or runtime gate is implemented here. The next decision is a piecewise common reconstruction with explicit physical interfaces and controlled geometry/velocity/transport error, or a complete approximation contract against the affine model. The proposed C2/C1 construction is an option within smooth regions, not a prescription to smooth all physical sections.

## Validation

**7/7 new tests; 18/18 with the unchanged SurfaceMotion tests.** Independent uniform/nonuniform interpolation identities, analytic bending omega/slip jumps, opposite tools, straight-axis twist discontinuity, zero-feed BE discontinuity, continuous quadratic control and the exact affine-deviation envelope all pass. A passing test here confirms the documented discontinuity; it does not certify integration.

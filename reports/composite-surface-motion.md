# Composite material surface motion

`kirchhoffCompositeSurfaceMotion.js` evaluates a local material surface slip and its complete first Jacobian on three shared position nodes and two independent edge angles per present tool. One tool contacts a stationary wall; two tools share the same reconstructed centerline foot. It changes no accepted inputs, frames, winding, labels or history. TimeStep, Element, Chain and Kinematics are unchanged.

## Declared reconstruction and equations

The caller must explicitly request `reconstruction: 'quadratic-hinge'`. On the interval between the two neighboring edge midpoints, quadratic Lagrange shape functions interpolate the three common position nodes:

```
q_h(x) = sum N_j(x) q_j
q_t    = sum N_j(x) (q_j - q_old,j) / dt
q_x    = sum N'_j(x) q_j
t      = q_x / |q_x|
u_i    = -s_i,t / s_i,x
v_i    = q_t + u_i q_x
```

The quadratic tangent equals the corresponding original edge tangent at each midpoint. Its interior position field is different from the existing affine-edge inertia field. `reconstruction.equalsAffineEdgeField` and `contactCertified` are explicitly false. Pointwise position and per-tool centerline-velocity differences from the affine field are reported. These measurements are not an interval error bound or an acceptance gate. Runtime integration still needs a common reconstruction or an explicit admissible-error certificate.

Each accepted reference frame is transported in time from its previous edge tangent to the current tangent. The left current director is then parallel-transported onto `t(x)` and rotated by `w*phi_i`, where `w` runs from zero to one between edge midpoints and `phi_i` is the unwrapped reference connection to the right frame. Each tool's angle interpolates independently between its two edge values. This defines a continuous local material frame, including winding; it does not assign zero spatial derivatives to the piecewise-constant DER orientation field.

With reference directors `d1,d2`, the angular velocity is

```
A_t     = d2 . (sum_j partial_qj(d1) * (q_j-q_old,j)/dt)
A_x     = d2 . partial_x(d1)
spin_i  = (theta_i-theta_old,i)/dt + u_i theta_i,x + A_t + u_i A_x
omega_i = t cross (t_t + u_i t_x) + spin_i t
```

`t_t` differentiates the same time-transport chart along the nodal BE rates. Old angles are supplied at the same fixed mesh coordinates; they must not already include material advection. Accepted reference tangents must match `previousPositions`. Gauge changes around those accepted tangents are compensated in both old/current angles and the winding anchors. Arbitrary replacement with a different previous geometry is not an equivalent BE chart and is rejected.

For lever arms `r_i` and the supplied two orthonormal tangent axes:

```
v_surface,i = v_i + omega_i cross r_i
slip        = dt * axes . (v_surface,1 - v_surface,2)
```

The stationary wall has zero velocity. Each lever explicitly uses a frozen world vector or components in `(material d1, material d2, tangent)`. Axes likewise use either frozen world vectors or a named tool's material frame. Derivatives of moving material levers and axes are included. Contact geometry and traction-history basis transport remain caller responsibilities.

## API and derivatives

Exports are `createCompositeSurfaceMotionWorkspace(toolCount)` and `evaluateCompositeSurfaceMotion(input, workspace?)`. Inputs are three current/previous positions, three coordinates, a query coordinate in the hinge interval, two accepted reference frames, positive `dt`, explicit reconstruction/axes and one or two named tools. Each tool supplies current/previous edge angles, an unwrapped reference-twist anchor, a finite material map `{sStart,dsDx,dsDt}` with `sStart` at the first node, and an explicit lever frame/vector. Missing angular history, map rates or frame conventions fail; no unknown rate defaults to zero.

The local columns are `q0.xyz, q1.xyz, q2.xyz`, then left/right angles for each tool in input order. Output `jacobian` has two rows and `9+2T` columns in row-major order. Output also exposes the relative velocity/Jacobian, full omega/Jacobian, separate time/spatial reference connections, material directors, lever arms and material labels.

Forward second derivatives provide the mixed derivatives needed when differentiating `A_t` and `A_x`; the final slip derivative is first order. All frame, neighboring-position and independent-spin contributions remain present. Runtime evaluation uses no finite differences. Workspaces reuse fixed derivative arenas; returned arrays are owned and survive reuse. No performance/FPS or integration claim is made.

## Validation

11/11 new tests and 36/36 combined with the frozen Element and Kinematics suites pass. Tests cover non-unit `q_x`, opposite feed/spin with convection, material movement along a curved stationary axis with bending omega, unwrapped reference winding, a common finite rigid motion with zero relative slip at the same surface foot, accepted time-frame gauge invariance including the Jacobian, world-frame covariance, midpoint compatibility, stationary wall and input/output ownership.

Independent finite differences verify every position and both-tool spin column of slip and omega with world/material axes and finite moving lever arms. A separate material-path finite difference of the entire director triad reconstructs full omega, independently checking the time/spatial frame connections and bending terms. Invalid or stale accepted frames and missing angular history reject explicitly.

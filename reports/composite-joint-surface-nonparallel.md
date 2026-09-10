# Finite nonparallel affine surface motion — bounded handoff

`kirchhoffCompositeJointSurfaceMotion.js` now also exports
`createCompositeJointSurfaceIncrementWorkspace(T)` and
`evaluateCompositeJointSurfaceIncrement(input, workspace)`. The new operator
evaluates a defined finite slip and its complete first-order AD Jacobian on a
regular chart of two independently moving **actual affine** physical edges.
The old instantaneous operator and parallel finite control are unchanged
byte-for-byte. No Step, Assembly, contact detector, normal row, geometry,
material history, mass, solver tolerance or runtime setting changed.

This provides a local finite Newton operator (`nonlinearReady: true`), not an
accepted time step or integrated Coulomb friction. The two tools' force map is
still the transpose of the **separate instantaneous rate Jacobian**. The finite
configuration Jacobian must not silently replace that map. There is no FPS or
full-runtime performance claim.

## Input and derivative contract

The existing own-edge tool input supplies its current and previous physical
endpoints, own accepted reference frame, own current/previous **unwrapped**
theta, own affine material maps, current contact coordinate and explicit
same-edge material path. Neither instantaneous endpoint rates nor angular
rates are needed by the new finite evaluator. They remain required by the
separate instantaneous evaluator; unknown rates are never set to zero.

The additional query input is:

```js
rotationPath: 'short-contact-frame-own-unwrapped-spins',
finiteGeometry: {
  kind: 'explicit-affine-side-queries',
  current:  { point, normal, tangent },
  previous: { point, normal, tangent }
}
```

These are two objective queries of the same declared geometric contact chart.
The caller owns its physical contact identity, support/provenance, branch
validity and geometry chain rule. Raw normals and tangents are normalized and
orthogonalized **inside AD**. They are not falsely treated as constants.

For two tools the Jacobian has 34 columns, in this order:

1. Fourteen current configuration entries: per tool `[q0.xyz,q1.xyz,theta]`.
2. Two current foot coordinates, in their own coordinate units.
3. Nine current query entries: `[point.xyz,normal.xyz,tangent.xyz]`.
4. Nine previous query entries in the same order.

`configurationJacobian` is 2×14; `queryJacobian` is 2×20; `jacobian` is 2×34.
The full three-component contact-frame increment also has its 3×34 Jacobian.
All return buffers are owned across workspace reuse. A query argument varying
with trial configuration or the selected label must be chained by the caller.
The actual strict-side projection regression includes the moving outer foot,
contact point, radial normal and outer tangent in this chain rule.

Current material labels select the old centers through each tool's own old
map. The derivative of the resulting old foot is included. Maps and accepted
physical frames remain frozen preparation inputs. Different label metrics,
independent feed and affine changes of `dsDx` are retained. Material leaving
its own represented edge, or missing one-sided endpoint traces, rejects with
the existing explicit transport-required error. No adjacent-hinge angular
history or artificial smooth centerline is manufactured.

For one tool there are 26 columns. The bounded wall variant requires an
explicit stationary wall and **identical wall material point and frame** in
both queries. A moving wall needs its own material history; a query velocity
does not supply that history. The tool itself may bend and be nonparallel to
the wall tangent.

## Defined finite rule

Build the objective contact triad `C=(n,t×n,t)` independently at both endpoints
of the step. The contact tangent transport and residual normal turn select
the explicit short branch. Antiparallel tangent charts and the ambiguous
normal/reference angle at π reject. The input does not encode a long common
contact-frame orbit; this operator does not infer one from two endpoints.

For each tool, express its current and old material centers relative to the
respective query origin in C, giving `c1,c0`; express its tangents as `t1,t0`.
The current own reference is the same world time-PT of the accepted own
director used by the elastic geometry. In C, transport the old own reference
by the shortest `t0→t1` swing. Let `γ` be its remaining signed angle to the
current own reference, and retain

`α = theta1 − theta0 + γ`.

Only the geometric phase γ uses the short angular branch. **Both tools' full
theta increments** remain unwrapped. The difference of their windings is not
a substitute: both tools rotating by +2π about distinct fixed axes still
produce nonzero surface slip when their levers differ.

Let `b` be the shortest tangent rotation vector and define the explicit path

`t(τ)=exp(τ[b]×) t0`, `ω(τ)=b+α t(τ)`, `0≤τ≤1`.

This corresponds to the own axial spin progressing during the short swing,
with orientation `exp(τ[b]×) Rot(t0,ατ)` applied to the old triad. Its angular
integral is `b+α tMean`. For `c=t0·t1` and
`f=acos(c)/sqrt(1−c²)`, `tMean=(t0+t1)/(f(1+c))` and `b=f(t0×t1)`.
The removable singularity at `c=1` uses the analytic series through degree 6
for `|1−c|<10⁻³`; omitted value terms are below 2×10⁻²³. This is a stable,
roundoff-accurate evaluation of the same function, differentiated by AD.
No runtime finite differences or principal logarithm of material orientation
are used. An independent Simpson integral of the Rodrigues tangent path
checks this mean, including the series join and parallel limit.

The stated symmetric discrete lever rule is

`rMean=−(c0+c1)/2`,
`Δsurface=(c1−c0)+(b+α tMean)×rMean`.

Take inner minus outer and project to contact axial/circumferential entries
`[z,y]`. This **approximates the coupled angular-velocity/lever integral by
the angular integral crossed with an endpoint-average lever**. It is not
claimed to integrate an arbitrary unknown finite material trajectory exactly.
Its explicit symmetric form reverses sign when the material states, query
states and own lifted angles are consistently swapped. Physical radii enter
through the caller's actual contact point and the distinct physical feet;
there is no second, potentially inconsistent, radius-based lever replacement.

Under common finite rigid motion, material centers, tangents and physical
orientations are unchanged in C; each `α` is zero, including the compensation
between own time-PT angle and geometric phase. Under an infinitesimal motion,
`cDot=Cᵀ(vMaterial−vPoint)−ΩC×c` and the relative angular rate is
`ωMaterial−ΩC`. With `r=−c`, the contact-frame angular terms cancel. The
difference of the tools also cancels `vPoint`, recovering exactly the separate
instantaneous surface velocity and its virtual-power limit. Spatial rotation
inside each own open affine DER edge is constant; feed across a real hinge
still requires a separate explicit transport history.

## Verification and measured numerical witnesses

Commands:

```sh
node --test tests/kirchhoffCompositeJointSurfaceMotion.test.js
node --test tests/kirchhoffCompositeJointSurfaceMotion.test.js tests/kirchhoffCompositeSurfaceMotion.test.js tests/kirchhoffCompositeSurfaceContinuity.test.js
node --check src/physics/kirchhoffCompositeJointSurfaceMotion.js
```

**22/22 owned tests and 40/40 affected tests pass.** Nine new tests cover all
34 AD columns, the independently differentiated actual strict-side query,
nonparallel common finite rigid motion with distinct levers, reference gauges,
reversal with +2π/−4π, both own full turns, departure from parallelism, the
tangent-path quadrature/series join, changing material metrics and opposite
feeds, zero-feed bending, one tool against a wall, explicit branch/transport
rejection and owned outputs/source histories. The earlier 13 tests remain.

Raw deterministic results are in
`composite-joint-surface-nonparallel-probe.json`:

| Witness | Result |
| --- | --- |
| All 34 finite-slip AD columns vs central differences, h=10⁻⁶ | max error 3.01323×10⁻⁹ |
| Nonparallel common finite R=0.9 rad | max slip 5.06839×10⁻¹⁵ |
| Forward/reverse with own +2π and −4π | max mismatch 5.48173×10⁻¹⁵ |
| Both own +2π; radii 0.25/0.5 in parallel control | circumferential increment −π/2, preserved |
| Virtual-power-limit max rate error at dt 10⁻²/10⁻³/10⁻⁴/10⁻⁵ | 1.74781×10⁻³ / 1.74284×10⁻⁴ / 1.74234×10⁻⁵ / 1.74228×10⁻⁶ |
| Corresponding power error | 7.89917×10⁻⁴ / 7.86900×10⁻⁵ / 7.86589×10⁻⁶ / 7.86913×10⁻⁷ |

The O(dt) rate-limit error is expected for the declared finite discrete rule.
No nonlinear acceptance gate was changed. The new common-R test uses a
3×10⁻¹⁴ arithmetic comparison, accounting for material-label subtraction near
100; it does not modify any existing test or physical tolerance.

The immutable bundle is `/tmp/oet-composite-joint-surface-nonparallel-final`,
including source, all three test files/dependencies, deterministic probe,
reports and a SHA-256 manifest. The previous parallel-only frozen bundle at
`/tmp/oet-composite-joint-surface-motion-final` remains unchanged.

Next integration requires the caller to provide current/old contact chart
history and geometry derivatives, then use finite slip for the friction row
and the separate physical rate map for forces. This patch does not supply a
hinge-crossing surface path, moving-wall history, general contact rechart,
friction state transition or accepted full-dt friction solve.

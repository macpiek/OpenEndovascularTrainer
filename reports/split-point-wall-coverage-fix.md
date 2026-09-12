# Vessel endpoint coverage and the deep-insertion checkpoint

The split endpoint supplement incorrectly generated a vessel contact when a
body's active vessel-capsule interval was empty. On the recorded anatomy case,
catheter `activeEnd=17`, `collisionStartSegment=17`, `collisionEndSegment=16`
and `sheathMaterialEndNode=17`. The old inclusive node loop still visited node
17 and constrained material owned by the introducer against the vessel field.

The supplement now intersects the active and collision **segment** intervals
first. An empty interval generates no endpoint rows; a nonempty interval retains
both endpoints of eligible capsules. Geometry, radii and tolerances are unchanged.

The independent [anatomy audit](split-anatomy-wall-audit.md) identifies the
exact point and query paths. It disproved the initial hypothesis that this
particular witness came from a false negative historical tangent-plane gap:
the point was already outside the vessel field's domain of relevance. Its
physical normal multiplier stayed zero. Its bogus geometric constraint caused
the subsequent bias phase to attempt a 6.114-mm repair.

Three regressions cover an empty interval in native World, both endpoints of
one genuine active capsule, and disjoint active/collision intervals sharing a
boundary node. The three independent wall force/moment oracles also pass.

## Anatomy after the fix

[`split-anatomy-coverage-fixed.json`](split-anatomy-coverage-fixed.json) is a
stable-source replay at 120 Hz after preparing 999.9 mm of guidewire. Prepared
pose fingerprints match the [earlier checkpoint](split-anatomy-first-feed-checkpoint.json).
The first eligible coupled step, at a prepared catheter command of 5.2 mm,
still rejects and rolls back:

- Physical phase converges after 30 passes; physical KKT residual is
  .000145218 mm and adaptation .000028843 mm.
- Maximum relevant raw penetration is now .016324780 mm, rather than 6.114 mm.
- Preserve-strain bias fails with `non-finite-direction`, seven near-null
  pivots, 1280 equality rows and 34 retained rows.
- The separate unsupported unequal static/kinetic wall-friction guard remains.
- No coupled physical timestep or history is committed for this attempt.

This is not a successful deep-insertion or FPS result. The next questions are
whether hard strain preservation leaves enough mechanical freedom for the
remaining correction, and how to implement both specified wall friction
coefficients with consistent stick/slide history.

## Synthetic trajectory

[`split-post-coverage-consecutive.json`](split-post-coverage-consecutive.json)
records seven accepted coupled steps after preparing 12.1 mm of wire. The next
command, to 7.3667 mm of catheter, fails preserve-strain bias with .002370199-mm
geometric violation. Physical KKT/material gates pass and there are no
unsupported-history guards. The smaller 144-row system supplies an additional
witness for bias-feasibility analysis.

## Snapshot optimization and validation

The separately integrated [whole-step scratch exclusion](split-step-scratch.md)
preserves mechanical owners and rebuilds disposable outputs after rollback.
The intra-trial frozen-batch mechanism now also includes split-wall solve rows,
residual and merit caches. A version counter rejects rebuilding those rows during
a frozen trial. Two native wall-batch tests verify retained physical ownership,
scaled retry after fresh measurement, and the rebuild guard.

The complete coupled suite has **405 tests: 404 pass and one previously recorded
position-history translated-mouth lifecycle test fails**. The optional stricter
analytical-accuracy diagnostic retains its separately documented baseline
limitations; no production accuracy gate was relaxed.

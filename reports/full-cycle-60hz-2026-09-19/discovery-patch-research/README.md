# Exact local BVH patches — no full-cycle speedup, removed

The experiment retained all BVH leaves intersecting a ball around a previous near-wall query. A candidate nearest point was reusable only when its distance was strictly below the conservative distance to every omitted subtree after subtracting the complete displacement. Sign/inside certificates and spatial-axis checks remained mandatory. Cache capacity was bounded and transient patch data was not serialized into physical replay state.

The first flat-leaf version changed tie-breaking between equally close triangles: a 10000-point anatomy probe found 35 different face indices with exactly identical distances. Those changes altered the solver path. The full 5757-step cycle averaged 37.684 ms, with an 18514.478 ms worst step and trajectory differences up to 18.42 mm. It was rejected.

The corrected version compressed unary paths but retained original child bounds and near/far ordering at every retained branch. The same probe then had zero face or distance differences. All 5757 movement steps completed with the reference Newton/factorization counts and byte-identical `shapes.json`, SHA-256 `f16483a72982ec1f17990aeff05357fbf5c487c81959d1f34e58be30dca6b5bb`.

Despite 9963012 cache hits out of 10239635 certified samples (97.3%), the corrected full cycle averaged 34.425 ms versus the 33.915 ms reference. P95 was 97.688 ms, maximum 1603.059 ms, and 4024 steps exceeded 16.67 ms. Retained typed patch memory was 2532768 bytes. The candidate did not demonstrate a speedup and was removed from the runtime, rather than enabled by its high hit rate.

Five focused geometry/replay tests passed before removal, including exact nearest-face ordering, conservative failure after movement or geometry replacement, capacity overflow, preservation of required inside proof, and non-serialization of transient caches. Both full experiments and their source changes are archived in the subdirectories. This work does not meet the 60 Hz goal.

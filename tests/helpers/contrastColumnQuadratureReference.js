// Frozen pre-optimization quadrature for GPU parity checks.
export const contrastColumnQuadratureReference = /* glsl */`
float junctionColumnAverage(int a, float sa, int b, float sb, vec3 entry, vec3 ray, float chord) {
    float componentA = texelFetch1D(columnRanges, uint(a * 2 + 1)).z;
    float componentB = texelFetch1D(columnRanges, uint(b * 2 + 1)).z;
    // Keep the original wall coordinates for the analytic baseline. Local
    // spatial projection has a separate, canonical, per-territory seed.
    float mixedBaseline = -1., sum = 0.;
    vec3 previous = junctionColumnValue(columnRestPosition(entry), a, sa, b, sb, componentA, componentB);
    vec4 pendingSamples[5]; vec3 pendingStates[5]; vec2 pendingSupports[5];
    for (int sampleIndex = 1; sampleIndex <= 24; sampleIndex++) {
        float lo = float(sampleIndex - 1) / 24., hi = float(sampleIndex) / 24.;
        vec3 current = junctionColumnValue(columnRestPosition(entry + ray * (chord * hi)), a, sa, b, sb, componentA, componentB);
        pendingSamples[0] = vec4(lo, hi, previous.x, current.x);
        pendingStates[0] = vec3(previous.z, current.z, 0.); pendingSupports[0] = vec2(previous.y, current.y);
        int top = 0;
        while (top >= 0) {
            vec4 interval = pendingSamples[top]; vec3 state = pendingStates[top]; vec2 support = pendingSupports[top]; top--;
            if (state.z >= 4. || state.x == state.y || interval.z == interval.w) {
                float beta = .5 * (support.x + support.y), base = 0.;
                if (beta < 1.) {
                    if (a == b) base = columnAverage(a, mix(sa, sb, interval.x), a, mix(sa, sb, interval.y));
                    else {
                        if (mixedBaseline < 0.) mixedBaseline = spatialColumnAverage(a, sa, b, sb, entry, ray, chord);
                        base = mixedBaseline;
                    }
                }
                // This is a convex combination of an exact prefix integral
                // and a positive spatial integral. It cannot subtract iodine
                // below zero when a narrow front falls between sample points.
                sum += (interval.y - interval.x) * ((1. - beta) * base + .5 * (support.x * interval.z + support.y * interval.w));
                continue;
            }
            float middle = .5 * (interval.x + interval.y);
            vec3 value = junctionColumnValue(columnRestPosition(entry + ray * (chord * middle)), a, sa, b, sb, componentA, componentB);
            top++; pendingSamples[top] = vec4(middle, interval.y, value.x, interval.w);
            pendingStates[top] = vec3(value.z, state.y, state.z + 1.); pendingSupports[top] = vec2(value.y, support.y);
            top++; pendingSamples[top] = vec4(interval.x, middle, interval.z, value.x);
            pendingStates[top] = vec3(state.x, value.z, state.z + 1.); pendingSupports[top] = vec2(support.x, value.y);
        }
        previous = current;
    }
    return sum;
}
`;

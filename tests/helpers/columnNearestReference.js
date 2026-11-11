// Frozen projection before the last-segment atlas cache. Keep the full iteration
// sequence for GPU A/B and Float32 parity checks, including corner oscillation.
export const columnNearestReference = /* glsl */`
float columnNearestRefined(int path, vec3 point, float seed, int iterations, out float score) {
    vec4 range = texelFetch1D(columnGeometryRanges, uint(path));
    float s = clamp(seed, 0., range.z), bestS = s, bestDistance = 1e30, bestRadius = 0.;
    vec4 derivative;
    for (int i = 0; i <= 6; i++) {
        vec4 local = columnGeometryAt(range, s, derivative);
        float next = s + dot(point - local.xyz, derivative.xyz) / max(1e-8, dot(derivative.xyz, derivative.xyz));
        float segment = min(range.y - 2., floor(s / max(range.w, 1e-8)));
        float candidate = clamp(next, segment * range.w, (segment + 1.) * range.w);
        vec4 nearest = local + derivative * (candidate - s);
        float distance = dot(point - nearest.xyz, point - nearest.xyz);
        if (distance < bestDistance || (distance == bestDistance && candidate < bestS)) {
            bestDistance = distance; bestS = candidate; bestRadius = nearest.w;
        }
        float bounded = clamp(next, 0., range.z);
        if (i >= iterations || bounded == s) break;
        s = bounded;
    }
    score = sqrt(max(0., bestDistance)) - bestRadius;
    return bestS;
}
`;

// Frozen finite-history lookup before exact displayAlpha endpoint fast paths.
export const columnSampleHistoryReference = /* glsl */`
vec4 columnSample(int path, float s) {
    vec4 range = texelFetch1D(columnRanges, uint(path * 2));
    int first = int(range.x), last = first + int(range.y) - 1;
    s = clamp(s, 0., range.z);
    vec4 lookup = texelFetch1D(columnRanges, uint(columnLookupOffset + path));
    int bucket = clamp(int(s / max(lookup.z, 1e-8)), 0, int(lookup.y) - 1);
    vec4 bracket = texelFetch1D(columnRanges, uint(columnLookupOffset + int(lookup.x) + bucket));
    int lo = int(bracket.x), hi = int(bracket.y);
    for (int i = 0; i < 24; i++) {
        if (hi - lo <= 1) break;
        int mid = (lo + hi) / 2;
        float x = texelFetch1D(columnCurrent, uint(mid)).x;
        if (x <= s) lo = mid; else hi = mid;
    }
    vec4 a = mix(texelFetch1D(columnPrevious, uint(lo)), texelFetch1D(columnCurrent, uint(lo)), displayAlpha);
    vec4 b = mix(texelFetch1D(columnPrevious, uint(hi)), texelFetch1D(columnCurrent, uint(hi)), displayAlpha);
    float dx = max(0., s - a.x), span = b.x - a.x;
    float c = span > 1e-8 ? mix(a.y, b.y, clamp(dx / span, 0., 1.)) : b.y;
    return vec4(max(0., c), a.z + dx * (a.y + c) * .5, s, float(lo));
}
`;
export const columnPathConstantHistoryReference = /* glsl */`
bool columnPathConstant(int path, out float value) {
    if (displayAlpha < 0. || displayAlpha > 1.) return false;
    int first = int(texelFetch1D(columnRanges, uint(path * 2)).x);
    float currentMin = texelFetch1D(columnCurrent, uint(first)).w;
    float currentMax = texelFetch1D(columnCurrent, uint(first + 1)).w;
    float previousMin = texelFetch1D(columnPrevious, uint(first)).w;
    float previousMax = texelFetch1D(columnPrevious, uint(first + 1)).w;
    if ((displayAlpha > 0. && currentMin != currentMax) || (displayAlpha < 1. && previousMin != previousMax)) return false;
    value = mix(previousMin, currentMin, displayAlpha);
    return true;
}
`;

export function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

export function smoothstep(edge0, edge1, value) {
    const t = clamp((value - edge0) / Math.max(1e-6, edge1 - edge0), 0, 1);
    return t * t * (3 - 2 * t);
}

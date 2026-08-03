export function dsaArchiveDimensions(width, height) {
    return {
        width: Math.max(1, Math.round(width)),
        height: Math.max(1, Math.round(height))
    };
}

export function dsaScoreDimensions(width, height, maxDimension = 256) {
    const normalizedWidth = Math.max(1, Math.round(width));
    const normalizedHeight = Math.max(1, Math.round(height));
    const limit = Math.max(1, Math.round(maxDimension));
    const scale = Math.min(
        1,
        limit / Math.max(normalizedWidth, normalizedHeight)
    );
    return {
        width: Math.max(1, Math.round(normalizedWidth * scale)),
        height: Math.max(1, Math.round(normalizedHeight * scale))
    };
}

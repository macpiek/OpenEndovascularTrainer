// Reorder, never remove, the existing eight dyadic candidate scales.
// Re-probe larger than the last successful correction. If every
// smaller candidate fails, still try every skipped larger candidate.
export function lineSearchLevel(trial, startLevel) {
    const smallerCount = 8 - startLevel;
    return trial < smallerCount ? startLevel + trial : trial - smallerCount;
}

export function createLineSearchStats() {
    return { accepted: Array(8).fill(0), rejected: Array(8).fill(0),
        rejectionTerms: {}, boundaryKinds: {}, boundaryWorstChanged: 0, predictedStarts: 0,
        largerFallbacks: 0, earlyRejections: 0 };
}

export function recordLineSearchTrial(stats, level, accepted, state, baseBoundary, boundary) {
    (accepted ? stats.accepted : stats.rejected)[level]++;
    if (accepted) return;
    if (state.earlyRejected) {
        // Only a sufficient boundary error was measured; do not present it as
        // the dominant term of a full residual calculation.
        stats.earlyRejections++;
        return;
    }
    let dominant = 'nonfinite', maximum = -Infinity;
    for (const key in state.meritTerms) {
        const value = state.meritTerms[key];
        if (!Number.isFinite(value)) { dominant = key; break; }
        if (value > maximum) { maximum = value; dominant = key; }
    }
    stats.rejectionTerms[dominant] = (stats.rejectionTerms[dominant] ?? 0) + 1;
    if (dominant === 'boundary') {
        const kind = boundary?.kind ?? 'unknown';
        stats.boundaryKinds[kind] = (stats.boundaryKinds[kind] ?? 0) + 1;
    }
    // This is a change of the WORST witness, not proof of contact instability.
    if (baseBoundary && boundary && ['kind', 'side', 'node', 'wallT', 'wallBranchId', 'wallFaceIndex']
        .some(key => baseBoundary[key] !== boundary[key])) stats.boundaryWorstChanged++;
}

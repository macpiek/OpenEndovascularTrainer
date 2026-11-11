/** Predict presentation cost without turning a one-time upload/compile into
 * seconds of minimum-size physics slices. Leave room for one normal slice;
 * the idle scheduler still measures the real remaining frame time. */
export function estimateRenderReserve(previousMs, measuredMs, {
    frameMs = 1000 / 60,
    guardMs = .75,
    physicsSliceMs = 4
} = {}) {
    const maximum = Math.max(0, frameMs - guardMs - physicsSliceMs);
    const minimum = Math.min(1.5, maximum);
    const previous = Number.isFinite(previousMs) ? Math.min(maximum, Math.max(minimum, previousMs)) : minimum;
    const measured = Number.isFinite(measuredMs) ? Math.max(0, measuredMs) : minimum;
    // Follow sustained costs immediately; release an isolated peak within a
    // few frames instead of retaining an unbounded compilation duration.
    return Math.min(maximum, Math.max(minimum, measured, previous * .85));
}

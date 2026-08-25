export const DEFAULT_FLUORO_PULSE_WIDTH_MS = 8;
export const REFERENCE_OUTPUT_MGY_PER_MAS_AT_80_KV = 0.55;

function finiteNonNegative(value, fallback = 0) {
    const numericValue = Number(value);
    return Number.isFinite(numericValue)
        ? Math.max(0, numericValue)
        : fallback;
}

export function estimateFluoroDoseRateMgyPerSecond({
    kv,
    ma,
    pulseRate,
    pulseWidthMs = DEFAULT_FLUORO_PULSE_WIDTH_MS,
    emitting = true
} = {}) {
    if (!emitting) return 0;

    const safeKv = Math.min(130, Math.max(40, finiteNonNegative(kv, 80)));
    const safeMa = finiteNonNegative(ma);
    const safePulseRate = finiteNonNegative(pulseRate);
    const safePulseWidthSeconds = Math.min(
        0.02,
        finiteNonNegative(pulseWidthMs, DEFAULT_FLUORO_PULSE_WIDTH_MS) / 1000
    );
    const tubeOutputMgyPerMas = REFERENCE_OUTPUT_MGY_PER_MAS_AT_80_KV *
        Math.pow(safeKv / 80, 2);
    const masPerSecond = safeMa * safePulseWidthSeconds * safePulseRate;
    return tubeOutputMgyPerMas * masPerSecond;
}

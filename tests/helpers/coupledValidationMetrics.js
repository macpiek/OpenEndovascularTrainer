// Preserve the existing browser acceptance limits; missing telemetry cannot pass.
export const COUPLED_VALIDATION_LIMITS = Object.freeze({
    meanStepMs: 4, p95StepMs: 6, wallPenetrationMm: 0.2,
    relativeLengthError: 0.01, lengthClosureRelative: 0.002,
    materialStrainRad: 0.005, isolatedLengthErrorMm: 0.001,
    reciprocalTranslationMm: 1e-5, freeSlideErrorMm: 0.003,
    noLoadQuaternionDotError: 1e-7
});

export function timingSummary(samples) {
    if (!samples.length) return { count: 0, meanMs: null, p95Ms: null, p99Ms: null, maximumMs: null };
    if (samples.some(value => !Number.isFinite(value) || value < 0)) throw new TypeError('timings must be finite and non-negative');
    const sorted = [...samples].sort((a, b) => a - b);
    const percentile = p => sorted[Math.floor((sorted.length - 1) * p)];
    return { count: samples.length, meanMs: samples.reduce((a, b) => a + b, 0) / samples.length,
        p95Ms: percentile(0.95), p99Ms: percentile(0.99), maximumMs: sorted.at(-1) };
}

/** Full-step CPU data may reject a budget, but cannot certify browser FPS.
 * Browser evidence must account for idle and render-frame physics steps alike.
 * A one-tick allowance handles the residual fractional scheduler accumulator.
 */
export function assessCoupledTiming({ expectedSteps, executedSteps, fullStepTimes = [],
    worldStepTimes = [], browser = null, fixedDt = 1 / 120 }) {
    const full = timingSummary(fullStepTimes), world = timingSummary(worldStepTimes);
    const complete = Number.isInteger(expectedSteps) && expectedSteps > 0 && executedSteps === expectedSteps;
    const allTimings = full.count === executedSteps && world.count === executedSteps;
    const budgetPass = stats => stats.count > 0 && stats.meanMs <= 4 && stats.p95Ms <= 6;
    let schedulerPass = null, renderingPass = null;
    if (browser) {
        const b = browser;
        const values = [b.acceptedSeconds, b.startBacklogSeconds, b.endBacklogSeconds,
            b.peakBacklogSeconds, b.executedSteps, b.renderFrameExecutedSteps, b.idleExecutedSteps, b.droppedSteps];
        const available = values.every(Number.isFinite) && values.every(value => value >= 0);
        const accountingError = available ? b.acceptedSeconds + b.startBacklogSeconds -
            (b.executedSteps * fixedDt + b.endBacklogSeconds) : Infinity;
        schedulerPass = available && complete && b.executedSteps === executedSteps &&
            b.renderFrameExecutedSteps + b.idleExecutedSteps === executedSteps &&
            b.droppedSteps === 0 && Math.abs(accountingError) <= 1e-6 &&
            b.endBacklogSeconds <= b.startBacklogSeconds + fixedDt + 1e-6 &&
            b.peakBacklogSeconds <= b.startBacklogSeconds + 2 * fixedDt + 1e-6;
        // Existing one-percent-low threshold is 55 FPS. Report P95/P99 frames,
        // require the same P99 boundary; a mean near 60 alone cannot pass.
        renderingPass = [b.averageFps, b.p95FrameMs, b.p99FrameMs].every(Number.isFinite) &&
            b.averageFps >= 59 && b.p95FrameMs <= 1000 / 55 && b.p99FrameMs <= 1000 / 55;
    }
    return { complete, allTimings, fullStep: full, worldStep: world,
        physicsBudgetPass: complete && allTimings && budgetPass(world),
        fullStepBudgetPass: complete && allTimings && budgetPass(full),
        schedulerPass, renderingPass,
        realTime60FpsPass: complete && allTimings && budgetPass(world) && budgetPass(full) &&
            schedulerPass === true && renderingPass === true };
}

export function maximumRelativeLengthError(body) {
    let maximum = 0;
    for (let i = body.activeStart; i < body.activeEnd; i++) {
        const length = Math.hypot(body.x[i + 1] - body.x[i], body.y[i + 1] - body.y[i], body.z[i + 1] - body.z[i]);
        if (!Number.isFinite(length)) return Infinity;
        maximum = Math.max(maximum, Math.abs(length - body.restLength[i]) / body.restLength[i]);
    }
    return maximum;
}

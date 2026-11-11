// Rendering and cooperative solver timers share one wall clock. Sampling from
// either source admits each interval once, even when GPU backpressure delays
// animation frames. Loading, pause and hidden-tab intervals never become debt.
export function createSimulationWallClock() {
    let previous = null, wasEnabled = false;
    return {
        sample(now, enabled) {
            if (!Number.isFinite(now)) throw new RangeError('A finite wall time is required');
            const elapsed = enabled && wasEnabled && previous !== null ? Math.max(0, now - previous) / 1000 : 0;
            previous = now; wasEnabled = enabled;
            return elapsed;
        },
        reset() {previous = null; wasEnabled = false;}
    };
}

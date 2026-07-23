export const BROWSER_BENCHMARK_DEFAULT_DURATION_MS = 10 * 60 * 1000;
export const BROWSER_BENCHMARK_SCENARIO_CYCLE_MS = 72 * 1000;

export function createBrowserBenchmarkCommands() {
    return {
        guidewireAdvance: 0,
        catheterAdvance: 0,
        catheterRotation: 0,
        catheterType: 'pigtail'
    };
}

export function browserBenchmarkCatheterType(elapsedMs) {
    const elapsed = Math.max(0, elapsedMs);
    return Math.floor(elapsed / BROWSER_BENCHMARK_SCENARIO_CYCLE_MS) % 2 === 0
        ? 'pigtail'
        : 'berenstein';
}

export function sampleBrowserBenchmarkCommands(elapsedMs, out) {
    const elapsed = Math.max(0, elapsedMs);
    const cyclePhase = elapsed % BROWSER_BENCHMARK_SCENARIO_CYCLE_MS;
    out.guidewireAdvance = 0;
    out.catheterAdvance = 0;
    out.catheterRotation = 0;
    out.catheterType = browserBenchmarkCatheterType(elapsed);

    if (cyclePhase < 15000) {
        out.guidewireAdvance = 1;
    } else if (cyclePhase < 25000) {
        out.catheterAdvance = 1;
    } else if (cyclePhase < 35000) {
        out.catheterRotation = Math.floor((cyclePhase - 25000) / 2500) % 2 === 0 ? 1 : -1;
    } else if (cyclePhase < 52000) {
        out.catheterAdvance = -1;
        out.catheterRotation = Math.floor((cyclePhase - 35000) / 2500) % 2 === 0 ? -1 : 1;
    } else if (cyclePhase < 67000) {
        out.guidewireAdvance = -1;
    }
    return out;
}

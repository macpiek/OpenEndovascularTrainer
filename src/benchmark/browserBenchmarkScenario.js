export const BROWSER_BENCHMARK_DEFAULT_DURATION_MS = 10 * 60 * 1000;
export const BROWSER_BENCHMARK_SCENARIO_CYCLE_MS = 72 * 1000;
export const GUIDEWIRE_BROWSER_BENCHMARK_CYCLE_MS = 28 * 1000;
export const BROWSER_BENCHMARK_MODE_COUPLED = 'coupled';
export const BROWSER_BENCHMARK_MODE_GUIDEWIRE = 'guidewire-only';

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

export function sampleGuidewireBrowserBenchmarkCommands(elapsedMs, out) {
    const cyclePhase = Math.max(0, elapsedMs) %
        GUIDEWIRE_BROWSER_BENCHMARK_CYCLE_MS;
    // Exercise both button-release boundaries in every smoke cycle. The idle
    // windows are intentionally long enough to expose elastic recovery while
    // the transport command is zero, without changing the 28 s runtime.
    out.guidewireAdvance = cyclePhase < 12000
        ? 1
        : cyclePhase < 14000
            ? 0
            : cyclePhase < 26000
                ? -1
                : 0;
    out.catheterAdvance = 0;
    out.catheterRotation = 0;
    out.catheterType = 'berenstein';
    return out;
}

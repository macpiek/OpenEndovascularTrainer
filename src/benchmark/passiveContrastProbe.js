// Passive observation of the application's committed clock. This module never
// advances physics, injects iodine, changes controls, or resets solver state.
const COSTS = ['simulationMs', 'updateMs', 'renderMs', 'rendererUpdateMs', 'contrastTransportMs'];
const bucket = () => ({wallMs: 0, acceptedSteps: 0, completedSeconds: 0, frames: 0,
    acquisitions: 0, startBacklogSeconds: null, endBacklogSeconds: null,
    backlogGrowthSeconds: 0, peakBacklogSeconds: 0, frameMs: [], costs: Object.fromEntries(COSTS.map(key => [key, []]))});
const summary = values => {
    if (!values.length) return {count: 0, meanMs: 0, medianMs: 0, p95Ms: 0, maxMs: 0, totalMs: 0};
    const sorted = [...values].sort((a, b) => a - b), totalMs = values.reduce((a, b) => a + b, 0);
    return {count: values.length, meanMs: totalMs / values.length,
        medianMs: sorted[Math.floor((sorted.length - 1) * .5)],
        p95Ms: sorted[Math.floor((sorted.length - 1) * .95)], maxMs: sorted.at(-1), totalMs};
};
function reportBucket(value) {
    const seconds = value.wallMs / 1000;
    return {wallSeconds: seconds, acceptedSteps: value.acceptedSteps,
        acceptedPhysicsHz: seconds > 0 ? value.acceptedSteps / seconds : 0,
        completedSimulationSeconds: value.completedSeconds,
        simulationToWallRatio: seconds > 0 ? value.completedSeconds / seconds : 0,
        frames: value.frames, framesPerSecond: seconds > 0 ? value.frames / seconds : 0,
        acquisitions: value.acquisitions, acquisitionsPerSecond: seconds > 0 ? value.acquisitions / seconds : 0,
        startBacklogSeconds: value.startBacklogSeconds, endBacklogSeconds: value.endBacklogSeconds,
        backlogGrowthSeconds: value.backlogGrowthSeconds,
        peakBacklogSeconds: value.peakBacklogSeconds, frameIntervals: summary(value.frameMs),
        costs: Object.fromEntries(COSTS.map(key => [key, summary(value.costs[key])]))};
}
const validState = state => Number.isInteger(state.acceptedSteps) && state.acceptedSteps >= 0 &&
    Number.isFinite(state.completedSimulationTime) && state.completedSimulationTime >= 0 &&
    Number.isFinite(state.accumulator) && state.accumulator >= -1e-9;

export function createPassiveContrastProbe({readState, now = () => performance.now(), durationMs = 20000, onUpdate = () => {}}) {
    let active = false, started = 0, lastTime = 0, lastNotice = -Infinity, initial, previous, windows, result = null;
    const notify = (force = false) => {
        if (!force && lastTime - lastNotice < 500) return;
        lastNotice = lastTime;
        onUpdate({running: active, elapsedSeconds: Math.max(0, lastTime - started) / 1000,
            remainingSeconds: Math.max(0, durationMs - (lastTime - started)) / 1000,
            frames: windows?.all.frames ?? 0, report: result});
    };
    const finish = (reason = 'complete', valid = true) => {
        if (!active) return result;
        active = false;
        result = {kind: 'passive-contrast-performance', valid, reason, requestedDurationSeconds: durationMs / 1000,
            wallSeconds: (lastTime - started) / 1000, initial, final: previous,
            failedSteps: Math.max(0, (previous.failedSteps ?? 0) - (initial.failedSteps ?? 0)),
            recoveries: Math.max(0, (previous.recoveries ?? 0) - (initial.recoveries ?? 0)),
            windows: Object.fromEntries(Object.entries(windows).map(([key, value]) => [key, reportBucket(value)])),
            limitations: ['Intervals are attributed to the contrast/paused state supplied for that rendered frame.',
                'CPU render submission excludes asynchronous GPU completion; acquisitions and committed physics are separate counters.',
                'This passive timing capture does not certify mechanical accuracy or alter the simulation.']};
        notify(true); return result;
    };
    return {
        get running() { return active; },
        get report() { return result; },
        start() {
            if (active) return false;
            initial = {...readState()};
            if (!validState(initial)) throw new Error('Passive probe requires committed steps, simulation time and backlog.');
            previous = initial; started = lastTime = now(); lastNotice = -Infinity;
            windows = {all: bucket(), contrastActive: bucket(), contrastInactive: bucket(), paused: bucket()};
            result = null; active = true; notify(true); return true;
        },
        recordFrame(frame = {}) {
            if (!active) return;
            const current = {...readState(), ...frame}, time = now(), elapsed = time - lastTime;
            if (!validState(current) || !Number.isFinite(elapsed) || elapsed < 0 ||
                current.acceptedSteps < previous.acceptedSteps || current.completedSimulationTime < previous.completedSimulationTime - 1e-9) {
                finish('counter-reset-or-invalid-state', false); return;
            }
            const key = current.paused ? 'paused' : current.contrastActive ? 'contrastActive' : 'contrastInactive';
            for (const value of [windows.all, windows[key]]) {
                value.wallMs += elapsed; value.acceptedSteps += current.acceptedSteps - previous.acceptedSteps;
                value.completedSeconds += current.completedSimulationTime - previous.completedSimulationTime;
                value.startBacklogSeconds ??= previous.accumulator;
                value.endBacklogSeconds = current.accumulator;
                value.backlogGrowthSeconds += current.accumulator - previous.accumulator;
                value.peakBacklogSeconds = Math.max(value.peakBacklogSeconds, previous.accumulator, current.accumulator);
                value.frames++; value.frameMs.push(elapsed);
                value.acquisitions += Math.max(0, Number(current.acquisitions ?? current.acquires) || 0);
                for (const cost of COSTS) if (Number.isFinite(current[cost]) && current[cost] >= 0) value.costs[cost].push(current[cost]);
            }
            previous = current; lastTime = time;
            if (lastTime - started >= durationMs) finish(); else notify();
        },
        finish: reason => finish(reason ?? 'stopped'),
        dispose() { if (active) finish('disposed'); }
    };
}

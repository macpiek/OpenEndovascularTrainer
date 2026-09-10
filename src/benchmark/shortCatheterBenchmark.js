export const SHORT_CATHETER_BENCHMARK_MODE = 'short-catheter';
export const DEEP_CATHETER_BENCHMARK_MODE = 'deep-catheter';
export const SHORT_CATHETER_BENCHMARK_DT = 1 / 120;
const wireStep = 44 * SHORT_CATHETER_BENCHMARK_DT;
const catheterStep = 52 * SHORT_CATHETER_BENCHMARK_DT;

// Integer physical steps, including fractional final feed commands. A slower
// machine must execute the same trajectory and held time, not stop earlier.
export const SHORT_CATHETER_BENCHMARK_PHASES = [
    { name: 'wire-feed', steps: Math.ceil(999.9 / wireStep), wireMm: 999.9 },
    { name: 'wire-settle', steps: 360 },
    { name: 'wire-only', steps: 600 },
    { name: 'catheter-feed-10mm', steps: Math.ceil(10 / catheterStep), catheterMm: 10 },
    { name: 'catheter-hold-10mm', steps: 600 },
    { name: 'catheter-feed-20mm', steps: Math.ceil(10 / catheterStep), catheterMm: 10 },
    { name: 'catheter-hold-20mm', steps: 600 },
    { name: 'catheter-feed-50mm', steps: Math.ceil(30 / catheterStep), catheterMm: 30 },
    { name: 'catheter-hold-50mm', steps: 600 }
];
export const SHORT_CATHETER_BENCHMARK_STEPS = SHORT_CATHETER_BENCHMARK_PHASES.reduce((n, p) => n + p.steps, 0);
export const SHORT_CATHETER_BENCHMARK_DURATION_MS = SHORT_CATHETER_BENCHMARK_STEPS * SHORT_CATHETER_BENCHMARK_DT * 1000;

// Deep feed uses the same full-wire preparation and measures each depth for
// five physical seconds, even when the renderer outpaces the physics scheduler.
export const DEEP_CATHETER_BENCHMARK_PHASES = SHORT_CATHETER_BENCHMARK_PHASES.slice(0, 3);
let previousDepthMm = 0;
for (const depthMm of [100, 200, 400, 600]) {
    const catheterMm = depthMm - previousDepthMm;
    DEEP_CATHETER_BENCHMARK_PHASES.push(
        { name: `catheter-feed-${depthMm}mm`, steps: Math.ceil(catheterMm / catheterStep), catheterMm },
        { name: `catheter-hold-${depthMm}mm`, steps: 600 }
    );
    previousDepthMm = depthMm;
}
DEEP_CATHETER_BENCHMARK_PHASES.push(
    { name: 'catheter-rotate-600mm', steps: 240, rotation: 1 },
    { name: 'catheter-counterrotate-600mm', steps: 240, rotation: -1 },
    { name: 'catheter-settle-600mm', steps: 600 }
);
export const DEEP_CATHETER_BENCHMARK_STEPS = DEEP_CATHETER_BENCHMARK_PHASES.reduce((n, p) => n + p.steps, 0);
export const DEEP_CATHETER_BENCHMARK_DURATION_MS = DEEP_CATHETER_BENCHMARK_STEPS * SHORT_CATHETER_BENCHMARK_DT * 1000;

export function poseFingerprint(body) {
    let hash = 2166136261;
    for (const values of [body.x, body.y, body.z, body.velocityX, body.velocityY, body.velocityZ,
        body.orientationX, body.orientationY, body.orientationZ, body.orientationW]) {
        for (const byte of new Uint8Array(values.buffer, values.byteOffset, values.byteLength)) {
            hash = Math.imul(hash ^ byte, 16777619);
        }
    }
    return (hash >>> 0).toString(16).padStart(8, '0');
}

export function sampleShortCatheterBenchmarkCommands(elapsedMs, out, phases = SHORT_CATHETER_BENCHMARK_PHASES) {
    let step = Math.floor(elapsedMs / (SHORT_CATHETER_BENCHMARK_DT * 1000) + 1e-6);
    out.guidewireAdvance = 0;
    out.catheterAdvance = 0;
    out.catheterRotation = 0;
    out.guidewireRotation = 0;
    out.catheterType = 'berenstein';
    out.benchmarkPhase = -1;
    for (let i = 0; i < phases.length; i++) {
        const phase = phases[i];
        if (step >= phase.steps) { step -= phase.steps; continue; }
        out.benchmarkPhase = i;
        out.catheterRotation = phase.rotation ?? 0;
        out.guidewireRotation = phase.wireRotation ?? 0;
        if (phase.wireMm) out.guidewireAdvance = Math.sign(phase.wireMm) *
            Math.max(0, Math.min(1, (Math.abs(phase.wireMm) - step * wireStep) / wireStep));
        if (phase.catheterMm) {
            // PigtailCatheter.advance uses 52 mm/s forward and 32 mm/s back.
            const increment = (phase.catheterMm > 0 ? 52 : 32) * SHORT_CATHETER_BENCHMARK_DT;
            out.catheterAdvance = Math.sign(phase.catheterMm) *
                Math.max(0, Math.min(1, (Math.abs(phase.catheterMm) - step * increment) / increment));
        }
        return out;
    }
    return out;
}

/** Read-only instrumentation shared by the browser and fixed-step replay. */
export class ShortCatheterBenchmarkMetrics {
    constructor(definitions = SHORT_CATHETER_BENCHMARK_PHASES) {
        this.definitions = definitions;
        this.phases = [];
    }

    recordFrame(frameMs) {
        const phase = this.phases.at(-1);
        if (!phase || frameMs <= 0) return;
        phase.frames++;
        phase.frameMs += frameMs;
    }

    recordStep(index, world, wireMm, catheterMm, backlogSeconds = 0) {
        if (index < 0) return;
        const phase = this.phases[index] ??= {
            name: this.definitions[index].name,
            steps: 0, frames: 0, frameMs: 0, stepMs: 0, closureMs: 0,
            outerPasses: 0, contactPasses: 0, unconvergedSteps: 0,
            jointSteps: 0, partitionedSteps: 0, jointFactorizations: 0,
            jointLinearIterations: 0, jointMaximumBand: 0, jointMaximumRows: 0,
            maxRelativeLengthError: 0, maxWallPenetrationMm: 0,
            maxLumenPenetrationMm: 0, maxSolverResidualMm: 0,
            stepTimes: [], startBacklogSeconds: backlogSeconds,
            peakBacklogSeconds: backlogSeconds
        };
        phase.steps++;
        phase.wireMm = wireMm;
        phase.catheterMm = catheterMm;
        phase.stepMs += world.timings.total.last;
        phase.closureMs += world.timings.constraintCoupledClosure.last;
        phase.outerPasses += world.lastCoupledClosurePasses;
        phase.contactPasses += world.lastCoupledContactPasses ?? 0;
        phase.unconvergedSteps += world.lastCoupledClosureConverged ? 0 : 1;
        phase.jointSteps += world.lastCoupledSolver === 'joint' ? 1 : 0;
        phase.partitionedSteps += world.lastCoupledSolver === 'partitioned' ? 1 : 0;
        phase.jointFactorizations += world.lastJointFactorizations ?? 0;
        phase.jointLinearIterations += world.lastJointLinearIterations ?? 0;
        phase.jointMaximumBand = Math.max(phase.jointMaximumBand, world.lastJointMaximumBand ?? 0);
        phase.jointMaximumRows = Math.max(phase.jointMaximumRows, world.lastJointMaximumRows ?? 0);
        phase.stepTimes.push(world.timings.total.last);
        phase.endBacklogSeconds = backlogSeconds;
        phase.peakBacklogSeconds = Math.max(phase.peakBacklogSeconds, backlogSeconds);
        phase.maxWallPenetrationMm = Math.max(phase.maxWallPenetrationMm, world.settledMaxPenetration);
        for (const contact of world.containments) {
            if (!contact.enabled) continue;
            phase.maxLumenPenetrationMm = Math.max(phase.maxLumenPenetrationMm, contact.kirchhoffMaxViolation);
            phase.maxSolverResidualMm = Math.max(phase.maxSolverResidualMm, contact.kirchhoffSolverResidual ?? 0);
        }
        for (const body of world.bodies) {
            for (let i = body.activeStart; i < body.activeEnd; i++) {
                const length = Math.hypot(body.x[i + 1] - body.x[i], body.y[i + 1] - body.y[i], body.z[i + 1] - body.z[i]);
                phase.maxRelativeLengthError = Math.max(phase.maxRelativeLengthError, Math.abs(length - body.restLength[i]) / body.restLength[i]);
            }
        }
        if (phase.steps === this.definitions[index].steps) {
            phase.endPoseFingerprints = world.bodies.map(body => ({ id: body.id, hash: poseFingerprint(body) }));
        }
    }

    report() {
        return this.phases.map(({ stepTimes, ...p }) => {
            const sorted = stepTimes.toSorted((a, b) => a - b);
            return {
                ...p, simulatedSeconds: p.steps * SHORT_CATHETER_BENCHMARK_DT,
                averageFps: p.frameMs ? p.frames * 1000 / p.frameMs : null,
                averageStepMs: p.stepMs / p.steps,
                p95StepMs: sorted[Math.floor((sorted.length - 1) * 0.95)],
                averageClosureMs: p.closureMs / p.steps,
                averageOuterPasses: p.outerPasses / p.steps,
                averageContactPasses: p.contactPasses / p.steps,
                averageJointFactorizations: p.jointSteps ? p.jointFactorizations / p.jointSteps : null,
                averageJointLinearIterations: p.jointSteps ? p.jointLinearIterations / p.jointSteps : null
            };
        });
    }
}

/** Full physical-step protocol. Existing short/deep protocols stay unchanged. */
export function createCoupledRebuildPhases({ maximumCatheterMm = 1000 } = {}) {
    if (!Number.isFinite(maximumCatheterMm) || maximumCatheterMm < 600 || maximumCatheterMm > 1000) {
        throw new RangeError('maximumCatheterMm must be between 600 and the runtime limit of 1000 mm');
    }
    const phases = DEEP_CATHETER_BENCHMARK_PHASES.map(phase => ({ ...phase }));
    phases.push(
        { name: 'wire-rotate-at-600mm', steps: 240, wireRotation: 1 },
        { name: 'wire-counterrotate-at-600mm', steps: 240, wireRotation: -1 }
    );
    let depth = 600;
    function move(target, holdSteps = 600, suffix = '') {
        const catheterMm = target - depth;
        if (!catheterMm) return;
        phases.push({
            name: `catheter-${catheterMm > 0 ? 'feed' : 'withdraw'}-${target}mm${suffix}`,
            steps: Math.ceil(Math.abs(catheterMm) / ((catheterMm > 0 ? 52 : 32) * SHORT_CATHETER_BENCHMARK_DT)),
            catheterMm, targetCatheterMm: target
        }, { name: `catheter-hold-${target}mm-after-reversal${suffix}`, steps: holdSteps, targetCatheterMm: target });
        depth = target;
    }
    move(400); // Reverse the moving portal before advancing to maximum insertion.
    move(maximumCatheterMm);
    phases.push(
        { name: 'catheter-rotate-maximum', steps: 240, rotation: 1 },
        { name: 'catheter-counterrotate-maximum', steps: 240, rotation: -1 },
        { name: 'catheter-settle-maximum', steps: 600 }
    );
    for (const target of [600, 400, 200, 100, 0]) move(target, 600, '-final');
    phases.push(
        { name: 'wire-withdraw-zero', steps: Math.ceil(999.9 / wireStep), wireMm: -999.9 },
        { name: 'all-withdrawn-hold', steps: 600 }
    );
    return phases;
}

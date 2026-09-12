import fs from 'node:fs';
import path from 'node:path';
import { createHash } from 'node:crypto';
import { gzipSync } from 'node:zlib';
import { createCoupledRuntimeFixture, loadCoupledRuntimeAnatomy } from '../../tests/helpers/coupledRuntimeFixture.js';
import { EndovascularPhysicsWorld } from '../../src/physics/endovascularPhysicsWorld.js';
import { configureKirchhoffToolRuntime } from '../../src/physics/kirchhoffToolRuntime.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../../src/physics/kirchhoffCoupledSystem.js';

// Acceptance replay, intentionally separate from application selection. The
// two trajectories evolve independently under identical actuator commands.
const output = path.resolve(process.argv[2] ?? 'reports/contact-runtime-verification-2026-09-12');
const fullBandDuringWire = process.env.OET_FULL_BAND_DURING_WIRE === '1';
const bandFromMm = Number(process.env.OET_BAND_FROM_MM ?? .5);
const stopOnParity = process.env.OET_STOP_ON_PARITY === '1';
if (!(Number.isFinite(bandFromMm) && bandFromMm >= 0)) throw new RangeError('Nonnegative band activation depth required');
fs.mkdirSync(output, { recursive: true });
const encode = (_key, value) => ArrayBuffer.isView(value) ? Array.from(value) :
    typeof value === 'number' && !Number.isFinite(value) ? String(value) : value;
const write = (name, value) => fs.writeFileSync(path.join(output, name), JSON.stringify(value, encode, 2) + '\n');
const sourceFiles = [...fs.readdirSync('src/physics').filter(n => n.endsWith('.js')).map(n => `src/physics/${n}`),
    'src/pigtailCatheter.js', 'tests/helpers/coupledRuntimeFixture.js', 'scripts/physics/verify-contact-structure-runtime.mjs'];
const hashes = () => Object.fromEntries(sourceFiles.map(file => [file, createHash('sha256').update(fs.readFileSync(file)).digest('hex')]));
const summarize = values => {
    const sorted = values.slice().sort((a, b) => a - b);
    return { count: values.length, mean: values.reduce((a, b) => a + b, 0) / (values.length || 1),
        p50: sorted[Math.floor(sorted.length / 2)] ?? 0, p95: sorted[Math.max(0, Math.ceil(sorted.length * .95) - 1)] ?? 0,
        max: sorted.at(-1) ?? 0 };
};
const fields = ['x', 'y', 'z', 'orientationX', 'orientationY', 'orientationZ', 'orientationW',
    'velocityX', 'velocityY', 'velocityZ', 'angularVelocityX', 'angularVelocityY', 'angularVelocityZ'];
function pose(fixture) {
    return fixture.world.bodies.map(body => ({ id: body.id, activeStart: body.activeStart, activeEnd: body.activeEnd,
        ...Object.fromEntries(fields.map(key => [key, Array.from(body[key])])) }));
}
function comparePoses(fixtures) {
    let maximumPositionMm = 0, maximumAngleRad = 0, rangesEqual = true, finite = true;
    for (let side = 0; side < 2; side++) {
        const [a, b] = fixtures.map(f => f.world.bodies[side]);
        rangesEqual &&= a.activeStart === b.activeStart && a.activeEnd === b.activeEnd;
        for (const body of [a, b]) for (const field of fields) for (let i = body.activeStart; i < Math.min(body.activeEnd + 1, body[field].length); i++)
            finite &&= Number.isFinite(body[field][i]);
        for (let i = Math.max(a.activeStart, b.activeStart); i <= Math.min(a.activeEnd, b.activeEnd); i++) {
            maximumPositionMm = Math.max(maximumPositionMm, Math.hypot(a.x[i] - b.x[i], a.y[i] - b.y[i], a.z[i] - b.z[i]));
            // Material frames/angular velocities live on segments, positions on nodes.
            if (i >= Math.min(a.activeEnd, b.activeEnd)) continue;
            let dot = 0, aa = 0, bb = 0;
            for (const key of fields.slice(3, 7)) { dot += a[key][i] * b[key][i]; aa += a[key][i] ** 2; bb += b[key][i] ** 2; }
            maximumAngleRad = Math.max(maximumAngleRad, 2 * Math.acos(Math.min(1, Math.abs(dot) / Math.sqrt(aa * bb))));
        }
    }
    return { maximumPositionMm, maximumAngleRad, rangesEqual, finite };
}
const phases = [{ name: 'wire-600', tool: 'guidewire', target: 600, rate: 44 }];
for (const target of [100, 200, 400]) {
    phases.push({ name: `feed-${target}`, tool: 'catheter', target, rate: 52 },
        { name: `hold-${target}`, steps: 30 },
        { name: `withdraw-${target}`, tool: 'catheter', target: target - (target === 400 ? 20 : 5), rate: 32 });
}
const report = { scope: 'Paired Node runtime replay, no browser rendering or FPS measurement', startedAt: new Date().toISOString(),
    sourceBefore: hashes(), node: process.version, platform: process.platform, arch: process.arch,
    fullBandDuringWire, bandFromMm, stopOnParity,
    policies: { independentComponents: true, physicalTrialState: true, earlyTrialRejection: true, toolRuntime: 'shared', fixedDt: 1 / 120 },
    protocol: phases, complete: false, accepted: false, failure: null, phases: [], checkpoints: [] };
const began = performance.now();
let lastProgress = began, stepIndex = 0, activePhase, fixtures = [];
// Geometry is immutable; each fixture receives its own field query caches.
const anatomies = [await loadCoupledRuntimeAnatomy(), await loadCoupledRuntimeAnatomy()];
class World extends EndovascularPhysicsWorld {
    stepFixed() { for (const body of this.bodies) configureKirchhoffToolRuntime(body); return super.stepFixed(); }
}
const variants = ['condensed', 'full-band'];
try {
    fixtures = variants.map((variant, side) => {
        const fixture = createCoupledRuntimeFixture({ ...anatomies[side], World, coupledSystem: {
            ...report.policies,
            solve(constraint, dt, options) {
                const result = solveKirchhoffCoupledSystem(constraint, dt, { ...options,
                    activeCondensation: true, simultaneousCoulomb: true,
                    ...(variant === 'full-band' && (fullBandDuringWire || fixture.catheter.progress > bandFromMm)
                        ? { coulombStructure: 'full-band' } : {}) });
                if (activePhase) {
                    const counters = activePhase.variants[side];
                    counters.linearCalls++; counters.linearFailures += Number(!result.diagnostics.converged);
                    counters.bandResults += Number(result.diagnostics.coulombStructure === 'full-band');
                    counters.maximumLinearResidual = Math.max(counters.maximumLinearResidual, result.diagnostics.maximumResidual ?? 0);
                }
                return result;
            }, apply: applyKirchhoffCoupledCorrection
        } });
        fixture.world.captureCoupledClosureTrace = true;
        return fixture;
    });
    report.config = Object.fromEntries(Object.entries(fixtures[0].config).filter(([_k, v]) => ['number', 'boolean', 'string'].includes(typeof v)));
    report.tolerances = Object.fromEntries(['coupledContainmentTolerance', 'coupledLengthTolerance', 'coupledAngularToleranceRad']
        .map(key => [key, fixtures[0].world[key]]));
    outer: for (const phase of phases) {
        activePhase = { name: phase.name, steps: 0, maximumPositionDifferenceMm: 0, maximumAngleDifferenceRad: 0,
            variants: variants.map(variant => ({ variant, times: [], physicsTimes: [], linearCalls: 0, linearFailures: 0, bandResults: 0,
                maximumLinearResidual: 0, nonlinearFailures: 0, maximumPenetrationMm: 0, maximumLengthRelativeError: 0,
                sums: {}, failures: [] })) };
        report.phases.push(activePhase);
        const start = phase.tool === 'guidewire' ? fixtures[0].transport.progress : fixtures[0].catheter.progress;
        const distance = phase.tool ? phase.target - start : 0;
        const steps = phase.tool ? Math.ceil(Math.abs(distance) / (phase.rate / 120)) : phase.steps;
        activePhase.plannedSteps = steps;
        for (let i = 0; i < steps; i++) {
            const commands = phase.tool ? { [phase.tool === 'guidewire' ? 'guidewireAdvance' : 'catheterAdvance']:
                Math.sign(distance) * Math.min(1, Math.max(0, (Math.abs(distance) - i * phase.rate / 120) / (phase.rate / 120))) } : {};
            const results = [];
            // Alternation removes a fixed first/second variant ordering bias.
            for (const side of stepIndex % 2 ? [1, 0] : [0, 1]) {
                const fixture = fixtures[side], world = fixture.world, counters = activePhase.variants[side];
                const t = performance.now(); results[side] = fixture.step(commands); counters.times.push(performance.now() - t);
                counters.physicsTimes.push(world.timings.total.last);
                counters.maximumPenetrationMm = Math.max(counters.maximumPenetrationMm, world.settledMaxPenetration ?? 0);
                counters.nonlinearFailures += Number(Boolean(world.lastJointNonlinearFailure));
                if (world.lastJointNonlinearFailure) counters.failures.push({ stepIndex, catheterMm: fixture.catheter.progress, ...world.lastJointNonlinearFailure });
                for (const [key, value] of Object.entries(world.lastJointCosts)) counters.sums[key] = (counters.sums[key] ?? 0) + value;
                for (const key of ['lastCoupledClosurePasses', 'lastJointTrialEvaluations', 'lastJointBacktracks', 'lastJointFactorizations'])
                    counters.sums[key] = (counters.sums[key] ?? 0) + (world[key] ?? 0);
                for (const body of world.bodies) for (let node = body.activeStart; node < body.activeEnd; node++)
                    counters.maximumLengthRelativeError = Math.max(counters.maximumLengthRelativeError,
                        Math.abs(Math.hypot(body.x[node + 1] - body.x[node], body.y[node + 1] - body.y[node], body.z[node + 1] - body.z[node]) - body.restLength[node]) / body.restLength[node]);
            }
            const parity = comparePoses(fixtures);
            activePhase.maximumPositionDifferenceMm = Math.max(activePhase.maximumPositionDifferenceMm, parity.maximumPositionMm);
            activePhase.maximumAngleDifferenceRad = Math.max(activePhase.maximumAngleDifferenceRad, parity.maximumAngleRad);
            activePhase.steps++; stepIndex++;
            const regression = fixtures[1].world.lastJointNonlinearFailure && !fixtures[0].world.lastJointNonlinearFailure;
            const parityFailure = stopOnParity && (parity.maximumPositionMm > report.tolerances.coupledContainmentTolerance ||
                parity.maximumAngleRad > report.tolerances.coupledAngularToleranceRad);
            if (!parity.finite || !parity.rangesEqual || results.some(result => result.accepted === false) || regression || parityFailure) {
                report.failure = { reason: !parity.finite ? 'nonfinite-pose' : !parity.rangesEqual ? 'active-range-mismatch' : regression ? 'additional-full-band-closure-failure' : parityFailure ? 'pose-parity-screen' : 'rejected-step',
                    phase: phase.name, stepIndex, commands, parity, snapshots: fixtures.map(f => f.snapshot()),
                    diagnostics: fixtures.map(f => ({ nonlinear: f.world.lastJointNonlinearFailure, linear: f.containment._jointDiagnostics,
                        trial: f.containment._jointTrialFailure, trace: f.world.coupledClosureTrace, costs: f.world.lastJointCosts })) };
                fs.writeFileSync(path.join(output, 'failure-poses.json.gz'), gzipSync(JSON.stringify(fixtures.map(pose), encode)));
                break;
            }
            if (performance.now() - lastProgress > 10000) {
                console.log(JSON.stringify({ phase: phase.name, stepIndex, catheterMm: fixtures[0].catheter.progress,
                    elapsedMs: performance.now() - began, parity, failures: activePhase.variants.map(v => v.nonlinearFailures) }));
                write('progress.json', { phase: phase.name, stepIndex, catheterMm: fixtures[0].catheter.progress, elapsedMs: performance.now() - began });
                write('runtime-partial.json', report);
                lastProgress = performance.now();
            }
        }
        for (const counters of activePhase.variants) {
            counters.fullMs = summarize(counters.times); counters.physicsMs = summarize(counters.physicsTimes);
            counters.meanCosts = Object.fromEntries(Object.entries(counters.sums).map(([k, v]) => [k, v / activePhase.steps]));
        }
        report.checkpoints.push({ phase: phase.name, snapshots: fixtures.map(f => f.snapshot()), poses: fixtures.map(pose) });
        write('runtime.json', report);
        console.log(JSON.stringify({ phaseComplete: phase.name, steps: activePhase.steps,
            variants: activePhase.variants.map(v => ({ variant: v.variant, fullMs: v.fullMs, nonlinearFailures: v.nonlinearFailures })), failure: report.failure?.reason }));
        if (report.failure) break outer;
    }
    report.complete = !report.failure && report.phases.length === phases.length;
    report.accepted = report.complete && report.phases.every(p => p.variants.every(v => !v.nonlinearFailures));
} catch (error) { report.failure = { reason: 'exception', message: error.message, stack: error.stack, stepIndex }; }
finally {
    report.elapsedMs = performance.now() - began;
    report.sourceAfter = hashes(); report.sourceStable = JSON.stringify(report.sourceBefore) === JSON.stringify(report.sourceAfter);
    report.acceptanceChecks = {
        complete: report.complete, sourceStable: report.sourceStable,
        allClosuresConverged: report.phases.every(p => p.variants.every(v => !v.nonlinearFailures)),
        // Conservative parity screen, not a bound on accumulated physical error.
        positionsWithinRuntimeTolerance: report.phases.every(p => p.maximumPositionDifferenceMm <= report.tolerances?.coupledContainmentTolerance),
        anglesWithinRuntimeTolerance: report.phases.every(p => p.maximumAngleDifferenceRad <= report.tolerances?.coupledAngularToleranceRad)
    };
    report.accepted = Object.values(report.acceptanceChecks).every(Boolean);
    report.unexecutedPhases = phases.slice(report.phases.length).map(p => p.name);
    write('runtime.json', report);
    fixtures.forEach(f => f.dispose()); anatomies.forEach(a => a.dispose());
    console.log(JSON.stringify({ complete: report.complete, accepted: report.accepted, failure: report.failure?.reason, elapsedMs: report.elapsedMs }));
    if (!report.accepted) process.exitCode = 2;
}

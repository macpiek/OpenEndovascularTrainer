// Paired measurement: alternate variant order each step to reduce background
// load/order bias. Both use the same anatomy and built-in mechanics, no renderer.
import fs from 'node:fs';
import { Session } from 'node:inspector';
import { createCoupledRuntimeFixture, loadCoupledRuntimeAnatomy } from '../../tests/helpers/coupledRuntimeFixture.js';
import { EndovascularPhysicsWorld } from '../../src/physics/endovascularPhysicsWorld.js';
import { configureKirchhoffToolRuntime } from '../../src/physics/kirchhoffToolRuntime.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../../src/physics/kirchhoffCoupledSystem.js';
import { ConstraintStageProfile } from '../../src/physics/constraintStageProfile.js';
import { sampleCatheterBrowserBenchmarkCommands, sampleBrowserBenchmarkCommands, sampleGuidewireBrowserBenchmarkCommands, createBrowserBenchmarkCommands } from '../../src/benchmark/browserBenchmarkScenario.js';

const steps = Number(process.argv[2] ?? 2386), anatomy = await loadCoupledRuntimeAnatomy();
const scenario = process.argv[4] ?? 'catheter';
function shortCoupled(ms, out) {
    out.guidewireAdvance = ms < 5000 ? 1 : 0;
    out.catheterAdvance = ms >= 5000 && ms < 9000 ? 1 : ms >= 11000 && ms < 14000 ? -1 : 0;
    out.catheterRotation = ms >= 9000 && ms < 11000 ? 1 : 0;
    out.catheterType = 'berenstein';
    return out;
}
const sampler = { 'short-coupled': shortCoupled, catheter: sampleCatheterBrowserBenchmarkCommands, coupled: sampleBrowserBenchmarkCommands, guidewire: sampleGuidewireBrowserBenchmarkCommands }[scenario];
if (!sampler) throw new Error('Unknown scenario');
const precisionKeys = ['x','y','z','previousX','previousY','previousZ','wallX','wallY','wallZ','wallNormalX','wallNormalY','wallNormalZ','wallT','wallGap'];
const variants = [];
let cpuSession;
const cpuPost = (method, params = {}) => new Promise((resolve, reject) =>
    cpuSession.post(method, params, (error, result) => error ? reject(error) : resolve(result)));
try {
    for (const wide of (process.env.OET_BENCHMARK_WIDE_ONLY === '1' ? [true] : [false, true])) {
        const earlyTrialRejection = true;
        const profile = new ConstraintStageProfile();
        class World extends EndovascularPhysicsWorld {
            createRod(...args) {
                const body = super.createRod(...args);
                // Reproduce the former storage BEFORE fixture initialization.
                if (!wide) for (const key of precisionKeys) body[key] = Float32Array.from(body[key]);
                return body;
            }
            stepFixed() {
                for (const body of this.bodies) configureKirchhoffToolRuntime(body);
                const result = super.stepFixed(); profile.record(this); return result;
            }
        }
        const fixture = createCoupledRuntimeFixture({ ...anatomy, World, coupledSystem: {
            independentComponents: true, physicalTrialState: true, earlyTrialRejection,
            solve: (c, dt, options) => solveKirchhoffCoupledSystem(c, dt, { ...options,
                activeCondensation: true, simultaneousCoulomb: true }), apply: applyKirchhoffCoupledCorrection
        } });
        variants.push({ fixture, profile, wide, phases: {}, depthBuckets: {}, quality: {
            finite: true, maximumLengthErrorMm: 0, maximumLengthErrorRatio: 0,
            maximumPenetrationMm: 0, maximumWireMm: 0, maximumCatheterMm: 0
        } });
    }
    if (process.env.OET_CPU_PROFILE_PATH) {
        cpuSession = new Session(); cpuSession.connect();
        await cpuPost('Profiler.enable');
        await cpuPost('Profiler.setSamplingInterval', { interval: 1000 });
        await cpuPost('Profiler.start');
    }
    const commands = createBrowserBenchmarkCommands();
    const wallLimitMs = Number(process.env.OET_BENCHMARK_WALL_LIMIT_MS ?? Infinity), began = performance.now();
    let wallLimitReached = false;
    for (let i = 0; i < steps; i++) {
        sampler(i * 1000 / 120, commands);
        for (const v of variants.map((_, j) => variants[(i + j) % variants.length])) {
            if (v.error) continue;
            try { v.fixture.step(commands); }
            catch (error) { v.error = { step: i, message: error.message, stack: error.stack, geometry: error.geometry }; }
        }
        for (const v of variants) {
            if (v.error) continue;
            const world = v.fixture.world, q = v.quality;
            const phase = commands.guidewireAdvance ? 'wire-feed-or-withdraw' : commands.catheterAdvance > 0 ? 'catheter-feed'
                : commands.catheterAdvance < 0 ? 'catheter-withdraw' : commands.catheterRotation ? 'rotation' : 'hold';
            (v.phases[phase] ??= new ConstraintStageProfile()).record(world);
            if (process.env.OET_PROFILE_DEPTH_BUCKETS === '1') {
                const depth = commands.guidewireAdvance ? v.fixture.transport.progress : v.fixture.catheter.progress;
                const bucket = `${phase}:${Math.floor(depth / 50) * 50}-${Math.floor(depth / 50) * 50 + 50}mm`;
                (v.depthBuckets[bucket] ??= new ConstraintStageProfile()).record(world);
            }
            q.maximumPenetrationMm = Math.max(q.maximumPenetrationMm, world.settledMaxPenetration);
            q.maximumWireMm = Math.max(q.maximumWireMm, v.fixture.transport.progress);
            q.maximumCatheterMm = Math.max(q.maximumCatheterMm, v.fixture.catheter.progress);
            for (const body of world.bodies) for (let j = body.activeStart; j < body.activeEnd; j++) {
                const length = Math.hypot(body.x[j+1]-body.x[j], body.y[j+1]-body.y[j], body.z[j+1]-body.z[j]);
                q.finite &&= Number.isFinite(length) && Number.isFinite(body.orientationW[j]);
                q.maximumLengthErrorMm = Math.max(q.maximumLengthErrorMm, Math.abs(length-body.restLength[j]));
                q.maximumLengthErrorRatio = Math.max(q.maximumLengthErrorRatio, Math.abs(length/body.restLength[j]-1));
            }
        }
        if ((i + 1) % 600 === 0) console.log(JSON.stringify({ completedScheduleSteps: i + 1,
            variants: variants.map(v => ({ wide: v.wide, completedSteps: v.profile.steps,
                error: v.error?.message ?? null, wireMm: v.fixture.transport.progress, catheterMm: v.fixture.catheter.progress })) }));
        if (variants.every(v => v.error)) break;
        if (performance.now() - began >= wallLimitMs) { wallLimitReached = true; break; }
    }
    const wallTimeMs = performance.now() - began;
    if (cpuSession) {
        const { profile } = await cpuPost('Profiler.stop');
        fs.writeFileSync(process.env.OET_CPU_PROFILE_PATH, JSON.stringify(profile));
        cpuSession.disconnect(); cpuSession = null;
    }
    const report = { steps, scenario, precisionKeys, wallLimitReached, wallTimeMs,
        cpuProfile: process.env.OET_CPU_PROFILE_PATH ?? null, method: 'Interleaved fixed steps, alternating variant order, shared read-only anatomy, no rendering',
        variants: variants.map(v => ({ wide: v.wide, error: v.error ?? null,
            quality: v.quality, depthBuckets: Object.fromEntries(Object.entries(v.depthBuckets).map(([k,p]) => [k,p.report()])), phases: Object.fromEntries(Object.entries(v.phases).map(([k,p]) => [k,p.report()])), profile: v.profile.report(), final: v.fixture.world.getStats() })) };
    if (process.argv[3]) fs.writeFileSync(process.argv[3], JSON.stringify(report, null, 2) + '\n');
    console.log(JSON.stringify(report.variants.map(v => ({wide:v.wide, error:v.error, quality:v.quality, failedSteps:v.profile.failedSteps, meanStepMs:v.profile.fields.total.mean, meanTrials:v.profile.fields.trials.mean}))));
} finally {
    cpuSession?.disconnect();
    for (const variant of variants) variant.fixture.dispose();
    anatomy.dispose();
}

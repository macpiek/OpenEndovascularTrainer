import fs from 'node:fs';
import path from 'node:path';
import os from 'node:os';
import { fileURLToPath } from 'node:url';
import { createHash } from 'node:crypto';
import { createCoupledRuntimeFixture, loadCoupledRuntimeAnatomy, poseFingerprint } from '../../tests/helpers/coupledRuntimeFixture.js';
import { createCoupledSolverSelection } from '../../src/physics/coupledSolverSelection.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../../src/physics/kirchhoffCoupledSystem.js';
import { solveKirchhoffTwoChannelSystem } from '../../src/physics/kirchhoffTwoChannelSystem.js';
import { configureKirchhoffSplitBias } from '../../src/physics/kirchhoffSplitMotion.js';
import { timingSummary, maximumRelativeLengthError } from '../../tests/helpers/coupledValidationMetrics.js';

// Short, paired CPU diagnosis. This intentionally does not launch a browser,
// change source, bypass rejected steps, or certify 120 Hz / 60 FPS.
const root = fileURLToPath(new URL('../../', import.meta.url));
const args = process.argv.slice(2);
const option = (key, fallback) => args.includes(key) ? args[args.indexOf(key) + 1] : fallback;
const output = path.resolve(option('--output', path.join(root, 'reports/composite-baseline.json')));
const pairs = Number(option('--pairs', '3'));
const targetWireMm = Number(option('--wire-mm', '318'));
const synthetic = args.includes('--synthetic');
if (!Number.isSafeInteger(pairs) || pairs < 1 || pairs > 5) throw new RangeError('--pairs must be 1 through 5');
if (!(targetWireMm > 0 && targetWireMm <= 1000)) throw new RangeError('--wire-mm must be in (0, 1000]');
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const json = value => JSON.stringify(value, (_, v) => typeof v === 'number' && !Number.isFinite(v) ? String(v) : v);
const sourcePaths = [];
function collectSources(directory) {
    for (const entry of fs.readdirSync(directory, { withFileTypes: true })) {
        const p = path.join(directory, entry.name);
        if (entry.isDirectory()) collectSources(p);
        else if (entry.name.endsWith('.js')) sourcePaths.push(path.relative(root, p));
    }
}
collectSources(path.join(root, 'src'));
sourcePaths.push('tests/helpers/coupledRuntimeFixture.js', 'tests/helpers/coupledValidationMetrics.js',
    'scripts/physics/benchmark-composite-baseline.mjs', 'package-lock.json');
sourcePaths.sort();
const sourceManifest = () => Object.fromEntries(sourcePaths.map(p => [p, sha(fs.readFileSync(path.join(root, p)))]));

function preparedState(fixture, commands) {
    const bodyInput = fixture.world.bodies.map(body => {
        // Physical arrays, including both momenta and constitutive state;
        // solver caches / timing scratch are deliberately excluded.
        const arrays = Object.fromEntries(Object.keys(body).sort().filter(key =>
            ArrayBuffer.isView(body[key]) && !/coupledClosure|diagnostic|scratch|repair/i.test(key))
            .map(key => [key, { type: body[key].constructor.name, length: body[key].length,
                sha256: sha(new Uint8Array(body[key].buffer, body[key].byteOffset, body[key].byteLength)) }]));
        const fields = ['count', 'segmentCount', 'activeStart', 'activeEnd', 'collisionStartSegment',
            'collisionEndSegment', 'sheathMaterialEndNode', 'mass', 'damping', 'angularDamping',
            'projectionVelocityRetention', 'toolProjectionVelocityRetention', 'relaxationPasses'];
        return { id: body.id, poseHash: poseFingerprint(body),
            scalars: Object.fromEntries(fields.filter(key => key in body).map(key => [key, body[key]])), arrays };
    });
    const containment = Object.fromEntries(['enabled', 'startNode', 'endNode', 'outerStartNode',
        'innerArcOffset', 'containedLength', 'innerRadius', 'friction', 'axialFriction', 'torsionalFriction',
        'enforceDistalPortal'].map(key => [key, fixture.containment[key]]));
    const externalContact = Object.fromEntries(['enabled', 'startSegmentA', 'endSegmentA',
        'startSegmentB', 'endSegmentB', 'friction'].map(key => [key, fixture.externalContact[key]]));
    const input = { fixedDt: fixture.world.fixedDt, commands, wireMm: fixture.transport.progress,
        catheterMm: fixture.catheter.progress, bodyInput, containment, externalContact };
    return { sha256: sha(json(input)), input };
}

function measureStep(fixture, selection, commands, name, kernelRecords) {
    let prepared, observerMs = 0, worldWallMs = 0, worldResult;
    const world = fixture.world, original = world.stepFixed;
    const before = { fixtureSteps: fixture.snapshot().executedSteps, worldSteps: world.stepCount };
    selection.resetDiagnostics();
    kernelRecords.length = 0;
    // The hook sees the exact state AFTER operator/transport preparation and
    // BEFORE prediction/solve. Its independently measured read-only hashing
    // cost is reported and subtracted; raw outer wall-clock is also retained.
    world.stepFixed = function () {
        const observationStart = performance.now();
        prepared = preparedState(fixture, commands);
        observerMs = performance.now() - observationStart;
        const worldStart = performance.now();
        try { return worldResult = original.call(this); }
        finally { worldWallMs = performance.now() - worldStart; }
    };
    const started = performance.now();
    let result, error;
    try { result = fixture.step(commands); }
    catch (caught) { error = { message: caught.message, stack: caught.stack }; }
    const wallMsIncludingObserver = performance.now() - started;
    world.stepFixed = original;
    const kernelObserverMs = kernelRecords.reduce((sum, record) => sum + record.observerMs, 0);
    const stats = world.getStats();
    const rawWorldStepDelta = world.stepCount - before.worldSteps;
    const rawFixtureStepDelta = fixture.snapshot().executedSteps - before.fixtureSteps;
    const accepted = !error && result?.accepted !== false && stats.coupledClosureConverged === true && rawWorldStepDelta === 1;
    const activeCoupling = prepared.input.containment.enabled || prepared.input.externalContact.enabled;
    return { name, commands, prepared, result, error, accepted,
        // A legacy void-return step can advance despite failed closure. It
        // remains visible as a counter defect, never as an accepted sample.
        accounting: { rawWorldStepDelta, rawFixtureStepDelta, executedCertifiedSteps: Number(accepted),
            historyCommits: stats.jointMotion?.historyCommits ?? null,
            historyCommitsAvailable: Number.isInteger(stats.jointMotion?.historyCommits),
            worldAccepted: worldResult?.accepted ?? null, activeCoupling,
            droppedTime: world.droppedTime, browserBacklog: null },
        time: { wallMsIncludingObserver, observerMs, kernelObserverMs,
            fullStepMs: wallMsIncludingObserver - observerMs - kernelObserverMs,
            worldWallMs, runtimeRecordedWorldMs: world.timings.total.last },
        after: fixture.snapshot(),
        work: { solver: stats.coupledSolver, coupledClosurePasses: stats.coupledClosurePasses,
            coupledContactPasses: stats.coupledContactPasses, jointFactorizations: stats.jointFactorizations,
            jointLinearIterations: stats.jointLinearIterations, jointTrialEvaluations: stats.jointTrialEvaluations,
            jointBacktracks: stats.jointBacktracks, jointMaximumRows: stats.jointMaximumRows,
            jointMaximumBand: stats.jointMaximumBand,
            directFactorizations: stats.bodies.map(b => b.directFactorizations),
            directFactorReuses: stats.bodies.map(b => b.directFactorReuses),
            contactCount: stats.contacts, containments: stats.containments,
            selection: selection.getReport(world), linearSolves: kernelRecords.slice() },
        gates: { coupledClosureConverged: stats.coupledClosureConverged,
            contactToleranceMm: world.coupledContainmentTolerance,
            lengthToleranceRelative: world.coupledLengthTolerance,
            angularToleranceRad: world.coupledAngularToleranceRad,
            coneTolerance: 1e-9, maximumRelativeLengthError: world.bodies.map(maximumRelativeLengthError),
            settledMaxPenetration: stats.settledMaxPenetration,
            jointMotion: stats.jointMotion } };
}

const report = { schema: 'oet-composite-baseline-v1', measuredAt: new Date().toISOString(),
    scope: 'Paired Node full-actuation-step CPU diagnosis at first coupling. Independent fixtures replay every preparation dt. No browser FPS/backlog certification; no skipped dt.',
    host: { cpu: os.cpus()[0]?.model, node: process.version, arch: process.arch, platform: process.platform },
    sourceBefore: sourceManifest(), protocol: { pairs, targetWireMm, anatomy: !synthetic,
        fixedDt: 1 / 120, preCouplingCatheterMm: 3.9, command: { catheterAdvance: 1 },
        order: 'AB / BA alternating, each candidate gets fresh, identically prepared fixture',
        timing: 'fullStepMs = outer fixture.step wall-clock minus separately timed read-only prepared-state observer; raw outer and World elapsed also included',
        warmup: 'All real uncoupled wire timesteps and catheter preparation; counts recorded per run, no artificial coupled warmup',
        limits: { meanMs: 4, p95Ms: 6, physicsHz: 120, fps: 60 },
        historyLimitation: 'reference exposes no one-commit counter; missing history telemetry is null, not one' },
    runs: [] };
let anatomy;
try {
    const anatomyStart = performance.now();
    if (!synthetic) anatomy = await loadCoupledRuntimeAnatomy();
    report.anatomyLoadMs = performance.now() - anatomyStart;
    report.anatomyFiles = anatomy ? Object.fromEntries(['Aorta_plain.stl', 'Aorta_plain.collision.bin']
        .map(name => [name, sha(fs.readFileSync(path.join(root, 'res', name)))])) : null;
    for (let pair = 0; pair < pairs; pair++) {
        const order = pair % 2 ? ['joint-two-channel', 'reference'] : ['reference', 'joint-two-channel'];
        for (const variant of order) {
            const kernelRecords = [];
            const observeSolve = solve => (constraint, dt, options) => {
                const start = performance.now();
                const result = solve(constraint, dt, options);
                const solveMs = performance.now() - start;
                const observationStart = performance.now();
                const record = { solveMs, diagnostics: structuredClone(result.diagnostics) };
                kernelRecords.push(record);
                record.observerMs = performance.now() - observationStart;
                return result;
            };
            const selection = createCoupledSolverSelection(variant, { solve: solveKirchhoffCoupledSystem,
                apply: applyKirchhoffCoupledCorrection, solveTwoChannel: observeSolve(solveKirchhoffTwoChannelSystem) });
            const fixture = createCoupledRuntimeFixture({ ...(anatomy ? { vessel: anatomy.vessel, field: anatomy.field } : {}),
                coupledSystem: selection.coupledSystem, jointMotionMode: selection.jointMotionMode });
            if (selection.biasMaterialMode) configureKirchhoffSplitBias(fixture.containment, { materialMode: selection.biasMaterialMode });
            const run = { pair, variant, order: order.indexOf(variant), preparation: {}, measured: [] };
            report.runs.push(run);
            const start = performance.now();
            try {
                let wireSteps = 0, catheterSteps = 0;
                while (fixture.transport.progress < targetWireMm - 1e-9) {
                    const command = Math.min(1, (targetWireMm - fixture.transport.progress) / (44 / 120));
                    const step = fixture.step({ guidewireAdvance: command });
                    if (step?.accepted === false) throw new Error('Wire preparation rejected; no dt skipped');
                    wireSteps++;
                }
                while (fixture.catheter.progress < 3.9 - 52 / 120 - 1e-9) {
                    const step = fixture.step({ catheterAdvance: 1 });
                    if (step?.accepted === false) throw new Error('Catheter preparation rejected; no dt skipped');
                    if (fixture.containment.enabled || fixture.externalContact.enabled) throw new Error('Unexpected early coupled preparation');
                    catheterSteps++;
                }
                run.preparation = { wallMs: performance.now() - start, wireSteps, catheterSteps,
                    snapshot: fixture.snapshot(), config: fixture.config };
                run.measured.push(measureStep(fixture, selection, { catheterAdvance: 1 }, 'before-coupling', kernelRecords));
                if (!run.measured.at(-1).accepted) throw new Error('Pre-coupling step uncertified; no continuation');
                run.measured.push(measureStep(fixture, selection, { catheterAdvance: 1 }, 'first-coupling', kernelRecords));
                process.stderr.write(JSON.stringify({ pair, variant, preparationMs: run.preparation.wallMs,
                    steps: run.measured.map(m => ({ name: m.name, wireMm: m.prepared.input.wireMm,
                        catheterMm: m.prepared.input.catheterMm, accepted: m.accepted,
                        worldMs: m.time.worldWallMs, fullStepMs: m.time.fullStepMs,
                        activeCoupling: m.accounting.activeCoupling })) }) + '\n');
            } catch (error) { run.error = { message: error.message, stack: error.stack }; }
            finally { fixture.dispose(); }
            fs.writeFileSync(output, JSON.stringify(report, null, 2) + '\n');
        }
    }
    report.preparedPairComparisons = Array.from({ length: pairs }, (_, pair) => {
        const a = report.runs.find(r => r.pair === pair && r.variant === 'reference');
        const b = report.runs.find(r => r.pair === pair && r.variant === 'joint-two-channel');
        return { pair, steps: ['before-coupling', 'first-coupling'].map(name => {
            const reference = a.measured.find(m => m.name === name)?.prepared;
            const joint = b.measured.find(m => m.name === name)?.prepared;
            return { name, referenceSha256: reference?.sha256, jointSha256: joint?.sha256,
                identical: !!reference && reference.sha256 === joint?.sha256 };
        }) };
    });
    report.summary = ['reference', 'joint-two-channel'].flatMap(variant =>
        ['before-coupling', 'first-coupling'].map(name => {
            const samples = report.runs.filter(r => r.variant === variant).flatMap(r => r.measured).filter(m => m.name === name);
            const accepted = samples.filter(m => m.accepted);
            const full = timingSummary(accepted.map(m => m.time.fullStepMs));
            const attempt = timingSummary(samples.map(m => m.time.fullStepMs));
            return { variant, name, attempts: samples.length, accepted: accepted.length,
                failed: samples.length - accepted.length, acceptedFullStep: full, attemptedFullStep: attempt,
                acceptedWorldStep: timingSummary(accepted.map(m => m.time.worldWallMs)),
                requiredMeanSpeedupTo4Ms: full.meanMs === null ? null : full.meanMs / 4,
                observedAttemptMeanOver4Ms: attempt.meanMs === null ? null : attempt.meanMs / 4,
                timingQualification: 'diagnosis only; this small sample does not establish P95 or browser performance' };
        }));
} catch (error) { report.error = { message: error.message, stack: error.stack }; }
finally {
    anatomy?.dispose();
    report.sourceAfter = sourceManifest();
    report.sourceStable = json(report.sourceBefore) === json(report.sourceAfter);
    fs.writeFileSync(output, JSON.stringify(report, null, 2) + '\n');
}
console.log(JSON.stringify({ output, sourceStable: report.sourceStable,
    preparedPairComparisons: report.preparedPairComparisons, summary: report.summary, error: report.error }, null, 2));

import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { createHash } from 'node:crypto';
import { createCoupledRebuildPhases, DEEP_CATHETER_BENCHMARK_PHASES,
    sampleShortCatheterBenchmarkCommands, ShortCatheterBenchmarkMetrics } from '../src/benchmark/shortCatheterBenchmark.js';
import { COUPLED_VALIDATION_LIMITS, assessCoupledTiming } from '../tests/helpers/coupledValidationMetrics.js';
import { createCoupledSolverSelection } from '../src/physics/coupledSolverSelection.js';

const projectRoot = fileURLToPath(new URL('../', import.meta.url));
const argv = process.argv.slice(2);
const option = name => { const i = argv.indexOf(name); return i < 0 ? null : argv[i + 1]; };
const allowed = ['--help', '--plan', '--full', '--deep', '--no-anatomy', '--source-root',
    '--anatomy-root', '--through-phase', '--steps', '--output', '--compare-browser', '--solver'];
for (const arg of argv.filter(value => value.startsWith('--'))) if (!allowed.includes(arg)) throw new Error(`Unknown option ${arg}`);
if (argv.includes('--help')) {
    console.log(`Usage: node scripts/benchmark-coupled-rebuild.mjs [options]
Default: 60-step smoke with runtime anatomy (not a performance acceptance run).
--plan                     Print the exact full protocol, no physics
--full                     Execute the full protocol; coordinate CPU use first
--deep                     Use the existing 10/20/40/60 cm browser protocol
--through-phase NAME       Execute all preparation and steps up through this phase
--steps N                  Cap the number of executed steps, never skip preparation
--source-root PATH         Replay that tree's engine with this audited adapter
--anatomy-root PATH        Read res/ in an existing tree, without copying anatomy
--solver NAME              reference (default), joint, or joint-active-coulomb
                           joint-active-coulomb enables active condensation + Coulomb Newton
--no-anatomy               Synthetic open-space smoke only
--compare-browser PATH    Compare completed phase fingerprints with a saved report
--output PATH             Save complete JSON results (phase progress goes to stderr)
Node measures full actuation + physics + synchronization CPU time, not rendering.
No option starts Vite, opens browser tabs, or claims browser 60 FPS.`);
    process.exit(0);
}
if (argv.includes('--solver') && !option('--solver')) throw new Error('--solver requires a variant name');
const definitions = argv.includes('--deep') ? DEEP_CATHETER_BENCHMARK_PHASES : createCoupledRebuildPhases();
const totalSteps = definitions.reduce((sum, phase) => sum + phase.steps, 0);
if (argv.includes('--plan')) {
    console.log(JSON.stringify({ fixedDt: 1 / 120, totalSteps, simulationSeconds: totalSteps / 120,
        maximumCatheterMm: 1000, guidewireTargetMm: 999.9, phases: definitions }, null, 2));
    process.exit(0);
}
let expectedSteps = argv.includes('--full') ? totalSteps : 60;
const through = option('--through-phase');
if (through) {
    const index = definitions.findIndex(p => p.name === through);
    if (index < 0) throw new Error(`Unknown phase ${through}`);
    expectedSteps = definitions.slice(0, index + 1).reduce((sum, p) => sum + p.steps, 0);
}
if (option('--steps')) {
    const cap = Number(option('--steps'));
    if (!Number.isSafeInteger(cap) || cap <= 0) throw new Error('--steps must be a positive integer');
    expectedSteps = argv.includes('--full') || through ? Math.min(expectedSteps, cap) : Math.min(totalSteps, cap);
}
const sourceRoot = path.resolve(option('--source-root') ?? projectRoot);
const anatomyRoot = path.resolve(option('--anatomy-root') ?? sourceRoot);
const digest = filename => createHash('sha256').update(fs.readFileSync(filename)).digest('hex');
function sourceManifest(directory) {
    const entries = [];
    for (const entry of fs.readdirSync(directory, { withFileTypes: true }).sort((a, b) => a.name.localeCompare(b.name))) {
        const filename = path.join(directory, entry.name);
        if (entry.isDirectory()) entries.push(...sourceManifest(filename));
        else if (entry.name.endsWith('.js')) entries.push([path.relative(sourceRoot, filename), digest(filename)]);
    }
    return entries;
}
let stage = null, anatomy = null, fixture = null;
try {
    let adapter = path.join(projectRoot, 'tests/helpers/coupledRuntimeFixture.js');
    if (sourceRoot !== path.resolve(projectRoot)) {
        // Only stage the adapter. Source/anatomy stay in the requested tree.
        stage = fs.mkdtempSync(path.join(os.tmpdir(), 'oet-coupled-validation-'));
        fs.mkdirSync(path.join(stage, 'tests/helpers'), { recursive: true });
        fs.writeFileSync(path.join(stage, 'package.json'), '{"type":"module"}\n');
        fs.symlinkSync(path.join(sourceRoot, 'src'), path.join(stage, 'src'));
        fs.symlinkSync(path.join(projectRoot, 'node_modules'), path.join(stage, 'node_modules'));
        fs.copyFileSync(adapter, path.join(stage, 'tests/helpers/coupledRuntimeFixture.js'));
        adapter = path.join(stage, 'tests/helpers/coupledRuntimeFixture.js');
    }
    const { createCoupledRuntimeFixture, loadCoupledRuntimeAnatomy } = await import(pathToFileURL(adapter));
    const sourceFiles = sourceManifest(path.join(sourceRoot, 'src'));
    const sourceHash = createHash('sha256').update(JSON.stringify(sourceFiles)).digest('hex');
    const solver = option('--solver') ?? 'reference';
    const kernel = solver === 'reference' ? {} : await import(pathToFileURL(path.join(sourceRoot, 'src/physics/kirchhoffCoupledSystem.js')));
    const selection = createCoupledSolverSelection(solver, {
        solve: kernel.solveKirchhoffCoupledSystem, apply: kernel.applyKirchhoffCoupledCorrection
    });
    if (!argv.includes('--no-anatomy')) anatomy = await loadCoupledRuntimeAnatomy(pathToFileURL(anatomyRoot + path.sep));
    fixture = createCoupledRuntimeFixture({
        ...(anatomy ? { vessel: anatomy.vessel, field: anatomy.field } : {}),
        coupledSystem: selection.coupledSystem
    });
    const initial = fixture.snapshot();
    const metrics = new ShortCatheterBenchmarkMetrics(definitions);
    const fullStepTimes = [], worldStepTimes = [], perPhaseFullTimes = definitions.map(() => []);
    const checkpoints = [];
    const commands = {};
    let elapsedMs = 0;
    const start = performance.now();
    for (let step = 0; step < expectedSteps; step++) {
        sampleShortCatheterBenchmarkCommands(elapsedMs, commands, definitions);
        if (commands.benchmarkPhase < 0) throw new Error('Protocol ended before its declared step count');
        const before = performance.now();
        const state = fixture.step(commands);
        const fullMs = performance.now() - before;
        fullStepTimes.push(fullMs);
        worldStepTimes.push(fixture.world.timings.total.last);
        perPhaseFullTimes[commands.benchmarkPhase].push(fullMs);
        metrics.recordStep(commands.benchmarkPhase, fixture.world, state.wireMm, state.catheterMm, 0);
        elapsedMs += fixture.config.fixedDt * 1000;
        const phase = metrics.phases[commands.benchmarkPhase];
        if (phase.steps === definitions[commands.benchmarkPhase].steps) {
            const checkpoint = { phase: phase.name, ...fixture.snapshot() };
            checkpoints.push(checkpoint);
            process.stderr.write(JSON.stringify({ phase: phase.name, steps: step + 1,
                wireMm: state.wireMm, catheterMm: state.catheterMm, fingerprints: checkpoint.fingerprints }) + '\n');
        }
    }
    const wallSeconds = (performance.now() - start) / 1000;
    const timing = assessCoupledTiming({ expectedSteps, executedSteps: fullStepTimes.length, fullStepTimes, worldStepTimes });
    const phases = metrics.report().map((phase, i) => {
        const accounting = assessCoupledTiming({ expectedSteps: definitions[i].steps, executedSteps: phase.steps,
            fullStepTimes: perPhaseFullTimes[i], worldStepTimes: metrics.phases[i].stepTimes });
        // The offline replay has no browser scheduler. Zero placeholders fed to
        // the shared metrics must never be published as measured zero backlog.
        return { ...phase, startBacklogSeconds: null, endBacklogSeconds: null, peakBacklogSeconds: null,
            timing: accounting };
    });
    let comparison = null;
    if (option('--compare-browser')) {
        const browser = JSON.parse(fs.readFileSync(option('--compare-browser'), 'utf8'));
        const rows = browser.uncoupledPoseFingerprints ?? (browser.phases ?? browser.deepCatheterPhases ?? [])
            .filter(p => p.endPoseFingerprints).map(p => [p.name, ...p.endPoseFingerprints.map(b => b.hash)]);
        comparison = rows.map(([name, ...hashes]) => {
            const actual = checkpoints.find(p => p.phase === name)?.fingerprints.map(p => p.hash) ?? null;
            return { phase: name, expected: hashes, actual,
                matches: actual ? hashes.length === actual.length && hashes.every((hash, i) => hash === actual[i]) : null };
        });
    }
    const report = { solver, coupledSolver: selection.getReport(fixture.world),
        schema: 'oet-coupled-rebuild-validation-v1', measuredAt: new Date().toISOString(),
        scope: 'Node fixed-step replay; no browser FPS/backlog certification',
        sourceRoot, anatomyRoot: anatomy ? anatomyRoot : null, sourceHash, sourceFiles,
        adapterSha256: digest(adapter),
        solverSelectorSha256: digest(path.join(projectRoot, 'src/physics/coupledSolverSelection.js')),
        packageLockSha256: digest(path.join(sourceRoot, 'package-lock.json')),
        anatomyFiles: anatomy ? ['Aorta_plain.stl', 'Aorta_plain.collision.bin'].map(name =>
            [name, digest(path.join(anatomyRoot, 'res', name))]) : [],
        nodeVersion: process.version, platform: `${process.platform}/${process.arch}`,
        config: fixture.config, limits: COUPLED_VALIDATION_LIMITS,
        protocol: argv.includes('--deep') ? 'deep-existing' : 'coupled-full',
        totalProtocolSteps: totalSteps, requestedSteps: expectedSteps,
        fullProtocolComplete: expectedSteps === totalSteps && timing.complete,
        simulationSeconds: fullStepTimes.length * fixture.config.fixedDt, wallSeconds,
        cpuTimeRatio: wallSeconds / (fullStepTimes.length * fixture.config.fixedDt),
        timing, initial, final: fixture.snapshot(), checkpoints, phases, fingerprintComparison: comparison,
        allStepTimesMs: fullStepTimes, allWorldStepTimesMs: worldStepTimes
    };
    // Verify the measured tree did not mutate during a run.
    report.sourceStable = sourceHash === createHash('sha256')
        .update(JSON.stringify(sourceManifest(path.join(sourceRoot, 'src')))).digest('hex');
    if (!report.sourceStable) process.exitCode = 2;
    const text = JSON.stringify(report, null, 2) + '\n';
    if (option('--output')) fs.writeFileSync(path.resolve(option('--output')), text);
    else process.stdout.write(text);
} finally {
    fixture?.dispose(); anatomy?.dispose();
    if (stage) fs.rmSync(stage, { recursive: true, force: true });
}

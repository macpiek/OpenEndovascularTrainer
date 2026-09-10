import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { createHash } from 'node:crypto';
import inspector from 'node:inspector';

const root = fileURLToPath(new URL('../../', import.meta.url));
if (process.argv[2] === '--trace') {
    await runTrace(process.argv[3], process.argv[4] ?? '/tmp/oet-composite-timestep-trace.json');
    process.exit(0);
}
const output = path.resolve(process.argv[2] ?? '/tmp/oet-composite-timestep.json');
const snapshot = fs.mkdtempSync('/tmp/oet-composite-timestep-snapshot-');
const entry = 'scripts/physics/helpers/compositeTimeStepBenchmark.js';
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const files = new Map(), provenance = ['src/physics/endovascularPhysicsWorld.js', 'src/simulator.js', 'src/pigtailCatheter.js',
    'tests/kirchhoffCompositeTimeStep.test.js', 'tests/kirchhoffCompositeWallTimeStep.test.js'];
function capture(relative, recurse = true) {
    if (files.has(relative)) return;
    const original = path.join(root, relative), bytes = fs.readFileSync(original);
    files.set(relative, { sha256: sha(bytes), bytes: bytes.length });
    const target = path.join(snapshot, relative); fs.mkdirSync(path.dirname(target), { recursive: true }); fs.writeFileSync(target, bytes);
    if (!recurse || !/\.(js|mjs)$/.test(relative)) return;
    const source = bytes.toString('utf8');
    const imports = [...source.matchAll(/(?:\bfrom\s*|\bimport\s*\(?\s*)['"]([^'"]+)['"]/g)].map(m => m[1]);
    for (const specifier of imports) {
        if (specifier.startsWith('.')) capture(path.normalize(path.join(path.dirname(relative), specifier)));
        else if (specifier === 'three') {
            capture('node_modules/three/package.json', false); capture('node_modules/three/build/three.module.js');
        } else if (!specifier.startsWith('node:')) throw new Error(`Uncaptured package ${specifier} in ${relative}`);
    }
}
capture(entry);
for (const p of provenance) capture(p, p.startsWith('tests/'));
capture('scripts/physics/benchmark-composite-timestep.mjs', false);
fs.writeFileSync(path.join(snapshot, 'package.json'), '{"type":"module"}\n');
const sourceStableAtCapture = [...files].every(([p, meta]) => sha(fs.readFileSync(path.join(root, p))) === meta.sha256);
if (!sourceStableAtCapture) throw new Error('Root changed dependencies during capture; rerun to create a coherent snapshot');
const snapshotManifest = { captured: new Date().toISOString(), root, snapshot, sourceStableAtCapture, files: Object.fromEntries(files) };
fs.writeFileSync(path.join(snapshot, 'manifest.json'), JSON.stringify(snapshotManifest, null, 2) + '\n');
for (const p of files.keys()) fs.chmodSync(path.join(snapshot, p), 0o444);
fs.chmodSync(path.join(snapshot, 'manifest.json'), 0o444); fs.chmodSync(path.join(snapshot, 'package.json'), 0o444);
function sealDirectories(directory) {
    for (const item of fs.readdirSync(directory, { withFileTypes: true })) if (item.isDirectory()) sealDirectories(path.join(directory, item.name));
    fs.chmodSync(directory, 0o555);
}
sealDirectories(snapshot);

// Transparent timing/count wrappers around WASM exports do not alter the
// immutable source or numerical inputs. Their small overhead is INCLUDED in
// every wall-clock row. Native functions receive their original arguments.
const NativeInstance = WebAssembly.Instance;
let counters = {};
WebAssembly.Instance = function(module, imports) {
    const instance = new NativeInstance(module, imports), exports = {};
    for (const [name, value] of Object.entries(instance.exports)) exports[name] = typeof value !== 'function' ? value : (...args) => {
        const begin = performance.now();
        try { return value(...args); }
        finally {
            const row = counters[name] ??= { count: 0, milliseconds: 0 }; row.count++; row.milliseconds += performance.now() - begin;
        }
    };
    return { exports };
};
const { makeFixture, preparedOptions, advance, conventions } = await import(pathToFileURL(path.join(snapshot, entry)).href);
const bodySource = fs.readFileSync(path.join(snapshot, 'src/physics/endovascularPhysicsWorld.js'), 'utf8');
const simulatorSource = fs.readFileSync(path.join(snapshot, 'src/simulator.js'), 'utf8');
const catheterSource = fs.readFileSync(path.join(snapshot, 'src/pigtailCatheter.js'), 'utf8');
const bodyMass = Number(bodySource.match(/guidewire: Object\.freeze\(\{[\s\S]*?mass:\s*([\d.]+)/)?.[1]);
const catheterMass = Number(bodySource.match(/catheter: Object\.freeze\(\{[\s\S]*?mass:\s*([\d.]+)/)?.[1]);
const wireSpacing = Number(simulatorSource.match(/const segmentLength = ([\d.]+)/)?.[1]);
const catheterSpacing = Number(catheterSource.match(/const DEFAULT_PATH_SPACING = ([\d.]+)/)?.[1]);
if (bodyMass / wireSpacing !== conventions.mass.wire || catheterMass / catheterSpacing !== conventions.mass.catheter)
    throw new Error('Captured Body mass/spacing convention changed; update the explicit benchmark variant before running');
function serial(value) {
    if (typeof value === 'function') return '[frozen-function]';
    if (ArrayBuffer.isView(value)) return Array.from(value, serial);
    if (Array.isArray(value)) return value.map(serial);
    if (value instanceof Map) return [...value].map(serial);
    if (value && typeof value === 'object') return Object.fromEntries(Object.entries(value).map(([k, v]) => [k, serial(v)]));
    return value;
}
const hash = value => sha(JSON.stringify(serial(value)));
const median = values => { const v = [...values].sort((a, b) => a - b), i = v.length >> 1;
    return v.length ? v.length % 2 ? v[i] : (v[i - 1] + v[i]) / 2 : null; };
const report = { date: new Date().toISOString(), snapshot, sourceStableAtCapture, snapshotManifestHash: sha(fs.readFileSync(path.join(snapshot, 'manifest.json'))),
    scope: 'FULL prepared fixed-topology dt, accepted or rejected. Exact real profile boundaries, no feed/remesh. Body nominal mass convention, quasi-static torsion, explicit frictionless analytic plane control. No 60FPS claim.',
    anatomy: { used: false, reason: 'Existing prepared anatomy fixtures contain two distinct tool curves and histories; there is no validated transfer to a common centerline. Snapping one curve onto the other would alter mechanics. Analytic-plane control uses the same capsule field interface without that hidden transfer.' },
    instrumentation: 'Immutable source snapshot. Transparent WASM export timing/count wrappers; overhead included. CPU sampling separately profiles the whole run. Captured root changes after snapshot do not change executed bytes.',
    conventions, repeats: 2, attemptsPerRepeat: 2, cases: [] };
const session = new inspector.Session(); session.connect();
const post = (name, args = {}) => new Promise((resolve, reject) => session.post(name, args, (err, result) => err ? reject(err) : resolve(result)));
await post('Profiler.enable'); await post('Profiler.setSamplingInterval', { interval: 500 }); await post('Profiler.start');
for (const scenario of ['contact-free', 'analytic-plane']) for (const insertion of [9, 160, 310]) {
    const seriesRows = { 'strict-tests': [], 'runtime-geometric': [] };
    for (let repeat = 0; repeat < report.repeats; repeat++) for (const series of repeat % 2 ? ['runtime-geometric', 'strict-tests'] : ['strict-tests', 'runtime-geometric']) {
        const preparationStart = performance.now(), fixture = makeFixture(insertion, scenario);
        const setupMs = performance.now() - preparationStart; let state = fixture.state;
        for (let attempt = 0; attempt < report.attemptsPerRepeat; attempt++) {
            const before = hash(state), oldTime = state.time, oldStep = state.step;
            counters = {}; const begin = performance.now(), options = preparedOptions(fixture, state, series), prepared = performance.now();
            const result = advance(state, options), done = performance.now();
            const afterOriginal = hash(state), changed = hash(result.state), d = result.diagnostics;
            if (before !== afterOriginal || (!result.accepted && result.state !== state)) throw new Error('Rejected/original state mutated');
            if (result.accepted ? result.state.step !== oldStep + 1 || result.state.time !== oldTime + conventions.dt || d.historyCommits !== 1
                : result.state.step !== oldStep || result.state.time !== oldTime || d.historyCommits !== 0) throw new Error('Physical dt/history accounting failed');
            seriesRows[series].push({ repeat, attempt, stage: oldStep === 0 ? 'initial-straight-or-retry' : 'consecutive-loaded-dt',
                setupMs: attempt ? 0 : setupMs, preparationMs: prepared - begin, solverMs: done - prepared, fullDtMs: done - begin,
                accepted: result.accepted, status: result.status, originalStateHash: before, resultStateHash: changed,
                preparedInputHash: hash(options), compiledMaterialHash: hash(fixture.mesh.materialCells.map(cell => ({
                    id: cell.id, vertex: cell.vertex, material: cell.material, materialLength: cell.materialLength }))),
                rollbackOriginalHash: afterOriginal, oldStep, step: result.state.step,
                oldTime, time: result.state.time, nodes: state.layout.nodeCount, dofCount: state.layout.dofCount,
                activeToolEdges: Object.fromEntries([...state.layout.spins].map(([id, offsets]) => [id, [...offsets].filter(dof => dof >= 0).length])),
                requiredBoundaries: fixture.topology.boundaries.map(b => b.x), wasm: structuredClone(counters),
                diagnostics: serial(d) });
            if (result.accepted) state = result.state;
            fs.writeFileSync(output + '.partial', JSON.stringify({ ...report, current: { scenario, insertion, seriesRows } }, null, 2) + '\n');
        }
    }
    report.cases.push({ scenario, insertion, series: seriesRows });
}
const { profile } = await post('Profiler.stop'); session.disconnect(); WebAssembly.Instance = NativeInstance;
fs.writeFileSync(output + '.cpuprofile', JSON.stringify(profile));
const frames = new Map(profile.nodes.map(node => [node.id, node.callFrame])), samples = new Map();
(profile.samples ?? []).forEach((id, i) => { const f = frames.get(id), key = `${f.functionName || '(anonymous)'} @ ${f.url}:${f.lineNumber + 1}`;
    samples.set(key, (samples.get(key) ?? 0) + (profile.timeDeltas?.[i] ?? 0)); });
report.cpuTopSelfMs = [...samples].sort((a, b) => b[1] - a[1]).slice(0, 25).map(([functionLocation, us]) => ({ functionLocation, milliseconds: us / 1000 }));
report.snapshotIntact = [...files].every(([p, meta]) => sha(fs.readFileSync(path.join(snapshot, p))) === meta.sha256);
report.rootChangedSinceSnapshot = [...files].filter(([p, meta]) => sha(fs.readFileSync(path.join(root, p))) !== meta.sha256).map(([p]) => p);
report.pairComparisons = report.cases.flatMap(c => Object.entries(c.series).flatMap(([series, rows]) => [0, 1].map(attempt => {
    const a = rows.find(r => r.repeat === 0 && r.attempt === attempt), b = rows.find(r => r.repeat === 1 && r.attempt === attempt);
    return { scenario:c.scenario,insertion:c.insertion,series,attempt,originalStateMatches:a.originalStateHash===b.originalStateHash,
        preparedInputMatches:a.preparedInputHash===b.preparedInputHash,compiledMaterialMatches:a.compiledMaterialHash===b.compiledMaterialHash,
        resultStateMatches:a.resultStateHash===b.resultStateHash,acceptedMatches:a.accepted===b.accepted };
})));
report.summary = report.cases.map(c => ({ scenario: c.scenario, insertion: c.insertion,
    series: Object.fromEntries(Object.entries(c.series).map(([name, rows]) => { const accepted = rows.filter(r => r.accepted), failed = rows.filter(r => !r.accepted);
        return [name, { accepted: accepted.length, rejected: failed.length, medianAcceptedDtMs: median(accepted.map(r => r.fullDtMs)),
            failedAttemptMs: failed.map(r => r.fullDtMs), statuses: rows.map(r => r.status), evaluations: rows.map(r => r.diagnostics.evaluations),
            directions: rows.map(r => r.diagnostics.directions), bandBacksolves: rows.map(r => r.wasm.solveBand?.count ?? 0) }]; })) }));
fs.writeFileSync(output, JSON.stringify(report, null, 2) + '\n');
console.log(JSON.stringify({ output, snapshot, snapshotIntact: report.snapshotIntact, rootChangedSinceSnapshot: report.rootChangedSinceSnapshot,
    summary: report.summary, cpuTopSelfMs: report.cpuTopSelfMs.slice(0, 12) }, null, 2));

async function runTrace(sourceSnapshot, destination) {
    if (!sourceSnapshot || !fs.existsSync(path.join(sourceSnapshot, 'manifest.json')))
        throw new Error('--trace requires an existing immutable snapshot directory');
    const traceRoot = fs.mkdtempSync('/tmp/oet-composite-timestep-trace-');
    fs.cpSync(sourceSnapshot, traceRoot, { recursive: true });
    const sourcePath = path.join(traceRoot, 'src/physics/kirchhoffCompositeTimeStep.js');
    const before = fs.readFileSync(sourcePath, 'utf8'), hashText = value => createHash('sha256').update(value).digest('hex');
    let source = before;
    const replace = (a, b) => {
        if (source.split(a).length !== 2) throw new Error(`Trace instrumentation anchor is missing/nonunique: ${a.slice(0, 90)}`);
        source = source.replace(a, b);
    };
    replace('let last = null;', 'let last = null; const trace = globalThis.__OET_COMPOSITE_TRACE__;');
    replace('return { objective: finite(chain.energy, \'objective\'), certificate, residuals, reactions, originalResidual: original, wallTrial,',
        `trace?.push({ event:'evaluate', augmented, evaluation:stats.evaluations, direction:stats.directions, outer:stats.outerIterations,
            penalty, wallPenalty, objective:chain.energy, original:{...certificate}, augmentedResidual:residualNorms(chain.gradient,layout,fixed),
            maximumLengthMultiplier:Math.max(...Array.from(lambda,Math.abs)), dualUpdates:stats.dualUpdates });
        return { objective: finite(chain.energy, 'objective'), certificate, residuals, reactions, originalResidual: original, wallTrial,`);
    replace('current = candidate; last = current; stats.acceptedTrials++; acceptedTrial = true; break;',
        `trace?.push({event:'accepted-alpha',evaluation:stats.evaluations,direction:stats.directions,outer:stats.outerIterations,
            alpha:scale,slope,objectiveBefore:current.objective,objectiveAfter:candidate.objective,
            maximumDirection:Math.max(...Array.from(delta,Math.abs)),original:{...candidate.certificate},augmentedResidual:{...candidate.augmented}});
        current = candidate; last = current; stats.acceptedTrials++; acceptedTrial = true; break;`);
    fs.chmodSync(sourcePath, 0o644); fs.writeFileSync(sourcePath, source); fs.chmodSync(sourcePath, 0o444);
    const { makeFixture, preparedOptions, advance } = await import(pathToFileURL(path.join(traceRoot, 'scripts/physics/helpers/compositeTimeStepBenchmark.js')).href);
    const fixture = makeFixture(310, 'contact-free'), options = preparedOptions(fixture, fixture.state, 'strict-tests');
    globalThis.__OET_COMPOSITE_TRACE__ = [];
    const begin = performance.now(), result = advance(fixture.state, options), elapsedMs = performance.now() - begin;
    const events = globalThis.__OET_COMPOSITE_TRACE__; delete globalThis.__OET_COMPOSITE_TRACE__;
    const accepted = events.filter(e => e.event === 'accepted-alpha'), evaluations = events.filter(e => e.event === 'evaluate');
    const trace = { scope:'Single costly cold310 contact-free trace. Separate copied snapshot has only diagnostics injected into TimeStep. Does not replace the immutable timing baseline.',
        sourceSnapshot, traceRoot, originalTimeStepSha:hashText(before), instrumentedTimeStepSha:hashText(source),
        baselineManifestSha:hashText(fs.readFileSync(path.join(sourceSnapshot,'manifest.json'))),
        elapsedMs, accepted:result.accepted,status:result.status,diagnostics:result.diagnostics,
        firstEvaluation:evaluations[0],lastAccepted:accepted.at(-1),lastEvaluation:evaluations.at(-1),
        alphaCounts:accepted.reduce((counts,e)=>(counts[e.alpha]=(counts[e.alpha]??0)+1,counts),{}),events };
    fs.writeFileSync(destination, JSON.stringify(trace,null,2)+'\n');
    console.log(JSON.stringify({destination,traceRoot,originalTimeStepSha:trace.originalTimeStepSha,instrumentedTimeStepSha:trace.instrumentedTimeStepSha,
        elapsedMs,status:trace.status,firstEvaluation:trace.firstEvaluation,lastAccepted:trace.lastAccepted,lastEvaluation:trace.lastEvaluation,alphaCounts:trace.alphaCounts},null,2));
}

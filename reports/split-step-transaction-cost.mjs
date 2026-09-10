import fs from 'node:fs';
import path from 'node:path';
import os from 'node:os';
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { pathToFileURL } from 'node:url';

const root = process.env.OET_COST_SOURCE_ROOT ?? '/Users/macpiek/.codex/worktrees/901c/OpenEndovascularTrainer';
const output = process.argv[2] ?? new URL('./split-step-transaction-cost.json', import.meta.url).pathname;
const freeze = fs.mkdtempSync('/tmp/oet-split-transaction-cost-');
const digest = value => createHash('sha256').update(value).digest('hex');
function walk(dir) {
    return fs.readdirSync(path.join(root, dir), { withFileTypes: true }).flatMap(e =>
        e.isDirectory() ? walk(path.join(dir, e.name)) : /\.(?:js|json)$/.test(e.name) ? [path.join(dir, e.name)] : []);
}
const files = [...walk('src'), 'tests/helpers/coupledRuntimeFixture.js', 'package.json', 'package-lock.json'].sort();
const hashes = () => Object.fromEntries(files.map(p => [p, digest(fs.readFileSync(path.join(root, p)))]));
const before = hashes();
for (const p of files) {
    const destination = path.join(freeze, p);
    fs.mkdirSync(path.dirname(destination), { recursive: true });
    fs.copyFileSync(path.join(root, p), destination);
    assert.equal(digest(fs.readFileSync(destination)), before[p]);
}
fs.symlinkSync(path.join(root, 'node_modules'), path.join(freeze, 'node_modules'));
const load = p => import(pathToFileURL(path.join(freeze, p)));
const { createCoupledRuntimeFixture } = await load('tests/helpers/coupledRuntimeFixture.js');
const { EndovascularPhysicsWorld } = await load('src/physics/endovascularPhysicsWorld.js');
const { createCoupledSolverSelection } = await load('src/physics/coupledSolverSelection.js');
const { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } = await load('src/physics/kirchhoffCoupledSystem.js');
const { configureKirchhoffSplitBias } = await load('src/physics/kirchhoffSplitMotion.js');
const { captureKirchhoffSplitStep: capture, restoreKirchhoffSplitStep: restore } = await load('src/physics/kirchhoffSplitStepTransaction.js');
const { captureKirchhoffCoupledTrialState: captureTrial } = await load('src/physics/kirchhoffCoupledTrialState.js');
const { buildKirchhoffCoupledFrictionRows, measureKirchhoffCoupledFrictionResidual } = await load('src/physics/kirchhoffCoupledFrictionRows.js');
const report = {
    scope: 'Read-only source audit. Native runtime first coupled step on procedural vessel, no anatomy field; synthetic stored full-active contact graph, no trajectory replay.',
    root, freeze, measuredAt: new Date().toISOString(), node: process.version,
    cpu: os.cpus()[0].model, sourceBefore: before, probeHash: digest(fs.readFileSync(new URL(import.meta.url))),
    timingProtocol: '3 warmup + 17 samples. Fresh whole-dt snapshot each time, separate capture/restore timing. No explicit GC. Previous local trial cache descriptor is reinstated outside timings so identical prepared input is used. Restore invalidates numerical caches; no solve interleaved. Median/p95 from samples, cold sample separate. No heap-byte estimate; bytes means owned typed-array byte copies.',
    states: [], solveCalls: [],
};
const stat = values => {
    const s = [...values].sort((a, b) => a - b);
    return { median: s[Math.floor(s.length / 2)], p95: s[Math.ceil(s.length * .95) - 1], minimum: s[0], maximum: s.at(-1) };
};
const metrics = records => ({ objects: records.length,
    bytes: records.reduce((n, r) => n + (r.copy?.byteLength ?? 0), 0),
    properties: records.reduce((n, r) => n + (r.keys?.length ?? 0), 0),
    kinds: records.reduce((n, r) => (n[r.kind] = (n[r.kind] ?? 0) + 1, n), {}) });

function graph(snapshot, world) {
    const trial = snapshot.trials[0], records = trial.records, byObject = new Map(records.map(r => [r.object, r]));
    const children = r => r.kind === 'object' ? r.dataDescriptors.map(d => d.value) :
        r.kind === 'map' ? r.entries.map(e => e[1]) : r.kind === 'set' ? r.entries : [];
    function reachable(object, seen = new Set(), force = false) {
        if ((!force && trial._barriers.has(object)) || seen.has(object)) return seen;
        const r = byObject.get(object); if (!r) return seen;
        seen.add(object); for (const child of children(r)) reachable(child, seen);
        return seen;
    }
    const count = seen => metrics([...seen].map(o => byObject.get(o)));
    const c = world.containments[0], attributed = [], seen = new Set();
    function row(label, object, force = false) {
        const old = new Set(seen); reachable(object, seen, force);
        const fresh = [...seen].filter(o => !old.has(o));
        const data = metrics(fresh.map(o => byObject.get(o)));
        if (data.objects) attributed.push({ label, ...data });
    }
    world.bodies.forEach((b, i) => row('body[' + i + ']', b, true));
    seen.add(c);
    for (const key of Object.keys(c)) row('joint.' + key, c[key]);
    world.toolContacts.forEach((o, i) => row('tool[' + i + ']', o));
    world.sheaths.forEach((o, i) => row('sheath[' + i + '].lambdas', o.lambdas));
    row('world', world, true);
    const split = reachable(c._splitMotion), accepted = reachable(c._acceptedPhysicalMotion);
    const both = new Set([...split, ...accepted]);
    const arrays = records.filter(r => r.kind === 'bytes');
    const buffers = new Map();
    for (const r of arrays) {
        const list = buffers.get(r.view.buffer) ?? [];
        list.push([r.view.byteOffset, r.view.byteOffset + r.view.byteLength]); buffers.set(r.view.buffer, list);
    }
    let uniqueBytes = 0;
    for (const intervals of buffers.values()) {
        intervals.sort((a, b) => a[0] - b[0]); let lo = -1, hi = -1;
        for (const [a, b] of intervals) {
            if (a > hi) { if (hi >= 0) uniqueBytes += hi - lo; lo = a; hi = b; } else hi = Math.max(hi, b);
        }
        if (hi >= 0) uniqueBytes += hi - lo;
    }
    const bodies = world.bodies.map(b => {
        const typed = Object.entries(b).filter(([, v]) => ArrayBuffer.isView(v));
        return { id: b.id, allocatedNodes: b.count, activeStart: b.activeStart, activeEnd: b.activeEnd,
            activeNodes: b.activeEnd - b.activeStart + 1,
            directTypedArrayBytes: typed.reduce((n, [, v]) => n + v.byteLength, 0),
            largestArrays: typed.map(([key, v]) => ({ key, length: v.length, bytes: v.byteLength }))
                .sort((a, b) => b.bytes - a.bytes).slice(0, 8) };
    });
    return { ...metrics(records), trialCount: snapshot.trials.length,
        bodySlotProperties: snapshot.bodySlots.reduce((n, s) => n + Reflect.ownKeys(s.descriptors).length, 0),
        sheathSlotProperties: snapshot.sheathSlots.reduce((n, s) => n + Reflect.ownKeys(s.descriptors).length, 0),
        bodies, uniqueBackingBufferBytes: uniqueBytes, overlappingViewDuplicateBytes: trial.bytes - uniqueBytes,
        previousSplitReachable: count(split), acceptedMotionReachable: count(accepted), historyUnionReachable: count(both),
        historySharedObjects: [...split].filter(o => accepted.has(o)).length,
        surfaceMotionAliasesSplit: c.surfaceMotion === c._splitMotion && !!c._splitMotion,
        topOwnership: attributed.sort((a, b) => b.properties - a.properties).slice(0, 30),
        largestTypedArrays: arrays.map(r => ({ bytes: r.copy.byteLength, type: r.object.constructor.name,
            bodyKey: world.bodies.flatMap(b => Object.entries(b).filter(([, v]) => v === r.object).map(([k]) => b.id + '.' + k)).join(',') }))
            .sort((a, b) => b.bytes - a.bytes).slice(0, 10),
    };
}

function signature(snapshot) {
    const ids = new Map(); let id = 0;
    const atom = value => typeof value === 'object' && value !== null || typeof value === 'function'
        ? (ids.has(value) || ids.set(value, ++id), '#' + ids.get(value)) : String(value);
    const hash = createHash('sha256');
    for (const r of snapshot.trials[0].records) {
        hash.update(r.kind + atom(r.object));
        if (r.kind === 'bytes') hash.update(r.copy);
        if (r.kind === 'object') for (const k of r.keys) {
            if (k === '_jointTrialState') continue;
            const d = r.descriptors[k];
            hash.update(k + JSON.stringify([atom(d.value), d.writable, d.enumerable, d.configurable, atom(d.get), atom(d.set)]));
        }
        if (r.kind === 'map') for (const [k, v] of r.entries) hash.update(atom(k) + ':' + atom(v));
        if (r.kind === 'set') for (const v of r.entries) hash.update(atom(v));
    }
    return hash.digest('hex');
}

function measure(label, world, progress = null) {
    const descriptors = world.containments.map(c => Object.getOwnPropertyDescriptor(c, '_jointTrialState'));
    const reinstateLocalCaches = () => world.containments.forEach((c, i) => {
        if (descriptors[i]) Object.defineProperty(c, '_jointTrialState', descriptors[i]); else delete c._jointTrialState;
    });
    let start = performance.now(); const cold = capture(world); const coldCaptureMs = performance.now() - start;
    const inputSignature = signature(cold), graphStats = graph(cold, world);
    start = performance.now(); restore(world, cold); const coldRestoreMs = performance.now() - start;
    reinstateLocalCaches();
    const samples = [];
    for (let i = -3; i < 17; i++) {
        start = performance.now(); const saved = capture(world); const captureMs = performance.now() - start;
        start = performance.now(); restore(world, saved); const restoreMs = performance.now() - start;
        reinstateLocalCaches(); if (i >= 0) samples.push({ captureMs, restoreMs });
    }
    const after = capture(world); const outputSignature = signature(after);
    assert.equal(outputSignature, inputSignature, 'Benchmark restores must preserve captured mechanical state');
    const state = { label, progress, graph: graphStats, inputSignature, outputSignature,
        mechanicalStateStable: true, coldCaptureMs, coldRestoreMs,
        captureMs: stat(samples.map(s => s.captureMs)), restoreMs: stat(samples.map(s => s.restoreMs)), samples };
    report.states.push(state); return state;
}

// Capture-only ablation is an estimate, NOT a valid rollback implementation.
function historyAblation(world) {
    const c = world.containments[0], external = world.containments.map(x => x._jointTrialState).filter(Boolean);
    const variants = [
        ['current-trial-part', {}, external],
        ['barrier-previous-phase-only', {}, [...external, c._splitMotion].filter(Boolean)],
        ['barrier-accepted-only', {}, [...external, c._acceptedPhysicalMotion].filter(Boolean)],
        ['barrier-both', {}, [...external, c._splitMotion, c._acceptedPhysicalMotion].filter(Boolean)],
    ];
    return variants.map(([label, options, external]) => {
        const samples = []; let last;
        for (let i = -3; i < 17; i++) {
            const start = performance.now(); last = captureTrial(c, { world, ...options, external });
            const elapsed = performance.now() - start; if (i >= 0) samples.push(elapsed);
        }
        return { label, ...metrics(last.records), captureMs: stat(samples) };
    });
}

const selection = createCoupledSolverSelection('joint-active-coulomb', {
    solve(c, dt, options) {
        const start = performance.now(); const result = solveKirchhoffCoupledSystem(c, dt, options);
        report.solveCalls.push({ phase: c._splitMotion?.phase, ms: performance.now() - start,
            factorizations: result.diagnostics.factorizations, rowCount: result.diagnostics.rowCount,
            status: result.diagnostics.status });
        return result;
    }, apply: applyKirchhoffCoupledCorrection,
});
const fixture = createCoupledRuntimeFixture({ coupledSystem: selection.coupledSystem, jointMotionMode: 'split-physical-bias' });
configureKirchhoffSplitBias(fixture.containment, { materialMode: 'preserve-strain' });
let coupled = false;
const nativeStep = fixture.world.stepFixed.bind(fixture.world);
fixture.world.stepFixed = function () {
    if (!coupled && fixture.containment.enabled) {
        coupled = true;
        measure('runtime-before-first-coupled-step', this, fixture.snapshot());
        const start = performance.now(); const result = nativeStep();
        report.firstCoupledStep = { wallMs: performance.now() - start, accepted: result?.accepted,
            status: result?.status, worldStepCount: this.stepCount, timedTotalMs: this.timings.total.last,
            solveMs: report.solveCalls.reduce((n, r) => n + r.ms, 0), solveCalls: report.solveCalls.length,
            physicalPasses: result?.diagnostics?.physicalPasses, biasPasses: result?.diagnostics?.biasPasses };
        measure('runtime-after-first-coupled-step', this, fixture.snapshot());
        report.historyAblation = historyAblation(this);
        return result;
    }
    return nativeStep();
};
try {
    for (let i = 0; i < 33; i++) fixture.step({ guidewireAdvance: 1 });
    for (let i = 0; i < 21 && !coupled; i++) fixture.step({ catheterAdvance: 1 });
    assert.ok(coupled, 'Bounded first-coupled preparation should enable a joint');
} finally { fixture.dispose(); }

function synthetic(contactCount) {
    const world = new EndovascularPhysicsWorld();
    const inner = world.createRod('synthetic-wire', 201, 5, { radius: .4445 });
    const outer = world.createRod('synthetic-catheter', 320, 5, { radius: .8 });
    for (const b of [inner, outer]) for (let i = 0; i < b.count; i++) b.setNodePosition(i, i * 5, b === inner ? .0405 : 0, 0);
    const c = world.addContainment(inner, outer, { innerRadius: .485, axialFriction: .2 });
    world.addToolContact(inner, outer, { enabled: false });
    for (let i = 0; i < contactCount; i++) {
        const innerSegment = i % inner.segmentCount, outerSegment = i % outer.segmentCount;
        const contact = c.manifold.upsertContact({ id: 'synthetic-' + i, innerMaterialSegmentId: i, outerMaterialSegmentId: i,
            innerSegmentIndex: innerSegment, outerSegmentIndex: outerSegment, normal: [0, 1, 0], tangentU: [1, 0, 0] });
        contact.normalLambda = 1; contact.tangentLambda.set([.01, .001]);
        c.kirchhoffContacts.push({ id: 'row-' + i, kind: 'side', gap: 0, normal: [0, 1, 0],
            _innerSegmentIndex: innerSegment, _outerSegmentIndex: outerSegment,
            innerT: .5, outerT: .5, innerWeights: [.5, .5], outerWeights: [.5, .5], manifoldContact: contact });
    }
    c._kirchhoffRuntimeRecords = [...c.kirchhoffContacts];
    c._jointFrictionBatch = buildKirchhoffCoupledFrictionRows(c, world.fixedDt);
    c._jointFrictionResidual = measureKirchhoffCoupledFrictionResidual(c, world.fixedDt);
    c._jointOptions = { additionalRows: [...c._jointFrictionBatch.rows], groups: [...c._jointFrictionBatch.groups] };
    return world;
}
for (const count of [2, 479]) {
    const world = synthetic(count);
    measure('synthetic-full-active-' + count + '-contacts', world);
    if (count === 2) {
        world.bodies[0].setActiveRange(197, 200); world.bodies[1].setActiveRange(0, 17);
        measure('synthetic-same-storage-active-4-and-18-no-solve', world);
    }
}
report.sourceAfter = hashes();
report.sourceStable = JSON.stringify(report.sourceAfter) === JSON.stringify(report.sourceBefore);
assert.ok(report.sourceStable, 'Root source changed during audit; frozen measurements remain valid but must be relabeled');
fs.writeFileSync(output, JSON.stringify(report, null, 2) + '\n');
fs.writeFileSync(path.join(freeze, 'manifest.json'), JSON.stringify({ root, hashes: before, probeHash: report.probeHash }, null, 2) + '\n');
console.log(JSON.stringify({ output, freeze, sourceStable: report.sourceStable, firstCoupledStep: report.firstCoupledStep,
    states: report.states.map(s => ({ label: s.label, bytes: s.graph.bytes, objects: s.graph.objects,
        properties: s.graph.properties, captureMs: s.captureMs.median, restoreMs: s.restoreMs.median,
        previousSplit: s.graph.previousSplitReachable, acceptedMotion: s.graph.acceptedMotionReachable })) }, null, 2));

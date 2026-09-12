import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { writeFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const defaultCandidate = fileURLToPath(new URL('../../src/physics/kirchhoffCoupledTrialState.js', import.meta.url));
const load = path => import(pathToFileURL(resolve(path)));
const percentile = (values, p) => [...values].sort((a, b) => a - b)[Math.ceil(p * values.length) - 1];
function summary(times) {
    return Object.fromEntries(['captureMs', 'restoreMs'].map(key => [key, {
        median: percentile(times.map(t => t[key]), .5),
        p95: percentile(times.map(t => t[key]), .95)
    }]));
}

/** Run only in an agreed measurement window. Both implementations receive the
 * SAME live runtime graph. No physics solve/geometry refresh occurs in timed
 * regions. All snapshots are barriers so neither captures the other's cache.
 * The optional reference-change case swaps equivalent normal-array storage;
 * it exercises discovery without changing a numerical contact value. */
export async function measureTrialState(world, constraint, {
    baselinePath, candidatePath = defaultCandidate, iterations = 30, warmup = 8,
    includeReferenceChange = true
} = {}) {
    if (!baselinePath) throw Error('An explicit frozen baselinePath is required');
    const baseline = await load(baselinePath), candidate = await load(candidatePath);
    const variants = [
        { name: 'baseline', implementation: baseline, snapshot: {} },
        { name: 'candidate', implementation: candidate, snapshot: {} }
    ];
    const options = { world, reusePropertyLayout: true, frozenFrictionBatches: true,
        external: [constraint._jointTrialState, ...variants.map(v => v.snapshot)].filter(Boolean) };
    for (const v of variants) v.implementation.captureKirchhoffCoupledTrialState(constraint, options, v.snapshot);
    const expected = variants[0].snapshot;
    function verifyMembership() {
        const actual = variants[1].snapshot;
        assert.equal(actual.objectCount, expected.objectCount);
        assert.equal(actual.bytes, expected.bytes);
        const members = new Set(expected.records.map(r => r.object));
        for (const r of actual.records) assert.ok(members.has(r.object), 'Captured graph membership differs');
    }
    verifyMembership();
    const identities = new Map();
    function identity(value) {
        if ((typeof value === 'object' && value !== null) || typeof value === 'function') {
            if (!identities.has(value)) identities.set(value, identities.size);
            return ['reference', identities.get(value)];
        }
        return [typeof value, typeof value === 'number' && Object.is(value, -0) ? '-0' : String(value)];
    }
    function fullHash(snapshot) {
        const hash = createHash('sha256');
        const add = value => hash.update(JSON.stringify(value) + '\n');
        for (const record of snapshot.records) {
            add([identity(record.object), record.kind]);
            if (record.kind === 'bytes') hash.update(new Uint8Array(record.object.buffer, record.object.byteOffset, record.object.byteLength));
            else if (record.kind === 'map') for (const [key, value] of record.object) add([identity(key), identity(value)]);
            else if (record.kind === 'set') for (const value of record.object) add(identity(value));
            else for (const key of Object.getOwnPropertyNames(record.object)) {
                const d = Object.getOwnPropertyDescriptor(record.object, key);
                if (record.filter && !record.filter(key, d.value)) continue;
                add([key, d.writable, d.enumerable, d.configurable,
                    'value' in d ? identity(d.value) : [identity(d.get), identity(d.set)]]);
            }
        }
        return hash.digest('hex');
    }
    const rollbackHashes = [];
    for (const { name, implementation, snapshot } of variants) {
        implementation.captureKirchhoffCoupledTrialState(constraint, options, snapshot);
        const before = fullHash(snapshot);
        constraint.innerBody.x[constraint.innerBody.activeStart] += 1;
        const contact = constraint.kirchhoffContacts[0];
        if (contact) { contact.gap += .125; contact.manifoldContact.tangentLambda[0] += .01; }
        const rejected = fullHash(snapshot);
        assert.notEqual(rejected, before);
        implementation.restoreKirchhoffCoupledTrialState(snapshot);
        const after = fullHash(snapshot);
        assert.equal(after, before, 'Rollback must restore the entire captured state');
        rollbackHashes.push({ name, before, rejected, after });
    }
    const results = [];
    const record = constraint.kirchhoffContacts.find(r => ArrayBuffer.isView(r.normal));
    const originalNormal = record?.normal;
    const replacements = originalNormal && [originalNormal.slice(), originalNormal.slice()];
    try {
        for (const mode of includeReferenceChange && record ? ['steady', 'changed-reference'] : ['steady']) {
            const timings = variants.map(() => []);
            let firstStructuralChange = null;
            if (mode === 'changed-reference') {
                record.normal = replacements[0];
                firstStructuralChange = [];
                for (const { name, implementation, snapshot } of variants) {
                    const began = performance.now();
                    implementation.captureKirchhoffCoupledTrialState(constraint, options, snapshot);
                    const middle = performance.now();
                    implementation.restoreKirchhoffCoupledTrialState(snapshot);
                    firstStructuralChange.push({ name, captureMs: middle - began, restoreMs: performance.now() - middle });
                }
                verifyMembership();
            }
            for (let i = -warmup; i < iterations; i++) {
                if (mode === 'changed-reference') record.normal = replacements[(i + warmup) % 2];
                // Alternate A/B order to avoid systematic warm-cache advantage.
                for (const index of i % 2 ? [1, 0] : [0, 1]) {
                    const { implementation, snapshot } = variants[index];
                    const began = performance.now();
                    implementation.captureKirchhoffCoupledTrialState(constraint, options, snapshot);
                    const middle = performance.now();
                    implementation.restoreKirchhoffCoupledTrialState(snapshot);
                    const end = performance.now();
                    if (i >= 0) timings[index].push({ captureMs: middle - began, restoreMs: end - middle });
                }
                verifyMembership();
            }
            results.push({ mode, objects: expected.objectCount, bytes: expected.bytes, firstStructuralChange,
                variants: variants.map((v, i) => ({ name: v.name, ...summary(timings[i]), samples: timings[i] })) });
        }
    } finally {
        if (record) record.normal = originalNormal;
    }
    return { baselinePath: resolve(baselinePath), candidatePath: resolve(candidatePath), iterations, warmup, rollbackHashes,
        notes: 'Same live graph, alternating A/B, unchanged physics. Steady state and equivalent reference replacement. Timings exclude preparation and membership verification. Restore invalidates numerical factor caches as in production.',
        results };
}

// Explicit bounded preparation; never run this CLI concurrently with root's
// replay. Imports can instead call measureTrialState on an already-live state.
if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
    const args = Object.fromEntries(process.argv.slice(2).map(arg => {
        const separator = arg.indexOf('='); return [arg.slice(2, separator), arg.slice(separator + 1)];
    }));
    if (!args.root || !args.baseline || args['through-mm'] !== '100' || !args.output)
        throw Error('Use --root=PATH --baseline=PATH --through-mm=100 --output=PATH in an agreed window');
    const root = resolve(args.root), source = path => load(resolve(root, path));
    const { createCoupledRuntimeFixture, loadCoupledRuntimeAnatomy } = await source('tests/helpers/coupledRuntimeFixture.js');
    const { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } = await source('src/physics/kirchhoffCoupledSystem.js');
    const { DEEP_CATHETER_BENCHMARK_PHASES: phases, sampleShortCatheterBenchmarkCommands } = await source('src/benchmark/shortCatheterBenchmark.js');
    const anatomy = await loadCoupledRuntimeAnatomy(pathToFileURL(root + '/'));
    const fixture = createCoupledRuntimeFixture(anatomy), world = fixture.world;
    world.coupledSystem = {
        solve: (c, dt, options) => solveKirchhoffCoupledSystem(c, dt,
            { ...options, activeCondensation: true, simultaneousCoulomb: true }),
        apply: applyKirchhoffCoupledCorrection
    };
    let step = 0; const command = {};
    try {
        for (let phase = 0; phase <= 3; phase++) {
            for (let i = 0; i < phases[phase].steps; i++, step++) {
                sampleShortCatheterBenchmarkCommands(step * 1000 / 120, command, phases);
                fixture.step(command);
                if (phase >= 3 && !world.lastCoupledClosureConverged)
                    throw Error('Prefix nonconvergence: ' + JSON.stringify(fixture.snapshot()));
            }
        }
        assert.ok(Math.abs(fixture.catheter.progress - 100) < 1e-8);
        const result = await measureTrialState(world, fixture.containment, { baselinePath: args.baseline });
        result.state = fixture.snapshot();
        writeFileSync(args.output, JSON.stringify(result, null, 2) + '\n');
        console.log(JSON.stringify({ output: args.output, state: result.state,
            results: result.results.map(r => ({ ...r, variants: r.variants.map(({ samples, ...v }) => v) })) }));
    } finally { fixture.dispose(); anatomy.dispose(); }
}

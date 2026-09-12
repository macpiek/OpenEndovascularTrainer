import assert from 'node:assert/strict';
import { readFileSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { resolve, dirname, relative } from 'node:path';
import { fileURLToPath } from 'node:url';
import { makeFixture, preparedOptions, advance } from './helpers/compositeTimeStepBenchmark.js';
import { createCompositeTimeStepWorkspace } from '../../src/physics/kirchhoffCompositeTimeStep.js';

const root = fileURLToPath(new URL('../../', import.meta.url));
const output = process.argv[2] ?? resolve(root, 'reports/composite-timestep-workspace.json');
const hash = value => createHash('sha256').update(typeof value === 'string' ? value : JSON.stringify(value)).digest('hex');
function sources(file = fileURLToPath(import.meta.url), result = {}) {
    const name = relative(root, file);
    if (name in result) return result;
    const source = readFileSync(file, 'utf8'); result[name] = hash(source);
    for (const match of source.matchAll(/(?:from\s*|import\s*)['"]([^'"]+)['"]/g)) {
        if (match[1].startsWith('.')) sources(resolve(dirname(file), match[1]), result);
    }
    return result;
}
const percentile = (values, fraction) => [...values].sort((a, b) => a - b)[Math.min(values.length - 1, Math.ceil(fraction * values.length) - 1)];
const summary = values => ({ count: values.length, medianMs: percentile(values, .5),
    meanMs: values.reduce((a, b) => a + b, 0) / values.length, minimumMs: Math.min(...values), maximumMs: Math.max(...values) });
const beforeSources = sources();
const report = { sourceBefore: beforeSources, node: process.version,
    scope: 'Paired full fixed-topology dt, fresh numeric scratch vs persistent numeric scratch. Same real material profiles, state/input/geometry per pair, strict force/torque/length/wall gates. No feed/remesh, finite-clearance/lumen/friction or browser FPS claim.',
    protocol: { warmupSteps: 3, measuredConsecutiveSteps: 8, ordering: 'alternating paired arms',
        timing: 'advance call including all per-dt copying, inertia compilation, nonlinear iterations, fresh acceptance and output ownership; reusable workspace creation separately measured and not free',
        cold: 'first pair separately recorded; JIT and host interference make it diagnostic, not a stable runtime percentile' }, cases: [] };

for (const elementBackend of ['wasm', 'wasm-exact']) for (const scenario of ['contact-free', 'analytic-plane']) for (const insertion of [9, 160, 310]) {
    const f = makeFixture(insertion, scenario), start = performance.now();
    const workspace = createCompositeTimeStepWorkspace(f.state.layout, { elementBackend });
    const row = { elementBackend, scenario, catheterMm: insertion, nodes: f.state.layout.nodeCount,
        workspaceCreationMs: performance.now() - start, cold: null, samples: [] };
    let state = f.state;
    for (let step = 0; step < 12; step++) {
        const options = { ...preparedOptions(f, state, 'strict-tests'), constraintSolver: 'mixed', elementBackend };
        const inputHash = hash({ state, options }), order = step % 2 ? ['reuse', 'fresh'] : ['fresh', 'reuse'];
        const pair = {};
        for (const arm of order) {
            const begin = performance.now();
            const result = advance(state, arm === 'reuse' ? { ...options, workspace } : options);
            const elapsedMs = performance.now() - begin;
            pair[arm] = { result, elapsedMs };
        }
        assert.equal(hash({ state, options }), inputHash, 'prepared input is unchanged');
        assert.deepEqual(pair.reuse.result.state, pair.fresh.result.state, 'all accepted state values and material histories are identical');
        const { reusedWorkspace: _a, ...a } = pair.reuse.result.diagnostics;
        const { reusedWorkspace: _b, ...b } = pair.fresh.result.diagnostics;
        assert.deepEqual(a, b, 'original equations and all solver work counts are identical');
        assert.equal(pair.reuse.result.accepted, pair.fresh.result.accepted);
        assert.equal(pair.reuse.result.status, pair.fresh.result.status);
        const sample = { step, inputHash, outputHash: hash(pair.reuse.result.state), freshMs: pair.fresh.elapsedMs,
            reuseMs: pair.reuse.elapsedMs, evaluations: a.evaluations, directions: a.directions,
            linearSolves: a.linearSolves, certificate: a.certificate, workspace: workspace.diagnostics,
            accepted: pair.reuse.result.accepted, status: pair.reuse.result.status };
        if (step === 0) row.cold = sample;
        if (!pair.reuse.result.accepted) { row.failure = sample; break; }
        if (step >= 4) row.samples.push(sample);
        state = pair.reuse.result.state;
    }
    row.fresh = row.samples.length ? summary(row.samples.map(s => s.freshMs)) : null;
    row.reuse = row.samples.length ? summary(row.samples.map(s => s.reuseMs)) : null;
    row.speedup = row.fresh === null ? null : row.fresh.medianMs / row.reuse.medianMs;
    report.cases.push(row);
    console.log(JSON.stringify({ backend: elementBackend, scenario, insertion, nodes: row.nodes,
        fresh: row.fresh?.medianMs, reuse: row.reuse?.medianMs, speedup: row.speedup, failure: row.failure ?? null,
        directions: row.samples.map(s => s.directions) }));
}
report.sourceAfter = sources(); assert.deepEqual(report.sourceAfter, beforeSources, 'executed source graph stayed unchanged');
report.allPairsEquivalent = true;
report.allStepsAccepted = report.cases.every(row => !row.failure);
writeFileSync(output, JSON.stringify(report, null, 2) + '\n');

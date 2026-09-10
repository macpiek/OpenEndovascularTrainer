import assert from 'node:assert/strict';
import { readFileSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { resolve, dirname, relative } from 'node:path';
import { fileURLToPath } from 'node:url';
import { makeFixture, preparedOptions, advance } from './helpers/compositeTimeStepBenchmark.js';
import { createCompositeTimeStepWorkspace } from '../../src/physics/kirchhoffCompositeTimeStep.js';

const root = fileURLToPath(new URL('../../', import.meta.url));
const output = process.argv[2] ?? resolve(root, 'reports/composite-timestep-gradient.json');
const hash = value => createHash('sha256').update(typeof value === 'string' ? value : JSON.stringify(value)).digest('hex');
function sources(file = fileURLToPath(import.meta.url), result = {}) {
    const name = relative(root, file); if (name in result) return result;
    const source = readFileSync(file, 'utf8'); result[name] = hash(source);
    for (const match of source.matchAll(/(?:from\s*|import\s*)['"]([^'"]+)['"]/g))
        if (match[1].startsWith('.')) sources(resolve(dirname(file), match[1]), result);
    return result;
}
function summary(values) {
    if (!values.length) return null;
    const sorted = [...values].sort((a, b) => a - b);
    return { count: values.length, medianMs: .5 * (sorted[Math.floor((sorted.length - 1) / 2)] + sorted[Math.floor(sorted.length / 2)]),
        meanMs: values.reduce((a, b) => a + b, 0) / values.length, minimumMs: sorted[0], maximumMs: sorted.at(-1) };
}
const report = { sourceBefore: sources(), node: process.version,
    scope: 'Matched full fixed-topology dt: full Exact tangent every evaluation, gradient-only final certificate, and lazy trial Exact tangent. Each arm retains its own numeric workspace. Identical prepared state/materials/loads and strict original gates. No feed/remesh, lumen/clearance/friction, anatomy or browser FPS claim.',
    protocol: { coldPairs: 1, warmupPairs: 3, measuredConsecutivePairs: 8, order: 'alternating',
        timing: 'whole advance call includes material/inertia preparation, all original residual/contact checks, nonlinear trials, tangent rebuilds and owned output; workspace creation is separately timed',
        budget: 'every constitutive gradient/full assembly consumes budget.evaluations, including lazy tangent rebuilds' }, cases: [] };
for (const scenario of ['contact-free', 'analytic-plane']) for (const insertion of [9, 160, 310]) {
    const f = makeFixture(insertion, scenario), workspaces = {}, setup = {};
    for (const arm of ['full', 'certificate', 'lazy']) {
        const start = performance.now(); workspaces[arm] = createCompositeTimeStepWorkspace(f.state.layout, { elementBackend: 'wasm-exact' });
        setup[arm] = performance.now() - start;
    }
    const row = { scenario, catheterMm: insertion, nodes: f.state.layout.nodeCount, workspaceCreationMs: setup, cold: null, samples: [] };
    let state = f.state;
    for (let step = 0; step < 12; step++) {
        const opts = { ...preparedOptions(f, state, 'strict-tests'), constraintSolver: 'mixed', elementBackend: 'wasm-exact' };
        const inputHash = hash({ state, opts }), pair = {};
        const arms = ['full', 'certificate', 'lazy'], offset = step % arms.length;
        for (const arm of arms.slice(offset).concat(arms.slice(0, offset))) {
            const cpu = process.cpuUsage(), thread = process.threadCpuUsage(), start = performance.now();
            const result = advance(state, { ...opts, workspace: workspaces[arm], assemblyPolicy: arm === 'certificate' ? 'auto' : arm });
            const ms = performance.now() - start, used = process.cpuUsage(cpu), main = process.threadCpuUsage(thread);
            pair[arm] = { result, ms, cpuMs: (used.user + used.system) / 1000, mainCpuMs: (main.user + main.system) / 1000 };
        }
        assert.equal(hash({ state, opts }), inputHash);
        for (const arm of ['certificate', 'lazy']) {
            assert.equal(pair.full.result.accepted, pair[arm].result.accepted);
            assert.equal(pair.full.result.status, pair[arm].result.status); assert.deepEqual(pair.full.result.state, pair[arm].result.state);
        }
        const a = pair.full.result.diagnostics, b = pair.lazy.result.diagnostics;
        const omitted = new Set(['evaluations', 'fullAssemblies', 'gradientAssemblies', 'tangentRebuilds', 'certificateAssembly', 'trialAssembly']);
        for (const arm of ['certificate', 'lazy']) for (const key of Object.keys(a)) if (!omitted.has(key))
            assert.deepEqual(a[key], pair[arm].result.diagnostics[key], key);
        const sample = { step, inputHash, outputHash: hash(pair.lazy.result.state), fullMs: pair.full.ms, lazyMs: pair.lazy.ms,
            fullCpuMs: pair.full.cpuMs, lazyCpuMs: pair.lazy.cpuMs, certificateMs: pair.certificate.ms,
            certificateCpuMs: pair.certificate.cpuMs, fullMainCpuMs: pair.full.mainCpuMs,
            certificateMainCpuMs: pair.certificate.mainCpuMs, lazyMainCpuMs: pair.lazy.mainCpuMs,
            accepted: pair.lazy.result.accepted, status: pair.lazy.result.status, directions: b.directions, linearSolves: b.linearSolves,
            assemblies: { full: { full: a.fullAssemblies, gradient: a.gradientAssemblies }, lazy: { full: b.fullAssemblies, gradient: b.gradientAssemblies },
                certificate: { full: pair.certificate.result.diagnostics.fullAssemblies, gradient: pair.certificate.result.diagnostics.gradientAssemblies } },
            certificate: b.certificate, fullWorkspace: workspaces.full.diagnostics, lazyWorkspace: workspaces.lazy.diagnostics };
        if (step === 0) row.cold = sample;
        if (!sample.accepted) { row.failure = sample; break; }
        if (step >= 4) row.samples.push(sample);
        state = pair.lazy.result.state;
    }
    row.full = summary(row.samples.map(s => s.fullMs)); row.lazy = summary(row.samples.map(s => s.lazyMs));
    row.fullCpu = summary(row.samples.map(s => s.fullCpuMs)); row.lazyCpu = summary(row.samples.map(s => s.lazyCpuMs));
    row.certificate = summary(row.samples.map(s => s.certificateMs)); row.certificateCpu = summary(row.samples.map(s => s.certificateCpuMs));
    row.fullMainCpu = summary(row.samples.map(s => s.fullMainCpuMs)); row.certificateMainCpu = summary(row.samples.map(s => s.certificateMainCpuMs));
    row.lazyMainCpu = summary(row.samples.map(s => s.lazyMainCpuMs));
    row.speedup = row.full ? row.full.medianMs / row.lazy.medianMs : null;
    report.cases.push(row);
    console.log(JSON.stringify({ scenario, insertion, full: row.full?.medianMs, certificate: row.certificate?.medianMs, lazy: row.lazy?.medianMs,
        mainCpu: [row.fullMainCpu?.medianMs, row.certificateMainCpu?.medianMs, row.lazyMainCpu?.medianMs], failure: row.failure ?? null }));
}
report.sourceAfter = sources(); assert.deepEqual(report.sourceAfter, report.sourceBefore);
report.allPairsEquivalent = true; report.allStepsAccepted = report.cases.every(row => !row.failure);
writeFileSync(output, JSON.stringify(report, null, 2) + '\n');

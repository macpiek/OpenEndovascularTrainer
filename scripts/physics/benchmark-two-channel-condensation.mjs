// A/B benchmark of the same frozen condensation, including owned recovery
// parity. This measures matrix assembly only, not a timestep or browser FPS.
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { createHash } from 'node:crypto';
import { gunzipSync } from 'node:zlib';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { condenseKirchhoffTwoChannelSystem as candidate } from '../../src/physics/kirchhoffTwoChannelCondensation.js';
import { solveCoulombNewton } from '../../src/physics/kirchhoffCoulombNewtonSolver.js';
import { measureCoupledLoadKKT } from '../../src/physics/kirchhoffCoupledLoadSolver.js';

const root = fileURLToPath(new URL('../../', import.meta.url)), args = process.argv.slice(2);
const option = (name, fallback) => { const i = args.indexOf(name); return i < 0 ? fallback : args[i + 1]; };
const baselinePath = path.resolve(option('--baseline', '/tmp/oet-condensation-response-baseline/kirchhoffTwoChannelCondensation.js'));
const output = path.resolve(option('--output', '/tmp/oet-two-channel-response-reuse.json'));
const pairs = Number(option('--pairs', 30)), warmup = Number(option('--warmup', 5));
assert.ok(Number.isInteger(pairs) && pairs > 0 && pairs <= 500);
assert.ok(Number.isInteger(warmup) && warmup >= 0 && warmup <= 100);
const { condenseKirchhoffTwoChannelSystem: baseline } = await import(pathToFileURL(baselinePath));
const fixturePath = path.join(root, 'tests/fixtures/kirchhoff-two-channel-condensation.json.gz');
const f = JSON.parse(gunzipSync(fs.readFileSync(fixturePath)),
    (_k, v) => v === 'Infinity' ? Infinity : v === '-Infinity' ? -Infinity : v);
const sourcePaths = [fileURLToPath(import.meta.url), fixturePath, baselinePath,
    ...['kirchhoffTwoChannelCondensation.js', 'kirchhoffLinearKernel.js', 'kirchhoffLinearKernelBytes.js',
        'kirchhoffCoulombNewtonSolver.js', 'kirchhoffCoupledLoadSolver.js'].map(n => path.join(root, 'src/physics', n)),
    ...['kirchhoffLinearKernel.js', 'kirchhoffLinearKernelBytes.js'].map(n => path.join(path.dirname(baselinePath), n))];
const hashes = () => Object.fromEntries(sourcePaths.map(p => [p, createHash('sha256').update(fs.readFileSync(p)).digest('hex')]));
const before = hashes(), samples = [];
let last;
for (let i = 0; i < pairs + warmup; i++) {
    const run = { i, warmup: i < warmup }, results = {};
    for (const [name, fn] of i % 2 ? [['candidate', candidate], ['baseline', baseline]] : [['baseline', baseline], ['candidate', candidate]]) {
        const start = performance.now();
        results[name] = fn(f.native, f.channels);
        run[name + 'Ms'] = performance.now() - start;
        assert.equal(results[name].status, 'condensed');
    }
    for (const key of ['matrix', 'rhs', 'lower', 'upper', 'groups', 'retainedRows', 'equalityOriginalRows'])
        assert.deepEqual(results.candidate[key], results.baseline[key], key);
    const direction = Float64Array.from({ length: results.candidate.count }, (_, j) => .01 * Math.sin(j + i));
    assert.deepEqual(results.candidate.recover(direction), results.baseline.recover(direction), 'owned response parity');
    samples.push(run); last = results;
}
const lower = [], upper = [], physical = [];
let count = 0;
f.native.order.forEach((original, sorted) => {
    physical[sorted] = count++; lower.push(f.native.lower[sorted]); upper.push(f.native.upper[sorted]);
    const b = f.channels[original].bias;
    if (b) { count++; lower.push(b.lower - b.lambda); upper.push(b.upper - b.lambda); }
});
function solveAndCertify(c) {
    const result = solveCoulombNewton(c.matrix, c.rhs, c.lower, c.upper, c.count, c.count, c.groups,
        { matrixFormat: 'row-major', tolerance: 2e-4 });
    assert.equal(result.diagnostics.converged, true);
    const recovered = c.recover(result.increment), x = recovered.fullIncrement;
    const residual = Float64Array.from(f.oracle.rhs, (v, i) => {
        for (let j = 0; j < count; j++) v -= f.oracle.matrix[i * count + j] * x[j]; return v;
    });
    const groups = f.native.groups.map(g => ({ ...g, rows: g.rows.map(i => physical[i]),
        normalRow: g.normalRow == null ? undefined : physical[g.normalRow],
        radii: g.normalRow == null ? g.radii : g.mu.map(mu => mu * Math.max(0, g.normalLambda + x[physical[g.normalRow]])) }));
    const kkt = measureCoupledLoadKKT(residual, x, lower, upper, groups);
    assert.ok(kkt.maximumResidual <= 2e-4);
    return { recovered, kkt };
}
const actual = solveAndCertify(last.candidate), old = solveAndCertify(last.baseline);
assert.deepEqual(actual, old, 'full original KKT and converged reaction parity');
const after = hashes(); assert.deepEqual(after, before, 'source stability');
const quantile = (values, p) => values.slice().sort((a, b) => a - b)[Math.ceil(p * values.length) - 1];
const timed = samples.filter(s => !s.warmup);
const statistics = Object.fromEntries(['baseline', 'candidate'].map(k => [k, {
    medianMs: quantile(timed.map(r => r[k + 'Ms']), .5), p95Ms: quantile(timed.map(r => r[k + 'Ms']), .95)
}]));
const report = { status: 'PASS', scope: 'One frozen 155-row system; condensation assembly only; no World, anatomy or FPS claim',
    sourceStable: true, sources: before, warmup, pairs, alternatingOrder: true,
    parity: 'exact matrix, RHS, bounds, groups, maps and recovered physical/bias reactions in every pair',
    statistics, assemblyMedianRatio: statistics.baseline.medianMs / statistics.candidate.medianMs,
    fullKKT: actual.kkt, work: { baseline: last.baseline.diagnostics, candidate: last.candidate.diagnostics }, samples };
fs.writeFileSync(output, JSON.stringify(report, null, 2) + '\n');
console.log(JSON.stringify({ output, status: report.status, statistics, assemblyMedianRatio: report.assemblyMedianRatio,
    equalitySolves: [last.baseline.diagnostics.equalitySolves, last.candidate.diagnostics.equalitySolves],
    fullKKT: { maximumResidual: report.fullKKT.maximumResidual, coneViolation: report.fullKKT.coneViolation,
        groupResidual: report.fullKKT.groupResidual } }, null, 2));

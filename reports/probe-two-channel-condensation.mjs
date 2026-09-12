// Frozen linear replay only. No World construction, integration or anatomy.
import fs from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { pathToFileURL } from 'node:url';
import assert from 'node:assert/strict';
const args = process.argv.slice(2), pos = args.indexOf('--source-root');
const root = pos < 0 ? new URL('../', import.meta.url) : pathToFileURL(args[pos + 1] + '/');
const [{ condenseKirchhoffTwoChannelSystem }, { solveCoulombNewton }, { measureCoupledLoadKKT }] = await Promise.all([
    import(new URL('src/physics/kirchhoffTwoChannelCondensation.js', root)), import(new URL('src/physics/kirchhoffCoulombNewtonSolver.js', root)),
    import(new URL('src/physics/kirchhoffCoupledLoadSolver.js', root))]);
const f = JSON.parse(gunzipSync(fs.readFileSync(new URL('../tests/fixtures/kirchhoff-two-channel-condensation.json.gz', import.meta.url))),
    (_key, v) => v === 'Infinity' ? Infinity : v === '-Infinity' ? -Infinity : v);
const options = { matrixFormat: 'row-major', tolerance: 2e-4 }, lower = [], upper = [], physical = [], bias = [];
let count = 0;
f.native.order.forEach((original, sorted) => {
    physical[sorted] = count++; lower.push(f.native.lower[sorted]); upper.push(f.native.upper[sorted]);
    const b = f.channels[original].bias;
    if (b) { bias[sorted] = count++; lower.push(b.lower - b.lambda); upper.push(b.upper - b.lambda); }
});
const groups = f.native.groups.map(g => ({ ...g, rows: g.rows.map(i => physical[i]), normalRow: g.normalRow == null ? undefined : physical[g.normalRow],
    lambda: [...g.lambda], radii: [...g.radii], ...(g.mu ? { mu: [...g.mu] } : {}) }));
function measure(increment) {
    const residual = Float64Array.from(f.oracle.rhs, (v, i) => {
        for (let j = 0; j < count; j++) v -= f.oracle.matrix[i * count + j] * increment[j]; return v;
    });
    const current = groups.map(g => ({ ...g, radii: g.normalRow == null ? g.radii : g.mu.map(mu => mu * Math.max(0, g.normalLambda + increment[g.normalRow])) }));
    const kkt = measureCoupledLoadKKT(residual, increment, lower, upper, current);
    let boundsViolation = 0;
    increment.forEach((v, i) => { boundsViolation = Math.max(boundsViolation, lower[i] - v, v - upper[i]); });
    assert.ok(kkt.maximumResidual <= options.tolerance && boundsViolation <= 1e-12, JSON.stringify(kkt));
    return { maximumResidual: kkt.maximumResidual, frictionResidual: kkt.groupResidual, coneViolation: kkt.coneViolation, boundsViolation };
}
const runs = [];
for (let run = 0; run < 4; run++) {
    const start = performance.now();
    const full = solveCoulombNewton(f.oracle.matrix, f.oracle.rhs, lower, upper, count, count, groups, options);
    const fullMs = performance.now() - start;
    assert.equal(full.diagnostics.converged, true, JSON.stringify(full.diagnostics));
    const begin = performance.now(), c = condenseKirchhoffTwoChannelSystem(f.native, f.channels);
    const assemblyMs = performance.now() - begin; assert.equal(c.status, 'condensed', JSON.stringify(c));
    const solveStart = performance.now();
    const small = solveCoulombNewton(c.matrix, c.rhs, c.lower, c.upper, c.count, c.count, c.groups, options);
    const solveMs = performance.now() - solveStart;
    assert.equal(small.diagnostics.converged, true, JSON.stringify(small.diagnostics));
    const recoveryStart = performance.now(), recovered = c.recover(small.increment), recoveryMs = performance.now() - recoveryStart;
    const fullKKT = measure(full.increment), recoveredKKT = measure(recovered.fullIncrement);
    const maxDifference = (a, b) => a.reduce((v, x, i) => Math.max(v, Math.abs(x - b[i])), 0);
    runs.push({ run, cold: run === 0, fullMs, assemblyMs, solveMs, recoveryMs, condensedTotalMs: assemblyMs + solveMs + recoveryMs,
        dense: { iterations: full.diagnostics.iterations, factors: full.diagnostics.factorizations, kkt: fullKKT },
        condensed: { iterations: small.diagnostics.iterations, factors: small.diagnostics.factorizations, kkt: recoveredKKT, ...c.diagnostics },
        maximumIncrementDifferenceVsFreshDense: maxDifference(recovered.fullIncrement, full.increment),
        maximumPhysicalIncrementDifferenceVsSavedOracle: maxDifference(recovered.physicalIncrement, f.oracle.physicalIncrement),
        maximumBiasIncrementDifferenceVsSavedOracle: maxDifference(recovered.biasIncrement, f.oracle.biasIncrement) });
}
const median = values => values.slice().sort((a, b) => a - b)[Math.floor(values.length / 2)], warm = runs.slice(1);
const summary = { status: 'PASS', scope: 'One saved linear system, first cold + 3 warmed paired solves; no anatomy/FPS claim', sourceRoot: root.href,
    tolerance: options.tolerance, nativeRows: f.native.count, fullRows: count, retainedRows: runs[0].condensed.retainedRows,
    warmDenseMedianMs: median(warm.map(r => r.fullMs)), warmCondensedMedianMs: median(warm.map(r => r.condensedTotalMs)), runs };
summary.warmSpeedRatio = summary.warmDenseMedianMs / summary.warmCondensedMedianMs;
fs.writeFileSync(new URL('./two-channel-condensation-frozen-comparison.json', import.meta.url), JSON.stringify(summary, null, 2));
console.log(JSON.stringify(summary, null, 2));

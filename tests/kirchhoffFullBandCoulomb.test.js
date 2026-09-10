import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { solveActiveCondensedCoupledQP } from '../src/physics/kirchhoffActiveCondensedSolver.js';
import { solveCoulombNewton, solveSeededCoulombNewton } from '../src/physics/kirchhoffCoulombNewtonSolver.js';
import { measureCoupledLoadKKT } from '../src/physics/kirchhoffCoupledLoadSolver.js';
import { auditFrozenCoupledSystem } from '../scripts/physics/audit-frozen-coupled-system.mjs';

const read = name => JSON.parse(gunzipSync(fs.readFileSync(new URL(`./fixtures/${name}.json.gz`, import.meta.url))),
    (_, value) => value === 'Infinity' ? Infinity : value === '-Infinity' ? -Infinity : value);
function audit(p, result) {
    // Full original symmetric operator, independent of Newton's compact row
    // layout, pivoting, and natural-map residual. Include all zero-load cones.
    const residual = Float64Array.from(p.rhs);
    for (let i = 0; i < p.count; i++) for (let j = Math.max(0, i - p.band + 1); j <= i; j++) {
        const a = p.matrix[i * p.band + i - j]; residual[i] -= a * result.increment[j];
        if (i !== j) residual[j] -= a * result.increment[i];
    }
    const groups = p.groups.map(g => ({ ...g, radii: g.normalRow == null ? g.radii :
        g.mu.map(mu => mu * Math.max(0, g.normalLambda + result.increment[g.normalRow])) }));
    const kkt = measureCoupledLoadKKT(residual, result.increment, p.lower, p.upper, groups);
    assert.ok(kkt.maximumResidual <= .0002, `original KKT ${kkt.maximumResidual}`);
    assert.ok(kkt.coneViolation <= 1e-9); assert.equal(result.allGroups.length, p.groups.length);
    result.increment.forEach((v, i) => assert.ok(Number.isFinite(v) && v >= p.lower[i] && v <= p.upper[i], `force ${i}`));
    residual.forEach((v, i) => assert.ok(Math.abs(v - result.residual[i]) < 1e-10 * Math.max(1, Math.abs(v)), `original residual ${i}`));
}

for (const [name, count] of [['kirchhoff-active-condensed-1999', 1999], ['kirchhoff-coupled-full-200', 2459]]) {
    test(`full band route keeps all ${count} original equations and cones without a material Schur complement`, () => {
        const p = read(name), args = ['matrix', 'rhs', 'lower', 'upper'].map(k => Float64Array.from(p[k]));
        const before = structuredClone(p);
        const options = { tolerance: .0002, numericalShift: 1e-8, initialFree: p.initialFree ?? new Uint8Array(p.count),
            simultaneousCoulomb: true, coulombStructure: 'full-band', workspace: {}, loadWorkspace: {}, frictionWorkspace: {} };
        const result = solveActiveCondensedCoupledQP(...args, p.count, p.band, p.groups, options);
        assert.ok(result.diagnostics.converged, JSON.stringify(result.diagnostics)); audit(p, result);
        assert.equal(result.diagnostics.rowCount, count); assert.equal(result.diagnostics.linearSolver, 'band-lu');
        assert.equal(result.diagnostics.coulombStructure, 'full-band'); assert.equal(result.diagnostics.responseColumns, undefined);
        assert.ok(result.diagnostics.jacobianEntries < count * count / 10);
        assert.ok(result.diagnostics.maximumLinearBackwardError < 1e-12);
        assert.equal(result.diagnostics.linearResidualFailures, 0);
        assert.deepEqual(p, before); args.forEach((array, i) => assert.deepEqual(array, Float64Array.from(p[['matrix', 'rhs', 'lower', 'upper'][i]])));
        const snapshot = result.increment.slice();
        const reused = solveActiveCondensedCoupledQP(...args, p.count, p.band, p.groups, options);
        assert.deepEqual(reused.increment, snapshot); assert.deepEqual(result.increment, snapshot);
        if (p.columns) {
            const certificate = auditFrozenCoupledSystem({ ...p, increment: result.increment });
            assert.ok(certificate.passed, JSON.stringify(certificate));
        }
    });
}

test('retained 144.2 mm band LU reproduces dense forces, residuals and bound recovery', () => {
    const p = read('kirchhoff-coulomb-144-bound-recovery'), args = ['matrix', 'rhs', 'lower', 'upper'].map(k => Float64Array.from(p[k]));
    const options = { ...p.options, tolerance: .0002, initialIncrement: p.initialIncrement };
    const dense = solveCoulombNewton(...args, p.count, p.band, p.groups, options);
    const band = solveCoulombNewton(...args, p.count, p.band, p.groups, { ...options, coulombLinearSolver: 'band-lu' });
    assert.ok(band.diagnostics.converged); audit(p, band);
    assert.deepEqual(band.increment, dense.increment); assert.deepEqual(band.residual, dense.residual);
    assert.equal(band.diagnostics.factorizations, dense.diagnostics.factorizations);
    assert.equal(band.diagnostics.boundRecoveries, dense.diagnostics.boundRecoveries);
    assert.ok(band.diagnostics.maximumLinearBackwardError < 1e-12);
});

test('full band entry owns output snapshots even when the fixed-load seed certifies immediately', () => {
    const options = { coulombStructure: 'full-band', workspace: {}, loadWorkspace: {}, tolerance: 1e-10 };
    const lo = Float64Array.of(-Infinity), hi = Float64Array.of(Infinity);
    const first = solveActiveCondensedCoupledQP(Float64Array.of(2), Float64Array.of(1), lo, hi, 1, 1, [], options);
    const original = structuredClone(first);
    const second = solveActiveCondensedCoupledQP(Float64Array.of(3), Float64Array.of(2), lo, hi, 1, 1, [], options);
    assert.ok(second.diagnostics.converged);
    for (const key of ['increment', 'residual', 'free', 'lower', 'upper']) assert.deepEqual(first[key], original[key]);
});

test('band diagnostics include failed normal-map attempts and cold retry work', () => {
    const p = read('kirchhoff-coulomb-144-bound-recovery'), observed = [];
    const result = solveSeededCoulombNewton(...['matrix', 'rhs', 'lower', 'upper'].map(key => Float64Array.from(p[key])),
        p.count, p.band, p.groups, {
            ...p.options, tolerance: .0002, maxCoulombNewtonIterations: 1, coulombLinearSolver: 'band-lu',
            debugCoulombResult({ result }) { observed.push({ ...result.diagnostics }); }
        });
    assert.ok(observed.length > 1, 'regression must exercise multiple actual Newton attempts');
    for (const key of ['rowSwaps', 'linearResidualFailures'])
        assert.equal(result.diagnostics[key], observed.reduce((sum, d) => sum + (d[key] ?? 0), 0), key);
    for (const key of ['maximumLinearBackwardError', 'maximumPivotGrowth'])
        assert.equal(result.diagnostics[key], Math.max(...observed.map(d => d[key] ?? 0)), key);
});

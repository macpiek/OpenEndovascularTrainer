import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { solveCoulombNewton, solveSeededCoulombNewton } from '../src/physics/kirchhoffCoulombNewtonSolver.js';
import { measureCoupledLoadKKT } from '../src/physics/kirchhoffCoupledLoadSolver.js';

function checkOriginal(p, result) {
    const residual = Float64Array.from(p.rhs), x = result.increment;
    for (let i = 0; i < p.count; i++) {
        residual[i] -= p.matrix[i * p.band] * x[i];
        for (let j = Math.max(0, i - p.band + 1); j < i; j++) {
            const value = p.matrix[i * p.band + i - j];
            residual[i] -= value * x[j]; residual[j] -= value * x[i];
        }
        assert.ok(x[i] >= p.lower[i] && x[i] <= p.upper[i], `force bound ${i}`);
    }
    const groups = p.groups.map(g => ({ ...g, radii: g.normalRow == null ? g.radii : g.mu.map(mu => mu * Math.max(0, g.normalLambda + x[g.normalRow])) }));
    const kkt = measureCoupledLoadKKT(residual, x, p.lower, p.upper, groups);
    assert.ok(kkt.maximumResidual <= p.options.tolerance, JSON.stringify(kkt));
    assert.ok(kkt.coneViolation <= 1e-9);
    residual.forEach((value, i) => assert.ok(Math.abs(value - result.residual[i]) < 1e-12, `complete reaction ${i}`));
    return kkt;
}

test('captured 144.2 mm Coulomb state recovers actual bounds and passes the unchanged full certificate', () => {
    const p = JSON.parse(gunzipSync(fs.readFileSync(new URL('./fixtures/kirchhoff-coulomb-144-bound-recovery.json.gz', import.meta.url))),
        (_, value) => value === 'Infinity' ? Infinity : value === '-Infinity' ? -Infinity : value);
    const args = ['matrix', 'rhs', 'lower', 'upper'].map(key => Float64Array.from(p[key]));
    assert.equal(p.baselineDiagnostics.converged, false);
    assert.ok(p.baselineIncrement[105] > 0 && p.baselineIncrement[105] < 1e-18);
    assert.ok(p.baselineResidual[105] < -p.options.tolerance, 'the old positive force was incompatible with its separated contact');
    const before = structuredClone({ args, groups: p.groups, initialIncrement: p.initialIncrement });
    const direct = solveCoulombNewton(...args, p.count, p.band, p.groups, { ...p.options, initialIncrement: p.initialIncrement });
    assert.ok(direct.diagnostics.converged, JSON.stringify(direct.diagnostics));
    assert.ok(direct.diagnostics.boundRecoveries > 0);
    assert.ok(direct.diagnostics.factorizations <= 4, 'correct a force-domain bound instead of repeating a stalled Newton solve');
    assert.ok(direct.diagnostics.backtracks <= 2);
    assert.equal(direct.increment[105], args[2][105]);
    checkOriginal({ ...p, lower: args[2], upper: args[3] }, direct);
    const seeded = solveSeededCoulombNewton(...args, p.count, p.band, p.groups, { ...p.options, retryWithoutHints: false });
    assert.ok(seeded.diagnostics.converged, JSON.stringify(seeded.diagnostics));
    assert.equal(seeded.diagnostics.coldSeedRetry, undefined);
    assert.equal(seeded.diagnostics.normalLoadIterations, 1);
    checkOriginal({ ...p, lower: args[2], upper: args[3] }, seeded);
    assert.deepEqual({ args, groups: p.groups, initialIncrement: p.initialIncrement }, before, 'numerical candidates do not mutate supplied physical forces');
});

for (const sign of [-1, 1]) test(`natural-bound recovery resolves a nonzero ghost force without a magnitude cutoff (${sign})`, () => {
    const lower = new Float64Array([sign > 0 ? 0 : -Infinity]), upper = new Float64Array([sign > 0 ? Infinity : 0]);
    const result = solveCoulombNewton(new Float64Array([1]), new Float64Array([-sign]), lower, upper, 1, 1, [],
        { tolerance: 1e-12, initialIncrement: new Float64Array([sign * 1e-20]) });
    assert.ok(result.diagnostics.converged, JSON.stringify(result.diagnostics));
    assert.equal(result.increment[0], 0);
    assert.equal(result.residual[0], -sign);
    assert.equal(result.diagnostics.boundRecoveries, 1);
});

test('a legitimate arbitrarily small loaded contact keeps its force and friction reaction', () => {
    const load = 1e-20, radius = 0.1 * load;
    const matrix = new Float64Array([1, 0, 0, 1, 0, 0, 1, 0, 0]);
    const group = { rows: [1, 2], normalRow: 0, normalLambda: 0, lambda: [0, 0], mu: [0.1, 0.1] };
    const result = solveCoulombNewton(matrix, new Float64Array([load, -1, 0]), new Float64Array([0, -Infinity, -Infinity]),
        new Float64Array(3).fill(Infinity), 3, 3, [group], { tolerance: 1e-12, initialIncrement: new Float64Array([load, -radius, 0]) });
    assert.ok(result.diagnostics.converged, JSON.stringify(result.diagnostics));
    assert.equal(result.increment[0], load);
    assert.equal(result.increment[1], -radius);
    assert.equal(result.diagnostics.boundRecoveries, 0);
});

test('bound recovery refreshes a dependent zero-load friction cone', () => {
    const oldLoad = 2e-20, oldFriction = [-1e-21, 1e-21];
    const p = { matrix: new Float64Array([1, 0, 0, 1, 0, 0, 1, 0, 0]), rhs: new Float64Array([-1, 1, -1]),
        lower: new Float64Array([-oldLoad, -Infinity, -Infinity]), upper: new Float64Array(3).fill(Infinity), count: 3, band: 3,
        groups: [{ rows: [1, 2], normalRow: 0, normalLambda: oldLoad, lambda: oldFriction, mu: [0.1, 0.2] }], options: { tolerance: 1e-12 } };
    const result = solveCoulombNewton(p.matrix, p.rhs, p.lower, p.upper, 3, 3, p.groups,
        { ...p.options, initialIncrement: new Float64Array([1e-20, 0, 0]) });
    assert.ok(result.diagnostics.converged, JSON.stringify(result.diagnostics));
    assert.equal(result.increment[0], -oldLoad);
    assert.deepEqual(Array.from(result.increment.slice(1)), oldFriction.map(value => -value));
    assert.deepEqual(result.allGroups[0].radii, [0, 0]);
    checkOriginal(p, result);
});

test('a projected bound cannot be certified until every changed coupled reaction passes KKT', () => {
    const matrix = new Float64Array([1, 0, 1, 0.8]), rhs = new Float64Array([0, 1.8]);
    const result = solveCoulombNewton(matrix, rhs, new Float64Array(2), new Float64Array(2).fill(Infinity), 2, 2, [],
        { tolerance: 1e-10, initialIncrement: new Float64Array([1, 1]), maxCoulombNewtonIterations: 0 });
    assert.equal(result.diagnostics.converged, false);
    assert.equal(result.increment[0], 0);
    assert.equal(result.residual[1], 0.8, 'removing force in row zero changes the other original equation');
    assert.equal(result.diagnostics.maximumResidual, 0.8);
});

import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { solveActiveCondensedCoupledQP } from '../src/physics/kirchhoffActiveCondensedSolver.js';
import { solveCoupledFrictionQP, measureCoupledFrictionKKT } from '../src/physics/kirchhoffCoupledFrictionSolver.js';
import { measureCoupledLoadKKT } from '../src/physics/kirchhoffCoupledLoadSolver.js';

function banded(A) {
    const n = A.length, out = new Float64Array(n * n);
    for (let i = 0; i < n; i++) for (let j = 0; j <= i; j++) out[i * n + i - j] = A[i][j];
    return out;
}
const bounds = n => [new Float64Array(n).fill(-Infinity), new Float64Array(n).fill(Infinity)];
function solve(A, b, lo, hi, groups = []) {
    return solveActiveCondensedCoupledQP(banded(A), Float64Array.from(b), lo, hi, b.length, b.length, groups, { tolerance: 1e-10 });
}

test('a previously inactive contact enters after the coupled material response', () => {
    const A = [[1, -.8, 0, 0], [-.8, 1, 0, 0], [0, 0, 2, 0], [0, 0, 0, 1]];
    const [lo, hi] = bounds(4); lo[0] = lo[1] = lo[3] = 0;
    const r = solve(A, [1, -.2, .3, -1], lo, hi);
    assert.ok(r.diagnostics.converged, JSON.stringify(r.diagnostics));
    assert.equal(r.diagnostics.expansions, 2);
    assert.equal(r.diagnostics.schurReuses, 1);
    assert.equal(r.diagnostics.schurEvaluations, 3);
    assert.equal(r.diagnostics.responseColumns, 2, 'the inactive fourth contact needs no response column');
    assert.ok(Math.abs(r.increment[0] - 7 / 3) < 1e-9);
    assert.ok(Math.abs(r.increment[1] - 5 / 3) < 1e-9);
    assert.ok(Math.abs(r.increment[2] - .15) < 1e-12); assert.equal(r.increment[3], 0);
});

test('Schur reuse follows original row identity when expansion inserts an earlier row', () => {
    const [lo, hi] = bounds(3); lo[0] = lo[1] = 0;
    const r = solve([[1, -.8, 0], [-.8, 1, 0], [0, 0, 2]], [-.2, 1, .3], lo, hi);
    assert.ok(r.diagnostics.converged, JSON.stringify(r.diagnostics));
    assert.equal(r.diagnostics.expansions, 2);
    assert.equal(r.diagnostics.schurReuses, 1);
    assert.equal(r.diagnostics.schurEvaluations, 3);
    assert.ok(Math.abs(r.increment[0] - 5 / 3) < 1e-9);
    assert.ok(Math.abs(r.increment[1] - 7 / 3) < 1e-9);
    assert.ok(Math.abs(r.increment[2] - .15) < 1e-12);
});

test('an inactive zero-load friction group enters together with its newly loaded normal', () => {
    const A = Array.from({ length: 6 }, (_, i) => Array.from({ length: 6 }, (_, j) => Number(i === j)));
    A[0][1] = A[1][0] = -.8;
    const [lo, hi] = bounds(6); lo[0] = lo[1] = 0;
    const groups = [{ rows: [2, 3], normalRow: 1, normalLambda: 0, lambda: [0, 0], mu: [.2, .3] }];
    const r = solve(A, [1, -.2, 3, 0, .1, .2], lo, hi, groups);
    assert.ok(r.diagnostics.converged, JSON.stringify(r.diagnostics));
    assert.equal(r.diagnostics.expansions, 2);
    assert.ok(Math.abs(r.increment[1] - 5 / 3) < 1e-9);
    assert.ok(Math.abs(r.increment[2] - 1 / 3) < 1e-9);
    assert.equal(r.increment[3], 0);
    assert.equal(r.allGroups.length, 1);
});

test('equality-only and fixed nonzero forces reconstruct the complete original solution', () => {
    const A = [[2, .3, .1], [.3, 2, .2], [.1, .2, 3]], [lo, hi] = bounds(3);
    lo[0] = hi[0] = -.4;
    const r = solve(A, [.2, .3, -.1], lo, hi);
    assert.ok(r.diagnostics.converged, JSON.stringify(r.diagnostics));
    assert.equal(r.diagnostics.retainedCount, 0); assert.equal(r.increment[0], -.4);
    for (const i of [1, 2]) assert.ok(Math.abs(r.residual[i]) < 1e-12);
});

test('an immovable violated row cannot disappear into the condensed certificate', () => {
    const [lo, hi] = bounds(2), r = solve([[1, 0], [0, 0]], [.2, .01], lo, hi);
    assert.equal(r.diagnostics.converged, false);
    assert.ok(r.diagnostics.maximumResidual >= .01);
});

for (let sample = 0; sample < 8; sample++) test(`anisotropic friction and fixed forces match the full SPD QP ${sample}`, () => {
    const n = 9, A = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) =>
        (i === j ? 2 : 0) + .3 * Math.cos(i - j)));
    const b = Float64Array.from({ length: n }, (_, i) => Math.sin(i + sample)), [lo, hi] = bounds(n);
    lo[0] = -.02; lo[1] = 0; lo[7] = hi[7] = .13;
    const groups = [{ rows: [3, 8], radii: [.05, .3], lambda: [.01, -.02] }];
    const r = solve(A, b, lo, hi, groups);
    const full = solveCoupledFrictionQP(banded(A), b, lo, hi, n, n, groups, { tolerance: 1e-10 });
    assert.ok(r.diagnostics.converged && full.diagnostics.converged, JSON.stringify(r.diagnostics));
    r.increment.forEach((value, i) => assert.ok(Math.abs(value - full.increment[i]) < 1e-8, `row ${i}`));
    assert.ok(measureCoupledFrictionKKT(r.residual, r.increment, lo, hi, groups).maximumResidual <= 1e-10);
});

test('held 100 mm contact/load problem satisfies the full equations after lazy material elimination', () => {
    const p = JSON.parse(gunzipSync(fs.readFileSync(new URL('./fixtures/kirchhoff-normal-load-cycle.json.gz', import.meta.url))),
        (_, v) => v === 'Infinity' ? Infinity : v === '-Infinity' ? -Infinity : v);
    const matrix = Float64Array.from(p.matrix), rhs = Float64Array.from(p.rhs), lo = Float64Array.from(p.lower), hi = Float64Array.from(p.upper);
    const r = solveActiveCondensedCoupledQP(matrix, rhs, lo, hi, p.count, p.band, p.groups, p.options);
    assert.ok(r.diagnostics.converged, JSON.stringify(r.diagnostics));
    assert.ok(r.diagnostics.retainedCount < 80); assert.ok(r.diagnostics.factorizations < 300);
    const residual = rhs.slice(), x = r.increment;
    for (let i = 0; i < p.count; i++) {
        residual[i] -= matrix[i * p.band] * x[i];
        for (let j = Math.max(0, i - p.band + 1); j < i; j++) {
            const a = matrix[i * p.band + i - j]; residual[i] -= a * x[j]; residual[j] -= a * x[i];
        }
        assert.ok(x[i] >= lo[i] && x[i] <= hi[i], `bound ${i}`);
    }
    const kkt = measureCoupledLoadKKT(residual, x, lo, hi, p.groups.map(g => ({ ...g,
        radii: g.mu.map(mu => mu * Math.max(0, g.normalLambda + x[g.normalRow])) })));
    assert.ok(kkt.maximumResidual <= p.options.tolerance, JSON.stringify(kkt));
    assert.ok(kkt.coneViolation <= 1e-9);
});

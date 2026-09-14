import test from 'node:test';
import assert from 'node:assert/strict';
import { createSharedAxisLinear, solveSharedAxisLinear } from '../src/physics/kirchhoffSharedAxisLinear.js';

const close = (a, b, tolerance = 1e-9) => assert.ok(Math.abs(a - b) < tolerance * Math.max(1, Math.abs(a), Math.abs(b)), `${a} vs ${b}`);
function fixture() {
    const layout = { dofCount: 14, band: 2 }, n = layout.dofCount;
    const rows = [
        { kind: 'wall', id: 'active-friction', dofs: [0, 1, 2, 3, 4, 5], jacobian: [1, 0, 0, 0, 0, 0], gap: -.4, multiplier: .2,
            extraForceDofs: [0, 1, 2, 3, 4, 5, 13], extraForceJacobian: [0, -.1, .2, 0, .15, 0, .8] },
        { kind: 'wall', id: 'released-friction', dofs: [6], jacobian: [1], gap: 10, multiplier: 2,
            extraForceDofs: [12, 13], extraForceJacobian: [-.4, -.3], geometricHessian: [3] },
        { kind: 'length', id: 'length', dofs: [11], jacobian: [1], gap: -.15, multiplier: -.4 }
    ];
    const H = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => i === j ? 2 : 0));
    H[13][12] = .3; H[12][13] = -.2;
    const tangent = new Float64Array(n * 3);
    H.forEach((r, i) => r.forEach((v, j) => { if (Math.abs(i - j) <= 1) tangent[i * 3 + j - i + 1] = v; }));
    const gradient = new Float64Array(n); gradient[0] = .3; gradient[6] = 1;
    for (const r of rows) {
        r.dofs.forEach((p, k) => { gradient[p] += (r.kind === 'wall' ? -1 : 1) * r.multiplier * r.jacobian[k]; });
        r.extraForceDofs?.forEach((p, k) => { gradient[p] += r.multiplier * r.extraForceJacobian[k]; });
    }
    const fixed = new Uint8Array(n); fixed[2] = 1;
    return { layout, chain: { layout, tangent }, H, rows, gradient, fixed };
}

// Independent dense construction in [all primal, all dual] order. The active
// set is known analytically for this fixture, not obtained from the band solver.
function denseSystem(f, active = [true, false, true]) {
    const n = f.layout.dofCount, count = n + f.rows.length;
    const A = Array.from({ length: count }, () => new Float64Array(count)), F = new Float64Array(count);
    f.H.forEach((r, i) => { A[i].set(r); F[i] = f.gradient[i]; });
    f.rows.forEach((r, index) => {
        const d = n + index;
        r.dofs.forEach((p, k) => { A[p][d] += (r.kind === 'wall' ? -1 : 1) * r.jacobian[k]; });
        r.extraForceDofs?.forEach((p, k) => { A[p][d] += r.extraForceJacobian[k]; });
        if (active[index]) { F[d] = r.gap; r.dofs.forEach((p, k) => { A[d][p] = r.jacobian[k]; }); }
        else { A[d][d] = 1; F[d] = r.multiplier; }
        if (r.geometricHessian) r.dofs.forEach((p, i) => r.dofs.forEach((q, j) => { A[p][q] += r.geometricHessian[i * r.dofs.length + j]; }));
    });
    for (let i = 0; i < count; i++) for (let j = 0; j < count; j++) if (f.fixed[i] || f.fixed[j]) A[i][j] = i === j ? 1 : 0;
    f.fixed.forEach((v, i) => { if (v) F[i] = 0; });
    return { A, F };
}
function denseSolve({ A, F }) {
    A = A.map(r => Array.from(r)); const b = Array.from(F, v => -v), n = b.length;
    for (let k = 0; k < n; k++) {
        let pivot = k; for (let i = k + 1; i < n; i++) if (Math.abs(A[i][k]) > Math.abs(A[pivot][k])) pivot = i;
        assert.ok(Math.abs(A[pivot][k]) > 1e-12);
        [A[k], A[pivot]] = [A[pivot], A[k]]; [b[k], b[pivot]] = [b[pivot], b[k]];
        for (let i = k + 1; i < n; i++) {
            const scale = A[i][k] / A[k][k];
            for (let j = k; j < n; j++) A[i][j] -= scale * A[k][j]; b[i] -= scale * b[k];
        }
    }
    const x = new Float64Array(n);
    for (let i = n - 1; i >= 0; i--) { let v = b[i]; for (let j = i + 1; j < n; j++) v -= A[i][j] * x[j]; x[i] = v / A[i][i]; }
    return x;
}
function check(f, result) {
    assert.equal(result.converged, true, JSON.stringify(result));
    const actual = [...result.increment, ...result.multiplierIncrement], expected = denseSolve(denseSystem(f));
    actual.forEach((v, i) => close(v, expected[i]));
    f.rows.forEach((r, i) => {
        const gap = r.gap + r.dofs.reduce((v, p, k) => v + r.jacobian[k] * result.increment[p], 0);
        const lambda = r.multiplier + result.multiplierIncrement[i];
        if (r.kind === 'wall') { assert.ok(gap >= -1e-9); assert.ok(lambda >= -1e-9); close(gap * lambda, 0); }
        else close(gap, 0);
    });
}

test('extra wall force and spin columns match independent dense nonsymmetric KKT, including inactive loaded rows', () => {
    const f = fixture(), before = structuredClone(f);
    for (const compactWorkingSet of [true, false]) for (const lazy of [true, false]) {
        const w = createSharedAxisLinear(f.layout, f.rows, { lazy });
        check(f, solveSharedAxisLinear(w, f.chain, { ...f, compactWorkingSet }));
    }
    assert.deepEqual(f, before, 'Force columns and physical reactions remain immutable');
});

test('force derivative occupies only stationarity column and agrees with finite differences', () => {
    const f = fixture(), w = createSharedAxisLinear(f.layout, f.rows);
    check(f, solveSharedAxisLinear(w, f.chain, { ...f, compactWorkingSet: false }));
    const d = w.dual[0], spin = w.primal[13], at = (i, j) => j < w.band.starts[i] || j > w.band.ends[i] ? 0 : w.matrix[w.band.offsets[i] + j];
    close(at(spin, d), .8); close(at(d, spin), 0);
    const stationarity = lambda => {
        const values = new Float64Array(f.layout.dofCount), r = f.rows[0];
        r.dofs.forEach((p, k) => { values[p] -= lambda * r.jacobian[k]; });
        r.extraForceDofs.forEach((p, k) => { values[p] += lambda * r.extraForceJacobian[k]; });
        return values;
    };
    const h = 1e-6, plus = stationarity(.2 + h), minus = stationarity(.2 - h);
    for (let p = 0; p < f.layout.dofCount; p++) if (!f.fixed[p]) close(at(w.primal[p], d), (plus[p] - minus[p]) / (2 * h), 1e-8);
});

test('changed optional support rebuilds compact/reference workspaces without reusing an incompatible band', () => {
    for (const compactWorkingSet of [true, false]) for (const lazy of [true, false]) {
        const f = fixture(), plain = f.rows.map(({ extraForceDofs, extraForceJacobian, ...r }) => r);
        const w = createSharedAxisLinear(f.layout, plain, { lazy });
        check(f, solveSharedAxisLinear(w, f.chain, { ...f, compactWorkingSet }));
        // Keep the same normal rows but relocate the optional torque support.
        // Rebuild the physical gradient from the modified force columns.
        const old = f.rows[0].extraForceDofs.at(-1), next = 10, value = f.rows[0].extraForceJacobian.at(-1);
        f.rows[0].extraForceDofs[f.rows[0].extraForceDofs.length - 1] = next;
        f.gradient[old] -= value * f.rows[0].multiplier; f.gradient[next] += value * f.rows[0].multiplier;
        check(f, solveSharedAxisLinear(w, f.chain, { ...f, compactWorkingSet }));
        if (compactWorkingSet) assert.ok(w.activeWorkspaces.size >= 2);
    }
});

test('malformed optional force columns fail explicitly instead of producing missing or nonfinite matrix entries', () => {
    for (const extra of [{ extraForceDofs: [13] }, { extraForceDofs: [14], extraForceJacobian: [1] },
        { extraForceDofs: [13], extraForceJacobian: [NaN] }]) {
        const f = fixture(); Object.assign(f.rows[0], extra);
        assert.throws(() => solveSharedAxisLinear(createSharedAxisLinear(f.layout, [], { lazy: true }), f.chain, f), /extra force/);
    }
});

test('dependent normal rows with distinct friction columns satisfy the complete original stationarity equations', () => {
    const layout = { dofCount: 3, band: 1 }, chain = { layout, hessian: new Float64Array([2, 3, 4]) };
    const rows = [
        { kind: 'wall', id: 'face-A', dofs: [0], jacobian: [1], gap: -.4, multiplier: .3,
            extraForceDofs: [1, 2], extraForceJacobian: [.3, .8] },
        { kind: 'wall', id: 'face-B', dofs: [0], jacobian: [1], gap: -.4, multiplier: .6,
            extraForceDofs: [1, 2], extraForceJacobian: [-.7, -.2] }
    ];
    const gradient = new Float64Array([.2 - .3 - .6, .3 * .3 - .7 * .6, .8 * .3 - .2 * .6]);
    const fixed = new Uint8Array(3), snapshot = structuredClone(rows), results = [];
    for (const compactWorkingSet of [true, false]) {
        const trace = [], result = solveSharedAxisLinear(createSharedAxisLinear(layout, rows, { lazy: true }), chain,
            { rows, gradient, fixed, compactWorkingSet, trace });
        assert.equal(result.converged, true, JSON.stringify(result));
        assert.ok(trace.some(row => row.kind === 'basis-pivot'), 'Exercise dependent-basis removal');
        const balance = Float64Array.from(gradient, (v, i) => v + chain.hessian[i] * result.increment[i]);
        rows.forEach((r, i) => {
            const change = result.multiplierIncrement[i], lambda = r.multiplier + change;
            assert.ok(lambda >= -1e-9); close(r.gap + result.increment[0], 0);
            balance[0] -= change;
            r.extraForceDofs.forEach((p, k) => { balance[p] += change * r.extraForceJacobian[k]; });
        });
        balance.forEach(v => close(v, 0));
        results.push([...result.increment, ...result.multiplierIncrement]);
    }
    results[0].forEach((v, i) => close(v, results[1][i]));
    assert.deepEqual(rows, snapshot);
});

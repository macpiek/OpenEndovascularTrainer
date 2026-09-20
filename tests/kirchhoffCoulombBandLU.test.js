import assert from 'node:assert/strict';
import test from 'node:test';
import { createCoulombBandLayout, createCoulombBandLU } from '../src/physics/kirchhoffCoulombBandLU.js';
import { solveCoulombNewton } from '../src/physics/kirchhoffCoulombNewtonSolver.js';

function generalLayout(A) {
    const n = A.length, starts = new Int32Array(n), ends = new Int32Array(n), offsets = new Int32Array(n);
    let entries = 0, kl = 0, ku = 0;
    for (let i = 0; i < n; i++) {
        starts[i] = ends[i] = i;
        for (let j = 0; j < n; j++) if (A[i][j] !== 0) { starts[i] = Math.min(starts[i], j); ends[i] = Math.max(ends[i], j); }
        offsets[i] = entries - starts[i]; entries += ends[i] - starts[i] + 1;
        kl = Math.max(kl, i - starts[i]); ku = Math.max(ku, ends[i] - i);
    }
    const packed = new Float64Array(entries);
    for (let i = 0; i < n; i++) for (let j = starts[i]; j <= ends[i]; j++) packed[offsets[i] + j] = A[i][j];
    return { starts, ends, offsets, entries, kl, ku, packed };
}

for (const [n, kl, ku] of [[4, 1, 1], [23, 3, 2], [51, 7, 4], [19, 0, 3], [17, 4, 0], [0, 0, 0]]) {
    test(`nonsymmetric band LU solves original equations with pivot fill (${n}/${kl}/${ku})`, () => {
        const A = Array.from({ length: n }, () => new Float64Array(n));
        for (let i = 0; i < n; i++) for (let j = Math.max(0, i - kl); j <= Math.min(n - 1, i + ku); j++)
            A[i][j] = i === j ? .03 + (i % 7) / 10 : Math.sin(i * 11 + j * 17 + .5);
        const layout = generalLayout(A), lu = createCoulombBandLU(layout, n), scale = new Float64Array(n).fill(1);
        const expected = Float64Array.from({ length: n }, (_, i) => Math.cos(i * .7));
        const F = Float64Array.from(A, row => -row.reduce((sum, v, j) => sum + v * expected[j], 0));
        const x = new Float64Array(n);
        assert.ok(lu.solve(layout.packed, F, scale, 0, x));
        for (let i = 0; i < n; i++) {
            const error = A[i].reduce((sum, v, j) => sum + v * x[j], F[i]);
            const magnitude = A[i].reduce((sum, v, j) => sum + Math.abs(v * x[j]), Math.abs(F[i]));
            assert.ok(Math.abs(error) <= 2e-14 * Math.max(1, magnitude), `original row ${i}: ${error}`);
        }
        if (kl && ku) assert.ok(lu.diagnostics.rowSwaps > 0, 'exercise actual partial pivoting');
        assert.equal(lu.diagnostics.factorEntries, n * (2 * kl + ku + 1));
        // Refill the same workspace after many row interchanges: no factor or
        // right-envelope state may survive into a subsequent system.
        const first = x.slice();
        assert.ok(lu.solve(layout.packed, F, scale, 0, x)); assert.deepEqual(x, first);
    });
}

test('LU rejects a singular pivot instead of imposing an SPD floor', () => {
    const layout = generalLayout([[0, 1], [0, 2]]), lu = createCoulombBandLU(layout, 2);
    assert.equal(lu.solve(layout.packed, Float64Array.of(1, 1), Float64Array.of(1, 1), 0, new Float64Array(2)), false);
});

test('large partial-pivot growth rejects an inaccurate Newton solve through original linear backward error', () => {
    // Wilkinson's growth example: pivot magnitudes tie, so U's last column
    // doubles at each elimination. Partial pivoting alone is no certificate.
    const n = 60, A = Array.from({ length: n }, (_, i) => Float64Array.from({ length: n },
        (_, j) => j === n - 1 || i === j ? 1 : j < i ? -1 : 0));
    const layout = generalLayout(A);
    const expected = Float64Array.from({ length: n }, (_, i) => Math.sin(i + .7));
    const F = Float64Array.from(A, row => -row.reduce((sum, v, j) => sum + v * expected[j], 0));
    const diagnostics=[];
    for(const wasmAssembly of [false,true]) {
        const lu=createCoulombBandLU(layout,n,{wasmAssembly}),output=new Float64Array(n).fill(123);
        assert.equal(lu.solve(layout.packed, F, new Float64Array(n).fill(1), 0, output), false);
        assert.ok(output.every(value=>value===123),'An uncertified solve must not publish a direction');
        assert.equal(lu.diagnostics.linearResidualFailures, 1);
        assert.ok(lu.diagnostics.maximumPivotGrowth > 1e15);
        assert.ok(lu.diagnostics.maximumLinearBackwardError > 1e-8);
        diagnostics.push(lu.diagnostics);
    }
    assert.deepEqual(diagnostics[1],diagnostics[0]);
});

test('exact envelopes retain tiny coefficients and distant non-associated normal and pair columns', () => {
    const n = 12, band = 2, A = new Float64Array(n * band);
    for (let i = 0; i < n; i++) A[i * band] = 2;
    A[3 * band + 1] = 1e-30;
    const layout = createCoulombBandLayout(A, n, band, [{ rows: [2, 8], normalRow: 11 }]);
    assert.equal(layout.starts[3], 2); assert.equal(layout.ends[2], 11);
    assert.equal(layout.starts[8], 2); assert.equal(layout.ends[8], 11);
    assert.equal(layout.starts[11], 11, 'no transpose normal coupling / associated dilation');
});

test('packed Newton Jacobian and direction equal dense reference for loaded, unloaded and fixed ellipses', () => {
    const n = 14, band = 4, A = new Float64Array(n * band), b = new Float64Array(n);
    const lo = new Float64Array(n).fill(-Infinity), hi = new Float64Array(n).fill(Infinity);
    for (let i = 0; i < n; i++) {
        A[i * band] = 3; b[i] = Math.sin(i + .8);
        for (let j = Math.max(0, i - band + 1); j < i; j++) A[i * band + i - j] = .1 * Math.cos(i + j);
    }
    lo[1] = 0; hi[12] = 0; A[6 * band + 3] = 1e-30;
    for (const normalMap of ['projection', 'fischer-burmeister']) for (const load of [0, .2]) {
        const groups = [{ rows: [2, 8], normalRow: 1, normalLambda: load, mu: [.2, .6], lambda: [.01, -.02] },
            { rows: [4, 10], radii: [.1, .3], lambda: [-.01, 0] }];
        const snapshots = [];
        for (const coulombLinearSolver of [undefined, 'band-lu']) {
            solveCoulombNewton(A, b, lo, hi, n, band, groups, { tolerance: 1e-10, normalMap,
                coulombLinearSolver, maxCoulombNewtonIterations: 1,
                debugCoulombDirection({ J, F, direction, layout }) {
                    const dense = new Float64Array(n * n);
                    if (!layout) dense.set(J);
                    else for (let i = 0; i < n; i++) for (let j = layout.starts[i]; j <= layout.ends[i]; j++) dense[i * n + j] = J[layout.offsets[i] + j];
                    snapshots.push({ J: dense, F: F.slice(), direction: direction.slice() });
                } });
        }
        assert.equal(snapshots.length, 2); assert.deepEqual(snapshots[1].F, snapshots[0].F);
        assert.deepEqual(snapshots[1].J, snapshots[0].J);
        for (let i = 0; i < n; i++) assert.ok(Math.abs(snapshots[1].direction[i] - snapshots[0].direction[i]) < 1e-12);
    }
});

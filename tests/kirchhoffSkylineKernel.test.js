import assert from 'node:assert/strict';
import test from 'node:test';
import { createKirchhoffLinearKernel } from '../src/physics/kirchhoffLinearKernel.js';
import { solveCoupledBandQP } from '../src/physics/kirchhoffCoupledLinearSolver.js';

for (const n of [3, 17, 63]) test(`WASM skyline preserves dense-band Cholesky and original equations for an irregular profile (${n})`, () => {
    const A = Array.from({ length: n }, () => new Float64Array(n));
    for (let i = 0; i < n; i++) A[i][i] = 1;
    for (let i = 1; i < n; i++) for (let j = 0; j < i; j++) {
        if ((i * 17 + j * 13) % 11 !== 0 && j !== i - 1) continue;
        const value = 0.1 * Math.cos(i + j);
        A[i][j] = A[j][i] = value;
        A[i][i] += Math.abs(value); A[j][j] += Math.abs(value);
    }
    // An arbitrarily tiny leading coefficient must remain in the skyline.
    A[n - 1][0] = A[0][n - 1] = 1e-30;
    const kernel = createKirchhoffLinearKernel(16 * n * n + 64 * n);
    const band = kernel.alloc(Float64Array, n * n), skyline = kernel.alloc(Float64Array, n * n);
    const starts = kernel.alloc(Int32Array, n), b = kernel.alloc(Float64Array, n), s = kernel.alloc(Float64Array, n);
    for (let i = 0; i < n; i++) {
        starts[i] = i;
        for (let j = 0; j <= i; j++) {
            band[i * n + i - j] = A[i][j];
            if (A[i][j]) starts[i] = Math.min(starts[i], j);
        }
        b[i] = s[i] = Math.sin(i + 0.4);
    }
    skyline.set(band);
    kernel.factorBand(band.byteOffset, n, n, -1, 0);
    kernel.factorSkyline(skyline.byteOffset, n, n, starts.byteOffset);
    band.forEach((value, i) => assert.ok(Math.abs(value - skyline[i]) < 1e-14, `factor ${i}`));
    kernel.solveBand(band.byteOffset, b.byteOffset, n, n);
    kernel.solveSkyline(skyline.byteOffset, s.byteOffset, n, n, starts.byteOffset);
    for (let i = 0; i < n; i++) {
        assert.ok(Math.abs(b[i] - s[i]) < 1e-14, `solution ${i}`);
        assert.ok(Math.abs(A[i].reduce((sum, value, j) => sum + value * s[j], 0) - Math.sin(i + 0.4)) < 1e-13, `original residual ${i}`);
    }
    assert.equal(starts[n - 1], 0);
});

test('skyline keeps numerical pivot floor identical without changing the original band exports', () => {
    const kernel = createKirchhoffLinearKernel(256), a = kernel.alloc(Float64Array, 4), b = kernel.alloc(Float64Array, 4);
    const starts = kernel.alloc(Int32Array, 2);
    a.set([1, 0, 1, 1]); b.set(a); starts.set([0, 0]);
    kernel.factorBand(a.byteOffset, 2, 2, -1, 0);
    kernel.factorSkyline(b.byteOffset, 2, 2, starts.byteOffset);
    assert.deepEqual(b, a);
    assert.equal(b[2], 1e-6);
});

test('compressed skyline and band box solves retain the same active forces and KKT', () => {
    const n = 12, band = 8, matrix = new Float64Array(n * band);
    for (let i = 0; i < n; i++) matrix[i * band] = 2;
    for (const [i, j, value] of [[2, 1, 0.4], [3, 1, -0.2], [8, 1, 0.3], [8, 6, 0.2], [9, 8, -0.5], [11, 7, 1e-30]]) matrix[i * band + i - j] = value;
    const rhs = new Float64Array(Array.from({ length: n }, (_, i) => Math.cos(i)));
    const lower = new Float64Array(n).fill(-Infinity), upper = new Float64Array(n).fill(Infinity);
    for (const i of [1, 3, 5, 7]) { lower[i] = -0.02; upper[i] = 0.01; }
    const skyline = solveCoupledBandQP(matrix, rhs, lower, upper, n, band, { tolerance: 1e-11 });
    const prior = solveCoupledBandQP(matrix, rhs, lower, upper, n, band, { tolerance: 1e-11, factorization: 'band' });
    assert.ok(skyline.diagnostics.converged && prior.diagnostics.converged);
    assert.equal(skyline.diagnostics.factorization, 'skyline');
    assert.ok(skyline.diagnostics.factorProfileEntries < skyline.diagnostics.factorEntries);
    skyline.increment.forEach((value, i) => assert.ok(Math.abs(value - prior.increment[i]) < 1e-11));
});

test('WASM equilibration/profile/compression exactly reproduces a dense principal matrix under changing masks', () => {
    const n = 13, band = 9, kernel = createKirchhoffLinearKernel(32 * n * band + 128 * n);
    const matrix = kernel.alloc(Float64Array, n * band), factor = kernel.alloc(Float64Array, n * band);
    const scales = kernel.alloc(Float64Array, n), starts = kernel.alloc(Int32Array, n), rawStarts = kernel.alloc(Int32Array, n);
    const factorStarts = kernel.alloc(Int32Array, n), rows = kernel.alloc(Int32Array, n), map = kernel.alloc(Int32Array, n);
    for (let pass = 0; pass < 4; pass++) {
        matrix.fill(0); map.fill(-1);
        for (let i = 0; i < n; i++) {
            matrix[i * band] = i + 2; scales[i] = 1 / Math.sqrt(i + 2);
            for (let j = Math.max(0, i - band + 1); j < i; j++) {
                if ((i * 7 + j * 11 + pass) % 9 === 0 || pass === 3 && j === i - 1) matrix[i * band + i - j] = Math.sin(i + j + 0.2);
            }
        }
        matrix[(n - 1) * band + 8] = 1e-30;
        const original = matrix.slice();
        kernel.findBandStarts(matrix.byteOffset, rawStarts.byteOffset, n, band);
        kernel.scaleBandProfile(matrix.byteOffset, scales.byteOffset, starts.byteOffset, n, band);
        for (let i = 0; i < n; i++) {
            let first = i;
            for (let j = Math.max(0, i - band + 1); j <= i; j++) {
                const index = i * band + i - j;
                assert.equal(matrix[index], original[index] * scales[i] * scales[j]);
                if (matrix[index]) first = Math.min(first, j);
            }
            assert.equal(starts[i], first); assert.equal(rawStarts[i], first);
        }
        let count = 0;
        for (let i = 0; i < n; i++) if ((i + pass) % 3 !== 0) { map[i] = count; rows[count++] = i; }
        const compressedBand = kernel.compressFreeBand(matrix.byteOffset, factor.byteOffset, rows.byteOffset, map.byteOffset,
            starts.byteOffset, factorStarts.byteOffset, count, band, 1e-12);
        for (let i = 0; i < count; i++) {
            let first = i;
            for (let j = 0; j <= i; j++) {
                const delta = rows[i] - rows[j], expected = (delta < band ? matrix[rows[i] * band + delta] : 0) + (i === j ? 1e-12 : 0);
                const actual = i - j < compressedBand ? factor[i * compressedBand + i - j] : 0;
                assert.equal(actual, expected, `pass ${pass}, ${i},${j}`);
                if (expected) first = Math.min(first, j);
            }
            assert.equal(factorStarts[i], first);
        }
    }
});

test('source profiles are rebuilt when reused cone matrices gain distant nonzeros', async () => {
    const { solveCoupledFrictionQP } = await import('../src/physics/kirchhoffCoupledFrictionSolver.js');
    const n = 12, band = n, workspace = {}, rhs = new Float64Array(n).fill(0.2);
    const lower = new Float64Array(n).fill(-Infinity), upper = new Float64Array(n).fill(Infinity);
    const groups = [{ rows: [1, 10], radii: [0.1, 0.2], lambda: [0, 0] }];
    for (const distant of [false, true, false]) {
        const matrix = new Float64Array(n * band);
        for (let i = 0; i < n; i++) matrix[i * band] = 2;
        if (distant) matrix[11 * band + 11] = 0.7;
        const reused = solveCoupledFrictionQP(matrix, rhs, lower, upper, n, band, groups, { tolerance: 1e-10, frictionWorkspace: workspace });
        const fresh = solveCoupledFrictionQP(matrix, rhs, lower, upper, n, band, groups, { tolerance: 1e-10 });
        assert.ok(reused.diagnostics.converged && fresh.diagnostics.converged);
        assert.equal(workspace.sourceStarts[11], distant ? 0 : 11);
        reused.increment.forEach((value, i) => assert.ok(Math.abs(value - fresh.increment[i]) < 1e-12));
    }
});

test('profile products preserve the band product accumulation order and shifted diagonal', () => {
    const n = 17, band = 12, kernel = createKirchhoffLinearKernel(n * band * 8 + n * 64);
    const matrix = kernel.alloc(Float64Array, n * band), starts = kernel.alloc(Int32Array, n);
    const x = kernel.alloc(Float64Array, n), y = kernel.alloc(Float64Array, n), z = kernel.alloc(Float64Array, n), diagonal = kernel.alloc(Float64Array, n);
    for (let i = 0; i < n; i++) {
        matrix[i * band] = 1.1 + i / 3; x[i] = Math.cos(i) * (i % 2 ? 1e10 : 1e-10); diagonal[i] = i + 0.1;
        for (let j = Math.max(0, i - band + 1); j < i; j++) if ((i + j) % 3 === 0) matrix[i * band + i - j] = Math.sin(i + j) / (i + 1);
    }
    matrix[16 * band + 11] = 1e-30;
    kernel.findBandStarts(matrix.byteOffset, starts.byteOffset, n, band);
    for (const shift of [0, 0.003]) {
        kernel.multiplyBand(matrix.byteOffset, x.byteOffset, y.byteOffset, n, band, diagonal.byteOffset, shift);
        kernel.multiplyBandProfile(matrix.byteOffset, x.byteOffset, z.byteOffset, n, band, diagonal.byteOffset, shift, starts.byteOffset);
        assert.deepEqual(z, y);
    }
});

test('persistent cone scratch survives group removal, permutation, radius changes and capacity growth', async () => {
    const { solveCoupledFrictionQP } = await import('../src/physics/kirchhoffCoupledFrictionSolver.js');
    const workspace = {};
    for (const [n, permutation] of [[12, false], [12, true], [7, false], [40, true], [12, false]]) {
        const matrix = new Float64Array(n * n), rhs = new Float64Array(n);
        const lower = new Float64Array(n).fill(-Infinity), upper = new Float64Array(n).fill(Infinity);
        for (let i = 0; i < n; i++) { matrix[i * n] = 2; rhs[i] = Math.sin(i + .3); }
        const groups = n === 7 ? [] : [
            { rows: [1, n - 2], radii: [.02, permutation ? .8 : .3], lambda: [.001, -.02] },
            { rows: [3, 5], radii: [permutation ? .2 : .01, .4], lambda: [-.01, .02] }
        ];
        if (permutation) groups.reverse();
        const reused = solveCoupledFrictionQP(matrix, rhs, lower, upper, n, n, groups,
            { tolerance: 1e-10, frictionWorkspace: workspace });
        const fresh = solveCoupledFrictionQP(matrix, rhs, lower, upper, n, n, groups, { tolerance: 1e-10 });
        assert.ok(reused.diagnostics.converged && fresh.diagnostics.converged);
        assert.deepEqual(reused.increment, fresh.increment);
        assert.equal(reused.diagnostics.maximumResidual, fresh.diagnostics.maximumResidual);
    }
});

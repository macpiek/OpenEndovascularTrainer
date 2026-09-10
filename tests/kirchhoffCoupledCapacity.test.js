import assert from 'node:assert/strict';
import test from 'node:test';
import { solveCoupledBandQP } from '../src/physics/kirchhoffCoupledLinearSolver.js';
import { solveCoupledFrictionQP } from '../src/physics/kirchhoffCoupledFrictionSolver.js';

for (const cone of [false, true]) test(`geometric numerical capacity preserves solutions and avoids per-row WASM allocation (cone=${cone})`, () => {
    const workspace = {}, kernels = new Set();
    let prior;
    for (const count of [...Array.from({ length: 17 }, (_, i) => 16 + i), 8, 31]) {
        const band = 3, matrix = new Float64Array(count * band), rhs = new Float64Array(count);
        for (let i = 0; i < count; i++) { matrix[i * band] = 1; rhs[i] = Math.sin(i); }
        rhs[0] = 3; rhs[1] = 4;
        const lower = new Float64Array(count).fill(-Infinity), upper = new Float64Array(count).fill(Infinity);
        const solved = cone ? solveCoupledFrictionQP(matrix, rhs, lower, upper, count, band,
            [{ rows: [0, 1], radii: [0.3, 0.3], lambda: [0, 0] }], { frictionWorkspace: workspace, tolerance: 1e-9 })
            : solveCoupledBandQP(matrix, rhs, lower, upper, count, band, { workspace, tolerance: 1e-9 });
        assert.ok(solved.diagnostics.converged, JSON.stringify(solved.diagnostics));
        for (let i = 0; i < count; i++) assert.ok(Math.abs(solved.increment[i] - (cone && i < 2 ? [0.18, 0.24][i] : rhs[i])) < 1e-9);
        if (prior && count >= 18 && count <= 21) assert.equal(workspace.kernel, prior);
        kernels.add(workspace.kernel); prior = workspace.kernel;
    }
    assert.equal(kernels.size, 3, '16→32 rows need only initialization, row-capacity growth and one band-storage growth');
});

import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { createHash } from 'node:crypto';
import { solveActiveCondensedCoupledQP } from '../src/physics/kirchhoffActiveCondensedSolver.js';
import { createKirchhoffLinearKernel } from '../src/physics/kirchhoffLinearKernel.js';
import { measureCoupledLoadKKT } from '../src/physics/kirchhoffCoupledLoadSolver.js';

function fullResidual(matrix, rhs, x, count, band) {
    const out = rhs.slice();
    for (let i = 0; i < count; i++) {
        out[i] -= matrix[i * band] * x[i];
        for (let j = Math.max(0, i - band + 1); j < i; j++) {
            const a = matrix[i * band + i - j];
            out[i] -= a * x[j]; out[j] -= a * x[i];
        }
    }
    return out;
}
const hash = value => createHash('sha256').update(new Uint8Array(value.buffer, value.byteOffset, value.byteLength)).digest('hex');

test('WASM full residual preserves original subtraction order through zeros and tiny couplings', () => {
    const n = 17, band = 12, kernel = createKirchhoffLinearKernel(n * band * 8 + n * 64);
    const A = kernel.alloc(Float64Array, n * band), rhs = kernel.alloc(Float64Array, n), x = kernel.alloc(Float64Array, n);
    const out = kernel.alloc(Float64Array, n), starts = kernel.alloc(Int32Array, n);
    for (let i = 0; i < n; i++) {
        A[i * band] = i + 2; rhs[i] = Math.sin(i + 0.3); x[i] = Math.cos(i) * (i % 2 ? 1e6 : 1e-6);
        for (let j = Math.max(0, i - band + 1); j < i; j++) if ((i + 3 * j) % 7 === 0) A[i * band + i - j] = 0.1 * Math.cos(i + j);
    }
    A[16 * band + 11] = 1e-30;
    kernel.findBandStarts(A.byteOffset, starts.byteOffset, n, band);
    kernel.residualBandProfile(A.byteOffset, rhs.byteOffset, x.byteOffset, out.byteOffset, n, band, starts.byteOffset);
    assert.deepEqual(out, fullResidual(A, rhs, x, n, band));
    assert.equal(starts[16], 5);
});

test('reused condensed storage refreshes matrix, active identities and loads while previous outputs stay owned', () => {
    const workspace = {}, options = { tolerance: 1e-10, workspace, frictionWorkspace: {}, loadWorkspace: {} };
    const outputs = [];
    let kernelAtCapacity, responseAtCapacity;
    for (const [pass, n] of [8, 13, 20, 7, 20].entries()) {
        const matrix = new Float64Array(n * n), rhs = new Float64Array(n);
        const lo = new Float64Array(n).fill(-Infinity), hi = new Float64Array(n).fill(Infinity);
        for (let i = 0; i < n; i++) {
            rhs[i] = Math.sin(i * 0.3 + pass);
            for (let j = 0; j <= i; j++) matrix[i * n + i - j] = (i === j ? 2 + pass * 0.1 : 0) + 0.1 * Math.cos(i - j);
        }
        lo[0] = 0; lo[1] = hi[1] = 0.03 * (pass - 1); lo[2] = -0.01;
        const groups = [{ rows: pass % 2 ? [n - 1, n - 3] : [n - 3, n - 1], radii: [0.03 + 0.01 * pass, 0.2], lambda: [0.04 - 0.01 * pass, -0.03] }];
        options.initialFree = new Uint8Array(n).fill(pass % 2);
        const sourceGroups = structuredClone(groups);
        const reused = solveActiveCondensedCoupledQP(matrix, rhs, lo, hi, n, n, groups, options);
        const fresh = solveActiveCondensedCoupledQP(matrix, rhs, lo, hi, n, n, groups, { tolerance: options.tolerance, initialFree: options.initialFree });
        assert.ok(reused.diagnostics.converged && fresh.diagnostics.converged, JSON.stringify(reused.diagnostics));
        for (const key of ['increment', 'residual', 'free', 'lower', 'upper']) assert.deepEqual(reused[key], fresh[key], `${key}, pass ${pass}`);
        assert.deepEqual(reused.residual, fullResidual(matrix, rhs, reused.increment, n, n));
        assert.deepEqual(groups, sourceGroups);
        for (const { result, snapshot } of outputs) for (const key of Object.keys(snapshot)) assert.deepEqual(result[key], snapshot[key], `prior owned ${key}`);
        outputs.push({ result: reused, snapshot: Object.fromEntries(['increment', 'residual', 'free', 'lower', 'upper'].map(key => [key, reused[key].slice()])) });
        if (pass === 2) { kernelAtCapacity = workspace.activeCondensed.kernel; responseAtCapacity = workspace.activeCondensed.responsePool[0]; }
        if (pass >= 3) {
            assert.equal(workspace.activeCondensed.kernel, kernelAtCapacity, 'WASM memory reused through shrink and regrowth');
            assert.equal(workspace.activeCondensed.responsePool[0], responseAtCapacity, 'response storage reused while values are recalculated');
        }
    }
});

test('frozen 1999-row system retains baseline force/residual bytes and all original KKT equations', () => {
    const p = JSON.parse(gunzipSync(fs.readFileSync(new URL('./fixtures/kirchhoff-active-condensed-1999.json.gz', import.meta.url))),
        (_, value) => value === 'Infinity' ? Infinity : value === '-Infinity' ? -Infinity : value);
    const matrix = Float64Array.from(p.matrix), rhs = Float64Array.from(p.rhs), lower = Float64Array.from(p.lower), upper = Float64Array.from(p.upper);
    const options = { tolerance: 0.0002, numericalShift: 1e-8, initialFree: p.initialFree, workspace: {}, loadWorkspace: {}, frictionWorkspace: {}, simultaneousCoulomb: true };
    for (let pass = 0; pass < 2; pass++) {
        const result = solveActiveCondensedCoupledQP(matrix, rhs, lower, upper, p.count, p.band, p.groups, options);
        assert.ok(result.diagnostics.converged, JSON.stringify(result.diagnostics));
        assert.equal(hash(result.increment), '403c2d086511d18ee2bf36d3ecc368980a2cd7d9efe4a3a5424b978578fd557d');
        assert.equal(hash(result.residual), '7382dc03aa5fef01a9b29f5a1fd84f1cc6c5864147cef0540bf6f555df9db84d');
        const residual = fullResidual(matrix, rhs, result.increment, p.count, p.band);
        assert.deepEqual(result.residual, residual);
        const groups = p.groups.map(g => ({ ...g, radii: g.normalRow == null ? g.radii : g.mu.map(mu => mu * Math.max(0, g.normalLambda + result.increment[g.normalRow])) }));
        const kkt = measureCoupledLoadKKT(residual, result.increment, result.lower, result.upper, groups);
        assert.ok(kkt.maximumResidual <= options.tolerance, JSON.stringify(kkt));
        assert.ok(kkt.coneViolation <= 1e-9);
        result.increment.forEach((value, row) => assert.ok(value >= result.lower[row] && value <= result.upper[row], `force bound ${row}`));
    }
});

test('zero-row and equality-only systems remain valid with an explicitly reused condensed workspace', () => {
    const condensedWorkspace = {};
    for (const count of [0, 1, 0, 1]) {
        const result = solveActiveCondensedCoupledQP(new Float64Array(count).fill(2), new Float64Array(count).fill(1),
            new Float64Array(count).fill(-Infinity), new Float64Array(count).fill(Infinity), count, 1, [], { condensedWorkspace });
        assert.ok(result.diagnostics.converged);
        if (count) assert.ok(Math.abs(result.increment[0] - 0.5) < 1e-15);
    }
});

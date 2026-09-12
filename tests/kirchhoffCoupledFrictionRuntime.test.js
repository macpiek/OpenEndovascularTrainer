import assert from 'node:assert/strict';
import fs from 'node:fs';
import { gunzipSync } from 'node:zlib';
import test from 'node:test';
import { measureCoupledFrictionKKT, solveCoupledFrictionQP } from '../src/physics/kirchhoffCoupledFrictionSolver.js';

test('a released friction boundary blocking the coupled direction rejoins the working set', () => {
    const source = JSON.parse(gunzipSync(fs.readFileSync(new URL('./fixtures/kirchhoff-friction-boundary-cycle.json.gz', import.meta.url))),
        (_, value) => value === 'Infinity' ? Infinity : value === '-Infinity' ? -Infinity : value);
    const { count, band, groups } = source;
    const matrix = Float64Array.from(source.matrix), rhs = Float64Array.from(source.rhs);
    const lower = Float64Array.from(source.lower), upper = Float64Array.from(source.upper);
    const result = solveCoupledFrictionQP(matrix, rhs, lower, upper, count, band, groups, source.options);
    assert.ok(result.diagnostics.converged, JSON.stringify(result.diagnostics));
    // The held 100 mm catheter formerly cycled for all 80 iterations / 671
    // factors: every other Newton direction hit a released ellipse at step 0.
    assert.ok(result.diagnostics.iterations <= 14, JSON.stringify(result.diagnostics));
    assert.ok(result.diagnostics.factorizations <= 110, JSON.stringify(result.diagnostics));
    assert.equal(result.diagnostics.gradientFallbacks, 0);
    assert.ok(result.diagnostics.boundaryActivations > 0);
    const residual = rhs.slice(), x = result.increment;
    for (let i = 0; i < count; i++) {
        residual[i] -= matrix[i * band] * x[i];
        for (let j = Math.max(0, i - band + 1); j < i; j++) {
            const value = matrix[i * band + i - j];
            residual[i] -= value * x[j]; residual[j] -= value * x[i];
        }
        assert.ok(x[i] >= lower[i] && x[i] <= upper[i], `bound ${i}`);
    }
    const kkt = measureCoupledFrictionKKT(residual, x, lower, upper, groups);
    assert.ok(kkt.maximumResidual <= source.options.tolerance, JSON.stringify(kkt));
    for (const group of groups) assert.ok(Math.hypot(...group.rows.map((row, axis) =>
        (x[row] + group.lambda[axis]) / group.radii[axis])) <= 1 + 1e-12);
});

test('captured coupled friction solve retains its force solution without repeated remote working sets', () => {
    const source = JSON.parse(gunzipSync(fs.readFileSync(new URL('./fixtures/kirchhoff-coupled-friction-step3745.json.gz', import.meta.url))),
        (_, value) => value === 'Infinity' ? Infinity : value === '-Infinity' ? -Infinity : value);
    const { count, band, groups } = source;
    const matrix = new Float64Array(source.matrix), rhs = new Float64Array(source.rhs);
    const lower = new Float64Array(source.lower), upper = new Float64Array(source.upper);
    const result = solveCoupledFrictionQP(matrix, rhs, lower, upper, count, band, groups,
        { ...source.options, initialFree: new Uint8Array(source.initialFree) });
    assert.ok(result.diagnostics.converged, JSON.stringify(result.diagnostics));
    // Operation count, not a noisy wall-clock threshold. The previous solver
    // needed 51 factors, because reduced hints changed identity and its first
    // Newton target was cut to 9.69e-7 at the exact ellipse boundary.
    assert.ok(result.diagnostics.factorizations <= 16, JSON.stringify(result.diagnostics));
    assert.equal(result.diagnostics.gradientFallbacks, 0);
    const residual = rhs.slice(), x = result.increment;
    let objective = 0;
    for (let i = 0; i < count; i++) {
        residual[i] -= matrix[i * band] * x[i];
        for (let j = Math.max(0, i - band + 1); j < i; j++) {
            const value = matrix[i * band + i - j];
            residual[i] -= value * x[j]; residual[j] -= value * x[i];
        }
    }
    for (let i = 0; i < count; i++) {
        objective += x[i] * (-0.5 * residual[i] - 0.5 * rhs[i]);
        assert.ok(x[i] >= lower[i] && x[i] <= upper[i], `bound ${i}`);
        assert.ok(Math.abs(x[i] - source.referenceIncrement[i]) < 1e-7, `force increment ${i}`);
    }
    const kkt = measureCoupledFrictionKKT(residual, x, lower, upper, groups);
    assert.ok(kkt.maximumResidual < 1e-5, JSON.stringify(kkt));
    assert.ok(Math.abs(objective - source.diagnostics.objective) < 1e-12);
    for (const group of groups) {
        const norm = Math.hypot(...group.rows.map((row, axis) => (x[row] + group.lambda[axis]) / group.radii[axis]));
        assert.ok(norm <= 1 + 1e-12, `exact ellipse feasibility ${norm}`);
    }
});

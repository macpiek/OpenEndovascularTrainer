import test from 'node:test';
import assert from 'node:assert/strict';
import { solveCoulombNewton, solveSeededCoulombNewton } from '../src/physics/kirchhoffCoulombNewtonSolver.js';
import { measureCoupledLoadKKT } from '../src/physics/kirchhoffCoupledLoadSolver.js';

const tolerance = 1e-10;
function problem(rows, expected, residual, lower, groups = []) {
    return { matrix: Float64Array.from(rows.flat()), count: rows.length,
        rhs: Float64Array.from(rows, (row, i) => row.reduce((sum, a, j) => sum + a * expected[j], residual[i])),
        lower: Float64Array.from(lower), upper: new Float64Array(rows.length).fill(Infinity), groups };
}
function solve(p, options = {}) {
    // A row-major matrix has no band parameter; deliberately supply zero.
    return solveCoulombNewton(p.matrix, p.rhs, p.lower, p.upper, p.count, 0, p.groups,
        { matrixFormat: 'row-major', tolerance, ...options });
}
function auditOriginal(p, result, expected) {
    // Read every original ordered entry, without Newton's layout, scales,
    // Jacobian, regularization, or its computed residual and cone radii.
    const residual = Float64Array.from(p.rhs);
    for (let i = 0; i < p.count; i++) {
        for (let j = 0; j < p.count; j++) residual[i] -= p.matrix[i * p.count + j] * result.increment[j];
        assert.ok(Number.isFinite(result.increment[i]));
        assert.ok(result.increment[i] >= p.lower[i] && result.increment[i] <= p.upper[i]);
        assert.ok(Math.abs(residual[i] - result.residual[i]) < 1e-12);
        if (expected) assert.ok(Math.abs(result.increment[i] - expected[i]) < 1e-8, `force ${i}`);
    }
    const groups = p.groups.map(g => ({ ...g, radii: g.normalRow == null ? g.radii :
        g.mu.map(mu => mu * Math.max(0, g.normalLambda + result.increment[g.normalRow])) }));
    const kkt = measureCoupledLoadKKT(residual, result.increment, p.lower, p.upper, groups);
    assert.equal(result.diagnostics.maximumResidual, kkt.maximumResidual);
    if (result.diagnostics.converged) {
        assert.ok(kkt.maximumResidual <= tolerance, JSON.stringify(kkt));
        assert.ok(kkt.coneViolation <= 1e-9);
    }
    return kkt;
}
function frictionProblem() {
    // Physical normal 0, tangents 1/2, geometry normal 3. Both tangent
    // residuals see the geometry column, but only normal 0 loads the cone.
    // At the exact solution the total tangent force is (-.144, .32), on
    // the (.24, .4) ellipse; its residual (-.75, .6) is outward normal.
    return problem([[2, .2, .1, -.7], [.4, 2, .3, -.6], [-.1, .2, 1.5, .8], [2, .2, .1, 1.3]],
        [.6, -.154, .34, 1.4], [0, -.75, .6, 0], [-.2, -Infinity, -Infinity, 0],
        [{ rows: [1, 2], normalRow: 0, normalLambda: .2, lambda: [.01, -.02], mu: [.3, .5] }]);
}

for (const normalMap of ['projection', 'fischer-burmeister']) {
    test(`nonsymmetric LCP preserves ordered coefficients and complementarity (${normalMap})`, () => {
        const p = problem([[2, -1, .5], [3, 2, -.5], [-1, 4, 3]], [1, 0, 2], [0, -1, 0], [0, 0, 0]);
        const before = structuredClone(p), result = solve(p, { normalMap });
        assert.ok(result.diagnostics.converged, JSON.stringify(result.diagnostics));
        auditOriginal(p, result, [1, 0, 2]);
        assert.equal(result.increment[1], 0);
        assert.ok(Math.abs(result.residual[1] + 1) < tolerance);
        assert.equal(result.diagnostics.linearSolver, 'dense-lu');
        assert.equal(result.diagnostics.matrixFormat, 'row-major');
        assert.equal(result.diagnostics.normalLoadIterations, 0);
        assert.deepEqual(p, before);
    });

    test(`two-channel [[A,-B],[A,A-B]] normal Schur is not symmetrized (${normalMap})`, () => {
        // A=.5, B=.3, C=.2; physical and geometry force are both active.
        const p = problem([[.5, -.3], [.5, .2]], [2, 1], [0, 0], [0, 0]);
        const result = solve(p, { normalMap });
        assert.ok(result.diagnostics.converged, JSON.stringify(result.diagnostics));
        auditOriginal(p, result, [2, 1]);
    });

    test(`anisotropic Coulomb sees the bias column but uses only physical normal load (${normalMap})`, () => {
        const p = frictionProblem(), before = structuredClone(p), result = solve(p, { normalMap });
        assert.ok(result.diagnostics.converged, JSON.stringify(result.diagnostics));
        auditOriginal(p, result, [.6, -.154, .34, 1.4]);
        assert.ok(Math.abs(result.allGroups[0].radii[0] - .24) < tolerance);
        assert.ok(Math.abs(result.allGroups[0].radii[1] - .4) < tolerance);
        assert.deepEqual(p, before);
    });

    test(`infeasible nonsymmetric LCP remains explicitly uncertified (${normalMap})`, () => {
        // r0=1+x1 is positive for EVERY feasible x1>=0. No choice of x0
        // satisfies either the zero-force or positive-force normal KKT.
        const p = problem([[0, -1], [1, 0]], [0, 0], [1, 0], [0, 0]);
        const result = solve(p, { normalMap, maxCoulombNewtonIterations: 12 });
        assert.equal(result.diagnostics.converged, false);
        assert.notEqual(result.diagnostics.status, 'converged');
        assert.ok(auditOriginal(p, result).maximumResidual >= 1);
    });
}

test('nonsymmetric friction Jacobian includes every ordered response and bias derivative', () => {
    const p = frictionProblem(); let checked = false;
    solve(p, { normalMap: 'projection', initialIncrement: Float64Array.of(.3, -.12, .2, .4),
        maxCoulombNewtonIterations: 1,
        debugCoulombDirection({ x, J, evaluate, layout }) {
            assert.equal(layout, null);
            const h = 1e-6, plus = new Float64Array(p.count), minus = new Float64Array(p.count), residual = new Float64Array(p.count);
            for (let j = 0; j < p.count; j++) {
                const input = x.slice(); input[j] += h; evaluate(input, plus, residual);
                input[j] -= 2 * h; evaluate(input, minus, residual);
                for (let i = 0; i < p.count; i++)
                    assert.ok(Math.abs(J[i * p.count + j] - (plus[i] - minus[i]) / (2 * h)) < 1e-7, `Jacobian ${i},${j}`);
            }
            assert.equal(J[3], -.7, 'no transpose derivative of the friction radius in physical normal equilibrium');
            assert.equal(J[12], 2, 'geometry-to-physical response remains different from physical-to-geometry');
            assert.ok(Math.abs(J[7]) > .001 && Math.abs(J[11]) > .001, 'both tangents retain the bias response');
            checked = true;
        } });
    assert.ok(checked);
});

test('zero diagonal with nonzero row is not treated as zero mobility', () => {
    const p = problem([[0, 1], [2, 3]], [1, 0], [0, 0], [-Infinity, -Infinity]); let checked = false;
    const result = solve(p, { debugCoulombDirection({ iteration, J }) {
        if (iteration === 0) { assert.equal(J[0], 0); assert.equal(J[1], 1); checked = true; }
    } });
    assert.ok(checked); assert.ok(result.diagnostics.converged);
    auditOriginal(p, result, [1, 0]);
});

test('general operator uses positive projection scaling even with a negative diagonal', () => {
    const p = problem([[-1, 2], [3, 1]], [1, 2], [0, 0], [-Infinity, -Infinity]), result = solve(p);
    assert.ok(result.diagnostics.converged); auditOriginal(p, result, [1, 2]);
});

test('a feasible projected iterate must satisfy all original nonsymmetric rows', () => {
    const p = problem([[2, -1, .5], [3, 2, -.5], [-1, 4, 3]], [1, 0, 2], [0, -1, 0], [0, 0, 0]);
    const result = solve(p, { maxCoulombNewtonIterations: 0, initialIncrement: Float64Array.of(1, -1, 1) });
    assert.equal(result.diagnostics.converged, false);
    assert.ok(auditOriginal(p, result).maximumResidual > tolerance);
});

test('row-major results own force, residual, bounds and all numeric cone arrays across calls', () => {
    const p = frictionProblem(), options = { initialIncrement: Float64Array.of(.3, -.12, .2, .4) };
    const first = solve(p, options), snapshot = structuredClone(first);
    assert.ok(first.diagnostics.converged);
    p.matrix.fill(0); p.rhs.fill(0); p.lower.fill(-Infinity); p.upper.fill(0); options.initialIncrement.fill(0);
    for (const group of p.groups) for (const key of ['rows', 'lambda', 'mu']) group[key].fill(0);
    solve(frictionProblem(), options);
    assert.deepEqual(first, snapshot);
});

test('explicit matrix format rejects incompatible seed, band LU and malformed input before solving', () => {
    const p = frictionProblem(), args = [p.matrix, p.rhs, p.lower, p.upper, p.count, 0, p.groups];
    const options = { matrixFormat: 'row-major' };
    assert.throws(() => solveSeededCoulombNewton(...args, options), /fixed-load QP seed requires symmetric-band/);
    assert.equal(options.loadWorkspace, undefined, 'no SPD solver workspace was entered');
    assert.throws(() => solve(p, { coulombLinearSolver: 'band-lu' }), /require the dense LU/);
    assert.throws(() => solve(p, { matrixFormat: 'symmetric' }), /Unknown Coulomb matrixFormat/);
    assert.throws(() => solve({ ...p, matrix: p.matrix.subarray(1) }), /count\*count finite/);
    p.matrix[1] = NaN; assert.throws(() => solve(p), /count\*count finite/);
    p.matrix[1] = Infinity; assert.throws(() => solve(p), /count\*count finite/);
});

test('explicit symmetric-band format preserves default results and diagnostics exactly', () => {
    const args = [Float64Array.of(2, 0, 2, .4), Float64Array.of(1, -2), Float64Array.of(0, -Infinity),
        new Float64Array(2).fill(Infinity), 2, 2, []];
    for (const entry of [solveCoulombNewton, solveSeededCoulombNewton]) for (const coulombLinearSolver of [undefined, 'band-lu']) {
        const options = { tolerance, coulombLinearSolver };
        assert.deepEqual(entry(...args, { ...options, matrixFormat: 'symmetric-band' }), entry(...args, options));
    }
});

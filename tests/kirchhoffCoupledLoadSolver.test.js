import test from 'node:test';
import assert from 'node:assert/strict';
import { solveCoupledLoadQP } from '../src/physics/kirchhoffCoupledLoadSolver.js';

function problem(normalLambda = 0, lambda = [0, 0]) {
    return { matrix: new Float64Array([2, 0, 0, 2, .4, 0, 2, 0, 0]),
        rhs: new Float64Array([1 - 2 * normalLambda - .4 * lambda[0], -4 - .4 * normalLambda - 2 * lambda[0], -2 * lambda[1]]),
        lower: new Float64Array([-normalLambda, -Infinity, -Infinity]), upper: new Float64Array(3).fill(Infinity),
        groups: [{ rows: [1, 2], lambda, normalLambda, normalRow: 0, mu: [.2, .2] }] };
}
function solve(p, options = {}) { return solveCoupledLoadQP(p.matrix, p.rhs, p.lower, p.upper, 3, 3, p.groups,
    { tolerance: 1e-10, ...options }); }

for (const initial of [0, .3]) test(`normal load and sliding friction close simultaneously from old Fn=${initial}`, () => {
    const p = problem(initial, [-.01, .003]), before = structuredClone(p), result = solve(p);
    const normal = result.increment[0] + initial, tangent = result.increment[1] - .01;
    assert.ok(result.diagnostics.converged, JSON.stringify(result.diagnostics));
    assert.ok(result.diagnostics.normalLoadIterations > 1);
    assert.ok(Math.abs(normal - 1 / 1.92) < 1e-10);
    assert.ok(Math.abs(tangent + .2 * normal) < 1e-10);
    assert.ok(Math.abs(result.increment[2] + .003) < 1e-10);
    assert.deepEqual(p, before, 'dry solve preserves old force state');
});

test('zero friction and unloading recover exact point/interval bounds', () => {
    const p = problem(.3, [-.01, .003]); p.groups[0].mu = [0, .2];
    let r = solve(p);
    assert.ok(r.diagnostics.converged, JSON.stringify(r.diagnostics));
    assert.equal(r.increment[1], .01);
    p.rhs[0] = -1;
    r = solve(p);
    assert.ok(r.diagnostics.converged, JSON.stringify(r.diagnostics));
    assert.equal(r.increment[0], -.3);
    assert.equal(r.increment[1], .01); assert.equal(r.increment[2], -.003);
});

test('anisotropic final-load solution matches an independently manufactured KKT state', () => {
    const p = problem(), normal = .7, mu = [.15, .35], angle = .7;
    const expected = [normal, -mu[0] * normal * Math.cos(angle), -mu[1] * normal * Math.sin(angle)];
    const residual = [0, expected[1] / (mu[0] * normal) ** 2, expected[2] / (mu[1] * normal) ** 2];
    p.groups[0].mu = mu;
    p.rhs.set([2 * expected[0] + .4 * expected[1], .4 * expected[0] + 2 * expected[1] + residual[1], 2 * expected[2] + residual[2]]);
    const r = solve(p);
    assert.ok(r.diagnostics.converged, JSON.stringify(r.diagnostics));
    r.increment.forEach((value, axis) => assert.ok(Math.abs(value - expected[axis]) < 1e-9));
});

test('one load iteration cannot claim convergence from an initially unloaded cone', () => {
    const r = solve(problem(), { maxNormalLoadIterations: 1 });
    assert.equal(r.diagnostics.converged, false);
    assert.equal(r.diagnostics.status, 'normal-load-iteration-limit');
});

test('local working-set hints survive load changes without retaining force guesses', () => {
    const p = problem(.2, [-.04, .001]), guesses = [];
    const before = structuredClone(p);
    const r = solve(p, { reuseLoadHints: true, debugLoadIteration: data => guesses.push(data.options.initialFree) });
    assert.ok(r.diagnostics.converged);
    assert.equal(guesses[0], undefined);
    assert.ok(guesses.slice(1).some(guess => guess?.some(value => value !== 0)));
    assert.deepEqual(p, before);
    assert.ok(Math.abs(r.increment[0] + .2 - 1 / 1.92) < 1e-10);
});

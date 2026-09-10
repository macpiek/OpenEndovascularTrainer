import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { projectLoadEllipseDerivative, solveSeededCoulombNewton, solveCoulombNewton } from '../src/physics/kirchhoffCoulombNewtonSolver.js';
import { measureCoupledLoadKKT } from '../src/physics/kirchhoffCoupledLoadSolver.js';

for (const [u, v, load, mu] of [[3, 4, .3, [.2, .4]], [.08, .01, .3, [.2, .4]], [0, 3, .04, [.015, .006]],
    [.001, .002, .5, [.2, .3]], [.2, .3, .4, [0, .2]]])
    test(`projection Jacobian matches force/load finite differences at ${u},${v},${load}`, () => {
        const p = projectLoadEllipseDerivative(u, v, load, mu), h = 1e-7;
        for (let k = 0; k < 3; k++) {
            const input = [u, v, load]; input[k] += h;
            const a = projectLoadEllipseDerivative(...input, mu).value; input[k] -= 2 * h;
            const b = projectLoadEllipseDerivative(...input, mu).value;
            for (let axis = 0; axis < 2; axis++) {
                const derivative = k < 2 ? p.input[axis * 2 + k] : p.load[axis];
                assert.ok(Math.abs((a[axis] - b[axis]) / (2 * h) - derivative) < 1e-7);
            }
        }
    });

test('zero-load projection has the correct one-sided loading response', () => {
    const p = projectLoadEllipseDerivative(2, -3, 0, [.015, .006]);
    assert.ok(p.value.every(value => value === 0)); assert.deepEqual(Array.from(p.input), [0, 0, 0, 0]);
    const epsilon = 1e-8, next = projectLoadEllipseDerivative(2, -3, epsilon, [.015, .006]);
    for (let i = 0; i < 2; i++) assert.ok(Math.abs(next.value[i] / epsilon - p.load[i]) < 1e-9);
});

function problem(normalLambda = 0, lambda = [0, 0]) {
    return { matrix: new Float64Array([2, 0, 0, 2, .4, 0, 2, 0, 0]),
        rhs: new Float64Array([1 - 2 * normalLambda - .4 * lambda[0], -4 - .4 * normalLambda - 2 * lambda[0], -2 * lambda[1]]),
        lower: new Float64Array([-normalLambda, -Infinity, -Infinity]), upper: new Float64Array(3).fill(Infinity),
        groups: [{ rows: [1, 2], lambda, normalLambda, normalRow: 0, mu: [.2, .2] }] };
}
const solve = p => solveSeededCoulombNewton(p.matrix, p.rhs, p.lower, p.upper, 3, 3, p.groups, { tolerance: 1e-10 });

for (const oldLoad of [0, .3]) test(`simultaneous Coulomb solve preserves the non-associated normal equation from Fn=${oldLoad}`, () => {
    const p = problem(oldLoad, [-.01, .003]), before = structuredClone(p), r = solve(p);
    assert.ok(r.diagnostics.converged, JSON.stringify(r.diagnostics));
    assert.ok(Math.abs(r.increment[0] + oldLoad - 1 / 1.92) < 1e-9, 'no radius-transpose/dilation in normal equilibrium');
    assert.ok(Math.abs(r.increment[1] - .01 + .2 / 1.92) < 1e-9);
    assert.ok(Math.abs(r.increment[2] + .003) < 1e-9);
    assert.deepEqual(p, before, 'the seed and Newton iterates never mutate physical input forces');
});

test('unloading and a zero friction axis preserve exact force feasibility', () => {
    const p = problem(.3, [-.01, .003]); p.groups[0].mu = [0, .2];
    let r = solve(p); assert.ok(r.diagnostics.converged, JSON.stringify(r.diagnostics)); assert.equal(r.increment[1], .01);
    p.rhs[0] = -1; r = solve(p);
    assert.ok(r.diagnostics.converged, JSON.stringify(r.diagnostics));
    assert.equal(r.increment[0], -.3); assert.equal(r.increment[1], .01); assert.equal(r.increment[2], -.003);
});

test('seeded Newton closes a captured alternating contact load without repeated fixed-load solves', () => {
    const p = JSON.parse(gunzipSync(fs.readFileSync(new URL('./fixtures/kirchhoff-condensed-load-cycle.json.gz', import.meta.url))),
        (_, v) => v === 'Infinity' ? Infinity : v === '-Infinity' ? -Infinity : v);
    const args = ['matrix', 'rhs', 'lower', 'upper'].map(key => Float64Array.from(p[key]));
    const r = solveSeededCoulombNewton(...args, p.count, p.band, p.groups, p.options);
    assert.ok(r.diagnostics.converged, JSON.stringify(r.diagnostics));
    assert.ok(r.diagnostics.factorizations <= 12); assert.equal(r.diagnostics.normalLoadIterations, 1);
    const residual = args[1].slice(), x = r.increment;
    for (let i = 0; i < p.count; i++) {
        residual[i] -= args[0][i * p.band] * x[i];
        for (let j = Math.max(0, i - p.band + 1); j < i; j++) {
            const a = args[0][i * p.band + i - j]; residual[i] -= a * x[j]; residual[j] -= a * x[i];
        }
        assert.ok(x[i] >= args[2][i] && x[i] <= args[3][i], `bound ${i}`);
    }
    const kkt = measureCoupledLoadKKT(residual, x, args[2], args[3], r.allGroups);
    assert.ok(kkt.maximumResidual <= p.options.tolerance, JSON.stringify(kkt));
    assert.ok(kkt.coneViolation <= 1e-9);
});

test('normal complementarity crosses a former min-map switching-plane stall', () => {
    const p = JSON.parse(gunzipSync(fs.readFileSync(new URL('./fixtures/kirchhoff-coulomb-switching-plane.json.gz', import.meta.url))),
        (_, v) => v === 'Infinity' ? Infinity : v === '-Infinity' ? -Infinity : v);
    const args = ['matrix', 'rhs', 'lower', 'upper'].map(key => Float64Array.from(p[key]));
    // Ask explicitly for the accuracy this regression asserts. Active-bound
    // recovery can otherwise certify an earlier valid 2e-4 fixture iterate.
    const r = solveCoulombNewton(...args, p.count, p.band, p.groups, { ...p.options, tolerance: 1e-5 });
    assert.ok(r.diagnostics.converged, JSON.stringify(r.diagnostics));
    assert.ok(r.diagnostics.factorizations <= 8); assert.ok(r.diagnostics.backtracks <= 6);
    assert.ok(r.diagnostics.maximumResidual < 1e-5);
    for (let i = 0; i < p.count; i++) assert.ok(r.increment[i] >= args[2][i] && r.increment[i] <= args[3][i]);
    for (const group of r.allGroups) {
        const load = group.normalLambda + r.increment[group.normalRow]; assert.ok(load >= 0);
        const force = group.rows.map((row, axis) => r.increment[row] + group.lambda[axis]);
        if (load === 0) assert.ok(force.every(value => value === 0));
        else assert.ok(Math.hypot(...force.map((value, axis) => value / (group.mu[axis] * load))) <= 1 + 1e-12);
    }
});

test('natural-bound recovery closes the captured 105.2 mm hinted branch without a cold retry', () => {
    const p = JSON.parse(gunzipSync(fs.readFileSync(new URL('./fixtures/kirchhoff-coulomb-hinted-seed.json.gz', import.meta.url))),
        (_, v) => v === 'Infinity' ? Infinity : v === '-Infinity' ? -Infinity : v);
    const args = ['matrix', 'rhs', 'lower', 'upper'].map(key => Float64Array.from(p[key]));
    const r = solveSeededCoulombNewton(...args, p.count, p.band, p.groups, { ...p.options, retryWithoutHints: false });
    assert.ok(r.diagnostics.converged, JSON.stringify(r.diagnostics));
    assert.ok(r.diagnostics.boundRecoveries > 0);
    assert.equal(r.diagnostics.coldSeedRetry, undefined);
    assert.equal(r.diagnostics.normalLoadIterations, 1);
    const residual = args[1].slice(), x = r.increment;
    for (let i = 0; i < p.count; i++) {
        residual[i] -= args[0][i * p.band] * x[i];
        for (let j = Math.max(0, i - p.band + 1); j < i; j++) {
            const a = args[0][i * p.band + i - j]; residual[i] -= a * x[j]; residual[j] -= a * x[i];
        }
        assert.ok(x[i] >= args[2][i] && x[i] <= args[3][i], `bound ${i}`);
    }
    const kkt = measureCoupledLoadKKT(residual, x, args[2], args[3], r.allGroups);
    assert.ok(kkt.maximumResidual <= p.options.tolerance, JSON.stringify(kkt));
    assert.ok(kkt.coneViolation <= 1e-9);
});

test('a reduced-system candidate cannot bypass the original-equation acceptance gate',()=>{
    let checks=0;
    const result=solveCoulombNewton(new Float64Array([1]),new Float64Array([1]),
        new Float64Array([-Infinity]),new Float64Array([Infinity]),1,1,[],
        {tolerance:1e-8,maxCoulombNewtonIterations:2,acceptCandidate:()=>{checks++;return false;}});
    assert.ok(checks>0);
    assert.equal(result.diagnostics.converged,false);
});

test('zero-diagonal mixed wall restores exactly zero force on the separated natural-map branch',()=>{
    const result=solveCoulombNewton(Float64Array.from([1,-1,1,0]),Float64Array.from([0,-.1]),
        Float64Array.from([-Infinity,0]),Float64Array.from([Infinity,Infinity]),2,2,[],
        {matrixFormat:'row-major',tolerance:1e-10,maxCoulombNewtonIterations:0,initialIncrement:Float64Array.from([0,3e-18])});
    assert.equal(result.diagnostics.converged,true,JSON.stringify(result.diagnostics));
    assert.equal(result.increment[1],0);
    assert.equal(result.diagnostics.boundRecoveries,1);
    assert.ok(result.diagnostics.maximumResidual<=1e-10);
});

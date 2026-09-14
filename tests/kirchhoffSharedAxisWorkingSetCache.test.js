import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { createSharedAxisLinear, solveSharedAxisLinear, iterateSharedAxisLinear } from '../src/physics/kirchhoffSharedAxisLinear.js';
import { advanceSharedAxis } from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import { loadCoupledRuntimeAnatomy } from './helpers/coupledRuntimeFixture.js';
import { captureSharedAxisReplay, restoreSharedAxisReplay } from './helpers/sharedAxisReplay.js';

function independent(n = 12) {
    const layout = { dofCount: n, band: 1 }, rows = Array.from({ length: n }, (_, i) => ({
        id: i, kind: 'wall', dofs: [i], jacobian: [1], multiplier: 0, gap: -1 - i / n }));
    return { layout, rows, chain: { layout, hessian: new Float64Array(n).fill(1) },
        gradient: new Float64Array(n), fixed: new Uint8Array(n) };
}
const output = r => ({ increment: Array.from(r.increment), multiplierIncrement: Array.from(r.multiplierIncrement),
    converged: r.converged, residual: r.residual, failure: r.failure });

test('reused compact maps refresh coefficients, masks and extra-force supports and match a fresh reference',()=>{
    const f=independent(12),w=createSharedAxisLinear(f.layout,f.rows,{lazy:true});
    for(let trial=0;trial<32;trial++) {
        for(let i=0;i<f.rows.length;i++) {
            const r=f.rows[i];r.gap=trial%3===0&&i%2?2:-.5-i/20;r.multiplier=(trial+i)%3?.1:0;
            r.jacobian[0]=1+trial/32;
            r.extraForceDofs=[(i+trial)%12];r.extraForceJacobian=[.01];
        }
        f.chain.hessian.fill(1+trial/10);f.gradient.fill(-trial/100);
        const expected=solveSharedAxisLinear(createSharedAxisLinear(f.layout,f.rows,{lazy:true}),f.chain,{...f,reuseStructure:false,trace:[]});
        const actual=solveSharedAxisLinear(w,f.chain,{...f,reuseStructure:true,trace:[]});
        assert.deepEqual(output(actual),output(expected));
        for(const key of ['factorizations','activeSetAttempts','workingSetReuses','batchFallback'])assert.equal(actual[key],expected[key]);
        assert.ok(actual.converged);
    }
});

test('fallback reuses certified results with owned solution copies and counts only actual factorizations', () => {
    const f = independent(), results = [];
    for (const reuseWorkingSet of [false, true]) {
        const w = createSharedAxisLinear(f.layout, f.rows), solve = w.lu.solve.bind(w.lu); let calls = 0;
        w.lu.solve = (...args) => ++calls === 2 ? false : solve(...args);
        const before = structuredClone(f), trace = [];
        const r = solveSharedAxisLinear(w, f.chain, { ...f, compactWorkingSet: false, reuseWorkingSet, trace });
        assert.equal(r.converged, true); assert.equal(r.batchFallback, true); assert.equal(r.batchFailure, 'band-lu-rejected');
        assert.equal(r.factorizations, calls); assert.deepEqual(f, before);
        results.push({ physical: output(r), calls, trace, reuses: r.workingSetReuses });
    }
    assert.deepEqual(results[0].physical, results[1].physical);
    assert.deepEqual(results[0].trace, results[1].trace, 'Reuse does not alter basis pivots or activation decisions');
    assert.equal(results[1].reuses, 1); assert.equal(results[1].calls, results[0].calls - 1);
});

test('cancelling a linear call discards its cached solutions before changed coefficients use the same workspace', () => {
    const f = independent(), w = createSharedAxisLinear(f.layout, f.rows), solve = w.lu.solve.bind(w.lu); let calls = 0;
    w.lu.solve = (...args) => ++calls === 2 ? false : solve(...args);
    const iterator = iterateSharedAxisLinear(w, f.chain, { ...f, compactWorkingSet: false });
    assert.equal(iterator.next().value.batchSize, 8);
    assert.equal(iterator.next().value.attempt, 1); // The first solution is now cached.
    assert.equal(iterator.next().value.batchSize, 1); // Failed batch enters reference fallback.
    assert.equal(iterator.return().done, true);
    f.chain.hessian.fill(2); f.gradient.fill(-.3); f.rows.forEach(r => { r.gap -= .2; });
    const r = solveSharedAxisLinear(w, f.chain, { ...f, compactWorkingSet: false });
    const reference = solveSharedAxisLinear(createSharedAxisLinear(f.layout, f.rows), f.chain,
        { ...f, compactWorkingSet: false, reuseWorkingSet: false });
    assert.equal(r.converged, true); assert.equal(r.workingSetReuses, 0);
    assert.deepEqual(output(r), output(reference));
    assert.equal(r.factorizations, reference.factorizations);
});

test('seeded constrained systems retain exact directions, residuals and pivot paths with or without the call-local cache', () => {
    for (let trial = 1; trial <= 60; trial++) {
        let seed = trial;
        const rng = () => { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed / 2 ** 32; };
        const n = 5, layout = { dofCount: n, band: n }, dofs = Array.from({ length: n }, (_, i) => i), target = dofs.map(() => 2 * rng() - 1);
        const rows = Array.from({ length: 16 }, (_, id) => {
            const jacobian = dofs.map(() => 2 * rng() - 1);
            return { id, kind: 'wall', dofs, jacobian, multiplier: 0,
                gap: -jacobian.reduce((v, x, i) => v + x * target[i], 0) + rng() * .2 };
        });
        const hessian = new Float64Array(n * n); for (let i = 0; i < n; i++) hessian[i * n] = 1;
        const options = { rows, gradient: Float64Array.from(dofs, () => rng() * 2 - 1), fixed: new Uint8Array(n) };
        const results = [false, true].map(reuseWorkingSet => {
            const trace = [], r = solveSharedAxisLinear(createSharedAxisLinear(layout, rows, { lazy: true }), { layout, hessian },
                { ...options, trace, reuseWorkingSet });
            assert.equal(r.converged, true, `seed ${trial}`);
            return { physical: output(r), trace };
        });
        assert.deepEqual(results[0], results[1], `seed ${trial}`);
    }
});

test('captured pigtail withdrawal preserves the complete accepted physical state while reducing LU work', async () => {
    const fixture = JSON.parse(readFileSync(new URL('./fixtures/shared-axis/anatomy-pigtail-withdraw-184.73-live-wall-incoming.json', import.meta.url)));
    const anatomy = await loadCoupledRuntimeAnatomy();
    try {
        const outcomes = [];
        for (const reuseWorkingSet of [false, true]) {
            const state = restoreSharedAxisReplay(fixture, anatomy.field), request = fixture.stepRequest;
            const iterator = advanceSharedAxis(state, request.rotations, request.dt, request.tools,
                { ...request.options, liveWallNormalLoad: true, reuseWorkingSet });
            let next; do { next = iterator.next(); } while (!next.done);
            assert.ok(next.value.state, JSON.stringify(next.value.result));
            assert.equal(next.value.result.converged, true);
            outcomes.push({ state: captureSharedAxisReplay(next.value.state, fixture.sheath), result: next.value.result });
        }
        assert.deepEqual(outcomes[1].state, outcomes[0].state, 'Positions, frames, reactions, velocity, friction history and topology are unchanged');
        for (const key of ['quality', 'residual', 'certificateBound', 'wallNormalFallbacks', 'iterations', 'backtracks', 'geometryRestarts', 'frictionIterations', 'substepAttempts'])
            assert.deepEqual(outcomes[1].result[key], outcomes[0].result[key], key);
        assert.ok(outcomes[1].result.factorizations < outcomes[0].result.factorizations,
            `${outcomes[1].result.factorizations} cached vs ${outcomes[0].result.factorizations} reference factorizations`);
    } finally { anatomy.dispose(); }
});

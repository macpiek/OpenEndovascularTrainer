import assert from 'node:assert/strict';
import test from 'node:test';
import { makeFixture, preparedOptions, advance } from '../scripts/physics/helpers/compositeTimeStepBenchmark.js';
import { createCompositeTimeStepWorkspace } from '../src/physics/kirchhoffCompositeTimeStep.js';
import { createCompositeChainWorkspace } from '../src/physics/kirchhoffCompositeChain.js';
import { createCompositeInertiaCache } from '../src/physics/kirchhoffCompositeInertiaCache.js';

const accept = result => assert.ok(result.accepted, JSON.stringify({ status: result.status, ...result.diagnostics }));
const options = (f, state = f.state) => ({ ...preparedOptions(f, state, 'strict-tests'),
    elementBackend: 'wasm-exact', constraintSolver: 'mixed' });
function samePhysics(a, b) {
    assert.equal(a.accepted, b.accepted); assert.equal(a.status, b.status); assert.deepEqual(a.state, b.state);
    for (const key of ['certificate', 'originalResidual', 'reactions', 'elasticEnergy', 'inertialEnergy', 'kineticEnergy', 'objective',
        'directions', 'linearSolves', 'factorizations', 'refinementSteps', 'wallQueries', 'acceptedTrials', 'rejectedTrials',
        'historyCommits', 'dualUpdates', 'wallDualUpdates', 'lineSearchTrials', 'outerIterations'])
        assert.deepEqual(a.diagnostics[key], b.diagnostics[key], key);
}

for (const scenario of ['contact-free', 'analytic-plane']) for (const insertion of [9, 160, 310]) {
    test(`lazy exact tangent preserves full-solve states, force certificates and history (${scenario}, ${insertion}mm)`, () => {
        const f = makeFixture(insertion, scenario), workspace = createCompositeTimeStepWorkspace(f.state.layout, { elementBackend: 'wasm-exact' });
        let state = f.state;
        for (let step = 0; step < 3; step++) {
            const opts = options(f, state), before = JSON.stringify(state);
            const full = advance(state, { ...opts, assemblyPolicy: 'full' }); accept(full);
            const lazy = advance(state, { ...opts, workspace, assemblyPolicy: 'lazy' }); accept(lazy); samePhysics(lazy, full);
            const certificateOnly = advance(state, { ...opts, workspace });
            accept(certificateOnly); samePhysics(certificateOnly, full);
            assert.equal(JSON.stringify(state), before);
            assert.equal(lazy.diagnostics.fullAssemblies, lazy.diagnostics.directions);
            assert.equal(lazy.diagnostics.tangentRebuilds, lazy.diagnostics.directions);
            assert.equal(lazy.diagnostics.gradientAssemblies, full.diagnostics.evaluations);
            assert.equal(lazy.diagnostics.evaluations, lazy.diagnostics.gradientAssemblies + lazy.diagnostics.fullAssemblies);
            assert.equal(certificateOnly.diagnostics.gradientAssemblies, 1);
            assert.equal(certificateOnly.diagnostics.fullAssemblies, full.diagnostics.fullAssemblies - 1);
            assert.ok(lazy.diagnostics.fullAssemblies < full.diagnostics.fullAssemblies);
            state = lazy.state;
        }
    });
}

test('a tangent rebuild consumes the same global assembly budget and a rejected attempt never commits', () => {
    const f = makeFixture(310, 'contact-free'), opts = options(f), before = JSON.stringify(f.state);
    const workspace = createCompositeTimeStepWorkspace(f.state.layout, { elementBackend: 'wasm-exact' });
    const r = advance(f.state, { ...opts, workspace, assemblyPolicy: 'lazy', budget: { ...opts.budget, evaluations: 2 } });
    assert.equal(r.accepted, false); assert.equal(r.status, 'evaluation-budget-exhausted'); assert.equal(r.state, f.state);
    assert.equal(r.diagnostics.evaluations, 2); assert.equal(r.diagnostics.gradientAssemblies, 1);
    assert.equal(r.diagnostics.fullAssemblies, 1); assert.equal(r.diagnostics.tangentRebuilds, 1);
    assert.equal(r.diagnostics.historyCommits, 0); assert.equal(JSON.stringify(f.state), before);
    const retried = advance(f.state, { ...opts, workspace, assemblyPolicy: 'lazy' }); accept(retried);
    const full = advance(f.state, { ...opts, assemblyPolicy: 'full' }); accept(full); samePhysics(retried, full);
});

test('compiled inertia gradient mode matches original energy/force while ignoring poisoned stale matrix bytes', () => {
    const f = makeFixture(160, 'contact-free'), opts = options(f), layout = f.state.layout;
    const cache = createCompositeInertiaCache({ layout, coordinates: f.state.data.coordinates,
        previousPositions: f.state.data.positions, dt: opts.dt, inertiaEdges: opts.inertiaEdges });
    const full = createCompositeChainWorkspace(layout), gradient = createCompositeChainWorkspace(layout);
    full.energy = gradient.energy = 3; full.gradient.fill(.2); gradient.gradient.fill(.2); gradient.hessian.fill(NaN);
    const positions = f.state.data.positions.map(p => [p[0] + .01, p[1] - .007, p[2] + .009]);
    const a = cache.append(positions, full), expected = { energy: a.energy, kineticEnergy: a.kineticEnergy,
        gradient: a.gradient.slice(), kineticGradient: a.kineticGradient.slice(), velocities: a.velocities.slice() };
    const b = cache.append(positions, gradient, { order: 'gradient' });
    for (const key of Object.keys(expected)) assert.deepEqual(b[key], expected[key], key);
    assert.equal(gradient.energy, full.energy); assert.deepEqual(gradient.gradient, full.gradient);
    assert.ok(gradient.hessian.every(Number.isNaN)); assert.equal(gradient.hessianValid, false);
    assert.throws(() => cache.append(positions, gradient), /Nonfinite/);
    assert.throws(() => cache.append(positions, gradient, { order: 'unknown' }), /order/);
});

test('automatic assembly retains the explicit full path for unsupported backends', () => {
    const f = makeFixture(9, 'contact-free'), opts = { ...options(f), elementBackend: 'wasm' };
    const r = advance(f.state, opts); accept(r);
    assert.equal(r.diagnostics.gradientAssemblies, 0); assert.equal(r.diagnostics.tangentRebuilds, 0);
    assert.equal(r.diagnostics.trialAssembly, 'full'); assert.equal(r.diagnostics.certificateAssembly, 'full');
    assert.throws(() => advance(f.state, { ...opts, assemblyPolicy: 'unknown' }), /assemblyPolicy/);
});

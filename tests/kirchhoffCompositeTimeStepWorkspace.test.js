import assert from 'node:assert/strict';
import test from 'node:test';
import { makeFixture, preparedOptions, advance } from '../scripts/physics/helpers/compositeTimeStepBenchmark.js';
import { createCompositeTimeStepWorkspace } from '../src/physics/kirchhoffCompositeTimeStep.js';

const accept = result => assert.ok(result.accepted, JSON.stringify({ status: result.status, ...result.diagnostics }));
const options = (fixture, state = fixture.state, elementBackend = 'wasm') => ({
    ...preparedOptions(fixture, state, 'strict-tests'), constraintSolver: 'mixed', elementBackend
});
function sameResult(actual, expected) {
    assert.equal(actual.accepted, expected.accepted); assert.equal(actual.status, expected.status);
    assert.deepEqual(actual.state, expected.state);
    const { reusedWorkspace: _a, ...a } = actual.diagnostics;
    const { reusedWorkspace: _b, ...b } = expected.diagnostics;
    assert.deepEqual(a, b);
}

for (const backend of ['wasm', 'wasm-exact']) for (const scenario of ['contact-free', 'analytic-plane']) {
    test(`persistent scratch preserves every accepted physical value and proof (${backend}, ${scenario})`, () => {
        const f = makeFixture(310, scenario), workspace = createCompositeTimeStepWorkspace(f.state.layout, { elementBackend: backend });
        let state = f.state;
        for (let step = 0; step < 2; step++) {
            const opts = options(f, state, backend), saved = JSON.stringify(state);
            const fresh = advance(state, opts), cached = advance(state, { ...opts, workspace });
            accept(fresh); accept(cached); sameResult(cached, fresh);
            assert.equal(JSON.stringify(state), saved);
            state = cached.state;
        }
        assert.equal(workspace.diagnostics.calls, 2); assert.equal(workspace.diagnostics.chainBuilds, 1);
        assert.ok(workspace.diagnostics.mixedHits > 0);
        assert.ok(workspace.diagnostics.retainedMixedSystems <= workspace.diagnostics.mixedCacheCapacity);
    });
}

test('an interrupted solve leaves no staged contact, inertia or factor in the next prepared retry', () => {
    const f = makeFixture(160, 'analytic-plane'), opts = options(f), workspace = createCompositeTimeStepWorkspace(f.state.layout);
    const saved = JSON.stringify(f.state);
    const rejected = advance(f.state, { ...opts, workspace, budget: { ...opts.budget, evaluations: 2 } });
    assert.equal(rejected.accepted, false); assert.equal(rejected.state, f.state);
    assert.equal(rejected.diagnostics.historyCommits, 0); assert.ok(rejected.diagnostics.directions > 0);
    assert.equal(JSON.stringify(f.state), saved);
    const fresh = advance(f.state, opts), retried = advance(f.state, { ...opts, workspace });
    accept(retried); sameResult(retried, fresh);
});

test('owned output and diagnostics survive later calls and cannot mutate the private workspace', () => {
    const f = makeFixture(9, 'contact-free'), opts = options(f), workspace = createCompositeTimeStepWorkspace(f.state.layout);
    const first = advance(f.state, { ...opts, workspace }); accept(first);
    const saved = JSON.stringify(first), next = advance(first.state, { ...options(f, first.state), workspace }); accept(next);
    assert.equal(JSON.stringify(first), saved);
    first.state.data.positions[0][0] += 123;
    first.state.layout.positions[0] = 42;
    first.state.layout.edgeToolIds[0].reverse();
    first.diagnostics.originalResidual.fill(123);
    const fresh = advance(f.state, opts), cached = advance(f.state, { ...opts, workspace });
    accept(cached); sameResult(cached, fresh);
});

test('topology/backend mismatches and failed argument validation do not poison a reusable workspace', () => {
    const f = makeFixture(9, 'contact-free'), opts = options(f), workspace = createCompositeTimeStepWorkspace(f.state.layout);
    assert.throws(() => advance(f.state, { ...opts, workspace: {} }), /workspace/);
    assert.throws(() => advance(f.state, { ...opts, workspace, elementBackend: 'wasm-exact' }), /backend/);
    const other = makeFixture(310, 'contact-free');
    assert.throws(() => advance(other.state, { ...options(other), workspace }), /topology/);
    assert.throws(() => advance(f.state, { ...opts, workspace, dt: 0 }), /positive/);
    accept(advance(f.state, { ...opts, workspace }));
    assert.throws(() => createCompositeTimeStepWorkspace(f.state.layout, { mixedCacheCapacity: 0 }), /1..4/);
});

test('nested material callbacks cannot reuse an in-flight numeric workspace', () => {
    const f = makeFixture(9, 'contact-free'), workspace = createCompositeTimeStepWorkspace(f.state.layout), opts = options(f);
    const tool = f.state.data.tools[0], original = tool.materialAt; let nested = 0;
    tool.materialAt = args => {
        assert.throws(() => advance(f.state, { ...opts, workspace }), /reentrantly/); nested++;
        return original(args);
    };
    accept(advance(f.state, { ...opts, workspace })); assert.ok(nested > 0);
    tool.materialAt = original;
    accept(advance(f.state, { ...opts, workspace }));
});

test('changing wall patterns keeps bounded storage without suppressing a constraint or reusing its numerical factor', () => {
    const f = makeFixture(310, 'analytic-plane'), workspace = createCompositeTimeStepWorkspace(f.state.layout, { mixedCacheCapacity: 1 });
    for (let pattern = 0; pattern < 3; pattern++) {
        const opts = options(f);
        opts.wall.contactOwners = structuredClone(opts.wall.contactOwners);
        for (const edge of opts.wall.contactOwners.edges) if (edge.edge % 3 !== pattern) edge.wall = null;
        // An open unloaded wall has no mixed unknown after exact inactive
        // elimination. Start this storage test with genuinely active rows so
        // each ownership pattern requires a distinct symbolic mixed layout.
        opts.initialGuess = new Float64Array(f.state.layout.dofCount);
        for (let node = 0; node < f.state.layout.nodeCount; node++) {
            const first = f.state.layout.positions[node];
            for (let axis = 0; axis < 3; axis++) opts.initialGuess[first + axis] = f.state.data.positions[node][axis];
            opts.initialGuess[first + 1] = .79;
        }
        opts.budget.linearSolves = 1;
        const fresh = advance(f.state, opts), cached = advance(f.state, { ...opts, workspace });
        sameResult(cached, fresh);
        assert.equal(workspace.diagnostics.retainedMixedSystems, 1);
    }
    assert.equal(workspace.diagnostics.chainBuilds, 1); assert.equal(workspace.diagnostics.wallBuilds, 1);
    assert.ok(workspace.diagnostics.mixedBuilds >= 3);
});

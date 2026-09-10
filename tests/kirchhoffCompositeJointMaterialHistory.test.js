import assert from 'node:assert/strict';
import test from 'node:test';
import { createCompositeJointMaterialHistory } from '../src/physics/kirchhoffCompositeJointMaterialHistory.js';
import { createCompositeChainLayout } from '../src/physics/kirchhoffCompositeChain.js';
import { captureCompositeReferenceFrames, compileCompositeMaterial } from '../src/physics/kirchhoffCompositeElement.js';
import { createCompositeJointTimeStepState, advanceCompositeJointTimeStep } from '../src/physics/kirchhoffCompositeJointTimeStep.js';
const span = (id, sStart, sEnd, velocities) => ({ id, sStart, sEnd, velocities, interpretation: 'physical-material-velocity', angularVelocity: null, materialSpin: null, frameSpin: null });
const entry = (edge, ...tools) => ({ edge, tools });
const map = (id, sStart, dsDx = 1, dsDt = 0) => ({ id, materialMap: { sStart, dsDx, dsDt } });
const close = (a, b, tolerance = 1e-12) => assert.ok(Number.isFinite(a) && Number.isFinite(b) && Math.abs(a - b) <= tolerance, `${a} != ${b}`);
const vectorClose = (a, b, tolerance) => a.forEach((v, k) => close(v, b[k], tolerance));

test('opposite feeds sample each own current material label exactly once, independently of label rate', () => {
    const h = createCompositeJointMaterialHistory({ materialVelocities: [entry(9,
        span('wire', 0, 20, [[0, 0, 1], [20, 40, 1]]), span('catheter', 100, 120, [[-100, 7, 0], [-120, 7, 0]]))] });
    const prepared = h.prepare({ coordinates: [0, 2], inertiaEdges: [{ tools: [map('wire', 3.25, 1.5, -100), map('catheter', 103.6, .5, 200)] }] });
    const [w, c] = prepared.inertiaEdges[0].tools;
    vectorClose(w.oldMaterialVelocities[0], [3.25, 6.5, 1]); vectorClose(w.oldMaterialVelocities[1], [6.25, 12.5, 1]);
    vectorClose(c.oldMaterialVelocities[0], [-103.6, 7, 0]); vectorClose(c.oldMaterialVelocities[1], [-104.6, 7, 0]);
    for (const t of [w, c]) for (const sample of t.pieces[0].samples) vectorClose(sample.oldMaterialVelocity, h.sample(t.id, sample.s));
    assert.equal(prepared.requiresSubdivision, false); assert.equal(w.angularVelocity, null); assert.equal(c.materialSpin, null); assert.equal(prepared.includesAngularHistory, false);
});

test('nonunit maps and nonuniform mesh split all crossed old edge fields by material label, not source edge index', () => {
    const h = createCompositeJointMaterialHistory({ materialVelocities: [entry(30, span('wire', 2, 6, [[2, 0, 0], [10, 0, 0]])),
        entry(2, span('wire', 0, 2, [[0, 0, 0], [2, 0, 0]]))] });
    const prepared = h.prepare({ coordinates: [5, 6.5, 7], inertiaEdges: [{ tools: [map('wire', .5, 2, [-.2, .4])] }, { tools: [map('wire', 3.5, 2)] }] });
    const [first, second] = prepared.inertiaEdges.map(e => e.tools[0]);
    assert.equal(first.pieces.length, 2); assert.equal(second.pieces.length, 1); assert.equal(first.oldMaterialVelocities, undefined);
    assert.deepEqual(first.pieces.map(p => p.sourceEdge), [2, 30]); assert.equal(first.compatibleWithAffineEdgeOperator, false);
    assert.deepEqual(first.requiredCuts, [{ edge: 0, toolId: 'wire', s: 2, x: 5.75, reason: 'own-affine-history-boundary' }]);
    assert.deepEqual(first.materialMap.dsDt, [-.2, .4]); vectorClose(first.pieces[0].oldMaterialVelocities[1], [2, 0, 0]);
    vectorClose(first.pieces[1].oldMaterialVelocities[1], [5, 0, 0]); vectorClose(second.oldMaterialVelocities[1], [7, 0, 0]);
    close(first.pieces.flatMap(p => p.samples).reduce((sum, s) => sum + s.materialWeight, 0), 3);
});

test('discontinuous velocities require explicit one-sided traces and piecewise quadrature preserves physical integrals', () => {
    const h = createCompositeJointMaterialHistory({ materialVelocities: [entry(0, span('wire', 0, 1, [[1, 0, 0], [1, 0, 0]])),
        entry(1, span('wire', 1, 3, [[3, 0, 0], [3, 0, 0]]))] });
    assert.throws(() => h.sample('wire', 1), { code: 'ambiguous-material-history-trace' });
    assert.deepEqual(h.sample('wire', 1, { trace: 'left' }), [1, 0, 0]); assert.deepEqual(h.sample('wire', 1, { trace: 'right' }), [3, 0, 0]);
    const r = h.prepare({ coordinates: [10, 12], inertiaEdges: [{ tools: [map('wire', 0, 1.5)] }] }), t = r.inertiaEdges[0].tools[0], samples = t.pieces.flatMap(p => p.samples);
    close(r.requiredCuts[0].x, 10 + 2 / 3); assert.equal(t.oldMaterialVelocities, undefined);
    close(samples.reduce((s, p) => s + p.materialWeight * p.oldMaterialVelocity[0], 0), 7);
    close(samples.reduce((s, p) => s + .5 * p.materialWeight * p.oldMaterialVelocity[0] ** 2, 0), 9.5);
    // Independent false control: two whole-edge Gauss points straddle the
    // jump but give 7.5 rather than the exact kinetic integral 9.5.
    const incorrect = [(1 - 1 / Math.sqrt(3)) * 1.5, (1 + 1 / Math.sqrt(3)) * 1.5]
        .reduce((s, label) => s + .5 * 1.5 * h.sample('wire', label)[0] ** 2, 0);
    close(incorrect, 7.5); assert.ok(Math.abs(incorrect - 9.5) > 1);
    assert.equal(t.pieces[0].endTrace, 'left'); assert.equal(t.pieces[1].startTrace, 'right');
});

test('continuous endpoint values do not erase a derivative discontinuity or an explicit old boundary', () => {
    const h = createCompositeJointMaterialHistory({ materialVelocities: [entry(0, span('wire', 0, 1, [[0, 0, 0], [1, 0, 0]])),
        entry(1, span('wire', 1, 3, [[1, 0, 0], [5, 0, 0]]))] });
    assert.deepEqual(h.sample('wire', 1), [1, 0, 0]);
    const prepared = h.prepare({ coordinates: [0, 3], inertiaEdges: [{ tools: [map('wire', 0)] }] });
    assert.equal(prepared.requiresSubdivision, true); assert.equal(prepared.inertiaEdges[0].tools[0].pieces.length, 2);
});

test('overlap and exposure use independent label histories and an explicit reservoir for newly entering material', () => {
    const calls = [], external = span('wire', -4, 0, [[9, 0, 0], [9, 0, 0]]);
    const h = createCompositeJointMaterialHistory({ materialVelocities: [entry(0, span('catheter', 100, 102, [[-2, 0, 0], [-2, 0, 0]]),
        span('wire', 0, 2, [[1, 0, 0], [1, 0, 0]])), entry(1, span('wire', 2, 4, [[3, 0, 0], [3, 0, 0]]))],
        reservoir(query) { calls.push(query); return query.toolId === 'wire' && query.s <= 0 ? external : null; } });
    const prepared = h.prepare({ coordinates: [0, 1, 3], inertiaEdges: [{ tools: [map('wire', -1), map('catheter', 100)] }, { tools: [map('wire', 0)] }] });
    assert.deepEqual(prepared.inertiaEdges[0].tools[0].oldMaterialVelocities, [[9, 0, 0], [9, 0, 0]]);
    assert.deepEqual(prepared.inertiaEdges[0].tools[1].oldMaterialVelocities, [[-2, 0, 0], [-2, 0, 0]]);
    assert.deepEqual(prepared.inertiaEdges[1].tools[0].oldMaterialVelocities, [[1, 0, 0], [1, 0, 0]]);
    assert.equal(calls.every(c => c.toolId === 'wire'), true);
    assert.throws(() => h.sample('catheter', 1), { code: 'missing-material-history' });
    assert.deepEqual(h.sample('wire', 0, { trace: 'left' }), [9, 0, 0]); assert.deepEqual(h.sample('wire', 0, { trace: 'right' }), [1, 0, 0]);
    assert.throws(() => h.sample('wire', 0), { code: 'ambiguous-material-history-trace' });
});

test('reservoir pieces crossing known history are clipped by ownership and never replace accepted material values', () => {
    const h = createCompositeJointMaterialHistory({ materialVelocities: [entry(0, span('wire', 1, 2, [[2, 0, 0], [2, 0, 0]]))],
        reservoir: ({ toolId }) => span(toolId, -10, 10, [[9, 0, 0], [9, 0, 0]]) });
    const r = h.prepare({ coordinates: [0, 3], inertiaEdges: [{ tools: [map('wire', 0)] }] });
    assert.deepEqual(r.inertiaEdges[0].tools[0].pieces.map(p => [p.sStart, p.sEnd, p.source]), [[0, 1, 'reservoir'], [1, 2, 'accepted'], [2, 3, 'reservoir']]);
    assert.deepEqual(r.requiredCuts.map(p => p.s), [1, 2]);
});

test('owned snapshots and outputs isolate source mutations, failed preparation and identical retries', () => {
    const source = [entry(0, span('wire', 0, 1, [[1, 2, 3], [4, 5, 6]]))], before = structuredClone(source), external = span('wire', 1, 4, [[7, 8, 9], [10, 11, 12]]);
    const h = createCompositeJointMaterialHistory({ materialVelocities: source, reservoir: () => external });
    source[0].tools[0].velocities[0][0] = 999; source[0].tools[0].sEnd = 500;
    assert.deepEqual(h.sample('wire', 0, { trace: 'right' }), before[0].tools[0].velocities[0]);
    const input = { coordinates: [0, 2], inertiaEdges: [{ tools: [map('wire', 0)] }] }, inputBefore = structuredClone(input);
    assert.throws(() => h.prepare({ ...input, maxPieces: 1 }), { code: 'history-piece-budget' });
    const first = h.prepare(input), again = h.prepare(input); assert.deepEqual(first, again); assert.deepEqual(input, inputBefore);
    external.velocities[0][0] = -99; assert.equal(first.inertiaEdges[0].tools[0].pieces[1].oldMaterialVelocities[0][0], 7);
    first.inertiaEdges[0].tools[0].pieces[0].oldMaterialVelocities[0][0] = -11;
    const v = h.sample('wire', .5); v[0] = 0; assert.deepEqual(h.sample('wire', .5), [2.5, 3.5, 4.5]);
});

test('invalid or absent history, ambiguous intervals and noncovering reservoirs reject without defaults', () => {
    const empty = createCompositeJointMaterialHistory({ materialVelocities: null });
    assert.throws(() => empty.sample('wire', 0), { code: 'missing-material-history' });
    assert.throws(() => empty.prepare({ coordinates: [0, 1], inertiaEdges: [{ tools: [map('wire', 0)] }] }), { code: 'missing-material-history' });
    assert.throws(() => createCompositeJointMaterialHistory({ materialVelocities: [entry(0, span('wire', 0, 2, [[0, 0, 0], [0, 0, 0]])), entry(1, span('wire', 1, 3, [[0, 0, 0], [0, 0, 0]]))] }), /Overlapping/);
    assert.throws(() => createCompositeJointMaterialHistory({ materialVelocities: [entry(0, { ...span('wire', 0, 2, [[0, 0, 0], [0, 0, 0]]), interpretation: 'advected-grid-velocity' })] }), /physical material velocity/);
    for (const supplied of [span('catheter', -1, 1, [[0, 0, 0], [0, 0, 0]]), span('wire', 2, 3, [[0, 0, 0], [0, 0, 0]])]) {
        const h = createCompositeJointMaterialHistory({ materialVelocities: null, reservoir: () => supplied });
        assert.throws(() => h.sample('wire', 0, { trace: 'right' }), /Reservoir must cover/);
    }
    const gaps = createCompositeJointMaterialHistory({ materialVelocities: [entry(0, span('wire', 0, 1, [[0, 0, 0], [0, 0, 0]])), entry(1, span('wire', 2, 3, [[0, 0, 0], [0, 0, 0]]))] });
    assert.throws(() => gaps.prepare({ coordinates: [0, 3], inertiaEdges: [{ tools: [map('wire', 0)] }] }), { code: 'missing-material-history' });
    assert.throws(() => gaps.sample('wire', 0, { trace: 'left' }), { code: 'missing-material-history' });
    assert.deepEqual(gaps.sample('wire', 0), [0, 0, 0]);
    assert.throws(() => gaps.sample('wire', 0, { trace: 'upwind' }), /left or right/);
});

function realStepFixture() {
    const coordinates = [0, 1, 3], positions = coordinates.map(x => [x, 0, 0]), wire = coordinates.map(x => [1.5 * x, .3, 0]);
    const layout = createCompositeChainLayout([['wire', 'catheter'], ['wire', 'catheter']]);
    const modes = coordinates.map((_, node) => ({ node, basis: [[1, 0, 0], [0, 1, 0], [0, 0, 1]] }));
    const angles = new Map(), restLengths = new Map(), tools = ['wire', 'catheter'].map(id => {
        const dsDx = id === 'wire' ? 1.5 : 1; angles.set(id, new Float64Array(2)); restLengths.set(id, Float64Array.from([dsDx, 2 * dsDx]));
        return { id, dsDx, reference: captureCompositeReferenceFrames(id === 'wire' ? wire : positions), referenceTwists: new Float64Array(1),
            material: compileCompositeMaterial({ stiffness: [[2, 0, 0], [0, 3, 0], [0, 0, 1]], intrinsic: [0, 0, 0] }) };
    });
    const state = createCompositeJointTimeStepState({ layout, coordinates, positions, relative: wire.flatMap((p, i) => p.map((v, k) => v - positions[i][k])), modes, angles, tools, restLengths, materialCoordinate: 'reference-arclength' });
    const inertiaEdges = layout.edgeToolIds.map((ids, edge) => ({ tools: ids.map(id => ({ ...map(id, (id === 'wire' ? 20 : 100) + coordinates[edge] * (id === 'wire' ? 1.5 : 1), id === 'wire' ? 1.5 : 1), massPerMaterialLength: id === 'wire' ? .13 : .24 })) }));
    return { state, inertiaEdges, velocities: new Map([['wire', [.2, -.1, .04]], ['catheter', [-.08, .03, .02]]]) };
}

test('actual accepted first and second dt histories prepare own affine velocities for the next unchanged material map', () => {
    const f = realStepFixture(), empty = createCompositeJointMaterialHistory({ materialVelocities: f.state.materialVelocities,
        reservoir: ({ toolId }) => span(toolId, -1000, 1000, [f.velocities.get(toolId), f.velocities.get(toolId)]) });
    let state = f.state, history = empty;
    for (let step = 0; step < 2; step++) {
        const source = structuredClone(state), prepared = history.prepare({ coordinates: state.coordinates, inertiaEdges: f.inertiaEdges });
        assert.equal(prepared.requiresSubdivision, false);
        const inertiaEdges = f.inertiaEdges.map((entry, e) => ({ tools: entry.tools.map((t, i) => ({ ...t, oldMaterialVelocities: prepared.inertiaEdges[e].tools[i].oldMaterialVelocities })) }));
        const result = advanceCompositeJointTimeStep(state, { dt: .1, torsionMode: 'quasi-static', contacts: 'none',
            boundaries: { positions: [], spins: [{ toolId: 'wire', edge: 0, value: 0 }, { toolId: 'catheter', edge: 0, value: 0 }] },
            inertia: { previousPositions: structuredClone(state.toolPositions), inertiaEdges } });
        assert.equal(result.accepted, true, JSON.stringify({ status: result.status, error: result.error, certificate: result.diagnostics.certificate }));
        assert.deepEqual(state, source);
        for (const e of result.state.materialVelocities) for (const t of e.tools) for (const v of t.velocities) vectorClose(v, f.velocities.get(t.id), 1e-10);
        history = createCompositeJointMaterialHistory({ materialVelocities: result.state.materialVelocities });
        const again = history.prepare({ coordinates: result.state.coordinates, inertiaEdges: f.inertiaEdges });
        again.inertiaEdges.forEach((e, i) => e.tools.forEach((t, j) => t.oldMaterialVelocities.forEach((v, end) =>
            vectorClose(v, result.state.materialVelocities[i].tools[j].velocities[end], 0))));
        state = result.state;
    }
    assert.equal(state.step, 2); close(state.time, .2, 0);
});

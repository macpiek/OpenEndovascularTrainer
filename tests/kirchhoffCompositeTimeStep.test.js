import assert from 'node:assert/strict';
import test from 'node:test';
import { createCompositeTimeStepState, advanceCompositeTimeStep as advance } from '../src/physics/kirchhoffCompositeTimeStep.js';
import { createCompositeChainLayout, createCompositeChainWorkspace, assembleCompositeChain } from '../src/physics/kirchhoffCompositeChain.js';
import { captureCompositeReferenceFrames, compileCompositeMaterial } from '../src/physics/kirchhoffCompositeElement.js';
import { assembleCompositeTranslationalInertia } from '../src/physics/kirchhoffCompositeKinematics.js';

const DT = 1 / 120;
const close = (a, b, tolerance = 1e-8) => assert.ok(Math.abs(a - b) <= tolerance * Math.max(1, Math.abs(a), Math.abs(b)), `${a} != ${b}`);
function fixture({ count = 5, spacing = 2, mass = .01 } = {}) {
    const positions = Array.from({ length: count }, (_, i) => [i * spacing, 0, 0]);
    const layout = createCompositeChainLayout(Array.from({ length: count - 1 }, () => ['wire', 'catheter']));
    const data = { positions, coordinates: positions.map(p => p[0]), reference: captureCompositeReferenceFrames(positions), tools: [
        { id: 'wire', angles: new Float64Array(count - 1), dsDx: 1, material: compileCompositeMaterial({ EI1: 1, GJ: 2 }) },
        { id: 'catheter', angles: new Float64Array(count - 1), dsDx: 1, material: compileCompositeMaterial({ EI1: 3, GJ: 5 }) }
    ] };
    return { state: createCompositeTimeStepState({ data, layout }), mass };
}
function spinRoots(state) {
    return [...state.layout.spins.values()].map(offsets => ({ dof: offsets.find(dof => dof >= 0), value: 0 }));
}
function clamped(state, nodes = 2) {
    const result = spinRoots(state);
    for (let node = 0; node < nodes; node++) for (let axis = 0; axis < 3; axis++)
        result.push({ dof: state.layout.positions[node] + axis, value: state.data.positions[node][axis] });
    return result;
}
function inertia(state, mass, { commonVelocity = [0, 0, 0], feed = { wire: 0, catheter: 0 }, useAcceptedVelocities = false } = {}) {
    return state.layout.edgeToolIds.map((ids, edge) => ({ tools: ids.map(id => {
        const length = state.data.coordinates[edge + 1] - state.data.coordinates[edge];
        const tangent = state.data.positions[edge + 1].map((v, axis) => (v - state.data.positions[edge][axis]) / length);
        const u = feed[id], velocity = commonVelocity.map((v, axis) => v + u * tangent[axis]);
        const old = useAcceptedVelocities ? state.materialVelocities[edge].tools.find(tool => tool.id === id).velocities : [velocity, velocity];
        return { id, materialMap: { sStart: (id === 'wire' ? 100 : 200) + state.data.coordinates[edge], dsDx: 1, dsDt: -u },
            massPerMaterialLength: mass * (id === 'wire' ? 1 : 2), oldMaterialVelocities: old.map(v => [...v]) };
    }) }));
}
function options(f, extra = {}) {
    return { dt: DT, torsionMode: 'quasi-static', inertiaEdges: inertia(f.state, f.mass), prescribed: clamped(f.state),
        tolerances: { force: 1e-7, torque: 1e-8, length: 1e-8 }, initialPenalty: 1e4,
        budget: { directions: 120, outerIterations: 20, lineSearchTrials: 24, evaluations: 1000 }, ...extra };
}
function accepted(result) { assert.equal(result.accepted, true, JSON.stringify(result.diagnostics)); }
function packed(state) {
    const values = new Float64Array(state.layout.dofCount);
    state.data.positions.forEach((p, node) => p.forEach((value, axis) => { values[state.layout.positions[node] + axis] = value; }));
    for (const tool of state.data.tools) state.layout.spins.get(tool.id).forEach((dof, edge) => { if (dof >= 0) values[dof] = tool.angles[edge]; });
    return values;
}
function bounds(result) {
    const d = result.diagnostics;
    assert.ok(d.directions <= d.limits.directions); assert.ok(d.evaluations <= d.limits.evaluations);
    assert.ok(d.outerIterations <= d.limits.outerIterations);
    assert.ok(d.lineSearchTrials <= d.directions * d.limits.lineSearchTrials);
}

test('rest accepts one real dt from independently measured forces and lengths with owned output', () => {
    const f = fixture(), saved = structuredClone(f.state), result = advance(f.state, options(f));
    accepted(result); bounds(result);
    assert.deepEqual(f.state, saved); assert.notEqual(result.state, f.state);
    assert.equal(result.state.step, 1); assert.equal(result.state.time, DT);
    assert.equal(result.diagnostics.historyCommits, 1); assert.equal(result.diagnostics.directions, 0);
    assert.equal(result.diagnostics.certificate.force, 0); assert.equal(result.diagnostics.certificate.length, 0);
    assert.deepEqual(result.state.data.positions, saved.data.positions);
    for (const edge of result.state.materialVelocities) for (const tool of edge.tools) {
        assert.ok(tool.velocities.flat().every(v => v === 0));
        assert.equal(tool.angularVelocity, null); assert.equal(tool.materialSpin, null); assert.equal(tool.frameSpin, null);
    }
});

test('free rigid translation with independent opposite advection keeps each material velocity', () => {
    const f = fixture(), commonVelocity = [.4, -.2, .1], feed = { wire: 1.7, catheter: -.8 };
    const opts = options(f, { prescribed: spinRoots(f.state), inertiaEdges: inertia(f.state, f.mass, { commonVelocity, feed }) });
    const result = advance(f.state, opts); accepted(result); bounds(result);
    result.state.data.positions.forEach((p, node) => p.forEach((v, axis) => close(v, f.state.data.positions[node][axis] + DT * commonVelocity[axis], 1e-9)));
    for (const edge of result.state.materialVelocities) for (const tool of edge.tools) for (const v of tool.velocities)
        v.forEach((value, axis) => close(value, commonVelocity[axis] + (axis === 0 ? feed[tool.id] : 0), 2e-8));
    assert.ok(result.diagnostics.inertialEnergy < 1e-15);
    assert.ok(result.diagnostics.kineticEnergy > 0);
});

test('loaded clamped bending converges a nonlinear dynamic step and balances physical support work', () => {
    const f = fixture({ mass: 1e-6 }), loads = new Float64Array(f.state.layout.dofCount);
    loads[f.state.layout.positions.at(-1) + 1] = .005;
    const opts = options(f, { loads }), result = advance(f.state, opts); accepted(result); bounds(result);
    assert.ok(result.state.data.positions.at(-1)[1] > .001);
    assert.ok(result.diagnostics.certificate.force <= opts.tolerances.force);
    assert.ok(result.diagnostics.certificate.length <= opts.tolerances.length);
    assert.ok(result.diagnostics.directions > 1);
    const support = [0, 0, 0];
    for (const start of f.state.layout.positions) for (let axis = 0; axis < 3; axis++) support[axis] += result.diagnostics.reactions[start + axis] + loads[start + axis];
    const momentumRate = [0, 0, 0];
    result.state.materialVelocities.forEach((edge, i) => edge.tools.forEach(tool => {
        const prepared = opts.inertiaEdges[i].tools.find(t => t.id === tool.id), length = f.state.data.coordinates[i + 1] - f.state.data.coordinates[i];
        for (let axis = 0; axis < 3; axis++) momentumRate[axis] += prepared.massPerMaterialLength * length *
            (tool.velocities[0][axis] + tool.velocities[1][axis] - prepared.oldMaterialVelocities[0][axis] - prepared.oldMaterialVelocities[1][axis]) / (2 * DT);
    }));
    support.forEach((v, i) => close(v, momentumRate[i], 5e-7));
});

test('accepted original force residual matches independent differences of elastic/inertial work plus physical lambda*g', () => {
    const f = fixture({ mass: 1e-6 }), loads = new Float64Array(f.state.layout.dofCount);
    loads[f.state.layout.positions.at(-1) + 1] = .005;
    const opts = options(f, { loads }), result = advance(f.state, opts); accepted(result);
    const data = structuredClone(result.state.data), layout = result.state.layout, origin = packed(f.state);
    // The objective for THIS dt differentiates the previous accepted reference
    // frames. New accepted frames belong to the next dt only.
    data.reference = structuredClone(f.state.data.reference);
    data.tools.forEach((tool, i) => { tool.referenceTwists = f.state.data.tools[i].referenceTwists; });
    const workspace = createCompositeChainWorkspace(layout), fixed = new Set(opts.prescribed.map(b => b.dof));
    const scalar = () => {
        let energy = assembleCompositeChain(data, workspace).energy;
        for (let edge = 0; edge < layout.nodeCount - 1; edge++) {
            const a = data.positions[edge], b = data.positions[edge + 1], length = data.coordinates[edge + 1] - data.coordinates[edge];
            energy += assembleCompositeTranslationalInertia({ dt: DT, coordinates: [data.coordinates[edge], data.coordinates[edge + 1]],
                positions: [a, b], previousPositions: [f.state.data.positions[edge], f.state.data.positions[edge + 1]],
                tools: opts.inertiaEdges[edge].tools }).energy;
            // Independent original bilateral equation, with accepted signed
            // physical lambda. No penalty or AL trial multiplier is used.
            energy += result.state.lengthMultipliers[edge] * (Math.hypot(...b.map((v, axis) => v - a[axis])) - length);
        }
        const values = packed({ data, layout });
        return energy - loads.reduce((sum, value, dof) => sum + value * (values[dof] - origin[dof]), 0);
    };
    const entries = [];
    data.positions.forEach((position, node) => position.forEach((_, axis) => entries.push({ dof: layout.positions[node] + axis,
        change: delta => { position[axis] += delta; }, tolerance: 2e-7 })));
    for (const tool of data.tools) layout.spins.get(tool.id).forEach((dof, edge) => {
        if (dof >= 0) entries.push({ dof, change: delta => { tool.angles[edge] += delta; }, tolerance: 2e-8 });
    });
    for (const { dof, change, tolerance } of entries) {
        const h = 2e-6;
        change(h); const plus = scalar(); change(-2 * h); const minus = scalar(); change(h);
        const derivative = (plus - minus) / (2 * h);
        close(derivative, result.diagnostics.originalResidual[dof], 2e-8);
        if (!fixed.has(dof)) assert.ok(Math.abs(derivative) <= tolerance);
    }
});

test('quasi-static opposite torques retain separate GJ and spin solutions on the same fixed axis', () => {
    const f = fixture(), loads = new Float64Array(f.state.layout.dofCount), prescribed = clamped(f.state, f.state.layout.nodeCount);
    const torque = { wire: .02, catheter: -.035 };
    for (const [id, offsets] of f.state.layout.spins) loads[offsets.at(-1)] = torque[id];
    const result = advance(f.state, options(f, { loads, prescribed })); accepted(result); bounds(result);
    for (const tool of result.state.data.tools) {
        const gj = tool.id === 'wire' ? 2 : 5;
        tool.angles.forEach((angle, edge) => close(angle, torque[tool.id] * edge * 2 / gj));
        close(result.diagnostics.reactions[result.state.layout.spins.get(tool.id)[0]], -torque[tool.id]);
    }
    assert.deepEqual(result.state.data.positions, f.state.data.positions);
    assert.equal(result.state.torsionMode, 'quasi-static');
});

test('stretched initial trial geometry is solved without projecting or changing the accepted input', () => {
    const f = fixture(), guess = packed(f.state), saved = structuredClone(f.state);
    guess[f.state.layout.positions.at(-1)] += .2;
    const result = advance(f.state, options(f, { initialGuess: guess })); accepted(result); bounds(result);
    assert.ok(result.diagnostics.initialCertificate.length > .19);
    assert.ok(result.diagnostics.certificate.length <= 1e-8);
    assert.ok(result.diagnostics.directions > 0);
    result.state.data.positions.forEach((p, node) => p.forEach((v, axis) => close(v, f.state.data.positions[node][axis], 1e-9)));
    assert.deepEqual(f.state, saved);
});

test('exhausted outer budget rolls back nonzero candidate multipliers, geometry and velocity history', () => {
    const f = fixture({ count: 3 }), snapshot = structuredClone(f.state), loads = new Float64Array(f.state.layout.dofCount);
    loads[f.state.layout.positions.at(-1)] = .1;
    const result = advance(f.state, options(f, { loads, initialPenalty: 10,
        budget: { directions: 10, outerIterations: 1, evaluations: 100, lineSearchTrials: 10 } }));
    assert.equal(result.accepted, false); assert.equal(result.state, f.state);
    assert.equal(result.status, 'outer-budget-exhausted'); assert.ok(result.diagnostics.dualUpdates >= 1);
    assert.equal(result.diagnostics.historyCommits, 0); assert.deepEqual(f.state, snapshot); bounds(result);
    assert.ok(result.diagnostics.certificate.length > 1e-8);
});

test('zero free augmented gradient cannot accept incompatible prescribed lengths', () => {
    const f = fixture({ count: 3 }), prescribed = clamped(f.state, 3);
    prescribed.find(b => b.dof === f.state.layout.positions.at(-1)).value += .1;
    const result = advance(f.state, options(f, { prescribed, budget: { directions: 0, outerIterations: 2, evaluations: 20 } }));
    assert.equal(result.accepted, false); bounds(result);
    assert.equal(result.diagnostics.certificate.force, 0); assert.equal(result.diagnostics.certificate.torque, 0);
    assert.ok(result.diagnostics.certificate.length > .09); assert.equal(result.diagnostics.historyCommits, 0);
});

test('force and every global work budget are enforced independently of a small or absent increment', () => {
    for (const budget of [
        { directions: 0 }, { evaluations: 1 }, { lineSearchTrials: 0 }, { outerIterations: 0 }
    ]) {
        const f = fixture(), saved = structuredClone(f.state), loads = new Float64Array(f.state.layout.dofCount);
        loads[f.state.layout.positions.at(-1) + 1] = .01;
        const result = advance(f.state, options(f, { loads, budget }));
        assert.equal(result.accepted, false); assert.equal(result.state, f.state); assert.deepEqual(f.state, saved);
        assert.equal(result.diagnostics.historyCommits, 0); bounds(result);
        if (budget.directions === 0) assert.ok(result.diagnostics.certificate.force > .009);
    }
});

test('two consecutive 120 Hz steps reuse accepted material velocity and commit exactly twice', () => {
    const f = fixture(), commonVelocity = [.3, .1, -.2];
    const first = advance(f.state, options(f, { prescribed: spinRoots(f.state), inertiaEdges: inertia(f.state, f.mass, { commonVelocity }) }));
    accepted(first);
    const savedFirst = structuredClone(first.state), next = { state: first.state, mass: f.mass };
    const second = advance(first.state, options(next, { prescribed: spinRoots(first.state), inertiaEdges: inertia(first.state, f.mass, { useAcceptedVelocities: true }) }));
    accepted(second); bounds(second);
    assert.equal(second.state.step, 2); assert.equal(second.state.time, 2 * DT); assert.deepEqual(first.state, savedFirst);
    second.state.data.positions.forEach((p, node) => p.forEach((v, axis) => close(v, f.state.data.positions[node][axis] + 2 * DT * commonVelocity[axis], 1e-9)));
    const loads = new Float64Array(second.state.layout.dofCount); loads[second.state.layout.positions.at(-1) + 1] = .01;
    const failed = advance(second.state, options({ state: second.state, mass: f.mass }, { loads, budget: { directions: 0 },
        inertiaEdges: inertia(second.state, f.mass, { useAcceptedVelocities: true }), prescribed: spinRoots(second.state) }));
    assert.equal(failed.accepted, false); assert.equal(failed.state, second.state);
    assert.equal(failed.state.step, 2); assert.equal(failed.state.time, 2 * DT);
    assert.equal(failed.state.materialVelocities, second.state.materialVelocities);
});

test('material providers are sampled once per hinge per dt and never during nonlinear trial replay', () => {
    const f = fixture({ mass: 1e-6 }), calls = new Map(), scaleCalls = new Map(), loads = new Float64Array(f.state.layout.dofCount);
    for (const tool of f.state.data.tools) {
        const material = tool.material;
        tool.materialAt = ({ vertex }) => { const key = `${tool.id}:${vertex}`; calls.set(key, (calls.get(key) ?? 0) + 1); return material; };
        tool.dsDx = coordinate => { const key = `${tool.id}:${coordinate}`; scaleCalls.set(key, (scaleCalls.get(key) ?? 0) + 1); return 1; };
    }
    loads[f.state.layout.positions.at(-1) + 1] = .005;
    const result = advance(f.state, options(f, { loads })); accepted(result);
    assert.ok(result.diagnostics.directions > 1);
    assert.equal(calls.size, 2 * (f.state.layout.nodeCount - 2)); assert.ok([...calls.values()].every(n => n === 1));
    assert.equal(scaleCalls.size, calls.size); assert.ok([...scaleCalls.values()].every(n => n === 1));
    for (let i = 0; i < f.state.data.tools.length; i++) assert.equal(result.state.data.tools[i].materialAt, f.state.data.tools[i].materialAt);
});

test('two loaded 120 Hz steps transport accepted frames only on success and retain original force gates', () => {
    const f = fixture({ mass: 1e-6 }), firstLoads = new Float64Array(f.state.layout.dofCount);
    firstLoads[f.state.layout.positions.at(-1) + 1] = .005;
    const first = advance(f.state, options(f, { loads: firstLoads })); accepted(first);
    const saved = structuredClone(first.state), secondLoads = new Float64Array(first.state.layout.dofCount);
    secondLoads[first.state.layout.positions.at(-1) + 1] = .003;
    secondLoads[first.state.layout.positions.at(-1) + 2] = .006;
    const second = advance(first.state, options({ state: first.state, mass: f.mass }, { loads: secondLoads,
        inertiaEdges: inertia(first.state, f.mass, { useAcceptedVelocities: true }) }));
    accepted(second); bounds(second); assert.deepEqual(first.state, saved);
    assert.equal(second.state.step, 2); assert.equal(second.state.time, 2 * DT);
    second.state.data.reference.forEach((frame, edge) => {
        const d = second.state.data.positions[edge + 1].map((v, axis) => v - second.state.data.positions[edge][axis]);
        const length = Math.hypot(...d);
        frame.tangent.forEach((v, axis) => close(v, d[axis] / length, 1e-12));
        close(Math.hypot(...frame.director), 1, 1e-12);
        close(frame.director.reduce((sum, v, axis) => sum + v * frame.tangent[axis], 0), 0, 1e-12);
    });
    assert.ok(second.diagnostics.certificate.force <= 1e-7);
    assert.ok(second.diagnostics.certificate.torque <= 1e-8);
    assert.ok(second.diagnostics.certificate.length <= 1e-8);
});

test('the torsion approximation and prepared physical inputs cannot be silently defaulted', () => {
    const f = fixture(), opts = options(f);
    delete opts.torsionMode; assert.throws(() => advance(f.state, opts), /explicit torsionMode/);
    assert.throws(() => advance(f.state, options(f, { prescribed: [] })), /spin boundary/);
    const missing = options(f); delete missing.inertiaEdges[0].tools[0].oldMaterialVelocities;
    const result = advance(f.state, missing);
    assert.equal(result.accepted, false); assert.equal(result.status, 'numerical-rejection');
    assert.match(result.diagnostics.error, /OLD MATERIAL/); assert.equal(result.state, f.state);
});

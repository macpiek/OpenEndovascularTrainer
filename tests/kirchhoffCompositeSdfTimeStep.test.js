import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import { VesselContactField, createContactResult } from '../src/physics/collision/vesselContactField.js';
import { decodeCollisionAsset } from '../src/physics/collision/collisionAssetFormat.js';
import { createCompositeTimeStepState, createCompositeTimeStepWorkspace, advanceCompositeTimeStep } from '../src/physics/kirchhoffCompositeTimeStep.js';
import { createCompositeChainLayout, createCompositeChainWorkspace, assembleCompositeChain } from '../src/physics/kirchhoffCompositeChain.js';
import { createCompositeInertiaCache } from '../src/physics/kirchhoffCompositeInertiaCache.js';
import { createCompositeLengthConstraintWorkspace, measureCompositeLengthConstraints } from '../src/physics/kirchhoffCompositeLengthConstraints.js';
import { createCompositeWallSdfBranchesWorkspace, evaluateCompositeWallSdfBranches, measureCompositeWallSdfBranches } from '../src/physics/kirchhoffCompositeWallSdfBranches.js';
import { captureCompositeReferenceFrames, compileCompositeMaterial } from '../src/physics/kirchhoffCompositeElement.js';
import { queryCompositeWallCapsuleGeometry } from '../src/physics/kirchhoffCompositeWallGeometry.js';

const close = (a, b, tolerance = 1e-7) => assert.ok(Math.abs(a - b) <= tolerance * Math.max(1, Math.abs(a), Math.abs(b)), `${a} != ${b}`);
const bytes = fs.readFileSync(new URL('../res/Aorta_plain.collision.bin', import.meta.url));
const anatomy = decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength));
function fixture(contactMode = 'capsule') {
    // Original real-anatomy P1 witness, extended by one straight free edge.
    // No provider options, sample policy or anatomy data are modified.
    const a = [65.00287246704102, -462.00980948623305, -79.56869888305664];
    const b = [64.95548751831055, -461.7916869276393, -79.65612350463867];
    const positions = [a, b, b.map((v, i) => 2 * v - a[i])];
    const h = Math.hypot(...b.map((v, i) => v - a[i]));
    const layout = createCompositeChainLayout([['wire'], ['wire']]);
    const data = { positions, coordinates: [0, h, 2 * h], reference: captureCompositeReferenceFrames(positions),
        tools: [{ id: 'wire', angles: new Float64Array(2), material: compileCompositeMaterial({ EI1: .001, GJ: .001 }) }] };
    const state = createCompositeTimeStepState({ data, layout }), field = new VesselContactField(anatomy);
    const options = { dt: 1 / 120, torsionMode: 'quasi-static', elementBackend: 'wasm-exact', constraintSolver: 'mixed',
        prescribed: [{ dof: layout.spins.get('wire')[0], value: 0 }],
        inertiaEdges: layout.edgeToolIds.map((_, e) => ({ tools: [{ id: 'wire', massPerMaterialLength: .01,
            materialMap: { sStart: e * h, dsDx: 1, dsDt: 0 }, oldMaterialVelocities: [[0, 0, 0], [0, 0, 0]] }] })),
        tolerances: { force: 1e-7, torque: 1e-8, length: 1e-8, linear: 1e-10 }, initialPenalty: 1e4,
        budget: { directions: 64, outerIterations: 64, evaluations: 1500, lineSearchTrials: 30 },
        wall: { field, friction: 'frictionless', contactMode, initialPenalty: 1e4,
            tolerances: { gap: 1e-8, force: 1e-7, work: 1e-7 },
            contactOwners: { edges: [{ edge: 0, wall: { owner: 'wire', radius: .4445 } }, { edge: 1, wall: null }] } } };
    return { state, field, options, h };
}

for (const mode of ['capsule', 'envelope']) test(`real sparse-SDF ${mode} timestep resolves P1 on its smooth branch with physical momentum balance`, () => {
    const f = fixture(mode), before = structuredClone(f.state);
    // Hold transverse handle coordinates to isolate the supported smooth
    // polynomial branch. The fully free version separately exercises the
    // two-reaction min-seam block below.
    for (let node = 0; node < f.state.layout.nodeCount; node++) for (const axis of [0, 2])
        f.options.prescribed.push({ dof: f.state.layout.positions[node] + axis, value: f.state.data.positions[node][axis] });
    const geometry = queryCompositeWallCapsuleGeometry({ field: f.field, positions: f.state.data.positions.slice(0, 2), radius: .4445 });
    assert.ok(geometry.supported, geometry.reason); assert.equal(geometry.source, 'sparse-sdf');
    close(geometry.gap, -.02, 1e-12); assert.ok(Math.abs(geometry.gradientNorm - 1) > .5);
    const r = advanceCompositeTimeStep(f.state, f.options);
    assert.ok(r.accepted, JSON.stringify({ status: r.status, ...r.diagnostics }));
    assert.deepEqual(f.state, before); assert.equal(r.state.step, 1); assert.equal(r.state.time, f.options.dt);
    assert.equal(r.diagnostics.historyCommits, 1); assert.ok(r.diagnostics.certificate.wall.converged);
    const contact = f.field.queryCapsuleCoordinates(...r.state.data.positions[0], ...r.state.data.positions[1], .4445, createContactResult());
    assert.ok(contact.signedGap >= -f.options.wall.tolerances.gap);
    const force = [0, 0, 0], momentumRate = [0, 0, 0];
    for (const row of r.state.wallContactState.records) {
        close(Math.hypot(...row.worldForce), row.history.normalForce, 1e-12);
        row.worldForce.forEach((v, axis) => { force[axis] += v; });
    }
    for (const edge of r.state.materialVelocities) for (const tool of edge.tools)
        for (let axis = 0; axis < 3; axis++) momentumRate[axis] += .01 * f.h / f.options.dt * .5 *
            (tool.velocities[0][axis] + tool.velocities[1][axis]);
    for (let axis = 0; axis < 3; axis++) {
        const boundaryReaction = Array.from(r.state.layout.positions).reduce((sum, first) => sum + r.diagnostics.reactions[first + axis], 0);
        close(force[axis] + boundaryReaction, momentumRate[axis], 3e-7);
    }
    assert.ok(Math.hypot(...force) > .1, 'the physical reaction must not be scaled away with the raw SDF gradient');
});

test('the original free P1 timestep resolves both physical seam reactions and passes fresh original equations', () => {
    const f = fixture(), before = structuredClone(f.state);
    let queries = 0;
    const query = f.field.queryCapsuleCoordinates.bind(f.field);
    f.field.queryCapsuleCoordinates = (...args) => { queries++; return query(...args); };
    const r = advanceCompositeTimeStep(f.state, f.options);
    assert.ok(r.accepted, JSON.stringify({ status: r.status, ...r.diagnostics }));
    assert.deepEqual(f.state, before); assert.equal(r.state.step, 1); assert.equal(r.state.time, f.options.dt);
    assert.equal(r.diagnostics.historyCommits, 1); assert.equal(r.diagnostics.wallQueries, queries);
    assert.equal(queries, r.diagnostics.evaluations, 'chart discovery/rebuild must not repeat provider queries');
    assert.equal(r.diagnostics.sdfChartActivations, 1); assert.equal(r.diagnostics.sdfChartRebuilds, 1);
    const saved = r.state.wallContactState.sdfSeams;
    assert.equal(saved.length, 1); assert.deepEqual(saved[0].face, { axis: 0, gridIndex: 434 });
    assert.ok(saved[0].forces.every(v => v > 0)); assert.equal(r.state.wallContactState.multipliers[0], 0);
    const raw = query(...r.state.data.positions[0], ...r.state.data.positions[1], .4445, createContactResult());
    const chart = evaluateCompositeWallSdfBranches({ field: f.field, face: saved[0].face,
        positions: r.state.data.positions.slice(0, 2), radius: .4445, contact: raw }, createCompositeWallSdfBranchesWorkspace());
    assert.ok(chart.supported, chart.reason); assert.equal(chart.onSeam, true);
    const cone = measureCompositeWallSdfBranches(chart, { forces: saved[0].forces, penalty: 1e4,
        gapTolerance: 1e-8, forceTolerance: 1e-7, workTolerance: 1e-7 });
    assert.ok(cone.converged); assert.equal(cone.domainAdmissible, true); assert.equal(cone.tieAdmissible, true);
    close(saved[0].forces[0], 2.192477870513528, 1e-6); close(saved[0].forces[1], .598007423867116, 1e-6);
    // Independent fresh elastic + consistent inertia + original length forces
    // and the cone resultant; never trust the timestep's reported residual.
    const layout = f.state.layout, chain = createCompositeChainWorkspace(layout, { elementBackend: 'wasm-exact' });
    const current = { ...r.state.data, reference: f.state.data.reference,
        tools: r.state.data.tools.map((tool, i) => ({ ...tool, referenceTwists: f.state.data.tools[i].referenceTwists })) };
    assembleCompositeChain(current, chain);
    const inertia = createCompositeInertiaCache({ layout, coordinates: current.coordinates,
        previousPositions: f.state.data.positions, dt: f.options.dt, inertiaEdges: f.options.inertiaEdges });
    inertia.append(current.positions, chain);
    const lengths = measureCompositeLengthConstraints({ positions: current.positions, coordinates: current.coordinates,
        multipliers: r.state.lengthMultipliers, tolerance: f.options.tolerances.length }, createCompositeLengthConstraintWorkspace(layout));
    const residual = Float64Array.from(chain.gradient, (v, i) => v - lengths.constraintForces[i]);
    for (let i = 0; i < 6; i++) residual[[...layout.positions.slice(0, 2)].flatMap(s => [s, s + 1, s + 2])[i]] += cone.physicalGradient[i];
    for (const dof of layout.positions) assert.ok(Math.hypot(...residual.slice(dof, dof + 3)) <= 1e-7);
    assert.ok(Math.abs(residual[layout.spins.get('wire')[1]]) <= 1e-8); assert.ok(lengths.maximumLengthResidual <= 1e-8);
    const records = r.state.wallContactState.records;
    assert.equal(records.length, 2);
    for (const record of records) close(Math.hypot(...record.worldForce), record.history.normalForce, 1e-12);
    for (let axis = 0; axis < 3; axis++) close(records.reduce((sum, record) => sum + record.worldForce[axis], 0), cone.resultant[axis], 1e-12);
});

test('unsupported physical-normal AL on the P1 sparse-SDF contact rejects without changing accepted state or time', () => {
    const f = fixture(), before = structuredClone(f.state);
    const result = advanceCompositeTimeStep(f.state, { ...f.options, constraintSolver: 'augmented' });
    assert.equal(result.accepted, false); assert.equal(result.state, f.state); assert.deepEqual(f.state, before);
    assert.equal(result.diagnostics.historyCommits, 0);
    assert.match(result.diagnostics.error, /physical|potential|sparse-sdf/i);
});

test('interrupted seam discovery rolls back both private forces and allows a clean retry with reusable scratch', () => {
    const f = fixture(), before = structuredClone(f.state);
    const workspace = createCompositeTimeStepWorkspace(f.state.layout, { elementBackend: 'wasm-exact' });
    const interrupted = advanceCompositeTimeStep(f.state, { ...f.options, workspace, budget: { ...f.options.budget, directions: 1 } });
    assert.equal(interrupted.accepted, false); assert.equal(interrupted.status, 'direction-budget-exhausted');
    assert.equal(interrupted.diagnostics.sdfChartActivations, 1); assert.equal(interrupted.state, f.state);
    assert.deepEqual(f.state, before); assert.equal(interrupted.diagnostics.historyCommits, 0);
    const retry = advanceCompositeTimeStep(f.state, { ...f.options, workspace });
    const fresh = fixture(), clean = advanceCompositeTimeStep(fresh.state, fresh.options);
    assert.ok(retry.accepted, JSON.stringify(retry.diagnostics)); assert.ok(clean.accepted);
    assert.deepEqual(retry.state, clean.state); assert.deepEqual(f.state, before);
    const retained = structuredClone(retry.state);
    advanceCompositeTimeStep(f.state, { ...f.options, workspace, budget: { ...f.options.budget, directions: 0 } });
    assert.deepEqual(retry.state, retained, 'accepted cone forces and records must own their buffers');
});

test('accepted cone history survives the next physical dt and lazy/full original certificates agree', () => {
    const f = fixture(), first = advanceCompositeTimeStep(f.state, f.options);
    assert.ok(first.accepted);
    const previous = structuredClone(first.state);
    const inertiaEdges = f.options.inertiaEdges.map((edge, i) => ({ tools: edge.tools.map((tool, j) => ({ ...tool,
        oldMaterialVelocities: first.state.materialVelocities[i].tools[j].velocities })) }));
    const signedTrial = advanceCompositeTimeStep(first.state, { ...f.options, inertiaEdges,
        budget: { ...f.options.budget, directions: 1 } });
    assert.equal(signedTrial.accepted, false); assert.equal(signedTrial.state, first.state);
    const negativeProof = signedTrial.diagnostics.certificate.wall.sdfSeams[0];
    assert.ok(negativeProof.forces.some(v => v < 0)); assert.ok(negativeProof.maximumNegativeForce > 0);
    assert.equal(negativeProof.converged, false); assert.equal(signedTrial.diagnostics.historyCommits, 0);
    assert.deepEqual(first.state, previous, 'negative private Fn fails the original gate and rolls back the entire dt');
    const full = advanceCompositeTimeStep(first.state, { ...f.options, inertiaEdges, assemblyPolicy: 'full' });
    const lazy = advanceCompositeTimeStep(first.state, { ...f.options, inertiaEdges, assemblyPolicy: 'lazy' });
    assert.ok(full.accepted, JSON.stringify({ status: full.status, ...full.diagnostics }));
    assert.ok(lazy.accepted, JSON.stringify({ status: lazy.status, ...lazy.diagnostics }));
    assert.deepEqual(first.state, previous); assert.equal(full.state.step, 2); assert.equal(full.state.time, 2 * f.options.dt);
    assert.deepEqual(lazy.state, full.state); assert.deepEqual(lazy.diagnostics.certificate, full.diagnostics.certificate);
    assert.equal(full.diagnostics.historyCommits, 1); assert.ok(full.diagnostics.certificate.wall.converged);
    assert.ok(full.state.wallContactState.sdfSeams.every(seam => seam.forces.every(v => v >= 0)));
    assert.ok(full.state.wallContactState.records.every(record => record.history.normalForce >= 0));
    assert.equal(full.state.wallContactState.sdfSeams[0].forces[0], 0, 'the wrong-domain left reaction is released exactly');
    assert.ok(full.state.data.positions[1][0] > 65); assert.ok(full.state.wallContactState.sdfSeams[0].forces[1] > 0);
});

test('a changed original source in the fresh commit query rejects the complete candidate and both cone reactions', () => {
    const f = fixture(), before = structuredClone(f.state), query = f.field.queryCapsuleCoordinates.bind(f.field);
    let previous = null, rejectedCommit = false;
    f.field.queryCapsuleCoordinates = (...args) => {
        const raw = query(...args), point = args.slice(0, 7);
        if (previous && point.every((v, i) => v === previous[i]) && args[3] === 65 && Math.abs(raw.signedGap) <= 1e-8) {
            raw.source = 'unsupported-fresh-commit-source'; rejectedCommit = true;
        }
        previous = point; return raw;
    };
    const result = advanceCompositeTimeStep(f.state, f.options);
    assert.ok(rejectedCommit, 'the original provider must be called again for final certification');
    assert.equal(result.accepted, false); assert.equal(result.state, f.state); assert.deepEqual(f.state, before);
    assert.equal(result.diagnostics.historyCommits, 0); assert.match(result.diagnostics.error, /Unsupported SDF cone chart.*unsupported-source/);
});

test('an open exactly unloaded wall is eliminated algebraically while queries and original gaps remain checked', () => {
    const f = fixture(); let queries = 0;
    const field = { queryCapsuleCoordinates(ax, ay, az, bx, by, bz, radius, out) {
        queries++; out.signedGap = 1; out.signedDistance = radius + 1; out.segmentT = .5;
        out.inward.values.set([0, 1, 0]); out.closestPoint.values.set([.5 * (ax + bx), .5 * (ay + by) - radius - 1, .5 * (az + bz)]);
        out.source = 'unsupported-test'; out.capsuleSampleCount = 2; return out;
    } };
    const loads = new Float64Array(f.state.layout.dofCount); loads[f.state.layout.positions[2]] = .002;
    const options = { ...f.options, loads, wall: { ...f.options.wall, field } };
    const free = advanceCompositeTimeStep(f.state, { ...options, wall: null });
    const open = advanceCompositeTimeStep(f.state, options);
    assert.ok(free.accepted, JSON.stringify(free.diagnostics)); assert.ok(open.accepted, JSON.stringify(open.diagnostics));
    assert.deepEqual(open.state.data, free.state.data); assert.deepEqual(open.state.materialVelocities, free.state.materialVelocities);
    assert.equal(open.diagnostics.mixedCount, free.diagnostics.mixedCount);
    assert.equal(open.diagnostics.wallQueries, queries); assert.ok(queries > 1);
    assert.ok(open.state.wallContactState.multipliers.every(v => v === 0));
    assert.equal(open.diagnostics.certificate.wall.maximumPenetration, 0);
    // A nonzero physical force at an open gap still enters stationarity and
    // must be released by the actual force column. Unknown derivatives cannot
    // be ignored merely because geometric penetration is zero.
    const loaded = { ...f.state, wallContactState: { contactMode: 'capsule', owners: ['wire', null], multipliers: new Float64Array([.1, 0]) } };
    const result = advanceCompositeTimeStep(loaded, options);
    assert.equal(result.accepted, false); assert.equal(result.state, loaded); assert.equal(result.diagnostics.historyCommits, 0);
    assert.match(result.diagnostics.error, /Unsupported active\/loaded wall derivative/);
});

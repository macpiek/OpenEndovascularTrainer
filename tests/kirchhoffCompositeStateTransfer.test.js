import assert from 'node:assert/strict';
import test from 'node:test';
import { refineCompositeState as refine, sampleCompositeTransferredMaterial as sample } from '../src/physics/kirchhoffCompositeStateTransfer.js';
import { createCompositeChainLayout, createCompositeChainWorkspace, assembleCompositeChain } from '../src/physics/kirchhoffCompositeChain.js';
import { captureCompositeReferenceFrames, compileCompositeMaterial } from '../src/physics/kirchhoffCompositeElement.js';
import { evaluateCompositeKinematics } from '../src/physics/kirchhoffCompositeKinematics.js';
import { buildKirchhoffCompositeMesh } from '../src/physics/kirchhoffCompositeMesh.js';
import { buildKirchhoffCompositeTopology, compositeToolFromTipProfile } from '../src/physics/kirchhoffCompositeTopology.js';
import { kirchhoffMaterialProfile } from '../src/physics/kirchhoffMaterialProfile.js';

const close = (a, b, tolerance = 1e-11) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b}, tolerance ${tolerance}`);
const tolerance = () => ({ position: 1e-12, rotation: 1e-12, materialLabel: 1e-12, velocity: 1e-12, angularVelocity: 1e-12, rotationGradient: 1e-12,
    mass: 1e-12, momentum: 1e-12, kineticEnergy: 1e-12, energy: 1e-11, force: 1e-10, torque: 1e-10, work: 1e-12 });
function fixture({ curved = false, partial = false } = {}) {
    const coordinates = [0, 2, 4, 6, 8], positions = coordinates.map((x, i) => [x, curved ? [0, .1, .45, .5, .3][i] : 0, curved ? .01 * i * i : 0]);
    const ids = partial ? [['catheter'], ['wire', 'catheter'], ['wire', 'catheter'], ['wire']] : coordinates.slice(1).map(() => ['wire', 'catheter']);
    const layout = createCompositeChainLayout(ids), tools = [...layout.spins.keys()].map(id => ({ id,
        dsDx: id === 'wire' ? 2 : .5, material: compileCompositeMaterial({ EI1: id === 'wire' ? 1 : 3, EI2: id === 'wire' ? 2 : 5, GJ: id === 'wire' ? 2 : 7 }),
        angles: Float64Array.from(ids, e => e.includes(id) ? 0 : NaN),
        referenceTwists: Float64Array.from(layout.hinges, h => h.tools.includes(id) ? 0 : NaN) }));
    const data = { coordinates: Float64Array.from(coordinates), positions, reference: captureCompositeReferenceFrames(positions), tools };
    const state = { data, layout, lengthMultipliers: Float64Array.from([2, -3, .5, 4]), time: .25, step: 30, materialVelocities: null, torsionMode: 'quasi-static' };
    const inertiaEdges = ids.map((e, edge) => ({ tools: e.map(id => {
        const dsDx = id === 'wire' ? 2 : .5;
        return { id, materialMap: { sStart: (id === 'wire' ? 100 : 200) + dsDx * coordinates[edge], dsDx, dsDt: id === 'wire' ? [-1 - edge, -2 - edge] : [.3 + edge, 1.3 + edge] },
            massPerMaterialLength: id === 'wire' ? .04 : .09,
            // Intentionally discontinuous one-sided old velocity traces.
            oldMaterialVelocities: [[.3 + edge, -.2, id === 'wire' ? .1 : -.1], [.5 + edge, .4, id === 'wire' ? .3 : -.3]] };
    }) }));
    return { state, inertiaEdges, options: { coordinates: [0, .3, 1, 2, 2.7, 4, 5, 5.9, 6, 7.1, 8],
        boundaries: [0, 2, 4, 6, 8], targetTools: tools, inertiaEdges, contactHistory: [], tolerances: tolerance() } };
}
function accepted(result) { assert.equal(result.accepted, true, JSON.stringify(result.diagnostics)); }
function loose(options) { return { ...options, tolerances: { ...options.tolerances, energy: 1e9, force: 1e9, torque: 1e9 } }; }
function mutableData(data) { return { ...data, positions: data.positions.map(p => [...p]), tools: data.tools.map(t => ({ ...t, angles: t.angles.slice() })) }; }
function addDof(data, layout, dof, delta) {
    for (const [node, start] of layout.positions.entries()) if (dof >= start && dof < start + 3) { data.positions[node][dof - start] += delta; return; }
    for (const t of data.tools) for (const [edge, row] of layout.spins.get(t.id).entries()) if (row === dof) { t.angles[edge] += delta; return; }
    throw new Error('DOF not found');
}

test('straight refinement preserves the exact old polyline, signed tension and same-time owned state', () => {
    const f = fixture(), before = structuredClone(f.state), fields = structuredClone(f.inertiaEdges), r = refine(f.state, f.options); accepted(r);
    assert.deepEqual(f.state, before); assert.deepEqual(f.inertiaEdges, fields); assert.notEqual(r.state, f.state);
    assert.equal(r.state.time, .25); assert.equal(r.state.step, 30); assert.equal(r.diagnostics.timeAdvanced, 0);
    assert.equal(r.diagnostics.historyCommits, 1); assert.equal(r.diagnostics.energyExact, false);
    r.state.data.positions.forEach((p, i) => assert.deepEqual(p, [f.options.coordinates[i], 0, 0]));
    r.parentEdges.forEach((parent, child) => assert.equal(r.state.lengthMultipliers[child], f.state.lengthMultipliers[parent]));
    r.oldNodeToNewNode.forEach((node, i) => assert.deepEqual(r.state.data.positions[node], f.state.data.positions[i]));
    for (const key of Object.keys(tolerance())) assert.ok(r.diagnostics.errors[key] <= f.options.tolerances[key]);
    for (const edge of r.inertiaEdges) for (const t of edge.tools) assert.equal(t.angularKinematics, null);
    const at = sample(r, { edge: 0, id: 'wire', fraction: .3 });
    assert.equal(at.materialSpin, null); assert.equal(at.oldAngularVelocity, null); assert.equal(at.kinematicsInput.thetaDt, undefined);
    r.state.data.positions[0][0] = 999; r.state.materialVelocities[0].tools[0].velocities[0][0] = 999;
    assert.deepEqual(f.state, before); assert.deepEqual(f.inertiaEdges, fields);
});

test('per-tool affine restriction preserves mass, momentum and kinetic energy by independent endpoint integrals', () => {
    const f = fixture(), r = refine(f.state, f.options); accepted(r);
    function integrate(edges, x) {
        const totals = new Map();
        edges.forEach((e, edge) => e.tools.forEach(t => {
            if (!totals.has(t.id)) totals.set(t.id, { m: 0, p: [0, 0, 0], k: 0 });
            const total = totals.get(t.id), m = t.massPerMaterialLength * t.materialMap.dsDx * (x[edge + 1] - x[edge]), [a, b] = t.oldMaterialVelocities;
            total.m += m;
            for (let i = 0; i < 3; i++) { total.p[i] += m * (a[i] + b[i]) / 2; total.k += m * (a[i] * a[i] + a[i] * b[i] + b[i] * b[i]) / 6; }
        }));
        return totals;
    }
    const a = integrate(f.inertiaEdges, f.state.data.coordinates), b = integrate(r.inertiaEdges, r.state.data.coordinates);
    for (const [id, old] of a) { close(old.m, b.get(id).m); close(old.k, b.get(id).k); old.p.forEach((v, i) => close(v, b.get(id).p[i])); }
    r.parentEdges.forEach((parent, edge) => r.inertiaEdges[edge].tools.forEach(t => {
        const old = f.inertiaEdges[parent].tools.find(p => p.id === t.id), oldX = f.state.data.coordinates;
        for (const fraction of [.137, .613, .919]) {
            const x = r.state.data.coordinates[edge] + fraction * (r.state.data.coordinates[edge + 1] - r.state.data.coordinates[edge]);
            const fOld = (x - oldX[parent]) / (oldX[parent + 1] - oldX[parent]), at = sample(r, { edge, id: t.id, fraction });
            at.oldMaterialVelocity.forEach((v, i) => close(v, old.oldMaterialVelocities[0][i] + fOld * (old.oldMaterialVelocities[1][i] - old.oldMaterialVelocities[0][i])));
        }
    }));
});

test('curved independent material directors and large unwrapped spin/reference winding survive at the same labels', () => {
    const f = fixture({ curved: true });
    f.state.data.tools.forEach((t, index) => {
        t.angles.forEach((_, i) => { t.angles[i] = (index ? -1 : 1) * (12 * Math.PI + .2 * i); });
        t.referenceTwists.forEach((_, i) => { t.referenceTwists[i] = (index ? -1 : 1) * (i + 1) * 2 * Math.PI; });
    });
    const r = refine(f.state, loose(f.options)); accepted(r);
    for (const t of r.state.data.tools) r.parentEdges.forEach((p, child) => assert.equal(t.angles[child], f.state.data.tools.find(a => a.id === t.id).angles[p]));
    const sourceW = assembleCompositeChain(f.state.data, createCompositeChainWorkspace(f.state.layout));
    for (const t of f.state.data.tools) {
        let winding = 0;
        for (let edge = 0; edge < f.state.layout.nodeCount - 1; edge++) {
            if (edge) winding += sourceW.evaluatedReferenceTwists.get(t.id)[edge - 1];
            const child = r.parentEdges.indexOf(edge), at = sample(r, { edge: child, id: t.id, fraction: .4 });
            close(at.orientation.unwrappedPhase, t.angles[edge] + winding, 1e-12);
            const oldFrame = f.state.data.reference[edge], tangent = oldFrame.tangent, d = oldFrame.director;
            const p = [tangent[1] * d[2] - tangent[2] * d[1], tangent[2] * d[0] - tangent[0] * d[2], tangent[0] * d[1] - tangent[1] * d[0]];
            at.orientation.director1.forEach((v, axis) => close(v, Math.cos(t.angles[edge]) * d[axis] + Math.sin(t.angles[edge]) * p[axis], 1e-12));
        }
    }
    assert.ok(r.diagnostics.errors.rotation < 1e-12); assert.ok(r.diagnostics.errors.energy > .01);
});

test('known angular derivative fields retain advection and frame connection without inventing full angular velocity', () => {
    const f = fixture();
    f.inertiaEdges.forEach((e, edge) => e.tools.forEach(t => {
        t.angularKinematics = { thetaDt: [2 + edge, 3 + edge], thetaDx: [.4, .8], frameSpin: { dt: [-.2, .6], dx: [.1, .3] } };
        if (t.id === 'wire') t.oldAngularVelocities = [[.1, .2, .3], [.5, -.2, .7]];
    }));
    const r = refine(f.state, f.options); accepted(r);
    r.parentEdges.forEach((parent, edge) => r.inertiaEdges[edge].tools.forEach(t => {
        const at = sample(r, { edge, id: t.id, fraction: .43 }), x = r.state.data.coordinates[edge] + .43 * (r.state.data.coordinates[edge + 1] - r.state.data.coordinates[edge]);
        const oldF = (x - f.state.data.coordinates[parent]) / 2, old = f.inertiaEdges[parent].tools.find(t0 => t0.id === t.id);
        const st = old.materialMap.dsDt[0] + oldF * (old.materialMap.dsDt[1] - old.materialMap.dsDt[0]), u = -st / old.materialMap.dsDx;
        const expected = 2 + parent + oldF + u * (.4 + .4 * oldF) - .2 + .8 * oldF + u * (.1 + .2 * oldF);
        close(at.materialSpin, expected);
        const actual = evaluateCompositeKinematics({ positionDt: [1, 2, 3], positionDx: [.8, .3, -.1], tools: [at.kinematicsInput] });
        close(actual.tools[0].spin, expected);
        if (t.id === 'catheter') assert.equal(at.oldAngularVelocity, null);
        else at.oldAngularVelocity.forEach((v, i) => close(v, old.oldAngularVelocities[0][i] + oldF * (old.oldAngularVelocities[1][i] - old.oldAngularVelocities[0][i])));
    }));
});

test('DER change is independently measured and rejects a curved remesh under tight caller tolerances', () => {
    const f = fixture({ curved: true }), old = structuredClone(f.state), r = refine(f.state, f.options);
    assert.equal(r.accepted, false); assert.equal(r.status, 'transfer-tolerance-rejected'); assert.equal(r.state, f.state);
    assert.equal(r.diagnostics.historyCommits, 0); assert.deepEqual(f.state, old);
    assert.ok(r.diagnostics.elastic.targetEnergy !== r.diagnostics.elastic.sourceEnergy);
    assert.ok(r.diagnostics.failedCriteria.includes('energy')); assert.ok(r.diagnostics.elastic.detailForce > 0);
});

test('pulled fine-minus-coarse elastic gradient matches independent finite differences on all original DOFs', () => {
    const f = fixture({ curved: true });
    f.state.data.tools.forEach((t, n) => t.angles.forEach((_, edge) => { t.angles[edge] = (n ? -.13 : .2) * edge; }));
    const r = refine(f.state, loose(f.options)); accepted(r);
    const a = mutableData(f.state.data), b = mutableData(r.state.data), wa = createCompositeChainWorkspace(f.state.layout, { elementBackend: 'javascript' }), wb = createCompositeChainWorkspace(r.state.layout, { elementBackend: 'javascript' });
    const scalar = () => assembleCompositeChain(b, wb).energy - assembleCompositeChain(a, wa).energy;
    const change = (dof, delta) => {
        addDof(a, f.state.layout, dof, delta);
        r.prolongation.forEach((row, fine) => row.forEach(([coarse, weight]) => { if (coarse === dof) addDof(b, r.state.layout, fine, delta * weight); }));
    };
    for (let dof = 0; dof < f.state.layout.dofCount; dof++) {
        const h = 1e-6; change(dof, h); const plus = scalar(); change(dof, -2 * h); const minus = scalar(); change(dof, h);
        close((plus - minus) / (2 * h), r.diagnostics.elastic.pulledGradient[dof] - r.diagnostics.elastic.sourceGradient[dof], 2e-8);
    }
});

test('signed axial tension copies onto children and preserves original lambda*g and arbitrary virtual work', () => {
    const f = fixture(); f.state.data.positions.forEach(p => { p[0] *= 1.02; });
    const r = refine(f.state, f.options); accepted(r);
    close(r.diagnostics.length.sourceWork, .02 * 2 * (2 - 3 + .5 + 4));
    close(r.diagnostics.length.sourceWork, r.diagnostics.length.targetWork);
    const coarseDelta = Float64Array.from({ length: f.state.layout.dofCount }, (_, i) => Math.sin(i * .7));
    const fineDelta = r.prolongation.map(row => row.reduce((sum, [col, weight]) => sum + weight * coarseDelta[col], 0));
    close(r.diagnostics.length.sourceGradient.reduce((sum, v, i) => sum + v * coarseDelta[i], 0),
        r.diagnostics.length.targetGradient.reduce((sum, v, i) => sum + v * fineDelta[i], 0));
});

test('persistent contact feet map once with material labels, opposite wrenches and owned opaque history intact', () => {
    const f = fixture(), payload = { normalLambda: 2, traction: [.4, -.2], stickAnchor: [102, .3], flags: new Uint8Array([1, 0]) };
    f.options.contactHistory = [
        { id: 'wire-contact', edge: 0, fraction: .5, owner: 'wire', s: 102, worldForce: [1, 2, 3], worldCouple: [.1, -.2, .4], payload },
        { id: 'catheter-reaction', edge: 0, fraction: .5, owner: 'catheter', s: 200.5, worldForce: [-1, -2, -3], worldCouple: [-.1, .2, -.4], payload: { normalLambda: 2 } },
        { id: 'distal-foot', edge: 3, fraction: 1, owner: 'wire', s: 116, worldForce: [0, 1, 0], worldCouple: [0, 0, .2], payload: { age: 19 } }
    ];
    f.state.contactHistory = f.options.contactHistory;
    const old = structuredClone(f.state), r = refine(f.state, f.options); accepted(r);
    assert.equal(r.state.contactHistory.length, 3); assert.deepEqual(f.state, old);
    assert.equal(r.state.contactHistory[0].edge, 2); assert.equal(r.state.contactHistory[0].fraction, 0);
    assert.equal(r.state.contactHistory[2].edge, r.state.layout.nodeCount - 2); assert.equal(r.state.contactHistory[2].fraction, 1);
    r.state.contactHistory.forEach((record, i) => {
        assert.equal(record.s, f.options.contactHistory[i].s); assert.deepEqual(record.worldForce, f.options.contactHistory[i].worldForce);
        assert.deepEqual(record.worldCouple, f.options.contactHistory[i].worldCouple); assert.deepEqual(record.payload, f.options.contactHistory[i].payload);
    });
    r.state.contactHistory[0].payload.flags[0] = 9; assert.equal(payload.flags[0], 1);
    r.diagnostics.contact.pulledForce.forEach((v, i) => close(v, r.diagnostics.contact.sourceForce[i]));
});

test('exact tool boundaries, inactive spins and independently rooted tool intervals remain explicit', () => {
    const f = fixture({ partial: true }), r = refine(f.state, f.options); accepted(r);
    r.parentEdges.forEach((parent, child) => {
        assert.deepEqual(r.state.layout.edgeToolIds[child], f.state.layout.edgeToolIds[parent]);
        for (const t of r.state.data.tools) if (!f.state.layout.edgeToolIds[parent].includes(t.id)) {
            assert.equal(r.state.layout.spins.get(t.id)[child], -1); assert.ok(Number.isNaN(t.angles[child]));
        }
    });
    // Prolongation is a field/virtual-work map. A proximal physical handle
    // selects proximalDof, whereas an entire prescribed old edge field would
    // constrain all children. The transfer never chooses that support for us.
    const root = f.state.layout.spins.get('wire').find(dof => dof >= 0), rows = r.prolongation.map((row, i) => row.some(([col]) => col === root) ? i : -1).filter(i => i >= 0);
    assert.equal(rows.length, r.parentEdges.filter(parent => parent === 1).length);
    const spin = r.spinDofMap.find(s => s.oldDof === root);
    assert.deepEqual(spin.childDofs, rows); assert.equal(spin.proximalDof, rows[0]); assert.equal(spin.distalDof, rows.at(-1));
});

test('coarsening, moved exact boundaries, mismatched target map and missing tolerance reject without state changes', () => {
    const f = fixture(), before = structuredClone(f.state);
    for (const opts of [
        { ...f.options, coordinates: [0, 1, 2, 3, 6, 8] },
        { ...f.options, boundaries: [0, 2 + 1e-12, 8] },
        { ...f.options, targetTools: f.options.targetTools.map(t => ({ ...t, dsDx: 1 })) },
        { ...f.options, targetTools: [] },
        { ...f.options, contactHistory: undefined }
    ]) { const r = refine(f.state, opts); assert.equal(r.accepted, false); assert.equal(r.state, f.state); assert.equal(r.diagnostics.historyCommits, 0); assert.deepEqual(f.state, before); }
    assert.throws(() => refine(f.state, { ...f.options, tolerances: {} }), /tolerance/);
});

test('target dual-cell materials are sampled anew and frozen independently of nonlinear assembly', () => {
    const f = fixture(), calls = new Map();
    const targetTools = f.state.data.tools.map(t => ({ id: t.id, dsDx: t.dsDx, materialAt({ vertex, start, end }) {
        const key = `${t.id}:${vertex}`; calls.set(key, { count: (calls.get(key)?.count ?? 0) + 1, start, end }); return t.material;
    } }));
    const r = refine(f.state, { ...f.options, targetTools }); accepted(r);
    assert.equal(calls.size, 2 * (r.state.layout.nodeCount - 2)); assert.ok([...calls.values()].every(c => c.count === 1));
    const w = createCompositeChainWorkspace(r.state.layout); assembleCompositeChain(r.state.data, w); assembleCompositeChain(r.state.data, w);
    assert.ok([...calls.values()].every(c => c.count === 1));
    const first = calls.get('wire:1'); close(first.start, .15); close(first.end, .65);
});

test('successive same-time refinements reuse known velocity and point histories without an implicit dt', () => {
    const f = fixture(), first = refine(f.state, f.options); accepted(first);
    const savedPositions = first.state.data.positions.map(p => [...p]), secondX = [...first.state.data.coordinates, .15, 7.5].sort((a, b) => a - b);
    const second = refine(first.state, { ...f.options, coordinates: secondX, inertiaEdges: first.inertiaEdges,
        contactHistory: first.state.contactHistory, targetTools: f.state.data.tools }); accepted(second);
    assert.equal(second.state.time, first.state.time); assert.equal(second.state.step, first.state.step);
    assert.deepEqual(first.state.data.positions, savedPositions);
    const wrong = structuredClone(first.inertiaEdges); wrong[0].tools[0].oldMaterialVelocities[0][0] += .1;
    const failed = refine(first.state, { ...f.options, coordinates: secondX, inertiaEdges: wrong, contactHistory: first.state.contactHistory });
    assert.equal(failed.accepted, false); assert.equal(failed.state, first.state); assert.ok(failed.diagnostics.errors.velocity > .09);
});

test('a second refinement preserves known angular history and rejects missing or altered derivative/full-omega fields', () => {
    const f = fixture();
    f.inertiaEdges.forEach(e => e.tools.forEach(t => {
        t.angularKinematics = { thetaDt: [1, 2], thetaDx: [.1, .4], frameSpin: { dt: [.2, .3], dx: [-.1, .2] } };
        t.oldAngularVelocities = [[.3, .1, -.1], [.4, -.2, .8]];
    }));
    const first = refine(f.state, f.options); accepted(first);
    const options = { ...f.options, coordinates: [...first.state.data.coordinates, .15].sort((a, b) => a - b), inertiaEdges: first.inertiaEdges,
        contactHistory: first.state.contactHistory };
    const second = refine(first.state, options); accepted(second);
    assert.ok(second.state.materialVelocities.every(e => e.tools.every(t => t.angularVelocity !== null && t.materialSpin !== null)));
    for (const change of [t => { t.angularKinematics = null; }, t => { t.oldAngularVelocities = null; },
        t => { t.angularKinematics.thetaDt[0] += .1; }, t => { t.oldAngularVelocities[0][0] += .1; },
        t => { t.angularKinematics.thetaDt[0] += .1; t.angularKinematics.frameSpin.dt[0] -= .1; },
        t => { t.angularKinematics.thetaDx[0] += .1; t.angularKinematics.frameSpin.dx[0] -= .1; }]) {
        const modified = structuredClone(first.inertiaEdges); change(modified[0].tools[0]);
        const r = refine(first.state, { ...options, inertiaEdges: modified });
        assert.equal(r.accepted, false); assert.equal(r.state, first.state); assert.equal(r.diagnostics.historyCommits, 0);
    }
});

test('real steel-J and pigtail mesh providers are rebuilt on refined dual cells and their DER change is reported', () => {
    const topology = buildKirchhoffCompositeTopology({
        wire: compositeToolFromTipProfile({ profile: kirchhoffMaterialProfile('steel-j-035'), insertion: 130, materialInterval: [0, 1000] }),
        catheter: compositeToolFromTipProfile({ profile: kirchhoffMaterialProfile('pigtail'), insertion: 120, materialInterval: [20, 820] }) });
    const build = coordinates => buildKirchhoffCompositeMesh({ topology, meshCoordinates: coordinates,
        sampleCenterline: x => [x, 0, 0], spinFields: { wire: .1, catheter: -.2 } });
    const coarse = build(Array.from({ length: 27 }, (_, i) => i * 5));
    const refinedX = Array.from(coarse.data.coordinates).flatMap((x, i, all) => i + 1 < all.length ? [x, (x + all[i + 1]) / 2] : [x]);
    const fine = build(refinedX), state = { data: coarse.data, layout: coarse.layout, time: .5, step: 60,
        lengthMultipliers: new Float64Array(coarse.layout.nodeCount - 1), materialVelocities: null };
    const inertiaEdges = coarse.layout.edgeToolIds.map((ids, edge) => ({ tools: ids.map(id => ({ id,
        massPerMaterialLength: id === 'wire' ? .002 : .004,
        materialMap: { ...coarse.materialMaps.get(id)[edge], dsDt: id === 'wire' ? -2 : 1 },
        oldMaterialVelocities: [[id === 'wire' ? 2 : -1, 0, 0], [id === 'wire' ? 2 : -1, 0, 0]] })) }));
    const options = { coordinates: refinedX, boundaries: topology.boundaries.map(b => b.x), targetTools: fine.data.tools,
        inertiaEdges, contactHistory: [], tolerances: { ...tolerance(), energy: 1e9, force: 1e9, torque: 1e9 } };
    const result = refine(state, options); accepted(result);
    const oracle = assembleCompositeChain(fine.data, createCompositeChainWorkspace(fine.layout, { elementBackend: 'javascript' }));
    close(result.diagnostics.elastic.targetEnergy, oracle.energy, 1e-8);
    assert.ok(result.diagnostics.errors.energy > 1e-6);
    assert.equal(result.diagnostics.energyExact, false); assert.equal(result.state.time, .5);
    const rejected = refine(state, { ...options, tolerances: tolerance() });
    assert.equal(rejected.accepted, false); assert.equal(rejected.state, state);
    assert.ok(rejected.diagnostics.failedCriteria.includes('energy'));
});

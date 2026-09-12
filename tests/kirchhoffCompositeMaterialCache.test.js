import assert from 'node:assert/strict';
import test from 'node:test';
import { createKirchhoffCompositeMaterialCache } from '../src/physics/kirchhoffCompositeMaterialCache.js';
import { createKirchhoffCompositeMeshUpdate } from '../src/physics/kirchhoffCompositeMeshUpdate.js';
import { buildKirchhoffCompositeTopology } from '../src/physics/kirchhoffCompositeTopology.js';
import { buildKirchhoffCompositeMesh } from '../src/physics/kirchhoffCompositeMesh.js';
import { kirchhoffMaterialProfile } from '../src/physics/kirchhoffMaterialProfile.js';
import { assembleCompositeChain, createCompositeChainWorkspace, solveCompositeChainIncrement } from '../src/physics/kirchhoffCompositeChain.js';

const close = (a, b, tolerance = 2e-9) => assert.ok(Math.abs(a - b) <= tolerance * Math.max(1, Math.abs(a), Math.abs(b)), `${a} != ${b}`);
const arrayClose = (a, b, tolerance) => { assert.equal(a.length, b.length); a.forEach((v, i) => close(v, b[i], tolerance)); };
const pose = x => [x, .23 * Math.sin(x / 23), .17 * Math.cos(x / 31)];
const spinFields = { wire: (s, c) => .001 * s + .002 * c.x, catheter: s => -.002 * s };
const winding = { wire: 4 * Math.PI, catheter: -2 * Math.PI };
function profileInputs(cache, insertion, extra = {}) {
    return { wire: cache.profileTool({ profile: kirchhoffMaterialProfile('glidewire'), materialInterval: [0, 500], insertion: 318,
        radius: .4445, ...extra.wire }), catheter: cache.profileTool({ profile: kirchhoffMaterialProfile('berenstein'),
        materialInterval: [0, 500], insertion, radius: .8, innerRadius: .485, ...extra.catheter }) };
}
function coordinates(topology) {
    const points = [topology.interval[0]];
    for (const section of topology.sections) {
        const n = Math.ceil((section.end - section.start) / 5);
        for (let i = 1; i <= n; i++) points.push(i === n ? section.end : section.start + (section.end - section.start) * i / n);
    }
    return points;
}
function build(topology, extra = {}) {
    return buildKirchhoffCompositeMesh({ topology, meshCoordinates: coordinates(topology), sampleCenterline: pose,
        spinFields, referenceTwistFields: winding, ...extra });
}
function compareMeshes(a, b) {
    arrayClose(a.data.coordinates, b.data.coordinates, 0);
    assert.deepEqual(a.layout, b.layout); assert.deepEqual(a.materialMaps, b.materialMaps);
    assert.deepEqual(a.contactOwners, b.contactOwners);
    for (const category of ['materialCells', 'boundaryCells']) {
        assert.equal(a[category].length, b[category].length);
        a[category].forEach((cell, i) => {
            const other = b[category][i];
            for (const key of ['start', 'end', 'nominalLength', 'materialLength', 'integratedMaterialLength']) close(cell[key], other[key], 0);
            arrayClose(cell.material.stiffness, other.material.stiffness, 3e-12);
            arrayClose(cell.material.intrinsic, other.material.intrinsic, 3e-10);
            close(cell.material.energyOffset, other.material.energyOffset, 2e-9);
            close(cell.energyAtZeroStrain, other.energyAtZeroStrain, 2e-9);
            assert.deepEqual(cell.pieces, other.pieces);
        });
    }
    const response = mesh => {
        const ws = createCompositeChainWorkspace(mesh.layout);
        assembleCompositeChain(mesh.data, ws);
        // Multi-turn winding intentionally creates large torsional loads.
        // A declared identical diagonal conditions this operator parity test;
        // it is not a timestep or a material modification in the cache.
        const result = solveCompositeChainIncrement(ws, { diagonal: new Float64Array(mesh.layout.dofCount).fill(1000), tolerance: 1e-7 });
        assert.equal(result.converged, true, `Direction residual ${result.maximumResidual}`);
        return ws;
    };
    const x = response(a), y = response(b);
    close(x.energy, y.energy, 3e-10); arrayClose(x.gradient, y.gradient, 3e-9);
    arrayClose(x.hessian, y.hessian, 3e-10); arrayClose(x.increment, y.increment, 3e-8);
    arrayClose(x.reactions, y.reactions, 3e-8);
}

test('real Glidewire/Berenstein advance and retract preserve full E/g/H and solve with independent maps', () => {
    for (const base of [9.17, 160.17, 310.17]) {
        const cache = createKirchhoffCompositeMaterialCache(), input = profileInputs(cache, base,
            { wire: { dsDx: 1.03 }, catheter: { dsDx: .97 } });
        for (const delta of [0, .013, .031, -.019, 0]) {
            const topology = buildKirchhoffCompositeTopology({ ...input, catheter: { ...input.catheter, insertion: base + delta },
                sheath: { interval: [0, 3.7], innerRadius: .9 } });
            const oracle = build(topology), fast = build(topology, { materialIntegrator: cache.integrator });
            compareMeshes(oracle, fast);
            assert.equal(fast.admission.certified, false);
            for (const boundary of topology.boundaries) assert.ok(fast.data.coordinates.includes(boundary.x));
        }
        assert.ok(cache.statistics.constantCells > 0); assert.ok(cache.statistics.cacheHits > 0);
        assert.ok(cache.statistics.adaptiveProfileCells > 0); assert.ok(cache.statistics.polynomialCells > 0);
    }
});

test('known variable Glidewire uses the original accepted adaptive partition and tensor integral', () => {
    const cache = createKirchhoffCompositeMaterialCache(), input = profileInputs(cache, 9.3);
    const topology = buildKirchhoffCompositeTopology(input), oracle = build(topology), fast = build(topology, { materialIntegrator: cache.integrator });
    for (const cell of fast.materialCells.filter(c => c.quadrature.rule === 'profile-adaptive-simpson-positive-boole')) {
        const other = oracle.materialCells.find(c => c.id === cell.id && c.vertex === cell.vertex);
        assert.deepEqual(Array.from(cell.material.stiffness), Array.from(other.material.stiffness));
        assert.equal(cell.quadrature.maximumDepth, other.quadrature.maximumDepth);
        assert.equal(cell.quadrature.maximumEstimatedComponentError, other.quadrature.maximumEstimatedComponentError);
        assert.equal(cell.quadrature.evaluations, other.quadrature.evaluations);
        assert.equal(cell.material.energyOffset, 0);
    }
});

test('Berenstein positive polynomial moments match independent composite midpoint energy integration', () => {
    const cache = createKirchhoffCompositeMaterialCache(), input = profileInputs(cache, 25.13);
    const topology = buildKirchhoffCompositeTopology(input), mesh = build(topology, { materialIntegrator: cache.integrator });
    const profile = kirchhoffMaterialProfile('berenstein');
    for (const cell of mesh.materialCells.filter(c => c.id === 'catheter' && c.material.energyOffset > 1e-8)) {
        for (const curvature of [-.05, .04, .16]) {
            let oracle = 0; const steps = 20000, h = (cell.end - cell.start) / steps;
            for (let j = 0; j < steps; j++) {
                const x = cell.start + (j + .5) * h, m = profile.sample(input.catheter.insertion - x);
                oracle += .5 * m.EI1 * (curvature - m.kappa01) ** 2 * h;
            }
            const m = cell.material, energy = cell.materialLength * (.5 * m.stiffness[0] * (curvature - m.intrinsic[0]) ** 2 + m.energyOffset);
            close(energy, oracle, 3e-8);
        }
    }
});

test('explicit copied constant spans keep anisotropic tensors, jumps, positive mismatch and offsets', () => {
    const cache = createKirchhoffCompositeMaterialCache();
    const first = { stiffness: [[8, 1, .5], [1, 5, -.2], [.5, -.2, 3]], intrinsic: [1e6 + .02, -.1, .04], energyOffset: .07 };
    const second = { stiffness: [[3, -.4, .1], [-.4, 7, .8], [.1, .8, 6]], intrinsic: [1e6 - .03, .4, -.02], energyOffset: .13 };
    const wire = cache.constantTool({ materialInterval: [0, 10], insertion: 10, dsDx: 2,
        spans: [{ start: 0, end: 5, material: first }, { start: 5, end: 10, material: second }] });
    first.stiffness[0][0] = -999; second.intrinsic[0] = Infinity;
    const topology = buildKirchhoffCompositeTopology({ wire, interval: [5, 10] });
    const oracle = build(topology), fast = build(topology, { materialIntegrator: cache.integrator });
    compareMeshes(oracle, fast);
    const jump = fast.materialCells.find(c => c.pieces.length === 2);
    assert.ok(jump.material.energyOffset > .07);
    assert.ok(cache.statistics.fallbackCells > 0); // General anisotropic jump uses exact oracle integration.
    assert.ok(cache.statistics.constantCells > 0);
    assert.throws(() => { fast.boundaryCells[0].material.stiffness[0] = 0; }, TypeError);
});

test('unknown/mutable profile and a familiar id never authorize a constant or support cache', () => {
    const cache = createKirchhoffCompositeMaterialCache(); let scale = 1;
    const profile = { id: 'berenstein', sample: s => ({ EI1: scale * (10 + .01 * s), GJ: 3, kappa01: .002 * s }) };
    const wire = cache.profileTool({ profile, insertion: 10, materialInterval: [0, 100] });
    const topology = buildKirchhoffCompositeTopology({ wire });
    const a = build(topology, { materialIntegrator: cache.integrator }); scale = 2;
    const b = build(topology, { materialIntegrator: cache.integrator });
    close(b.materialCells[0].material.stiffness[0], 2 * a.materialCells[0].material.stiffness[0]);
    assert.equal(cache.statistics.cacheHits, 0); assert.equal(cache.statistics.entries, 0);
    assert.ok(cache.statistics.fallbackCells > 0);
});

test('error budget, map, boundary splits and profile identity invalidate exact support entries', () => {
    const cache = createKirchhoffCompositeMaterialCache({ maximumEntriesPerProfile: 4 });
    const input = profileInputs(cache, 9.1), topology = buildKirchhoffCompositeTopology(input);
    build(topology, { materialIntegrator: cache.integrator });
    assert.ok(cache.statistics.evictions > 0); assert.ok(cache.statistics.entries <= 8);
    const strict = { absoluteTolerance: 0, relativeTolerance: 1e-15, maxDepth: 0 };
    assert.throws(() => build(topology, { materialIntegrator: cache.integrator, quadrature: strict }), /did not converge/);
    assert.throws(() => build(topology, { quadrature: strict }), /did not converge/);
    for (const changes of [{ wire: { ...input.wire, dsDx: 1.01 } }, { boundaries: [6.25] },
        { catheter: { ...input.catheter, insertion: 9.12 } }]) {
        const current = buildKirchhoffCompositeTopology({ ...input, ...changes });
        compareMeshes(build(current), build(current, { materialIntegrator: cache.integrator }));
    }
});

test('persistent fixed mesh refresh retains position/frame/spin/winding objects and all Voronoi cells', () => {
    const cache = createKirchhoffCompositeMaterialCache();
    const input = profileInputs(cache, 340), topology = buildKirchhoffCompositeTopology({ ...input, interval: [0, 100] });
    const mesh = build(topology, { materialIntegrator: cache.integrator });
    const refs = { positions: mesh.data.positions, reference: mesh.data.reference, layout: mesh.layout,
        coordinates: mesh.data.coordinates, tools: mesh.data.tools, cells: mesh.materialCells,
        maps: mesh.materialMaps.get('catheter'), angles: mesh.data.tools[1].angles, twists: mesh.data.tools[1].referenceTwists };
    const snapshot = structuredClone(mesh.data.reference);
    const updater = createKirchhoffCompositeMeshUpdate(mesh, { materialIntegrator: cache.integrator });
    for (const delta of [.3, -.1, .7]) {
        const current = buildKirchhoffCompositeTopology({ ...input, catheter: { ...input.catheter, insertion: 340 + delta }, interval: [0, 100] });
        const result = updater.refresh(current); assert.equal(result.updated, true);
        const oracle = build(current, { spinFields: Object.fromEntries(mesh.data.tools.map(t => [t.id, (_, c) => t.angles[c.edge]])),
            referenceFrames: mesh.data.reference, referenceTwistFields: winding });
        compareMeshes(oracle, mesh);
        for (const key of ['positions', 'reference', 'coordinates', 'tools']) assert.equal(mesh.data[key], refs[key]);
        assert.equal(mesh.layout, refs.layout); assert.equal(mesh.materialCells, refs.cells);
        assert.equal(mesh.materialMaps.get('catheter'), refs.maps); assert.equal(mesh.data.tools[1].angles, refs.angles);
        assert.equal(mesh.data.tools[1].referenceTwists, refs.twists); assert.deepEqual(mesh.data.reference, snapshot);
    }
});

test('moving tip/material/sheath or ownership change is an untouched candidate requiring state transfer', () => {
    const cache = createKirchhoffCompositeMaterialCache(), input = profileInputs(cache, 9.1);
    const topology = buildKirchhoffCompositeTopology(input), mesh = build(topology, { materialIntegrator: cache.integrator });
    const updater = createKirchhoffCompositeMeshUpdate(mesh, { materialIntegrator: cache.integrator });
    const before = { coordinates: mesh.data.coordinates.slice(), materials: mesh.materialCells.map(c => c.material),
        maps: structuredClone(mesh.materialMaps), positions: structuredClone(mesh.data.positions), frame: structuredClone(mesh.data.reference) };
    for (const delta of [.01, -.01, 2, -2]) {
        const current = buildKirchhoffCompositeTopology({ ...input, catheter: { ...input.catheter, insertion: 9.1 + delta } });
        const result = updater.refresh(current);
        assert.equal(result.updated, false); assert.equal(result.status, 'candidate-needs-state-transfer');
        for (const boundary of current.boundaries) assert.ok(result.proposedCoordinates.includes(boundary.x));
        assert.deepEqual(mesh.data.coordinates, before.coordinates); assert.deepEqual(mesh.materialMaps, before.maps);
        assert.deepEqual(mesh.data.positions, before.positions); assert.deepEqual(mesh.data.reference, before.frame);
        mesh.materialCells.forEach((c, i) => assert.equal(c.material, before.materials[i]));
    }
    const sheath = updater.refresh(buildKirchhoffCompositeTopology({ ...input, sheath: { interval: [0, 2.7], innerRadius: .9 } }));
    assert.equal(sheath.updated, false); assert.ok(sheath.missingBoundaries.includes(2.7));
});

test('failed material refresh is atomic and does not keep earlier section approvals', () => {
    const cache = createKirchhoffCompositeMaterialCache(), input = profileInputs(cache, 340);
    const topology = buildKirchhoffCompositeTopology({ ...input, interval: [0, 100] });
    const mesh = build(topology, { materialIntegrator: cache.integrator, approvedSections: topology.sections });
    const saved = mesh.materialCells.map(c => c.material);
    const bad = createKirchhoffCompositeMeshUpdate(mesh, { materialIntegrator: () => { throw new Error('quadrature failed'); } });
    assert.throws(() => bad.refresh(topology), /quadrature failed/);
    mesh.materialCells.forEach((c, i) => assert.equal(c.material, saved[i]));
    const good = createKirchhoffCompositeMeshUpdate(mesh, { materialIntegrator: cache.integrator });
    assert.equal(good.refresh(topology).updated, true); assert.equal(mesh.admission.status, 'candidate');
});

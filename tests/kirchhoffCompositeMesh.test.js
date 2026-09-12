import assert from 'node:assert/strict';
import test from 'node:test';
import { buildKirchhoffCompositeMesh as mesh } from '../src/physics/kirchhoffCompositeMesh.js';
import { buildKirchhoffCompositeTopology as topology, compositeToolFromTipProfile } from '../src/physics/kirchhoffCompositeTopology.js';
import { assembleCompositeChain, createCompositeChainWorkspace } from '../src/physics/kirchhoffCompositeChain.js';
import { kirchhoffMaterialProfile } from '../src/physics/kirchhoffMaterialProfile.js';
import { GUIDEWIRE_RADIUS_MM, PIGTAIL_CATHETER_RADIUS_MM, PIGTAIL_CATHETER_INNER_RADIUS_MM } from '../src/toolDimensions.js';
import { STEEL_J_GUIDEWIRE_TIP_CORE_LENGTH_MM, STEEL_J_GUIDEWIRE_TIP_TRANSITION_LENGTH_MM,
    STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM, STEEL_J_GUIDEWIRE_CURVATURE_TRANSITION_MM } from '../src/physics/guidewireMaterialProfile.js';
import { PIGTAIL_NATURAL_ARC_LENGTH_MM, PIGTAIL_CURVATURE_TRANSITION_MM } from '../src/physics/catheterMaterialProfile.js';

const close = (a, b, tolerance = 1e-9) => assert.ok(Math.abs(a - b) <= tolerance * Math.max(1, Math.abs(a), Math.abs(b)), `${a} != ${b}`);
const material = (EI = 10, GJ = 4, kappa0 = [0, 0]) => ({ EI1: EI, EI2: EI, GJ, kappa0 });
const tool = (insertion, extra = {}) => ({ insertion, materialInterval: [0, 100], material: material(), ...extra });
const build = (t, coordinates, extra = {}) => mesh({ topology: t, meshCoordinates: coordinates,
    sampleCenterline: x => [x, 0, 0], spinFields: { wire: 0, catheter: 0 }, ...extra });
const assembled = result => assembleCompositeChain(result.data, createCompositeChainWorkspace(result.layout));
const matvec = (k, x) => [0, 1, 2].map(i => k[3 * i] * x[0] + k[3 * i + 1] * x[1] + k[3 * i + 2] * x[2]);
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
const cellEnergy = (cell, strain) => {
    const error = strain.map((v, i) => v - cell.material.intrinsic[i]);
    return cell.materialLength * (.5 * dot(error, matvec(cell.material.stiffness, error)) + cell.material.energyOffset);
};

test('one mesh inserts exact tip/material/sheath boundaries and maps each surface owner once', () => {
    const t = topology({ wire: tool(10), catheter: tool(4.25, { materialBreakpoints: [96, 98], radius: .8, innerRadius: .5 }),
        sheath: { interval: [0, 1.25], innerRadius: .9 } });
    const result = build(t, [0, 2, 4, 6, 8, 10]);
    assert.deepEqual(result.insertedCoordinates, [.25, 1.25, 2.25, 4.25]);
    assert.deepEqual(Array.from(result.data.coordinates), [0, .25, 1.25, 2, 2.25, 4, 4.25, 6, 8, 10]);
    for (let i = 0; i < result.layout.edgeToolIds.length; i++) {
        const owner = result.contactOwners.edges[i];
        assert.equal(owner.sectionIndex, result.edgeSections[i]);
        assert.deepEqual(result.layout.edgeToolIds[i], owner.start < 4.25 ? ['wire', 'catheter'] : ['wire']);
        assert.equal(owner.start < 1.25 ? owner.sheath.owner : owner.wall.owner, owner.start < 4.25 ? 'catheter' : 'wire');
    }
    const portals = result.contactOwners.nodes.filter(v => v.portal);
    assert.equal(portals.length, 1); assert.equal(result.data.coordinates[portals[0].node], 4.25);
    assert.equal(result.layout.dofCount, 3 * 10 + 9 + 6);
    assert.equal(result.admission.status, 'candidate'); assert.equal(result.admission.clearanceCertified, false);
    assert.throws(() => build(t, [0, 2, 4, 6, 8, 10], { boundaryPolicy: 'reject' }), /misses topology boundary/);
    assert.ok(Number.isFinite(assembled(result).energy));
});

test('the first short tool insertion receives a midpoint and a real independent hinge', () => {
    for (const insertion of [.01, .1, .3]) {
        const result = build(topology({ wire: tool(1), catheter: tool(insertion) }), [0, 1]);
        assert.ok(result.strainCoordinates.includes(insertion / 2));
        assert.equal(result.layout.edgeToolIds.filter(ids => ids.includes('catheter')).length, 2);
        assert.equal(result.materialCells.filter(cell => cell.id === 'catheter').length, 1);
        assert.equal(result.layout.spins.get('catheter').filter(i => i >= 0).length, 2);
        assert.ok(Number.isFinite(assembled(result).energy));
    }
});

test('each edge samples independent material labels and scalar spins with unequal dsDx', () => {
    const seen = [];
    const t = topology({ wire: tool(4, { materialInterval: [20, 40], dsDx: 2 }),
        catheter: tool(3, { materialInterval: [100, 130], dsDx: .5 }) });
    const result = build(t, [0, 1, 2, 4], { spinFields: {
        wire: (s, context) => { seen.push(context); return .03 * s; }, catheter: s => -.02 * s
    } });
    for (const data of result.data.tools) for (let edge = 0; edge < data.angles.length; edge++) {
        const map = result.materialMaps.get(data.id)[edge];
        if (map === null) { assert.ok(Number.isNaN(data.angles[edge])); continue; }
        const original = t.tools.find(tool => tool.id === data.id), x = (result.data.coordinates[edge] + result.data.coordinates[edge + 1]) / 2;
        close(map.s, original.materialInterval[1] + original.dsDx * (x - original.insertion));
        close(data.angles[edge], (data.id === 'wire' ? .03 : -.02) * map.s);
        close(map.sEnd - map.sStart, map.dsDx * (result.data.coordinates[edge + 1] - result.data.coordinates[edge]));
    }
    assert.ok(seen.every(context => context.reference === result.data.reference[context.edge]));
    const response = assembled(result);
    assert.ok(response.energy > 0);
});

test('a dualcell straddling a material jump preserves the entire energy polynomial and mismatch', () => {
    const t = topology({ wire: tool(2, { materialInterval: [0, 2], materialBreakpoints: [1],
        material: (s, { side }) => material(s < 1 || s === 1 && side === 'left' ? 2 : 8, 4,
            [s < 1 || s === 1 && side === 'left' ? 1 : 3, 0]) }) });
    const result = build(t, [0, 2]), cell = result.materialCells[0];
    assert.deepEqual(cell.pieces.map(p => [p.start, p.end]), [[.5, 1], [1, 1.5]]);
    close(cell.material.stiffness[0], 5); close(cell.material.intrinsic[0], 2.6);
    close(cell.material.energyOffset, 1.6);
    for (const curvature of [-2, 0, 2.6, 5]) close(cellEnergy(cell, [curvature, 0, 0]),
        .25 * (2 * (curvature - 1) ** 2 + 8 * (curvature - 3) ** 2));
    close(assembled(result).energy, 18.5);
    close(result.boundaryCells.reduce((sum, c) => sum + c.energyAtZeroStrain, 0), 18.5);
    assert.ok(result.boundaryCells.every(c => c.assembled === false && c.strain === null));
    assert.deepEqual(result.boundaryCells.map(c => c.geometry), [
        { edge: 0, positionNodes: [0, 1], startFraction: 0, endFraction: .5 },
        { edge: 1, positionNodes: [1, 2], startFraction: .5, endFraction: 1 }
    ]);
    close(assembled(result).energy + result.boundaryCells.reduce((sum, c) => sum + c.energyAtZeroStrain, 0), 37);
});

test('anisotropic bend/twist tensors integrate K*kappa0 rather than arithmetic intrinsic means', () => {
    const tensors = [
        { stiffness: [[8, 1, .5], [1, 5, -.2], [.5, -.2, 3]], intrinsic: [.2, -.1, .04] },
        { stiffness: [[3, -.4, .1], [-.4, 7, .8], [.1, .8, 6]], intrinsic: [-.3, .4, -.02] }
    ];
    const t = topology({ wire: tool(2, { materialInterval: [0, 2], dsDx: 1, materialBreakpoints: [1],
        material: (s, { side }) => tensors[s < 1 || s === 1 && side === 'left' ? 0 : 1] }) });
    const cell = build(t, [0, 2]).materialCells[0];
    for (const strain of [[0, 0, 0], [.1, .2, -.3], [-.6, .4, .2]]) {
        const oracle = tensors.reduce((sum, tensor) => {
            const error = strain.map((v, i) => v - tensor.intrinsic[i]);
            return sum + .25 * dot(error, matvec(tensor.stiffness.flat(), error));
        }, 0);
        close(cellEnergy(cell, strain), oracle, 1e-12);
    }
    assert.ok(cell.material.energyOffset > 0);
});

test('large intrinsic offsets retain small positive mismatch without subtractive cancellation', () => {
    const base = 1e10, delta = .01;
    const t = topology({ wire: tool(2, { materialInterval: [0, 2], materialBreakpoints: [1],
        material: (s, { side }) => material(2, 4, [base + (s < 1 || s === 1 && side === 'left' ? -delta : delta), 0]) }) });
    const cell = build(t, [0, 2]).materialCells[0];
    const actualDelta = ((base + delta) - (base - delta)) / 2;
    close(cell.material.energyOffset, actualDelta ** 2, 1e-12);
    assert.ok(cell.material.energyOffset > 1e-5);
});

test('nominal DER supports keep constant-material weights and expose boundary half-cells separately', () => {
    const result = build(topology({ wire: tool(8, { material: material(2, 3) }),
        catheter: tool(6, { material: material(7, 11) }) }), [0, 2, 4, 6, 8]);
    for (const cell of result.materialCells) {
        assert.equal(cell.end - cell.start, cell.nominalLength);
        close(cell.material.stiffness[0], cell.id === 'wire' ? 2 : 7);
        close(cell.material.energyOffset, 0);
    }
    assert.deepEqual(result.boundaryCells.map(c => [c.id, c.side, c.start, c.end]), [
        ['wire', 'proximal', 0, 1], ['wire', 'distal', 7, 8],
        ['catheter', 'proximal', 0, 1], ['catheter', 'distal', 5, 6]
    ]);
    for (const id of ['wire', 'catheter']) {
        const total = [...result.materialCells, ...result.boundaryCells].filter(c => c.id === id)
            .reduce((sum, c) => sum + c.integratedMaterialLength, 0);
        close(total, id === 'wire' ? 8 : 6);
    }
});

test('feed rebuilds exact mesh boundaries and material maps without modifying an earlier mesh', () => {
    const wire = tool(10), catheter = tool(4.2, { materialBreakpoints: [98] });
    const before = build(topology({ wire, catheter }), [0, 2, 4, 6, 8, 10]);
    const savedCoordinates = before.data.coordinates.slice(), savedEnergy = assembled(before).energy;
    for (const insertion of [4.5, 6.1, 3.7]) {
        const after = build(topology({ wire, catheter: { ...catheter, insertion } }), [0, 2, 4, 6, 8, 10]);
        assert.ok(after.data.coordinates.includes(insertion)); assert.ok(after.data.coordinates.includes(insertion - 2));
        assert.equal(after.contactOwners.nodes.find(b => b.portal).x, insertion);
        assert.deepEqual(before.data.coordinates, savedCoordinates); close(assembled(before).energy, savedEnergy);
    }
    const toolData = before.data.tools[0], cell = before.materialCells.find(c => c.id === toolData.id);
    assert.throws(() => toolData.materialAt({ vertex: cell.vertex, coordinate: cell.coordinate + 1,
        start: cell.nominalStart, end: cell.nominalEnd }), /rebuild/);
});

test('admission refers to the current sections and cannot silently certify finite-clearance overlap', () => {
    const t = topology({ wire: tool(4), catheter: tool(3) });
    const initial = build(t, [0, 1, 2, 4]);
    assert.equal(initial.admission.candidate, true);
    const approved = build(t, [0, 1, 2, 4], { approvedSections: t.sections.filter(s => s.kind === 'overlap') });
    assert.equal(approved.admission.status, 'caller-approved'); assert.equal(approved.admission.candidate, false);
    assert.equal(approved.admission.certified, false); assert.equal(approved.admission.clearanceCertified, false);
    const newer = topology({ wire: tool(4), catheter: tool(3.1) });
    assert.throws(() => build(newer, [0, 1, 2, 4], { approvedSections: t.sections }), /current topology/);
});

test('compiled mesh Chain gradients include both spin fields and retain constant mismatch energy', () => {
    const t = topology({ wire: tool(4, { material: material(4, 3, [.02, -.03]) }),
        catheter: tool(3, { material: material(7, 5, [-.01, .02]) }) });
    const result = build(t, [0, 1, 2, 4], { sampleCenterline: x => [x, .02 * x * x, .01 * x * x],
        spinFields: { wire: (_, c) => .04 * c.x, catheter: (_, c) => -.07 * c.x } });
    const workspace = createCompositeChainWorkspace(result.layout);
    const gradient = assembleCompositeChain(result.data, workspace).gradient.slice();
    const changes = [];
    result.data.positions.forEach((p, i) => p.forEach((_, axis) => changes.push([
        result.layout.positions[i] + axis, delta => { p[axis] += delta; }
    ])));
    for (const tool of result.data.tools) for (let edge = 0; edge < tool.angles.length; edge++) {
        const index = result.layout.spins.get(tool.id)[edge];
        if (index >= 0) changes.push([index, delta => { tool.angles[edge] += delta; }]);
    }
    for (const [dof, change] of changes) {
        const h = 1e-6;
        change(h); const plus = assembleCompositeChain(result.data, workspace).energy;
        change(-2 * h); const minus = assembleCompositeChain(result.data, workspace).energy;
        change(h); close(gradient[dof], (plus - minus) / (2 * h), 3e-8);
    }
});

test('caller reference winding survives frame branch cuts and is independent for each tool', () => {
    const gamma = 5 * Math.PI + .01;
    const t = topology({ wire: tool(4), catheter: tool(4) });
    const result = build(t, [0, 2, 4], {
        referenceFrames: [{ tangent: [1, 0, 0], director: [0, 1, 0] },
            { tangent: [1, 0, 0], director: [0, Math.cos(gamma), Math.sin(gamma)] }],
        spinFields: { wire: (_, c) => c.edge ? -gamma : 0, catheter: (_, c) => c.edge ? -gamma : 0 },
        referenceTwistFields: { wire: gamma, catheter: () => gamma }
    });
    assert.ok(result.data.tools.every(tool => tool.referenceTwists[0] === gamma));
    assert.ok(assembled(result).energy < 1e-25);
});

function realTopology(catheterInsertion = 120) {
    return topology({ wire: compositeToolFromTipProfile({ profile: kirchhoffMaterialProfile('steel-j-035'),
        insertion: 130, materialInterval: [0, 1000], radius: GUIDEWIRE_RADIUS_MM, tipBreakpoints: [
            STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM - STEEL_J_GUIDEWIRE_CURVATURE_TRANSITION_MM,
            STEEL_J_GUIDEWIRE_CURVED_TIP_LENGTH_MM, STEEL_J_GUIDEWIRE_TIP_CORE_LENGTH_MM,
            STEEL_J_GUIDEWIRE_TIP_CORE_LENGTH_MM + STEEL_J_GUIDEWIRE_TIP_TRANSITION_LENGTH_MM
        ] }), catheter: compositeToolFromTipProfile({ profile: kirchhoffMaterialProfile('pigtail'), insertion: catheterInsertion,
        materialInterval: [20, 820], radius: PIGTAIL_CATHETER_RADIUS_MM, innerRadius: PIGTAIL_CATHETER_INNER_RADIUS_MM,
        tipBreakpoints: [PIGTAIL_NATURAL_ARC_LENGTH_MM - PIGTAIL_CURVATURE_TRANSITION_MM, PIGTAIL_NATURAL_ARC_LENGTH_MM] }) });
}
function simpson(fn, a, b) {
    const n = 4096, h = (b - a) / n;
    let sum = fn(a) + fn(b);
    for (let i = 1; i < n; i++) sum += (i % 2 ? 4 : 2) * fn(a + h * i);
    return sum * h / 3;
}
test('real profiles on a fed/retracted mesh match independently integrated preform energy', () => {
    for (const insertion of [120, 123.25, 117.5]) {
        const t = realTopology(insertion), result = build(t, Array.from({ length: 27 }, (_, i) => i * 5));
        const response = assembled(result);
        let oracle = 0;
        for (const id of ['wire', 'catheter']) {
            const cells = result.materialCells.filter(c => c.id === id), input = t.tools.find(tool => tool.id === id);
            const profile = kirchhoffMaterialProfile(id === 'wire' ? 'steel-j-035' : 'pigtail');
            const density = x => { const m = profile.sample(input.insertion - x);
                return .5 * (m.EI1 * m.kappa01 ** 2 + m.EI2 * m.kappa02 ** 2 + m.GJ * m.tau0 ** 2); };
            for (const cell of cells) for (const piece of cell.pieces) oracle += simpson(density, piece.start, piece.end);
        }
        close(response.energy, oracle, 5e-8);
        assert.ok(result.materialCells.some(c => c.material.energyOffset > 0));
        assert.ok(result.data.coordinates.includes(insertion));
        close(result.contactOwners.edges.find(e => e.lumen).lumen.clearance, .0405);
        assert.equal(result.quadrature.converged, true); assert.equal(result.quadrature.certified, false);
    }
});

test('invalid coverage, mesh fields and unconverged material quadrature fail visibly', () => {
    const t = topology({ wire: tool(2) });
    assert.throws(() => build(t, [0, 1, 1, 2]), /strictly increase/);
    assert.throws(() => build(t, [-1, 2]), /outside/);
    assert.throws(() => build(t, [0, 2], { spinFields: {} }), /spin field/);
    assert.throws(() => build(t, [0, 2], { sampleCenterline: () => [0, 0, 0] }), /nonzero/);
    const gap = topology({ interval: [0, 3], wire: tool(1, { materialInterval: [0, 1] }) });
    assert.throws(() => build(gap, [0, 1, 3]), /continuously covered/);
    const varying = topology({ wire: tool(2, { materialInterval: [0, 2], material: s => material(Math.exp(s)) }) });
    assert.throws(() => build(varying, [0, 1, 2], { quadrature: { absoluteTolerance: 1e-15, relativeTolerance: 0, maxDepth: 0 } }), /did not converge/);
});

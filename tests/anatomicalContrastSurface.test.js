import test from 'node:test';
import assert from 'node:assert/strict';
import {surfaceFixture} from './helpers/contrastSurfaceFixture.js';
import * as THREE from 'three';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';
import {AnatomicalContrastSurface, buildAnatomicalContrastSurface} from '../src/contrast/anatomicalContrastSurface.js';
import {FlowConcentrationField, sampleFlowConcentration} from '../src/contrast/flowConcentrationField.js';

test('shared junction wall follows the local bolus gradient instead of a constant node concentration', () => {
    const points = [new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 40, 0), new THREE.Vector3(0, 80, 0)];
    const segments = points.slice(1).map((end, id) => ({id, start: points[id], end,
        radiusStart: 8, radiusEnd: 8, nodeStartId: id, nodeEndId: id + 1}));
    // A comparable-calibre branch starts inside the common blood volume.
    // Unlike a tiny side outlet this splits the parent sampling territory.
    segments.push({id: 2, start: points[1], end: new THREE.Vector3(1, 42, 5),
        radiusStart: 6, radiusEnd: 6, nodeStartId: 1, nodeEndId: 3});
    const network = new ContrastFlowNetwork(segments, {rootPoint: points[0], cellLengthMm: 2});
    const geometry = new THREE.CylinderGeometry(8, 8, 80, 48, 80, true).translate(0, 40, 0);
    for (let i = 0; i < geometry.index.count; i += 3) {
        const b = geometry.index.getX(i + 1);
        geometry.index.setX(i + 1, geometry.index.getX(i + 2)); geometry.index.setX(i + 2, b);
    }
    const wall = geometry.toNonIndexed();
    let count = 0;
    const offsets = Uint32Array.from(network.edges, e => {const offset = count; count += e.cellCount; return offset;});
    const field = new FlowConcentrationField(network, offsets, count), cells = new Float32Array(count);
    const profile = y => Math.max(0, Math.min(1, .5 + (40 - y) / 20));
    for (const e of network.edges) for (let i = 0; i < e.cellCount; i++)
        cells[offsets[e.index] + i] = profile(e.start.y + (e.end.y - e.start.y) * (i + .5) / e.cellCount);
    field.update(cells);
    const built = buildAnatomicalContrastSurface(network, {fallbackGeometry: wall}, field);
    try {
        const {position: p, flowStencil: s} = built.geometry.attributes;
        const shared = new Map();
        for (let i = 0; i < p.count; i++) {
            const key = [p.getX(i), p.getY(i), p.getZ(i)].join(',');
            if (!shared.has(key)) shared.set(key, {indices: [], owners: new Set()});
            const entry = shared.get(key);
            entry.indices.push(i); entry.owners.add(built.triangleEdges[Math.floor(i / 3)]);
        }
        let checked = 0, axialSpan = 0;
        for (const {indices, owners} of shared.values()) {
            if (owners.size < 2) continue;
            const first = indices[0], y = p.getY(first);
            if (y < 32 || y > 48) continue;
            const values = indices.map(i => sampleFlowConcentration(field.values, s.getX(i), s.getY(i), s.getZ(i), s.getW(i)));
            assert.ok(Math.max(...values) - Math.min(...values) < 1e-7,
                `Connected wall point ${p.getX(first)},${y},${p.getZ(first)} must have one concentration`);
            assert.ok(Math.abs(values[0] - profile(y)) < .075,
                `Junction seam at y=${y} must retain its local gradient, not the node's value: ${values[0]}`);
            axialSpan = Math.max(axialSpan, Math.abs(y - 40)); checked++;
        }
        assert.ok(checked > 50, 'Exercise the curved seam between parent and branch sampling territories');
        assert.ok(axialSpan >= 6, 'The seam extends beyond the graph node cross-section');
        assert.equal(built.sourceTriangleCount, built.triangleEdges.length, 'Retain every inner anatomical triangle');

        // A refluxing daughter can make node blood much denser than the
        // incoming parent boundary layer. The common seam must retain exactly
        // the same continuous perfusion limit as its neighbouring wall points.
        for (const edge of network.edges) cells.fill(edge.sourceIndex === 0 ? .04 : .47,
            offsets[edge.index], offsets[edge.index] + edge.cellCount);
        network.getFaceSignedFlowMm3PerS = i => network.edges[i].sourceIndex === 0 ? 3000 :
            network.edges[i].sourceIndex === 2 ? -5000 : 8000;
        field.update(cells);
        const parent = network.edges.find(e => e.sourceIndex === 0);
        const parentCell = offsets[parent.index] + parent.cellCount - 1;
        let limited = 0;
        for (const {indices, owners} of shared.values()) {
            if (owners.size < 2) continue;
            for (const i of indices) {
                if (s.getX(i) !== parentCell && s.getY(i) !== parentCell) continue;
                const c = sampleFlowConcentration(field.values, s.getX(i), s.getY(i), s.getZ(i), s.getW(i));
                assert.ok(c <= .080001,
                    `The seam cannot bypass the dilute parent's reconstruction and expose a dense node (${c})`);
                limited++;
            }
        }
        assert.ok(limited > 50, 'Exercise shared parent/reflux wall samples with unequal incoming concentrations');
    } finally {built.geometry.dispose(); wall.dispose(); geometry.dispose();}
});

test('an intraluminal side-branch prefix cannot cut or darken a bolus in the parent wall', () => {
    const points = [new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 39, 0),
        new THREE.Vector3(0, 39.1, .45), new THREE.Vector3(0, 41, .2), new THREE.Vector3(0, 80, 0)];
    const segments = points.slice(1).map((end, id) => ({id, start: points[id], end,
        radiusStart: 8, radiusEnd: 8, nodeStartId: id, nodeEndId: id + 1}));
    // The small outlet's technical axis is inside the parent blood volume.
    // Euclidean nearest-axis mapping steals a wide patch of the aortic wall.
    segments.push({id: 4, start: points[2], end: new THREE.Vector3(1, 40, 5),
        radiusStart: .6, radiusEnd: .6, nodeStartId: 2, nodeEndId: 5});
    const network = new ContrastFlowNetwork(segments, {rootPoint: points[0], cellLengthMm: 2});
    const geometry = new THREE.CylinderGeometry(8, 8, 80, 48, 80, true).translate(0, 40, 0);
    for (let i = 0; i < geometry.index.count; i += 3) {
        const b = geometry.index.getX(i + 1);
        geometry.index.setX(i + 1, geometry.index.getX(i + 2)); geometry.index.setX(i + 2, b);
    }
    const wall = geometry.toNonIndexed();
    let count = 0;
    const offsets = Uint32Array.from(network.edges, e => {const i = count; count += e.cellCount; return i;});
    const profile = y => Math.max(0, Math.min(1, .5 + (40 - y) / 20));
    const surface = new AnatomicalContrastSurface(network, offsets, {fallbackGeometry: wall});
    try {
        assert.equal(surface.diagnostics.lumenTriangleCount, wall.attributes.position.count / 3);
        for (const branchConcentration of [0, 1]) {
            for (const e of network.edges) for (let i = 0; i < e.cellCount; i++) {
                const y = e.start.y + (e.end.y - e.start.y) * (i + .5) / e.cellCount;
                e.massMg[i] = e.volumes[i] * .3 * (e.sourceIndex === 4 ? branchConcentration : profile(y));
            }
            surface.update(new Float32Array(count), .3, false);
            const {position, flowStencil: s, flowAxis: axis} = surface.samplingGeometry.attributes;
            let checked = 0;
            for (let i = 0; i < position.count; i++) {
                const y = position.getY(i);
                if (y < 32 || y > 48) continue;
                const c = sampleFlowConcentration(surface.current, s.getX(i), s.getY(i), s.getZ(i), s.getW(i));
                assert.ok(Math.abs(c - profile(y)) < .075,
                    `Parent wall must retain the local bolus gradient at y=${y}: ${c}`);
                assert.ok(Math.abs(axis.getZ(i)) < .15,
                    'A radial atlas connector must not become a beam-parallel optical axis');
                checked++;
            }
            assert.ok(checked > 1000);
        }
    } finally {surface.dispose(); wall.dispose(); geometry.dispose();}
});

for (const fork of [false, true]) test(`a bolus gradient stays continuous across anatomical triangles (fork=${fork})`, () => {
    const path = new THREE.CatmullRomCurve3([
        new THREE.Vector3(0, 0, 0), new THREE.Vector3(5, 25, 0),
        new THREE.Vector3(-4, 55, 2), new THREE.Vector3(0, 80, 0)
    ]);
    const geometry = new THREE.TubeGeometry(path, 37, 4, 24, false);
    // Blood-facing surface, with wall rings deliberately independent of the
    // finite-volume grid. STL has duplicated vertices at triangle boundaries.
    for (let i = 0; i < geometry.index.count; i += 3) {
        const b = geometry.index.getX(i + 1);
        geometry.index.setX(i + 1, geometry.index.getX(i + 2));
        geometry.index.setX(i + 2, b);
    }
    const wall = geometry.toNonIndexed();
    const points = path.getSpacedPoints(31);
    const segments = points.slice(1).map((end, i) => ({
        id: i, start: points[i], end, radiusStart: 4, radiusEnd: 4,
        nodeStartId: i, nodeEndId: i + 1
    }));
    if (fork) segments.push({id: 31, start: points[15], end: new THREE.Vector3(100, 40, 30),
        radiusStart: 1, radiusEnd: 1, nodeStartId: 15, nodeEndId: 32});
    const network = new ContrastFlowNetwork(segments, {rootPoint: points[0], cellLengthMm: 2});
    if (fork) network.edges.find(e => e.sourceIndex === 31).lumenMappingExcluded = true;
    const offsets = new Uint32Array(network.edges.length);
    let count = 0;
    for (const e of network.edges) {
        offsets[e.index] = count; count += e.cellCount;
        for (let c = 0; c < e.cellCount; c++) {
            const y = e.start.y + (c + .5) / e.cellCount * (e.end.y - e.start.y);
            e.massMg[c] = e.volumes[c] * .3 / (1 + Math.exp((y - 42) / 3));
        }
    }
    const surface = new AnatomicalContrastSurface(network, offsets, {fallbackGeometry: wall});
    try {
        surface.update(new Float32Array(count), .3, false);
        const {position, flowStencil} = surface.samplingGeometry.attributes;
        const samples = new Map();
        let shared = 0, maxJump = 0;
        for (let i = 0; i < position.count; i++) {
            const key = [position.getX(i), position.getY(i), position.getZ(i)].join(',');
            const concentration = sampleFlowConcentration(surface.current, flowStencil.getX(i), flowStencil.getY(i), flowStencil.getZ(i), flowStencil.getW(i));
            if (samples.has(key)) {
                shared++;
                maxJump = Math.max(maxJump, Math.abs(samples.get(key) - concentration));
            } else samples.set(key, concentration);
        }
        assert.ok(shared > 1000, 'exercise shared STL vertices');
        assert.ok(maxJump < 1e-7, `the same wall point cannot have two concentrations (${maxJump})`);
    } finally {surface.dispose(); wall.dispose(); geometry.dispose();}
});

for (const radius of [.2, 1, 8]) test(`STL lumen ${radius} mm: exact inner triangles, no outer wall or expansion`, () => {
    const f = surfaceFixture(radius), {volume: v, network: n} = f;
    try {
        assert.equal(v.diagnostics.lumenTriangleCount, v.diagnostics.sourceTriangleCount / 2);
        const p = v.mesh.geometry.attributes.position;
        for (let i = 0; i < p.count; i++)
            assert.ok(Math.abs(Math.hypot(p.getX(i), p.getZ(i)) - radius) < 1e-6);
        const mass = n.edges.map(e => Array.from(e.massMg));
        for (const e of n.edges) for (let i = 0; i < e.cellCount; i++) e.massMg[i] = e.volumes[i] * .03;
        v.update(f.plume, .3, false);
        assert.ok(v.current.every(c => Math.abs(c - .1) < 1e-6));
        assert.ok(v.mesh.visible);
        const before = n.edges.map(e => Array.from(e.massMg));
        v.update(f.plume, .3, true);
        assert.deepEqual(n.edges.map(e => Array.from(e.massMg)), before, 'renderer cannot change iodine mass');
        assert.deepEqual(v.previous, v.current);
        n.edges.forEach(e => e.massMg.fill(0));
        v.update(f.plume, .3, true); v.update(f.plume, .3, true);
        assert.equal(v.mesh.visible, false, 'no persistent stain after washout');
        assert.deepEqual(n.edges.map(e => Array.from(e.massMg)), mass);
        v.resetHistory(); assert.ok(v.previous.every(c => c === 0));
    } finally {v.dispose(); f.geometry.dispose();}
});

test('surface respects excluded graft cells and leaves shared source geometry owned by anatomy', () => {
    const f = surfaceFixture(1), e = f.network.edges[0];
    e.massMg.fill(1); e.graftSections = Array(e.cellCount).fill({});
    f.volume.update(f.plume, .3, false, true);
    assert.ok(f.volume.current.every(c => c === 0));
    assert.equal(f.volume.mesh.visible, false);
    let disposed = false; f.geometry.addEventListener('dispose', () => disposed = true);
    f.volume.dispose(); assert.equal(disposed, false); f.geometry.dispose();
});

test('volume raster uploads only projection inputs while retaining CPU reconstruction and draw ranges', () => {
    const f = surfaceFixture(1), v = f.volume;
    const cpu = v.samplingGeometry, raster = v.mesh.geometry;
    let sourceDisposals = 0, cpuDisposals = 0, rasterDisposals = 0;
    f.geometry.addEventListener('dispose', () => sourceDisposals++);
    cpu.addEventListener('dispose', () => cpuDisposals++);
    raster.addEventListener('dispose', () => rasterDisposals++);
    try {
        assert.notEqual(raster, cpu);
        assert.ok(Object.keys(raster.attributes).every(name => ['position', 'projectionOwner'].includes(name)),
            'Unused CPU stencils, axes, radii and normals must not become GPU buffers');
        assert.equal(raster.getAttribute('position'), cpu.getAttribute('position'), 'No duplicate position allocation');
        if (cpu.hasAttribute('projectionOwner')) assert.equal(raster.getAttribute('projectionOwner'), cpu.getAttribute('projectionOwner'));
        for (const name of ['flowStencil', 'flowAxis', 'opticalRadius', 'normal'])
            assert.ok(cpu.hasAttribute(name), `Keep ${name} available for CPU diagnostics`);
        for (const edge of f.network.edges) for (let i = 0; i < edge.cellCount; i++) edge.massMg[i] = edge.volumes[i] * .3;
        v.update(f.plume, .3, false);
        assert.ok(raster.groups.length > 0);
        assert.deepEqual(raster.groups, cpu.groups, 'Activity ranges must reach the actual raster geometry');
        assert.equal(raster.groups.reduce((sum, group) => sum + group.count, 0), raster.getAttribute('position').count,
            'Every anatomical entry, including a clear cap, remains eligible for projection');
        for (const edge of f.network.edges) edge.massMg.fill(0);
        v.update(f.plume, .3, false);
        assert.equal(v.mesh.visible, false);
        assert.equal(raster.groups.length, 0, 'Inactive projection must clear previous raster draws');
    } finally {v.dispose();}
    assert.equal(cpuDisposals, 1); assert.equal(rasterDisposals, 1); assert.equal(sourceDisposals, 0);
    f.geometry.dispose();
});

test('legacy scalar preview keeps its shader attributes on its raster geometry', () => {
    const f = surfaceFixture(1);
    const legacy = new AnatomicalContrastSurface(f.network, new Uint32Array([0]),
        {fallbackGeometry: f.geometry}, {projection: 'surface'});
    try {
        assert.equal(legacy.samplingGeometry, legacy.mesh.geometry);
        for (const name of ['flowStencil', 'flowAxis', 'opticalRadius', 'normal'])
            assert.ok(legacy.mesh.geometry.hasAttribute(name));
    } finally {legacy.dispose(); f.volume.dispose(); f.geometry.dispose();}
});

test('production renderer uses STL for small branches without a second proxy overlay, in both view modes', async () => {
    const {HybridContrastSystem} = await import('../src/contrast/hybridContrastSystem.js');
    const {ContrastVolumeRenderer} = await import('../src/contrast/contrastVolumeRenderer.js');
    const {AnatomicalContrastSurface} = await import('../src/contrast/anatomicalContrastSurface.js');
    const f = surfaceFixture(.2);
    const system = new HybridContrastSystem({centerlineSegments: f.network.sourceSegments,
        contactField: {fallbackGeometry: f.geometry}, localOptions: {capacity: 8}});
    const renderer = new ContrastVolumeRenderer(system);
    try {
        assert.ok(renderer._flowVolume instanceof AnatomicalContrastSurface);
        const e = system.flowNetwork.edges[0];
        e.renalSide = 'left'; e.active = true; e.massMg.fill(.01);
        for (const debug of [true, false]) {
            renderer.setDebugMode(debug); renderer.update();
            assert.equal(renderer._flowVolume.uniforms.debugMode.value, debug);
            assert.equal(renderer._flowVolume.mesh.visible, true);
            assert.equal(renderer.flowMesh.visible, false, 'renal tubes cannot expand or double the native lumen');
        }
    } finally {renderer.dispose(); f.volume.dispose(); f.geometry.dispose();}
});

test('lumen mapping selects the nearest eligible axis independently of spatial bucket size', async () => {
    const {createLumenSurfaceMapping} = await import('../src/contrast/lumenSurfaceMapping.js');
    const THREE = await import('three');
    const edges = Array.from({length: 41}, (_, index) => {
        const start = new THREE.Vector3(Math.sin(index)*12, index*.7, Math.cos(index)*8);
        const end = start.clone().add(new THREE.Vector3(Math.cos(index*.4)*3, 2, Math.sin(index*.3)*2));
        const axis = end.clone().sub(start); const length = axis.length(); axis.normalize();
        return {index, start, end, axis, length, radiusStart: .2, radiusEnd: .4,
            transportExcluded: index % 11 === 0, lumenMappingExcluded: index % 13 === 0};
    });
    const locate = createLumenSurfaceMapping({edges});
    for (let i = 0; i < 100; i++) {
        const p = new THREE.Vector3(Math.sin(i*.73)*15, i*.31, Math.cos(i*.93)*10);
        const reference = edges.filter(e => !e.transportExcluded && !e.lumenMappingExcluded).map(e => {
            const line = new THREE.Line3(e.start, e.end), point = line.closestPointToPoint(p, true, new THREE.Vector3());
            return {index:e.index, distance:p.distanceTo(point)};
        }).sort((a,b)=>a.distance-b.distance||a.index-b.index)[0];
        const actual = locate(p.x,p.y,p.z,{});
        assert.equal(actual.edgeIndex, reference.index);
        assert.ok(Math.abs(actual.distance-reference.distance)<1e-10);
    }
});

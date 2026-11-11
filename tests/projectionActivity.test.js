import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {createProjectionActivity} from '../src/contrast/projectionActivity.js';

function fixture() {
    const vertices = 4096 * 3, positions = new Float32Array(vertices * 2 * 3);
    const stencils = new Float32Array(vertices * 2 * 4), owners = new Uint32Array(vertices * 2 / 3);
    for (let vessel = 0; vessel < 2; vessel++) for (let vertex = 0; vertex < vertices; vertex++) {
        const i = vessel * vertices + vertex, corner = vertex % 6;
        positions.set([vessel * 100 + (corner & 1 ? 10 : -10), corner & 2 ? 20 : -20,
            corner & 4 ? 10 : -10], i * 3);
        stencils.set([vessel * 3, vessel * 3, 0, -1], i * 4);
        owners[Math.floor(i / 3)] = vessel;
    }
    const geometry = new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(positions, 3))
        .setAttribute('flowStencil', new THREE.BufferAttribute(stencils, 4));
    const network = {edges: [0, 1].map(index => ({index, start: new THREE.Vector3(index * 100, -20, 0),
        end: new THREE.Vector3(index * 100, 20, 0), radiusStart: .1, radiusEnd: .1, cellCount: 3,
        parentEdgeIndex: -1, childEdgeIndices: []}))};
    const field = {cellCount: 6, offsets: [0, 3], start: [6, 8], end: [7, 9],
        nodes: [0, 1, 2, 3].map(i => ({parentEdgeIndex: i % 2 ? Math.floor(i / 2) : -1,
            childEdgeIndices: i % 2 ? [] : [Math.floor(i / 2)]}))};
    return {network, field, geometry, owners, current: new Float32Array(10), previous: new Float32Array(10)};
}

test('activity bounds retain a hidden middle bolus and use actual aneurysm wall extent', () => {
    const f = fixture();
    const activity = createProjectionActivity(f.network, f.geometry, f.field, f.owners);
    f.current[1] = 1; // No inlet/outlet node or wall-vertex sample contains iodine.
    assert.equal(activity.update(f.current, f.previous), true);
    assert.equal(activity.diagnostics.activeEdges, 1);
    assert.equal(activity.diagnostics.fallbackEdges, 0);
    for (const point of [new THREE.Vector3(-10, -20, -10), new THREE.Vector3(10, 20, 10)])
        assert.ok(activity.bounds.containsPoint(point), 'Nominal .1 mm radius must not clip the actual 10 mm wall');
    assert.ok(activity.bounds.max.x < 20, 'An unrelated clear vessel does not enlarge support');
    f.current.fill(0); f.previous[1] = 1;
    assert.equal(activity.update(f.current, f.previous), true, 'Display interpolation retains preceding blood');
    f.previous.fill(0); assert.equal(activity.update(f.current, f.previous), false);
    f.geometry.dispose();
});

test('camera culling preserves clear entry chunks and restores draw range after each draw', () => {
    const f = fixture(), activity = createProjectionActivity(f.network, f.geometry, f.field, f.owners);
    f.current[1] = 1; activity.update(f.current, f.previous); activity.setGroups();
    const material = new THREE.ShaderMaterial({uniforms: {}}), mesh = new THREE.Mesh(f.geometry, [material]);
    activity.install(mesh); mesh.updateMatrixWorld(true);
    for (const position of [new THREE.Vector3(0, 0, 100), new THREE.Vector3(0, 100, 0)]) {
        const camera = new THREE.OrthographicCamera(-150, 150, 150, -150, .1, 1000);
        camera.position.copy(position); camera.up.set(0, 0, 1); camera.lookAt(0, 0, 0); camera.updateMatrixWorld(true);
        for (const [i, group] of f.geometry.groups.entries()) {
            mesh.onBeforeRender(null, null, camera, f.geometry, material, group);
            assert.equal(f.geometry.drawRange.count, i === 0 ? group.count : 0);
            mesh.onAfterRender(null, null, camera, f.geometry, material, group);
            assert.equal(f.geometry.drawRange.count, Infinity);
        }
    }
    material.dispose(); f.geometry.dispose();
});

test('an unmixed outlet jet keeps its exact entry walls active before bulk iodine arrives', () => {
    const f = fixture(), activity = createProjectionActivity(f.network, f.geometry, f.field, f.owners);
    const jetBounds = new THREE.Box3(new THREE.Vector3(-1, -2, -1), new THREE.Vector3(1, 2, 1));
    assert.equal(activity.update(f.current, f.previous, jetBounds), true);
    assert.ok(activity.bounds.containsBox(jetBounds));
    assert.equal(activity.diagnostics.activeEdges, 0, 'the bulk remains clear');
    assert.ok(activity.bounds.max.x < 20, 'unrelated vessels do not enlarge jet support');
    assert.equal(activity.update(f.current, f.previous, new THREE.Box3()), false);
    f.geometry.dispose();
});

test('an active edge without anatomical support falls back to the complete mesh', () => {
    const f = fixture();
    const missing = {index: 2, start: new THREE.Vector3(0, 0, 0), end: new THREE.Vector3(0, 1, 0),
        cellCount: 1, parentEdgeIndex: -1, childEdgeIndices: []};
    f.network.edges.push(missing); f.field.offsets.push(10); f.field.start.push(6); f.field.end.push(7);
    f.field.cellCount = 11; f.field.nodes = [];
    const activity = createProjectionActivity(f.network, f.geometry, f.field, f.owners);
    const current = new Float32Array(11); current[10] = 1;
    assert.equal(activity.update(current, new Float32Array(11)), true);
    assert.equal(activity.diagnostics.fallbackEdges, 1);
    assert.ok(activity.bounds.containsPoint(new THREE.Vector3(110, 20, 10)));
    f.geometry.dispose();
});

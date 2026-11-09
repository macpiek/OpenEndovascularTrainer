import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {finiteCylinderIodineIntegral} from './helpers/contrastAxialProjectionFixture.js';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';
import {FlowConcentrationField} from '../src/contrast/flowConcentrationField.js';
import {buildAnatomicalContrastProjection} from '../src/contrast/anatomicalContrastProjection.js';

const radius = 3, length = 24;
const near = (actual, expected) => assert.ok(Math.abs(actual - expected) < 1e-9,
    `Expected iodine path integral ${expected}, got ${actual}`);
const integral = (origin, direction, concentrations = new Float64Array(32).fill(1)) =>
    finiteCylinderIodineIntegral(new THREE.Vector3(...origin),
        new THREE.Vector3(...direction).normalize(), radius, length, concentrations);

test('independent projection oracle measures the full finite axial column in either direction', () => {
    near(integral([0, 40, 0], [0, -1, 0]), length);
    near(integral([0, -40, 0], [0, 1, 0]), length);
    near(integral([radius - .01, 40, 0], [0, -1, 0]), length);
    near(integral([0, 0, 0], [0, 1, 0]), length / 2);
    near(integral([0, 40, 0], [0, 1, 0]), 0);
});

test('independent projection oracle uses radial chords laterally and finite caps obliquely', () => {
    near(integral([0, .1, 40], [0, 0, -1]), 2 * radius);
    near(integral([1.5, .1, 40], [0, 0, -1]), 2 * Math.sqrt(radius ** 2 - 1.5 ** 2));
    for (const angle of [.5, 5, 30, 80]) {
        const a = angle * Math.PI / 180, axis = [0, Math.cos(a), Math.sin(a)];
        const expected = Math.min(length / Math.cos(a), 2 * radius / Math.sin(a));
        near(integral(axis.map(v => v * 40), axis.map(v => -v)), expected);
    }
});

test('independent projection oracle integrates a bolus with clear inlet and outlet cells', () => {
    const bolus = new Float64Array(32);
    bolus.fill(.4, 12, 20); // Eight 0.75 mm cells: a 6 mm internal column.
    near(integral([0, 40, 0], [0, -1, 0], bolus), 2.4);
    near(integral([0, -40, 0], [0, 1, 0], bolus), 2.4);
    near(integral([0, .1, 40], [0, 0, -1], bolus), 2.4);
    near(integral([0, 6, 40], [0, 0, -1], bolus), 0);
    near(integral([0, 40, 0], [0, -1, 0], new Float64Array(32)), 0);
});

test('independent projection oracle rejects rays beyond the lumen and zero-length tangencies', () => {
    near(integral([radius + .01, 40, 0], [0, -1, 0]), 0);
    near(integral([radius, .1, 40], [0, 0, -1]), 0);
    near(integral([0, length / 2 + .01, 40], [0, 0, -1]), 0);
    near(integral([0, .1, 40], [0, 0, 1]), 0);
});

test('linear concentration oracle preserves bolus mass and resolves partial-volume fronts', () => {
    const bolus = new Float64Array(32);
    bolus.fill(1, 12, 20);
    const linear = (origin, direction) => finiteCylinderIodineIntegral(new THREE.Vector3(...origin),
        new THREE.Vector3(...direction).normalize(), radius, length, bolus, 'linear');
    near(linear([0, 40, 0], [0, -1, 0]), 6);
    near(linear([0, 0, 40], [0, 0, -1]), 6);
    near(linear([0, 3, 40], [0, 0, -1]), 3);
    near(linear([0, 3.1875, 40], [0, 0, -1]), 1.5);
    near(linear([0, 3.375, 40], [0, 0, -1]), 0);
});

test('independent projection oracle is unchanged by subdivision of the concentration cells', () => {
    const coarse = Float64Array.from([.1, 0, .4, 1, .2, 0]);
    const refined = Float64Array.from(Array.from(coarse).flatMap(c => [c, c, c, c]));
    for (const [origin, direction] of [
        [[0, 40, 0], [0, -1, 0]], [[0, -40, 0], [0, 1, 0]],
        [[0, 24, 8], [0, -3, -1]], [[1.5, .1, 40], [0, 0, -1]]
    ]) near(integral(origin, direction, coarse), integral(origin, direction, refined));
});

function projectionFixture() {
    const point = i => new THREE.Vector3(123.456789 + i * .31, -1320.865234375 + i * 2.7, 13.7 - i * .29);
    const segments = Array.from({length: 9}, (_, id) => ({id, start: point(id), end: point(id + 1),
        nodeStartId: id, nodeEndId: id + 1, radiusStart: 1 + id * .1, radiusEnd: 1.1 + id * .1}));
    segments.push({id: 9, start: point(4), end: point(4).add(new THREE.Vector3(8, 0, 3)),
        nodeStartId: 4, nodeEndId: 10, radiusStart: .8, radiusEnd: .7});
    const network = new ContrastFlowNetwork(segments, {rootPoint: point(0), cellLengthMm: .75});
    let count = 0;
    const offsets = Uint32Array.from(network.edges, e => {const offset = count; count += e.cellCount; return offset;});
    const field = new FlowConcentrationField(network, offsets, count);
    const geometry = new THREE.BoxGeometry(20, 40, 20, 2, 4, 2).translate(125, -1305, 12);
    return {network, field, geometry};
}

test('projection lookup preserves graph samples and leaves explicitly empty vessels available for ownership', () => {
    const {network, field, geometry} = projectionFixture();
    network.edges.find(e => e.sourceIndex === 2).transportExcluded = true;
    network.edges.find(e => e.sourceIndex === 6).lumenMappingExcluded = true;
    const empty = network.edges.find(e => e.sourceIndex === 7);
    empty.renderExcluded = true;
    const projection = buildAnatomicalContrastProjection(network, geometry, field);
    try {
        const records = projection.uniforms.projectionEdges.value.image.data;
        for (const e of network.edges.filter(e => !e.transportExcluded && !e.lumenMappingExcluded)) {
            const i = e.index * 24;
            assert.deepEqual(Array.from(records.slice(i, i + 3)), e.start.toArray().map(Math.fround));
            assert.deepEqual(Array.from(records.slice(i + 16, i + 19)), e.end.toArray().map(Math.fround));
            assert.deepEqual(Array.from(records.slice(i + 9, i + 13)),
                [field.offsets[e.index], e.cellCount, field.start[e.index], field.end[e.index]]);
            assert.equal(records[i + 13], network.nodes.get(e.startNodeId).childEdgeIndices.length > 1 ? 1 : 0);
            assert.equal(records[i + 14], network.nodes.get(e.endNodeId).childEdgeIndices.length > 1 ? 1 : 0);
        }
        assert.equal(records[empty.index * 24 + 10], empty.cellCount,
            'An empty anatomical vessel must retain its own axis rather than borrowing a filled neighbour');
    } finally {projection.dispose(); geometry.dispose();}
});

test('projection search bounds contain every eligible edge at translated anatomy coordinates', () => {
    const {network, field, geometry} = projectionFixture();
    network.edges[2].transportExcluded = true;
    network.edges[5].lumenMappingExcluded = true;
    const projection = buildAnatomicalContrastProjection(network, geometry, field);
    try {
        const bounds = projection.uniforms.projectionNodes.value.image.data;
        const order = projection.uniforms.projectionOrder.value.image.data;
        const visit = node => {
            const i = node * 12, offset = bounds[i + 9], count = bounds[i + 10];
            const edges = offset >= 0 ? Array.from({length: count}, (_, j) => order[(offset + j) * 4]) :
                [...visit(bounds[i + 7]), ...visit(bounds[i + 8])];
            for (const index of edges) {
                const edge = network.edges[index];
                for (const point of [edge.start, edge.end]) for (let axis = 0; axis < 3; axis++) {
                    assert.ok(bounds[i + axis] <= point.getComponent(axis));
                    assert.ok(bounds[i + 4 + axis] >= point.getComponent(axis));
                }
                assert.ok(bounds[i + 3] >= Math.max(edge.radiusStart, edge.radiusEnd));
            }
            return edges;
        };
        assert.deepEqual(visit(0).sort((a, b) => a - b), network.edges
            .filter(e => !e.transportExcluded && !e.lumenMappingExcluded).map(e => e.index));
        assert.ok(projection.uniforms.projectionSampleSpacing.value > 0);
        assert.ok(projection.uniforms.projectionSampleSpacing.value <=
            Math.min(...network.edges.filter(e => !e.transportExcluded && !e.lumenMappingExcluded)
                .map(e => e.cellLength)) * .5);
    } finally {projection.dispose(); geometry.dispose();}
});

test('projection owns its GPU lookup resources without mutating or disposing the source mesh', () => {
    const {network, field, geometry} = projectionFixture();
    const positions = geometry.attributes.position.array.slice(), indices = geometry.index.array.slice();
    const groups = structuredClone(geometry.groups);
    let sourceDisposed = false;
    geometry.addEventListener('dispose', () => {sourceDisposed = true;});
    const projection = buildAnatomicalContrastProjection(network, geometry, field);
    const bvh = projection.uniforms.lumenBvh.value;
    const textures = [bvh.index, bvh.position, bvh.bvhBounds, bvh.bvhContents,
        ...Object.values(projection.uniforms).map(u => u.value).filter(value => value?.isTexture)];
    const disposed = new Set();
    for (const texture of textures) texture.addEventListener('dispose', () => disposed.add(texture));
    assert.equal(projection.memoryBytes, textures.reduce((sum, t) => sum + t.image.data.byteLength, 0));
    projection.dispose();
    assert.equal(disposed.size, textures.length);
    assert.equal(sourceDisposed, false);
    assert.deepEqual(geometry.attributes.position.array, positions);
    assert.deepEqual(geometry.index.array, indices);
    assert.deepEqual(geometry.groups, groups);
    geometry.dispose();
});

test('projection lookup remains finite when all transport axes are excluded', () => {
    const {network, field, geometry} = projectionFixture();
    for (const edge of network.edges) edge.transportExcluded = true;
    const projection = buildAnatomicalContrastProjection(network, geometry, field);
    try {
        assert.ok(projection.uniforms.projectionNodes.value.image.data.every(Number.isFinite));
        assert.equal(projection.uniforms.projectionNodes.value.image.data[10], 0);
        assert.ok(Number.isFinite(projection.uniforms.projectionSampleSpacing.value));
    } finally {projection.dispose(); geometry.dispose();}
});

test('empty-space ownership search stays local instead of visiting the whole arterial atlas', () => {
    const network = new ContrastFlowNetwork(Array.from({length: 1024}, (_, i) => ({id: i,
        nodeStartId: i * 2, nodeEndId: i * 2 + 1,
        start: new THREE.Vector3(i * 20, 0, 0), end: new THREE.Vector3(i * 20, 2, 0),
        radiusStart: .2, radiusEnd: .2})));
    let cells = 0;
    const offsets = Uint32Array.from(network.edges, edge => {const start = cells; cells += edge.cellCount; return start;});
    const field = new FlowConcentrationField(network, offsets, cells);
    const geometry = new THREE.BoxGeometry(1, 1, 1);
    const projection = buildAnatomicalContrastProjection(network, geometry, field);
    try {
        const nodes = projection.uniforms.projectionNodes.value.image.data;
        const point = [10250, 1, 0], stack = [0];
        let visited = 0, candidates = 0;
        while (stack.length) {
            const i = stack.pop() * 12; visited++;
            const d2 = point.reduce((sum, p, k) => sum + Math.max(nodes[i + k] - p, 0, p - nodes[i + 4 + k]) ** 2, 0);
            if (d2 > nodes[i + 11] ** 2) continue;
            if (nodes[i + 9] >= 0) candidates += nodes[i + 10];
            else stack.push(nodes[i + 7], nodes[i + 8]);
        }
        assert.ok(visited < 24, `Unassigned blood must not scan ${visited} atlas nodes`);
        assert.ok(candidates <= 8, `Unassigned blood must not test visibility to ${candidates} remote axes`);
    } finally {projection.dispose(); geometry.dispose();}
});

test('ownership reach includes the actual dilated wall rather than its nominal graph radius', () => {
    const network = new ContrastFlowNetwork([{id: 0, nodeStartId: 0, nodeEndId: 1,
        start: new THREE.Vector3(0, 10, 0), end: new THREE.Vector3(0, -10, 0),
        radiusStart: 1, radiusEnd: 1}]);
    const field = new FlowConcentrationField(network, new Uint32Array([0]), network.edges[0].cellCount);
    const geometry = new THREE.CylinderGeometry(18, 18, 20, 24, 4).toNonIndexed();
    geometry.setAttribute('projectionOwner', new THREE.BufferAttribute(new Float32Array(geometry.attributes.position.count), 1));
    const projection = buildAnatomicalContrastProjection(network, geometry, field);
    try {
        const edgeReach = projection.uniforms.projectionEdges.value.image.data[21];
        const rootReach = projection.uniforms.projectionNodes.value.image.data[11];
        assert.ok(edgeReach >= 19 - 1e-5, `Dilated lumen needs at least19mm reach, got${edgeReach}`);
        assert.ok(rootReach >= edgeReach);
    } finally {projection.dispose(); geometry.dispose();}
});

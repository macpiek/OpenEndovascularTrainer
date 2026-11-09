import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {FlowConcentrationField} from '../src/contrast/flowConcentrationField.js';
import {buildAnatomicalColumnProjection} from '../src/contrast/anatomicalColumnProjection.js';
import {VesselDeformation} from '../src/vesselDeformation.js';
import {axialProjectionFixture} from './helpers/contrastAxialProjectionFixture.js';

function flowFixture(count = 2) {
    const nodes = new Map(), edges = [];
    for (let index = 0; index < count; index++) {
        const start = new THREE.Vector3(index * 20, 12, 0), end = new THREE.Vector3(index * 20, -12, 0);
        const edge = {index, start, end, length: 24, axis: end.clone().sub(start).normalize(),
            radiusStart: 3, radiusEnd: 3, cellCount: 32, cellLength: .75,
            parentEdgeIndex: -1, childEdgeIndices: [], startNodeId: index * 2, endNodeId: index * 2 + 1,
            areas: new Float64Array(32).fill(Math.PI * 9)};
        edges.push(edge);
        nodes.set(edge.startNodeId, {id: edge.startNodeId, point: start, parentEdgeIndex: -1, childEdgeIndices: [index]});
        nodes.set(edge.endNodeId, {id: edge.endNodeId, point: end, parentEdgeIndex: index, childEdgeIndices: []});
    }
    const network = {edges, nodes}, offsets = Uint32Array.from(edges, edge => edge.index * 32);
    const field = new FlowConcentrationField(network, offsets, count * 32);
    return {network, field, territories: Int32Array.from(edges, edge => edge.index),
        continuations: new Int32Array(count).fill(-1)};
}

function codedGeometry() {
    const source = new THREE.CylinderGeometry(3, 3, 24, 40, 20, false).toNonIndexed();
    const positions = source.attributes.position.array, triangles = positions.length / 9;
    const shuffled = new Float32Array(positions.length), columns = new Float32Array(triangles * 6);
    // Interleave the distal and proximal parts so a BVH must reorder faces.
    for (let i = 0; i < triangles; i++) {
        const sourceTriangle = i % 2 ? triangles - 1 - (i >> 1) : i >> 1;
        shuffled.set(positions.subarray(sourceTriangle * 9, sourceTriangle * 9 + 9), i * 9);
        for (let k = 0; k < 3; k++) columns.set([i % 2, i * 4 + k + .25], i * 6 + k * 2);
    }
    source.setAttribute('position', new THREE.BufferAttribute(shuffled, 3));
    source.setAttribute('columnCoordinate', new THREE.BufferAttribute(columns, 2));
    return source;
}

test('GPU face coordinates follow indirect BVH triangle ordering without losing seam ownership', () => {
    const f = flowFixture(), geometry = codedGeometry();
    const positions = geometry.attributes.position.array.slice(), columns = geometry.attributes.columnCoordinate.array.slice();
    const projection = buildAnatomicalColumnProjection(f.network, geometry, f.field, f);
    try {
        const originalTriangles = new Map();
        for (let i = 0; i < positions.length; i += 9) originalTriangles.set(Array.from(positions.subarray(i, i + 9)).join(','), i / 9);
        const bvh = projection.uniforms.lumenBvh.value, indices = bvh.index.image.data, vertices = bvh.position.image.data;
        const indexStride = indices.length / (bvh.index.image.width * bvh.index.image.height);
        const positionStride = vertices.length / (bvh.position.image.width * bvh.position.image.height);
        const faces = projection.uniforms.columnFaces.value.image.data;
        let reordered = 0;
        for (let i = 0; i < positions.length / 9; i++) {
            const triangle = [];
            for (let k = 0; k < 3; k++) {
                const vertex = indices[i * indexStride + k];
                triangle.push(...vertices.subarray(vertex * positionStride, vertex * positionStride + 3));
            }
            const original = originalTriangles.get(triangle.join(','));
            assert.notEqual(original, undefined, 'Every GPU triangle must be an exact original triangle');
            if (original !== i) reordered++;
            assert.deepEqual(Array.from(faces.subarray(i * 4, i * 4 + 4)),
                [columns[original * 6 + 1], columns[original * 6 + 3], columns[original * 6 + 5], columns[original * 6]],
                'Exit barycentric coordinates must use this face, not the original index at its BVH slot');
        }
        assert.ok(reordered > positions.length / 18, 'The fixture must exercise a substantial indirect permutation');
        assert.deepEqual(geometry.attributes.position.array, positions);
        assert.deepEqual(geometry.attributes.columnCoordinate.array, columns);
    } finally {projection.dispose(); geometry.dispose();}
});

test('column projection keeps disconnected roots and their iodine prefix histories separate', () => {
    const f = flowFixture(), geometry = codedGeometry();
    const projection = buildAnatomicalColumnProjection(f.network, geometry, f.field, f);
    try {
        const values = new Float32Array(f.field.cellCount); values.fill(.6, 0, 32);
        f.field.update(values); projection.update(f.field.values, new Float32Array(f.field.values.length));
        const ranges = projection.uniforms.columnRanges.value.image.data;
        const current = projection.uniforms.columnCurrent.value.image.data;
        const previous = projection.uniforms.columnPrevious.value.image.data;
        for (let path = 0; path < 2; path++) {
            const at = path * 8, first = ranges[at], last = first + ranges[at + 1] - 1;
            assert.equal(ranges[at + 3], 0, 'A disconnected root has depth zero');
            assert.equal(ranges[at + 4], -1, 'A disconnected root has no parent to borrow from');
            assert.equal(ranges[at + 5], 0);
            assert.ok(Number.isFinite(current[last * 4 + 2]));
            if (path === 0) assert.ok(current[last * 4 + 2] > 14);
            else assert.equal(current[last * 4 + 2], 0);
            for (let i = first; i <= last; i++) {
                if (path === 1) assert.equal(current[i * 4 + 1], 0);
                assert.equal(previous[i * 4 + 1], 0); assert.equal(previous[i * 4 + 2], 0);
            }
        }
    } finally {projection.dispose(); geometry.dispose();}
});

test('column projection retains deduplicated BVH positions and bounded per-face metadata', () => {
    const f = flowFixture(), geometry = codedGeometry();
    const projection = buildAnatomicalColumnProjection(f.network, geometry, f.field, f);
    try {
        const bvh = projection.uniforms.lumenBvh.value;
        const sourceBytes = geometry.attributes.position.array.byteLength;
        assert.ok(bvh.position.image.data.byteLength < sourceBytes / 2,
            'Shared position-only BVH must not expand back to three independent vertices per face');
        const faceCount = geometry.attributes.position.count / 3;
        const faceBytes = projection.uniforms.columnFaces.value.image.data.byteLength;
        assert.ok(faceBytes < faceCount * 16 + 4096,
            'Column lookup stores one RGBA record per triangle, not repeated per-vertex or cell arrays');
        const textures = [bvh.index, bvh.position, bvh.bvhBounds, bvh.bvhContents,
            projection.uniforms.columnFaces.value, projection.uniforms.columnRanges.value,
            projection.uniforms.columnCurrent.value, projection.uniforms.columnPrevious.value,
            projection.uniforms.columnGeometry.value, projection.uniforms.columnGeometryRanges.value,
            projection.uniforms.columnJunctionBounds.value, projection.uniforms.columnJunctionRecords.value];
        assert.equal(projection.memoryBytes, textures.reduce((sum, texture) => sum + texture.image.data.byteLength, 0));
        const atlasBytes = projection.uniforms.columnGeometry.value.image.data.byteLength +
            projection.uniforms.columnGeometryRanges.value.image.data.byteLength;
        assert.ok(atlasBytes < 4096,
            'Spatial concentration geometry is shared per flow path, not duplicated per wall triangle');
    } finally {projection.dispose(); geometry.dispose();}
});

test('indirect column BVH contact cache updates every affected vertex and restores the original geometry', () => {
    const f = flowFixture(1), geometry = new THREE.CylinderGeometry(3, 3, 24, 48, 32, false).toNonIndexed();
    const coordinate = new Float32Array(geometry.attributes.position.count * 2);
    for (let i = 0; i < geometry.attributes.position.count; i++) coordinate[i * 2 + 1] = 12 - geometry.attributes.position.getY(i);
    geometry.setAttribute('columnCoordinate', new THREE.BufferAttribute(coordinate, 2));
    const projection = buildAnatomicalColumnProjection(f.network, geometry, f.field, f);
    const material = new THREE.ShaderMaterial({vertexShader: projection.vertexShader,
        fragmentShader: projection.fragmentShader, uniforms: {...projection.uniforms}});
    material.userData.attachVesselDeformation = projection.attachVesselDeformation;
    const deformation = new VesselDeformation();
    try {
        deformation.attach(material);
        const texture = projection.uniforms.lumenBvh.value.position, rest = texture.image.data.slice();
        const stride = rest.length / (texture.image.width * texture.image.height);
        const vertexCount = Math.max(...projection.uniforms.lumenBvh.value.index.image.data) + 1;
        const point = new THREE.Vector3(), delta = new THREE.Vector3();
        let moved = 0;
        for (const patches of [[{center: [3, -5, 0], width: 2.8, normal: [1, .2, 0], displacement: 1.1}],
            [{center: [-3, 8, 0], width: 2.4, normal: [-1, 0, .3], displacement: .7}], []]) {
            deformation.update(patches);
            for (let i = 0; i < vertexCount; i++) {
                point.fromArray(rest, i * stride); deformation.displacementAt(point, delta);
                if (delta.lengthSq() > 0) moved++;
                for (let axis = 0; axis < 3; axis++) assert.equal(texture.image.data[i * stride + axis],
                    Math.fround(point.getComponent(axis) + delta.getComponent(axis)),
                    `Indirect BVH vertex ${i}, component ${axis}, must match the raster contact deformation`);
            }
        }
        assert.ok(moved > 100, 'Local patches must exercise many BVH leaves on opposite cylinder sides');
        assert.deepEqual(texture.image.data, rest);
        assert.equal(projection.deformationDiagnostics.updatedVertices, 0);
        projection.dispose(); assert.equal(deformation.updateListeners.size, 0);
    } finally {projection.dispose(); geometry.dispose(); material.dispose(); deformation.dispose();}
});

test('column surface keeps clear entry caps and active draw groups for iodine hidden inside', () => {
    const f = axialProjectionFixture({projection: 'column'});
    try {
        f.fill(y => Math.abs(y) < 3 ? 1 : 0);
        const surface = f.surface, samples = surface.uniforms.columnCurrent.value.image.data;
        const range = surface.uniforms.columnRanges.value.image.data;
        const first = range[0], last = first + range[1] - 1;
        assert.equal(samples[first * 4 + 1], 0); assert.equal(samples[last * 4 + 1], 0);
        assert.ok(samples[last * 4 + 2] > 5.5, 'An internal bolus must retain its nonzero integrated iodine');
        assert.equal(surface.mesh.visible, true);
        const position = surface.samplingGeometry.attributes.position;
        const coordinates = surface.samplingGeometry.attributes.columnCoordinate;
        let clearEntryVertices = 0;
        for (const group of surface.mesh.geometry.groups) for (let i = group.start; i < group.start + group.count; i++) {
            if (Math.abs(position.getY(i) - 12) < 1e-5) {
                assert.equal(coordinates.getY(i), 0);
                clearEntryVertices++;
            }
        }
        assert.ok(clearEntryVertices > 100, 'The clear proximal entry cap must remain available to axial rays');
        f.fill(0); assert.equal(surface.mesh.visible, false);
    } finally {f.dispose();}
});

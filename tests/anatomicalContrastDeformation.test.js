import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {VesselDeformation} from '../src/vesselDeformation.js';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';
import {FlowConcentrationField} from '../src/contrast/flowConcentrationField.js';
import {buildAnatomicalContrastProjection} from '../src/contrast/anatomicalContrastProjection.js';

function fixture() {
    const geometry = new THREE.CylinderGeometry(3, 3, 12, 48, 24, false);
    // The projection consumes the inward-facing blood boundary.
    const index = geometry.index.array;
    for (let i = 0; i < index.length; i += 3) [index[i + 1], index[i + 2]] = [index[i + 2], index[i + 1]];
    const network = new ContrastFlowNetwork([{id: 0, nodeStartId: 0, nodeEndId: 1,
        start: new THREE.Vector3(0, 6, 0), end: new THREE.Vector3(0, -6, 0),
        radiusStart: 3, radiusEnd: 3}], {cellLengthMm: .75});
    const field = new FlowConcentrationField(network, new Uint32Array([0]), network.edges[0].cellCount);
    const projection = buildAnatomicalContrastProjection(network, geometry, field);
    const material = new THREE.ShaderMaterial({vertexShader: projection.vertexShader,
        fragmentShader: projection.fragmentShader, uniforms: {...projection.uniforms}});
    material.userData.attachVesselDeformation = projection.attachVesselDeformation;
    const deformation = new VesselDeformation();
    return {geometry, projection, material, deformation,
        dispose() {material.dispose(); projection.dispose(); geometry.dispose(); deformation.dispose();}};
}

const strongPatches = () => [
    {center: [3, 0, 0], width: 3, normal: [1, 0, 0], displacement: 1.5},
    {center: [2.9, .25, .1], width: 3, normal: [-1, 0, 0], displacement: 1.4},
    {center: [2.8, -.3, .35], width: 3, normal: [0, .6, .8], displacement: 1.2}
];

test('anatomical projection applies deformation to raster and ray triangles exactly once', () => {
    const f = fixture();
    try {
        const before = [f.material.vertexShader, f.material.fragmentShader];
        f.deformation.attach(f.material);
        const after = [f.material.vertexShader, f.material.fragmentShader];
        assert.notEqual(after[0], before[0]);
        assert.notEqual(after[1], before[1]);
        assert.equal(after[0].match(/vec3 vesselDisplacement\(vec3 p\)/g)?.length, 1);
        assert.doesNotMatch(after[1], /vesselDisplacement|uniform sampler2D vessel(?:Patches|Buckets|References)/,
            'Fragment traversal must use cached positions, never nested patch texture loops');
        assert.match(after[0], /projectionVertexPosition\(vec3 point\)\s*\{\s*return point \+ vesselDisplacement\(point\)/);
        assert.match(after[0], /vec4 view = modelViewMatrix \* vec4\(projected, 1\.\)/);
        assert.match(after[0], /lumenPosition = projected;/);
        assert.match(after[1], /projectionTrianglePosition\(vec3 point\)\s*\{\s*return point;/);
        // Every triangle vertex reads the revision-cached position texture.
        assert.equal(after[1].match(/projectionTrianglePosition\(texelFetch1D\(positionAttr, indices\.[xyz]\)\.rgb\)/g)?.length, 3);
        assert.match(after[1], /float padding = projectionBoundsPadding\(boundsMin, boundsMax\)/);
        assert.match(after[1], /boundsMin - vec3\(padding\), boundsMax \+ vec3\(padding\)/);
        f.deformation.attach(f.material);
        assert.deepEqual([f.material.vertexShader, f.material.fragmentShader], after);
        for (const [name, uniform] of Object.entries(f.deformation.uniforms))
            assert.equal(f.material.uniforms[name], uniform, `Live deformation uniform ${name} must be shared`);
    } finally {f.dispose();}
});

test('projection bounds follow growing, shrinking and removed live contact patches', () => {
    const f = fixture();
    try {
        f.deformation.attach(f.material);
        const sharedMaximum = f.material.uniforms.vesselDeformMaxDisplacement;
        const sharedActive = f.material.uniforms.vesselDeformActive;
        assert.equal(sharedMaximum.value, 0);
        f.deformation.update(strongPatches());
        assert.equal(sharedMaximum.value, 1.5);
        assert.equal(sharedActive.value, true);
        f.deformation.update([{center: [-3, 0, 0], width: 2, normal: [-1, 0, 0], displacement: .12}]);
        assert.equal(sharedMaximum.value, .12, 'Bounds must use the current maximum, not accumulate inflation');
        assert.equal(f.material.uniforms.vesselDeformMaxDisplacement, sharedMaximum);
        f.deformation.update([]);
        assert.equal(sharedMaximum.value, 0);
        assert.equal(sharedActive.value, false);
        assert.deepEqual(f.deformation.displacementAt(new THREE.Vector3(-3, 0, 0)), new THREE.Vector3());
    } finally {f.dispose();}
});

test('inflated BVH bounds contain strongly displaced vertices with competing contact normals', () => {
    const f = fixture();
    try {
        f.deformation.attach(f.material);
        const restPositions = f.projection.uniforms.lumenBvh.value.position.image.data.slice();
        f.deformation.update(strongPatches());
        const bvh = f.projection.uniforms.lumenBvh.value;
        const positions = restPositions, bounds = bvh.bvhBounds.image.data;
        const contents = bvh.bvhContents.image.data, indices = bvh.index.image.data;
        const positionStride = positions.length / (bvh.position.image.width * bvh.position.image.height);
        const indexStride = indices.length / (bvh.index.image.width * bvh.index.image.height);
        const maximum = f.material.uniforms.vesselDeformMaxDisplacement.value;
        const patchMin = f.material.uniforms.vesselDeformMin.value, patchMax = f.material.uniforms.vesselDeformMax.value;
        const deformed = new Map(), point = new THREE.Vector3(), displacement = new THREE.Vector3();
        let moved = 0, escapedRestRoot = 0, checkedNodes = 0, unaffectedNodes = 0;
        const displaced = vertex => {
            if (deformed.has(vertex)) return deformed.get(vertex);
            point.fromArray(positions, vertex * positionStride);
            f.deformation.displacementAt(point, displacement);
            assert.ok(displacement.length() <= maximum + 1e-12,
                'Competing patch directions cannot exceed the largest contact displacement');
            if (displacement.length() > 1e-4) moved++;
            const value = point.clone().add(displacement);
            if ([0, 1, 2].some(axis => value.getComponent(axis) < bounds[axis] || value.getComponent(axis) > bounds[4 + axis]))
                escapedRestRoot++;
            deformed.set(vertex, value);
            return value;
        };
        const visit = node => {
            checkedNodes++;
            const affected = ![0, 1, 2].some(axis => bounds[node * 8 + axis] > patchMax.getComponent(axis) ||
                bounds[node * 8 + 4 + axis] < patchMin.getComponent(axis));
            const padding = affected ? maximum + .0001 : 0;
            if (!affected) unaffectedNodes++;
            const packed = contents[node * 2], reference = contents[node * 2 + 1];
            let vertices;
            if (packed & 0xffff0000) {
                vertices = new Set();
                for (let triangle = reference; triangle < reference + (packed & 0xffff); triangle++)
                    for (let axis = 0; axis < 3; axis++) vertices.add(indices[triangle * indexStride + axis]);
            } else vertices = new Set([...visit(node + 1), ...visit(reference)]);
            for (const vertex of vertices) {
                const value = displaced(vertex);
                for (let axis = 0; axis < 3; axis++) {
                    assert.ok(value.getComponent(axis) >= bounds[node * 8 + axis] - padding,
                        `Deformed vertex ${vertex} escaped node ${node}'s lower bound`);
                    assert.ok(value.getComponent(axis) <= bounds[node * 8 + 4 + axis] + padding,
                        `Deformed vertex ${vertex} escaped node ${node}'s upper bound`);
                }
            }
            return vertices;
        };
        visit(0);
        assert.ok(checkedNodes > 20, 'Exercise both branch nodes and many leaves');
        assert.ok(unaffectedNodes > 20, 'Unchanged distant nodes must remain valid without any padding');
        assert.ok(moved > 100, 'The strong patches must visibly deform the fixture');
        assert.ok(escapedRestRoot > 0, 'At least one displaced vertex must actually require inflated bounds');
    } finally {f.dispose();}
});

test('revision cache matches CPU-deformed vertices, restores released contacts and skips unchanged updates', () => {
    const f = fixture();
    try {
        const texture = f.projection.uniforms.lumenBvh.value.position;
        const vertexCount = f.projection.uniforms.lumenBvh.value.index.image.data.reduce((a, b) => Math.max(a, b), 0) + 1;
        const rest = texture.image.data.slice(), stride = rest.length / (texture.image.width * texture.image.height);
        const point = new THREE.Vector3(), delta = new THREE.Vector3();
        f.deformation.attach(f.material);
        const cache = f.projection.deformationDiagnostics;
        assert.equal(cache.uploads, 0);
        for (const patches of [strongPatches(), strongPatches().slice(0, 1), []]) {
            f.deformation.update(patches);
            for (let at = 0; at < rest.length; at += stride) {
                point.set(rest[at], rest[at + 1], rest[at + 2]);
                // Texture padding is unused by triangles and remains untouched.
                if (at / stride >= vertexCount) continue;
                f.deformation.displacementAt(point, delta);
                assert.equal(texture.image.data[at], Math.fround(point.x + delta.x));
                assert.equal(texture.image.data[at + 1], Math.fround(point.y + delta.y));
                assert.equal(texture.image.data[at + 2], Math.fround(point.z + delta.z));
            }
            const version = texture.version, uploads = cache.uploads;
            assert.equal(f.deformation.update(patches), false);
            assert.equal(texture.version, version);
            assert.equal(cache.uploads, uploads);
        }
        assert.deepEqual(texture.image.data, rest);
        assert.equal(cache.updatedVertices, 0);
        assert.equal(f.deformation.updateListeners.size, 1);
        f.projection.dispose();
        assert.equal(f.deformation.updateListeners.size, 0);
    } finally {f.dispose();}
});

test('deformation updates preserve the source mesh and static projection BVH data', () => {
    const f = fixture();
    try {
        const sourcePositions = f.geometry.attributes.position.array.slice();
        const sourceIndices = f.geometry.index.array.slice();
        const bvh = f.projection.uniforms.lumenBvh.value;
        const bvhPositions = bvh.position.image.data.slice(), bvhBounds = bvh.bvhBounds.image.data.slice();
        f.deformation.attach(f.material);
        for (const patches of [strongPatches(), [], strongPatches().slice(0, 1), []]) f.deformation.update(patches);
        assert.deepEqual(f.geometry.attributes.position.array, sourcePositions);
        assert.deepEqual(f.geometry.index.array, sourceIndices);
        assert.deepEqual(bvh.position.image.data, bvhPositions);
        assert.deepEqual(bvh.bvhBounds.image.data, bvhBounds,
            'Live bound inflation belongs to traversal, not destructive edits to the rest BVH');
    } finally {f.dispose();}
});

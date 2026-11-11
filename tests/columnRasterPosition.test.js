import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {attachColumnRasterPosition} from '../src/contrast/columnRasterPosition.js';
import {VesselDeformation} from '../src/vesselDeformation.js';
import {axialProjectionFixture} from './helpers/contrastAxialProjectionFixture.js';

test('unindexed raster addresses the exact BVH vertex before, during and after contact', () => {
    const f = axialProjectionFixture({projection: 'column'}), deformation = new VesselDeformation();
    try {
        const surface = f.surface, material = surface.material, geometry = surface.mesh.geometry;
        assert.equal(geometry.index, null);
        deformation.attach(material);
        attachColumnRasterPosition(material, geometry);
        assert.match(material.vertexShader, /uniform BVH lumenBvh/);
        assert.doesNotMatch(material.vertexShader, /vesselDisplacement\(/);
        assert.equal(attachColumnRasterPosition(material, geometry), false, 'Attachment is idempotent');
        const bvh = surface.projection.uniforms.lumenBvh.value;
        const faces = geometry.attributes.columnFace, positions = geometry.attributes.position;
        const ids = bvh.index.image.data, xyz = bvh.position.image.data;
        const idStride = ids.length / (bvh.index.image.width * bvh.index.image.height);
        const stride = xyz.length / (bvh.position.image.width * bvh.position.image.height);
        const point = new THREE.Vector3(), delta = new THREE.Vector3();
        for (const patches of [[], [{center: [3, 0, 0], width: 5, normal: [1, .1, 0], displacement: .8}],
            [{center: [-3, 3, 0], width: 4, normal: [-1, 0, 0], displacement: .4}], []]) {
            deformation.update(patches);
            for (let vertex = 0; vertex < positions.count; vertex++) {
                const id = ids[Math.round(faces.getX(vertex)) * idStride + vertex % 3];
                point.fromBufferAttribute(positions, vertex); deformation.displacementAt(point, delta);
                for (let axis = 0; axis < 3; axis++) assert.equal(xyz[id * stride + axis],
                    Math.fround(point.getComponent(axis) + delta.getComponent(axis)),
                    `Raster vertex ${vertex}, axis ${axis} must share the ray's current wall`);
            }
        }
    } finally {deformation.dispose(); f.dispose();}
});

test('indexed custom rasters retain their original vertex implementation', () => {
    const geometry = new THREE.BoxGeometry(), material = new THREE.ShaderMaterial();
    try {
        const original = material.vertexShader;
        assert.equal(attachColumnRasterPosition(material, geometry), false);
        assert.equal(material.vertexShader, original);
    } finally {geometry.dispose(); material.dispose();}
});

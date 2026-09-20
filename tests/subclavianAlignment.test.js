import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import crypto from 'node:crypto';
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import { MeshBVH } from 'three-mesh-bvh';
import { transformAortaGeometry } from '../src/aortaTransform.js';
import { generateVessel } from '../src/vesselGeometry.js';
import { decodeCollisionAsset } from '../src/physics/collision/collisionAssetFormat.js';
import { PackedLumenField } from '../src/physics/collision/packedLumenField.js';

const root = new URL('../res/', import.meta.url);
const report = JSON.parse(fs.readFileSync(new URL('Aorta_plain.subclavian-alignment.json', root)));
const bytes = fs.readFileSync(new URL('Aorta_plain.stl', root));
const arrayBuffer = b => b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength);
const geometry = new STLLoader().parse(arrayBuffer(bytes));
transformAortaGeometry(geometry, generateVessel(140, 0).vessel);
const wall = new MeshBVH(geometry);
const asset = decodeCollisionAsset(arrayBuffer(fs.readFileSync(new URL('Aorta_plain.collision.bin', root))));
const lumen = new PackedLumenField(asset.metadata, asset.arrays);

test('visible anatomy, collision field and alignment manifest identify the same model', () => {
    const sha = crypto.createHash('sha256').update(bytes).digest('hex');
    assert.equal(report.outputSha256, sha);
    assert.equal(asset.metadata.source.stlSha256, sha);
    assert.equal(report.triangles, bytes.readUInt32LE(80));
    assert.equal(asset.metadata.centerline.diagnostics.componentCount, 1);
    assert.equal(asset.metadata.centerline.diagnostics.centerlineInvalidSegmentCountFinal, 0);
});

test('both subclavian paths retain continuous clearance for a 5 Fr catheter', () => {
    const hit = { point: new THREE.Vector3() }, p = new THREE.Vector3();
    const radius = 1.667 / 2;
    for (const curve of report.landmarks) {
        const start = new THREE.Vector3(...curve.actual[0]);
        assert.ok(lumen.query(start).inside, `${curve.side} entrance must be inside the lumen`);
        for (let i = 1; i < curve.actual.length; i++) {
            const a = new THREE.Vector3(...curve.actual[i - 1]), b = new THREE.Vector3(...curve.actual[i]);
            const count = Math.ceil(a.distanceTo(b) / .5);
            // Distance to a surface is 1-Lipschitz. The half-sample-spacing
            // margin certifies the continuous interval, including between probes.
            const margin = a.distanceTo(b) / count / 2;
            for (let j = 0; j <= count; j++) {
                p.copy(a).lerp(b, j / count);
                wall.closestPointToPoint(p, hit);
                assert.ok(hit.distance > radius + margin,
                    `${curve.side} junction obstructed at ${p.toArray()}: ${hit.distance}`);
            }
        }
    }
});

test('subclavian arches are medial and the main axes clear the loaded skeleton', () => {
    const skeleton = new OBJLoader().parse(fs.readFileSync(new URL('skeleton.obj', root), 'utf8'));
    const center = new THREE.Box3().setFromObject(skeleton).getCenter(new THREE.Vector3());
    // boneModel.js transform plus simulator.js parent offset, expressed in the
    // physics coordinates of the vessel (vascular render layer is Y - 15 mm).
    skeleton.position.sub(center);
    skeleton.rotation.z = -Math.PI / 3;
    skeleton.scale.multiplyScalar(9);
    skeleton.position.add(new THREE.Vector3(-1760, -645, -120));
    skeleton.updateMatrixWorld(true);
    const meshes = [];
    skeleton.traverse(object => {
        if (object.isMesh) meshes.push(object.geometry.clone().applyMatrix4(object.matrixWorld));
    });
    const bones = mergeGeometries(meshes), tree = new MeshBVH(bones);
    const hit = { point: new THREE.Vector3() }, p = new THREE.Vector3();
    for (const curve of report.landmarks) {
        const peak = curve.actual.reduce((a, b) => a[1] > b[1] ? a : b);
        assert.ok(Math.abs(peak[0]) < 100, `${curve.side} arch must not turn above the lateral shoulder`);
        assert.ok(peak[2] < -20, `${curve.side} arch must pass posteriorly`);
        for (let i = 1; i < curve.actual.length; i++) for (let j = 0; j <= 16; j++) {
            p.fromArray(curve.actual[i - 1]).lerp(new THREE.Vector3(...curve.actual[i]), j / 16);
            tree.closestPointToPoint(p, hit);
            assert.ok(hit.distance > 4.5, `${curve.side} axis too close to bone at ${p.toArray()}`);
        }
    }
    bones.dispose();
    for (const mesh of meshes) mesh.dispose();
});

test('reconstructed small shoulder branches still have terminal walls', () => {
    for (const branch of report.shoulderBranches) {
        const end = new THREE.Vector3(...branch.points.at(-1));
        const normal = end.clone().sub(new THREE.Vector3(...branch.points.at(-2))).normalize();
        const ray = new THREE.Ray(end.clone().addScaledVector(normal, -1), normal);
        const hit = wall.raycastFirst(ray, THREE.DoubleSide, 0, 2);
        assert.ok(hit, `${branch.side} thoracoacromial terminal must remain closed`);
    }
});

test.after(() => geometry.dispose());

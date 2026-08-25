import assert from 'node:assert/strict';
import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';
import {
    closestPointToPointScalarBvh,
    createScalarMeshBvhScratch
} from '../src/physics/collision/scalarMeshBvhClosestPoint.js';

const geometry = new THREE.TorusKnotGeometry(12, 2.5, 160, 12);
geometry.boundsTree = new MeshBVH(geometry);
const scratch = createScalarMeshBvhScratch();
const libraryTarget = {
    point: new THREE.Vector3(),
    distance: Infinity,
    faceIndex: -1
};
const scalarTarget = {
    point: new THREE.Vector3(),
    distance: Infinity,
    faceIndex: -1
};
let seed = 0x5f3759df;
const random = () => {
    seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
    return seed / 0x100000000;
};
const positions = geometry.attributes.position;
let previousFace = -1;
for (let sample = 0; sample < 1200; sample++) {
    const vertex = Math.floor(random() * positions.count);
    const x = positions.getX(vertex) + (random() * 2 - 1) * 5;
    const y = positions.getY(vertex) + (random() * 2 - 1) * 5;
    const z = positions.getZ(vertex) + (random() * 2 - 1) * 5;
    const point = new THREE.Vector3(x, y, z);
    libraryTarget.distance = Infinity;
    scalarTarget.distance = Infinity;
    const libraryResult = geometry.boundsTree.closestPointToPoint(
        point,
        libraryTarget
    );
    const scalarResult = closestPointToPointScalarBvh(
        geometry.boundsTree,
        x,
        y,
        z,
        scalarTarget,
        scratch,
        libraryResult.distance + 1e-12,
        previousFace
    );
    assert.equal(scalarResult, true);
    assert.equal(scalarTarget.distance, libraryResult.distance);
    assert.equal(scalarTarget.point.x, libraryResult.point.x);
    assert.equal(scalarTarget.point.y, libraryResult.point.y);
    assert.equal(scalarTarget.point.z, libraryResult.point.z);
    previousFace = libraryResult.faceIndex;
}

console.log('Scalar MeshBVH closest-point tests passed');

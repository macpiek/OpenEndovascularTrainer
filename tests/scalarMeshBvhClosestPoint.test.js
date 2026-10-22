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


// A cached triangle is only a search hint. If all triangles are outside the
// bounded query, reporting the hint as a hit suppresses the caller's exact
// unbounded retry and can create an invalid wall-clearance certificate.
for (const hint of [-1, previousFace]) {
    const point = new THREE.Vector3(0, 0, 40);
    const exact = geometry.boundsTree.closestPointToPoint(point, libraryTarget);
    const maximum = exact.distance * .5;
    const bounded = closestPointToPointScalarBvh(geometry.boundsTree,
        point.x, point.y, point.z, scalarTarget, scratch, maximum, hint);
    assert.equal(bounded, false, `Out-of-range triangle must not count as a hit (hint=${hint})`);
    assert.equal(closestPointToPointScalarBvh(geometry.boundsTree,
        point.x, point.y, point.z, scalarTarget, scratch, Infinity, hint), true);
    assert.ok(Math.abs(scalarTarget.distance-exact.distance)<1e-10);
}
geometry.dispose();

// Collapsed Boolean seam triangles must retain their finite edge distance.
const collapsed=new THREE.BufferGeometry();
collapsed.setAttribute('position',new THREE.Float32BufferAttribute([0,0,0,0,0,0,1,0,0],3));
collapsed.boundsTree=new MeshBVH(collapsed);
const collapsedTarget={point:new THREE.Vector3(),distance:Infinity,faceIndex:-1};
assert.equal(closestPointToPointScalarBvh(collapsed.boundsTree,.5,1,0,collapsedTarget,createScalarMeshBvhScratch()),true);
assert.equal(collapsedTarget.distance,1);
assert.deepEqual(collapsedTarget.point.toArray(),[.5,0,0]);
assert.equal(collapsedTarget.faceIndex,0);
collapsed.dispose();
console.log('Scalar MeshBVH closest-point tests passed');

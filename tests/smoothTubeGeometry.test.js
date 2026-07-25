import assert from 'node:assert/strict';
import * as THREE from 'three';
import { createSmoothTubeGeometry } from '../src/smoothTubeGeometry.js';

const points = [
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(5, 0, 0),
    new THREE.Vector3(10, 4, 0),
    new THREE.Vector3(15, 4, 2)
];
const geometry = createSmoothTubeGeometry(points, {
    radius: 0.5,
    samplesPerSegment: 4,
    radialSegments: 12
});

assert.equal(geometry.type, 'TubeGeometry');
assert.equal(geometry.userData.smoothTube.sourcePointCount, points.length);
assert.equal(geometry.userData.smoothTube.tubularSegments, 12);
assert.equal(geometry.userData.smoothTube.radialSegments, 12);
assert.ok(
    geometry.getAttribute('position').count > points.length * 12,
    'the smooth tube should interpolate substantially more rings than the source polyline'
);
assert.ok(geometry.getAttribute('normal'), 'the smooth tube should provide normals for a continuous outline');
assert.ok(geometry.index?.count > 0, 'the smooth tube should produce indexed surface triangles');
geometry.dispose();

const emptyGeometry = createSmoothTubeGeometry([points[0]], { radius: 0.5 });
assert.equal(emptyGeometry.getAttribute('position'), undefined);
emptyGeometry.dispose();

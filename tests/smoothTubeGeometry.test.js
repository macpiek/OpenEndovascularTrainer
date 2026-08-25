import assert from 'node:assert/strict';
import * as THREE from 'three';
import {
    createSmoothTubeGeometry,
    updateSmoothTubeGeometry
} from '../src/smoothTubeGeometry.js';

function referenceTube(sourcePoints, options) {
    const path = sourcePoints.length === 2
        ? new THREE.LineCurve3(sourcePoints[0], sourcePoints[1])
        : new THREE.CatmullRomCurve3(
            sourcePoints,
            false,
            'centripetal',
            0.5
        );
    return new THREE.TubeGeometry(
        path,
        options.tubularSegments,
        options.radius,
        options.radialSegments,
        false
    );
}

function assertGeometryMatches(actual, expected, tolerance = 0) {
    assert.deepEqual(
        Array.from(actual.index.array),
        Array.from(expected.index.array),
        'tube indices must be unchanged'
    );
    for (const name of ['position', 'normal', 'uv']) {
        const actualArray = actual.getAttribute(name).array;
        const expectedArray = expected.getAttribute(name).array;
        assert.equal(actualArray.length, expectedArray.length, `${name} length`);
        let maximumDifference = 0;
        for (let index = 0; index < actualArray.length; index++) {
            maximumDifference = Math.max(
                maximumDifference,
                Math.abs(actualArray[index] - expectedArray[index])
            );
        }
        assert.ok(
            maximumDifference <= tolerance,
            `${name} differs by ${maximumDifference}`
        );
    }
}

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

// The reusable implementation must produce the same surface as THREE's
// TubeGeometry, including arc-length sampling and Frenet-frame orientation.
const reference = referenceTube(points, {
    tubularSegments: 12,
    radius: 0.5,
    radialSegments: 12
});
assertGeometryMatches(geometry, reference);
reference.dispose();

// Moving points with unchanged topology updates the existing typed arrays and
// GPU attributes rather than allocating a new BufferGeometry each mesh tick.
const originalPositionArray = geometry.getAttribute('position').array;
const originalNormalArray = geometry.getAttribute('normal').array;
geometry.computeBoundingSphere();
points[1].set(4, 1, -0.5);
points[2].set(11, 5, 1.5);
const updated = updateSmoothTubeGeometry(geometry, points, {
    radius: 0.65,
    samplesPerSegment: 4,
    radialSegments: 12
});
assert.equal(updated, geometry, 'unchanged topology must reuse the geometry');
assert.equal(updated.getAttribute('position').array, originalPositionArray);
assert.equal(updated.getAttribute('normal').array, originalNormalArray);
const movedReference = referenceTube(points, {
    tubularSegments: 12,
    radius: 0.65,
    radialSegments: 12
});
assertGeometryMatches(updated, movedReference);
assert.ok(updated.boundingSphere, 'an existing culling bound must be refreshed');
movedReference.dispose();

// Callers can keep a capacity-sized point pool and pass its active prefix,
// eliminating the previous per-frame Array.slice allocation.
const pooledPoints = [...points, new THREE.Vector3(100, 100, 100)];
const prefixUpdated = updateSmoothTubeGeometry(updated, pooledPoints, {
    pointCount: points.length,
    radius: 0.65,
    samplesPerSegment: 4,
    radialSegments: 12
});
assert.equal(prefixUpdated, updated);
const prefixReference = referenceTube(points, {
    tubularSegments: 12,
    radius: 0.65,
    radialSegments: 12
});
assertGeometryMatches(prefixUpdated, prefixReference);
prefixReference.dispose();

// A real topology change returns a replacement so the caller can dispose the
// previous GPU buffers exactly once.
const topologyChanged = updateSmoothTubeGeometry(updated, pooledPoints, {
    radius: 0.65,
    samplesPerSegment: 4,
    radialSegments: 12
});
assert.notEqual(topologyChanged, updated);
assert.equal(topologyChanged.userData.smoothTube.tubularSegments, 16);
updated.dispose();
topologyChanged.dispose();

// The two-point path retains LineCurve3/TubeGeometry behavior as well.
const linePoints = [
    new THREE.Vector3(-2, 1, 3),
    new THREE.Vector3(9, -4, 7)
];
const lineGeometry = createSmoothTubeGeometry(linePoints, {
    radius: 0.25,
    samplesPerSegment: 3,
    radialSegments: 8
});
const lineReference = referenceTube(linePoints, {
    tubularSegments: 8,
    radius: 0.25,
    radialSegments: 8
});
assertGeometryMatches(lineGeometry, lineReference);
lineGeometry.dispose();
lineReference.dispose();

const emptyGeometry = createSmoothTubeGeometry([points[0]], { radius: 0.5 });
assert.equal(emptyGeometry.getAttribute('position'), undefined);
emptyGeometry.dispose();

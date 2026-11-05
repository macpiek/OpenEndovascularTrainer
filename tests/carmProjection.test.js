import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {applyAnteriorCArmPose} from '../src/carmPose.js';
import {applyCArmProjection} from '../src/imaging/carmProjection.js';

function close(actual, expected, message) {
    assert.ok(Math.abs(actual - expected) < 1e-7, `${message}: ${actual} != ${expected}`);
}

for (const detectorSideMm of [210, 300]) {
test(`${detectorSideMm / 10} cm detector corners coincide with the aperture at every zoom, SID and pose`, () => {
    const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 1600);
    for (const sourceRadius of [350, 650]) {
        for (const detectorRadius of [200, 350, 600]) {
            for (const zoom of [1, 1.4, 2.1]) {
                const fieldMm = detectorSideMm / zoom;
                for (const angles of [[0, 0, 0], [Math.PI / 2, Math.PI / 4, 0], [-Math.PI / 3, -Math.PI / 6, Math.PI / 2]]) {
                    applyAnteriorCArmPose(camera, new THREE.Vector3(30, -390, 160), sourceRadius, ...angles);
                    applyCArmProjection(camera, sourceRadius, detectorRadius, zoom, detectorSideMm);
                    for (const x of [-1, 1]) {
                        for (const y of [-1, 1]) {
                            // A known-size physical square at the detector plane,
                            // independently of the FOV formula under test.
                            const corner = new THREE.Vector3(x * fieldMm / 2, y * fieldMm / 2, -sourceRadius - detectorRadius);
                            corner.applyMatrix4(camera.matrixWorld).project(camera);
                            close(corner.x, x * 0.88, 'horizontal detector edge');
                            close(corner.y, y * 0.88, 'vertical detector edge');
                            close(corner.z, 1, 'detector depth');
                        }
                    }
                }
            }
        }
    }
});
}

test('objects beyond the detector are clipped and image scale is independent of display pixel count', () => {
    const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 1600);
    applyCArmProjection(camera, 350, 350);
    const frustum = new THREE.Frustum().setFromProjectionMatrix(camera.projectionMatrix);
    assert.ok(frustum.containsPoint(new THREE.Vector3(0, 0, -699)));
    assert.ok(!frustum.containsPoint(new THREE.Vector3(0, 0, -701)));
    for (const displaySidePx of [320, 685, 1370, 2048]) {
        // Resize follows the app's square viewport. Pixel density must never
        // expand the anatomy captured by a fixed physical detector.
        camera.aspect = displaySidePx / displaySidePx;
        camera.updateProjectionMatrix();
        const edge = new THREE.Vector3(150, 0, -700).project(camera);
        close(edge.x, 0.88, 'detector edge after resize');
    }
});

test('legacy fixed 45 degree projection exposes anatomy outside the detector', () => {
    const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 1600);
    const beyondDetectorWidth = new THREE.Vector3(120, 0, -510);
    assert.ok(Math.abs(beyondDetectorWidth.clone().project(camera).x) < 0.88);
    applyCArmProjection(camera, 350, 350);
    assert.ok(Math.abs(beyondDetectorWidth.project(camera).x) > 0.88);
});

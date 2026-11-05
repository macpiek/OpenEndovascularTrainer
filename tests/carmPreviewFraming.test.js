import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {createCArmModel} from '../src/ui/carmModel.js';
import {createOperatingTable} from '../src/ui/operatingTable.js';
import {createCArmPreviewFramer} from '../src/ui/carmPreviewFraming.js';
import {disposeThreeResources} from '../src/disposeThreeResources.js';

function previewCamera(aspect = 150 / 156) {
    const camera = new THREE.PerspectiveCamera(39, aspect, 0.1, 1000);
    camera.position.set(268, 146, 289);
    camera.lookAt(new THREE.Vector3(0, 24, -30));
    return camera;
}

test('complete C-arm, patient and table fit after movement and resizing with a fixed viewing direction', () => {
    const arm = createCArmModel(), table = createOperatingTable();
    arm.group.position.x = 10;
    const objects = [arm.group, table], camera = previewCamera();
    const orientation = camera.quaternion.clone();
    const fit = createCArmPreviewFramer(camera, objects);
    const point = new THREE.Vector3();
    const yawRotation = new THREE.Quaternion(), pitchRotation = new THREE.Quaternion();
    const xAxis = new THREE.Vector3(1, 0, 0), zAxis = new THREE.Vector3(0, 0, 1);
    const deg = THREE.MathUtils.degToRad;
    try {
        for (const aspect of [150 / 156, 260 / 148, 0.5, 3.2]) {
            camera.aspect = aspect;
            for (const [yaw, pitch, roll, height, slideX, slideZ] of [
                [0, 0, 0, 0, 0, 0],
                [-30, 0, 0, 0, 0, 0], [30, 0, 0, 0, 0, 0],
                [-90, -45, -90, -12, -72.8, -16],
                [90, 45, 90, 14.4, 55.2, 16],
                [-90, 45, 90, 14.4, -72.8, 16],
                [90, -45, -90, -12, 55.2, -16],
                [0, 0, 0, -12, 0, 0], [0, 0, 0, 14.4, 0, 0]
            ]) {
                yawRotation.setFromAxisAngle(xAxis, -deg(yaw));
                pitchRotation.setFromAxisAngle(zAxis, deg(pitch));
                arm.gantryGroup.quaternion.copy(pitchRotation).multiply(yawRotation);
                arm.detectorAssembly.rotation.y = deg(roll);
                arm.liftGroup.position.y = height;
                table.userData.slideGroup.position.set(slideX, 0, slideZ);
                assert.equal(fit(), true);
                assert.ok(camera.quaternion.equals(orientation), 'reframing must not orbit or flip the view');
                let occupiedExtent = 0;
                for (const root of objects) root.traverseVisible(object => {
                    if (!object.isMesh) return;
                    const positions = object.geometry.getAttribute('position');
                    // Check actual model vertices, independently of the mesh
                    // bounds and distance calculation used by the framer.
                    for (let index = 0; index < positions.count; index++) {
                        point.fromBufferAttribute(positions, index)
                            .applyMatrix4(object.matrixWorld).project(camera);
                        const extent = Math.max(Math.abs(point.x), Math.abs(point.y));
                        assert.ok(extent <= 0.920001, `clipped equipment at aspect ${aspect}: ${extent}`);
                        assert.ok(point.z > -1 && point.z < 1, 'equipment must be inside the depth range');
                        occupiedExtent = Math.max(occupiedExtent, extent);
                    }
                });
                assert.ok(occupiedExtent > 0.8, 'equipment must fill the available dimension');
            }
        }
    } finally {
        disposeThreeResources({roots: objects});
    }
});

test('framing uses only visible equipment and is stable on repeated renders', () => {
    const equipment = new THREE.Mesh(new THREE.BoxGeometry(100, 80, 60), new THREE.MeshBasicMaterial());
    const hidden = equipment.clone();
    hidden.position.x = 10000;
    hidden.visible = false;
    const root = new THREE.Group();
    root.add(equipment, hidden);
    const camera = previewCamera(), fit = createCArmPreviewFramer(camera, [root]);
    try {
        fit();
        const position = camera.position.clone(), projection = camera.projectionMatrix.clone();
        for (let index = 0; index < 20; index++) fit();
        assert.ok(camera.position.equals(position));
        assert.ok(camera.projectionMatrix.equals(projection));
        hidden.visible = true;fit();
        assert.ok(camera.position.distanceTo(position) > 1000, 'visible equipment must be included');
    } finally {
        disposeThreeResources({roots: [root]});
    }
});

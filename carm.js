import * as THREE from 'three';
import { renderCArmPreview } from './carmPreview.js';

export function setupCArmControls(camera, vessel, cameraRadius, previewGroup) {
    const carmYawSlider = document.getElementById('carmYaw');
    const carmPitchSlider = document.getElementById('carmPitch');
    const carmRollSlider = document.getElementById('carmRoll');
    const carmXSlider = document.getElementById('carmX');
    const carmYSlider = document.getElementById('carmY');
    const carmZSlider = document.getElementById('carmZ');

    const sliders = [
        carmYawSlider,
        carmPitchSlider,
        carmRollSlider,
        carmXSlider,
        carmYSlider,
        carmZSlider
    ];
    sliders.forEach(s => s.addEventListener('change', () => s.blur()));

    let carmYaw = 0;
    let carmPitch = 0;
    let carmRoll = 0;
    let carmX = 0;
    let carmY = -60;
    let carmZ = 0;

    function getPivotPoint() {
        return new THREE.Vector3(
            vessel.branchPoint.x + carmX,
            vessel.branchPoint.y + carmY,
            vessel.branchPoint.z + carmZ
        );
    }

    function updateCamera() {
        const pivot = getPivotPoint();
        const offset = new THREE.Vector3().setFromSpherical(
            new THREE.Spherical(cameraRadius, Math.PI / 2 - carmPitch, carmYaw)
        );
        camera.position.copy(pivot).add(offset);
        camera.up.set(0, 1, 0);
        camera.lookAt(pivot);
        camera.rotateZ(carmRoll);

        if (previewGroup) {
            previewGroup.position.copy(pivot);
            previewGroup.rotation.set(carmPitch, carmYaw, carmRoll, 'YXZ');
            renderCArmPreview();
        }
    }

    updateCamera();

    carmYawSlider.addEventListener('input', e => {
        carmYaw = parseFloat(e.target.value) * Math.PI / 180;
        updateCamera();
    });
    carmPitchSlider.addEventListener('input', e => {
        carmPitch = parseFloat(e.target.value) * Math.PI / 180;
        updateCamera();
    });
    carmRollSlider.addEventListener('input', e => {
        carmRoll = parseFloat(e.target.value) * Math.PI / 180;
        updateCamera();
    });
    carmXSlider.addEventListener('input', e => {
        carmX = parseFloat(e.target.value);
        updateCamera();
    });
    carmYSlider.addEventListener('input', e => {
        carmY = parseFloat(e.target.value);
        updateCamera();
    });
    carmZSlider.addEventListener('input', e => {
        carmZ = parseFloat(e.target.value);
        updateCamera();
    });
}


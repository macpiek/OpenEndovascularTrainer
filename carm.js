import * as THREE from 'three';
import { renderCArmPreview } from './carmPreview.js';

export function setupCArmControls(camera, vessel, cameraRadius, previewGroup, previewGantry) {
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

    let carmYaw = parseFloat(carmYawSlider.value) * Math.PI / 180;
    let carmPitch = parseFloat(carmPitchSlider.value) * Math.PI / 180;
    let carmRoll = parseFloat(carmRollSlider.value) * Math.PI / 180;
    let carmX = parseFloat(carmXSlider.value);
    let carmY = parseFloat(carmYSlider.value);
    let carmZ = parseFloat(carmZSlider.value);

    const initialX = carmX;
    const initialY = carmY;
    const initialZ = carmZ;

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
            previewGroup.position.set(
                carmX - initialX,
                carmY - initialY,
                carmZ - initialZ
            );
        }

        if (previewGantry) {
            previewGantry.rotation.set(0, 0, 0);
            previewGantry.rotateY(carmYaw);
            previewGantry.rotateX(carmPitch);
            previewGantry.rotateZ(carmRoll);
        }

        if (previewGroup || previewGantry) {
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


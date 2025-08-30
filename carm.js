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
            // Yaw: lean toward patient's sides (rotate around Z)
            previewGantry.rotateZ(carmPitch);
            // Pitch: tilt toward head or feet (rotate around X)
            previewGantry.rotateX(carmRoll);
            // Roll: spin around vertical axis
            previewGantry.rotateY(carmYaw);
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
    const joystick = document.getElementById('joystick');
    const joystickHandle = document.getElementById('joystick-handle');
    if (joystick && joystickHandle) {
        const handleRadius = joystickHandle.offsetWidth / 2;
        const maxDistance = joystick.offsetWidth / 2 - handleRadius;
        let dragging = false;
        const handleTransition = 'transform 0.2s ease-out';

        let speedX = 0;
        let speedY = 0;
        const minX = parseFloat(carmXSlider.min);
        const maxX = parseFloat(carmXSlider.max);
        const minY = parseFloat(carmYSlider.min);
        const maxY = parseFloat(carmYSlider.max);
        const maxSpeedX = (maxX - minX) / 2;
        const maxSpeedY = (maxY - minY) / 2;
        let lastTime = performance.now();

        function step(now) {
            const dt = (now - lastTime) / 1000;
            lastTime = now;
            if (speedX !== 0 || speedY !== 0) {
                carmX = Math.min(Math.max(carmX + speedX * maxSpeedX * dt, minX), maxX);
                carmY = Math.min(Math.max(carmY + speedY * maxSpeedY * dt, minY), maxY);
                carmXSlider.value = carmX;
                carmYSlider.value = carmY;
                updateCamera();
            }
            requestAnimationFrame(step);
        }
        requestAnimationFrame(step);


        function updateFromJoystick(clientX, clientY) {
            const rect = joystick.getBoundingClientRect();
            let x = clientX - rect.left - rect.width / 2;
            let y = clientY - rect.top - rect.height / 2;
            const dist = Math.hypot(x, y);
            if (dist > maxDistance) {
                const ratio = maxDistance / dist;
                x *= ratio;
                y *= ratio;
            }
            joystickHandle.style.transform = `translate(-50%, -50%) translate(${x}px, ${y}px)`;
            const normX = x / maxDistance;
            const normY = y / maxDistance;
            speedX = normX;
            speedY = -normY;
        }

        joystick.addEventListener('mousedown', e => {
            dragging = true;
            joystickHandle.style.transition = 'none';
            updateFromJoystick(e.clientX, e.clientY);
        });
        window.addEventListener('mousemove', e => {
            if (!dragging) return;
            updateFromJoystick(e.clientX, e.clientY);
        });
        window.addEventListener('mouseup', () => {
            dragging = false;
            joystickHandle.style.transition = handleTransition;
            joystickHandle.style.transform = 'translate(-50%, -50%)';
            speedX = 0;
            speedY = 0;

        });
    }
}


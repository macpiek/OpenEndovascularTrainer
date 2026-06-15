import * as THREE from 'three';

// The perspective camera is placed at the X-ray source so that the rendered
// image matches what a detector would capture. A virtual detector sits opposite
// the source and its distance from the isocentre can be adjusted to vary
// magnification. Rays diverge from the source toward this plane, so objects
// nearer the source appear larger on the on-screen "detector" view just as in a
// real C-arm.
export function setupCArmControls(camera, vessel, cameraRadius, previewGroup, previewGantry, previewLift, previewTable, renderPreview = () => {}) {
    const carmXSlider = document.getElementById('carmX');
    const carmYSlider = document.getElementById('carmY');
    const carmZSlider = document.getElementById('carmZ');
    const carmDetDistSlider = document.getElementById('carmDetDist');
    const carmZUpButton = document.getElementById('carmZUp');
    const carmZDownButton = document.getElementById('carmZDown');
    const carmRollLeftButton = document.getElementById('carmRollLeft');
    const carmRollRightButton = document.getElementById('carmRollRight');
    const carmAngleResetButton = document.getElementById('carmAngleReset');
    const carmLao30Button = document.getElementById('carmLao30');
    const carmRao30Button = document.getElementById('carmRao30');
    const carmYawReadout = document.getElementById('carmYawReadout');
    const carmPitchReadout = document.getElementById('carmPitchReadout');
    const carmRollReadout = document.getElementById('carmRollReadout');

    const sliders = [
        carmXSlider,
        carmYSlider,
        carmZSlider,
        carmDetDistSlider
    ];
    sliders.filter(Boolean).forEach(s => s.addEventListener('change', () => s.blur()));

    let carmYaw = 0;
    let carmPitch = 0;
    let carmRoll = 0;
    let carmX = parseFloat(carmXSlider.value);
    let carmY = parseFloat(carmYSlider.value);
    let carmZ = parseFloat(carmZSlider.value);
    let detectorRadius = parseFloat(carmDetDistSlider.value);

    const initialX = carmX;
    const initialY = carmY;
    const initialZ = carmZ;
    const previewPelvisX = 10;
    const previewYawAxis = new THREE.Vector3(1, 0, 0);
    const previewPitchAxis = new THREE.Vector3(0, 0, 1);
    const previewRollAxis = new THREE.Vector3(0, 1, 0);
    const previewYawQuat = new THREE.Quaternion();
    const previewPitchQuat = new THREE.Quaternion();
    const previewRollQuat = new THREE.Quaternion();

    function getPivotPoint() {
        const fluoroscopyZ = initialZ - (carmZ - initialZ);
        return new THREE.Vector3(
            vessel.branchPoint.x + carmX,
            vessel.branchPoint.y + carmY,
            vessel.branchPoint.z + fluoroscopyZ
        );
    }

    // Distance from isocentre to detector is set by the slider. The source
    // remains `cameraRadius` away on the opposite side.

    function toDegrees(rad) {
        return Math.round(THREE.MathUtils.radToDeg(rad));
    }

    function formatYaw(deg) {
        if (deg === 0) return 'AP 0°';
        return `${deg > 0 ? 'LAO' : 'RAO'} ${Math.abs(deg)}°`;
    }

    function formatPitch(deg) {
        if (deg === 0) return 'CRA 0°';
        return `${deg > 0 ? 'CRA' : 'CAU'} ${Math.abs(deg)}°`;
    }

    function updateReadouts() {
        if (carmYawReadout) carmYawReadout.textContent = formatYaw(toDegrees(carmYaw));
        if (carmPitchReadout) carmPitchReadout.textContent = formatPitch(toDegrees(carmPitch));
        if (carmRollReadout) carmRollReadout.textContent = `Roll ${toDegrees(carmRoll)}°`;
    }

    function updateCamera() {
        const pivot = getPivotPoint();
        // Direction from isocentre toward the source/detector axis.
        const dir = new THREE.Vector3().setFromSpherical(
            new THREE.Spherical(1, Math.PI / 2 - carmPitch, carmYaw)
        ).normalize();

        // Position the source (camera) opposite the detector.
        const sourcePos = pivot.clone().addScaledVector(dir, cameraRadius);
        const detectorPos = pivot.clone().addScaledVector(dir, -detectorRadius);

        // Render from the source position while looking toward the detector so
        // the perspective matches the detector's recorded image.
        camera.position.copy(sourcePos);
        camera.up.set(0, 1, 0);
        camera.lookAt(detectorPos);
        camera.rotateZ(carmRoll);

        const previewDx = carmX - initialX;
        const previewDy = carmY - initialY;
        const previewDz = carmZ - initialZ;
        if (previewGroup) {
            previewGroup.position.set(previewPelvisX, 0, 0);
        }
        if (previewLift) {
            previewLift.position.y = previewDz * 0.12;
        }

        if (previewTable) {
            const slideGroup = previewTable.userData.slideGroup || previewTable;
            slideGroup.position.set(previewDy * 0.08, 0, previewDx * 0.08);
        }

        if (previewGantry) {
            // Patient axis in preview: X=head/feet, Y=vertical beam, Z=lateral.
            // AP starts over the pelvis; LAO/RAO and CRA/CAU are composed on
            // the initial fixed axes so cranial/caudal tilt is not dragged by
            // the current LAO/RAO angle.
            previewYawQuat.setFromAxisAngle(previewYawAxis, -carmYaw);
            previewPitchQuat.setFromAxisAngle(previewPitchAxis, carmPitch);
            previewRollQuat.setFromAxisAngle(previewRollAxis, carmRoll);
            previewGantry.quaternion.copy(previewPitchQuat).multiply(previewYawQuat).multiply(previewRollQuat);
        }

        if (previewGroup || previewGantry || previewLift || previewTable) {
            renderPreview();
        }
        updateReadouts();
    }

    updateCamera();
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
    carmDetDistSlider.addEventListener('input', e => {
        detectorRadius = parseFloat(e.target.value);
        updateCamera();
    });
    const positionJoystick = document.getElementById('positionJoystick');
    const positionJoystickHandle = document.getElementById('positionJoystickHandle');
    const angleJoystick = document.getElementById('angleJoystick');
    const angleJoystickHandle = document.getElementById('angleJoystickHandle');

    let rollSpeed = 0;
    let angleSpeedYaw = 0;
    let angleSpeedPitch = 0;
    let angleResetActive = false;
    let angleTargetYaw = null;
    let activeAngleTargetButton = null;
    const maxYaw = THREE.MathUtils.degToRad(90);
    const maxPitch = THREE.MathUtils.degToRad(45);
    const maxRoll = THREE.MathUtils.degToRad(45);
    const yawRate = THREE.MathUtils.degToRad(22);
    const pitchRate = THREE.MathUtils.degToRad(18);
    const rollRate = THREE.MathUtils.degToRad(18);
    const angleResetRate = THREE.MathUtils.degToRad(24);
    const angleTargetRate = THREE.MathUtils.degToRad(24);
    const angleAxisDeadzone = 0.22;

    function applyAxisDeadzone(value, deadzone) {
        const magnitude = Math.abs(value);
        if (magnitude < deadzone) return 0;
        return Math.sign(value) * ((magnitude - deadzone) / (1 - deadzone));
    }

    function wireJoystick(joystick, joystickHandle, onMove, onRelease, { resetOnRelease = true } = {}) {
        if (!joystick || !joystickHandle) return;
        const handleRadius = joystickHandle.offsetWidth / 2;
        const maxDistance = joystick.offsetWidth / 2 - handleRadius;
        let dragging = false;
        const handleTransition = 'transform 0.2s ease-out';
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
            onMove(x / maxDistance, y / maxDistance);
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
            if (!dragging) return;
            dragging = false;
            joystickHandle.style.transition = handleTransition;
            if (resetOnRelease) joystickHandle.style.transform = 'translate(-50%, -50%)';
            onRelease();
        });
        joystick.addEventListener('pointerdown', e => {
            dragging = true;
            joystick.setPointerCapture?.(e.pointerId);
            joystickHandle.style.transition = 'none';
            updateFromJoystick(e.clientX, e.clientY);
        });
        joystick.addEventListener('pointermove', e => {
            if (!dragging) return;
            updateFromJoystick(e.clientX, e.clientY);
        });
        joystick.addEventListener('pointerup', e => {
            if (!dragging) return;
            dragging = false;
            joystick.releasePointerCapture?.(e.pointerId);
            joystickHandle.style.transition = handleTransition;
            if (resetOnRelease) joystickHandle.style.transform = 'translate(-50%, -50%)';
            onRelease();
        });
        joystick.addEventListener('pointercancel', e => {
            if (!dragging) return;
            dragging = false;
            joystick.releasePointerCapture?.(e.pointerId);
            joystickHandle.style.transition = handleTransition;
            if (resetOnRelease) joystickHandle.style.transform = 'translate(-50%, -50%)';
            onRelease();
        });
        joystick.addEventListener('touchstart', e => {
            e.preventDefault();
            dragging = true;
            joystickHandle.style.transition = 'none';
            const t = e.touches[0];
            updateFromJoystick(t.clientX, t.clientY);
        });
        window.addEventListener('touchmove', e => {
            if (!dragging) return;
            const t = e.touches[0];
            updateFromJoystick(t.clientX, t.clientY);
        }, { passive: false });
        window.addEventListener('touchend', () => {
            if (!dragging) return;
            dragging = false;
            joystickHandle.style.transition = handleTransition;
            if (resetOnRelease) joystickHandle.style.transform = 'translate(-50%, -50%)';
            onRelease();
        });
    }

    let speedX = 0;
    let speedY = 0;
    let speedZ = 0;
    const minX = parseFloat(carmXSlider.min);
    const maxX = parseFloat(carmXSlider.max);
    const minY = parseFloat(carmYSlider.min);
    const maxY = parseFloat(carmYSlider.max);
    const minZ = parseFloat(carmZSlider.min);
    const maxZ = parseFloat(carmZSlider.max);
    const maxSpeedX = (maxX - minX) * 0.18;
    const maxSpeedY = (maxY - minY) * 0.18;
    const maxSpeedZ = (maxZ - minZ) * 0.18;
    let lastTime = performance.now();

    function moveTowardZero(value, amount) {
        if (Math.abs(value) <= amount) return 0;
        return value - Math.sign(value) * amount;
    }

    function step(now) {
        const dt = (now - lastTime) / 1000;
        lastTime = now;
        let updated = false;
        if (speedX !== 0 || speedY !== 0) {
            carmX = Math.min(Math.max(carmX + speedX * maxSpeedX * dt, minX), maxX);
            carmY = Math.min(Math.max(carmY + speedY * maxSpeedY * dt, minY), maxY);
            carmXSlider.value = Math.round(carmX);
            carmYSlider.value = Math.round(carmY);
            updated = true;
        }
        if (speedZ !== 0) {
            const nextZ = THREE.MathUtils.clamp(carmZ + speedZ * maxSpeedZ * dt, minZ, maxZ);
            updated = updated || nextZ !== carmZ;
            carmZ = nextZ;
            carmZSlider.value = Math.round(carmZ);
        }
        if (angleSpeedYaw !== 0 || angleSpeedPitch !== 0) {
            carmYaw = Math.min(Math.max(carmYaw + angleSpeedYaw * yawRate * dt, -maxYaw), maxYaw);
            carmPitch = Math.min(Math.max(carmPitch + angleSpeedPitch * pitchRate * dt, -maxPitch), maxPitch);
            updated = true;
        }
        if (rollSpeed !== 0) {
            carmRoll = Math.min(Math.max(carmRoll + rollSpeed * rollRate * dt, -maxRoll), maxRoll);
            updated = true;
        }
        if (angleResetActive) {
            angleSpeedYaw = 0;
            angleSpeedPitch = 0;
            angleTargetYaw = null;
            activeAngleTargetButton?.classList.remove('active');
            activeAngleTargetButton = null;
            rollSpeed = 0;
            const resetStep = angleResetRate * dt;
            const nextYaw = moveTowardZero(carmYaw, resetStep);
            const nextPitch = moveTowardZero(carmPitch, resetStep);
            const nextRoll = moveTowardZero(carmRoll, resetStep);
            updated = updated || nextYaw !== carmYaw || nextPitch !== carmPitch || nextRoll !== carmRoll;
            carmYaw = nextYaw;
            carmPitch = nextPitch;
            carmRoll = nextRoll;
        }
        if (angleTargetYaw !== null) {
            angleSpeedYaw = 0;
            const targetStep = angleTargetRate * dt;
            const delta = angleTargetYaw - carmYaw;
            const nextYaw = Math.abs(delta) <= targetStep
                ? angleTargetYaw
                : carmYaw + Math.sign(delta) * targetStep;
            updated = updated || nextYaw !== carmYaw;
            carmYaw = THREE.MathUtils.clamp(nextYaw, -maxYaw, maxYaw);
        }
        if (updated) {
            updateCamera();
        }
        requestAnimationFrame(step);
    }
    requestAnimationFrame(step);

    function startZ(dir) {
        speedZ = dir;
    }
    function stopZ() {
        speedZ = 0;
    }
    if (carmZUpButton && carmZDownButton) {
        carmZUpButton.addEventListener('mousedown', () => startZ(1));
        carmZDownButton.addEventListener('mousedown', () => startZ(-1));
        window.addEventListener('mouseup', stopZ);
        carmZUpButton.addEventListener('touchstart', e => { e.preventDefault(); startZ(1); });
        carmZDownButton.addEventListener('touchstart', e => { e.preventDefault(); startZ(-1); });
        window.addEventListener('touchend', stopZ);
        window.addEventListener('touchcancel', stopZ);
    }

    function startRoll(dir) {
        rollSpeed = dir;
    }
    function stopRoll() {
        rollSpeed = 0;
    }
    if (carmRollLeftButton && carmRollRightButton) {
        carmRollLeftButton.addEventListener('mousedown', () => startRoll(-1));
        carmRollRightButton.addEventListener('mousedown', () => startRoll(1));
        window.addEventListener('mouseup', stopRoll);
        carmRollLeftButton.addEventListener('touchstart', e => { e.preventDefault(); startRoll(-1); });
        carmRollRightButton.addEventListener('touchstart', e => { e.preventDefault(); startRoll(1); });
        window.addEventListener('touchend', stopRoll);
        window.addEventListener('touchcancel', stopRoll);
    }

    function startAngleReset(e) {
        e?.preventDefault?.();
        angleResetActive = true;
        angleSpeedYaw = 0;
        angleSpeedPitch = 0;
        rollSpeed = 0;
        if (angleJoystickHandle) {
            angleJoystickHandle.style.transition = 'transform 0.2s ease-out';
            angleJoystickHandle.style.transform = 'translate(-50%, -50%)';
        }
        carmAngleResetButton?.classList.add('active');
    }
    function stopAngleReset() {
        angleResetActive = false;
        carmAngleResetButton?.classList.remove('active');
    }
    if (carmAngleResetButton) {
        carmAngleResetButton.addEventListener('pointerdown', e => {
            carmAngleResetButton.setPointerCapture?.(e.pointerId);
            startAngleReset(e);
        });
        carmAngleResetButton.addEventListener('pointerup', e => {
            carmAngleResetButton.releasePointerCapture?.(e.pointerId);
            stopAngleReset();
        });
        carmAngleResetButton.addEventListener('pointercancel', stopAngleReset);
        carmAngleResetButton.addEventListener('pointerleave', stopAngleReset);
        carmAngleResetButton.addEventListener('click', e => e.preventDefault());
        window.addEventListener('blur', stopAngleReset);
    }

    function startAngleTarget(targetYaw, button, e) {
        e?.preventDefault?.();
        angleResetActive = false;
        carmAngleResetButton?.classList.remove('active');
        angleSpeedYaw = 0;
        angleSpeedPitch = 0;
        angleTargetYaw = THREE.MathUtils.clamp(targetYaw, -maxYaw, maxYaw);
        if (activeAngleTargetButton && activeAngleTargetButton !== button) {
            activeAngleTargetButton.classList.remove('active');
        }
        activeAngleTargetButton = button;
        activeAngleTargetButton?.classList.add('active');
        if (angleJoystickHandle) {
            angleJoystickHandle.style.transition = 'transform 0.2s ease-out';
            angleJoystickHandle.style.transform = 'translate(-50%, -50%)';
        }
    }
    function stopAngleTarget() {
        angleTargetYaw = null;
        activeAngleTargetButton?.classList.remove('active');
        activeAngleTargetButton = null;
    }
    function bindAngleTarget(button, targetYaw) {
        if (!button) return;
        button.addEventListener('pointerdown', e => {
            button.setPointerCapture?.(e.pointerId);
            startAngleTarget(targetYaw, button, e);
        });
        button.addEventListener('pointerup', e => {
            button.releasePointerCapture?.(e.pointerId);
            stopAngleTarget();
        });
        button.addEventListener('pointercancel', stopAngleTarget);
        button.addEventListener('pointerleave', stopAngleTarget);
        button.addEventListener('click', e => e.preventDefault());
    }
    bindAngleTarget(carmLao30Button, THREE.MathUtils.degToRad(30));
    bindAngleTarget(carmRao30Button, THREE.MathUtils.degToRad(-30));
    window.addEventListener('blur', stopAngleTarget);

    wireJoystick(positionJoystick, positionJoystickHandle, (normX, normY) => {
        speedX = -normY;
        speedY = -normX;
    }, () => {
        speedX = 0;
        speedY = 0;
    });

    wireJoystick(angleJoystick, angleJoystickHandle, (normX, normY) => {
        stopAngleTarget();
        angleSpeedYaw = applyAxisDeadzone(-normY, angleAxisDeadzone);
        angleSpeedPitch = applyAxisDeadzone(-normX, angleAxisDeadzone);
    }, () => {
        angleSpeedYaw = 0;
        angleSpeedPitch = 0;
    });
}

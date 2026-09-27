import * as THREE from 'three';

const bounded = (value, limit) => Number.isFinite(value)
    ? THREE.MathUtils.clamp(value, -limit, limit) : 0;

/** Absolute pose on the anterior (+Z) hemisphere about the isocentre.
 * Both source position and optical frame come from the same bounded angles.
 * At either lateral stop depth is zero; there is no posterior branch to select
 * when changing direction. No previous camera Euler angles are read back.
 */
export function applyAnteriorCArmPose(camera, pivot, radius, yaw, pitch, roll) {
    yaw = bounded(yaw, Math.PI / 2);
    pitch = bounded(pitch, Math.PI / 4);
    roll = bounded(roll, Math.PI / 2);
    const sinYaw = Math.sin(yaw), cosYaw = Math.max(0, Math.cos(yaw));
    const sinPitch = Math.sin(pitch), cosPitch = Math.cos(pitch);
    const outward = new THREE.Vector3(sinYaw * cosPitch, sinPitch, cosYaw * cosPitch);
    const right = new THREE.Vector3(cosYaw, 0, -sinYaw);
    const up = new THREE.Vector3().crossVectors(outward, right).normalize();
    const frame = new THREE.Matrix4().makeBasis(right, up, outward);

    camera.position.copy(pivot).addScaledVector(outward, radius);
    camera.quaternion.setFromRotationMatrix(frame);
    camera.rotateZ(roll);
    camera.up.set(0, 1, 0);
    // Publish one complete pose for both direct Debug and offscreen Fluoro.
    camera.updateMatrix();
    camera.updateMatrixWorld(true);
    return {yaw, pitch, roll};
}

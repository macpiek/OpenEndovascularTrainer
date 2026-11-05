import * as THREE from 'three';

// Fit only the equipment and patient, keeping the preview's viewing direction
// fixed. Mesh bounds are cheap to update as the table and gantry move, and much
// tighter than one world-axis box around the entire scene.
export function createCArmPreviewFramer(camera, objects, padding = 0.08) {
    const point = new THREE.Vector3();
    const viewRotation = new THREE.Matrix4();
    const meshToView = new THREE.Matrix4();

    return function fit() {
        const halfHeight = Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2) /
            camera.zoom * (1 - padding);
        const halfWidth = halfHeight * camera.aspect;
        let left = -Infinity, right = Infinity;
        let bottom = -Infinity, top = Infinity;
        let minZ = Infinity, maxZ = -Infinity;
        // The preview camera is a direct child of the scene. Rotate world
        // points into its optical frame without applying a camera translation.
        viewRotation.makeRotationFromQuaternion(camera.quaternion).invert();
        for (const root of objects) {
            root.updateWorldMatrix(true, true);
            root.traverseVisible(object => {
                if (!object.isMesh || !object.geometry) return;
                const geometry = object.geometry;
                if (!geometry.boundingBox) geometry.computeBoundingBox();
                const bounds = geometry.boundingBox;
                if (!bounds || bounds.isEmpty()) return;
                meshToView.multiplyMatrices(viewRotation, object.matrixWorld);
                for (let corner = 0; corner < 8; corner++) {
                    point.set(
                        corner & 1 ? bounds.max.x : bounds.min.x,
                        corner & 2 ? bounds.max.y : bounds.min.y,
                        corner & 4 ? bounds.max.z : bounds.min.z
                    ).applyMatrix4(meshToView);
                    left = Math.max(left, point.x + halfWidth * point.z);
                    right = Math.min(right, point.x - halfWidth * point.z);
                    bottom = Math.max(bottom, point.y + halfHeight * point.z);
                    top = Math.min(top, point.y - halfHeight * point.z);
                    minZ = Math.min(minZ, point.z);
                    maxZ = Math.max(maxZ, point.z);
                }
            });
        }
        if (!Number.isFinite(minZ)) return false;

        // Perspective containment: |x - cameraX| <= halfWidth * (cameraZ - z).
        // Intersect the bounds for every corner to find the nearest camera
        // distance that fits both dimensions, leaving a small edge margin.
        const cameraZ = Math.max(
            (left - right) / (2 * halfWidth),
            (bottom - top) / (2 * halfHeight),
            maxZ + 1
        );
        camera.position.set((left + right) / 2, (bottom + top) / 2, cameraZ)
            .applyQuaternion(camera.quaternion);
        camera.near = Math.max(0.1, (cameraZ - maxZ) * 0.5);
        camera.far = (cameraZ - minZ) * 1.1;
        camera.updateProjectionMatrix();
        camera.updateMatrixWorld(true);
        return true;
    };
}

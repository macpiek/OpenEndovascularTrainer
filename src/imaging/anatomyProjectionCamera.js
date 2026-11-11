import { Box3, Vector3 } from 'three';

/** Keep the complete anatomy in the projection when the table moves.
 *
 * Detector distance calibrates magnification and the 2D aperture. It is not a
 * clipping surface through the patient: the simulator's table controls do not
 * implement physical detector/patient collision. Use a separate depth range
 * for the anatomy passes, preserving the source pose, FOV and zoom exactly.
 */
export function createAnatomyProjectionCamera(sourceCamera, anatomy) {
    const camera = sourceCamera.clone();
    const bounds = new Box3();
    const corner = new Vector3();

    function update() {
        sourceCamera.updateWorldMatrix(true, false);
        camera.copy(sourceCamera, false);
        // The source can be parented. The projection camera is rendered alone,
        // so retain its world pose rather than the source's local transform.
        sourceCamera.matrixWorld.decompose(camera.position, camera.quaternion, camera.scale);
        camera.matrixAutoUpdate = false;
        camera.matrix.copy(sourceCamera.matrixWorld);
        camera.updateMatrixWorld(true);
        anatomy.updateWorldMatrix(true, true);
        bounds.setFromObject(anatomy);
        let far = sourceCamera.far;
        if (!bounds.isEmpty()) {
            for (let index = 0; index < 8; index++) {
                corner.set(
                    index & 1 ? bounds.max.x : bounds.min.x,
                    index & 2 ? bounds.max.y : bounds.min.y,
                    index & 4 ? bounds.max.z : bounds.min.z
                ).applyMatrix4(camera.matrixWorldInverse);
                far = Math.max(far, -corner.z + 1);
            }
        }
        camera.far = far;
        camera.updateProjectionMatrix();
        return camera;
    }

    return {camera, update};
}

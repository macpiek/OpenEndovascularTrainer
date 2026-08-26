import * as THREE from 'three';

export const AORTA_MODEL_URL = new URL('../res/Aorta_plain.stl', import.meta.url).href;
export const AORTA_COLLISION_URL = new URL('../res/Aorta_plain.collision.bin', import.meta.url).href;
export const AORTA_MODEL_SCALE = 1.3;
export const AORTA_MODEL_Y_OFFSET = 40;
export const AORTA_TRANSFORM_VERSION = 1;

export function aortaTargetFromVessel(vessel) {
    const ys = [];
    for (const segment of vessel?.segments || []) {
        if (segment.isSheath) continue;
        ys.push(segment.start.y, segment.end.y);
    }

    const top = Math.max(...ys, 0) + 15;
    const bottom = Math.min(...ys, -420) - 15;
    return {
        center: new THREE.Vector3(
            vessel?.branchPoint?.x || 0,
            (top + bottom) * 0.5 + AORTA_MODEL_Y_OFFSET,
            vessel?.branchPoint?.z || 0
        ),
        length: Math.max(300, top - bottom)
    };
}

export function transformAortaGeometry(geometry, vessel) {
    geometry.computeBoundingBox();
    const sourceBox = geometry.boundingBox.clone();
    const sourceSize = sourceBox.getSize(new THREE.Vector3());
    const sourceCenter = sourceBox.getCenter(new THREE.Vector3());
    const target = aortaTargetFromVessel(vessel);
    const scale = target.length * AORTA_MODEL_SCALE / Math.max(1e-6, sourceSize.z);

    geometry.translate(-sourceCenter.x, -sourceCenter.y, -sourceCenter.z);
    geometry.rotateX(-Math.PI / 2);
    geometry.scale(scale, scale, scale);
    geometry.translate(target.center.x, target.center.y, target.center.z);
    geometry.computeBoundingBox();

    return {
        version: AORTA_TRANSFORM_VERSION,
        rotationX: -Math.PI / 2,
        scale,
        sourceCenter: sourceCenter.toArray(),
        sourceSize: sourceSize.toArray(),
        targetCenter: target.center.toArray(),
        targetLength: target.length
    };
}

export function transformMetadataForPreprocess(transform) {
    return {
        ...transform,
        sourceCenter: new THREE.Vector3(...transform.sourceCenter),
        sourceSize: new THREE.Vector3(...transform.sourceSize),
        targetCenter: new THREE.Vector3(...transform.targetCenter)
    };
}

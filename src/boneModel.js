import * as THREE from 'three';
import { loadWorkerAsset } from './loadWorkerAsset.js';
import { unpackSkeletonGeometry } from './skeletonGeometry.js';

export function createBoneModel({ onLoaded, onError, signal } = {}) {
    const material = new THREE.MeshBasicMaterial({
        color: 0xffffff,
        transparent: true,
        opacity: 0.42,
        depthWrite: false,
        depthTest: false,
        blending: THREE.AdditiveBlending,
        side: THREE.DoubleSide
    });

    const group = new THREE.Group();
    const ready = loadWorkerAsset(
        () => new Worker(new URL('./workers/skeleton.worker.js', import.meta.url), { type: 'module' }),
        { url: new URL('../res/skeleton.obj', import.meta.url).href },
        signal
    ).then(({ meshes }) => {
            if (signal?.aborted) return;
            const obj = unpackSkeletonGeometry(meshes, material);

            const box = new THREE.Box3().setFromObject(obj);
            const center = box.getCenter(new THREE.Vector3());
            obj.position.sub(center);

            obj.rotation.z = -Math.PI / 3;
            obj.scale.multiplyScalar(9);
            obj.position.x -= 1760;
            obj.position.y -= 300;
            obj.position.z -= 70;

            group.add(obj);
            if (typeof onLoaded === 'function') onLoaded({ group, object: obj, material });
        }).catch(error => {
            if (signal?.aborted) return;
            console.warn('Failed to load skeleton OBJ model', error);
            if (typeof onError === 'function') onError(error);
        });

    return { group, material, ready };
}

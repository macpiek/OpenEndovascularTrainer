import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';

export function createBoneModel() {
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
    const loader = new OBJLoader();
    loader.load('res/skeleton.obj', (obj) => {
        obj.traverse(child => {
            if (child.isMesh) {
                child.material = material;
            }
        });

        const box = new THREE.Box3().setFromObject(obj);
        const center = box.getCenter(new THREE.Vector3());
        obj.position.sub(center);

        obj.rotation.z = -Math.PI / 3;
        obj.scale.multiplyScalar(9);
        obj.position.x -= 1760;
        obj.position.y -= 300;
        obj.position.z -= 70;

        group.add(obj);
    });

    return { group, material };
}

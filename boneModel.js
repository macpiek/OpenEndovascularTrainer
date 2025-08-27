import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';

export function createBoneModel() {
    // Use additive blending so bones brighten underlying geometry without occluding it
    const material = new THREE.MeshStandardMaterial({
        color: 0xffffff,
        transparent: true,
        opacity: 0.5, // reduce brightness so bones are less dominant
        depthWrite: false,
        depthTest: false, // rely on render order so vessels draw on top
        blending: THREE.AdditiveBlending
    });

    const group = new THREE.Group();
    const loader = new OBJLoader();
    loader.load('skeleton.obj', (obj) => {
        // Apply material to all meshes in the loaded model
        obj.traverse(child => {
            if (child.isMesh) {
                child.material = material;
            }
        });

        // Center the model so positioning behaves like the generated bones
        const box = new THREE.Box3().setFromObject(obj);
        const center = box.getCenter(new THREE.Vector3());
        obj.position.sub(center);

        group.add(obj);
    });

    return group;
}


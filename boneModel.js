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

    // Load skeleton geometry from external OBJ file
    const loader = new OBJLoader();
    loader.load(new URL('./skeleton.obj', import.meta.url).href, obj => {
        obj.traverse(child => {
            if (child.isMesh) {
                child.material = material;
            }
        });
        group.add(obj);
    });

    return group;
}


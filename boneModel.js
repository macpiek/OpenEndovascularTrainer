import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';

export function createBoneModel() {
    const material = new THREE.ShaderMaterial({
        transparent: true,
        depthWrite: false,

        vertexShader: `
            void main() {
                gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
            }
        `,
        fragmentShader: `
            void main() {
                // Render bones as semi-transparent white geometry
                gl_FragColor = vec4(1.0, 1.0, 1.0, 0.1);
            }
        `
    });

    const group = new THREE.Group();
    const loader = new OBJLoader();
    loader.load('skeleton.obj', (obj) => {
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


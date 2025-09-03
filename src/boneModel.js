import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';

export function createBoneModel() {
    const material = new THREE.ShaderMaterial({
        transparent: true,
        depthWrite: false,

        vertexShader: `
            varying vec3 vNormal;
            void main() {
                vNormal = normalize(normalMatrix * normal);
                gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
            }
        `,
        fragmentShader: `
            varying vec3 vNormal;
            void main() {
                // Fade bone edges by reducing opacity near the silhouette
                float edgeFactor = abs(vNormal.z);
                gl_FragColor = vec4(1.0, 1.0, 1.0, 0.15 * edgeFactor);
            }
        `
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


import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';

export function createBoneModel() {
    const material = new THREE.ShaderMaterial({
        uniforms: {
            thicknessMap: { value: null },
            muBone: { value: 4.0 },
            resolution: { value: new THREE.Vector2(1, 1) }
        },
        transparent: true,
        depthWrite: false,
        depthTest: false,
        blending: THREE.NoBlending,
        vertexShader: `
            void main() {
                gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
            }
        `,
        fragmentShader: `
            uniform sampler2D thicknessMap;
            uniform float muBone;
            uniform vec2 resolution;
            void main() {
                vec2 uv = gl_FragCoord.xy / resolution;
                float d = texture2D(thicknessMap, uv).r;
                float absorb = 1.0 - exp(-muBone * d);
                gl_FragColor = vec4(vec3(absorb), 1.0);
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


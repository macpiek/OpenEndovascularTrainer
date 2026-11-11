import * as THREE from 'three';

export const BONE_DEPTH_SCALE = 0.00072;

/** Signed boundary integral of bone thickness within the projection depth range. */
export function createBoneProjectionMaterial() {
    const material = new THREE.ShaderMaterial({
        defines: {BONE_DEPTH_SCALE},
        uniforms: {rayNear: {value: 0.1}, rayFar: {value: 1600}},
        transparent: true,
        forceSinglePass: true,
        blending: THREE.CustomBlending,
        blendEquation: THREE.AddEquation,
        blendSrc: THREE.OneFactor,
        blendDst: THREE.OneFactor,
        blendEquationAlpha: THREE.AddEquation,
        blendSrcAlpha: THREE.OneFactor,
        blendDstAlpha: THREE.OneFactor,
        side: THREE.DoubleSide,
        depthTest: false,
        depthWrite: false,
        vertexShader: `
            varying vec3 vViewNormal;
            varying vec3 vViewPosition;
            varying vec3 vWorldPosition;
            void main() {
                vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
                vViewNormal = normalize(normalMatrix * normal);
                vViewPosition = mvPosition.xyz;
                vec4 worldPosition = modelMatrix * vec4(position, 1.0);
                vWorldPosition = worldPosition.xyz;
                gl_Position = projectionMatrix * mvPosition;
                // Retain exit faces beyond the depth limit for the boundary sum.
                // XY/W is unchanged: this never enlarges the detector field.
                gl_Position.z = clamp(gl_Position.z, -gl_Position.w, gl_Position.w);
            }
        `,
        fragmentShader: `
            uniform float rayNear;
            uniform float rayFar;
            varying vec3 vViewNormal;
            varying vec3 vViewPosition;
            varying vec3 vWorldPosition;

            float hash(vec3 p) {
                return fract(sin(dot(p, vec3(17.13, 47.71, 91.37))) * 43758.5453);
            }

            float valueNoise(vec3 p) {
                vec3 i = floor(p);
                vec3 f = fract(p);
                f = f * f * (3.0 - 2.0 * f);
                float n000 = hash(i + vec3(0.0, 0.0, 0.0));
                float n100 = hash(i + vec3(1.0, 0.0, 0.0));
                float n010 = hash(i + vec3(0.0, 1.0, 0.0));
                float n110 = hash(i + vec3(1.0, 1.0, 0.0));
                float n001 = hash(i + vec3(0.0, 0.0, 1.0));
                float n101 = hash(i + vec3(1.0, 0.0, 1.0));
                float n011 = hash(i + vec3(0.0, 1.0, 1.0));
                float n111 = hash(i + vec3(1.0, 1.0, 1.0));
                float nx00 = mix(n000, n100, f.x);
                float nx10 = mix(n010, n110, f.x);
                float nx01 = mix(n001, n101, f.x);
                float nx11 = mix(n011, n111, f.x);
                float nxy0 = mix(nx00, nx10, f.y);
                float nxy1 = mix(nx01, nx11, f.y);
                return mix(nxy0, nxy1, f.z);
            }

            void main() {
                vec3 normal = normalize(vViewNormal);
                vec3 rayDir = isOrthographic ? vec3(0., 0., 1.) : normalize(-vViewPosition);
                float incidence = clamp(abs(dot(normal, rayDir)), 0.12, 1.0);
                float anglePath = clamp(pow(1.0 / incidence, 0.82), 1.0, 4.2);

                vec3 p = vWorldPosition * 0.035;
                float coarse = valueNoise(p);
                float fine = valueNoise(p * 2.8 + vec3(4.0, 11.0, 2.0));
                float trabeculae = smoothstep(0.42, 0.92, coarse * 0.62 + fine * 0.38);
                float marrowMottle = mix(0.72, 1.08, valueNoise(p * 1.35 + vec3(2.0, 7.0, 13.0)));

                // Integrate within the anatomy camera's depth range.
                // A clipped exit still closes the volume at rayFar. Clamping
                // boundary depths (rather than dropping faces) also makes a bone
                // wholly beyond that range cancel to zero. The anatomy camera
                // fits this range to the full skeleton when the table moves.
                float viewDepth = -vViewPosition.z;
                float rayScale = isOrthographic ? 1. : length(vViewPosition) / max(1e-6, viewDepth);
                float encodedDepth = clamp(viewDepth - rayNear, 0., rayFar - rayNear) * rayScale * BONE_DEPTH_SCALE;
                float physicalWall = step(rayNear, viewDepth) * step(viewDepth, rayFar);
                float entryDepth = gl_FrontFacing ? encodedDepth : 0.0;
                float exitDepth = gl_FrontFacing ? 0.0 : encodedDepth;
                float grazingCortex = smoothstep(1.35, 3.8, anglePath);
                float corticalPath = anglePath * 0.0048 + grazingCortex * 0.036 + trabeculae * marrowMottle * 0.0009;
                float trabecularTexture = trabeculae * marrowMottle * 0.026;

                gl_FragColor = vec4(entryDepth, exitDepth, corticalPath * physicalWall, trabecularTexture * physicalWall);
            }
        `
    });
    material.onBeforeRender = (_renderer, _scene, camera) => {
        material.uniforms.rayNear.value = camera.near;
        material.uniforms.rayFar.value = camera.far;
    };
    return material;
}

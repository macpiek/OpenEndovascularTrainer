import { ShaderMaterial, WebGLRenderTarget } from 'three';
import { DETECTOR_APERTURE_HALF_EXTENT } from './collimatorGeometry.js';
import {
    GUIDEWIRE_TYPE_GLIDEWIRE,
    GUIDEWIRE_TYPE_GLIDEWIRE_ANGLED
} from '../physics/guidewireMaterialProfile.js';

// Thin wires need at least two samples per CSS pixel, including the aperture
// crop used when presenting the detector. Keep the expensive anatomy passes
// at their own resolution and bound this small device-only pass on large screens.
export function metalProjectionSize(displaySize, maxTextureSize = 2048) {
    return Math.max(1, Math.min(2048, maxTextureSize,
        Math.ceil(displaySize * 2 / DETECTOR_APERTURE_HALF_EXTENT)));
}

export function createMetalProjectionTarget(displaySize, capabilities) {
    const size = metalProjectionSize(displaySize, capabilities.maxTextureSize);
    return new WebGLRenderTarget(size, size, {
        samples: capabilities.isWebGL2 ? 4 : 0,
        // Wire, graft and per-object projection materials all disable depth
        // testing/writes. Retain every color/MSAA sample, but do not allocate
        // an unused multisampled depth attachment (64 MiB at 2048², 4× MSAA).
        depthBuffer: false
    });
}

export function createGuidewireProjectionMaterial(type = GUIDEWIRE_TYPE_GLIDEWIRE) {
    const glidewire = type === GUIDEWIRE_TYPE_GLIDEWIRE || type === GUIDEWIRE_TYPE_GLIDEWIRE_ANGLED;
    // Visual attenuation presets, not measured manufacturer transmission data.
    // metalAt decodes the mask as 5.25 * signal^1.18. Scale optical depth before
    // that conversion, so anatomy remains visible through Glidewire and the
    // detector still combines its absorption with the tissue underneath.
    const opticalDepth = glidewire ? 0.8 : 5.25;
    return new ShaderMaterial({
        uniforms: { signalScale: { value: Math.pow(opticalDepth / 5.25, 1 / 1.18) } },
        vertexShader: `
            varying vec3 vWireViewNormal;
            void main() {
                mat4 instanceTransform = mat4(1.0);
                #ifdef USE_INSTANCING
                    instanceTransform = instanceMatrix;
                #endif
                vec4 mvPosition = modelViewMatrix * instanceTransform * vec4(position, 1.0);
                vWireViewNormal = normalMatrix * mat3(instanceTransform) * normal;
                gl_Position = projectionMatrix * mvPosition;
            }
        `,
        fragmentShader: `
            uniform float signalScale;
            varying vec3 vWireViewNormal;
            void main() {
                // Evaluate the cylindrical profile per fragment. Interpolating
                // a nonlinear intensity per vertex exposes the tube's facets.
                vec3 viewNormal = normalize(vWireViewNormal);
                float radial = pow(clamp(abs(viewNormal.z), 0.0, 1.0), 0.75);
                float profile = mix(0.06, 1.0, smoothstep(0.08, 0.94, radial));
                gl_FragColor = vec4(vec3(profile * signalScale), 1.0);
            }
        `,
        depthTest: false,
        depthWrite: false,
        toneMapped: false
    });
}

import { HalfFloatType, WebGLRenderTarget } from 'three';
import { DETECTOR_APERTURE_HALF_EXTENT } from './collimatorGeometry.js';

// Resolve iodine at two samples per displayed CSS pixel, including the crop
// to the detector aperture. Limit allocation independently of anatomy passes.
export function contrastProjectionSize(displaySize, maxTextureSize = 2048) {
    return Math.max(1, Math.min(2048, maxTextureSize,
        Math.ceil(displaySize * 2 / DETECTOR_APERTURE_HALF_EXTENT)));
}

export function createContrastProjectionTarget(displaySize, capabilities) {
    const size = contrastProjectionSize(displaySize, capabilities.maxTextureSize);
    return new WebGLRenderTarget(size, size, {
        type: HalfFloatType,
        samples: capabilities.isWebGL2 ? Math.min(4, capabilities.maxSamples) : 0,
        // Iodine integrates along the ray; all contrast materials add their
        // signal without depth testing. No multisampled depth storage is needed.
        depthBuffer: false
    });
}

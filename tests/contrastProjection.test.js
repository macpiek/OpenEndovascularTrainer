import test from 'node:test';
import assert from 'node:assert/strict';
import { HalfFloatType } from 'three';
import { contrastProjectionSize, createContrastProjectionTarget } from '../src/imaging/contrastProjection.js';
import { DETECTOR_APERTURE_HALF_EXTENT } from '../src/imaging/collimatorGeometry.js';

test('iodine target retains independent supersampled rays without redundant MSAA storage', () => {
    const capabilities = { isWebGL2: true, maxSamples: 4, maxTextureSize: 4096 };
    for (const displaySize of [384, 684, 1095]) {
        const target = createContrastProjectionTarget(displaySize, capabilities);
        const expectedSize = Math.min(2048, Math.ceil(2 * displaySize / DETECTOR_APERTURE_HALF_EXTENT));
        assert.equal(target.width, expectedSize);
        assert.equal(target.height, expectedSize);
        assert.equal(target.samples, 0);
        assert.equal(target.texture.type, HalfFloatType);
        assert.equal(target.depthBuffer, false);
        target.dispose();
    }
});

test('iodine projection respects the device texture limit', () => {
    assert.equal(contrastProjectionSize(2000, 1024), 1024);
    assert.equal(contrastProjectionSize(2000, 8192), 2048);
    assert.equal(contrastProjectionSize(0, 1024), 1);
});

import test from 'node:test';
import assert from 'node:assert/strict';
import {iodineOpticalDepth, dsaLumaFromOpticalDepth, DSA_NEUTRAL_LUMA,
    DSA_DARK_FLOOR_LUMA, DSA_BRIGHT_CEILING_LUMA, detectorOpticalNoiseSigma,
    measuredDetectorOpticalDepth} from '../src/imaging/dsaDisplayParameters.js';

test('iodine attenuation is proportional to concentration and path length before transmission', () => {
    const depth = iodineOpticalDepth(.14 * 5 * .2);
    assert.equal(iodineOpticalDepth(.14 * 5 * .4), 2 * depth);
    assert.equal(iodineOpticalDepth(.14 * 10 * .2), 2 * depth);
    assert.ok(Math.abs(Math.exp(-2 * depth) - Math.exp(-depth) ** 2) < 1e-12);
    assert.equal(iodineOpticalDepth(.1, 0), 0);
    assert.equal(iodineOpticalDepth(.1, 5, 0), 0);
});

test('logarithmic subtraction cancels background and preserves equal iodine through different anatomy', () => {
    const addedDepth = iodineOpticalDepth(.14 * 5 * .1);
    const expected = dsaLumaFromOpticalDepth(addedDepth);
    for (const background of [.1, .5, 1.5, 3]) {
        const maskIntensity = Math.exp(-background), liveIntensity = Math.exp(-(background + addedDepth));
        assert.ok(Math.abs(dsaLumaFromOpticalDepth(Math.log(maskIntensity / liveIntensity)) - expected) < 1e-12);
        assert.equal(dsaLumaFromOpticalDepth(background - background), DSA_NEUTRAL_LUMA);
    }
});

test('diluted and dense bifurcation contrast remain distinct in an eight-bit cine', () => {
    for (const radius of [2, 5, 8]) {
        const levels = [.025, .05, .1, .25, .5, 1].map(concentration =>
            Math.round(255 * dsaLumaFromOpticalDepth(iodineOpticalDepth(.14 * radius * concentration))));
        for (let i = 1; i < levels.length; i++)
            assert.ok(levels[i - 1] - levels[i] >= 5, `radius ${radius}: ${levels}`);
        assert.ok(levels.at(-1) > 255 * DSA_DARK_FLOOR_LUMA + 5);
    }
});

test('the DSA shoulder is monotone without black clipping and retains positive motion residuals', () => {
    let previous = DSA_NEUTRAL_LUMA;
    for (let i = 1; i <= 1000; i++) {
        const positive = dsaLumaFromOpticalDepth(i / 100);
        const negative = dsaLumaFromOpticalDepth(-i / 100);
        assert.ok(positive < previous && positive > DSA_DARK_FLOOR_LUMA);
        assert.ok(negative > DSA_NEUTRAL_LUMA && negative < DSA_BRIGHT_CEILING_LUMA);
        previous = positive;
    }
    assert.equal(dsaLumaFromOpticalDepth(2, 0), DSA_NEUTRAL_LUMA);
});

test('optical acquisition noise follows dose and transmission without overwhelming dilution', () => {
    for (const attenuation of [0, .2, 1.4, 5]) for (const sample of [-3, 0, 3])
        assert.equal(measuredDetectorOpticalDepth(attenuation, sample, {noiseLevel: 0}), attenuation);
    const normal = detectorOpticalNoiseSigma(1, {pulseRate: 30});
    assert.ok(Math.abs(detectorOpticalNoiseSigma(1, {pulseRate: 15}) / normal - Math.SQRT2) < 1e-12);
    assert.ok(detectorOpticalNoiseSigma(2) > detectorOpticalNoiseSigma(1));
    assert.ok(detectorOpticalNoiseSigma(1, {zoomNoiseScale: 2}) > detectorOpticalNoiseSigma(1));
    for (const sample of [-3, 3])
        assert.ok(Number.isFinite(measuredDetectorOpticalDepth(100, sample, {noiseLevel: 1, pulseRate: 4, zoomNoiseScale: 10})));
    // Even through the darker fixture background, a 5% versus 10% mixture in
    // a 10 mm vessel is separated by several acquisition noise sigmas.
    const c05 = iodineOpticalDepth(.14 * 5 * .05), c10 = 2 * c05;
    assert.ok(c10 - c05 > 4 * detectorOpticalNoiseSigma(1.4 + c10));
});

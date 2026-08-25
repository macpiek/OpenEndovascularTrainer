import assert from 'node:assert/strict';
import {
    DETECTOR_ZOOM_MODES,
    applyDetectorZoomTechnique,
    getDetectorZoomMode,
    nextDetectorZoomMode
} from '../src/imaging/detectorZoomModes.js';
import {
    DEFAULT_FLUORO_PULSE_WIDTH_MS,
    estimateFluoroDoseRateMgyPerSecond
} from '../src/imaging/xrayDoseRate.js';

assert.deepEqual(
    DETECTOR_ZOOM_MODES.map(mode => mode.id),
    ['none', 'medium', 'maximum']
);
assert.equal(getDetectorZoomMode('unknown').id, 'none');
assert.equal(nextDetectorZoomMode('none').id, 'medium');
assert.equal(nextDetectorZoomMode('medium').id, 'maximum');
assert.equal(nextDetectorZoomMode('maximum').id, 'none');
assert.deepEqual(
    DETECTOR_ZOOM_MODES.map(mode => mode.inputFieldCm),
    [21, 15, 10]
);
assert.deepEqual(
    DETECTOR_ZOOM_MODES.map(mode => mode.zoomFactor),
    [1, 1.4, 2.1]
);

for (let index = 1; index < DETECTOR_ZOOM_MODES.length; index++) {
    const previous = DETECTOR_ZOOM_MODES[index - 1];
    const current = DETECTOR_ZOOM_MODES[index];
    assert.ok(current.zoomFactor > previous.zoomFactor);
    assert.ok(current.fieldOfViewPercent < previous.fieldOfViewPercent);
    assert.ok(
        Math.abs(current.fieldOfViewPercent - 100 / current.zoomFactor) < 1
    );
    assert.ok(
        current.referenceDoseRateMultiplier >
            previous.referenceDoseRateMultiplier
    );
    assert.ok(current.quantumNoiseScale < previous.quantumNoiseScale);
    assert.ok(current.scatterScale < previous.scatterScale);
}

const baseTechnique = {
    kv: 77,
    ma: 3.8,
    pulseWidthMs: DEFAULT_FLUORO_PULSE_WIDTH_MS
};
const doseRates = DETECTOR_ZOOM_MODES.map(mode => {
    const technique = applyDetectorZoomTechnique(baseTechnique, mode);
    return estimateFluoroDoseRateMgyPerSecond({
        ...technique,
        pulseRate: 15
    });
});
assert.deepEqual(
    applyDetectorZoomTechnique(baseTechnique, 'none'),
    baseTechnique
);
for (let index = 0; index < DETECTOR_ZOOM_MODES.length; index++) {
    const actualMultiplier = doseRates[index] / doseRates[0];
    assert.ok(
        Math.abs(
            actualMultiplier -
            DETECTOR_ZOOM_MODES[index].referenceDoseRateMultiplier
        ) < 0.02,
        `${DETECTOR_ZOOM_MODES[index].id} dose multiplier was ${actualMultiplier}`
    );
}

console.log('detector zoom mode tests passed');

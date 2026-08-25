import assert from 'node:assert/strict';
import {
    estimateFluoroDoseRateMgyPerSecond
} from '../src/imaging/xrayDoseRate.js';

const normalFluoro = estimateFluoroDoseRateMgyPerSecond({
    kv: 77,
    ma: 3.8,
    pulseRate: 15
});
assert.ok(normalFluoro > 0.15 && normalFluoro < 0.35);
assert.equal(
    estimateFluoroDoseRateMgyPerSecond({
        kv: 77,
        ma: 3.8,
        pulseRate: 15,
        emitting: false
    }),
    0
);
assert.ok(
    estimateFluoroDoseRateMgyPerSecond({
        kv: 90,
        ma: 3.8,
        pulseRate: 15
    }) > normalFluoro
);
assert.ok(
    estimateFluoroDoseRateMgyPerSecond({
        kv: 77,
        ma: 3.8,
        pulseRate: 30
    }) > normalFluoro * 1.9
);

console.log('X-ray dose-rate tests passed');

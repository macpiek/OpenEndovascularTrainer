import assert from 'node:assert/strict';
import {
    BROWSER_BENCHMARK_DEFAULT_DURATION_MS,
    BROWSER_BENCHMARK_SCENARIO_CYCLE_MS,
    browserBenchmarkCatheterType,
    createBrowserBenchmarkCommands,
    sampleBrowserBenchmarkCommands
} from '../src/benchmark/browserBenchmarkScenario.js';

assert.equal(BROWSER_BENCHMARK_DEFAULT_DURATION_MS, 600000);
assert.equal(BROWSER_BENCHMARK_DEFAULT_DURATION_MS / BROWSER_BENCHMARK_SCENARIO_CYCLE_MS, 25 / 3);

const commands = createBrowserBenchmarkCommands();
const sample = elapsedMs => ({ ...sampleBrowserBenchmarkCommands(elapsedMs, commands) });

assert.deepEqual(sample(0), {
    guidewireAdvance: 1,
    catheterAdvance: 0,
    catheterRotation: 0,
    catheterType: 'pigtail'
});
assert.deepEqual(sample(15000), {
    guidewireAdvance: 0,
    catheterAdvance: 1,
    catheterRotation: 0,
    catheterType: 'pigtail'
});
assert.equal(sample(25000).catheterRotation, 1);
assert.equal(sample(27500).catheterRotation, -1);
assert.deepEqual(sample(35000), {
    guidewireAdvance: 0,
    catheterAdvance: -1,
    catheterRotation: -1,
    catheterType: 'pigtail'
});
assert.equal(sample(37500).catheterRotation, 1);
assert.deepEqual(sample(52000), {
    guidewireAdvance: -1,
    catheterAdvance: 0,
    catheterRotation: 0,
    catheterType: 'pigtail'
});
assert.deepEqual(sample(67000), {
    guidewireAdvance: 0,
    catheterAdvance: 0,
    catheterRotation: 0,
    catheterType: 'pigtail'
});
assert.equal(browserBenchmarkCatheterType(72000), 'berenstein');
assert.equal(sampleBrowserBenchmarkCommands(72000, commands), commands, 'the hot sampler should reuse its output');
assert.equal(commands.guidewireAdvance, 1);
assert.equal(commands.catheterType, 'berenstein');
assert.equal(browserBenchmarkCatheterType(144000), 'pigtail');

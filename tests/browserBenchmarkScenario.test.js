import assert from 'node:assert/strict';
import {
    BROWSER_BENCHMARK_DEFAULT_DURATION_MS,
    BROWSER_BENCHMARK_SCENARIO_CYCLE_MS,
    GUIDEWIRE_BROWSER_BENCHMARK_CYCLE_MS,
    browserBenchmarkCatheterType,
    createBrowserBenchmarkCommands,
    sampleBrowserBenchmarkCommands,
    sampleGuidewireBrowserBenchmarkCommands
} from '../src/benchmark/browserBenchmarkScenario.js';
import {
    ARCH_BOLUS_CATHETER_TARGET_MM,
    ARCH_BOLUS_GUIDEWIRE_TARGET_MM,
    AORTA_SETUP_CATHETER_TARGET_MM,
    AORTA_SETUP_GUIDEWIRE_TARGET_MM,
    createCatheterAortaSetupState,
    ILIAC_BUG_CATHETER_TARGET_MM,
    ILIAC_BUG_GUIDEWIRE_TARGET_MM,
    RETROGRADE_GAP_CATHETER_TARGET_MM,
    RETROGRADE_GAP_GUIDEWIRE_TARGET_MM,
    sampleCatheterAortaSetup,
    startCatheterAortaSetup
} from '../src/benchmark/catheterAortaSetup.js';

assert.equal(ARCH_BOLUS_GUIDEWIRE_TARGET_MM, 559);
assert.equal(ARCH_BOLUS_CATHETER_TARGET_MM, 583);
assert.equal(RETROGRADE_GAP_GUIDEWIRE_TARGET_MM, 77);
assert.equal(RETROGRADE_GAP_CATHETER_TARGET_MM, 133);

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

const guidewireCommands = createBrowserBenchmarkCommands();
const sampleGuidewire = elapsedMs => ({
    ...sampleGuidewireBrowserBenchmarkCommands(elapsedMs, guidewireCommands)
});
assert.equal(GUIDEWIRE_BROWSER_BENCHMARK_CYCLE_MS, 28000);
assert.deepEqual(sampleGuidewire(0), {
    guidewireAdvance: 1,
    catheterAdvance: 0,
    catheterRotation: 0,
    catheterType: 'berenstein'
});
assert.deepEqual(sampleGuidewire(14000), {
    guidewireAdvance: -1,
    catheterAdvance: 0,
    catheterRotation: 0,
    catheterType: 'berenstein'
});
assert.equal(sampleGuidewire(12000).guidewireAdvance, 0,
    'guidewire smoke cycle must exercise release after insertion');
assert.equal(sampleGuidewire(13999).guidewireAdvance, 0);
assert.equal(sampleGuidewire(25999).guidewireAdvance, -1);
assert.equal(sampleGuidewire(26000).guidewireAdvance, 0,
    'guidewire smoke cycle must exercise release after withdrawal');
assert.equal(sampleGuidewireBrowserBenchmarkCommands(28000, guidewireCommands),
    guidewireCommands, 'the guidewire-only hot sampler should reuse its output');
assert.equal(guidewireCommands.guidewireAdvance, 1);

const aortaSetup = createCatheterAortaSetupState();
startCatheterAortaSetup(aortaSetup);
const aortaSetupCommands = {};
const sampleAortaSetup = (guidewireProgressMm, catheterProgressMm) => ({
    ...sampleCatheterAortaSetup(
        aortaSetup,
        { guidewireProgressMm, catheterProgressMm },
        aortaSetupCommands
    )
});

assert.deepEqual(sampleAortaSetup(0, 0), {
    guidewireAdvance: 1,
    catheterAdvance: 0,
    catheterRotation: 0,
    catheterType: 'berenstein'
});
assert.equal(aortaSetup.phase, 'guidewire');
assert.deepEqual(sampleAortaSetup(AORTA_SETUP_GUIDEWIRE_TARGET_MM, 0), {
    guidewireAdvance: 0,
    catheterAdvance: 1,
    catheterRotation: 0,
    catheterType: 'berenstein'
});
assert.equal(aortaSetup.phase, 'catheter');
assert.deepEqual(
    sampleAortaSetup(
        AORTA_SETUP_GUIDEWIRE_TARGET_MM,
        AORTA_SETUP_CATHETER_TARGET_MM
    ),
    {
        guidewireAdvance: 0,
        catheterAdvance: 0,
        catheterRotation: 0,
        catheterType: 'berenstein'
    }
);
assert.equal(aortaSetup.running, false);
assert.equal(aortaSetup.phase, 'ready');
assert.equal(
    sampleCatheterAortaSetup(
        aortaSetup,
        {
            guidewireProgressMm: AORTA_SETUP_GUIDEWIRE_TARGET_MM,
            catheterProgressMm: AORTA_SETUP_CATHETER_TARGET_MM
        },
        aortaSetupCommands
    ),
    null
);
assert.ok(
    aortaSetupCommands.guidewireAdvance >= 0 && aortaSetupCommands.catheterAdvance >= 0,
    'the setup sequence must never withdraw the guidewire or catheter'
);

const iliacBugSetup = createCatheterAortaSetupState();
startCatheterAortaSetup(iliacBugSetup, {
    catheterTargetMm: ILIAC_BUG_CATHETER_TARGET_MM,
    finalGuidewireTargetMm: ILIAC_BUG_GUIDEWIRE_TARGET_MM
});
const iliacBugCommands = {};
assert.equal(sampleCatheterAortaSetup(
    iliacBugSetup,
    {
        guidewireProgressMm: AORTA_SETUP_GUIDEWIRE_TARGET_MM,
        catheterProgressMm: ILIAC_BUG_CATHETER_TARGET_MM
    },
    iliacBugCommands
).guidewireAdvance, -1);
assert.equal(iliacBugSetup.phase, 'guidewire-withdraw');
assert.deepEqual(sampleCatheterAortaSetup(
    iliacBugSetup,
    {
        guidewireProgressMm: ILIAC_BUG_GUIDEWIRE_TARGET_MM,
        catheterProgressMm: ILIAC_BUG_CATHETER_TARGET_MM
    },
    iliacBugCommands
), {
    guidewireAdvance: 0,
    catheterAdvance: 0,
    catheterRotation: 0,
    catheterType: 'berenstein'
});
assert.equal(iliacBugSetup.phase, 'ready');

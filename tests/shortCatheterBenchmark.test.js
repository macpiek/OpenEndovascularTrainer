import assert from 'node:assert/strict';
import test from 'node:test';
import {
    SHORT_CATHETER_BENCHMARK_DT as dt,
    SHORT_CATHETER_BENCHMARK_PHASES as phases,
    SHORT_CATHETER_BENCHMARK_STEPS as steps,
    DEEP_CATHETER_BENCHMARK_PHASES, DEEP_CATHETER_BENCHMARK_STEPS,
    sampleShortCatheterBenchmarkCommands
} from '../src/benchmark/shortCatheterBenchmark.js';

test('short-catheter replay executes the same exact feeds and held physical time', () => {
    const counts = new Array(phases.length).fill(0);
    const wire = counts.slice();
    const catheter = counts.slice();
    const command = {};
    let elapsedMs = 0;
    for (let i = 0; i < steps; i++) {
        // Accumulate the real runtime's floating-point clock, rather than
        // using integer multiplication that would hide boundary roundoff.
        sampleShortCatheterBenchmarkCommands(elapsedMs, command);
        elapsedMs += dt * 1000;
        const phase = command.benchmarkPhase;
        assert.ok(phase >= 0 && phase < phases.length);
        assert.ok(command.guidewireAdvance >= 0 && command.guidewireAdvance <= 1);
        assert.ok(command.catheterAdvance >= 0 && command.catheterAdvance <= 1);
        counts[phase]++;
        wire[phase] += command.guidewireAdvance * 44 * dt;
        catheter[phase] += command.catheterAdvance * 52 * dt;
    }
    assert.deepEqual(counts, phases.map(p => p.steps));
    for (let i = 0; i < phases.length; i++) {
        assert.ok(Math.abs(wire[i] - (phases[i].wireMm ?? 0)) < 1e-8);
        assert.ok(Math.abs(catheter[i] - (phases[i].catheterMm ?? 0)) < 1e-8);
    }
    sampleShortCatheterBenchmarkCommands(elapsedMs, command);
    assert.equal(command.benchmarkPhase, -1);
    assert.equal(command.guidewireAdvance, 0);
    assert.equal(command.catheterAdvance, 0);
});

test('deep replay reaches 600 mm and executes rotation in both directions without shortening held time', () => {
    const phases = DEEP_CATHETER_BENCHMARK_PHASES;
    const counts = new Array(phases.length).fill(0);
    let elapsedMs = 0, wireMm = 0, catheterMm = 0;
    const command = {};
    for (let i = 0; i < DEEP_CATHETER_BENCHMARK_STEPS; i++) {
        sampleShortCatheterBenchmarkCommands(elapsedMs, command, phases);
        elapsedMs += dt * 1000;
        const index = command.benchmarkPhase;
        counts[index]++;
        wireMm += command.guidewireAdvance * 44 * dt;
        catheterMm += command.catheterAdvance * 52 * dt;
        assert.equal(command.catheterRotation, phases[index].rotation ?? 0);
        if (counts[index] === phases[index].steps && phases[index].name.startsWith('catheter-hold-')) {
            const depthMm = Number(phases[index].name.match(/(\d+)mm$/)[1]);
            assert.ok(Math.abs(catheterMm - depthMm) < 1e-8);
        }
    }
    assert.deepEqual(counts, phases.map(p => p.steps));
    assert.ok(Math.abs(wireMm - 999.9) < 1e-8);
    assert.ok(Math.abs(catheterMm - 600) < 1e-8);
    sampleShortCatheterBenchmarkCommands(elapsedMs, command, phases);
    assert.equal(command.benchmarkPhase, -1);
    assert.equal(command.catheterRotation, 0);
});

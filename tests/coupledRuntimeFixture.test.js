import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import { createCoupledRuntimeFixture, COUPLED_RUNTIME_DEFAULTS, poseFingerprint } from './helpers/coupledRuntimeFixture.js';
import { createCoupledRebuildPhases, sampleShortCatheterBenchmarkCommands,
    poseFingerprint as browserFingerprint } from '../src/benchmark/shortCatheterBenchmark.js';
import { assessCoupledTiming } from './helpers/coupledValidationMetrics.js';

test('runtime settings and initial material state reproduce across independent instances', () => {
    const a = createCoupledRuntimeFixture(), b = createCoupledRuntimeFixture();
    try {
        assert.deepEqual(a.config, COUPLED_RUNTIME_DEFAULTS);
        assert.equal(a.catheter.pathSpacing, 5);
        assert.equal(a.catheterBody.segmentLength, 5);
        assert.equal(a.wireBody.count, 201);
        assert.equal(a.catheterBody.count, 219);
        assert.equal(a.world.fixedDt, 1 / 120);
        assert.deepEqual(a.snapshot(), b.snapshot());
        for (let i = 0; i < 12; i++) {
            const commands = { guidewireAdvance: 1, catheterRotation: i < 6 ? 1 : -1,
                guidewireRotation: i < 6 ? -1 : 1 };
            a.step(commands); b.step(commands);
        }
        assert.deepEqual(a.snapshot(), b.snapshot());
        assert.ok(Math.abs(a.catheter.rotation) < 1e-12);
        assert.ok(Math.abs(a.snapshot().wireRotation) < 1e-12);
        assert.equal(poseFingerprint(a.wireBody), browserFingerprint(a.wireBody), 'same hash, fields and byte precision as browser');
        assert.equal(a.wireBody.maxFrameDisplacement, Infinity);
    } finally { a.dispose(); b.dispose(); }
});

test('source-audited runtime defaults fail visibly if simulator constants drift', () => {
    const runtime = fs.readFileSync(new URL('../src/simulator.js', import.meta.url), 'utf8');
    for (const [name, expected] of Object.entries({
        segmentLength: 5, nodeCount: 201, catheterShaftStiffnessScale: 58.1,
        catheterTipStiffnessScale: 87, guidewireShaftStiffnessScale: 39,
        guidewireTipStiffnessScale: 30.7, guidewireRelaxationRate: 1, catheterRelaxationRate: 1
    })) {
        const match = runtime.match(new RegExp(`(?:const|let) ${name} = ([\\d.]+);`));
        assert.equal(Number(match?.[1]), expected, `audit adapter after ${name} changes`);
    }
});

test('diagnostic dt changes preserve prepared state and use one clock for feed and both rotations', () => {
    const fixture = createCoupledRuntimeFixture();
    try {
        fixture.step({ guidewireAdvance: 1 });
        const before = fixture.snapshot();
        fixture.setFixedDtForComparison(1 / 60);
        assert.deepEqual(fixture.snapshot(), before, 'changing the diagnostic clock must not reset or project mechanics');
        assert.equal(fixture.world.fixedDt, 1 / 60);
        assert.equal(fixture.config.fixedDt, 1 / 60);
        fixture.step({ guidewireAdvance: 1, catheterAdvance: 1, guidewireRotation: 1, catheterRotation: -1 });
        const after = fixture.snapshot();
        assert.ok(Math.abs(after.wireMm - before.wireMm - 44 / 60) < 1e-12);
        assert.ok(Math.abs(after.catheterMm - before.catheterMm - 52 / 60) < 1e-12);
        assert.ok(Math.abs(after.wireRotation - Math.PI * .9 / 60) < 1e-12);
        assert.ok(Math.abs(after.catheterRotation + Math.PI * .9 / 60) < 1e-12);
        for (const invalid of [0, -1, Infinity, NaN]) assert.throws(() => fixture.setFixedDtForComparison(invalid), RangeError);
        assert.equal(fixture.world.fixedDt, 1 / 60);
    } finally { fixture.dispose(); }
});

test('full protocol reaches exact depth targets and reverses both tools without skipping physical holds', () => {
    const phases = createCoupledRebuildPhases(), command = {};
    assert.equal(new Set(phases.map(phase => phase.name)).size, phases.length, 'phase limits and checkpoints require unique names');
    const counts = phases.map(() => 0);
    let elapsedMs = 0, wireMm = 0, catheterMm = 0, wireAngle = 0, catheterAngle = 0;
    const total = phases.reduce((sum, phase) => sum + phase.steps, 0);
    const visited = [];
    for (let i = 0; i < total; i++) {
        sampleShortCatheterBenchmarkCommands(elapsedMs, command, phases);
        const phase = phases[command.benchmarkPhase];
        assert.ok(phase);
        counts[command.benchmarkPhase]++;
        for (const value of [command.guidewireAdvance, command.catheterAdvance]) assert.ok(value >= -1 && value <= 1);
        wireMm += command.guidewireAdvance * 44 / 120;
        catheterMm += command.catheterAdvance * (command.catheterAdvance > 0 ? 52 : 32) / 120;
        wireAngle += command.guidewireRotation * Math.PI * 0.9 / 120;
        catheterAngle += command.catheterRotation * Math.PI * 0.9 / 120;
        assert.ok(catheterMm >= -1e-8 && catheterMm <= 1000 + 1e-8);
        assert.ok(wireMm >= -1e-8 && wireMm <= 999.9 + 1e-8);
        if (counts[command.benchmarkPhase] === phase.steps) {
            if (phase.targetCatheterMm != null) assert.ok(Math.abs(catheterMm - phase.targetCatheterMm) < 1e-8);
            if (phase.name.startsWith('catheter-hold-')) visited.push(catheterMm);
        }
        elapsedMs += 1000 / 120;
    }
    assert.deepEqual(counts, phases.map(phase => phase.steps));
    for (const depth of [100, 200, 400, 600, 1000, 0]) assert.ok(visited.some(value => Math.abs(value - depth) < 1e-8));
    for (const residual of [wireMm, catheterMm, wireAngle, catheterAngle]) assert.ok(Math.abs(residual) < 1e-8);
    sampleShortCatheterBenchmarkCommands(elapsedMs, command, phases);
    assert.equal(command.benchmarkPhase, -1);
    assert.equal(command.catheterAdvance, 0);
    assert.equal(command.guidewireRotation, 0);
});

test('catheter maximum is 1000 mm even when the wire stops at 999.9 mm', () => {
    const fixture = createCoupledRuntimeFixture();
    try {
        // Actuator limit unit test only; this is not a simulated insertion.
        fixture.catheter.advance(1, 100, 999.9);
        assert.equal(fixture.catheter.progress, 1000);
        assert.equal(fixture.catheter.guidewireInserted, 999.9);
        assert.ok(Math.abs(fixture.catheter.progress - fixture.catheter.guidewireInserted - 0.1) < 1e-9);
        fixture.catheter.advance(-1, 100, 999.9);
        assert.equal(fixture.catheter.progress, 0);
    } finally { fixture.dispose(); }
});

test('60 FPS cannot conceal backlog, dropped/omitted steps, missing samples or a slow full step', () => {
    const valid = { expectedSteps: 120, executedSteps: 120,
        fullStepTimes: Array(120).fill(3), worldStepTimes: Array(120).fill(2),
        browser: { averageFps: 60, p95FrameMs: 1000 / 60, p99FrameMs: 1000 / 60,
            acceptedSeconds: 1, startBacklogSeconds: 0, endBacklogSeconds: 0,
            peakBacklogSeconds: 1 / 60, executedSteps: 120, renderFrameExecutedSteps: 100,
            idleExecutedSteps: 20, droppedSteps: 0 } };
    assert.equal(assessCoupledTiming(valid).realTime60FpsPass, true);
    const cases = [
        { ...valid, browser: null },
        { ...valid, executedSteps: 119 },
        { ...valid, fullStepTimes: Array(119).fill(3) },
        { ...valid, fullStepTimes: Array(120).fill(5) },
        { ...valid, worldStepTimes: [...Array(110).fill(3), ...Array(10).fill(7)] },
        { ...valid, browser: { ...valid.browser, acceptedSeconds: 2, endBacklogSeconds: 1, peakBacklogSeconds: 1 } },
        { ...valid, browser: { ...valid.browser, idleExecutedSteps: 0 } },
        { ...valid, browser: { ...valid.browser, droppedSteps: 1 } },
        { ...valid, browser: { ...valid.browser, p99FrameMs: 50 } }
    ];
    for (const data of cases) assert.equal(assessCoupledTiming(data).realTime60FpsPass, false);
});

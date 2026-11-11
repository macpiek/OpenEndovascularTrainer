import test from 'node:test';
import assert from 'node:assert/strict';
import {createPassiveContrastProbe} from '../src/benchmark/passiveContrastProbe.js';

test('passive capture separates committed mechanics, rendered frames and contrast acquisition rates', () => {
    let time = 0, state = {acceptedSteps: 100, completedSimulationTime: 100 / 60, accumulator: 0};
    const probe = createPassiveContrastProbe({readState: () => state, now: () => time, durationMs: 4000});
    assert.equal(probe.start(), true); assert.equal(probe.start(), false);
    const frame = (steps, active, backlog, rendererUpdateMs, acquisitions) => {
        time += 1000; state = {...state, acceptedSteps: state.acceptedSteps + steps,
            completedSimulationTime: state.completedSimulationTime + steps / 60, accumulator: backlog};
        probe.recordFrame({contrastActive: active, rendererUpdateMs, acquisitions});
    };
    frame(60, false, 0, 0, 0); frame(30, true, .5, 25, 15);
    frame(30, true, 1, 40, 15); frame(60, false, 1, 0, 0);
    const report = probe.report;
    assert.equal(probe.running, false); assert.equal(report.valid, true);
    assert.equal(report.windows.all.acceptedPhysicsHz, 45);
    assert.equal(report.windows.contrastActive.acceptedPhysicsHz, 30);
    assert.equal(report.windows.contrastInactive.acceptedPhysicsHz, 60);
    assert.equal(report.windows.contrastActive.acquisitionsPerSecond, 15);
    assert.equal(report.windows.all.framesPerSecond, 1);
    assert.equal(report.windows.contrastActive.backlogGrowthSeconds, 1);
    assert.equal(report.windows.contrastActive.costs.rendererUpdateMs.maxMs, 40);
    assert.equal(report.windows.contrastActive.costs.rendererUpdateMs.meanMs, 32.5);
    assert.equal(state.acceptedSteps, 280, 'measurement must not modify the source');
});

test('paused intervals cannot be mistaken for a slow contrast mechanics window', () => {
    let time = 0, state = {acceptedSteps: 0, completedSimulationTime: 0, accumulator: .1};
    const probe = createPassiveContrastProbe({readState: () => state, now: () => time, durationMs: 2000});
    probe.start(); time = 1000; probe.recordFrame({paused: true, contrastActive: true});
    state = {...state, acceptedSteps: 60, completedSimulationTime: 1};
    time = 2000; probe.recordFrame({paused: false, contrastActive: true});
    assert.equal(probe.report.windows.paused.wallSeconds, 1);
    assert.equal(probe.report.windows.contrastActive.acceptedPhysicsHz, 60);
    assert.equal(probe.report.windows.all.acceptedPhysicsHz, 30);
});

test('counter resets invalidate a capture instead of reporting fabricated throughput', () => {
    let time = 0, state = {acceptedSteps: 200, completedSimulationTime: 4, accumulator: 0};
    const probe = createPassiveContrastProbe({readState: () => state, now: () => time});
    probe.start(); time = 500; state = {acceptedSteps: 0, completedSimulationTime: 0, accumulator: 0};
    probe.recordFrame({contrastActive: true});
    assert.equal(probe.running, false); assert.equal(probe.report.valid, false);
    assert.equal(probe.report.reason, 'counter-reset-or-invalid-state');
    assert.equal(probe.report.windows.all.acceptedSteps, 0);
});

test('separated contrast windows attribute only their own backlog changes', () => {
    let time = 0, state = {acceptedSteps: 0, completedSimulationTime: 0, accumulator: 0};
    const probe = createPassiveContrastProbe({readState: () => state, now: () => time, durationMs: 3000});
    probe.start();
    for (const [contrastActive, accumulator] of [[true, .3], [false, .1], [true, .2]]) {
        time += 1000; state = {...state, accumulator}; probe.recordFrame({contrastActive});
    }
    assert.ok(Math.abs(probe.report.windows.contrastActive.backlogGrowthSeconds - .4) < 1e-12);
    assert.ok(Math.abs(probe.report.windows.contrastInactive.backlogGrowthSeconds + .2) < 1e-12);
    assert.ok(Math.abs(probe.report.windows.all.backlogGrowthSeconds - .2) < 1e-12);
});

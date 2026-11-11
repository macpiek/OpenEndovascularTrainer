import test from 'node:test';
import assert from 'node:assert/strict';
import {createSimulationWallClock} from '../src/physics/simulationWallClock.js';

test('delayed GPU frames cannot starve or double-charge fixed physics ticks', () => {
    const clock = createSimulationWallClock();
    let debt = 0, ticks = 0, frames = 0;
    clock.sample(0, true);
    for (let time = 4; time <= 2000; time += 4) {
        // Timer-driven work proceeds even with just four display frames/sec.
        debt += clock.sample(time, true);
        while (debt + 1e-9 >= 1 / 60) {debt -= 1 / 60; ticks++;}
        if (time % 250 === 0) {frames++; debt += clock.sample(time, true);}
    }
    assert.equal(frames, 4);
    assert.equal(ticks, 120);
    assert.ok(Math.abs(debt) < 1e-12);
});

test('loading, pause and background time never become catch-up debt', () => {
    const clock = createSimulationWallClock();
    assert.equal(clock.sample(0, false), 0);
    assert.equal(clock.sample(60000, true), 0);
    assert.equal(clock.sample(60010, true), .01);
    assert.equal(clock.sample(60020, false), 0);
    assert.equal(clock.sample(90000, true), 0);
    assert.equal(clock.sample(90020, true), .02);
    clock.reset();
    assert.equal(clock.sample(120000, true), 0);
    assert.equal(clock.sample(120010, true), .01);
});

import assert from 'node:assert/strict';
import test from 'node:test';
import {createStartupDiagnostics} from '../src/startupDiagnostics.js';

function fixture({frames = true} = {}) {
    let id = 0, time = 100;
    const pendingFrames = new Map(), timers = new Map(), messages = [], logs = [];
    const host = {
        setTimeout(callback, delay) { const key = ++id; timers.set(key, {callback, delay}); return key; },
        clearTimeout(key) { timers.delete(key); },
        ...(frames ? {
            requestAnimationFrame(callback) { const key = ++id; pendingFrames.set(key, callback); return key; },
            cancelAnimationFrame(key) { pendingFrames.delete(key); }
        } : {})
    };
    const diagnostics = createStartupDiagnostics({host, now: () => time,
        show: message => messages.push(message), log: (message, timing) => logs.push({message, ...timing})});
    return {diagnostics, pendingFrames, timers, messages, logs,
        advance: value => { time += value; },
        fireFrame() { const [key, callback] = pendingFrames.entries().next().value; pendingFrames.delete(key); callback(); },
        fireTimer(delay) {
            const [key, timer] = [...timers].find(([, value]) => value.delay === delay);
            timers.delete(key); timer.callback();
        }};
}

test('startup checkpoints identify the stage and time spent in the preceding stage', () => {
    const f = fixture();
    f.diagnostics.mark('Loading simulator code');
    f.advance(125);
    f.diagnostics.mark('Creating graphics context');
    assert.deepEqual(f.messages, ['Loading simulator code', 'Creating graphics context']);
    assert.deepEqual(f.logs[1], {message: 'Creating graphics context', elapsedMs: 125,
        previousStage: 'Loading simulator code', previousStageMs: 125});
    assert.equal(f.timers.size + f.pendingFrames.size, 0);
});

test('the stage is shown before work resumes after a paint opportunity, with no recurring work', async () => {
    const f = fixture();
    let resumed = false;
    const ready = f.diagnostics.before('Preparing renderer').then(() => { resumed = true; });
    assert.deepEqual(f.messages, ['Preparing renderer']);
    assert.equal(resumed, false);
    f.fireFrame();
    await Promise.resolve();
    assert.equal(resumed, false, 'a frame callback runs before paint, so resume in the following task');
    f.fireTimer(0);
    await ready;
    assert.equal(resumed, true);
    assert.equal(f.timers.size + f.pendingFrames.size, 0);
});

test('a background tab cannot leave startup waiting indefinitely for animation frames', async () => {
    const f = fixture();
    const ready = f.diagnostics.before('Preparing renderer');
    f.fireTimer(100);
    await ready;
    assert.equal(f.timers.size + f.pendingFrames.size, 0);
});

test('runtime disposal cancels the pending paint yield and prevents startup from continuing', async () => {
    const f = fixture();
    const controller = new AbortController();
    const ready = f.diagnostics.before('Preparing renderer', {signal: controller.signal});
    f.fireFrame();
    controller.abort();
    await assert.rejects(ready, {name: 'AbortError'});
    assert.equal(f.timers.size + f.pendingFrames.size, 0);
    await assert.rejects(f.diagnostics.before('Must not run', {signal: controller.signal}), {name: 'AbortError'});
    assert.deepEqual(f.messages, ['Preparing renderer']);
});

test('hosts without animation frames yield one task and finish', async () => {
    const f = fixture({frames: false});
    const ready = f.diagnostics.before('Preparing renderer');
    f.fireTimer(0);
    await ready;
    assert.equal(f.timers.size, 0);
});

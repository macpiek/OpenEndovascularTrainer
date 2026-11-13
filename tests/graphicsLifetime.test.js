import test from 'node:test';
import assert from 'node:assert/strict';
import {HalfFloatType, WebGLRenderTarget} from 'three';
import {createManagedWebGLRenderer, renderTargetDiagnostics} from '../src/graphicsLifetime.js';
import {createRuntimeLifetime, disposeSimulatorRuntime, registerSimulatorRuntime} from '../src/runtimeLifetime.js';

function fixture(t) {
    t.mock.method(console, 'info', () => {});
    t.mock.method(console, 'error', () => {});
    const host = new EventTarget();
    const frames = new Map(), timers = new Map();
    let next = 1, lost = false;
    Object.assign(host, {
        requestAnimationFrame: callback => {const id = next++; frames.set(id, callback); return id;},
        cancelAnimationFrame: id => frames.delete(id),
        setTimeout: callback => {const id = next++; timers.set(id, callback); return id;},
        clearTimeout: id => timers.delete(id)
    });
    const canvas = Object.assign(new EventTarget(), {width: 640, height: 640});
    const renderer = {domElement: canvas, info: {memory: {geometries: 7, textures: 3}, programs: [{}, {}]},
        getContext: () => ({isContextLost: () => lost})};
    const runtime = createRuntimeLifetime(host);
    t.after(() => runtime.dispose());
    return {host, frames, timers, canvas, renderer, runtime, lose: () => {lost = true;}};
}

test('bootstrap failure disposes the registered partial runtime without needing successful module exports', t => {
    const {runtime, frames, timers} = fixture(t);
    registerSimulatorRuntime(runtime);
    let cleanup = 0;
    runtime.onDispose(() => cleanup++);
    runtime.frame(() => assert.fail('render after startup failure'));
    runtime.timeout(() => assert.fail('physics after startup failure'), 4);
    disposeSimulatorRuntime();
    disposeSimulatorRuntime();
    assert.equal(cleanup, 1);
    assert.equal(runtime.disposed, true);
    assert.equal(frames.size + timers.size, 0);
});

test('creation failure records the browser status and removes its temporary event listener', t => {
    const {runtime, canvas} = fixture(t);
    let adds = 0, removes = 0;
    const add = canvas.addEventListener.bind(canvas), remove = canvas.removeEventListener.bind(canvas);
    canvas.addEventListener = (...args) => {adds++; add(...args);};
    canvas.removeEventListener = (...args) => {removes++; remove(...args);};
    assert.throws(() => createManagedWebGLRenderer(() => {
        const event = new Event('webglcontextcreationerror');
        event.statusMessage = 'GPU process unavailable';
        canvas.dispatchEvent(event);
        throw new Error('Error creating WebGL context.');
    }, {canvas, label: 'failed-main', runtime}), /Error creating WebGL/);
    assert.equal(adds, removes);
    const failure = globalThis.__OET_GRAPHICS__.snapshot().at(-1);
    assert.equal(failure.kind, 'creation-failed');
    assert.equal(failure.detail, 'GPU process unavailable');
    assert.equal(failure.memory, null);
});

test('loss stops rendering, physics and listeners once and snapshots resources before teardown', t => {
    const {runtime, canvas, renderer, frames, timers, lose} = fixture(t);
    let resources = 12, failures = 0, cleanups = 0;
    const {guard} = createManagedWebGLRenderer(() => renderer, {canvas, label: 'loss-main', runtime,
        readResources: () => ({ownedResources: resources}), onFailure: () => {failures++; runtime.dispose();}});
    runtime.onDispose(() => {resources = 0; cleanups++;});
    runtime.frame(() => assert.fail('late render'));
    runtime.timeout(() => assert.fail('late physics'), 4);
    const lateFrame = [...frames.values()][0], lateTimer = [...timers.values()][0];
    runtime.listen(canvas, 'pointerdown', () => assert.fail('control survived failure'));
    lose(); canvas.dispatchEvent(new Event('webglcontextlost'));
    canvas.dispatchEvent(new Event('webglcontextlost'));
    canvas.dispatchEvent(new Event('pointerdown'));
    lateFrame(); lateTimer();
    assert.equal(guard.check(), false);
    assert.equal(failures, 1);
    assert.equal(cleanups, 1);
    assert.equal(frames.size + timers.size, 0);
    const record = globalThis.__OET_GRAPHICS__.snapshot().at(-1);
    assert.equal(record.contextLost, true);
    assert.equal(record.ownedResources, 12);
    assert.deepEqual(record.memory, {geometries: 7, textures: 3});
});

test('loss discovered before the asynchronous event stops work; intentional disposal does not report a crash', t => {
    const {runtime, renderer, canvas, lose} = fixture(t);
    let failures = 0;
    const {guard} = createManagedWebGLRenderer(() => renderer, {canvas, label: 'early-loss', runtime,
        onFailure: () => {failures++;}});
    lose();
    assert.equal(guard.check(), false);
    canvas.dispatchEvent(new Event('webglcontextlost'));
    assert.equal(failures, 1);
    const second = createManagedWebGLRenderer(() => renderer, {canvas, label: 'intentional-dispose', runtime,
        onFailure: () => assert.fail('intentional context release reported as crash')});
    runtime.dispose();
    canvas.dispatchEvent(new Event('webglcontextlost'));
    assert.equal(second.guard.check(), false);
});

test('secondary context loss can release its own view without stopping the main lifetime', t => {
    const {runtime, renderer, canvas, lose} = fixture(t);
    let previewCleanups = 0;
    const {guard} = createManagedWebGLRenderer(() => renderer, {canvas, label: 'preview', runtime,
        onFailure: () => {previewCleanups++; guard.dispose();}});
    lose(); canvas.dispatchEvent(new Event('webglcontextlost'));
    assert.equal(previewCleanups, 1);
    assert.equal(runtime.disposed, false);
});

test('target diagnostics include resolved color, multisampling and depth without allocating GPU storage', () => {
    const single = new WebGLRenderTarget(10, 20, {type: HalfFloatType, depthBuffer: false});
    const msaa = new WebGLRenderTarget(10, 20, {samples: 4});
    const result = renderTargetDiagnostics({single, msaa});
    assert.equal(result.targets[0].estimatedBytes, 10 * 20 * 8);
    assert.equal(result.targets[1].estimatedBytes, 10 * 20 * (4 * 5 + 4 * 4));
    assert.equal(result.estimatedTargetBytes, 8800);
});

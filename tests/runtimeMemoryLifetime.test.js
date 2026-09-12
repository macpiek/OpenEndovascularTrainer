import assert from 'node:assert/strict';
import test from 'node:test';
import { BoxGeometry, DataTexture, Group, Mesh, ShaderMaterial, WebGLRenderTarget } from 'three';
import { createRuntimeLifetime } from '../src/runtimeLifetime.js';
import { disposeThreeResources } from '../src/disposeThreeResources.js';
import { loadWorkerAsset } from '../src/loadWorkerAsset.js';
import { releaseOwnedBuffers } from '../src/releaseOwnedBuffers.js';

function fakeHost() {
    const host = new EventTarget();
    let next = 1;
    host.frames = new Map();
    host.timers = new Map();
    host.requestAnimationFrame = callback => { const id = next++; host.frames.set(id, callback); return id; };
    host.cancelAnimationFrame = id => host.frames.delete(id);
    host.setTimeout = callback => { const id = next++; host.timers.set(id, callback); return id; };
    host.clearTimeout = id => host.timers.delete(id);
    return host;
}

test('reload cancels scheduled work, aborts loading and removes listeners exactly once', () => {
    const host = fakeHost();
    const runtime = createRuntimeLifetime(host);
    let callbacks = 0, cleanups = 0;
    runtime.frame(() => callbacks++);
    runtime.timeout(() => callbacks++, 0);
    runtime.listen(host, 'resize', () => callbacks++);
    runtime.onDispose(() => cleanups++);
    const lateFrame = [...host.frames.values()][0];
    const lateTimer = [...host.timers.values()][0];
    host.dispatchEvent(new Event('pagehide'));
    runtime.dispose();
    host.dispatchEvent(new Event('resize'));
    lateFrame(); lateTimer();
    assert.equal(callbacks, 0);
    assert.equal(cleanups, 1);
    assert.equal(host.frames.size + host.timers.size, 0);
    assert.equal(runtime.signal.aborted, true);
    assert.equal(runtime.frame(() => callbacks++), null);
});

test('back/forward cache preserves the runtime for restoration', () => {
    const host = fakeHost();
    const runtime = createRuntimeLifetime(host);
    const event = new Event('pagehide');
    Object.defineProperty(event, 'persisted', { value: true });
    host.dispatchEvent(event);
    assert.equal(runtime.disposed, false);
    runtime.dispose();
});

test('shared geometry, materials, uniforms and offscreen textures release their CPU references', () => {
    const geometry = new BoxGeometry();
    geometry.boundsTree = { retainedBuffer: new Float32Array(100) };
    const texture = new DataTexture(new Uint8Array(16), 2, 2);
    const oldPositions = geometry.attributes.position.array;
    const oldPixels = texture.source.data.data;
    const material = new ShaderMaterial({ uniforms: { image: { value: texture } } });
    const root = new Group();
    root.add(new Mesh(geometry, material), new Mesh(geometry, material));
    let geometryDisposals = 0, materialDisposals = 0, textureDisposals = 0;
    geometry.addEventListener('dispose', () => geometryDisposals++);
    material.addEventListener('dispose', () => materialDisposals++);
    texture.addEventListener('dispose', () => textureDisposals++);
    const target = new WebGLRenderTarget(4, 4);
    disposeThreeResources({ roots: [root], textures: [texture], targets: [target] });
    assert.equal(geometryDisposals, 1);
    assert.equal(materialDisposals, 1);
    assert.equal(textureDisposals, 1);
    assert.equal(root.children.length, 0);
    assert.deepEqual(geometry.attributes, {});
    assert.equal(geometry.index, null);
    assert.equal(geometry.boundsTree, null);
    assert.equal(texture.source.data, null);
    assert.deepEqual(material.uniforms, {});
    if (typeof ArrayBuffer.prototype.transfer === 'function') {
        assert.equal(oldPositions.byteLength, 0, 'retained geometry views must lose their backing storage');
        assert.equal(oldPixels.byteLength, 0, 'retained texture views must lose their backing storage');
    }
});

test('buffer destruction handles shared views and older engines without copying the payload', () => {
    const buffer = new ArrayBuffer(128);
    const view = new Float32Array(buffer);
    releaseOwnedBuffers([buffer, view, view, undefined]);
    if (typeof ArrayBuffer.prototype.transfer === 'function') assert.equal(view.byteLength, 0);
    const legacyBuffer = new ArrayBuffer(8);
    Object.defineProperty(legacyBuffer, 'transfer', { value: undefined });
    assert.doesNotThrow(() => releaseOwnedBuffers([legacyBuffer]));
    assert.equal(legacyBuffer.byteLength, 8);
});

function fakeWorker() {
    return { terminations: 0, postMessage() {}, terminate() { this.terminations++; } };
}

for (const outcome of ['success', 'error', 'messageerror', 'abort', 'send-error']) {
    test(`asset worker releases its owner on ${outcome}`, async () => {
        const worker = fakeWorker();
        const controller = new AbortController();
        if (outcome === 'send-error') worker.postMessage = () => { throw new Error('send failed'); };
        const result = loadWorkerAsset(() => worker, { url: 'test.obj' }, controller.signal);
        if (outcome === 'success') worker.onmessage({ data: { meshes: [] } });
        else if (outcome === 'error') worker.onerror({ message: 'failed' });
        else if (outcome === 'messageerror') worker.onmessageerror();
        else if (outcome === 'abort') controller.abort();
        if (outcome === 'success') assert.deepEqual(await result, { meshes: [] });
        else await assert.rejects(result);
        controller.abort();
        assert.equal(worker.terminations, 1);
        assert.equal(worker.onmessage, null);
        assert.equal(worker.onerror, null);
        assert.equal(worker.onmessageerror, null);
    });
}

test('an already aborted load never creates a worker', async () => {
    const controller = new AbortController();
    controller.abort();
    await assert.rejects(loadWorkerAsset(() => {
        assert.fail('worker must not be created after teardown');
    }, {}, controller.signal), { name: 'AbortError' });
});

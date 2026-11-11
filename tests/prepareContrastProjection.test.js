import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { prepareContrastProjection } from '../src/contrast/prepareContrastProjection.js';

function fixture({ failCompile = false, failDraw = false, failLink = false, parallel = true, cached = false } = {}) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute([0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1], 3));
    geometry.setIndex([0, 1, 2, 1, 2, 3]); geometry.addGroup(3, 3, 0); geometry.setDrawRange(3, 3);
    const iodine = new Float32Array([0, .2, .7, 1]);
    const material = new THREE.ShaderMaterial({ uniforms: { iodine: { value: iodine } } });
    const source = new THREE.Mesh(geometry, [material]); source.visible = false;
    const camera = new THREE.PerspectiveCamera(); camera.position.set(0, 4, 10);
    let temporary, disposed = false, draws = 0, queries = 0, lost = false;
    const program = {ready: false, deleted: false};
    const gl = {
        DELETE_STATUS: 1, LINK_STATUS: 2,
        linkProgram(value) {assert.equal(value, program);},
        getExtension(name) {assert.equal(name, 'KHR_parallel_shader_compile'); return parallel ? {COMPLETION_STATUS_KHR: 3} : null;},
        isContextLost() {return lost;}, isProgram(value) {return value === program && !program.deleted;},
        getProgramParameter(value, parameter) {
            assert.equal(value, program); assert.equal(program.deleted, false, 'never query a deleted WebGL program'); queries++;
            if (parameter === this.DELETE_STATUS) return false;
            if (parameter === this.LINK_STATUS) return !failLink;
            if (parameter === 3) return program.ready;
            throw Error(`Unknown program parameter ${parameter}`);
        },
        getProgramInfoLog() {return 'controlled link failure';}
    };
    const initialTarget = { name: 'display' };
    const renderer = {
        target: initialTarget, face: 2, level: 3,
        domElement: new EventTarget(), getContext() {return gl;},
        xr: { enabled: true }, info: { render: { frame: 7, calls: 9, triangles: 31 } },
        getRenderTarget() { return this.target; },
        getActiveCubeFace() { return this.face; }, getActiveMipmapLevel() { return this.level; },
        setRenderTarget(value, face = 0, level = 0) { this.target = value; this.face = face; this.level = level; },
        compileAsync() {throw Error('Uncancellable Three compileAsync must not be called');},
        compile(scene, usedCamera) {
            assert.notEqual(usedCamera, camera, 'warmup cannot update the live camera matrices');
            assert.equal(this.target.width, 1); assert.equal(this.target.height, 1);
            temporary = scene.children[0];
            temporary.geometry.addEventListener('dispose', () => {
                disposed = true;
                assert.equal(Object.keys(temporary.geometry.attributes).length, 0);
                assert.equal(temporary.geometry.index, null, 'disposal must preserve the shared index buffer');
            });
            if (failCompile) throw Error('compile failed');
            if (!cached) gl.linkProgram(program);
        },
        render(scene) {
            draws++;
            const mesh = scene.children[0];
            assert.equal(mesh.material, source.material);
            assert.equal(mesh.geometry.getAttribute('position'), source.geometry.getAttribute('position'));
            assert.equal(mesh.geometry.index, source.geometry.index);
            assert.deepEqual(mesh.geometry.drawRange, { start: 0, count: 3 });
            assert.deepEqual(mesh.scale.toArray(), [0, 0, 0], 'no fragments may run the anatomical integral');
            this.info.render.calls = 100;
            if (failDraw) throw Error('draw failed');
        }
    };
    return { renderer, source, camera, initialTarget, gl, program,
        complete() {program.ready = true;}, get disposed() { return disposed; }, get draws() { return draws; },
        get queries() {return queries;}, loseContext() {lost = true; renderer.domElement.dispatchEvent(new Event('webglcontextlost'));},
        dispose() { geometry.dispose(); material.dispose(); } };
}

test('projector warmup uploads shared buffers with one degenerate triangle and restores state across async loading', async () => {
    const f = fixture();
    try {
        const pending = prepareContrastProjection(f.renderer, f.source, f.camera);
        assert.equal(f.renderer.target, f.initialTarget, 'do not leave offscreen target bound while compilation waits');
        assert.equal(f.renderer.xr.enabled, true);
        const laterTarget = { name: 'next loading frame' };
        f.renderer.setRenderTarget(laterTarget, 1, 4);
        f.complete();
        assert.deepEqual(await pending, { prepared: true, drawnVertices: 3, uploadedVertices: 4 });
        assert.equal(f.renderer.target, laterTarget);
        assert.equal(f.renderer.face, 1); assert.equal(f.renderer.level, 4);
        assert.deepEqual(f.renderer.info.render, { frame: 7, calls: 9, triangles: 31 });
        assert.equal(f.source.visible, false);
        assert.deepEqual(f.source.geometry.groups, [{ start: 3, count: 3, materialIndex: 0 }]);
        assert.deepEqual(f.source.geometry.drawRange, { start: 3, count: 3 });
        assert.deepEqual(Array.from(f.source.material[0].uniforms.iodine.value), [0, Math.fround(.2), Math.fround(.7), 1]);
        assert.equal(f.disposed, true); assert.equal(f.draws, 1);
    } finally { f.dispose(); }
});

for (const phase of ['Compile', 'Draw']) test(`projector warmup restores resources after ${phase.toLowerCase()} failure`, async () => {
    const f = fixture({ [`fail${phase}`]: true });
    try {
        const pending = prepareContrastProjection(f.renderer, f.source, f.camera);
        if (phase === 'Draw') f.complete();
        await assert.rejects(pending, new RegExp(phase.toLowerCase() + ' failed'));
        assert.equal(f.renderer.target, f.initialTarget);
        assert.equal(f.renderer.face, 2); assert.equal(f.renderer.level, 3);
        assert.equal(f.renderer.xr.enabled, true);
        assert.equal(f.disposed, true);
        assert.equal(f.source.geometry.getAttribute('position').count, 4);
    } finally { f.dispose(); }
});

test('cancelled loading cannot draw after asynchronous shader compilation', async () => {
    const f = fixture(), controller = new AbortController();
    try {
        const pending = prepareContrastProjection(f.renderer, f.source, f.camera, { signal: controller.signal });
        controller.abort(); f.complete();
        await assert.rejects(pending, { name: 'AbortError' });
        assert.equal(f.draws, 0);
        assert.equal(f.disposed, true);
        assert.equal(f.renderer.target, f.initialTarget);
    } finally { f.dispose(); }
});

test('warmup restores custom current viewport/scissor without altering target configuration', async () => {
    const f = fixture();
    try {
        const target = new THREE.WebGLRenderTarget(20, 30);
        target.viewport.set(0, 0, 20, 30); target.scissor.set(0, 0, 20, 30);
        const viewport = new THREE.Vector4(2, 3, 7, 11), scissor = new THREE.Vector4(4, 5, 6, 8);
        let scissorTest = true;
        f.renderer.target = target;
        f.renderer.getCurrentViewport = out => out.copy(viewport);
        Object.assign(f.gl, {SCISSOR_BOX: 4, SCISSOR_TEST: 5,
            getParameter: () => scissor.toArray(), isEnabled: () => scissorTest});
        const setTarget = f.renderer.setRenderTarget.bind(f.renderer);
        f.renderer.setRenderTarget = (value, face, level) => {
            setTarget(value, face, level);
            if (value?.viewport) { viewport.copy(value.viewport); scissor.copy(value.scissor); scissorTest = value.scissorTest; }
        };
        const pending = prepareContrastProjection(f.renderer, f.source, f.camera);
        f.complete(); await pending;
        assert.deepEqual(viewport.toArray(), [2, 3, 7, 11]);
        assert.deepEqual(scissor.toArray(), [4, 5, 6, 8]);
        assert.equal(scissorTest, true);
        assert.deepEqual(target.viewport.toArray(), [0, 0, 20, 30]);
        assert.deepEqual(target.scissor.toArray(), [0, 0, 20, 30]);
        assert.equal(target.scissorTest, false);
        target.dispose();
    } finally { f.dispose(); }
});

for (const object of ['material', 'geometry']) test(`disposing the source ${object} cancels polling before drawing`, async () => {
    const f = fixture(), originalLink = f.gl.linkProgram;
    try {
        const pending = prepareContrastProjection(f.renderer, f.source, f.camera);
        assert.equal(f.gl.linkProgram, originalLink, 'The public WebGL method must be restored before yielding');
        if (object === 'material') f.source.material[0].dispose(); else f.source.geometry.dispose();
        await assert.rejects(pending, {name: 'AbortError'});
        const queries = f.queries;
        await new Promise(resolve => setTimeout(resolve, 25));
        assert.equal(f.queries, queries, 'No readiness timer may survive disposal');
        assert.equal(f.draws, 0); assert.equal(f.disposed, true);
        assert.equal(f.renderer.target, f.initialTarget);
    } finally {f.dispose();}
});

test('a program deleted by renderer disposal is never queried for readiness', async () => {
    const f = fixture();
    try {
        const pending = prepareContrastProjection(f.renderer, f.source, f.camera);
        f.program.deleted = true;
        await assert.rejects(pending, {name: 'AbortError'});
        assert.equal(f.draws, 0); assert.equal(f.disposed, true);
    } finally {f.dispose();}
});

test('context loss rejects preparation and releases the pending poll immediately', async () => {
    const f = fixture();
    try {
        const pending = prepareContrastProjection(f.renderer, f.source, f.camera);
        f.loseContext();
        await assert.rejects(pending, /WebGL context lost/);
        assert.equal(f.draws, 0); assert.equal(f.disposed, true);
    } finally {f.dispose();}
});

test('link failure is rejected through the returned promise with WebGL diagnostics', async () => {
    const f = fixture({failLink: true});
    try {
        const pending = prepareContrastProjection(f.renderer, f.source, f.camera);
        f.complete();
        await assert.rejects(pending, /Projection shader link failed: controlled link failure/);
        assert.equal(f.draws, 0); assert.equal(f.disposed, true);
    } finally {f.dispose();}
});

for (const options of [{cached: true}, {parallel: false}]) test(`public synchronous compile works ${options.cached ? 'with a cached program' : 'without parallel compilation'}`, async () => {
    const f = fixture(options), originalLink = f.gl.linkProgram;
    try {
        f.complete();
        const result = await prepareContrastProjection(f.renderer, f.source, f.camera);
        assert.equal(result.prepared, true); assert.equal(f.draws, 1);
        assert.equal(f.gl.linkProgram, originalLink); assert.equal(f.disposed, true);
    } finally {f.dispose();}
});

test('a never-ready parallel program times out and leaves no readiness callbacks', async () => {
    const f = fixture(), originalLink = f.gl.linkProgram;
    try {
        await assert.rejects(prepareContrastProjection(f.renderer, f.source, f.camera, {timeoutMs: 8}),
            /Projection shader preparation timed out after 8 ms/);
        const queries = f.queries;
        await new Promise(resolve => setTimeout(resolve, 25));
        assert.equal(f.queries, queries, 'The timeout must cancel future WebGL polling');
        assert.equal(f.gl.linkProgram, originalLink);
        assert.equal(f.draws, 0); assert.equal(f.disposed, true);
        assert.equal(f.renderer.target, f.initialTarget);
        assert.equal(f.renderer.xr.enabled, true);
    } finally {f.dispose();}
});

test('compile-only renderer stubs do not require a WebGL context or compileAsync', async () => {
    const f = fixture();
    try {
        delete f.renderer.getContext; delete f.renderer.compileAsync;
        assert.equal((await prepareContrastProjection(f.renderer, f.source, f.camera)).prepared, true);
        assert.equal(f.draws, 1); assert.equal(f.disposed, true);
    } finally {f.dispose();}
});

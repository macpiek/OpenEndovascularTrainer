import { BufferGeometry, HalfFloatType, Mesh, Scene, Vector4, WebGLRenderTarget } from 'three';

// Compile and upload the real projector without integrating a full anatomical
// image. Shared BufferAttributes retain their GPU allocation after the temporary
// geometry is removed; neither iodine samples nor the visible mesh are changed.
export async function prepareContrastProjection(renderer, source, camera, { signal, timeoutMs = 60000 } = {}) {
    if (!Number.isFinite(timeoutMs) || timeoutMs <= 0) throw new RangeError('Projection preparation timeout must be positive and finite');
    const deadline = performance.now() + timeoutMs;
    let stopped = null, pollTimer = null, rejectWaiting = null;
    const abortError = message => new DOMException(message, 'AbortError');
    const stop = reason => {
        stopped ??= reason;
        if (pollTimer !== null) {clearTimeout(pollTimer); pollTimer = null;}
        rejectWaiting?.(stopped);
    };
    const gl = renderer.getContext?.();
    const checkAbort = () => {
        if (signal?.aborted) throw signal.reason ?? new DOMException('Projection preparation aborted', 'AbortError');
        if (stopped) throw stopped;
        if (gl?.isContextLost?.()) throw new Error('WebGL context lost during projection preparation');
        if (performance.now() >= deadline) throw new Error(`Projection shader preparation timed out after ${timeoutMs} ms`);
    };
    checkAbort();
    const onAbort = () => stop(signal.reason ?? abortError('Projection preparation aborted'));
    const onDispose = () => stop(abortError('Projection source disposed during preparation'));
    const onContextLost = () => stop(new Error('WebGL context lost during projection preparation'));
    const materials = Array.isArray(source.material) ? source.material : [source.material];
    signal?.addEventListener('abort', onAbort, {once: true});
    source.geometry.addEventListener('dispose', onDispose);
    for (const material of materials) material.addEventListener('dispose', onDispose);
    renderer.domElement?.addEventListener('webglcontextlost', onContextLost);
    const geometry = new BufferGeometry();
    for (const [name, attribute] of Object.entries(source.geometry.attributes))
        geometry.setAttribute(name, attribute);
    if (source.geometry.index) geometry.setIndex(source.geometry.index);
    const vertexCount = geometry.index?.count ?? geometry.getAttribute('position')?.count ?? 0;
    const drawnVertices = Math.min(3, vertexCount);
    geometry.setDrawRange(0, drawnVertices);
    geometry.addGroup(0, drawnVertices, 0);
    const mesh = new Mesh(geometry, source.material);
    mesh.frustumCulled = false;
    // All three vertices coincide after model transformation: uploads and
    // shader binding occur, but the rasterizer produces no fragments/rays.
    mesh.scale.set(0, 0, 0);
    const scene = new Scene(); scene.add(mesh);
    const warmCamera = camera.clone();
    const target = new WebGLRenderTarget(1, 1, { type: HalfFloatType, samples: 0, depthBuffer: false });
    const withTarget = action => {
        const previous = renderer.getRenderTarget();
        const face = renderer.getActiveCubeFace(), level = renderer.getActiveMipmapLevel();
        const configuredViewport = previous?.viewport?.clone();
        const configuredScissor = previous?.scissor?.clone();
        const configuredScissorTest = previous?.scissorTest;
        const currentViewport = configuredViewport && renderer.getCurrentViewport(new Vector4());
        const gl = configuredScissor ? renderer.getContext() : null;
        const currentScissor = gl ? gl.getParameter(gl.SCISSOR_BOX) : null;
        const currentScissorTest = gl ? gl.isEnabled(gl.SCISSOR_TEST) : false;
        const xrEnabled = renderer.xr?.enabled;
        const renderInfo = renderer.info?.render ? { ...renderer.info.render } : null;
        try {
            if (renderer.xr) renderer.xr.enabled = false;
            renderer.setRenderTarget(target);
            return action();
        } finally {
            // A caller may have changed its viewport after binding a target.
            // Restore that current state without changing the target's stored
            // configuration or the renderer's global viewport/scissor values.
            if (currentViewport) previous.viewport.copy(currentViewport);
            if (currentScissor) { previous.scissor.fromArray(currentScissor); previous.scissorTest = currentScissorTest; }
            try { renderer.setRenderTarget(previous, face, level); }
            finally {
                if (configuredViewport) previous.viewport.copy(configuredViewport);
                if (configuredScissor) { previous.scissor.copy(configuredScissor); previous.scissorTest = configuredScissorTest; }
            }
            if (renderer.xr) renderer.xr.enabled = xrEnabled;
            if (renderInfo) Object.assign(renderer.info.render, renderInfo);
        }
    };
    try {
        // Three r160 compileAsync owns an uncancellable timer which dereferences
        // disposed material properties. Use the public synchronous compile API
        // and capture its public WebGL link handles only for that call. Polling
        // belongs to this preparation and is cancelled with the source/context.
        const programs = new Set(), linkProgram = gl?.linkProgram;
        const captureLink = program => {programs.add(program); return linkProgram.call(gl, program);};
        try {
            if (typeof linkProgram === 'function') gl.linkProgram = captureLink;
            withTarget(() => renderer.compile(scene, warmCamera));
        } finally {
            if (typeof linkProgram === 'function' && gl.linkProgram === captureLink) gl.linkProgram = linkProgram;
        }
        checkAbort();
        // An already cached program produces no link calls. In that case the
        // tiny upload draw can proceed; no private renderer properties are read.
        if (programs.size) {
            const parallel = gl.getExtension('KHR_parallel_shader_compile');
            await new Promise((resolve, reject) => {
                rejectWaiting = reject;
                const poll = () => {
                    pollTimer = null;
                    try {
                        checkAbort();
                        for (const program of programs) {
                            if (!gl.isProgram(program) || gl.getProgramParameter(program, gl.DELETE_STATUS))
                                throw abortError('Projection shader program disposed during preparation');
                            if (parallel && !gl.getProgramParameter(program, parallel.COMPLETION_STATUS_KHR)) continue;
                            if (!gl.getProgramParameter(program, gl.LINK_STATUS))
                                throw new Error(`Projection shader link failed: ${gl.getProgramInfoLog(program) || 'Unknown WebGL error'}`);
                            programs.delete(program);
                        }
                        if (programs.size) pollTimer = setTimeout(poll, Math.min(16, Math.max(0, deadline - performance.now())));
                        else {rejectWaiting = null; resolve();}
                    } catch (error) {rejectWaiting = null; reject(error);}
                };
                // Without KHR the link-status read may block. Give loading/UI
                // one turn first, and still permit cancellation before the read.
                if (parallel) poll(); else pollTimer = setTimeout(poll, 0);
            });
        }
        checkAbort();
        withTarget(() => renderer.render(scene, warmCamera));
        return { prepared: true, drawnVertices, uploadedVertices: geometry.getAttribute('position').count };
    } finally {
        if (pollTimer !== null) clearTimeout(pollTimer);
        rejectWaiting = null;
        signal?.removeEventListener('abort', onAbort);
        source.geometry.removeEventListener('dispose', onDispose);
        for (const material of materials) material.removeEventListener('dispose', onDispose);
        renderer.domElement?.removeEventListener('webglcontextlost', onContextLost);
        scene.remove(mesh);
        // Three disposes every attribute still attached to a geometry. Detach
        // borrowed buffers first, otherwise warmup would immediately evict them.
        for (const name of Object.keys(geometry.attributes)) geometry.deleteAttribute(name);
        geometry.setIndex(null);
        geometry.dispose(); target.dispose();
    }
}

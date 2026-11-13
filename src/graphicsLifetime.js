// Diagnostics run at lifecycle boundaries, never as a per-frame GPU readback.
const events = [];
const MAX_EVENTS = 20;

export function renderTargetDiagnostics(targets) {
    const items = Object.entries(targets).map(([name, target]) => {
        const textures = target.textures ?? [target.texture];
        const colorBytes = textures.reduce((sum, texture) => {
            // These simulator targets use RGBA8 or RGBA16F. Keep the estimate
            // explicit rather than presenting driver-private VRAM as measured.
            const bytesPerComponent = texture.type === 1016 ? 2 : texture.type === 1015 ? 4 : 1;
            return sum + target.width * target.height * 4 * bytesPerComponent;
        }, 0);
        const samples = target.samples || 0;
        const estimatedBytes = colorBytes * (1 + samples) +
            (target.depthBuffer ? target.width * target.height * 4 * Math.max(1, samples) : 0);
        return {name, width: target.width, height: target.height, samples,
            depthBuffer: target.depthBuffer, estimatedBytes};
    });
    return {targets: items, estimatedTargetBytes: items.reduce((sum, item) => sum + item.estimatedBytes, 0)};
}

export function recordGraphicsEvent(kind, label, renderer, detail = '', readResources = () => ({})) {
    let resources = {};
    try { resources = readResources(); } catch { /* Startup can fail before resources exist. */ }
    const context = renderer?.getContext();
    let capabilities;
    if (kind === 'context-created' && typeof context?.getParameter === 'function') {
        const debug = context.getExtension('WEBGL_debug_renderer_info');
        capabilities = {
            version: context.getParameter(context.VERSION),
            renderer: context.getParameter(debug?.UNMASKED_RENDERER_WEBGL ?? context.RENDERER),
            vendor: context.getParameter(debug?.UNMASKED_VENDOR_WEBGL ?? context.VENDOR),
            maxTextureSize: renderer.capabilities.maxTextureSize,
            maxSamples: renderer.capabilities.maxSamples
        };
    }
    const event = {
        kind, label, detail, atMs: Math.round(performance.now()),
        contextLost: context?.isContextLost() ?? null,
        drawingBuffer: renderer ? [renderer.domElement.width, renderer.domElement.height] : null,
        memory: renderer ? {...renderer.info.memory} : null,
        programs: renderer?.info.programs?.length ?? null,
        ...(capabilities ? {capabilities} : {}),
        ...resources
    };
    events.push(event);
    if (events.length > MAX_EVENTS) events.shift();
    globalThis.__OET_GRAPHICS__ = {snapshot: () => events.map(item => structuredClone(item))};
    console[kind === 'context-lost' || kind.endsWith('-failed') ? 'error' : 'info'](
        '[graphics]', JSON.stringify(event));
    return event;
}

export function createManagedWebGLRenderer(createRenderer, {
    canvas, label, runtime, onFailure, readResources = () => ({})
}) {
    let creationMessage = '';
    const creationError = event => { creationMessage = event.statusMessage || ''; };
    canvas.addEventListener('webglcontextcreationerror', creationError);
    let renderer;
    try {
        renderer = createRenderer();
    } catch (error) {
        recordGraphicsEvent('creation-failed', label, null, creationMessage || error.message, readResources);
        throw error;
    } finally {
        canvas.removeEventListener('webglcontextcreationerror', creationError);
    }
    let failed = false, disposed = false;
    function fail(detail) {
        if (failed || disposed || runtime?.disposed) return false;
        failed = true;
        const diagnostic = recordGraphicsEvent('context-lost', label, renderer, detail, readResources);
        onFailure?.(diagnostic);
        return true;
    }
    const lost = event => fail(event.statusMessage || 'WebGL context lost');
    canvas.addEventListener('webglcontextlost', lost);
    const guard = {
        check() {
            if (failed || disposed || runtime?.disposed) return false;
            if (renderer.getContext().isContextLost()) {
                fail('WebGL context lost before its event was delivered');
                return false;
            }
            return true;
        },
        snapshot: (kind, detail = '') => recordGraphicsEvent(kind, label, renderer, detail, readResources),
        dispose() {
            if (disposed) return;
            disposed = true;
            canvas.removeEventListener('webglcontextlost', lost);
        }
    };
    runtime?.onDispose(guard.dispose);
    guard.snapshot('context-created');
    return {renderer, guard};
}

export function showSimulatorGraphicsFailure(message, document = globalThis.document) {
    if (!document) return;
    document.body.classList.add('system-initializing');
    // A stopped runtime has no interactive controls. Keep the recovery action
    // outside those panels, in the detector cell that already owns loading.
    for (const id of ['controls', 'simulation-sidebar', 'simulation-toolbar']) {
        const element = document.getElementById(id);
        if (element) element.inert = true;
    }
    const viewport = document.getElementById('simulation-viewport');
    if (!viewport) return;
    document.getElementById('loadingScreen')?.remove();
    let panel = document.getElementById('graphicsFailure');
    if (!panel) {
        panel = document.createElement('div');
        panel.id = 'graphicsFailure';
        panel.className = 'loading-screen';
        panel.style.pointerEvents = 'auto';
        panel.setAttribute('role', 'alert');
        viewport.appendChild(panel);
    }
    const content = document.createElement('div');
    content.className = 'loading-panel';
    const text = document.createElement('p');
    text.textContent = message;
    const reload = document.createElement('button');
    reload.type = 'button';
    reload.textContent = 'Uruchom ponownie';
    reload.addEventListener('click', () => globalThis.location.reload(), {once: true});
    content.append(text, reload);
    panel.replaceChildren(content);
}

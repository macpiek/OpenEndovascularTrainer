// Own only this simulator's scheduled work and listeners. Do not modify
// browser globals or force collection with artificial allocations.
export function createRuntimeLifetime(host = window) {
    const controller = new AbortController();
    const frames = new Set();
    const timers = new Set();
    const cleanups = [];
    let disposed = false;
    const runtime = {
        get disposed() { return disposed; },
        signal: controller.signal,
        onDispose(callback) {
            if (disposed) callback();
            else cleanups.push(callback);
        },
        frame(callback) {
            if (disposed) return null;
            const id = host.requestAnimationFrame(time => {
                frames.delete(id);
                if (!disposed) callback(time);
            });
            frames.add(id);
            return id;
        },
        timeout(callback, delay) {
            if (disposed) return null;
            const id = host.setTimeout(() => {
                timers.delete(id);
                if (!disposed) callback();
            }, delay);
            timers.add(id);
            return id;
        },
        clearTimeout(id) {
            timers.delete(id);
            host.clearTimeout(id);
        },
        listen(target, type, callback, options = {}) {
            target.addEventListener(type, callback, { ...options, signal: controller.signal });
        },
        dispose() {
            if (disposed) return;
            disposed = true;
            controller.abort();
            for (const id of frames) host.cancelAnimationFrame(id);
            for (const id of timers) host.clearTimeout(id);
            frames.clear();
            timers.clear();
            const errors = [];
            // Continue freeing other owners even if one cleanup fails.
            for (const callback of cleanups.splice(0).reverse()) {
                try { callback(); } catch (error) { errors.push(error); }
            }
            if (errors.length) console.error('Simulator cleanup failed', new AggregateError(errors));
        }
    };
    runtime.listen(host, 'pagehide', event => {
        // A page in the back/forward cache must remain usable on return.
        // Reloads and actual document destruction have persisted === false.
        if (!event.persisted) runtime.dispose();
    });
    return runtime;
}

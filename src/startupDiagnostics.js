// These checkpoints run only during startup. The bounded paint yield lets the
// loading panel identify a slow synchronous stage without a recurring timer.
export function createStartupDiagnostics({
    host = globalThis,
    now = () => performance.now(),
    show = message => {
        const element = host.document?.getElementById('loadingMessage');
        if (element) element.textContent = message;
    },
    log = (message, timing) => host.console?.info('[startup]', message, JSON.stringify(timing))
} = {}) {
    let startedAt;
    let previousAt;
    let previousStage;

    function mark(message) {
        const time = now();
        startedAt ??= time;
        const timing = {
            elapsedMs: Math.round(time - startedAt),
            previousStage,
            previousStageMs: previousAt === undefined ? 0 : Math.round(time - previousAt)
        };
        show(message);
        log(message, timing);
        previousAt = time;
        previousStage = message;
        return timing;
    }

    async function before(message, {signal} = {}) {
        signal?.throwIfAborted();
        mark(message);
        await new Promise((resolve, reject) => {
            let frame = null;
            let afterPaint = null;
            let fallback = null;
            let settled = false;
            function finish(error) {
                if (settled) return;
                settled = true;
                if (frame !== null) host.cancelAnimationFrame(frame);
                if (afterPaint !== null) host.clearTimeout(afterPaint);
                if (fallback !== null) host.clearTimeout(fallback);
                signal?.removeEventListener('abort', abort);
                if (error) reject(error);
                else resolve();
            }
            function abort() {
                finish(signal.reason || new DOMException('Startup cancelled', 'AbortError'));
            }
            signal?.addEventListener('abort', abort, {once: true});
            // Background tabs may not receive animation frames. They must still
            // progress, and disposal must never leave a startup promise pending.
            fallback = host.setTimeout(() => finish(), host.requestAnimationFrame ? 100 : 0);
            if (host.requestAnimationFrame) {
                frame = host.requestAnimationFrame(() => {
                    afterPaint = host.setTimeout(() => finish(), 0);
                });
            }
        });
    }

    return {mark, before};
}

export const startupDiagnostics = createStartupDiagnostics();

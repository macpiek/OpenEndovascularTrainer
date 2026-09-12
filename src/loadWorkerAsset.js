// A one-shot worker owns the temporary loading/parsing heap. Terminate on
// success as well as abort/error; clear listeners so it cannot retain callers.
export function loadWorkerAsset(createWorker, message, signal) {
    return new Promise((resolve, reject) => {
        if (signal?.aborted) {
            reject(signal.reason ?? new DOMException('Aborted', 'AbortError'));
            return;
        }
        const worker = createWorker();
        let settled = false;
        const finish = (error, value) => {
            if (settled) return;
            settled = true;
            signal?.removeEventListener('abort', abort);
            worker.onmessage = worker.onerror = worker.onmessageerror = null;
            worker.terminate();
            if (error) reject(error);
            else resolve(value);
        };
        const abort = () => finish(signal.reason ?? new DOMException('Aborted', 'AbortError'));
        worker.onmessage = ({ data }) => finish(data.error ? new Error(data.error) : null, data);
        worker.onerror = event => finish(new Error(event.message || 'Asset worker failed'));
        worker.onmessageerror = () => finish(new Error('Asset worker returned invalid data'));
        signal?.addEventListener('abort', abort, { once: true });
        try { worker.postMessage(message); }
        catch (error) { finish(error); }
    });
}

// Optional diagnostic queries only; never wait for the GPU on the simulation
// thread. A disjoint timer invalidates all outstanding measurements.
export function createGpuTimerProbe(gl) {
    const ext = gl.getExtension('EXT_disjoint_timer_query_webgl2');
    let active = null, pending = [], samples = [], disjointCount = 0;
    const clear = () => {
        if (active) {gl.endQuery(ext.TIME_ELAPSED_EXT); gl.deleteQuery(active); active = null;}
        for (const query of pending) gl.deleteQuery(query);
        pending = [];
    };
    const poll = () => {
        if (!ext) return;
        if (gl.getParameter(ext.GPU_DISJOINT_EXT)) {clear(); samples = []; disjointCount++; return;}
        while (pending.length && gl.getQueryParameter(pending[0], gl.QUERY_RESULT_AVAILABLE)) {
            const query = pending.shift();
            samples.push(gl.getQueryParameter(query, gl.QUERY_RESULT) / 1e6);
            gl.deleteQuery(query);
        }
    };
    return {
        begin() {
            poll();
            if (!ext || active || pending.length >= 64 || gl.isContextLost()) return;
            active = gl.createQuery(); gl.beginQuery(ext.TIME_ELAPSED_EXT, active);
        },
        end() {if (active) {gl.endQuery(ext.TIME_ELAPSED_EXT); pending.push(active); active = null;}},
        poll,
        reset() {clear(); samples = []; disjointCount = 0;},
        report() {
            poll(); const sorted = [...samples].sort((a, b) => a - b);
            return {supported: !!ext, disjointCount, pending: pending.length, count: samples.length,
                meanMs: samples.length ? samples.reduce((a, b) => a + b, 0) / samples.length : null,
                p95Ms: sorted.length ? sorted[Math.floor((sorted.length - 1) * .95)] : null,
                maxMs: sorted.at(-1) ?? null};
        },
        dispose: clear
    };
}

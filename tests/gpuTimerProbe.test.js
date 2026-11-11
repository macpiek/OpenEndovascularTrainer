import test from 'node:test';
import assert from 'node:assert/strict';
import {createGpuTimerProbe} from '../src/benchmark/gpuTimerProbe.js';

test('GPU measurement is asynchronous, drops disjoint results, and releases every query', () => {
    let disjoint = false, ready = false, serial = 0;
    const deleted = [], gl = {
        QUERY_RESULT_AVAILABLE: 1, QUERY_RESULT: 2,
        getExtension: () => ({TIME_ELAPSED_EXT: 3, GPU_DISJOINT_EXT: 4}),
        getParameter: () => disjoint, isContextLost: () => false,
        createQuery: () => ++serial, beginQuery() {}, endQuery() {},
        getQueryParameter: (q, key) => key === 1 ? ready : q * 1e6,
        deleteQuery: q => deleted.push(q)
    };
    const probe = createGpuTimerProbe(gl);
    probe.begin(); probe.end();
    assert.equal(probe.report().count, 0);
    assert.equal(probe.report().pending, 1);
    ready = true; assert.equal(probe.report().meanMs, 1);
    probe.begin(); probe.end(); disjoint = true;
    assert.equal(probe.report().count, 0);
    assert.equal(probe.report().pending, 0);
    disjoint = false; probe.begin(); probe.dispose();
    assert.deepEqual(deleted, [1, 2, 3]);
});

test('unsupported GPU timing cannot be reported as a zero-cost render', () => {
    const probe = createGpuTimerProbe({getExtension: () => null});
    probe.begin(); probe.end(); probe.poll(); probe.reset();
    assert.equal(probe.report().supported, false);
    assert.equal(probe.report().meanMs, null);
    probe.dispose();
});

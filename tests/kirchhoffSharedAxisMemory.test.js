import test from 'node:test';
import assert from 'node:assert/strict';
import {execFileSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';

for(const mode of ['feed','dynamic','wasm','wasm-owned'])test(`shared-axis ${mode} releases superseded states while the newest simulation remains alive`,()=>{
    const result=JSON.parse(execFileSync(process.execPath,[
        '--expose-gc',fileURLToPath(new URL('./helpers/sharedAxisMemoryProbe.mjs',import.meta.url)),
        new URL('../',import.meta.url).href,mode
    ],{encoding:'utf8',timeout:30000}));
    assert.ok(result.count>=30);
    assert.equal(result.held.latestAlive,true,'the current state must remain reachable during the check');
    for(const key of ['states','positions','bodies','materialMemories']) {
        assert.equal(result.held[key],0,`old ${key} retained by the live simulation`);
        assert.equal(result.released[key],0,`${key} retained after simulation disposal`);
    }
    assert.equal(result.released.latestAlive,false);
    assert.equal(result.released.cacheAlive,false,'linear scratch cache must be released with the simulation');
    if(mode.startsWith('wasm'))assert.equal(result.materialMemoryCount,result.count);
    if(mode!=='feed') {
        assert.ok(result.factorizations>0,'exercise actual dynamic solves, not an equilibrium-only fast path');
        if(mode==='wasm') {
            // Optimized ordinary solves borrow a bounded numeric arena instead
            // of retaining per-state layouts. The WeakRef checks above still
            // require it to release every old state, body and material memory.
            assert.ok(result.held.linearScratch.compactLayouts>0,'exercise shared workspace reuse');
            assert.ok(result.held.linearScratch.compactLayouts<=16,'shared layout cache remains bounded');
            assert.ok(result.held.linearScratch.compactNumericBytes>0,'exercise shared numeric storage');
        } else assert.ok(result.held.cacheSize>0,'exercise owned workspace reuse between states');
    }
});

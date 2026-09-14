import test from 'node:test';
import assert from 'node:assert/strict';
import {execFileSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';

for(const mode of ['feed','dynamic'])test(`shared-axis ${mode} releases superseded states while the newest simulation remains alive`,()=>{
    const result=JSON.parse(execFileSync(process.execPath,[
        '--expose-gc',fileURLToPath(new URL('./helpers/sharedAxisMemoryProbe.mjs',import.meta.url)),
        new URL('../',import.meta.url).href,mode
    ],{encoding:'utf8',timeout:30000}));
    assert.ok(result.count>=30);
    assert.equal(result.held.latestAlive,true,'the current state must remain reachable during the check');
    for(const key of ['states','positions','bodies']) {
        assert.equal(result.held[key],0,`old ${key} retained by the live simulation`);
        assert.equal(result.released[key],0,`${key} retained after simulation disposal`);
    }
    assert.equal(result.released.latestAlive,false);
    assert.equal(result.released.cacheAlive,false,'linear scratch cache must be released with the simulation');
    if(mode==='dynamic') {
        assert.ok(result.factorizations>0,'exercise actual dynamic solves, not an equilibrium-only fast path');
        assert.ok(result.held.cacheSize>0,'exercise workspace reuse between states');
    }
});

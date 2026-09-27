import test from 'node:test';
import assert from 'node:assert/strict';
import {spawnSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';

test('live graft contacts release previous solver states and their WASM workspaces',()=>{
 const result=spawnSync(process.execPath,['--expose-gc',fileURLToPath(new URL('./helpers/stentGraftContactLifetime.mjs',import.meta.url))],{encoding:'utf8',timeout:30000});
 assert.equal(result.status,0,result.error?.message??result.stderr??result.stdout);
});

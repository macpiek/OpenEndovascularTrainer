import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from '../../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../../src/physics/kirchhoffSharedAxisAppSystem.js';
const anatomy=await loadCoupledRuntimeAnatomy(),samples=[];
try {
 for(const name of ['anatomy-berenstein-feed-138.67-live-cycle','anatomy-pigtail-wire-withdraw-200.27-incoming']) {
  const fixture=JSON.parse(readFileSync(new URL(`../../tests/fixtures/shared-axis/${name}.json`,import.meta.url))),req=fixture.stepRequest;
  for(let repeat=0;repeat<15;repeat++) {
   const out=[];
   for(const mode of repeat%2?[1,0]:[0,1]) {
    const input=restoreSharedAxisReplay(fixture,anatomy.field),start=performance.now();
    const iterator=advanceSharedAxis(input,req.rotations,req.dt,req.tools,{...req.options,promoteTrialAssembly:true,projectionMode:'reduced',wasmMaterial:true,reuseConstraintWork:true,reuseMatrixAssembly:!!mode});
    let next;do{next=iterator.next();}while(!next.done);
    out[mode]={ms:performance.now()-start,...next.value};
   }
   assert.deepEqual(captureSharedAxisReplay(out[1].state,fixture.sheath),captureSharedAxisReplay(out[0].state,fixture.sheath));
   samples.push({name,repeat,warmup:repeat<3,referenceMs:out[0].ms,optimizedMs:out[1].ms,referenceLinearMs:out[0].result.timings.linearMs,optimizedLinearMs:out[1].result.timings.linearMs});
  }
 }
}finally{anatomy.dispose();}
writeFileSync(new URL('./replay-profile.json',import.meta.url),JSON.stringify(samples,null,2));
for(const name of new Set(samples.map(s=>s.name))) {
 const ss=samples.filter(s=>s.name===name&&!s.warmup),mean=k=>ss.reduce((v,s)=>v+s[k],0)/ss.length;
 console.log(name,Object.fromEntries(['referenceMs','optimizedMs','referenceLinearMs','optimizedLinearMs'].map(k=>[k,mean(k)])));
}

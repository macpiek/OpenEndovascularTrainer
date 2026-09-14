import {readFileSync,writeFileSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from '../../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../../src/physics/kirchhoffSharedAxisAppSystem.js';
const root=new URL('./',import.meta.url),anatomy=await loadCoupledRuntimeAnatomy(),results=[];
try {
for(const path of ['reference/captured-incoming.json','../../tests/fixtures/shared-axis/anatomy-berenstein-feed-312.87-projection.json','../../tests/fixtures/shared-axis/anatomy-pigtail-wire-withdraw-200.27-incoming.json']) {
 const fixture=JSON.parse(readFileSync(new URL(path,root))),req=fixture.stepRequest,outputs=[];
 for(const stagnationFallback of [false,true]) {
  const input=restoreSharedAxisReplay(fixture,anatomy.field),cpu=process.cpuUsage(),start=performance.now();
  const iterator=advanceSharedAxis(input,req.rotations,req.dt,req.tools,{...req.options,promoteTrialAssembly:true,projectionMode:'reduced',stagnationFallback});let next;do{next=iterator.next();}while(!next.done);
  const elapsed=performance.now()-start,usage=process.cpuUsage(cpu),{state,result}=next.value;
  outputs.push({state:state?captureSharedAxisReplay(state,fixture.sheath):null,result,elapsed,cpuMs:(usage.user+usage.system)/1000});
 }
 const [a,b]=outputs;const stats=v=>({status:v.result.status,iterations:v.result.iterations,factorizations:v.result.factorizations,backtracks:v.result.backtracks,ms:v.elapsed,cpuMs:v.cpuMs,residual:v.result.residual,certificate:v.result.certificateBound,fallback:v.result.wallNormalFallback,recovery:v.result.stagnationRecovery,subdivisions:v.result.subdivisions});
 results.push({path,exactState:JSON.stringify(a.state)===JSON.stringify(b.state),reference:stats(a),guard:stats(b)});
}
}finally{anatomy.dispose();}
writeFileSync(new URL('replay.json',root),JSON.stringify(results,null,2));console.log(JSON.stringify(results,null,2));

import {readFileSync,writeFileSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from '../../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from '../../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../../src/physics/kirchhoffSharedAxisAppSystem.js';
if(!readFileSync(new URL('../../src/physics/kirchhoffSharedAxisLinear.js',import.meta.url),'utf8').includes('solveSharedAxisProjectionSchur'))throw new Error('Historical opt-in prototype: apply schur-linear-hook.patch first; never compare the inactive production path as Schur.');
const anatomy=await loadCoupledRuntimeAnatomy(),results=[];
try {
 for(const name of ['anatomy-berenstein-feed-138.67-live-cycle','anatomy-pigtail-wire-withdraw-200.27-incoming']) {
  const fixture=JSON.parse(readFileSync(new URL('../../tests/fixtures/shared-axis/'+name+'.json',import.meta.url))),req=fixture.stepRequest,outputs=[];
  for(const projectionMode of [false,'schur']) {
   const input=restoreSharedAxisReplay(fixture,anatomy.field),trace=[];
   const iterator=advanceSharedAxis(input,req.rotations,req.dt,req.tools,{...req.options,promoteTrialAssembly:true,projectionMode,
    observeTrial:e=>{if(e.kind==='trial')trace.push({iteration:e.iteration,trial:e.trial,accept:e.accept,force:e.candidate.force,energy:e.candidate.energy});}});
   let next;do{next=iterator.next();}while(!next.done);
   const {state,result}=next.value;
   outputs.push({trace,state:state?captureSharedAxisReplay(state,fixture.sheath):null,result});
  }
  const [a,b]=outputs,firstDifference=a.trace.findIndex((v,i)=>JSON.stringify(v)!==JSON.stringify(b.trace[i]));
  results.push({name,exactState:JSON.stringify(a.state)===JSON.stringify(b.state),exactTrace:JSON.stringify(a.trace)===JSON.stringify(b.trace),firstDifference,
   referenceAtDifference:a.trace[firstDifference],schurAtDifference:b.trace[firstDifference],
   reference:{status:a.result.status,iterations:a.result.iterations,factorizations:a.result.factorizations,backtracks:a.result.backtracks,ms:a.result.ms},
   schur:{status:b.result.status,iterations:b.result.iterations,factorizations:b.result.factorizations,backtracks:b.result.backtracks,ms:b.result.ms}});
 }
}finally{anatomy.dispose();}
writeFileSync(new URL('schur-parity.json',import.meta.url),JSON.stringify(results,null,2));console.log(JSON.stringify(results,null,2));

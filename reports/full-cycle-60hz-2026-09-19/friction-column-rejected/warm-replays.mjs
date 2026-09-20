import {readFileSync,writeFileSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from '/Users/macpiek/.codex/worktrees/e55c/OpenEndovascularTrainer/tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from '/Users/macpiek/.codex/worktrees/e55c/OpenEndovascularTrainer/tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '/Users/macpiek/.codex/worktrees/e55c/OpenEndovascularTrainer/src/physics/kirchhoffSharedAxisAppSystem.js';
let input;
const anatomy=await loadCoupledRuntimeAnatomy(),outputs=[],variants=[1,.75,.5,.25,0];let fixture;
let baseline;
function run(variant,warmup) {
 const s=restoreSharedAxisReplay(input,anatomy.field),req=input.stepRequest;
 const start=performance.now(),cpu=process.cpuUsage();
 const iterator=advanceSharedAxis(s,req.rotations,req.dt,req.tools,{...req.options,wasmLinearAssembly:true,normalForceColumnScale:variant});
 let next;do{next=iterator.next();}while(!next.done);
 const elapsed=performance.now()-start,used=process.cpuUsage(cpu),{state,result}=next.value;
 if(!state||!result.converged)throw Error(result.status);
 if(!baseline)baseline=state.positions.map(p=>p.slice());
 outputs.push({fixture,variant,warmup,elapsedMs:elapsed,processCpuMs:(used.user+used.system)/1000,linearMs:result.timings.linearMs,assemblyMs:result.timings.assemblyMs,factorizations:result.factorizations,iterations:result.iterations,
  maxPositionDifferenceMm:Math.max(...state.positions.map((p,i)=>Math.hypot(...p.map((v,k)=>v-baseline[i][k]))))});
}
try {
 for(fixture of ['slow-777','slow-842','slow-1300','slow-4245','slow-4895']) {
 input=JSON.parse(readFileSync('/tmp/oet-full-cycle-certified-samples/'+fixture+'.json'));baseline=null;
 for(let i=0;i<2;i++)for(const v of variants)run(v,true);
 for(let i=0;i<4;i++)for(const v of i%2?variants.slice().reverse():variants)run(v,false);
 }
 writeFileSync('/tmp/oet-warm-friction-column.json',JSON.stringify(outputs,null,2));
 for(const f of [...new Set(outputs.map(r=>r.fixture))])for(const v of variants){const r=outputs.filter(x=>x.fixture===f&&x.variant===v&&!x.warmup),median=k=>r.map(x=>x[k]).sort((a,b)=>a-b).slice(1,3).reduce((a,b)=>a+b)/2;console.log(f,v,{medianAssemblyMs:median('assemblyMs'),medianMs:median('elapsedMs'),medianCpuMs:median('processCpuMs'),medianLinearMs:median('linearMs'),factorizations:r[0].factorizations,maxPositionDifferenceMm:Math.max(...r.map(x=>x.maxPositionDifferenceMm))});}
}finally{anatomy.dispose();}

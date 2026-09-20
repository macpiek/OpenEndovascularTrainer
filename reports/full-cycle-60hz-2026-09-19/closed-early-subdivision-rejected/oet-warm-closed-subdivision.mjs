import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {loadCoupledRuntimeAnatomy} from '/Users/macpiek/.codex/worktrees/e55c/OpenEndovascularTrainer/tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from '/Users/macpiek/.codex/worktrees/e55c/OpenEndovascularTrainer/tests/helpers/sharedAxisReplay.js';
import {captureSharedAxisNative} from '/Users/macpiek/.codex/worktrees/e55c/OpenEndovascularTrainer/src/physics/kirchhoffSharedAxisNative.js';
import {advanceSharedAxis} from '/Users/macpiek/.codex/worktrees/e55c/OpenEndovascularTrainer/src/physics/kirchhoffSharedAxisAppSystem.js';
const anatomy=await loadCoupledRuntimeAnatomy(),outputs=[];
function point(s,x){let i=0;while(i+2<s.coordinates.length&&s.coordinates[i+1]<x)i++;const a=s.coordinates[i],b=s.coordinates[i+1],t=(x-a)/(b-a);return s.positions[i].map((v,k)=>(1-t)*v+t*s.positions[i+1][k]+s.origin[k]);}
try {
 for(const fixture of ['slow-1115','slow-1153','failed-incoming']){
  const input=JSON.parse(readFileSync('/tmp/oet-closed-baseline/'+fixture+'.json'));let baseline;
  for(let trial=0;trial<3;trial++)for(const enabled of trial%2?[true,false]:[false,true]){
   const s=restoreSharedAxisReplay(input,anatomy.field),before=captureSharedAxisNative(s),q=input.stepRequest;
   const start=performance.now(),it=advanceSharedAxis(s,q.rotations,q.dt,q.tools,{...q.options,earlySubdivision:enabled});let n;do{n=it.next();}while(!n.done);
   const ms=performance.now()-start,{state,result:r}=n.value;assert.deepEqual(captureSharedAxisNative(s),before);
   if(!enabled&&state)baseline=state;
   let maxDelta=null;if(baseline&&state){maxDelta=0;for(let x=0;x<=state.coordinates.at(-1);x+=1){const a=point(baseline,x),b=point(state,x);maxDelta=Math.max(maxDelta,Math.hypot(...a.map((v,k)=>v-b[k])));}}
   if(state){assert.ok(r.certificateBound<=q.options.forceTolerance);assert.ok(r.residual.length<=q.options.lengthTolerance);assert.ok(r.quality.finite);assert.ok(r.quality.maxPenetration<=q.options.lengthTolerance);}
   const row={fixture,enabled,trial,ms,status:r.status,converged:r.converged,subdivisions:r.subdivisions,factorizations:r.factorizations,iterations:r.iterations,assemblies:r.fullAssemblies+r.residualAssemblies,maxPositionDeltaMm:maxDelta,force:r.certificateBound,length:r.residual?.length,penetration:r.quality?.maxPenetration};outputs.push(row);console.log(JSON.stringify(row));
  }
 }
}finally{writeFileSync('/tmp/oet-warm-closed-subdivision.json',JSON.stringify(outputs,null,2));anatomy.dispose();}

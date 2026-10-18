// Isolated contact-basis timing on captured anatomical Newton rows.
// Alternating versions, four warmups and exact reactions/pivots comparison.
// This does not measure complete accepted physics steps or browser FPS.
import fs from 'node:fs';import assert from 'node:assert/strict';import {gunzipSync} from 'node:zlib';import {pathToFileURL,fileURLToPath} from 'node:url';import {resolve} from 'node:path';
if(!process.argv[2])throw new Error('Pass the baseline checkout path');
const root=fileURLToPath(new URL('../',import.meta.url)),baseline=resolve(process.argv[2])+'/';
const output=process.argv[3]??root+'reports/performance-scenarios/001-basis-scalar-isolated.json';
const afterOptions=JSON.parse(process.argv[4]??'{}');
const api=async(p,f)=>import(pathToFileURL(p+f));
const {loadCoupledRuntimeAnatomy}=await api(root,'tests/helpers/coupledRuntimeFixture.js');
const {restoreSharedAxisReplay}=await api(root,'tests/helpers/sharedAxisReplay.js');
const {advanceSharedAxis}=await api(root,'src/physics/kirchhoffSharedAxisAppSystem.js');
const versions=await Promise.all([baseline,root].map(async p=>(await api(p,'src/physics/kirchhoffSharedAxisActiveBasis.js')).prepareSharedAxisActiveBasis));
const fixture=JSON.parse(gunzipSync(fs.readFileSync(root+'reports/performance-scenarios/001-stage-05-replay.json.gz'))),req=fixture.stepRequest,anatomy=await loadCoupledRuntimeAnatomy(undefined,fixture.anatomy),snapshots=[];
try {
 const state=restoreSharedAxisReplay({...fixture,contactCandidateMargin:.01},anatomy.field);
 const it=advanceSharedAxis(state,req.rotations,req.dt,req.tools,{...req.options,contactCandidateMargin:.01,lazyBasisCoefficients:true,deferActiveBasis:true,
  observeIteration:({state,base})=>snapshots.push({fixed:state.fixed.slice(),rows:base.rows.map(r=>({kind:r.kind,id:r.id,dofs:r.dofs.slice(),jacobian:r.jacobian.slice(),gap:r.gap,multiplier:r.multiplier,compliance:r.compliance}))})});
 let n;do{n=it.next();}while(!n.done);assert.ok(n.value.state);
 const samples=[[],[]];
 for(let repeat=0;repeat<24;repeat++)for(const snapshot of snapshots) {
  const outputs=[];
  for(const index of repeat%2?[1,0]:[0,1]) {
   const {rows,fixed}=snapshot,activeSet=Uint8Array.from(rows,r=>r.kind==='length'||r.multiplier>1e-8),dual=Float64Array.from(rows,r=>r.kind==='wall'?Math.max(0,r.multiplier):r.multiplier),trace=[];
   const start=performance.now(),result=versions[index]({rows,fixed,activeSet,dual,trace,lazyBasisCoefficients:true,basisCache:Symbol('new-linearization'),...(index===1?afterOptions:{})});
   if(repeat>=4)samples[index].push(performance.now()-start);outputs[index]={result,activeSet,dual,trace};
  }
  assert.deepEqual(outputs[1],outputs[0]);
 }
 const stats=samples.map(a=>({runs:a.length,totalMs:a.reduce((s,v)=>s+v,0),medianMs:a.sort((a,b)=>a-b)[a.length>>1]}));
 const report={recordedAt:new Date().toISOString(),passed:true,afterOptions,snapshots:snapshots.length,before:stats[0],after:stats[1],scope:'Dependency preparation only on actual first-step crossover Newton rows; every repetition rebuilds the Jacobian basis, exact reactions/active set/trace comparison; not full-step throughput'};
 fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');console.log(report);
}finally{anatomy.dispose();}

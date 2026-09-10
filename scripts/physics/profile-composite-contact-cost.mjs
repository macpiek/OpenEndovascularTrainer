import assert from 'node:assert/strict';
import { Session } from 'node:inspector/promises';
import { writeFileSync } from 'node:fs';
import { pathToFileURL } from 'node:url';
import { resolve } from 'node:path';

const root=process.argv[2], output=process.argv[3];
const load=p=>import(pathToFileURL(resolve(root,p)));
const {mechanicalMeshFixture,mechanicalMeshOptions,gradedMechanicalNodes}=await load('tests/fixtures/compositeMechanicalMesh.js');
const {createCompositeJointTimeStepWorkspace,advanceCompositeJointTimeStep}=await load('src/physics/kirchhoffCompositeJointTimeStep.js');
const f=mechanicalMeshFixture(gradedMechanicalNodes),workspace=createCompositeJointTimeStepWorkspace(f.state);
const run=()=>advanceCompositeJointTimeStep(f.state,mechanicalMeshOptions(f,f.state,{force:[.1,.4,0],forceNode:0,workspace}));
for(let i=0;i<30;i++)assert.equal(run().accepted,true);
const session=new Session();session.connect();
await session.post('Profiler.enable');
await session.post('Profiler.setSamplingInterval',{interval:500});
await session.post('Profiler.start');
const measurements=[];
for(let i=0;i<200;i++){
  const r=run();assert.equal(r.accepted,true);assert.equal(r.diagnostics.certificate.converged,true);
  measurements.push({totalMs:r.diagnostics.totalMs,preparationMs:r.diagnostics.preparationMs,directionMs:r.diagnostics.directionMs,evaluations:r.diagnostics.evaluations});
}
const {profile}=await session.post('Profiler.stop');session.disconnect();
writeFileSync(output+'.cpuprofile',JSON.stringify(profile));
const parents=new Map(),nodes=new Map(profile.nodes.map(n=>[n.id,n]));
for(const n of profile.nodes)for(const c of n.children??[])parents.set(c,n.id);
const key=n=>{const f=n.callFrame;return `${f.url.replace(root+'/','')}:${f.lineNumber+1} ${f.functionName||'(anonymous)'}`;};
const self=new Map(),inclusive=new Map();let totalUs=0;
for(let i=0;i<profile.samples.length;i++){
  const us=profile.timeDeltas[i],id=profile.samples[i];totalUs+=us;
  const k=key(nodes.get(id));self.set(k,(self.get(k)??0)+us);
  const seen=new Set();let cur=id;
  while(cur!==undefined){const k=key(nodes.get(cur));if(!seen.has(k)){inclusive.set(k,(inclusive.get(k)??0)+us);seen.add(k);}cur=parents.get(cur);}
}
const top=m=>[...m].sort((a,b)=>b[1]-a[1]).slice(0,50).map(([frame,us])=>({frame,ms:us/1000,percent:100*us/totalUs}));
const stats=key=>{const x=measurements.map(m=>m[key]).sort((a,b)=>a-b);return {median:x[100],p95:x[189],mean:x.reduce((a,b)=>a+b,0)/x.length};};
const summary={scope:'200 repeated accepted synthetic whole steps from the same initial state, after 30 warmup steps; CPU sampling adds overhead; not sequential feed, anatomy, or browser FPS',totalMs:totalUs/1000,samples:profile.samples.length,stats:Object.fromEntries(['totalMs','preparationMs','directionMs','evaluations'].map(k=>[k,stats(k)])),self:top(self),inclusive:top(inclusive)};
writeFileSync(output+'.json',JSON.stringify(summary,null,2)+'\n');
console.log(JSON.stringify(summary,null,2));

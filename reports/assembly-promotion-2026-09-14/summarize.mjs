import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=new URL('./',import.meta.url),names=['reference-a','promoted-a','promoted-b','reference-b'];
const profiles=Object.fromEntries(names.map(name=>[name,JSON.parse(readFileSync(new URL(name+'/profile.json',root)))]));
const omitted=new Set(['ms','totalMs','timings','fullAssemblies','residualAssemblies','promotedAssemblies','linearScratch']);
function physical(value){
 if(Array.isArray(value))return value.map(physical);
 if(value&&typeof value==='object')return Object.fromEntries(Object.entries(value).filter(([k])=>!omitted.has(k)).map(([k,v])=>[k,physical(v)]));
 return value;
}
const reference=profiles['reference-a'],terminal=readFileSync(new URL('reference-a/terminal.json',root));
const terminalHash=createHash('sha256').update(terminal).digest('hex');
const runs={};
for(const [name,p] of Object.entries(profiles)){
 assert.equal(p.failed,false,name+' failed');
 assert.deepEqual(physical(p.samples),physical(reference.samples),name+' physical diagnostics and decisions');
 assert.equal(createHash('sha256').update(readFileSync(new URL(name+'/terminal.json',root))).digest('hex'),terminalHash,name+' terminal state');
 const phases={};
 for(const phase of ['wire','catheter']){
  const rows=p.samples.filter(s=>s.phase===phase),sorted=rows.map(s=>s.totalMs).sort((a,b)=>a-b),mean=f=>rows.reduce((sum,s)=>sum+f(s),0)/rows.length;
  phases[phase]={samples:rows.length,meanMs:mean(s=>s.totalMs),p95Ms:sorted[Math.ceil(sorted.length*.95)-1],p99Ms:sorted[Math.ceil(sorted.length*.99)-1],maxMs:sorted.at(-1),assemblyMs:mean(s=>s.timings.assemblyMs),linearMs:mean(s=>s.timings.linearMs),frictionMs:mean(s=>s.timings.frictionMs??0),fullAssemblies:mean(s=>s.fullAssemblies),residualAssemblies:mean(s=>s.residualAssemblies),promotedAssemblies:p.samples.some(s=>'promotedAssemblies' in s)?mean(s=>s.promotedAssemblies??0):null};
 }
 const worst=p.samples.toSorted((a,b)=>b.totalMs-a.totalMs)[0];
 runs[name]={sourceTreeHash:p.sourceTreeHash,phases,worst:{wire:worst.wire,catheter:worst.catheter,totalMs:worst.totalMs,iterations:worst.iterations,factorizations:worst.factorizations,backtracks:worst.backtracks,wallNormalFallbacks:worst.wallNormalFallbacks}};
}
const pooled={};
for(const phase of ['wire','catheter']){
 const a=(runs['reference-a'].phases[phase].meanMs+runs['reference-b'].phases[phase].meanMs)/2;
 const b=(runs['promoted-a'].phases[phase].meanMs+runs['promoted-b'].phases[phase].meanMs)/2;
 pooled[phase]={referenceMeanMs:a,promotedMeanMs:b,timeReductionPercent:100*(1-b/a),speedup:a/b};
}
const summary={scope:reference.timingScope,terminalHash,exactPhysicalSampleParity:true,samplesPerRun:reference.samples.length,runs,pooled};
writeFileSync(new URL('summary.json',root),JSON.stringify(summary,null,2)+'\n');
console.log(JSON.stringify({pooled,exactPhysicalSampleParity:true,terminalHash},null,2));

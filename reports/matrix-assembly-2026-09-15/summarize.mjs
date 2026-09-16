import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
const root=new URL('./',import.meta.url),name=process.argv[2]??'paired';
const data=JSON.parse(readFileSync(new URL(`${name}/profile.json`,root)));
const stats=values=>{const a=[...values].sort((x,y)=>x-y),n=a.length;return {mean:values.reduce((x,y)=>x+y,0)/n,
    median:(a[Math.floor((n-1)/2)]+a[Math.ceil((n-1)/2)])/2,p95:a[Math.floor(.95*(n-1))],p99:a[Math.floor(.99*(n-1))],max:a.at(-1)};};
const invariants=['quality','residual','certificateBound','subdivisions','interToolRows','iterations','factorizations','backtracks',
    'geometryRestarts','substepAttempts','fullAssemblies','residualAssemblies','promotedAssemblies','wallNormalFallback','stagnationRecovery'];
for(const s of data.samples)for(const key of invariants)assert.deepEqual(s[key],s.paired.optimizedResult[key],`${s.phase}/${s.catheter}/${key}`);
const summary={failed:data.failed,pairs:data.samples.length,exactStates:data.samples.every(s=>s.paired.exactState),sameDecisions:true,phases:{}};
for(const phase of [...new Set(data.samples.map(s=>s.phase))]) {
    const s=data.samples.filter(s=>s.phase===phase),a=s.map(s=>s.paired.referenceMs),b=s.map(s=>s.paired.optimizedMs);
    const mean=vs=>vs.reduce((x,y)=>x+y,0)/vs.length;
    summary.phases[phase]={count:s.length,referenceMs:stats(a),optimizedMs:stats(b),meanReductionPercent:100*(1-mean(b)/mean(a)),
        pairedSavingMs:stats(a.map((x,i)=>x-b[i])),
        stages:Object.fromEntries(['assemblyMs','tangentAssemblyMs','residualAssemblyMs','linearMs','frictionMs'].map(k=>[k,
            {referenceMean:mean(s.map(x=>x.timings[k])),optimizedMean:mean(s.map(x=>x.paired.optimizedResult.timings[k]))}])),
        counters:Object.fromEntries(['iterations','factorizations','backtracks'].map(k=>[k,s.reduce((a,x)=>a+x[k],0)])),
        maxCertificate:Math.max(...s.map(x=>x.certificateBound??0)),maxPenetration:Math.max(...s.map(x=>x.quality?.maxPenetration??0))};
}
writeFileSync(new URL(`${name}-summary.json`,root),JSON.stringify(summary,null,2));console.log(JSON.stringify(summary,null,2));

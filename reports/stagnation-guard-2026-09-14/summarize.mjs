import {readFileSync,writeFileSync} from 'node:fs';
const root=new URL('./',import.meta.url),name=process.argv[2]??'paired';
const data=JSON.parse(readFileSync(new URL(`${name}/profile.json`,root)));
const stats=values=>{const sorted=[...values].sort((a,b)=>a-b);return {
    mean:values.reduce((a,b)=>a+b,0)/values.length,median:sorted[Math.floor(sorted.length/2)],
    p95:sorted[Math.floor(.95*(sorted.length-1))],p99:sorted[Math.floor(.99*(sorted.length-1))],max:sorted.at(-1)};};
const summary={failed:data.failed,pairs:data.samples.length,exactStates:data.samples.every(s=>s.paired.exactState),phases:{}};
for(const phase of [...new Set(data.samples.map(s=>s.phase))]) {
    const samples=data.samples.filter(s=>s.phase===phase),optimized=samples.map(s=>s.paired.optimizedResult);
    const counts=key=>[samples,optimized].map(list=>list.reduce((sum,s)=>sum+(s[key]??0),0));
    summary.phases[phase]={count:samples.length,referenceMs:stats(samples.map(s=>s.paired.referenceMs)),optimizedMs:stats(samples.map(s=>s.paired.optimizedMs)),
        counts:Object.fromEntries(['iterations','factorizations','backtracks','fullAssemblies','residualAssemblies','geometryRestarts','substepAttempts'].map(key=>[key,counts(key)])),
        maxCertificate:Math.max(...optimized.map(s=>s.certificateBound??0)),maxPenetration:Math.max(...optimized.map(s=>s.quality?.maxPenetration??0)),
        recoveries:optimized.filter(s=>s.stagnationRecovery).length,
        changed:samples.filter(s=>s.iterations!==s.paired.optimizedResult.iterations).map(s=>({wire:s.wire,catheter:s.catheter,
            iterations:[s.iterations,s.paired.optimizedResult.iterations],factorizations:[s.factorizations,s.paired.optimizedResult.factorizations],
            backtracks:[s.backtracks,s.paired.optimizedResult.backtracks],ms:[s.paired.referenceMs,s.paired.optimizedMs]}))};
}
writeFileSync(new URL(`${name}-summary.json`,root),JSON.stringify(summary,null,2));console.log(JSON.stringify(summary,null,2));

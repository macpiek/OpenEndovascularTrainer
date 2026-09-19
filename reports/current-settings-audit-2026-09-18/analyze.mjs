import {readFileSync,writeFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';

const directory=process.argv[2];
const read=name=>{
    try{return JSON.parse(readFileSync(`${directory}/${name}`));}
    catch(e){if(e.code!=='ENOENT')throw e;return JSON.parse(gunzipSync(readFileSync(`${directory}/${name}.gz`)));}
};
const profile=read('profile.json');
const mean=values=>values.reduce((a,b)=>a+b,0)/values.length;
const phases={};
for(const phase of new Set(profile.samples.map(s=>s.phase))) {
    const rows=profile.samples.filter(s=>s.phase===phase),times=rows.map(s=>s.totalMs).sort((a,b)=>a-b);
    const timing=key=>mean(rows.map(s=>s.timings[key]??0));
    const total=mean(times),assembly=timing('assemblyMs'),linear=timing('linearMs'),friction=timing('frictionMs');
    phases[phase]={steps:rows.length,rejected:rows.filter(s=>!s.converged).length,meanMs:total,p95Ms:times[Math.ceil(times.length*.95)-1],maxMs:times.at(-1),
        stages:Object.fromEntries(Object.entries({assembly,linear,friction,other:total-assembly-linear-friction}).map(([k,ms])=>[k,{ms,percent:100*ms/total}])),
        projectionIncludedInLinearMs:timing('projectionMs'),
        averages:Object.fromEntries(['iterations','factorizations','fullAssemblies','residualAssemblies','backtracks','geometryRestarts','frictionIterations','wallNormalFallbacks','substepAttempts','mechanicalNodes','mechanicalDofs'].map(k=>[k,mean(rows.map(s=>s[k]??0))]))};
}
let cpu=null;
try {
    const p=read('catheter.cpuprofile'),nodes=new Map(p.nodes.map(n=>[n.id,n])),parents=new Map();
    for(const n of p.nodes)for(const id of n.children??[])parents.set(id,n.id);
    const categories={},functions={};let total=0;
    for(let i=0;i<p.samples.length;i++) {
        const stack=[];let id=p.samples[i];
        while(id!==undefined){const f=nodes.get(id).callFrame;stack.push(`${f.functionName}@${f.url.split('/').at(-1)}`);id=parents.get(id);}
        const us=p.timeDeltas[i],has=prefix=>stack.some(s=>s.startsWith(prefix));total+=us;
        for(const name of new Set(stack))functions[name]=(functions[name]??0)+us;
        // Mutually exclusive groups. Inclusive function percentages below overlap.
        let category='other';
        if(has('assembleSharedAxisNative@'))category=has('sample@kirchhoffSharedAxisVesselWitnesses')?'assembly/discovery':has('gapForWitness@')?'assembly/retainedGeometry':
            has('assembleSharedAxisMaterialTangent@')||has('assembleSharedAxisGaussNewton@')?'assembly/material':has('assembleSharedAxisWallFriction@')?'assembly/friction':has('assembleSharedAxisInertia@')?'assembly/inertia':'assembly/other';
        else if(has('iterateSharedAxisLinear@')||has('iterateSharedAxisProjection@'))category='linear';
        else if(has('feedSharedAxisNative@'))category='feed/remesh';
        else if(has('(garbage collector)@'))category='garbage collector';
        categories[category]=(categories[category]??0)+us;
    }
    const summarize=entries=>Object.entries(entries).sort((a,b)=>b[1]-a[1]).map(([name,us])=>({name,ms:us/1000,percent:100*us/total}));
    cpu={phase:'catheter',sampledMs:total/1000,exclusiveCategories:summarize(categories),inclusiveFunctions:summarize(functions).slice(0,40)};
}catch(e){if(e.code!=='ENOENT')throw e;}
const result={scope:'Separate Node trajectory with matching controls, not a replay of the browser pose/history. Includes failed attempts; no rendering.',failed:profile.failed,parameters:profile.modelParameters,phases,cpu};
writeFileSync(`${directory}/summary.json`,JSON.stringify(result,null,2));
console.log(JSON.stringify({failed:result.failed,phases,cpu},null,2));

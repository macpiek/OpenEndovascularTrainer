import {readFileSync,writeFileSync} from 'node:fs';
const [referencePath,experimentPath,outputPath]=process.argv.slice(2);
if(!outputPath)throw new Error('Usage: compare-modified-newton.mjs reference/profile.json experiment/profile.json summary.json');
const r=JSON.parse(readFileSync(referencePath)),e=JSON.parse(readFileSync(experimentPath));
const parameters=p=>{const v=structuredClone(p.modelParameters);delete v.options.modifiedNewton;return JSON.stringify(v);};
if(parameters(r)!==parameters(e))throw new Error('Different physical settings');
if(r.failed||e.failed||r.samples.length!==e.samples.length)throw new Error('Incomplete trajectory');
const groups={};
r.samples.forEach((a,i)=>{
    const b=e.samples[i];if(a.phase!==b.phase||a.wire!==b.wire||a.catheter!==b.catheter)throw new Error('Different commands');
    const error=Math.max(...a.toolTips.map((t,j)=>Math.hypot(...t.position.map((v,k)=>v-b.toolTips[j].position[k]))));
    (groups[a.phase]??=[]).push({a,b,error});
});
const phases=Object.fromEntries(Object.entries(groups).map(([phase,ss])=>{
    const stats=side=>{
        const xs=ss.map(s=>s[side]),ms=xs.map(s=>s.totalMs).sort((a,b)=>a-b);
        return {meanMs:ms.reduce((a,b)=>a+b,0)/ms.length,p95Ms:ms[Math.floor((ms.length-1)*.95)],
            ...Object.fromEntries(['iterations','fullAssemblies','factorizations','modifiedAttempts','modifiedAccepted','modifiedFallbacks'].map(k=>[k,xs.reduce((sum,s)=>sum+(s[k]??0),0)]))};
    };
    const reference=stats('a'),experiment=stats('b');
    return [phase,{steps:ss.length,reference,experiment,timeReductionPercent:100*(1-experiment.meanMs/reference.meanMs),maxTipDeviationMm:Math.max(...ss.map(s=>s.error))}];
}));
const summary={note:'One sequential Node pair of independent trajectories. Same mesh/tolerances/material settings; only modifiedNewton differs. Timings are not browser FPS or confidence intervals.',
    steps:e.samples.length,parameters:r.modelParameters,referenceSourceHash:r.sourceTreeHash,experimentSourceHash:e.sourceTreeHash,
    maxTipDeviationMm:Math.max(...Object.values(phases).map(p=>p.maxTipDeviationMm)),
    allFinite:e.samples.every(s=>s.quality.finite),maxPenetrationMm:Math.max(...e.samples.map(s=>s.quality.maxPenetration)),
    maxCertifiedResidual:Math.max(...e.samples.map(s=>s.certificateBound)),phases};
writeFileSync(outputPath,JSON.stringify(summary,null,2)+'\n');console.log(JSON.stringify(summary,null,2));

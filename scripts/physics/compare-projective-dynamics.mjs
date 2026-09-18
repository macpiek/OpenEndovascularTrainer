import {readFileSync,writeFileSync} from 'node:fs';
const [referencePath,experimentPath,outputPath]=process.argv.slice(2);
if(!outputPath)throw new Error('Usage: compare-projective-dynamics.mjs reference/profile.json experiment/profile.json summary.json');
const reference=JSON.parse(readFileSync(referencePath)),experiment=JSON.parse(readFileSync(experimentPath));
const settings=p=>{const v=structuredClone(p.modelParameters);delete v.options.projectiveDynamics;delete v.options.projective;return JSON.stringify(v);};
if(settings(reference)!==settings(experiment))throw new Error('Different input settings');
if(reference.failed||experiment.failed||reference.samples.length!==experiment.samples.length)throw new Error('Incomplete trajectory');
const groups={};
reference.samples.forEach((a,i)=>{
    const b=experiment.samples[i];if(a.phase!==b.phase||a.wire!==b.wire||a.catheter!==b.catheter)throw new Error('Different commands');
    const error=Math.max(...a.toolTips.map(t=>{const other=b.toolTips.find(v=>v.id===t.id);return Math.hypot(...t.position.map((v,k)=>v-other.position[k]));}));
    (groups[a.phase]??=[]).push({a,b,error});
});
const phases=Object.fromEntries(Object.entries(groups).map(([phase,samples])=>{
    const stats=side=>{
        const xs=samples.map(s=>s[side]),ms=xs.map(s=>s.totalMs).sort((a,b)=>a-b);
        return {meanMs:ms.reduce((a,b)=>a+b,0)/ms.length,p95Ms:ms[Math.floor((ms.length-1)*.95)],
            ...Object.fromEntries(['iterations','factorizations','substepAttempts'].map(k=>[k,xs.reduce((sum,s)=>sum+(s[k]??0),0)]))};
    };
    const r=stats('a'),e=stats('b');
    return [phase,{steps:samples.length,reference:r,projective:e,timeReductionPercent:100*(1-e.meanMs/r.meanMs),maxTipDeviationMm:Math.max(...samples.map(s=>s.error))}];
}));
const summary={note:'One sequential Node pair, independent trajectories and different rod/friction models. Identical input controls and adaptive mesh budgets; timings exclude rendering and are not FPS or confidence intervals.',
    steps:experiment.samples.length,referenceParameters:reference.modelParameters,projectiveParameters:experiment.modelParameters,
    referenceSourceHash:reference.sourceTreeHash,projectiveSourceHash:experiment.sourceTreeHash,
    maxTipDeviationMm:Math.max(...Object.values(phases).map(p=>p.maxTipDeviationMm)),allFinite:experiment.samples.every(s=>s.quality.finite),
    maxPenetrationMm:Math.max(...experiment.samples.map(s=>s.quality.maxPenetration)),
    maxLengthErrorPercent:100*Math.max(...experiment.samples.flatMap(s=>s.quality.bodies.map(b=>b.maxLengthError))),
    maxShear:Math.max(...experiment.samples.map(s=>s.pd.maxShear)),iterationBudgetSteps:experiment.samples.filter(s=>!s.pd.localGlobalConverged).length,
    totalReferenceMs:reference.samples.reduce((sum,s)=>sum+s.totalMs,0),totalProjectiveMs:experiment.samples.reduce((sum,s)=>sum+s.totalMs,0),phases};
writeFileSync(outputPath,JSON.stringify(summary,null,2)+'\n');console.log(JSON.stringify(summary,null,2));

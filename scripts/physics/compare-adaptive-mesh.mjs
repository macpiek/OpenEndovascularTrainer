import {readFileSync,writeFileSync} from 'node:fs';

const [referencePath,adaptivePath,outputPath]=process.argv.slice(2);
if(!referencePath||!adaptivePath||!outputPath)throw new Error('Usage: node scripts/physics/compare-adaptive-mesh.mjs reference/profile.json adaptive/profile.json summary.json');
const reference=JSON.parse(readFileSync(referencePath)),adaptive=JSON.parse(readFileSync(adaptivePath));
if(reference.failed||adaptive.failed)throw new Error('Cannot certify a comparison containing a rejected step');
if(reference.samples.length!==adaptive.samples.length)throw new Error('Different trajectory lengths');
const rows=reference.samples.map((r,i)=>{
    const a=adaptive.samples[i];
    if(r.phase!==a.phase||r.wire!==a.wire||r.catheter!==a.catheter)throw new Error(`Different commands at step ${i}`);
    const errors=r.toolTips.map((t,j)=>{
        if(t.id!==a.toolTips[j].id)throw new Error('Different tool order');
        return Math.hypot(...t.position.map((v,k)=>v-a.toolTips[j].position[k]));
    });
    return {phase:r.phase,referenceMs:r.totalMs,adaptiveMs:a.totalMs,
        referenceNodes:r.mechanicalNodes,adaptiveNodes:a.mechanicalNodes,
        referenceDofs:r.mechanicalDofs,adaptiveDofs:a.mechanicalDofs,
        tipDeviationMm:Math.max(...errors),penetrationMm:a.quality.maxPenetration,
        relativeLengthError:Math.max(...a.quality.bodies.map(b=>b.maxLengthError)),
        residual:a.certificateBound,finite:a.quality.finite};
});
const mean=(samples,key)=>samples.reduce((sum,s)=>sum+s[key],0)/samples.length;
const p95=(samples,key)=>samples.map(s=>s[key]).sort((a,b)=>a-b)[Math.floor((samples.length-1)*.95)];
const maximum=(samples,key)=>Math.max(...samples.map(s=>s[key]));
const summary={
    note:'One sequential Node comparison of independent trajectories, no rendering. Both grid and nonlinear tolerances differ. Not a browser FPS result or a global error guarantee.',
    steps:rows.length,allFinite:rows.every(r=>r.finite),
    reference:{sourceTreeHash:reference.sourceTreeHash,parameters:reference.modelParameters},
    adaptive:{sourceTreeHash:adaptive.sourceTreeHash,parameters:adaptive.modelParameters},
    maxTipDeviationMm:maximum(rows,'tipDeviationMm'),maxPenetrationMm:maximum(rows,'penetrationMm'),
    maxRelativeLengthError:maximum(rows,'relativeLengthError'),maxCertifiedResidual:maximum(rows,'residual'),
    phases:Object.fromEntries([...new Set(rows.map(s=>s.phase))].map(phase=>{
        const samples=rows.filter(s=>s.phase===phase);
        return [phase,{steps:samples.length,referenceMeanMs:mean(samples,'referenceMs'),adaptiveMeanMs:mean(samples,'adaptiveMs'),
            timeReductionPercent:100*(1-mean(samples,'adaptiveMs')/mean(samples,'referenceMs')),
            referenceP95Ms:p95(samples,'referenceMs'),adaptiveP95Ms:p95(samples,'adaptiveMs'),
            referenceMeanNodes:mean(samples,'referenceNodes'),adaptiveMeanNodes:mean(samples,'adaptiveNodes'),
            referenceMeanDofs:mean(samples,'referenceDofs'),adaptiveMeanDofs:mean(samples,'adaptiveDofs'),
            maxTipDeviationMm:maximum(samples,'tipDeviationMm')}];
    }))
};
writeFileSync(outputPath,JSON.stringify(summary,null,2)+'\n');
console.log(JSON.stringify({steps:summary.steps,allFinite:summary.allFinite,maxTipDeviationMm:summary.maxTipDeviationMm,phases:summary.phases},null,2));

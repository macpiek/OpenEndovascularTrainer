import {readFileSync,writeFileSync} from 'node:fs';import {gunzipSync} from 'node:zlib';
const file=process.argv[2],p=JSON.parse(file.endsWith('.gz')?gunzipSync(readFileSync(file)):readFileSync(file)),phases={};
for(const phase of ['all',...new Set(p.samples.map(s=>s.phase))]) {
 const ss=p.samples.filter(s=>phase==='all'?s.phase!=='initial':s.phase===phase),avg=(m,key)=>ss.reduce((s,x)=>s+(key==='assemblies'?x[m].fullAssemblies+x[m].residualAssemblies:x[m][key]??0),0)/ss.length;
 const metrics=Object.fromEntries(['ms','iterations','factorizations','assemblies','frictionIterations','retainedDiscoveryTrials','coupledFrictionRefreshes','coupledFrictionFallbacks'].map(k=>{const a=avg('reference',k),b=avg('optimized',k);return[k,{reference:a,optimized:b,reductionPercent:a?100*(1-b/a):null}];}));
 const p95=m=>ss.map(x=>x[m].ms).sort((a,b)=>a-b)[Math.ceil(ss.length*.95)-1];
 phases[phase]={steps:ss.length,metrics,p95:{reference:p95('reference'),optimized:p95('optimized')},maxShapeDeviationMm:Math.max(...ss.map(x=>x.maxShapeDeviationMm??0)),rejected:ss.filter(x=>!x.reference.converged||!x.optimized.converged).length,
    maxCertificateBound:Math.max(...ss.map(x=>x.optimized.certificateBound??0)),maxLengthResidual:Math.max(...ss.map(x=>x.optimized.residual?.length??0)),maxPenetration:Math.max(...ss.map(x=>x.optimized.quality?.maxPenetration??0))};
}
const summary={independent:p.independent,failed:p.failed,phases};writeFileSync(file.replace(/profile.json(?:.gz)?$/,'summary.json'),JSON.stringify(summary,null,2));
console.log(JSON.stringify(summary,null,2));

import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
const read=p=>JSON.parse(p.endsWith('.gz')?gunzipSync(readFileSync(p)):readFileSync(p));
const mean=xs=>xs.reduce((a,b)=>a+b,0)/xs.length;
const p95=xs=>xs.slice().sort((a,b)=>a-b)[Math.ceil(xs.length*.95)-1];
const results=[];
for(const path of process.argv.slice(2)) {
    const p=read(path);assert.equal(p.failed,false);assert.ok(p.samples.every(s=>s.converged));
    const phases={};
    for(const phase of new Set(p.samples.map(s=>s.phase))) {
        const ss=p.samples.filter(s=>s.phase===phase),paired=!!ss[0].paired;
        const r=ss.map(s=>s.paired?.referenceMs??s.totalMs),e=ss.map(s=>s.paired?.optimizedMs??s.totalMs);
        if(p.pairedIndexedBasis)for(const s of ss) {
            assert.equal(s.paired.exactState,true);
            for(const k of ['quality','residual','certificateBound','iterations','factorizations','backtracks','geometryRestarts','frictionIterations'])
                assert.deepEqual(s.paired.optimizedResult[k],s[k],k);
        }
        const optimized=ss.map(s=>s.paired?.optimizedResult??s);
        phases[phase]={steps:ss.length,referenceMs:mean(r),optimizedMs:mean(e),reductionPercent:100*(1-mean(e)/mean(r)),
            referenceP95Ms:p95(r),optimizedP95Ms:p95(e),meanNodes:mean(ss.map(s=>s.mechanicalNodes)),
            ...(paired?{maxShapeDeviationMm:Math.max(...ss.map(s=>s.paired.maxShapeDeviationMm??0)),
                referenceDefinitions:mean(ss.map(s=>s.paired.referenceDefinitions??0)),optimizedDefinitions:mean(ss.map(s=>s.paired.optimizedDefinitions??0))}:{}),
            maxPenetrationMm:Math.max(...optimized.map(s=>s.quality.maxPenetration)),maxCertificate:Math.max(...optimized.map(s=>s.certificateBound)),
            referenceIterations:mean(ss.map(s=>s.iterations)),optimizedIterations:mean(optimized.map(s=>s.iterations))};
    }
    const movement=p.samples.filter(s=>s.phase!=='initial');
    const referenceMs=movement.reduce((a,s)=>a+(s.paired?.referenceMs??s.totalMs),0),optimizedMs=movement.reduce((a,s)=>a+(s.paired?.optimizedMs??s.totalMs),0);
    const {samples,...metadata}=p;
    results.push({path,metadata,steps:samples.length,phases,movement:{referenceMs,optimizedMs,reductionPercent:100*(1-optimizedMs/referenceMs)}});
}
console.log(JSON.stringify(results,null,2));

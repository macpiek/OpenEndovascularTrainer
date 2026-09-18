import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';

const mean = values => values.reduce((a,b)=>a+b,0)/values.length;
const quantile = (values,q) => [...values].sort((a,b)=>a-b)[Math.ceil(values.length*q)-1];
const reports=[];
for(const path of process.argv.slice(2)) {
    const raw=readFileSync(path),p=JSON.parse(path.endsWith('.gz')?gunzipSync(raw):raw);
    assert.equal(p.failed,false);
    const phases={};
    for(const s of p.samples) {
        assert.equal(s.converged,true);
        assert.ok(s.paired?.exactState,'Profiler must compare complete serialized states');
        for(const key of ['converged','status','friction','wallNormalFallbacks','quality','residual','certificateBound','iterations','factorizations','backtracks',
            'geometryRestarts','frictionIterations','substepAttempts','fullAssemblies','residualAssemblies'])
            assert.deepEqual(s.paired.optimizedResult[key],s[key],`${s.phase}: ${key}`);
    }
    for(const phase of new Set(p.samples.map(s=>s.phase))) {
        const samples=p.samples.filter(s=>s.phase===phase),reference=samples.map(s=>s.paired.referenceMs),optimized=samples.map(s=>s.paired.optimizedMs);
        phases[phase]={steps:samples.length,referenceMs:mean(reference),optimizedMs:mean(optimized),
            reductionPercent:100*(1-mean(optimized)/mean(reference)),
            medianPairSavingMs:quantile(reference.map((v,i)=>v-optimized[i]),.5),
            referenceP95Ms:quantile(reference,.95),optimizedP95Ms:quantile(optimized,.95),
            referenceAssemblyMs:mean(samples.map(s=>s.timings.assemblyMs)),
            optimizedAssemblyMs:mean(samples.map(s=>s.paired.optimizedResult.timings.assemblyMs)),
            referenceRefreshMs:mean(samples.map(s=>s.timings.frictionMs)),
            optimizedRefreshMs:mean(samples.map(s=>s.paired.optimizedResult.timings.frictionMs))};
    }
    const {samples,...metadata}=p;
    const movement=samples.filter(s=>s.phase!=='initial');
    const referenceMs=movement.reduce((v,s)=>v+s.paired.referenceMs,0),optimizedMs=movement.reduce((v,s)=>v+s.paired.optimizedMs,0);
    reports.push({metadata,steps:samples.length,exactStates:true,phases,movement:{referenceMs,optimizedMs,reductionPercent:100*(1-optimizedMs/referenceMs)}});
}
console.log(JSON.stringify(reports,null,2));

import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {solveActiveCondensedCoupledQP as solve} from '../../src/physics/kirchhoffActiveCondensedSolver.js';
const names=['friction-boundary-cycle','normal-load-cycle','active-condensed-1999','coupled-full-200'];
const report=[];
for(const name of names){
    const p=JSON.parse(gunzipSync(fs.readFileSync(new URL(`../../tests/fixtures/kirchhoff-${name}.json.gz`,import.meta.url))),(_,v)=>v==='Infinity'?Infinity:v==='-Infinity'?-Infinity:v);
    const args=['matrix','rhs','lower','upper'].map(k=>Float64Array.from(p[k]));
    const variants=[false,true].map(batchEqualityResponses=>({name:batchEqualityResponses?'batch4':'scalar',
        options:{tolerance:.0002,numericalShift:1e-8,...p.options,initialFree:p.initialFree??p.options?.initialFree,
            simultaneousCoulomb:true,batchEqualityResponses,workspace:{},frictionWorkspace:{},loadWorkspace:{}},samples:[]}));
    for(let iteration=0;iteration<25;iteration++)for(const v of iteration%2?[...variants].reverse():variants){
        const start=performance.now();v.result=solve(...args,p.count,p.band,p.groups,v.options);
        if(!v.result.diagnostics.converged)throw new Error(`${name} ${v.name} failed original KKT`);
        if(iteration>=5)v.samples.push({totalMs:performance.now()-start,...v.result.diagnostics.condensedCosts});
    }
    if(!variants[0].result.increment.every((x,i)=>Object.is(x,variants[1].result.increment[i])))throw new Error(`${name}: changed solution`);
    report.push({name,count:p.count,band:p.band,variants:variants.map(v=>({name:v.name,
        means:Object.fromEntries(Object.keys(v.samples[0]).map(k=>[k,v.samples.reduce((s,x)=>s+x[k],0)/v.samples.length])),
        samples:v.samples,diagnostics:v.result.diagnostics}))});
}
if(process.argv[2])fs.writeFileSync(process.argv[2],JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report.map(r=>({name:r.name,variants:r.variants.map(v=>({name:v.name,means:v.means,columns:v.diagnostics.responseColumns,kernelCalls:v.diagnostics.responseKernelCalls}))})),null,2));

import fs from 'node:fs';
import os from 'node:os';
import {createHash} from 'node:crypto';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../../src/physics/kirchhoffCompositeElement.js';
import * as baseline from '../../src/physics/kirchhoffCompositeChain.js';
import {loadCompositeFastChainOverlay} from '../../tests/helpers/compositeFastChainOverlay.js';

const root=new URL('../../',import.meta.url),output=process.argv[2]??'/tmp/oet-composite-element-fast-benchmark.json';
const paths=['src/physics/kirchhoffCompositeElement.js','src/physics/kirchhoffCompositeElementFast.js',
    'src/physics/kirchhoffCompositeElementFastKernelBytes.js','src/physics/kirchhoffCompositeChain.js',
    'src/physics/kirchhoffLinearKernel.js','src/physics/kirchhoffLinearKernelBytes.js',
    'scripts/physics/build-composite-element-fast.mjs','scripts/physics/benchmark-composite-element-fast.mjs',
    'tests/helpers/compositeFastChainOverlay.js'];
const hashes=()=>Object.fromEntries(paths.map(p=>[p,createHash('sha256').update(fs.readFileSync(new URL(p,root))).digest('hex')]));
const overlay=await loadCompositeFastChainOverlay(),candidate=overlay.module;
const report={scope:'Paired common-chain assembly + band factor + primal solve using explicit javascript and wasm element backends. No physics dt, contacts, adaptive admission or browser FPS claim.',
    date:new Date().toISOString(),node:process.version,cpu:os.cpus()[0]?.model,
    sourceBefore:hashes(),overlaySha256:overlay.sourceSha256,warmups:6,pairs:8,measurements:[]};
const summary=samples=>{
    const sorted=samples.toSorted((a,b)=>a-b);
    return{meanMs:samples.reduce((a,b)=>a+b,0)/samples.length,medianMs:(sorted[3]+sorted[4])/2,
        minimumMs:sorted[0],maximumMs:sorted.at(-1)};
};
function parity(a,b,ra,rb) {
    let maximumDifference=0,maximumScaledDifference=0,byteIdentical=true;
    const check=(x,y)=>{
        if(!Number.isFinite(x)||!Number.isFinite(y))throw new Error('Nonfinite comparison output');
        maximumDifference=Math.max(maximumDifference,Math.abs(x-y));
        maximumScaledDifference=Math.max(maximumScaledDifference,Math.abs(x-y)/(1+Math.max(Math.abs(x),Math.abs(y))));
    };
    check(a.energy,b.energy);
    for(const [x,y] of [[a.gradient,b.gradient],[a.hessian,b.hessian],
        [ra.increment,rb.increment],[ra.residual,rb.residual],[ra.reactions,rb.reactions]]) {
        if(x.length!==y.length)throw new Error('Response topology differs');
        for(let i=0;i<x.length;i++)check(x[i],y[i]);
        byteIdentical&&=Buffer.from(x.buffer,x.byteOffset,x.byteLength).equals(Buffer.from(y.buffer,y.byteOffset,y.byteLength));
    }
    if(maximumScaledDifference>2e-12)throw new Error('Full assembly/response differs from the original element');
    return{maximumDifference,maximumScaledDifference,byteIdentical};
}
for(const nodes of [32,65,128,201]) {
    const coordinates=Array.from({length:nodes},(_,i)=>i*3),positions=coordinates.map(x=>[x,.2*Math.sin(x/40),.1*Math.cos(x/37)]);
    const layout=baseline.createCompositeChainLayout(Array.from({length:nodes-1},()=>['wire','catheter']));
    const a=baseline.createCompositeChainWorkspace(layout,{elementBackend:'javascript'}),b=candidate.createCompositeChainWorkspace(layout);
    if(a.elementBackend!=='javascript'||b.elementBackend!=='wasm')throw new Error('Backend comparison is not independent');
    const data={positions,coordinates,reference:captureCompositeReferenceFrames(positions),tools:[
        {id:'wire',angles:Float64Array.from(coordinates.slice(1),x=>.1*Math.sin(x/50)),material:compileCompositeMaterial({EI1:2,EI2:3,GJ:1.5,kappa0:[.001,0]})},
        {id:'catheter',angles:Float64Array.from(coordinates.slice(1),x=>.3+.02*x),material:compileCompositeMaterial({EI1:7,EI2:5,GJ:3,kappa0:[0,.003]})}
    ]};
    const diagonal=new Float64Array(layout.dofCount).fill(3),rows=[];
    const run=(module,workspace)=>{
        const start=performance.now();module.assembleCompositeChain(data,workspace);const assembled=performance.now();
        const result=module.solveCompositeChainIncrement(workspace,{diagonal,tolerance:1e-10});const done=performance.now();
        if(!result.converged)throw new Error('The unchanged primal equations are not satisfied');
        return{result,assemblyMs:assembled-start,solveMs:done-assembled,totalMs:done-start};
    };
    let lastA,lastB;
    for(let round=0;round<report.warmups+report.pairs;round++) {
        if(round%2){lastB=run(candidate,b);lastA=run(baseline,a);}
        else{lastA=run(baseline,a);lastB=run(candidate,b);}
        const compared=parity(a,b,lastA.result,lastB.result);
        if(round>=report.warmups)rows.push({order:round%2?'BA':'AB',baseline:{assemblyMs:lastA.assemblyMs,solveMs:lastA.solveMs,totalMs:lastA.totalMs},
            candidate:{assemblyMs:lastB.assemblyMs,solveMs:lastB.solveMs,totalMs:lastB.totalMs},parity:compared});
    }
    const stats=Object.fromEntries(['baseline','candidate'].map(id=>[id,Object.fromEntries(['assemblyMs','solveMs','totalMs']
        .map(key=>[key,summary(rows.map(row=>row[id][key]))]))]));
    report.measurements.push({nodes,dofCount:layout.dofCount,band:layout.band,matrixEntries:layout.dofCount*layout.band,
        factorizations:lastB.result.factorizations,maximumResidual:lastB.result.maximumResidual,
        medianWholeOperatorSpeedup:stats.baseline.totalMs.medianMs/stats.candidate.totalMs.medianMs,
        candidateFasterPairs:rows.filter(row=>row.candidate.totalMs<row.baseline.totalMs).length,stats,rows});
}
report.sourceAfter=hashes();report.sourceStable=JSON.stringify(report.sourceBefore)===JSON.stringify(report.sourceAfter);
fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({output,sourceStable:report.sourceStable,measurements:report.measurements.map(({rows,...rest})=>rest)},null,2));

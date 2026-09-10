import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {solveSeededCoulombNewton} from '../../src/physics/kirchhoffCoulombNewtonSolver.js';
import {measureCoupledLoadKKT} from '../../src/physics/kirchhoffCoupledLoadSolver.js';
const p=JSON.parse(gunzipSync(fs.readFileSync(new URL('../../tests/fixtures/kirchhoff-wall-witness-browser.json.gz',import.meta.url))),(_,v)=>v==='Infinity'?Infinity:v==='-Infinity'?-Infinity:v);
const [matrix,rhs,lower,upper]=['matrix','rhs','lower','upper'].map(k=>Float64Array.from(p[k]));
const variants={baseline:{},choleskyDeletes:{choleskyDeletes:true},bandLU:{coulombLinearSolver:'band-lu'},shortSeed:{maxSeedFrictionIterations:4,maxSeedActiveSetIterations:16},newtonFirst:{tryNewtonBeforeSeed:true}};
// Alternate order to reduce fixed-order/JIT bias; each variant owns its buffers.
const runs=Object.entries(variants).map(([name,variant])=>({name,
    options:{...p.options,...variant,workspace:{},frictionWorkspace:{},loadWorkspace:{}},samples:[],result:null}));
for(let repeat=0;repeat<28;repeat++)for(const run of repeat%2?[...runs].reverse():runs){
    const start=performance.now();
    run.result=solveSeededCoulombNewton(matrix,rhs,lower,upper,p.count,p.band,p.groups,run.options);
    if(repeat>=4)run.samples.push(performance.now()-start);
}
const report=[];
for(const {name,samples,result} of runs){
    const residual=Float64Array.from(rhs),x=result.increment;
    for(let i=0;i<p.count;i++)for(let j=Math.max(0,i-p.band+1);j<=i;j++){
        const a=matrix[i*p.band+i-j];residual[i]-=a*x[j];if(i!==j)residual[j]-=a*x[i];
    }
    const groups=p.groups.map(g=>({...g,radii:g.normalRow==null?g.radii:g.mu.map(mu=>mu*Math.max(0,g.normalLambda+x[g.normalRow]))}));
    const kkt=measureCoupledLoadKKT(residual,x,lower,upper,groups);
    const sorted=[...samples].sort((a,b)=>a-b);
    report.push({name,samplesMs:samples,meanMs:samples.reduce((a,b)=>a+b,0)/samples.length,medianMs:sorted[Math.floor(sorted.length/2)],p95Ms:sorted[Math.ceil(sorted.length*.95)-1],converged:result.diagnostics.converged,kkt:{maximumResidual:kkt.maximumResidual,coneViolation:kkt.coneViolation},diagnostics:result.diagnostics});
}
const output={count:p.count,band:p.band,groups:p.groups.length,originalTolerance:p.options.tolerance,report};
console.log(JSON.stringify(output,null,2));
if(process.argv[2])fs.writeFileSync(process.argv[2],JSON.stringify(output,null,2)+'\n');
if(report.some(r=>!r.converged||r.kkt.maximumResidual>p.options.tolerance||r.kkt.coneViolation>1e-9))process.exitCode=1;

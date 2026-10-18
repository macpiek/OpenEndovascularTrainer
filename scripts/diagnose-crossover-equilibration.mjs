// Research only: replay unchanged production steps in an isolated source copy,
// then compare diagonal scalings of captured KKT systems. No alternative
// solution is fed back into the physical continuation.
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {fileURLToPath,pathToFileURL} from 'node:url';
import {gunzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {createCoulombBandLU} from '../src/physics/kirchhoffCoulombBandLU.js';
import {captureBloodFlow} from '../src/physics/bloodFlowCoupling.js';
import {createCrossoverFlow} from '../tests/helpers/crossoverFlow.js';

const root=fileURLToPath(new URL('../',import.meta.url));
const output=process.argv[2]??path.join(root,'reports/performance-scenarios/001-equilibration-screen.json');
const fixtureBytes=fs.readFileSync(path.join(root,'reports/performance-scenarios/001-stage-05-replay.json.gz'));
const fixture=JSON.parse(gunzipSync(fixtureBytes)),request=fixture.stepRequest;
const temporary=fs.mkdtempSync(path.join(os.tmpdir(),'oet-equilibration-'));
for(const name of ['src','tests'])fs.cpSync(path.join(root,name),path.join(temporary,name),{recursive:true});
for(const name of ['res','node_modules'])fs.symlinkSync(path.join(root,name),path.join(temporary,name),'dir');
fs.copyFileSync(path.join(root,'package.json'),path.join(temporary,'package.json'));
const linearPath=path.join(temporary,'src/physics/kirchhoffSharedAxisLinear.js');
const original=fs.readFileSync(linearPath,'utf8');
const marker='    primal.forEach((p, i) => { w.increment[i] = w.solution[p]; });';
assert.equal(original.split(marker).length,2,'Unique diagnostic hook location');
fs.writeFileSync(linearPath,original.replace(marker,
    '    globalThis.__oetEquilibrationCapture?.(w,tolerance,residual,factorizations);\n'+marker));
const selected=new Set([0,29,40,60,99,152,173,203,239,250,288]);
const snapshots=[],seen=new Set(),perStep=new Map();let currentStep=-1,totalFailures=0;
const hash=bytes=>createHash('sha256').update(bytes).digest('hex');
globalThis.__oetEquilibrationCapture=(w,tolerance,residual,factorizations)=>{
    if(!selected.has(currentStep))return;
    const failed=residual>tolerance;if(failed)totalFailures++;
    const key=currentStep+'/'+failed,n=perStep.get(key)??0;
    if(n>=(failed?4:2))return;
    const identity=hash(Buffer.concat([Buffer.from(w.matrix.buffer,w.matrix.byteOffset,w.matrix.byteLength),
        Buffer.from(w.residual.buffer,w.residual.byteOffset,w.residual.byteLength)]));
    if(seen.has(identity))return;seen.add(identity);perStep.set(key,n+1);
    snapshots.push({step:currentStep,identity,count:w.count,
        band:{...w.band,starts:w.band.starts.slice(),ends:w.band.ends.slice(),offsets:w.band.offsets.slice()},
        matrix:w.matrix.slice(),rhs:w.residual.slice(),tolerance,residual,factorizations,
        solution:w.solution.slice()});
};

function residualOf(s,x,error){
    let maximum=0;const {starts,ends,offsets}=s.band;
    for(let i=0;i<s.count;i++){
        let high=s.rhs[i],low=0;
        for(let j=starts[i];j<=ends[i];j++){
            const a=s.matrix[offsets[i]+j],b=x[j];if(a===0||b===0)continue;
            const product=a*b,sa=134217729*a,sb=134217729*b;
            if(!Number.isFinite(sa)||!Number.isFinite(sb)||!Number.isFinite(product))return Infinity;
            const ah=sa-(sa-a),bh=sb-(sb-b),al=a-ah,bl=b-bh;
            const productError=((ah*bh-product)+ah*bl+al*bh)+al*bl;
            const sum=high+product,v=sum-high;
            low+=(high-(sum-v))+(product-v)+productError;high=sum;
        }
        const v=high+low;error[i]=v;maximum=Math.max(maximum,Math.abs(v));
    }
    return Number.isFinite(maximum)?maximum:Infinity;
}
function equilibrate(s,iterations,powerTwo,symmetric){
    const left=new Float64Array(s.count).fill(1),right=left.slice(),{starts,ends,offsets}=s.band;
    const rows=new Float64Array(s.count),columns=rows.slice();
    for(let pass=0;pass<iterations;pass++){
        rows.fill(0);columns.fill(0);
        for(let i=0;i<s.count;i++)for(let j=starts[i];j<=ends[i];j++){
            const value=Math.abs(s.matrix[offsets[i]+j]*left[i]*right[j]);
            rows[i]=Math.max(rows[i],value);columns[j]=Math.max(columns[j],value);
        }
        const scale=v=>powerTwo?2**Math.round(-.5*Math.log2(Math.max(v,1e-30))):1/Math.sqrt(Math.max(v,1e-30));
        for(let i=0;i<s.count;i++){
            left[i]*=scale(rows[i]);right[i]*=scale(symmetric?rows[i]:columns[i]);
        }
    }
    return {left,right};
}
function solve(s,variant){
    const {left,right}=equilibrate(s,variant.iterations,variant.powerTwo,variant.symmetric);
    const matrix=s.matrix.slice(),rhs=s.rhs.slice(),ones=new Float64Array(s.count).fill(1);
    for(let i=0;i<s.count;i++){
        rhs[i]*=left[i];
        for(let j=s.band.starts[i];j<=s.band.ends[i];j++)matrix[s.band.offsets[i]+j]*=left[i]*right[j];
    }
    // Reproduce the production multiplication order for the control variant.
    const baseline=variant.name==='control',lu=createCoulombBandLU(s.band,s.count,{wasmAssembly:true});
    const x=new Float64Array(s.count),error=x.slice(),correction=x.slice();let residual=Infinity,factorizations=0;
    for(let attempt=0;attempt<3;attempt++){
        if(attempt)for(let i=0;i<s.count;i++)rhs[i]=error[i]*left[i];
        factorizations++;
        const solved=baseline?lu.solve(s.matrix,attempt?error:s.rhs,left,0,correction):lu.solve(matrix,rhs,ones,0,correction);
        if(!solved)break;
        for(let i=0;i<s.count;i++)x[i]+=correction[i]*right[i];
        residual=residualOf(s,x,error);if(residual<=s.tolerance)break;
    }
    return {name:variant.name,residual:Number.isFinite(residual)?residual:null,converged:residual<=s.tolerance,
        factorizations,maximumSolutionDifference:Math.max(...x.map((v,i)=>Math.abs(v-s.solution[i])))};
}
const report={recordedAt:new Date().toISOString(),researchOnly:true,productionDefaultsChanged:false,
    fixtureSha256:hash(fixtureBytes),scriptSha256:hash(fs.readFileSync(fileURLToPath(import.meta.url))),
    linearSourceSha256:hash(original),selectedSteps:[...selected],cases:[],complete:false,
    limitations:['Isolated matrix diagnostic, not a timestep speed or trajectory benchmark.',
        'Sample includes at most four distinct residual failures and two passing systems per selected step.',
        'Original force, length, penetration and timestep tolerances remain unchanged.',
        'No alternative direction is used by the production replay; successful matrix solves do not establish nonlinear convergence.']};
let anatomy;
try{
    const url=relative=>pathToFileURL(path.join(temporary,relative));
    anatomy=await (await import(url('tests/helpers/coupledRuntimeFixture.js'))).loadCoupledRuntimeAnatomy(undefined,fixture.anatomy);
    const {restoreSharedAxisReplay}=await import(url('tests/helpers/sharedAxisReplay.js'));
    const {advanceSharedAxis}=await import(url('src/physics/kirchhoffSharedAxisAppSystem.js'));
    const options={...request.options,contactCandidateMargin:.01,lazyBasisCoefficients:true,deferActiveBasis:true,wasmBasis:true,reuseFrictionAssembly:true};
    let state=restoreSharedAxisReplay({...fixture,...options},anatomy.field),rotations=request.rotations;
    const flow=createCrossoverFlow();flow.time=request.options.bloodFlow.time;
    for(currentStep=0;currentStep<=Math.max(...selected);currentStep++){
        const offset=currentStep<30?0:currentStep<50?-(currentStep-29)*.5:currentStep<70?-(69-currentStep)*.5:0;
        const tools=request.tools.map(t=>({...t,insertion:t.insertion+offset}));
        const bloodFlow=captureBloodFlow(state,flow,request.options.bloodFlow.sheathLength,Math.max(...tools.map(t=>t.insertion)));
        const iterator=advanceSharedAxis(state,rotations,request.dt,tools,{...options,bloodFlow});
        let next;do{next=iterator.next();}while(!next.done);
        const r=next.value.result;
        assert.ok(next.value.state&&r.converged&&r.quality.finite&&r.certificateBound<=1e-6&&r.residual.length<=1e-5&&r.quality.maxPenetration<=1e-5);
        state=next.value.state;rotations=next.value.rotations;flow.update(request.dt);
    }
    delete globalThis.__oetEquilibrationCapture;
    report.observedFailures=totalFailures;
    const variants=[{name:'control',iterations:1,powerTwo:false,symmetric:true}];
    for(const iterations of [1,3,6,10])for(const powerTwo of [false,true])for(const symmetric of [false,true]){
        if(iterations===1&&!powerTwo&&symmetric)continue;
        variants.push({name:`${symmetric?'symmetric':'two-sided'}-${powerTwo?'binary':'real'}-${iterations}`,iterations,powerTwo,symmetric});
    }
    for(const s of snapshots){
        const entry={step:s.step,matrixSha256:s.identity,count:s.count,tolerance:s.tolerance,
            original:{residual:s.residual,converged:s.residual<=s.tolerance,factorizations:s.factorizations},
            variants:variants.map(v=>solve(s,v))};
        report.cases.push(entry);
        console.log(JSON.stringify({step:s.step,original:entry.original,passing:entry.variants.filter(v=>v.converged).map(v=>v.name)}));
    }
    report.summary=Object.fromEntries(variants.map(v=>[v.name,{passed:report.cases.filter(c=>c.variants.find(x=>x.name===v.name).converged).length,
        recovered:report.cases.filter(c=>!c.original.converged&&c.variants.find(x=>x.name===v.name).converged).length,
        regressed:report.cases.filter(c=>c.original.converged&&!c.variants.find(x=>x.name===v.name).converged).length}]));
    report.complete=true;
}catch(error){report.error=error.stack;process.exitCode=1;console.error(error.stack);}
finally{delete globalThis.__oetEquilibrationCapture;anatomy?.dispose();fs.rmSync(temporary,{recursive:true,force:true});fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');}

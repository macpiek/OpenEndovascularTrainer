// Research only: quantify the cost and trajectory effect of a looser force
// residual. This does not modify the runtime or the strict throughput test.
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from '../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis,sampleSharedAxisPosition} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {captureBloodFlow} from '../src/physics/bloodFlowCoupling.js';
import {createCrossoverFlow} from '../tests/helpers/crossoverFlow.js';

const root=new URL('../',import.meta.url);
const output=process.argv[2]??new URL('reports/performance-scenarios/001-force-tolerance-screen.json',root);
const stepCount=Number(process.argv[3]??100);
assert.ok(Number.isInteger(stepCount)&&stepCount>=100&&stepCount<=1200);
const source=fs.readFileSync(new URL('reports/performance-scenarios/001-stage-05-replay.json.gz',root));
const fixture=JSON.parse(gunzipSync(source)),request=fixture.stepRequest,dt=request.dt;
const options={contactCandidateMargin:.01,lazyBasisCoefficients:true,deferActiveBasis:true,wasmBasis:true,reuseFrictionAssembly:true};
const tolerances=JSON.parse(process.argv[4]??'[1e-6,1e-5,1e-4]');
assert.ok(Array.isArray(tolerances)&&tolerances[0]===1e-6&&tolerances.length>=2&&tolerances.length<=4&&
    tolerances.every(t=>Number.isFinite(t)&&t>=1e-6&&t<=.01),'Strict reference followed by explicit research force tolerances');
const contexts=[];
const report={recordedAt:new Date().toISOString(),fixtureSha256:createHash('sha256').update(source).digest('hex'),
    researchOnly:true,productionDefaultsChanged:false,dt,stepCount,tolerances,options,runs:[],completed:false,
    sourceHashes:Object.fromEntries(['kirchhoffSharedAxisLinear.js','kirchhoffSharedAxisNative.js','kirchhoffSharedAxisTimeStep.js','kirchhoffSharedAxisWallFriction.js'].map(file=>
        [file,createHash('sha256').update(fs.readFileSync(new URL('src/physics/'+file,root))).digest('hex')])),
    limitations:['Looser force criteria are an explicit model-accuracy experiment, not a pass of the original 1e-6 certificate.',
        'Force units are model units; measured geometric differences are not clinical validation.',
        'Continuation of the recorded long-insertion state; not the entire manual crossover.',
        'CPU timing excludes browser rendering and scheduling.']};
function stats(steps){
    const values=steps.map(s=>s.cpuMs).sort((a,b)=>a-b),totalMs=values.reduce((a,b)=>a+b,0);
    const maxima=['shapeDifferenceMm','tipDifferenceMm','maxPenetrationMm','maxLengthErrorMm','certificateBound','maxSpeed'];
    return {steps:steps.length,totalMs,cpuStepsPerSecond:steps.length*1000/totalMs,
        medianMs:values[Math.floor(values.length/2)],p95Ms:values[Math.ceil(values.length*.95)-1],maxMs:values.at(-1),
        ...Object.fromEntries(maxima.map(key=>[key,Math.max(...steps.map(s=>s[key]))])),
        originalForceCertificatePassed:steps.filter(s=>s.certificateBound<=1e-6).length,
        ...Object.fromEntries(['iterations','factorizations'].map(key=>[key,steps.reduce((sum,s)=>sum+s[key],0)]))};
}
try {
    // Separate contact-field caches for every variant. All clocks and controls
    // start from the same recorded state on each repeat.
    for(const tolerance of tolerances)contexts.push({tolerance,anatomy:await loadCoupledRuntimeAnatomy(undefined,fixture.anatomy)});
    for(let repeat=0;repeat<2;repeat++) {
        for(const c of contexts){
            c.state=restoreSharedAxisReplay({...fixture,...options},c.anatomy.field);
            c.rotations=request.rotations;c.flow=createCrossoverFlow();c.flow.time=request.options.bloodFlow.time;
            c.run={forceTolerance:c.tolerance,repeat,steps:[]};report.runs.push(c.run);
        }
        for(let step=0;step<stepCount;step++){
            const phase=step<30?'hold':step<50?'withdraw':step<70?'advance':'settle';
            const offset=step<30?0:step<50?-(step-29)*.5:step<70?-(69-step)*.5:0;
            const targets=request.tools.map(t=>({...t,insertion:t.insertion+offset}));
            for(const c of (step+repeat)%2?[...contexts].reverse():contexts){
                const start=performance.now(),cpuStart=process.cpuUsage();
                const bloodFlow=captureBloodFlow(c.state,c.flow,request.options.bloodFlow.sheathLength,Math.max(...targets.map(t=>t.insertion)));
                const it=advanceSharedAxis(c.state,c.rotations,dt,targets,{...request.options,...options,forceTolerance:c.tolerance,bloodFlow});
                let next;do{next=it.next();}while(!next.done);
                const {result:r,state,rotations}=next.value;
                assert.ok(state&&r.converged&&r.quality.finite,`${c.tolerance} at ${step}: ${r.status}`);
                // Only the force tolerance differs. Inextensibility, exact
                // vessel contacts and acceptance of the entire dt are retained.
                assert.ok(r.certificateBound<=c.tolerance&&r.residual.length<=1e-5&&r.quality.maxPenetration<=1e-5&&
                    r.quality.bodies.every(b=>b.maxLengthError<1e-5),'Per-variant force and unchanged geometric certificates');
                c.state=state;c.rotations=rotations;c.flow.update(dt);
                const cpuMs=performance.now()-start,used=process.cpuUsage(cpuStart);
                c.last={step,phase,cpuMs,processCpuMs:(used.user+used.system)/1000,iterations:r.iterations,factorizations:r.factorizations,
                    subdivisions:r.subdivisions,certificateBound:r.certificateBound,maxPenetrationMm:r.quality.maxPenetration,
                    maxLengthErrorMm:Math.max(...r.quality.bodies.map(b=>b.maxLengthError)),maxSpeed:Math.max(...r.quality.bodies.map(b=>b.maxSpeed)),
                    nodes:state.positions.length};
            }
            const reference=contexts[0].state;
            for(const c of contexts){
                assert.deepEqual(c.state.coordinates,reference.coordinates,'Identical spatial discretization');
                const differences=c.state.positions.map((p,i)=>Math.hypot(...p.map((v,k)=>(v+c.state.origin[k])-(reference.positions[i][k]+reference.origin[k]))));
                c.last.shapeDifferenceMm=Math.max(...differences);
                c.last.tipDifferenceMm=Math.max(...c.state.materials.map(({spec})=>{
                    const p=sampleSharedAxisPosition(c.state,spec.insertion),q=sampleSharedAxisPosition(reference,spec.insertion);
                    return Math.hypot(...p.map((v,k)=>v-q[k]));
                }));
                c.run.steps.push(c.last);
            }
        }
        for(const c of contexts){c.run.summary=stats(c.run.steps);console.log(JSON.stringify({forceTolerance:c.tolerance,repeat,...c.run.summary}));}
    }
    report.summary=tolerances.map(forceTolerance=>({forceTolerance,...stats(report.runs.filter(r=>r.forceTolerance===forceTolerance).flatMap(r=>r.steps))}));
    report.completed=true;
} catch(error){report.error=error.stack;process.exitCode=1;console.error(error.message);}
finally {for(const c of contexts)c.anatomy.dispose();fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');}

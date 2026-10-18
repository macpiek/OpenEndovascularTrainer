// Full accepted-step CPU budget for the captured long-tool crossover.
// Optional baseline checkout: npm run benchmark:crossover:throughput -- /path/to/baseline
// Both versions use identical controls, flow and tolerances; rendering is excluded.
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import {pathToFileURL} from 'node:url';
import assert from 'node:assert/strict';
import {captureBloodFlow} from '../src/physics/bloodFlowCoupling.js';
import {createCrossoverFlow} from '../tests/helpers/crossoverFlow.js';

const root=new URL('../',import.meta.url),baseline=process.argv[2],output=process.argv[3]??new URL('reports/performance-scenarios/001-throughput-result.json',root);
const afterOptions=JSON.parse(process.argv[4]??'{}'),trajectoryToleranceMm=Number(process.argv[5]??1e-7);
const beforeOptions=JSON.parse(process.argv[6]??'{}');
const stepCount=Number(process.argv[7]??100);
assert.ok(Number.isInteger(stepCount)&&stepCount>=100&&stepCount<=1200,'100–1200 continuation steps');
assert.ok(Number.isFinite(trajectoryToleranceMm)&&trajectoryToleranceMm>0&&trajectoryToleranceMm<=.001);
const source=fs.readFileSync(new URL('reports/performance-scenarios/001-stage-05-replay.json.gz',root));
const fixture=JSON.parse(gunzipSync(source)),request=fixture.stepRequest,dt=request.dt;
const versions=[...(baseline?[{name:'before',root:pathToFileURL(resolve(baseline)+'/')}]:[]),{name:'after',root}];
const sourceFiles=['kirchhoffBandRefinement.js','kirchhoffLinearKernel.js','kirchhoffLinearKernelBytes.js','kirchhoffBandSimdKernelBytes.js','kirchhoffSharedAxisFrictionKernel.js','kirchhoffSharedAxisFrictionKernelBytes.js','kirchhoffSharedAxisWallFriction.js','collision/scalarMeshBvhClosestPoint.js','collision/vesselContactField.js','kirchhoffCoulombBandLU.js','kirchhoffSharedAxisProjection.js','kirchhoffSharedAxisDynamics.js','kirchhoffSharedAxisMaterialKernel.js','kirchhoffSharedAxisMaterialKernelBytes.js','kirchhoffSharedAxisInertiaKernelBytes.js','kirchhoffSharedAxisMaterialTangent.js','kirchhoffSharedAxisActiveBasis.js','kirchhoffSharedAxisBasisKernel.js','kirchhoffSharedAxisBasisKernelBytes.js','kirchhoffSharedAxisRejectedLinearization.js','kirchhoffSharedAxisConstraintRows.js','kirchhoffSharedAxisMatrixAssembly.js','kirchhoffSharedAxisDiscoveryCache.js','kirchhoffSharedAxisVesselWitnesses.js','kirchhoffSharedAxisNative.js','kirchhoffSharedAxisModifiedNewton.js','kirchhoffIncrementalContactLU.js','kirchhoffSharedAxisAppSystem.js','kirchhoffSharedAxisLinear.js','kirchhoffSharedAxisTimeStep.js','kirchhoffSharedAxisInactiveContacts.js','kirchhoffSharedAxisBendLimit.js','kirchhoffSharedAxisContacts.js','kirchhoffSharedAxisReplay.js','../simulator.js'];
for(const v of versions) {
    v.anatomy=await (await import(new URL('tests/helpers/coupledRuntimeFixture.js',v.root))).loadCoupledRuntimeAnatomy(undefined,fixture.anatomy);
    v.restore=(await import(new URL('tests/helpers/sharedAxisReplay.js',v.root))).restoreSharedAxisReplay;
    v.advance=(await import(new URL('src/physics/kirchhoffSharedAxisAppSystem.js',v.root))).advanceSharedAxis;
    v.hashes=Object.fromEntries(sourceFiles.map(f=>[f,fs.existsSync(new URL('src/physics/'+f,v.root))?createHash('sha256').update(fs.readFileSync(new URL('src/physics/'+f,v.root))).digest('hex'):null]));
}
const report={recordedAt:new Date().toISOString(),node:process.version,fixtureSha256:createHash('sha256').update(source).digest('hex'),
    dt,stepCount,afterOptions,beforeOptions,trajectoryToleranceMm,rendering:false,acceptedStepTargetMs:1000/60,sourceHashes:Object.fromEntries(versions.map(v=>[v.name,v.hashes])),runs:[],passed:false,
    referenceVersion:versions[0].name,pairing:'alternating accepted steps, reversed order each step and repeat',contactFields:'separate field and BVH scratch loaded from each checkout',limitations:['CPU throughput is not browser FPS; rendering and the frame scheduler add work.',
        'Exact captured stage-05 state; continuation controls exercise hold, withdrawal, advancement and settling, not the entire manual crossover.']};
// Each checkout owns its contact field, BVH scratch and discovery history.
// Sharing the current field would also give the baseline the new collision code.
// Optional second-repetition CPU sampling; anatomy, flow setup and replay
// restoration are excluded. The first repetition warms the same code paths.
// Profiled runs are diagnostic and must not serve as timing comparisons.
let profiler,profileStarted=false;
const profilePath=process.env.OET_CROSSOVER_CPU_PROFILE;
if(profilePath) {
    const {Session}=await import('node:inspector/promises');
    profiler=new Session();profiler.connect();
    await profiler.post('Profiler.enable');
}
const references=[];
function statistics(samples) {
    const values=samples.map(s=>s.cpuMs).sort((a,b)=>a-b),total=values.reduce((s,v)=>s+v,0);
    return {steps:values.length,totalMs:total,medianMs:values[Math.floor(values.length/2)],p95Ms:values[Math.ceil(values.length*.95)-1],
        maxMs:values.at(-1),cpuStepsPerSecond:values.length*1000/total,withinBudget:samples.filter(s=>s.cpuMs<=report.acceptedStepTargetMs).length};
}
try {
    // Alternate complete steps to pair the same workload under nearly the
    // same machine load. Each version owns its physical state and flow clock.
    for(let repeat=0;repeat<2;repeat++) {
        const contexts=versions.map(version=>{
            const flow=createCrossoverFlow();flow.time=request.options.bloodFlow.time;
            const run={version:version.name,repeat,steps:[],maxWorldDifferenceMm:0};report.runs.push(run);
            const restoreOptions={...(version.name==='after'?afterOptions:beforeOptions)};
            // Row pruning takes effect on the next feed. The incoming fixture
            // must first restore its own serialized constraint topology.
            delete restoreOptions.omitInactiveSampleRows;
            return {version,flow,run,state:version.restore({...fixture,...restoreOptions},version.anatomy.field),rotations:request.rotations};
        });
        if(profiler&&repeat===1){await profiler.post('Profiler.start');profileStarted=true;}
        for(let step=0;step<stepCount;step++)for(const context of (step+repeat)%2?[...contexts].reverse():contexts) {
            const {version,flow,run}=context;let {state,rotations}=context;
            const phase=step<30?'hold':step<50?'withdraw':step<70?'advance':'settle';
            const offset=step<30?0:step<50?-(step-29)*.5:step<70?-(69-step)*.5:0;
            const targets=request.tools.map(t=>({...t,insertion:t.insertion+offset}));
            const cpuStart=process.cpuUsage(),start=performance.now();
            const bloodFlow=captureBloodFlow(state,flow,request.options.bloodFlow.sheathLength,Math.max(...targets.map(t=>t.insertion)));
            const iterator=version.advance(state,rotations,dt,targets,{...request.options,...(version.name==='after'?afterOptions:beforeOptions),bloodFlow});
            let next;do{next=iterator.next();}while(!next.done);
            const r=next.value.result;
            assert.ok(next.value.state&&r.converged&&r.quality.finite,`${version.name}, ${step}: ${r.status}`);
            assert.ok(r.certificateBound<=1e-6&&r.residual.length<=1e-5&&r.quality.maxPenetration<=1e-5&&r.quality.bodies.every(b=>b.maxLengthError<1e-5),'Physical certificate');
            state=next.value.state;rotations=next.value.rotations;flow.update(dt);
            const cpuMs=performance.now()-start,cpuUsage=process.cpuUsage(cpuStart);
            context.state=state;context.rotations=rotations;
            const world=state.positions.map(p=>p.map((v,k)=>v+state.origin[k]));
            context.world={step,coordinates:state.coordinates,world};
            if(version.name===report.referenceVersion&&!references[step])references[step]=context.world;
            if(references[step])for(const compared of contexts)if(compared.world?.step===step) {
                assert.deepEqual(compared.world.coordinates,references[step].coordinates,'Identical spatial resolution');
                for(let i=0;i<world.length;i++)compared.run.maxWorldDifferenceMm=Math.max(compared.run.maxWorldDifferenceMm,
                    Math.hypot(...compared.world.world[i].map((v,k)=>v-references[step].world[i][k])));
                assert.ok(compared.run.maxWorldDifferenceMm<=trajectoryToleranceMm,'Optimization exceeded the trajectory comparison budget');
            }
            run.steps.push({step,phase,cpuMs,processCpuMs:(cpuUsage.user+cpuUsage.system)/1000,iterations:r.iterations,factorizations:r.factorizations,modifiedAttempts:r.modifiedAttempts,modifiedAccepted:r.modifiedAccepted,modifiedFallbacks:r.modifiedFallbacks,fullAssemblies:r.fullAssemblies,residualAssemblies:r.residualAssemblies,promotedAssemblies:r.promotedAssemblies,coupledFrictionRefreshes:r.coupledFrictionRefreshes,fallbackLinearizationReuses:r.fallbackLinearizationReuses??0,certificateBound:r.certificateBound,
                linearScratch:r.linearScratch,timings:r.timings,attempts:r.attempts,subdivisions:r.subdivisions,backtracks:r.backtracks,geometryRestarts:r.geometryRestarts,maxPenetrationMm:r.quality.maxPenetration,nodes:state.positions.length,rows:state.definitions.length});
        }
        for(const {version,run} of contexts) {
            run.summary=statistics(run.steps);
            const scratch=version.anatomy.field._scalarBvhScratch;
            run.bvhNeighborhood=scratch?{cumulative:true,winnerHits:scratch.neighborhoodWinnerHits??0,hits:scratch.neighborhoodHits??0,misses:scratch.neighborhoodMisses??0,entries:scratch.neighborhoods?.size??0,numericBytes:scratch.neighborhoodBytes??0}:null;
            run.phases=Object.fromEntries(['hold','withdraw','advance','settle'].map(p=>[p,statistics(run.steps.filter(s=>s.phase===p))]));
            console.log(JSON.stringify({version:version.name,repeat,...run.summary,maxWorldDifferenceMm:run.maxWorldDifferenceMm}));
        }
    }
    report.summary=Object.fromEntries(versions.map(v=>[v.name,statistics(report.runs.filter(r=>r.version===v.name).flatMap(r=>r.steps))]));
    report.cpuBudgetMet=report.summary.after.withinBudget===report.summary.after.steps;
    report.passed=true;
} catch(error) {report.error=error.stack;process.exitCode=1;console.error(error.message);}
finally {
    if(profiler) {
        if(profileStarted) {
            const {profile}=await profiler.post('Profiler.stop');
            fs.writeFileSync(profilePath,JSON.stringify(profile));report.profiled=true;
            report.profileScope='Second repetition only; excludes anatomy loading, replay restoration and flow construction';
        }
        profiler.disconnect();
    }
    for(const v of versions)v.anatomy.dispose();fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');
}

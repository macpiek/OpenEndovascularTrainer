// Research gate for a coarser mechanical model. This intentionally allows a
// different spatial discretization; the exact optimization benchmark does not.
// Usage: node scripts/benchmark-crossover-reduced-model.mjs output.json '{mesh overrides}' [steps] [shape budget mm] '{contact options}'
import fs from 'node:fs';
import {gunzipSync,gzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from '../tests/helpers/sharedAxisReplay.js';
import {advanceSharedAxis,sampleSharedAxisPosition} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {adaptiveMeshOptions} from '../src/physics/kirchhoffSharedAxisAdaptiveMesh.js';
import {CROSSOVER_REDUCED_MODEL} from '../tests/helpers/crossoverReducedModel.js';
import {captureBloodFlow} from '../src/physics/bloodFlowCoupling.js';
import {createCrossoverFlow} from '../tests/helpers/crossoverFlow.js';

const output=process.argv[2]??'reports/performance-scenarios/001-reduced-model.json';
const meshOverrides=JSON.parse(process.argv[3]??'{}');
// null keeps the full grid when isolating a contact-strategy experiment.
const mesh=meshOverrides===null?null:adaptiveMeshOptions({...CROSSOVER_REDUCED_MODEL,...meshOverrides});
const steps=Number(process.argv[4]??180);
assert.ok(Number.isInteger(steps)&&steps>=100&&steps<=1200);
const shapeBudgetMm=Number(process.argv[5]??.1);
assert.ok(Number.isFinite(shapeBudgetMm)&&shapeBudgetMm>0&&shapeBudgetMm<=.1,'User-authorized shape screening budget must be in (0, 0.1] mm');
const source=fs.readFileSync(new URL('../reports/performance-scenarios/001-stage-05-replay.json.gz',import.meta.url));
const fixture=JSON.parse(gunzipSync(source)),request=fixture.stepRequest;
const options={...request.options,contactCandidateMargin:.01,lazyBasisCoefficients:true,deferActiveBasis:true,wasmBasis:true,reuseFrictionAssembly:true,
    forceTolerance:1e-6,lengthTolerance:1e-5};
const contactOptions=JSON.parse(process.argv[6]??'{}');
assert.ok(Object.keys(contactOptions).every(key=>
    ['pruneInactiveWitnesses','recoverNegativeContact','omitInactiveSampleRows'].includes(key)&&typeof contactOptions[key]==='boolean'||
    key==='newtonActiveSetLimit'&&Number.isInteger(contactOptions[key])&&contactOptions[key]>=4&&contactOptions[key]<=16||
    key==='contactCandidateMargin'&&Number.isFinite(contactOptions[key])&&contactOptions[key]>=0&&contactOptions[key]<=.5),
    'Only explicit contact strategy experiments are supported; solver tolerances remain fixed');
const report={recordedAt:new Date().toISOString(),node:process.version,fixtureSha256:createHash('sha256').update(source).digest('hex'),
    mesh,contactOptions,dt:request.dt,steps,wasmBasis:options.wasmBasis,reuseFrictionAssembly:options.reuseFrictionAssembly,forceTolerance:options.forceTolerance,lengthTolerance:options.lengthTolerance,
    sampleSpacingMm:2.5,exploratoryShapeBudgetMm:shapeBudgetMm,passed:false,goalAchieved:false,rows:[],
    shapeComparison:'All breakpoints of both polygonal axes and both tips at the same spatial coordinate, plus samples every 2.5 mm; maximum norm on each affine interval occurs at an endpoint.',
    contactFields:'Independent anatomy fields, BVH scratch and discovery history for each model',
    limitations:['Experimental approximation, not an exact algebraic optimization or a clinical accuracy guarantee.',
        `The ${shapeBudgetMm} mm trajectory budget is an exploratory screening threshold; passing is not validation for all maneuvers.`,
        'CPU only; rendering, browser scheduling and the full manual crossover are not measured.',
        'Identical hold/withdraw/advance/settle controls and physical tolerances. No relaxed adaptive solver tolerances.'],
    sourceHashes:Object.fromEntries(['kirchhoffBandRefinement.js','kirchhoffCoulombBandLU.js','kirchhoffLinearKernel.js','kirchhoffLinearKernelBytes.js','kirchhoffBandSimdKernelBytes.js',
        'kirchhoffSharedAxisMaterialKernel.js','kirchhoffSharedAxisMaterialKernelBytes.js','kirchhoffSharedAxisMaterialTangent.js',
        'kirchhoffSharedAxisConstraintRows.js','kirchhoffSharedAxisMatrixAssembly.js','kirchhoffSharedAxisWallFriction.js','kirchhoffSharedAxisWitnessPruning.js',
        'kirchhoffSharedAxisAdaptiveMesh.js','kirchhoffSharedAxisNative.js','kirchhoffSharedAxisLinear.js',
        'kirchhoffSharedAxisAppSystem.js','kirchhoffSharedAxisRestore.js','kirchhoffSharedAxisCrossoverPreset.js',
        'kirchhoffSharedAxisGeometryIdentity.js','kirchhoffSharedAxisReplay.js','kirchhoffSharedAxisVesselWitnesses.js','kirchhoffSharedAxisTimeStep.js',
        'kirchhoffSharedAxisActiveBasis.js','kirchhoffSharedAxisBasisKernel.js','kirchhoffSharedAxisBasisKernelBytes.js'].map(name=>
        [name,createHash('sha256').update(fs.readFileSync(new URL('../src/physics/'+name,import.meta.url))).digest('hex')]))};
const contexts=[];
for(const name of ['reference','reduced']) {
    const anatomy=await loadCoupledRuntimeAnatomy(undefined,fixture.anatomy);
    const state=restoreSharedAxisReplay(fixture,anatomy.field),flow=createCrossoverFlow();
    if(name==='reduced')state.adaptiveMesh=mesh;
    flow.time=request.options.bloodFlow.time;
    contexts.push({name,anatomy,state,rotations:request.rotations,flow,rows:[],options:name==='reduced'?{...options,...contactOptions}:options});
}
const distance=(a,b)=>Math.hypot(...a.map((v,k)=>v-b[k]));
function summary(rows) {
    if(!rows.length)return {steps:0};
    const costs=rows.map(r=>r.cpuMs).sort((a,b)=>a-b),total=costs.reduce((a,b)=>a+b,0);
    return {steps:costs.length,totalMs:total,medianMs:costs[Math.floor(costs.length*.5)],p95Ms:costs[Math.ceil(costs.length*.95)-1],
        maxMs:costs.at(-1),cpuStepsPerSecond:costs.length*1000/total,withinBudget:costs.filter(v=>v<=1000/60).length,
        minNodes:Math.min(...rows.map(r=>r.nodes)),maxNodes:Math.max(...rows.map(r=>r.nodes)),
        maxPenetrationMm:Math.max(...rows.map(r=>r.maxPenetrationMm)),maxLengthError:Math.max(...rows.map(r=>r.maxLengthError)),
        maxCertificateBound:Math.max(...rows.map(r=>r.certificateBound)),maxSpeed:Math.max(...rows.map(r=>r.maxSpeed))};
}
try {
    for(let step=0;step<steps;step++) {
        // Match the established 100-step benchmark, then hold for a longer
        // stability check (including the former adaptive failure at step 122).
        const offset=step<30?0:step<50?-(step-29)*.5:step<70?-(69-step)*.5:0;
        const targets=request.tools.map(t=>({...t,insertion:t.insertion+offset}));
        const row={step,phase:step<30?'hold':step<50?'withdraw':step<70?'advance':'settle'};
        report.rows.push(row);
        for(const context of step%2?[...contexts].reverse():contexts) {
            const {state,rotations,flow}=context,start=performance.now();
            const bloodFlow=captureBloodFlow(state,flow,request.options.bloodFlow.sheathLength,Math.max(...targets.map(t=>t.insertion)));
            const iterator=advanceSharedAxis(state,rotations,request.dt,targets,{...context.options,bloodFlow});
            let next;do{next=iterator.next();}while(!next.done);
            const {result:r,state:accepted}=next.value;
            // Include rejected attempts, subdivision, recentering and flow;
            // exclude shape sampling and JSON/report work from the CPU budget.
            if(accepted)flow.update(request.dt);
            const cpuMs=performance.now()-start;
            if(!accepted||!r.converged) {
                row[context.name]={cpuMs,failure:r};
                const checkpoint=output.replace(/\.json$/,'')+`.failed-${context.name}.json.gz`;
                fs.writeFileSync(checkpoint,gzipSync(JSON.stringify({...captureSharedAxisReplay(state,fixture.sheath),
                    anatomy:fixture.anatomy,stepRequest:{dt:request.dt,rotations,tools:targets,options:{...context.options,bloodFlow}}})));
                report.failureCheckpoint=checkpoint;
                throw new Error(`${context.name} failed at step ${step}: ${r.status}`);
            }
            assert.ok(r.quality.finite&&r.certificateBound<=options.forceTolerance&&r.residual.length<=options.lengthTolerance&&
                r.quality.maxPenetration<=options.lengthTolerance&&r.quality.bodies.every(b=>b.maxLengthError<options.lengthTolerance),
                `${context.name} physical certificate, step ${step}`);
            const result={cpuMs,nodes:accepted.positions.length,rows:accepted.definitions.length,iterations:r.iterations,
                factorizations:r.factorizations,subdivisions:r.subdivisions,attempts:r.attempts,certificateBound:r.certificateBound,
                maxPenetrationMm:r.quality.maxPenetration,maxLengthError:Math.max(...r.quality.bodies.map(b=>b.maxLengthError)),
                maxSpeed:Math.max(...r.quality.bodies.map(b=>b.maxSpeed)),timings:r.timings};
            row[context.name]=result;context.rows.push(result);
            context.state=accepted;context.rotations=next.value.rotations;
        }
        const [reference,reduced]=contexts.map(c=>c.state);
        const coordinates=new Set([...reference.coordinates,...reduced.coordinates,...targets.map(t=>t.insertion)]);
        for(let x=reference.coordinates[0];x<reference.coordinates.at(-1);x+=report.sampleSpacingMm)coordinates.add(x);
        let worstCoordinate=0,shapeError=0;
        for(const x of coordinates) {
            const error=distance(sampleSharedAxisPosition(reference,x),sampleSharedAxisPosition(reduced,x));
            if(error>shapeError){shapeError=error;worstCoordinate=x;}
        }
        row.maxShapeDifferenceMm=shapeError;row.worstCoordinateMm=worstCoordinate;
        // Retain the local topology around a discrepancy, so remeshing errors
        // can be distinguished from gradual changes in the shaft trajectory.
        row.worstNeighborhood=Object.fromEntries(contexts.map(c=>[c.name,
            c.state.coordinates.filter(x=>Math.abs(x-worstCoordinate)<=15).map(x=>({x,p:sampleSharedAxisPosition(c.state,x)}))]));
        row.tipDifferencesMm=Object.fromEntries(targets.map(t=>[t.id,
            distance(sampleSharedAxisPosition(reference,t.insertion),sampleSharedAxisPosition(reduced,t.insertion))]));
        if(step%30===29)console.log(JSON.stringify({step,nodes:reduced.positions.length,maxShapeDifferenceMm:shapeError,
            reference:summary(contexts[0].rows).medianMs,reduced:summary(contexts[1].rows).medianMs}));
    }
    report.physicalCertificatesPassed=true;
    report.maxShapeDifferenceMm=Math.max(...report.rows.map(r=>r.maxShapeDifferenceMm));
    report.maxTipDifferenceMm=Math.max(...report.rows.flatMap(r=>Object.values(r.tipDifferencesMm)));
    report.passed=report.maxShapeDifferenceMm<=report.exploratoryShapeBudgetMm;
    if(!report.passed){report.rejection='Shape deviation exceeds the exploratory approximation budget';process.exitCode=1;}
} catch(error) {report.error=error.stack;process.exitCode=1;console.error(error.message);}
finally {
    report.summary=Object.fromEntries(contexts.map(c=>[c.name,summary(c.rows)]));
    const pairs=report.rows.filter(r=>Number.isFinite(r.maxShapeDifferenceMm));
    report.completedPairedSteps=pairs.length;
    report.pairedSummary=Object.fromEntries(contexts.map(c=>[c.name,summary(pairs.map(r=>r[c.name]))]));
    report.maxShapeDifferenceMm=pairs.length?Math.max(...pairs.map(r=>r.maxShapeDifferenceMm)):null;
    report.maxTipDifferenceMm=pairs.length?Math.max(...pairs.flatMap(r=>Object.values(r.tipDifferencesMm))):null;
    report.cpuBudgetMet=report.passed&&report.summary.reduced.withinBudget===steps;
    // Browser throughput and the complete maneuver require separate evidence.
    report.goalAchieved=false;
    report.phaseSummary=Object.fromEntries(['hold','withdraw','advance','settle'].map(phase=>[phase,
        Object.fromEntries(contexts.map(c=>[c.name,summary(pairs.filter(r=>r.phase===phase).map(r=>r[c.name]))]))]));
    for(const context of contexts)context.anatomy.dispose();
    fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');
    console.log(JSON.stringify({passed:report.passed,goalAchieved:report.goalAchieved,summary:report.summary,
        maxShapeDifferenceMm:report.maxShapeDifferenceMm,maxTipDifferenceMm:report.maxTipDifferenceMm}));
}

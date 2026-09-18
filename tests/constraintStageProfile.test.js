import test from 'node:test';
import assert from 'node:assert/strict';
import {ConstraintStageProfile,recordSharedAxisQuality,assessSharedAxisBenchmarkTiming} from '../src/physics/constraintStageProfile.js';

function world() {
    return {wholeStepSystem:{id:'shared-axis'},lastJointCosts:{assemblyMs:999},
        lastJointFactorizations:999,timings:{total:{last:.5}},lastStepResult:null};
}
function accepted(cpuMs=20) {
    return {accepted:true,status:'converged',diagnostics:{last:{cpuMs,wallMs:200,
        timings:{assemblyMs:7,linearMs:8,frictionMs:2},iterations:3,factorizations:5,backtracks:1,
        substepAttempts:2,geometryRestarts:1,frictionIterations:2}}};
}
const assess=(profile,extra={})=>assessSharedAxisBenchmarkTiming({profile:profile.report(),fixedDt:1/60,
    executedSteps:profile.steps,admittedSeconds:profile.steps/60,startBacklogSeconds:0,endBacklogSeconds:0,
    peakBacklogSeconds:0,renderingPass:true,...extra});

test('cooperative slices are counted separately from completed timestep CPU and native costs are not reused',()=>{
    const p=new ConstraintStageProfile(),w=world();
    for(let i=0;i<8;i++){w.lastStepResult={accepted:false,status:'shared-axis-pending',diagnostics:accepted().diagnostics};p.record(w);}
    assert.equal(p.report().steps,0);assert.equal(p.report().pendingSlices,8);assert.equal(p.report().fields.total.mean,null);
    w.lastStepResult=accepted(23);p.record(w,{fullStepCpuMs:28});p.record(w,{fullStepCpuMs:28});
    const r=p.report();assert.equal(r.steps,1);assert.equal(r.solverSteps,1);assert.equal(r.failedSteps,0);
    assert.equal(r.fields.total.mean,28);assert.equal(r.fields.activeStepCpuMs.mean,28);assert.equal(r.fields.providerCpuMs.mean,23);
    assert.equal(r.fields.assemblyMs.mean,7);assert.equal(r.fields.linearMs.mean,8);assert.equal(r.fields.factorizations.sum,5);
    assert.equal(r.fields.snapshotMs.mean,null);assert.equal(r.fields.narrowPhase.mean,null);
    assert.equal(r.timingSource,'preparation-solver-publication-cpu');assert.equal(assess(p).physicsBudgetPass,false);
});

test('sleep steps neither replay old solver costs nor hide expensive active timesteps',()=>{
    const p=new ConstraintStageProfile(),w=world();w.lastStepResult=accepted(30);p.record(w,{fullStepCpuMs:31});
    for(let i=0;i<100;i++) {
        w.lastStepResult={accepted:true,status:'sleeping',diagnostics:w.lastStepResult.diagnostics};p.record(w,{fullStepCpuMs:.1});
    }
    const r=p.report();assert.equal(r.steps,101);assert.equal(r.solverSteps,1);assert.equal(r.sleepingSteps,100);
    assert.equal(r.fields.factorizations.sum,5);assert.equal(r.fields.activeStepCpuMs.mean,31);
    assert.ok(r.fields.total.mean<1);assert.equal(assess(p).physicsBudgetPass,false);
});

test('adaptive provider reports its own whole-step costs instead of legacy projection costs',()=>{
    const p=new ConstraintStageProfile(),w=world();w.wholeStepSystem.id='shared-axis-adaptive';
    w.lastStepResult=accepted(12);p.record(w,{fullStepCpuMs:14});
    const r=p.report();assert.equal(r.mode,'shared-axis-adaptive');assert.equal(r.lineSearch,null);
    assert.equal(r.fields.activeStepCpuMs.mean,14);assert.equal(r.fields.factorizations.sum,5);
});

test('terminal failures are distinguished from pending slices and missing telemetry cannot pass',()=>{
    const p=new ConstraintStageProfile(),w=world();
    assert.equal(assess(p).physicsBudgetPass,false);
    w.lastStepResult={accepted:false,status:'shared-axis-pending'};p.record(w);
    w.lastStepResult={accepted:false,status:'line-search',diagnostics:{last:{error:'failed physical solve'}}};p.record(w);p.record(w);
    assert.equal(p.report().pendingSlices,1);assert.equal(p.report().failedSteps,1);assert.equal(p.report().firstFailure.status,'line-search');
    w.lastStepResult={accepted:true,status:'converged',diagnostics:{last:{}}};p.record(w);
    assert.equal(p.report().fields.total.sampleCount,0);assert.equal(assess(p).samplesPass,false);
    p.reset();assert.equal(p.steps,0);assert.equal(p.failedSteps,0);assert.equal(p.report().firstFailure,null);
});

test('60 Hz budget uses full active-step costs and rendering success cannot hide simulation backlog',()=>{
    const p=new ConstraintStageProfile(),w=world();
    for(let i=0;i<60;i++){w.lastStepResult=accepted(5);p.record(w,{fullStepCpuMs:6});}
    const good=assess(p);assert.deepEqual(good.limits,{meanStepMs:8,p95StepMs:12});assert.equal(good.realTime60FpsPass,true);
    const lag=assess(p,{admittedSeconds:2,endBacklogSeconds:1,peakBacklogSeconds:1});
    assert.equal(lag.physicsBudgetPass,true);assert.equal(lag.renderingPass,true);assert.equal(lag.simulationRealtimePass,false);
    assert.equal(lag.simulatedToAdmittedTimeRatio,.5);assert.equal(lag.realTime60FpsPass,false);
    assert.equal(assess(p,{renderingPass:false}).realTime60FpsPass,false);
});

function envelope() {
    return {steps:0,finite:true,maxPostStepPenetrationMm:0,maxSegmentErrorPercent:0,maxBendAngleDegrees:0,
        maxGuidewireRawDisplacementSpeedMmPerSecond:999,maxGuidewireProjectionLeakSpeedMmPerSecond:999};
}
function qualityResult() {
    return {accepted:true,status:'converged',diagnostics:{last:{certificateBound:9e-7,quality:{finite:true,maxPenetration:.00001,bodies:[
        {id:'wire',finite:true,maxLengthError:.0002,maxBendAngleDegrees:24,maxBendNode:10,maxBendLimitDegrees:45,maxSpeed:3},
        {id:'catheter',finite:true,maxLengthError:.0001,maxBendAngleDegrees:35,maxBendNode:12,maxBendLimitDegrees:45,maxSpeed:4}
    ]}}}};
}

test('shared quality reports physical common-axis strain and bend instead of native display resampling',()=>{
    const e=envelope(),r=qualityResult();recordSharedAxisQuality(e,r);
    assert.equal(e.steps,1);assert.equal(e.solverSteps,1);assert.equal(e.source,'shared-axis-quality');
    assert.equal(e.maxPostStepPenetrationMm,.00001);assert.equal(e.maxSegmentErrorPercent,.02);assert.equal(e.maxSegmentErrorBodyId,'wire');
    assert.equal(e.maxBendAngleDegrees,35);assert.equal(e.maxBendBodyId,'catheter');assert.equal(e.maxBendNodeIndex,12);
    assert.equal(e.maxBendLimitDegrees,45);assert.equal(e.maxSharedAxisSpeedMmPerSecond,4);assert.equal(e.maxCertifiedResidual,9e-7);
    assert.equal(e.maxGuidewireRawDisplacementSpeedMmPerSecond,null);assert.equal(e.maxGuidewireProjectionLeakSpeedMmPerSecond,null);
    assert.equal(e.maxTransientPenetrationMm,null);assert.equal(e.maxPostStepPenetrationX,null);assert.equal(e.maxBendX,null);
});

test('sleep quality is not recorded again and absent/nonfinite physical certificates are explicit',()=>{
    const e=envelope(),r=qualityResult();recordSharedAxisQuality(e,r);
    r.status='sleeping';r.diagnostics.last.quality.maxPenetration=999;recordSharedAxisQuality(e,r);
    assert.equal(e.steps,2);assert.equal(e.solverSteps,1);assert.equal(e.sleepingSteps,1);assert.equal(e.maxPostStepPenetrationMm,.00001);
    assert.equal(recordSharedAxisQuality(e,{accepted:false,status:'shared-axis-pending'}),false);assert.equal(e.steps,2);
    recordSharedAxisQuality(e,{accepted:true,status:'converged'});assert.equal(e.missingQualitySteps,1);assert.equal(e.finite,false);
});

test('legacy profile retains actual synchronous costs and bounded percentile sampling',()=>{
    const p=new ConstraintStageProfile(2),w={lastJointCosts:{assemblyMs:2},timings:Object.fromEntries(
        ['total','constraints','narrowPhase','integrate','velocity'].map(key=>[key,{last:4}]))};
    for(let i=0;i<3;i++)p.record(w);
    const r=p.report();assert.equal(r.steps,3);assert.equal(r.percentileSteps,2);assert.equal(r.fields.total.mean,4);
    assert.equal(r.fields.total.sampleCount,3);assert.equal(r.fields.total.percentileSamples,2);assert.equal(r.fields.assemblyMs.mean,2);
    assert.equal(r.fields.activeStepCpuMs.mean,null);
});


test('PD keeps its timing identity and does not invent a Newton residual certificate',()=>{
    const w=world(),p=new ConstraintStageProfile();w.wholeStepSystem.id='shared-axis-projective';
    const result=accepted();result.diagnostics.last.pd={localGlobalConverged:false};
    result.diagnostics.last.quality={finite:true,maxPenetration:.02,bodies:[{id:'wire',finite:true,maxLengthError:.01,maxSpeed:2,maxBendAngleDegrees:10,maxBendLimitDegrees:45}]};
    w.lastStepResult=result;p.record(w);assert.equal(p.report().mode,'shared-axis-projective');assert.equal(p.report().lineSearch,null);
    const envelope={steps:0,maxSegmentErrorPercent:0,maxBendAngleDegrees:0,maxPostStepPenetrationMm:0,finite:true};
    recordSharedAxisQuality(envelope,result);assert.equal(envelope.maxCertifiedResidual,null);assert.equal(envelope.maxSegmentErrorPercent,1);
});

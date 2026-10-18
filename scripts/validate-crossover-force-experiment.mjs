// Research-only report: explicitly relaxed force equilibrium for both long
// tools. This is not a pass of the original 1e-6 physical acceptance gate.
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {validateCrossoverRecording,crossoverShapeDifference,summarizeCrossoverRecording} from '../src/benchmark/crossoverScenarioRecording.js';

const [referencePath,candidatePath,outputPath]=process.argv.slice(2);
assert.ok(referencePath&&candidatePath&&outputPath,
    'Usage: node scripts/validate-crossover-force-experiment.mjs reference.json.gz candidate.json.gz output.json');
const load=path=>{
    const bytes=fs.readFileSync(path);
    return {bytes,record:JSON.parse(path.endsWith('.gz')?gunzipSync(bytes):bytes)};
};
const reference=load(referencePath),candidate=load(candidatePath);
const a=validateCrossoverRecording(reference.record),b=candidate.record;
const hash=bytes=>createHash('sha256').update(bytes).digest('hex');
assert.equal(hash(reference.bytes),'bf2067e132ee933e052f39c4a68ba8d1f7e0c2fada445cedc86177a70324dbac');
assert.equal(b.kind,a.kind);assert.equal(b.schemaVersion,a.schemaVersion);assert.equal(b.dt,a.dt);
assert.equal(b.complete,true);assert.equal(b.fullManeuverRecorded,true);assert.equal(b.inputSequenceVerified,true);
assert.equal(b.replayOf,a.recordedAt);assert.equal(b.steps.length,a.steps.length);
assert.deepEqual(b.checkpoints,a.checkpoints);assert.equal(b.checkpoints.length,5);
assert.deepEqual(b.configuration,a.configuration);
assert.equal(b.validation.shapeDifferenceLimitMm,1);
const relaxedForceTolerance=b.validation.forceCertificateLimit;
assert.ok([1e-5,1e-4].includes(relaxedForceTolerance));
const bounded=(value,limit)=>Number.isFinite(value)&&value>=0&&value<=limit;
let maximumShapeDifferenceMm=0,maximumCertificate=0,maximumPenetrationMm=0,maximumLengthErrorMm=0;
let firstRelaxedStep=null,relaxedSteps=0,originalCertificateFailures=0;
for(let i=0;i<a.steps.length;i++) {
    const expected=a.steps[i],actual=b.steps[i];
    for(const key of ['inputs','tools','stage','moving'])assert.deepEqual(actual[key],expected[key],`Step ${i+1}: ${key}`);
    for(const key of ['intervalMs','cpuMs'])assert.ok(bounded(actual[key],Number.MAX_VALUE));
    const difference=crossoverShapeDifference(expected.shape,actual.shape);
    assert.ok(difference<=1,`Step ${i+1}: ${difference} mm`);
    assert.ok(Math.abs(actual.shapeDifferenceMm-difference)<=1e-10);
    maximumShapeDifferenceMm=Math.max(maximumShapeDifferenceMm,difference);
    const eligible=actual.tools.wire>=400&&actual.tools.catheter>=400;
    assert.equal(actual.quality.length,2);assert.equal(actual.solverWork.length,2);
    for(let side=0;side<2;side++) {
        const q=actual.quality[side],limit=side===0&&eligible?relaxedForceTolerance:1e-6;
        const used=actual.solverWork[side].forceTolerance;
        assert.ok(used===1e-6||side===0&&eligible&&used===relaxedForceTolerance,`Step ${i+1}: unexpected force tolerance`);
        if(used===relaxedForceTolerance){firstRelaxedStep??=i+1;relaxedSteps++;}
        assert.equal(q.converged,true);assert.equal(q.quality.finite,true);
        assert.ok(bounded(q.certificateBound,Math.min(limit,used)));
        assert.ok(bounded(q.residual.length,1e-5));assert.ok(bounded(q.quality.maxPenetration,1e-5));
        assert.ok(q.quality.bodies.length>0);
        for(const body of q.quality.bodies)assert.ok(bounded(body.maxLengthError,1e-5)&&body.maxLengthError<1e-5);
        if(q.certificateBound>1e-6)originalCertificateFailures++;
        maximumCertificate=Math.max(maximumCertificate,q.certificateBound);
        maximumPenetrationMm=Math.max(maximumPenetrationMm,q.quality.maxPenetration);
        maximumLengthErrorMm=Math.max(maximumLengthErrorMm,q.residual.length,...q.quality.bodies.map(body=>body.maxLengthError));
    }
    // The strict part of this experiment must reproduce the reference exactly.
    if(firstRelaxedStep===null)assert.equal(difference,0,`Step ${i+1}: strict prefix changed`);
}
const summary=summarizeCrossoverRecording(b.steps),minimumMeasuredHz=59.5;
const performanceTargetMet=summary.duringMotion.physicsHz>=minimumMeasuredHz&&
    summary.stages.every(stage=>stage.duringMotion.steps>0&&stage.duringMotion.physicsHz>=minimumMeasuredHz)&&
    summary.minimumMovingWindowHz>=minimumMeasuredHz;
const report={reference:referencePath,candidate:candidatePath,referenceSha256:hash(reference.bytes),candidateSha256:hash(candidate.bytes),
    steps:b.steps.length,dt:b.dt,verifiedInputsAndTools:true,verifiedCheckpoints:5,qualityChecks:b.steps.length*2,
    experimentalForcePolicy:{thresholdInsertionMm:400,strictForceTolerance:1e-6,longInsertionForceTolerance:relaxedForceTolerance,
        firstRelaxedStep,relaxedSteps,originalCertificateFailures},
    shapeDifferenceLimitMm:1,maximumShapeDifferenceMm,maximumCertificate,maximumPenetrationMm,maximumLengthErrorMm,
    summary,performanceTargetMet,goalAchieved:false,productionDefaultChanged:false,
    limitations:['This experiment relaxes force equilibrium; it does not satisfy the original 1e-6 certificate on all steps.',
        'The insertion threshold is a research restriction, not a detector of anatomical branch selection.',
        'Stage placement is operator-confirmed. This result is not universal validation of the accuracy policy.']};
fs.writeFileSync(outputPath,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({steps:report.steps,maximumShapeDifferenceMm,motionHz:summary.duringMotion.physicsHz,
    lastStageMotionHz:summary.stages[4].duringMotion.physicsHz,firstRelaxedStep,originalCertificateFailures,performanceTargetMet}));

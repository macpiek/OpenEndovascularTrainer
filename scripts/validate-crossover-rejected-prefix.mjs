// Inspect a rejected browser replay without turning a partial run into a pass.
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {crossoverShapeDifference,summarizeCrossoverRecording} from '../src/benchmark/crossoverScenarioRecording.js';

const [referencePath,candidatePath,outputPath]=process.argv.slice(2);
assert.ok(referencePath&&candidatePath&&outputPath,'Provide reference, rejected replay and report paths');
const read=path=>{const bytes=fs.readFileSync(path);return {bytes,data:JSON.parse(gunzipSync(bytes))};};
const a=read(referencePath),b=read(candidatePath),hash=x=>createHash('sha256').update(x).digest('hex');
assert.equal(hash(a.bytes),'bf2067e132ee933e052f39c4a68ba8d1f7e0c2fada445cedc86177a70324dbac');
assert.equal(b.data.complete,false);assert.equal(b.data.fullManeuverRecorded,false);
assert.equal(b.data.replayOf,a.data.recordedAt);assert.equal(b.data.dt,a.data.dt);
assert.deepEqual(b.data.configuration,a.data.configuration);assert.deepEqual(b.data.checkpoints,a.data.checkpoints);
assert.equal(b.data.validation.shapeDifferenceLimitMm,1);
assert.ok(b.data.steps.length>0&&b.data.steps.length<a.data.steps.length);
let maxDifference=0,firstDifferentStep=null,firstOverLimitStep=null,maxCertificate=0,maxPenetration=0,maxLengthError=0;
const bounded=(v,limit)=>Number.isFinite(v)&&v>=0&&v<=limit;
for(let i=0;i<b.data.steps.length;i++) {
    const expected=a.data.steps[i],actual=b.data.steps[i];
    for(const key of ['inputs','tools','stage','moving'])assert.deepEqual(actual[key],expected[key],`Step ${i+1}: ${key}`);
    assert.ok(bounded(actual.intervalMs,Number.MAX_VALUE));
    const d=crossoverShapeDifference(expected.shape,actual.shape);
    assert.ok(Number.isFinite(d));assert.ok(Math.abs(d-actual.shapeDifferenceMm)<1e-10);
    if(d>0)firstDifferentStep??=i+1;
    if(d>1)firstOverLimitStep??=i+1;
    maxDifference=Math.max(maxDifference,d);
    assert.equal(actual.quality.length,2);
    for(const q of actual.quality) {
        assert.equal(q.converged,true);assert.equal(q.quality.finite,true);
        assert.ok(bounded(q.certificateBound,1e-6));assert.ok(bounded(q.quality.maxPenetration,1e-5));
        assert.ok(bounded(q.residual.length,1e-5));assert.ok(q.quality.bodies.length>0);
        for(const body of q.quality.bodies)assert.ok(bounded(body.maxLengthError,1e-5));
        maxCertificate=Math.max(maxCertificate,q.certificateBound);
        maxPenetration=Math.max(maxPenetration,q.quality.maxPenetration);
        maxLengthError=Math.max(maxLengthError,q.residual.length,...q.quality.bodies.map(x=>x.maxLengthError));
    }
}
assert.equal(firstOverLimitStep,b.data.steps.length,'Expected stop at the first shape-budget violation');
const report={reference:referencePath,candidate:candidatePath,referenceSha256:hash(a.bytes),candidateSha256:hash(b.bytes),
    complete:false,accepted:false,goalAchieved:false,targetSteps:a.data.steps.length,verifiedTargetPrefix:true,
    steps:b.data.steps.length,firstDifferentStep,firstOverLimitStep,maximumShapeDifferenceMm:maxDifference,
    physicalChecks:b.data.steps.length*2,maximumCertificate:maxCertificate,maximumPenetrationMm:maxPenetration,
    maximumLengthErrorMm:maxLengthError,summary:summarizeCrossoverRecording(b.data.steps),
    scope:'Rejected browser prefix only. Neither its throughput nor physical admissibility constitutes a complete-scenario pass.'};
fs.writeFileSync(outputPath,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({steps:report.steps,firstDifferentStep,maximumShapeDifferenceMm:maxDifference}));

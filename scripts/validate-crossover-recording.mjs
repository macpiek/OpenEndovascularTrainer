// Verify a complete app replay against the original human input recording.
// Run after (never concurrently with) the browser's wall-clock measurement.
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {validateCrossoverRecording,crossoverShapeDifference,summarizeCrossoverRecording} from '../src/benchmark/crossoverScenarioRecording.js';
import {createCrossoverRouteCheck,summarizeCrossoverRoute} from '../src/benchmark/crossoverFunctionalRoute.js';
import {createCrossoverFlow} from '../tests/helpers/crossoverFlow.js';

const [referencePath,candidatePath,outputPath]=process.argv.slice(2);
assert.ok(referencePath&&candidatePath&&outputPath,
    'Usage: node scripts/validate-crossover-recording.mjs reference.json.gz candidate.json.gz output.json');
const load=path=>{
    const bytes=fs.readFileSync(path);
    return {bytes,record:validateCrossoverRecording(JSON.parse(path.endsWith('.gz')?gunzipSync(bytes):bytes))};
};
const reference=load(referencePath),candidate=load(candidatePath),a=reference.record,b=candidate.record;
const policy=JSON.parse(fs.readFileSync(new URL('../reports/performance-scenarios/004-optimization-policy.json',import.meta.url)));
const hash=bytes=>createHash('sha256').update(bytes).digest('hex');
assert.equal(hash(reference.bytes),policy.referenceSha256,'Use the original full human recording');
assert.equal(a.fullManeuverRecorded,true);
assert.equal(b.fullManeuverRecorded,true);
assert.equal(b.inputSequenceVerified,true);
assert.equal(b.replayOf,a.recordedAt);
assert.equal(b.steps.length,a.steps.length);
assert.equal(a.checkpoints.length,5);
assert.deepEqual(b.checkpoints,a.checkpoints);
assert.deepEqual(b.configuration,a.configuration);
let maximumShapeDifferenceMm=0,maximumCertificate=0,maximumPenetrationMm=0,maximumLengthErrorMm=0;
for(let i=0;i<a.steps.length;i++) {
    const expected=a.steps[i],actual=b.steps[i];
    for(const key of ['inputs','tools','stage','moving'])assert.deepEqual(actual[key],expected[key],`Step ${i+1}: ${key}`);
    const difference=crossoverShapeDifference(expected.shape,actual.shape);
    if(policy.shapeDifferenceLimitMm!==null)
        assert.ok(difference<=policy.shapeDifferenceLimitMm,`Step ${i+1}: shape differs by ${difference} mm`);
    assert.ok(Math.abs((actual.shapeDifferenceMm??0)-difference)<=1e-10,`Step ${i+1}: recorded shape metric differs`);
    maximumShapeDifferenceMm=Math.max(maximumShapeDifferenceMm,difference);
    for(const q of actual.quality) {
        // validateCrossoverRecording already checks the physical gates for
        // both accesses. Retain their maxima as inspectable evidence.
        maximumCertificate=Math.max(maximumCertificate,q.certificateBound);
        maximumPenetrationMm=Math.max(maximumPenetrationMm,q.quality.maxPenetration);
        maximumLengthErrorMm=Math.max(maximumLengthErrorMm,q.residual.length,...q.quality.bodies.map(body=>body.maxLengthError));
    }
}
const summary=summarizeCrossoverRecording(b.steps),minimumMeasuredHz=59.5;
const routeCheck=createCrossoverRouteCheck(createCrossoverFlow());
const functionalRoute=summarizeCrossoverRoute(b.checkpoints.map(c=>{
    const step=b.steps[c.afterStep-1];return routeCheck.checkpoint(c.stage,step.shape,step.tools);
}),{complete:b.complete});
const performanceTargetMet=summary.duringMotion.physicsHz>=minimumMeasuredHz&&
    summary.stages.every(stage=>stage.duringMotion.steps>0&&stage.duringMotion.physicsHz>=minimumMeasuredHz)&&
    summary.minimumMovingWindowHz>=minimumMeasuredHz;
const report={reference:referencePath,candidate:candidatePath,referenceSha256:hash(reference.bytes),candidateSha256:hash(candidate.bytes),
    steps:b.steps.length,dt:b.dt,verifiedInputsAndTools:true,verifiedCheckpoints:5,
    acceptanceMode:policy.acceptanceMode,shapeComparisonRole:policy.shapeDifferenceLimitMm===null?'diagnostic':'acceptance',
    shapeDifferenceLimitMm:policy.shapeDifferenceLimitMm,maximumShapeDifferenceMm,functionalRoute,
    qualityChecks:b.steps.length*2,maximumCertificate,maximumPenetrationMm,maximumLengthErrorMm,
    summary,performanceTargetMet,minimumMeasuredHz,goalAchieved:false,
    scope:'Entire recorded maneuver from empty tools. Throughput uses common accepted steps per wall-clock second during movement. Rendering FPS is not a performance gate.',
    limitations:['Branch screening is coarse. Review the complete movement for pigtail seating, femoral entry, stability and freedom from blocking.',
        'A completed report alone does not replace review of browser conditions, regressions and the final implementation.']};
fs.writeFileSync(outputPath,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({steps:report.steps,maximumShapeDifferenceMm,motionHz:summary.duringMotion.physicsHz,
    lastStageMotionHz:summary.stages[4].duringMotion.physicsHz,performanceTargetMet}));

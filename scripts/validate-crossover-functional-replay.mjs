// Audit a complete or explicitly interrupted full-app replay. A valid prefix
// is never promoted to a completed maneuver or a performance result.
import fs from 'node:fs';
import assert from 'node:assert/strict';
import {gunzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import {validateCrossoverRecording,crossoverShapeDifference,summarizeCrossoverRecording} from '../src/benchmark/crossoverScenarioRecording.js';
import {createCrossoverRouteCheck,summarizeCrossoverRoute} from '../src/benchmark/crossoverFunctionalRoute.js';
import {createCrossoverFlow} from '../tests/helpers/crossoverFlow.js';

const [candidatePath,outputPath,forceLimitArgument='0.000001']=process.argv.slice(2);
assert.ok(candidatePath&&outputPath,'Usage: candidate.json.gz output.json [explicit force tolerance]');
const forceCertificateLimit=Number(forceLimitArgument);
assert.ok(Number.isFinite(forceCertificateLimit)&&forceCertificateLimit>0&&forceCertificateLimit<=1e-4);
const referencePath='reports/performance-scenarios/003-crossover-full-user-inputs.json.gz';
const referenceBytes=fs.readFileSync(referencePath),candidateBytes=fs.readFileSync(candidatePath);
const hash=b=>createHash('sha256').update(b).digest('hex');
assert.equal(hash(referenceBytes),'bf2067e132ee933e052f39c4a68ba8d1f7e0c2fada445cedc86177a70324dbac');
const reference=validateCrossoverRecording(JSON.parse(gunzipSync(referenceBytes)));
const candidate=JSON.parse(gunzipSync(candidateBytes));
assert.equal(candidate.replayOf,reference.recordedAt);
assert.equal(candidate.validation.shapeDifferenceLimitMm,null);
assert.equal(candidate.validation.forceCertificateLimit??1e-6,forceCertificateLimit);
assert.deepEqual(candidate.configuration,reference.configuration);
assert.deepEqual(candidate.checkpoints,reference.checkpoints);
assert.ok(candidate.steps.length>0&&candidate.steps.length<=reference.steps.length);
const complete=candidate.steps.length===reference.steps.length&&candidate.complete===true&&!candidate.error;
assert.equal(candidate.complete,complete);
if(!complete)assert.ok(candidate.error,'Interrupted replay must state the failure');
// Reuse the per-step format/physics validator on an isolated prefix view. The
// original report and its completeness flag remain unchanged in the audit.
validateCrossoverRecording({...candidate,complete:true,
    checkpoints:candidate.checkpoints.filter(c=>c.afterStep<=candidate.steps.length)},{forceCertificateLimit});
let maxDifference=0,maxCertificate=0,maxPenetration=0,maxLength=0;
for(const [i,s] of candidate.steps.entries()) {
    const expected=reference.steps[i];
    for(const key of ['inputs','tools','stage','moving'])assert.deepEqual(s[key],expected[key],`Step ${i+1}: ${key}`);
    const difference=crossoverShapeDifference(expected.shape,s.shape);
    assert.ok(Math.abs(difference-s.shapeDifferenceMm)<1e-10);
    maxDifference=Math.max(maxDifference,difference);
    for(const q of s.quality){maxCertificate=Math.max(maxCertificate,q.certificateBound);
        maxPenetration=Math.max(maxPenetration,q.quality.maxPenetration);
        maxLength=Math.max(maxLength,q.residual.length,...q.quality.bodies.map(b=>b.maxLengthError));}
}
const check=createCrossoverRouteCheck(createCrossoverFlow());
const functionalRoute=summarizeCrossoverRoute(candidate.checkpoints.filter(c=>c.afterStep<=candidate.steps.length).map(c=>{
    const s=candidate.steps[c.afterStep-1];return check.checkpoint(c.stage,s.shape,s.tools);
}),{complete});
const summary=summarizeCrossoverRecording(candidate.steps);
const performanceTargetMet=complete&&summary.duringMotion.physicsHz>=59.5&&summary.minimumMovingWindowHz>=59.5&&
    summary.stages.every(s=>s.duringMotion.steps>0&&s.duringMotion.physicsHz>=59.5);
const report={candidate:candidatePath,candidateSha256:hash(candidateBytes),referenceSha256:hash(referenceBytes),
    acceptanceMode:'functional',shapeDifferenceLimitMm:null,forceCertificateLimit,
    complete,acceptedSteps:candidate.steps.length,targetSteps:reference.steps.length,error:candidate.error??null,
    verifiedInputPrefix:true,physicalChecks:2*candidate.steps.length,maxCertificate,maxPenetration,maxLength,
    maximumShapeDifferenceMm:maxDifference,functionalRoute,summary,performanceTargetMet,goalAchieved:false,
    limitations:['Incomplete replay timing describes only its observed prefix.',
        'Coarse branch checks do not replace review of pigtail seating, femoral entry, jitter and operator control.',
        'A relaxed force tolerance is an explicit research setting, not the strict reference certificate.']};
fs.writeFileSync(outputPath,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({complete,steps:report.acceptedSteps,error:report.error,routeChecksPassed:functionalRoute.routeChecksPassed,
    motionHz:summary.duringMotion.physicsHz,lastStageMotionHz:summary.stages[4].duringMotion.physicsHz,performanceTargetMet,maxDifference,maxCertificate,maxPenetration}));

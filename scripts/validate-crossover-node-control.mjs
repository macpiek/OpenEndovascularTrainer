// A failed or divergent Node control is diagnostic evidence, never a new
// reference that relaxes the original full-browser acceptance requirements.
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {validateCrossoverRecording,crossoverShapeDifference} from '../src/benchmark/crossoverScenarioRecording.js';

const [controlPath,outputPath]=process.argv.slice(2);
assert.ok(controlPath&&outputPath,'Provide Node control and validation output paths');
const directory=new URL('../reports/performance-scenarios/',import.meta.url);
const bytes=fs.readFileSync(new URL('003-crossover-full-user-inputs.json.gz',directory));
const reference=validateCrossoverRecording(JSON.parse(gunzipSync(bytes)));
const policy=JSON.parse(fs.readFileSync(new URL('004-optimization-policy.json',directory)));
const hash=x=>createHash('sha256').update(x).digest('hex');
assert.equal(hash(bytes),policy.referenceSha256);
const candidateBytes=fs.readFileSync(controlPath),control=JSON.parse(gunzipSync(candidateBytes));
assert.equal(control.kind,'crossover-node-control');assert.equal(control.schemaVersion,1);
assert.equal(control.sourceSha256,hash(bytes));assert.equal(control.targetSteps,reference.steps.length);
assert.equal(control.dt,reference.dt);assert.deepEqual(control.configuration,reference.configuration);
assert.equal(control.goalAchieved,false);assert.equal(control.applicationFpsVerified,false);
assert.equal(control.candidateQualified,false);
assert.equal(control.complete,control.rows.length===reference.steps.length&&!control.error);
assert.ok(control.rows.length>0&&control.rows.length<=reference.steps.length);
const bounded=(value,limit)=>Number.isFinite(value)&&value>=0&&value<=limit;
// Older control reports were explicitly measured with the historical 1 mm
// policy; do not reinterpret their recorded diagnostic when policy changes.
const shapeLimit=Object.hasOwn(control,'shapeDifferenceLimitMm')?control.shapeDifferenceLimitMm:1;
let maximumDifference=0,firstBudgetViolation=null,maxCertificate=0,maxPenetration=0,maxLength=0;
for(const [i,row] of control.rows.entries()) {
    assert.equal(row.step,i+1);
    for(const key of ['stage','moving','tools'])assert.deepEqual(row[key],reference.steps[i][key]);
    const difference=crossoverShapeDifference(reference.steps[i].shape,row.shape);
    assert.equal(difference,row.browserShapeDifferenceMm);
    if(shapeLimit!==null&&difference>shapeLimit)firstBudgetViolation??=i+1;
    maximumDifference=Math.max(maximumDifference,difference);
    const q=row.quality;
    assert.equal(q.converged,true);assert.equal(q.quality.finite,true);
    assert.ok(bounded(q.certificateBound,1e-6)&&bounded(q.quality.maxPenetration,1e-5)&&bounded(q.residual.length,1e-5));
    assert.ok(q.quality.bodies.length>0&&q.quality.bodies.every(b=>bounded(b.maxLengthError,1e-5)));
    maxCertificate=Math.max(maxCertificate,q.certificateBound);
    maxPenetration=Math.max(maxPenetration,q.quality.maxPenetration);
    maxLength=Math.max(maxLength,q.residual.length,...q.quality.bodies.map(b=>b.maxLengthError));
}
assert.equal(maximumDifference,control.maximumBrowserShapeDifferenceMm);
assert.equal(firstBudgetViolation,control.firstBrowserBudgetViolation);
if(!control.complete)assert.ok(control.error,'Incomplete control must state its failure');
const report={control:controlPath,sha256:hash(candidateBytes),verifiedPrefixSteps:control.rows.length,
    targetSteps:reference.steps.length,complete:control.complete,workerSteps:control.workerSteps,
    firstBrowserBudgetViolation:firstBudgetViolation,maximumBrowserShapeDifferenceMm:maximumDifference,
    maximumCertificate:maxCertificate,maximumPenetrationMm:maxPenetration,maximumLengthErrorMm:maxLength,
    error:control.error??null,reproducibilityVerified:false,qualifiedReference:false,goalAchieved:false,
    scope:'Checks original target prefix, every recorded physical gate and cross-runtime shape metric. No application FPS or same-runtime reproducibility claim.'};
fs.writeFileSync(outputPath,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({steps:control.rows.length,complete:control.complete,firstBudgetViolation,maximumDifference}));

// Diagnose runtime parity independently of speed and candidate qualification.
// A math oracle is a recorded arithmetic trace, never a deployable solver.
import fs from 'node:fs';
import path from 'node:path';
import {gunzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {isDeepStrictEqual} from 'node:util';
import {validateCrossoverRecording,crossoverShapeDifference} from '../src/benchmark/crossoverScenarioRecording.js';

const directory=process.argv[2]??'reports/performance-scenarios';
const output=process.argv[3]??path.join(directory,'017-runtime-audit-validation.json');
const names={reference:'003-crossover-full-user-inputs.json.gz',browser:'017-browser-inputs.json.gz',
    node:'017-node-worker-inputs.json.gz',browserMath:'017-browser-math.json.gz',
    nodeMath:'017-node-math.json.gz',oracle:'017-node-math-oracle.json.gz'};
const hashes={},records={};
for(const [key,name] of Object.entries(names)) {
    const bytes=fs.readFileSync(path.join(directory,name));
    hashes[key]=createHash('sha256').update(bytes).digest('hex');
    records[key]=JSON.parse(gunzipSync(bytes));
}
const policy=JSON.parse(fs.readFileSync(path.join(directory,'004-optimization-policy.json')));
assert.equal(hashes.reference,policy.referenceSha256);
const {reference,browser,node,browserMath,nodeMath,oracle}=records;
validateCrossoverRecording(reference);
assert.equal(reference.fullManeuverRecorded,true);
const bounded=(value,limit)=>Number.isFinite(value)&&value>=0&&value<=limit;
const validateQuality=q=>{
    assert.equal(q.converged,true);assert.equal(q.quality.finite,true);
    assert.ok(bounded(q.certificateBound,1e-6)&&bounded(q.residual.length,1e-5)&&bounded(q.quality.maxPenetration,1e-5));
    assert.ok(q.quality.bodies.length>0&&q.quality.bodies.every(b=>bounded(b.maxLengthError,1e-5)));
};
for(const capture of [browser,browserMath]) {
    // The recording validator requires a complete file. Audit prefixes retain
    // their real incomplete status; validate their original inputs and gates.
    assert.equal(capture.schemaVersion,reference.schemaVersion);assert.equal(capture.kind,reference.kind);
    assert.equal(capture.dt,reference.dt);assert.equal(capture.complete,false);
    assert.equal(capture.fullManeuverRecorded,false,'Diagnostic capture must not claim completion');
    assert.equal(capture.replayOf,reference.recordedAt);
    assert.deepEqual(capture.configuration,reference.configuration);
    assert.deepEqual(capture.checkpoints,reference.checkpoints);
    assert.ok(capture.steps.length>0&&capture.steps.length<reference.steps.length);
    for(const [i,step] of capture.steps.entries()) {
        for(const key of ['inputs','tools','stage','moving','shape'])
            assert.deepEqual(step[key],reference.steps[i][key],`Passive capture changed step ${i+1}: ${key}`);
        assert.equal(step.quality.length,2);step.quality.forEach(validateQuality);
    }
}
assert.equal(node.rows.length,browser.steps.length);
let lastWorkerStep=0,maxDifference=0,firstDifferentStep=null;
for(const [i,row] of node.rows.entries()) {
    const capture=browser.steps[i],audit=capture.solverWork[0].requestAudit;
    assert.equal(row.step,i+1);assert.equal(row.workerStep,audit.step);
    assert.ok(audit.step===lastWorkerStep||audit.step===lastWorkerStep+1);
    lastWorkerStep=audit.step;
    assert.equal(audit.request.dt,reference.dt);
    const difference=crossoverShapeDifference(capture.shape,row.shape);
    assert.equal(row.shapeDifferenceMm,difference);
    maxDifference=Math.max(maxDifference,difference);
    if(difference>0)firstDifferentStep??=i+1;
    validateQuality(row.quality);
}
assert.equal(lastWorkerStep,browser.steps[0].solverWork[0].requestAudit.limit);
assert.equal(maxDifference,node.maximumShapeDifferenceMm);
assert.equal(firstDifferentStep,node.firstDifferentStep);
assert.equal(oracle.mathOracle,true);
assert.equal(nodeMath.rows.length,browserMath.steps.length);
assert.equal(oracle.rows.length,browserMath.steps.length);
let firstArithmeticDifference=null,validatedOracleCalls=0;
for(const [i,step] of browserMath.steps.entries()) {
    const observed=step.solverWork[0].requestAudit.math,measured=nodeMath.rows[i],replayed=oracle.rows[i];
    assert.equal(measured.step,i+1);assert.equal(replayed.step,i+1);
    assert.equal(observed.length,measured.math.length);
    assert.equal(measured.shapeDifferenceMm,crossoverShapeDifference(step.shape,measured.shape));
    assert.deepEqual(replayed.math,observed,'Oracle must consume the same functions, arguments and results');
    assert.deepEqual(replayed.shape,step.shape,'Oracle must recover the browser pose exactly');
    assert.equal(replayed.shapeDifferenceMm,0);
    validatedOracleCalls+=observed.length;
    for(let j=0;j<observed.length&&!firstArithmeticDifference;j++) {
        if(isDeepStrictEqual(observed[j],measured.math[j]))continue;
        assert.equal(observed[j].name,measured.math[j].name);
        assert.deepEqual(observed[j].args,measured.math[j].args,'First arithmetic difference must have identical inputs');
        firstArithmeticDifference={step:i+1,callIndex:j,function:observed[j].name,args:observed[j].args,
            browserResult:observed[j].value,nodeResult:measured.math[j].value};
    }
}
assert.ok(firstArithmeticDifference,'Require an observed arithmetic difference');
const report={artifacts:names,sha256:hashes,goalAchieved:false,completeManeuver:false,
    browserCapture:{steps:browser.steps.length,workerRequests:lastWorkerStep,originalShapesUnchanged:true},
    exactRequestNodeReplay:{steps:node.rows.length,firstDifferentStep,maximumShapeDifferenceMm:maxDifference},
    firstArithmeticDifference,mathOracle:{steps:oracle.rows.length,validatedCalls:validatedOracleCalls,exactBrowserShapes:true},
    conclusion:'Identical requests are insufficient for bitwise Node/browser parity. The first observed arithmetic difference is a function return for identical arguments. Replaying the browser arithmetic trace recovers both audited poses exactly.',
    limitations:['The oracle is diagnostic only and cannot run a new maneuver or qualify a physics solver.',
        'Only two steps have arithmetic tracing; this does not prove all later divergence has the same cause.',
        'The 659-step request audit is a prefix, not a performance or full-maneuver test.',
        'Node comparisons require a same-runtime baseline for attribution; the original full browser replay remains authoritative.']};
fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({browserSteps:browser.steps.length,workerRequests:lastWorkerStep,firstArithmeticDifference,
    oracleSteps:oracle.rows.length,validatedOracleCalls,maximumNodeDifferenceMm:maxDifference}));

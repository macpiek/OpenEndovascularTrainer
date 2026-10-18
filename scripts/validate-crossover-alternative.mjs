// Independent coverage/metric audit of research runs. Current reports include
// poses; legacy reports without them cannot certify whole-trajectory shape.
import fs from 'node:fs';
import assert from 'node:assert/strict';
import {gunzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import {summarizeAlternative} from '../src/benchmark/crossoverAlternativeSolvers.js';
import {crossoverShapeDifference} from '../src/benchmark/crossoverScenarioRecording.js';
import {createCrossoverRouteCheck,summarizeCrossoverRoute} from '../src/benchmark/crossoverFunctionalRoute.js';
import {createCrossoverFlow} from '../tests/helpers/crossoverFlow.js';
const [candidatePath,outputPath]=process.argv.slice(2);
assert.ok(candidatePath&&outputPath,'Provide candidate and output paths');
const referencePath='reports/performance-scenarios/003-crossover-full-user-inputs.json.gz';
const bytes=fs.readFileSync(referencePath),decoded=gunzipSync(bytes),reference=JSON.parse(decoded);
const candidateBytes=fs.readFileSync(candidatePath),candidate=JSON.parse(candidatePath.endsWith('.gz')?gunzipSync(candidateBytes):candidateBytes);
const hash=b=>createHash('sha256').update(b).digest('hex');
assert.equal(hash(bytes),'bf2067e132ee933e052f39c4a68ba8d1f7e0c2fada445cedc86177a70324dbac');
assert.ok([hash(bytes),hash(decoded)].includes(candidate.sourceSha256));
assert.ok(candidate.rows.length<=reference.steps.length);
const routeCheck=createCrossoverRouteCheck(createCrossoverFlow()),routeCheckpoints=[];
for(const [i,row] of candidate.rows.entries()) {
    assert.equal(row.step,i+1);assert.deepEqual(row.tools,reference.steps[i].tools);
    for(const key of ['stage','moving'])assert.deepEqual(row[key],reference.steps[i][key]);
    assert.ok(Number.isFinite(row.cpuMs)&&row.cpuMs>=0);
    if(row.accepted) {
        assert.equal(row.quality.finite,true);assert.ok(row.quality.maxPenetration<=1e-5);
        assert.ok(row.quality.bodies.every(b=>b.maxLengthError<=1e-5));
        assert.ok(Number.isFinite(row.shapeDifferenceMm)&&row.shapeDifferenceMm>=0);
        assert.equal(row.shapeWithinLimit,row.shapeDifferenceMm<=1);
        if(row.shape)assert.ok(Math.abs(crossoverShapeDifference(reference.steps[i].shape,row.shape)-row.shapeDifferenceMm)<1e-10);
        const milestone=reference.checkpoints.find(c=>c.afterStep===i+1);
        if(row.shape&&milestone){
            const checked=routeCheck.checkpoint(milestone.stage,row.shape,row.tools);
            if(row.routeCheckpoint)assert.deepEqual(row.routeCheckpoint,checked);
            routeCheckpoints.push(checked);
        }else assert.equal(row.routeCheckpoint,undefined,'Route evidence requires an accepted milestone pose');
        if(row.xpbd?.maxLengthErrorMm!==undefined)assert.ok(row.xpbd.maxLengthErrorMm<=1e-5);
        if(row.pd?.parameters?.hardClosure)assert.ok(row.pd.maxLengthErrorMm<=1e-5);
        assert.ok((row.xpbd?.maxShear??row.pd?.maxShear)<=1e-5);
    } else assert.equal(i,candidate.rows.length-1,'No inputs consumed after rejection');
}
const summary=summarizeAlternative(candidate.rows,reference.steps.length);
const report={reference:referencePath,candidate:candidatePath,referenceSha256:hash(bytes),referenceDecodedSha256:hash(decoded),
    candidateSha256:hash(candidateBytes),rightAccessTargetPrefixVerified:true,stepsChecked:candidate.rows.length,
    expectedSteps:reference.steps.length,summary,
    observedWallThroughputHz:candidate.wallMs?summary.acceptedSteps*1000/candidate.wallMs:null,
    candidatePolylinesVerified:candidate.rows.filter(r=>r.accepted).every(r=>!!r.shape),
    functionalRoute:summarizeCrossoverRoute(routeCheckpoints,{complete:summary.complete}),
    physicalForceCertificate:false,completeCommonStepReplay:false,goalAchieved:false,
    limitations:['Research harness directly applies recorded right-access targets, not the app input handler or both accesses.',
        ...(candidate.rows.some(r=>r.accepted&&!r.shape)?['Shape errors were measured by the running harness; candidate polylines are not included for independent recomputation.']:[]),
        'CPU capacity and the simple browser test are not full-application physics FPS.',
        'Incomplete/rejected trajectories cannot pass optimization acceptance.']};
fs.writeFileSync(outputPath,JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(summary));

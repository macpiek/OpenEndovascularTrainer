// Same-runtime control for solver research. It does not replace the recorded
// browser trajectory, certify an optimization, or measure application FPS.
import fs from 'node:fs';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
import {gunzipSync,gzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {validateCrossoverRecording,crossoverShapeDifference} from '../src/benchmark/crossoverScenarioRecording.js';

const [checkout='.',output='reports/performance-scenarios/018-node-control.json.gz',optionText='{}']=process.argv.slice(2);
const root=pathToFileURL(path.resolve(checkout)+'/'),overrides=JSON.parse(optionText);
const load=async file=>import(new URL(file,root));
const [{createSharedAxisWorkerEngine},{sharedAxisFieldMessage},{loadCoupledRuntimeAnatomy},
    {captureBloodFlow},{createCrossoverFlow},{sharedAxisGeometryIdentity}]=await Promise.all([
    load('src/physics/kirchhoffSharedAxisWorkerEngine.js'),load('src/physics/kirchhoffSharedAxisWorkerData.js'),
    load('tests/helpers/coupledRuntimeFixture.js'),load('src/physics/bloodFlowCoupling.js'),
    load('tests/helpers/crossoverFlow.js'),load('src/physics/kirchhoffSharedAxisGeometryIdentity.js')]);
const hash=bytes=>createHash('sha256').update(bytes).digest('hex');
const directory=new URL('../reports/performance-scenarios/',import.meta.url);
const sourceBytes=fs.readFileSync(new URL('003-crossover-full-user-inputs.json.gz',directory));
const record=validateCrossoverRecording(JSON.parse(gunzipSync(sourceBytes)));
const policy=JSON.parse(fs.readFileSync(new URL('004-optimization-policy.json',directory)));
assert.equal(hash(sourceBytes),policy.referenceSha256);assert.equal(record.fullManeuverRecorded,true);
// Exact published tool layouts and sheath were audited against this recording.
// Only the initial request is reused; later flow is recomputed from each
// evolving Node pose. No recorded pose is restored into the control.
const inputBytes=fs.readFileSync(new URL('017-browser-inputs.json.gz',directory));
const audited=JSON.parse(gunzipSync(inputBytes));
assert.deepEqual(audited.configuration,record.configuration);assert.equal(audited.replayOf,record.recordedAt);
const initial=audited.steps[0].solverWork[0].requestAudit;
const {requestAuditSteps,requestAuditMath,...baseOptions}=initial.options;
const options={...baseOptions,adaptiveMesh:null,...overrides,
    physicsOptions:{...baseOptions.physicsOptions,...overrides.physicsOptions,forceTolerance:1e-6,lengthTolerance:1e-5}};
const anatomy=await loadCoupledRuntimeAnatomy(undefined,record.configuration.anatomy),engine=createSharedAxisWorkerEngine();
assert.equal(sharedAxisGeometryIdentity(anatomy.field.fallbackGeometry),record.configuration.geometry);
const flow=createCrossoverFlow();assert.deepEqual(flow.hemodynamics,record.configuration.flow.parameters);
const sheath=structuredClone(initial.request.sheath),tools=structuredClone(initial.request.tools);
const xyz=p=>Array.isArray(p)?p:[p.x,p.y,p.z];
const sheathLength=Math.hypot(...xyz(sheath.end).map((v,k)=>v-xyz(sheath.start)[k]));
const sourceHashes={};
for(const dir of ['src/physics','src/physics/collision','src/contrast'])
    for(const entry of fs.readdirSync(new URL(dir+'/',root)).filter(x=>x.endsWith('.js')).sort()) {
        const file=dir+'/'+entry;sourceHashes[file]=hash(fs.readFileSync(new URL(file,root)));
    }
const rows=[],report={schemaVersion:1,kind:'crossover-node-control',startedAt:new Date().toISOString(),
    sourceSha256:hash(sourceBytes),auditedInputsSha256:hash(inputBytes),runtimeVersions:process.versions,
    options,sourceHashes,targetSteps:record.steps.length,dt:record.dt,configuration:record.configuration,rows,
    complete:false,goalAchieved:false,candidateQualified:false,applicationFpsVerified:false,
    scope:'Full original targets with recomputed live flow, one access in Node. Browser differences are diagnostic only. This control is not an accepted alternative to the original browser reference.'};
let packet,lastKey=null,workerSteps=0;
try {
    engine.dispatch('initialize',{options,field:structuredClone(sharedAxisFieldMessage(anatomy.field))});
    for(const [index,reference] of record.steps.entries()) {
        for(const tool of tools){tool.insertion=reference.tools[tool.id];tool.rotation=reference.tools[tool.id+'Rotation'];}
        const started=performance.now(),tip=Math.max(...tools.map(t=>t.insertion));
        // There is no blood-flow force inside the sheath, including first step.
        const bloodFlow=tip>sheathLength&&record.configuration.flow.enabled?
            captureBloodFlow(packet.flowState,flow,sheathLength,tip):null;
        const key=JSON.stringify({tools,sheath,flowTime:bloodFlow?.time??null});
        if(!packet||packet.result.status!=='sleeping'||key!==lastKey) {
            packet=engine.dispatch('step',{dt:record.dt,tools,sheath,flow:bloodFlow});workerSteps++;lastKey=key;
        }
        const cpuMs=performance.now()-started;
        assert.ok(packet.result.accepted,`Rejected step ${index+1}: ${packet.result.status}`);
        const q=packet.diagnostics.last,pose=packet.flowState;
        const shape={coordinates:pose.coordinates,positions:pose.positions.flatMap(p=>p.map((v,k)=>v+pose.origin[k]))};
        const difference=crossoverShapeDifference(reference.shape,shape);
        rows.push({step:index+1,stage:reference.stage,moving:reference.moving,tools:reference.tools,cpuMs,
            shape,browserShapeDifferenceMm:difference,mesh:packet.diagnostics.mesh,quality:q});
        assert.ok(q.converged&&q.quality.finite&&q.certificateBound<=1e-6&&q.residual.length<=1e-5&&
            q.quality.maxPenetration<=1e-5&&q.quality.bodies.every(b=>b.maxLengthError<=1e-5),`Physical gate at ${index+1}`);
        flow.update(record.dt);
        if((index+1)%600===0)console.log(JSON.stringify({step:index+1,cpuMs,browserDifferenceMm:difference}));
    }
    report.complete=true;
}catch(error){report.error=error.stack;process.exitCode=1;console.error(error.message);}
finally {
    report.finishedAt=new Date().toISOString();report.workerSteps=workerSteps;
    report.maximumBrowserShapeDifferenceMm=Math.max(0,...rows.map(r=>r.browserShapeDifferenceMm));
    report.shapeDifferenceLimitMm=policy.shapeDifferenceLimitMm;
    report.firstBrowserBudgetViolation=policy.shapeDifferenceLimitMm===null?null:
        rows.find(r=>r.browserShapeDifferenceMm>policy.shapeDifferenceLimitMm)?.step??null;
    engine.dispose();anatomy.dispose();
    fs.writeFileSync(output,gzipSync(JSON.stringify(report)));
    console.log(JSON.stringify({steps:rows.length,complete:report.complete,maximumBrowserShapeDifferenceMm:report.maximumBrowserShapeDifferenceMm}));
}

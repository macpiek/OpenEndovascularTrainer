// Replay every recorded physical input from empty tools. This is a solver
// profiler, not a substitute for the full app's wall-clock motion benchmark.
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {validateCrossoverRecording,crossoverShapeDifference} from '../src/benchmark/crossoverScenarioRecording.js';
import {createSharedAxisAppSystem,SHARED_AXIS_APP_PHYSICS_OPTIONS} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {SHARED_AXIS_BODY_FIELDS} from '../src/physics/kirchhoffSharedAxisWorkerData.js';
import {sharedAxisGeometryIdentity} from '../src/physics/kirchhoffSharedAxisGeometryIdentity.js';
import {CROSSOVER_WIDE_CONTACT_MODEL} from '../src/physics/kirchhoffSharedAxisCrossoverPreset.js';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {createCrossoverFlow} from '../tests/helpers/crossoverFlow.js';
import {GUIDEWIRE_RADIUS_MM,PIGTAIL_CATHETER_RADIUS_MM,INTRODUCER_SHEATH_INNER_RADIUS_MM} from '../src/toolDimensions.js';
import {DEFAULT_TOOL_PROFILES} from '../src/physics/endovascularPhysicsWorld.js';
import {catheterNodeMass} from '../src/physics/catheterDiscretization.js';

const input=process.argv[2]??'reports/performance-scenarios/003-crossover-full-user-inputs.json.gz';
const output=process.argv[3]??'reports/performance-scenarios/003-full-solver-profile.json';
const experiment=JSON.parse(process.argv[4]??'{}');
const policy=JSON.parse(fs.readFileSync(new URL('../reports/performance-scenarios/004-optimization-policy.json',import.meta.url)));
const bytes=fs.readFileSync(input),record=validateCrossoverRecording(JSON.parse(gunzipSync(bytes)));
assert.equal(record.fullManeuverRecorded,true,'Require the complete user maneuver');
const anatomy=await loadCoupledRuntimeAnatomy(undefined,record.configuration.anatomy);
assert.equal(sharedAxisGeometryIdentity(anatomy.field.fallbackGeometry),record.configuration.geometry);
const access=record.configuration.accesses.find(a=>a.id==='right');
const flow=createCrossoverFlow();
assert.deepEqual(flow.hemodynamics,record.configuration.flow.parameters);
const sheath={start:Object.fromEntries(['x','y','z'].map((k,i)=>[k,access.sheath.start[i]])),
    end:Object.fromEntries(['x','y','z'].map((k,i)=>[k,access.sheath.end[i]])),innerRadius:INTRODUCER_SHEATH_INNER_RADIUS_MM,proximalExtension:40};
// Physical profiles used by simulator.js. Render sampling is omitted from
// timing comparisons; these bodies only receive the provider's publication.
const tools=[{id:'wire',type:'glidewire',mass:DEFAULT_TOOL_PROFILES.guidewire.mass,radius:GUIDEWIRE_RADIUS_MM,wallStaticFriction:access.friction[0],wallKineticFriction:access.friction[1]},
    {id:'catheter',type:'pigtail',mass:catheterNodeMass(DEFAULT_TOOL_PROFILES.catheter.mass),radius:PIGTAIL_CATHETER_RADIUS_MM,wallStaticFriction:.002,wallKineticFriction:.002}].map((t,i)=>{
    const count=208,body={count,segmentCount:count-1,...t};
    for(const k of SHARED_AXIS_BODY_FIELDS)body[k]=new Float64Array(k.startsWith('orientation')?count-1:count);
    return {...t,insertion:0,rotation:0,shaftStiffness:access.stiffness[i*2],tipStiffness:access.stiffness[i*2+1],body,
        nodeCoordinates:Array.from({length:count},(_,n)=>-40+5*n)};
});
const system=createSharedAxisAppSystem({readTools:()=>tools,readSheath:()=>sheath,readBloodFlow:()=>record.configuration.flow.enabled?flow:null,
    workSliceMs:Infinity,contactCandidateMargin:.01,lazyBasisCoefficients:true,wasmBasis:true,deferActiveBasis:true,
    reuseFrictionAssembly:true,coupledFrictionNewton:true,predictiveNewton:true,
    ...experiment,adaptiveMesh:experiment.adaptiveMesh==='wide'?CROSSOVER_WIDE_CONTACT_MODEL:experiment.adaptiveMesh??null,
    physicsOptions:{...SHARED_AXIS_APP_PHYSICS_OPTIONS,...experiment.physicsOptions,forceTolerance:1e-6,lengthTolerance:1e-5}});
const world={contactField:anatomy.field},rows=[];
const report={source:input,sha256:createHash('sha256').update(bytes).digest('hex'),experiment,dt:record.dt,
    startedAt:new Date().toISOString(),passed:false,goalAchieved:false,rows,
    acceptanceMode:policy.acceptanceMode,shapeDifferenceLimitMm:policy.shapeDifferenceLimitMm,runtimeVersions:process.versions,
    crossRuntimeReferenceCalibrated:false,candidateQualified:false,
    scope:'Every recorded right-access physical target, from empty tools, compared to the browser shape at every step. CPU solver timing only; browser replay measures controls, rendering and both accesses. Node/browser arithmetic differs (017-runtime-audit-validation.json); without a same-runtime control, a shape failure cannot be attributed to the candidate alone.'};
try {
    for(const [index,reference] of record.steps.entries()) {
        for(const tool of tools){tool.insertion=reference.tools[tool.id];tool.rotation=reference.tools[tool.id+'Rotation'];}
        const start=performance.now(),result=system.step(world,record.dt),cpuMs=performance.now()-start;
        assert.ok(result.accepted,`Step ${index+1}: ${result.status}`);
        const d=system.diagnostics.last,pose=system.captureFlowSamplingState();
        const shapeDifferenceMm=crossoverShapeDifference(reference.shape,{coordinates:pose.coordinates,
            positions:pose.positions.flatMap(p=>p.map((x,k)=>x+pose.origin[k]))});
        rows.push({step:index+1,stage:reference.stage,moving:reference.moving,cpuMs,shapeDifferenceMm,
            nodes:system.diagnostics.mesh.nodes,rows:system.diagnostics.mesh.constraintRows,
            ...Object.fromEntries(['iterations','factorizations','backtracks','geometryRestarts','frictionIterations','substepAttempts','subdivisions',
                'predictorFallbacks','coupledFrictionFallbacks','wallNormalFallbacks','recenteredRoundoff','replayCaptureMs','replayCaptureTimes','timings',
                'certificateBound','residual','quality','attempts'].map(k=>[k,d[k]]))});
        assert.ok(d.converged&&d.quality.finite&&d.certificateBound<=1e-6&&d.residual.length<=1e-5&&
            d.quality.maxPenetration<=1e-5&&d.quality.bodies.every(b=>b.maxLengthError<=1e-5),`Physical gate at ${index+1}`);
        if(policy.shapeDifferenceLimitMm!==null)
            assert.ok(shapeDifferenceMm<=policy.shapeDifferenceLimitMm,`Cross-runtime shape ${shapeDifferenceMm} mm at ${index+1}; candidate attribution requires a same-runtime control`);
        flow.update(record.dt);
        if((index+1)%120===0)console.log(JSON.stringify({step:index+1,cpuMs,shapeDifferenceMm,nodes:system.diagnostics.mesh.nodes}));
    }
    report.passed=true;
}catch(error){report.error=error.stack;process.exitCode=1;console.error(error.message);}
finally {
    report.maximumShapeDifferenceMm=Math.max(0,...rows.map(r=>r.shapeDifferenceMm));
    report.finishedAt=new Date().toISOString();fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');
    system.reset();anatomy.dispose();
    console.log(JSON.stringify({steps:rows.length,passed:report.passed,maximumShapeDifferenceMm:report.maximumShapeDifferenceMm}));
}

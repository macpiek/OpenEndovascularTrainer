import test from 'node:test';
import assert from 'node:assert/strict';
import {createCrossoverScenarioRecording,crossoverShapeDifference,summarizeCrossoverRecording,validateCrossoverRecording} from '../src/benchmark/crossoverScenarioRecording.js';

const input=advance=>Object.fromEntries(['right','left'].map(side=>[side,{advance:side==='right'?advance:0,rotation:0,catheterAdvance:0,catheterRotation:0,
    wireType:'glidewire',catheterType:'pigtail',catheterTool:'pigtail',automatic:{guidewireAdvance:0,catheterAdvance:0}}]));
const shape={coordinates:[0,5,10],positions:[0,0,0,0,5,0,0,10,0]};
const diagnostics=[{last:{converged:true,certificateBound:1e-8,residual:{length:1e-10},quality:{finite:true,maxPenetration:0,bodies:[{maxLengthError:1e-10}]}}}];
const sample={dt:1/60,diagnostics:[...diagnostics,...diagnostics],shape,tools:{wire:10,catheter:0,wireRotation:0,catheterRotation:0}};

test('commands are frozen at preparation; retries do not consume or rerecord inputs',()=>{
    let time=0;const session=createCrossoverScenarioRecording({now:()=>time});session.startRecording({anatomy:'test'});
    const live=input(1),frozen=session.boundary(live,{anatomy:'test'});live.right.advance=-1;frozen.right.advance=0;
    for(let i=0;i<4;i++)session.slice(2,{pending:true});
    assert.equal(session.record.steps.length,0);time=20;session.commit(sample);
    assert.equal(session.record.steps.length,1);assert.equal(session.record.steps[0].inputs.right.advance,1);
    assert.equal(session.record.steps[0].cpuMs,8);assert.equal(session.record.steps[0].moving,true);
    session.finishRecording();assert.equal(session.record.complete,true);assert.equal(session.record.fullManeuverRecorded,false);
    const original=structuredClone(session.record);time=1000;session.startReplay({anatomy:'test'},original);
    assert.deepEqual(session.boundary(input(-1),{anatomy:'test'}),input(1));time+=50;session.commit(sample);
    assert.equal(session.mode,'finished');assert.equal(session.record.complete,true);assert.equal(session.record.summary.duringMotion.physicsHz,20);
    assert.equal(session.record.steps.length,original.steps.length);assert.equal(session.record.summary.maximumShapeDifferenceMm,0);
});

test('motion throughput excludes idle periods and keeps per-stage stalls',()=>{
    const steps=[{moving:false,intervalMs:1000,cpuMs:1,stage:0},{moving:true,intervalMs:20,cpuMs:12,stage:0},
        {moving:true,intervalMs:80,cpuMs:62,stage:1},{moving:false,intervalMs:1000,cpuMs:2,stage:1}];
    const summary=summarizeCrossoverRecording(steps);
    assert.equal(summary.duringMotion.physicsHz,20);assert.equal(summary.stationary.physicsHz,1);
    assert.equal(summary.stages[1].wallMs,1080);assert.equal(summary.duringMotion.maxCpuMs,62);
});

test('whole-polyline comparison catches deviations between reference vertices on a changed mesh',()=>{
    const altered={coordinates:[0,2.5,5,10],positions:[0,0,0,.101,2.5,0,0,5,0,0,10,0]};
    assert.equal(crossoverShapeDifference(shape,altered),.101);
    let time=0;const session=createCrossoverScenarioRecording({now:()=>time});session.startRecording({});session.boundary(input(1),{});time=20;session.commit(sample);session.finishRecording();
    const source=structuredClone(session.record);session.startReplay({},source,{shapeDifferenceLimitMm:.1});session.boundary(input(0),{});time=40;session.commit({...sample,shape:altered});
    assert.equal(session.record.complete,false);assert.match(session.record.error,/przekracza/);
    assert.equal(session.record.summary.maximumShapeDifferenceMm,.101);
});

test('failure and configuration changes cannot produce a complete scenario or consume failed inputs',()=>{
    const session=createCrossoverScenarioRecording();session.startRecording({flow:72});session.boundary(input(1),{flow:72});
    session.slice(50,{terminal:true,status:'iteration-limit'});assert.equal(session.record.steps.length,0);assert.equal(session.record.complete,false);
    session.startRecording({flow:72});session.boundary(input(1),{flow:80});assert.equal(session.mode,'finished');assert.equal(session.record.complete,false);
    assert.throws(()=>session.startReplay({flow:72},{...session.record,complete:false}),/niekompletne/);
});

test('all five stage boundaries are explicit and a stopped fragment is never labeled a complete maneuver',()=>{
    let time=0;const session=createCrossoverScenarioRecording({now:()=>time});session.startRecording({});
    for(let stage=0;stage<5;stage++){
        session.boundary(input(1),{});time+=20;session.commit(sample);session.checkpoint();
        if(stage<4)assert.throws(()=>session.checkpoint(),/dalszego przebiegu/);
    }
    assert.equal(session.record.complete,true);assert.equal(session.record.fullManeuverRecorded,true);assert.equal(session.record.fullManeuverVerified,false);
    assert.deepEqual(session.record.steps.map(s=>s.stage),[0,1,2,3,4]);
    assert.deepEqual(session.record.checkpoints.map(s=>s.afterStep),[1,2,3,4,5]);
});

function oneStepRecord() {
    let time=0;const session=createCrossoverScenarioRecording({now:()=>time});session.startRecording({});
    session.boundary(input(1),{});time=20;session.commit(sample);session.finishRecording();return session.record;
}

test('functional replay measures large shape changes without stopping or claiming a successful maneuver',()=>{
    const source=oneStepRecord(),session=createCrossoverScenarioRecording();
    const altered={...shape,positions:shape.positions.map((x,i)=>i===3?12:x)};
    session.startReplay({},source);session.boundary(input(0),{});session.commit({...sample,shape:altered});
    assert.equal(session.record.complete,true);
    assert.equal(session.record.summary.maximumShapeDifferenceMm,12);
    assert.equal(session.record.validation.shapeDifferenceLimitMm,null);
    assert.equal(session.record.validation.mode,'functional');
    assert.equal(session.record.inputSequenceVerified,true);
    assert.equal(session.record.fullManeuverVerified,false);
    assert.equal(session.record.goalAchieved,false);
    assert.equal(validateCrossoverRecording(source),source);
});

test('functional replay still rejects physical failure, nonfinite shape and wrong tool commands',()=>{
    const source=oneStepRecord();
    for(const damage of [s=>s.diagnostics[1].last.quality.maxPenetration=1,
        s=>s.diagnostics[0].last.converged=false,s=>s.shape.positions[3]=NaN,
        s=>s.tools.catheter+=1]) {
        const session=createCrossoverScenarioRecording(),invalid=structuredClone(sample);damage(invalid);
        session.startReplay({},source);session.boundary(input(0),{});session.commit(invalid);
        assert.equal(session.record.complete,false);assert.equal(session.record.steps.length,0);
    }
});

test('an optional historical shape limit must be finite and positive',()=>{
    for(const shapeDifferenceLimitMm of [0,-1,Infinity,NaN,'1'])
        assert.throws(()=>createCrossoverScenarioRecording().startReplay({},oneStepRecord(),{shapeDifferenceLimitMm}),/Limit/);
});

test('research force tolerance is explicit and cannot weaken geometry or promote a candidate to a strict reference',()=>{
    const source=oneStepRecord(),candidate=structuredClone(sample);
    candidate.diagnostics[0].last.certificateBound=8e-5;
    const strict=createCrossoverScenarioRecording();strict.startReplay({},source);strict.boundary(input(0),{});strict.commit(candidate);
    assert.equal(strict.record.complete,false);
    const loose=createCrossoverScenarioRecording();loose.startReplay({},source,{forceCertificateLimit:1e-4});loose.boundary(input(0),{});loose.commit(candidate);
    assert.equal(loose.record.complete,true);
    assert.equal(loose.record.validation.forceCertificateLimit,1e-4);
    assert.throws(()=>validateCrossoverRecording(loose.record));
    assert.equal(validateCrossoverRecording(loose.record,{forceCertificateLimit:1e-4}),loose.record);
    for(const damage of [q=>q.certificateBound=1.001e-4,q=>q.quality.maxPenetration=1.001e-5,
        q=>q.quality.bodies[0].maxLengthError=1e-5,q=>q.converged=false]){
        const invalid=structuredClone(candidate);damage(invalid.diagnostics[0].last);
        const session=createCrossoverScenarioRecording();session.startReplay({},source,{forceCertificateLimit:1e-4});session.boundary(input(0),{});session.commit(invalid);
        assert.equal(session.record.complete,false);assert.equal(session.record.steps.length,0);
    }
    for(const forceCertificateLimit of [0,-1,NaN,Infinity,1e-3])
        assert.throws(()=>createCrossoverScenarioRecording().startReplay({},source,{forceCertificateLimit}),/force/);
});

test('imports require ordered checkpoints, valid shapes, commands and stability evidence from both accesses',()=>{
    const record=oneStepRecord();assert.equal(validateCrossoverRecording(record),record);
    for(const corrupt of [r=>r.steps[0].quality=[],r=>r.steps[0].quality.pop(),
        r=>r.steps[0].quality[1].certificateBound=1e-3,r=>r.steps[0].shape.positions[1]=NaN,
        r=>r.steps[0].tools.catheter=NaN,r=>r.steps[0].inputs.right.advance=2,
        r=>r.steps[0].moving=false,r=>r.steps[0].intervalMs=-1,r=>r.steps[0].stage=4,
        r=>r.checkpoints=[{stage:0,afterStep:0}],r=>r.checkpoints=[{stage:1,afterStep:1}],
        r=>r.schemaVersion=1,r=>r.complete=false]) {
        const invalid=structuredClone(record);corrupt(invalid);assert.throws(()=>validateCrossoverRecording(invalid));
    }
});

test('recording fails closed if an access, shape or tool coordinate is missing',()=>{
    for(const invalid of [{...sample,diagnostics:[]},{...sample,diagnostics:sample.diagnostics.slice(0,1)},
        {...sample,shape:null},{...sample,tools:{...sample.tools,wireRotation:NaN}}]) {
        const session=createCrossoverScenarioRecording();session.startRecording({});session.boundary(input(1),{});session.commit(invalid);
        assert.equal(session.mode,'finished');assert.equal(session.record.complete,false);assert.equal(session.record.steps.length,0);
    }
});

test('matching the shared axis cannot conceal a different catheter insertion or material rotation',()=>{
    for(const tools of [{...sample.tools,catheter:1},{...sample.tools,catheterRotation:.01}]) {
        const source=oneStepRecord(),session=createCrossoverScenarioRecording();session.startReplay({},source);
        session.boundary(input(0),{});session.commit({...sample,tools});
        assert.equal(session.record.complete,false);assert.match(session.record.error,/Długość wsunięcia lub obrót/);
    }
});

test('live progress is constant work and does not scan or sort the accumulated history',()=>{
    const session=createCrossoverScenarioRecording({now:()=>20});session.startRecording({});session.boundary(input(1),{});session.commit(sample);
    session.record.steps=new Proxy(session.record.steps,{get(target,key){if(key==='length')return target.length;throw Error('History scanned');}});
    assert.equal(session.progress.stage,0);assert.equal(session.progress.motionHz,null);
});

test('a short slow movement is reported separately and uninterrupted 60-step windows expose stalls',()=>{
    const steps=Array.from({length:65},(_,i)=>({moving:true,intervalMs:i===32?100:1000/60,cpuMs:1,stage:0}));
    const summary=summarizeCrossoverRecording(steps);
    assert.ok(summary.minimumMovingWindowHz<56);assert.equal(summary.duringMotion.steps,65);
    assert.equal(summary.stages[0].duringMotion.intervalsOverBudget,1);
});

test('work profiling owns its measurements and does not confuse worker waiting with main-thread CPU',()=>{
    let time=0;const session=createCrossoverScenarioRecording({now:()=>time});
    const measured=structuredClone(sample);
    measured.diagnostics[0].mesh={nodes:201};
    Object.assign(measured.diagnostics[0].last,{cpuMs:19,workerResponseMs:21,iterations:5,
        timings:{linearMs:12},replayCaptureTimes:{snapshot:2}});
    session.startRecording({});session.boundary(input(1),{});session.slice(.8,{pending:true});
    time=25;session.commit(measured);session.finishRecording();
    measured.diagnostics[0].mesh.nodes=10;
    measured.diagnostics[0].last.timings.linearMs=0;
    measured.diagnostics[0].last.replayCaptureTimes.snapshot=0;
    const saved=session.record.steps[0];
    assert.equal(saved.cpuMs,.8);assert.equal(saved.providerMs[0],19);
    assert.equal(saved.solverWork[0].workerResponseMs,21);
    assert.equal(saved.solverWork[0].mesh.nodes,201);
    assert.equal(saved.solverWork[0].timings.linearMs,12);
    assert.equal(saved.solverWork[0].replayCaptureTimes.snapshot,2);
    assert.equal(session.record.summary.duringMotion.physicsHz,40);
});

import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {createSynchronizedStep} from '../src/physics/synchronizedStep.js';
import {createFixedStepTransaction} from '../src/physics/fixedStepTransaction.js';
import {HybridContrastSystem} from '../src/contrast/hybridContrastSystem.js';

const dt=1/60;
function member(outcomes) {
    let frameBlocked=false;
    return {steps:0,attempts:0,canAttempt:()=>!frameBlocked,
        beginFrame(){frameBlocked=false;},
        attempt(){this.attempts++;const outcome=outcomes.shift()??'accept';
            if(outcome==='reject')frameBlocked=true;
            if(outcome==='accept')this.steps++;
            return {accepted:outcome==='accept',pending:outcome==='yield',context:{steps:this.steps}};
        }};
}
function contrast() {
    const start=new THREE.Vector3(0,0,0),end=new THREE.Vector3(0,100,0);
    return new HybridContrastSystem({centerlineSegments:[{id:0,nodeStartId:0,nodeEndId:1,start,end,radiusStart:5,radiusEnd:5}],
        sheath:{start,end:new THREE.Vector3(0,20,0),radius:1}});
}

test('asymmetric solvers publish one shared time and contrast dose only after both accept',()=>{
    const right=member(['accept','accept']),left=member(['yield','reject','yield','accept','accept']);
    const system=contrast();
    assert.equal(system.startInjection({source:'sheath',volumeMl:1,rateMlPerSec:1}).ok,true);
    let time=0,inputs=0,published=[0,0];
    const step=createSynchronizedStep({members:[right,left],begin:()=>inputs++,publish:(delta,results)=>{
        published=results.map(result=>result.context.steps);system.advanceSynchronized(delta);time+=delta;
    }});
    step.beginFrame();
    for(let i=0;i<3;i++)assert.equal(step.attempt(dt).accepted,false);
    assert.equal(step.canAttempt(),false);assert.equal(step.pending,true);
    assert.equal(right.attempts,1);assert.deepEqual(published,[0,0]);
    assert.equal(system.totalDeliveredVolumeMl,0);assert.equal(time,0);
    step.beginFrame();assert.equal(step.attempt(dt).accepted,false);
    assert.equal(step.attempt(dt).accepted,true);
    assert.deepEqual(published,[1,1]);assert.equal(inputs,1);assert.equal(right.attempts,1);
    assert.equal(time,dt);assert.equal(system.simulationTimeSeconds,time);
    assert.ok(system.totalDeliveredVolumeMl>0);
    const dose=system.totalDeliveredVolumeMl;
    assert.equal(step.attempt(dt).accepted,false);
    assert.equal(system.totalDeliveredVolumeMl,dose);
    assert.equal(step.attempt(dt).accepted,true);
    assert.deepEqual(published,[2,2]);assert.equal(inputs,2);
    assert.equal(system.simulationTimeSeconds,2*dt);
    assert.ok(Math.abs(system.totalDeliveredVolumeMl-2*dose)<1e-10);
    assert.equal(system._solverAccumulator,0);
});

test('a terminally rejected member can recover without advancing its accepted peer twice',()=>{
    let accept=false,command=1,publications=0;
    const world={fixedDt:dt,accumulator:0,stepCount:0,
        advance(elapsed,prepare){this.accumulator+=elapsed;prepare();
            this.lastStepResult={accepted:accept,terminal:!accept,status:'rejected'};
            if(accept){this.accumulator-=dt;this.stepCount++;return 1;}return 0;},
        abandonFailedWholeStep(){this.accumulator-=dt;}};
    const owner=createFixedStepTransaction({world,prepare:()=>({}),
        recovery:{readKey:()=>command,rollback:()=>{}}});
    const fast=member([]);
    const step=createSynchronizedStep({members:[fast,owner],publish:()=>publications++});
    step.beginFrame();step.attempt(dt);step.attempt(dt);
    for(let i=0;i<30;i++){step.beginFrame();step.attempt(dt);}
    assert.equal(fast.steps,1);assert.equal(publications,0);assert.equal(world.stepCount,0);
    accept=true;command=0;step.beginFrame();assert.equal(step.attempt(dt).accepted,true);
    assert.equal(fast.steps,1);assert.equal(publications,1);assert.equal(world.stepCount,1);
});

test('dt changes and publication failures stop the barrier without replaying accepted solvers',()=>{
    const a=member([]),b=member([]);
    const step=createSynchronizedStep({members:[a,b],publish:()=>{throw new Error('consumer failed');}});
    step.attempt(dt);assert.throws(()=>step.attempt(dt),/consumer failed/);
    assert.equal(step.canAttempt(),false);step.attempt(dt);assert.equal(a.steps,1);assert.equal(b.steps,1);
    const other=createSynchronizedStep({members:[member([]),member([])],publish:()=>{}});
    other.attempt(dt);assert.throws(()=>other.attempt(dt*2),/retain/);
});

test('synchronized contrast reaches every 60 Hz boundary without dropped transport time',()=>{
    const system=contrast();
    for(let i=0;i<600;i++)system.advanceSynchronized(dt);
    assert.ok(Math.abs(system.simulationTimeSeconds-10)<1e-10);
    assert.equal(system._solverAccumulator,0);
    assert.throws(()=>system.advanceSynchronized(NaN),/Invalid/);
});

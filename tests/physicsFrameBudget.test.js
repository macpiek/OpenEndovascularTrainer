import test from 'node:test';
import assert from 'node:assert/strict';
import {runPhysicsFrameBudget,runPhysicsIdleBudget} from '../src/physics/physicsFrameBudget.js';
import {createFixedStepTransaction} from '../src/physics/fixedStepTransaction.js';
import {createSynchronizedStep} from '../src/physics/synchronizedStep.js';

test('pending slices finish one prepared timestep within spare frame budget',()=>{
    let prepared=0,solves=0,commits=0,time=0,debt=1;
    const world={fixedDt:1/60,accumulator:0,stepCount:0,admitted:[],advance(dt,prepare) {
        this.admitted.push(dt);this.accumulator+=dt;if(dt>0)prepare();
        const accepted=++solves===3;this.lastStepResult={accepted,pending:!accepted};
        if(accepted){this.stepCount++;this.accumulator-=this.fixedDt;commits++;debt--;}
        return accepted?1:0;
    }};
    const transaction=createFixedStepTransaction({world,prepare:()=>{prepared++;return {};}});
    transaction.beginFrame();
    const r=runPhysicsFrameBudget({hasDebt:()=>debt>0,canAttempt:()=>transaction.canAttempt(),canFit:()=>time+4+3.5<16.667,
        attempt:()=>{time+=4;return transaction.attempt(1/60);},resumePending:true});
    assert.deepEqual(world.admitted,[1/60,0,0]);assert.equal(prepared,1);assert.equal(commits,1);assert.deepEqual(r,{steps:1,slices:3});
});

test('render reserve, numerical rejection and slice cap bound unfinished work',()=>{
    for(const [pending,limit,expected] of [[true,6,3],[true,2,2],[false,6,1]]) {
        let time=0;
        const r=runPhysicsFrameBudget({hasDebt:()=>true,canAttempt:()=>true,canFit:()=>time+4+3.5<16.667,
            attempt:()=>{time+=4;return {accepted:false,pending};},maxSlices:limit,resumePending:true});
        assert.equal(r.slices,expected);assert.equal(r.steps,0);
    }
});

test('reference scheduling stops on pending and accepted-step cap cannot be exceeded',()=>{
    const base={hasDebt:()=>true,canAttempt:()=>true,canFit:()=>true};
    assert.deepEqual(runPhysicsFrameBudget({...base,attempt:()=>({accepted:false,pending:true})}),{steps:0,slices:1});
    assert.deepEqual(runPhysicsFrameBudget({...base,resumePending:true,attempt:()=>({accepted:true}),maxSteps:2}),{steps:2,slices:2});
    assert.deepEqual(runPhysicsFrameBudget({...base,canAttempt:()=>false,attempt:()=>{throw Error('blocked');}}),{steps:0,slices:0});
});

test('cooperative frame resumption preserves the common clock and never repeats an already accepted access',()=>{
    let elapsed=0,debt=1,prepared=0,published=0,fastCalls=0,slowCalls=0;
    const barrier=createSynchronizedStep({begin:()=>prepared++,publish:dt=>{assert.equal(dt,1/60);published++;debt--;},members:[
        {canAttempt:()=>true,beginFrame(){},attempt(dt,budget){fastCalls++;elapsed+=1;assert.ok(budget<=4);return {accepted:true,pending:false};}},
        {canAttempt:()=>true,beginFrame(){},attempt(dt,budget){slowCalls++;elapsed+=budget;return {accepted:slowCalls===3,pending:slowCalls<3};}}
    ]});
    const room=()=>1000/60-elapsed-3.5-.75;
    barrier.beginFrame();
    const result=runPhysicsFrameBudget({hasDebt:()=>debt>0,canAttempt:()=>barrier.canAttempt(),canFit:()=>room()>.5,
        attempt:()=>({accepted:barrier.attempt(1/60,Math.min(4,room())).accepted,pending:barrier.canContinueFrame(true)}),
        resumePending:true,maxSteps:2,maxSlices:6});
    assert.equal(prepared,1);assert.equal(published,1);assert.equal(fastCalls,1);assert.equal(slowCalls,3);
    assert.deepEqual(result,{steps:1,slices:4});assert.ok(elapsed<=1000/60-3.5-.75);
});

test('idle catch-up preserves the prepared shared step, obeys the render deadline and consumes only committed time',()=>{
    let time=10,debt=1,prepared=0,published=0,fastCalls=0,slowCalls=0;
    const budgets=[];
    const barrier=createSynchronizedStep({begin:()=>prepared++,publish:dt=>{assert.equal(dt,1/60);published++;debt--;},members:[
        {canAttempt:()=>true,beginFrame(){},attempt(){fastCalls++;return {accepted:true,pending:false};}},
        {canAttempt:()=>true,beginFrame(){},attempt(dt,budget){slowCalls++;budgets.push(budget);time+=2;return {accepted:slowCalls===3,pending:slowCalls<3};}}
    ]});
    barrier.beginFrame();barrier.attempt(1/60,2);
    assert.equal(published,0);assert.equal(debt,1);
    const result=runPhysicsIdleBudget({deadline:20,now:()=>time,guardMs:1,maxSliceMs:4,
        hasDebt:()=>debt>0,canAttempt:()=>barrier.canAttempt(),canFit:room=>room>=.5,
        attempt:budget=>barrier.attempt(1/60,budget).accepted});
    assert.equal(prepared,1);assert.equal(published,1);assert.equal(fastCalls,1);assert.equal(slowCalls,3);
    assert.deepEqual(budgets,[4,4,4]);assert.deepEqual(result,{steps:1,slices:3});
});

test('idle work cannot start without room, continue after rejection, or monopolize a frame',()=>{
    const base={deadline:10,now:()=>0,guardMs:1,maxSliceMs:4,hasDebt:()=>true,canAttempt:()=>true,canFit:room=>room>=.5};
    assert.deepEqual(runPhysicsIdleBudget({...base,now:()=>8.6,attempt:budget=>{assert.ok(budget<=.4+1e-12);throw Error('No slice fits');}}),{steps:0,slices:0});
    let blocked=false,calls=0;
    assert.deepEqual(runPhysicsIdleBudget({...base,canAttempt:()=>!blocked,attempt:()=>{calls++;blocked=true;return false;}}),{steps:0,slices:1});
    assert.equal(calls,1);
    assert.deepEqual(runPhysicsIdleBudget({...base,maxSlices:3,attempt:()=>true}),{steps:3,slices:3});
    let time=5;
    const budgets=[];
    const result=runPhysicsIdleBudget({...base,now:()=>time,attempt:budget=>{budgets.push(budget);time+=3;return false;}});
    assert.deepEqual(budgets,[4,1]);assert.deepEqual(result,{steps:0,slices:2});
});

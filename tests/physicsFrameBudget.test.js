import test from 'node:test';
import assert from 'node:assert/strict';
import {runPhysicsFrameBudget} from '../src/physics/physicsFrameBudget.js';
import {createFixedStepTransaction} from '../src/physics/fixedStepTransaction.js';

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

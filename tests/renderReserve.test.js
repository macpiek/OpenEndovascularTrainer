import test from 'node:test';
import assert from 'node:assert/strict';
import {estimateRenderReserve} from '../src/physics/renderReserve.js';
import {runPhysicsFrameBudget,runPhysicsIdleBudget} from '../src/physics/physicsFrameBudget.js';
import {createFixedStepTransaction} from '../src/physics/fixedStepTransaction.js';

const frameMs=1000/60,guardMs=.75,maxSliceMs=4;

test('presentation estimate follows normal cost and releases a first-upload spike promptly',()=>{
    let reserve=3.5;
    for(const measured of [3.5,7,7,9,9]) {
        reserve=estimateRenderReserve(reserve,measured);
        assert.ok(reserve>=measured,'Reserve the observed sustainable presentation work');
    }
    reserve=estimateRenderReserve(reserve,500);
    assert.ok(frameMs-reserve-guardMs>=maxSliceMs-1e-12,'A compile cannot reduce the next normal slice');
    for(let i=0;i<8;i++)reserve=estimateRenderReserve(reserve,3.5);
    assert.equal(reserve,3.5,'An isolated spike must not suppress work for hundreds of frames');
    for(let i=0;i<30;i++) {
        reserve=estimateRenderReserve(reserve,40);
        assert.ok(reserve>=9&&frameMs-reserve-guardMs>=maxSliceMs-1e-12,
            'Persistent overload reserves substantial render time while physics keeps progressing');
    }
});

test('compile recovery preserves fixed timesteps and drains the real backlog through frame and idle slices',()=>{
    // A 500ms first draw has really elapsed: keep all thirty owed timesteps.
    let debt=30,clock=0,budget=0,remaining=0,prepared=0,reserve=estimateRenderReserve(3.5,500);
    const admitted=[],budgets=[];
    const world={fixedDt:1/60,accumulator:0,stepCount:0,advance(dt,prepare){
        admitted.push(dt);this.accumulator+=dt;
        if(dt>0){prepare();remaining=5;}
        const work=Math.min(budget,remaining);remaining-=work;clock+=work;
        const accepted=remaining<=1e-10;
        this.lastStepResult={accepted,pending:!accepted};
        if(accepted){this.stepCount++;this.accumulator-=this.fixedDt;debt--;}
        return accepted?1:0;
    }};
    const transaction=createFixedStepTransaction({world,prepare:()=>{prepared++;return {};},now:()=>clock});
    const attempt=available=>{
        budget=available;budgets.push(budget);
        return transaction.attempt(world.fixedDt);
    };
    for(let frame=0;frame<60;frame++) {
        const start=frame*frameMs;clock=start;debt++;transaction.beginFrame();
        const room=()=>frameMs-(clock-start)-reserve-guardMs;
        runPhysicsFrameBudget({hasDebt:()=>debt>0,canAttempt:()=>transaction.canAttempt(),canFit:()=>room()>.5,
            attempt:()=>attempt(Math.max(.5,Math.min(maxSliceMs,room()))),resumePending:true,maxSteps:2,maxSlices:6});
        const presentationStart=clock;clock+=3.5;
        reserve=estimateRenderReserve(reserve,clock-presentationStart);
        runPhysicsIdleBudget({deadline:start+frameMs,now:()=>clock,guardMs,maxSliceMs,
            hasDebt:()=>debt>0,canAttempt:()=>transaction.canAttempt(),canFit:room=>room>=.5,
            attempt:available=>attempt(available).accepted,maxSlices:6});
        assert.ok(clock<=start+frameMs+1e-9,'Catch-up stays within the actual idle deadline');
    }
    assert.equal(world.stepCount,90,'All elapsed time is solved, including the compile backlog');
    assert.equal(debt,0);assert.equal(world.accumulator,0);assert.equal(prepared,90);
    assert.equal(admitted.filter(dt=>dt>0).length,90,'Pending slices cannot readmit a timestep');
    assert.ok(admitted.every(dt=>dt===0||dt===1/60),'Never enlarge or skip the fixed physical timestep');
    assert.ok(budgets.every(ms=>ms>0&&ms<=maxSliceMs),'Keep the existing cooperative slice cap');
});

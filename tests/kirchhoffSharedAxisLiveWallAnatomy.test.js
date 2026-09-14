import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';

test('actual pigtail withdrawal recovers live-load cycling with a certified atomic frozen fallback',async()=>{
    const fixture=JSON.parse(readFileSync(new URL('./fixtures/shared-axis/anatomy-pigtail-withdraw-184.73-live-wall-incoming.json',import.meta.url)));
    const anatomy=await loadCoupledRuntimeAnatomy();
    try {
        const req=fixture.stepRequest,s=restoreSharedAxisReplay(fixture,anatomy.field),incoming=captureSharedAxisReplay(s,fixture.sheath);
        assert.equal(s.materials.find(m=>m.spec.id==='catheter').spec.mass,1.75);
        let frozenAttempts=0,linearLU=0;const frozenStates=new WeakSet();
        const iterator=advanceSharedAxis(s,req.rotations,req.dt,req.tools,{...req.options,
            observeIteration:({state})=>{
                if(state.wallFrictionStep&&!state.wallFrictionStep.liveNormalLoad&&!frozenStates.has(state)) {
                    frozenStates.add(state);frozenAttempts++;
                }
            },
            observeTrial:e=>{if(e.kind==='direction')linearLU+=e.direction.factorizations;}});
        let next;do{next=iterator.next();}while(!next.done);
        const {state:accepted,result}=next.value;
        assert.ok(accepted,JSON.stringify(result));assert.equal(result.converged,true);
        assert.equal(frozenAttempts,1);assert.equal(result.wallNormalFallback.converged,true);
        assert.equal(result.wallNormalFallback.liveFailure,'linear-solve');assert.equal(result.subdivisions,1);
        assert.ok(result.factorizations>=linearLU,'Failed live directions must remain in the reported cost');
        assert.ok(result.certificateBound<=1e-6);assert.ok(result.residual.length<=1e-5);
        assert.ok(result.quality.maxPenetration<=1e-5);assert.equal(result.interToolRows,0);
        assert.equal(accepted.materials.find(m=>m.spec.id==='catheter').spec.insertion,req.tools.find(m=>m.id==='catheter').insertion);
        assert.equal(accepted.wallFrictionStep,null);assert.equal(accepted.dynamicStep,null);
        assert.ok(accepted.wallFrictionHistory.length>0);
        assert.ok(accepted.wallFrictionHistory.every(r=>r.elastic.every(Number.isFinite)&&r.owner==='catheter'));
        assert.deepEqual(captureSharedAxisReplay(s,fixture.sheath),incoming,'No failed trial may change the published input or its friction history');
    } finally {anatomy.dispose();}
});

import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {feedSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {stepSharedAxis} from '../src/physics/kirchhoffSharedAxisTimeStep.js';

test('cyclic catheter feed switches strategy early with identical certified physical output',async()=>{
    const fixture=JSON.parse(readFileSync(new URL('./fixtures/shared-axis/anatomy-berenstein-feed-138.67-live-cycle.json',import.meta.url)));
    const anatomy=await loadCoupledRuntimeAnatomy(),req=fixture.stepRequest;
    try {
        const outputs=[];
        for(const earlyLiveFallback of [false,true]) {
            const input=restoreSharedAxisReplay(fixture,anatomy.field),incoming=captureSharedAxisReplay(input,fixture.sheath);
            const iterator=advanceSharedAxis(input,req.rotations,req.dt,req.tools,{...req.options,earlyLiveFallback});
            let next;do{next=iterator.next();}while(!next.done);
            const {state,result}=next.value;
            assert.ok(state,JSON.stringify(result));assert.equal(result.converged,true);
            assert.equal(result.subdivisions,1);assert.equal(result.interToolRows,0);
            assert.ok(result.certificateBound<=req.options.forceTolerance);
            assert.ok(result.residual.length<=req.options.lengthTolerance);
            assert.ok(result.quality.maxPenetration<=req.options.lengthTolerance);
            assert.deepEqual(captureSharedAxisReplay(input,fixture.sheath),incoming,'Trials must leave published input and friction history unchanged');
            outputs.push({result,terminal:captureSharedAxisReplay(state,fixture.sheath)});
        }
        const [reference,guard]=outputs;
        assert.equal(reference.result.wallNormalFallback.liveFailure,'iteration-limit');
        assert.equal(guard.result.wallNormalFallback.liveFailure,'live-contact-cycle');
        assert.equal(guard.result.wallNormalFallback.detectedCycle.period,2);
        assert.ok(guard.result.iterations<reference.result.iterations/2);
        assert.ok(guard.result.factorizations<reference.result.factorizations/2);
        assert.deepEqual(guard.terminal,reference.terminal,'Pose, reactions, velocities and wall friction history must agree exactly');
        assert.deepEqual(guard.result.residual,reference.result.residual);
        assert.deepEqual(guard.result.quality,reference.result.quality);

        const feeds=Object.fromEntries(req.tools.map(t=>[t.id,t.insertion]));
        const feedById=Object.fromEntries(fixture.tools.map(t=>[t.id,feeds[t.id]-t.insertion]));
        const noFallback=feedSharedAxisNative(restoreSharedAxisReplay(fixture,anatomy.field),feeds);
        const disabled=stepSharedAxis(noFallback,req.dt,{...req.options,feedById,
            maxIterations:20,wallNormalFallback:false,earlyLiveFallback:true});
        assert.equal(disabled.converged,false);
        assert.equal(disabled.status,'iteration-limit','Do not terminate a live attempt early if no frozen retry is allowed');
        assert.equal(disabled.detectedCycle,undefined);
        assert.equal(disabled.wallNormalFallback,undefined);

        // Cancellation during a live iteration may neither publish a trial
        // nor start the frozen retry.
        const input=restoreSharedAxisReplay(fixture,anatomy.field),incoming=captureSharedAxisReplay(input,fixture.sheath);
        let liveIterations=0,frozenIterations=0;
        const iterator=advanceSharedAxis(input,req.rotations,req.dt,req.tools,{...req.options,
            observeIteration:({state})=>{if(state.wallFrictionStep?.liveNormalLoad)liveIterations++;else frozenIterations++;}});
        while(liveIterations<10){assert.equal(iterator.next().done,false);}
        iterator.return();
        assert.equal(frozenIterations,0);
        assert.deepEqual(captureSharedAxisReplay(input,fixture.sheath),incoming);
    }finally{anatomy.dispose();}
});

import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';

test('long crossover escapes opposing near-parallel reactions without relaxing accuracy or publishing a rejected trial',async()=>{
    const f=JSON.parse(gunzipSync(readFileSync(new URL('../reports/performance-scenarios/001-long-hold-replay.json.gz',import.meta.url))));
    const anatomy=await loadCoupledRuntimeAnatomy(undefined,f.anatomy),request=f.stepRequest;
    const drain=it=>{let next;do{next=it.next();}while(!next.done);return next.value;};
    try {
        const input=restoreSharedAxisReplay(f,anatomy.field),before=captureSharedAxisReplay(input,f.sheath);
        // Compare factor counts under the same observer policy. Recovery is
        // instrumented below, and observer callbacks deliberately retain all
        // original retries instead of reusing work across fallback searches.
        const failed=drain(advanceSharedAxis(input,request.rotations,request.dt,request.tools,
            {...request.options,recoverNegativeContact:false,observeTrial:()=>{}}));
        assert.equal(failed.state,undefined);assert.equal(failed.result.status,'linear-solve');
        assert.deepEqual(captureSharedAxisReplay(input,f.sheath),before);
        let recoveries=0;
        const options={...request.options,observeTrial:e=>{if(e.kind==='direction')recoveries+=e.direction.negativeReactionRecoveries??0;}};
        const {state,result}=drain(advanceSharedAxis(input,request.rotations,request.dt,request.tools,options));
        assert.ok(state&&result.converged);assert.ok(recoveries>0);
        assert.equal(result.subdivisions,1);assert.ok(result.factorizations<failed.result.factorizations+50,'Recovery adds one bounded solve after the original rejection');
        assert.deepEqual(result.negativeContactRecovery,{attempted:true,originalFailure:'linear-solve',converged:true});
        assert.ok(result.certificateBound<=1e-6&&result.residual.length<=1e-5);
        assert.ok(result.quality.finite&&result.quality.maxPenetration<=1e-5&&result.quality.bodies.every(b=>b.maxLengthError<1e-5));
        for(let i=0;i<state.positions.length;i++)assert.ok(Math.hypot(...state.positions[i].map((v,k)=>
            v+state.origin[k]-input.positions[i][k]-input.origin[k]))<1,'Never apply the enormous uncertified direction');
        assert.deepEqual(captureSharedAxisReplay(input,f.sheath),before);
        recoveries=0;
        const cancelled=advanceSharedAxis(input,request.rotations,request.dt,request.tools,options);
        for(let i=0;i<1000&&!recoveries;i++)assert.equal(cancelled.next().done,false);
        assert.ok(recoveries>0);cancelled.return();assert.deepEqual(captureSharedAxisReplay(input,f.sheath),before);
    } finally {anatomy.dispose();}
});

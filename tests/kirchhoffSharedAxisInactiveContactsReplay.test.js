import test, {after} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';

const anatomy=await loadCoupledRuntimeAnatomy();after(()=>anatomy.dispose());
const options={promoteTrialAssembly:true,projectionMode:'reduced',wasmMaterial:true,reuseConstraintWork:true,reuseMatrixAssembly:true,reuseRowBuffers:true};
for(const name of ['anatomy-berenstein-feed-138.67-live-cycle','anatomy-pigtail-wire-withdraw-200.27-incoming'])
test(`inactive-contact bounds preserve complete physical output and cancellation: ${name}`,()=>{
    const fixture=JSON.parse(readFileSync(new URL(`./fixtures/shared-axis/${name}.json`,import.meta.url))),req=fixture.stepRequest,outputs=[];
    for(const cullInactiveContacts of [false,true]) {
        const input=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisReplay(input,fixture.sheath);
        const run=()=>advanceSharedAxis(input,req.rotations,req.dt,req.tools,{...req.options,...options,cullInactiveContacts});
        if(cullInactiveContacts) {
            const cancelled=run();for(let i=0;i<12;i++)if(cancelled.next().done)break;
            cancelled.return();assert.deepEqual(captureSharedAxisReplay(input,fixture.sheath),before);
        }
        const iterator=run();let next;do{next=iterator.next();}while(!next.done);
        const {state,result}=next.value;assert.ok(state,JSON.stringify(result));
        assert.deepEqual(captureSharedAxisReplay(input,fixture.sheath),before);
        assert.ok(result.certificateBound<=req.options.forceTolerance);
        outputs.push({state:captureSharedAxisReplay(state,fixture.sheath),result});
    }
    assert.deepEqual(outputs[1].state,outputs[0].state,'Pose, reactions, exact accepted gaps, frames, velocities and friction history');
    for(const key of ['quality','residual','iterations','factorizations','backtracks','geometryRestarts','substepAttempts','fullAssemblies','residualAssemblies'])
        assert.deepEqual(outputs[1].result[key],outputs[0].result[key],key);
});

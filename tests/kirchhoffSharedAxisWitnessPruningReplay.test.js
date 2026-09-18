import test,{after} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis,sampleSharedAxisPosition} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
const anatomy=await loadCoupledRuntimeAnatomy();after(()=>anatomy.dispose());
const finish=it=>{let n;do{n=it.next();}while(!n.done);return n.value;};
for(const name of ['anatomy-berenstein-feed-138.67-live-cycle','anatomy-pigtail-wire-withdraw-200.27-incoming'])
test(`pruned witness solve retains certificates, local shape and cancellation: ${name}`,()=>{
    const fixture=JSON.parse(readFileSync(new URL(`./fixtures/shared-axis/${name}.json`,import.meta.url))),req=fixture.stepRequest;
    const input=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisReplay(input,fixture.sheath);
    const options={...req.options,promoteTrialAssembly:true,projectionMode:'reduced',wasmMaterial:true,reuseConstraintWork:true,
        reuseMatrixAssembly:true,reuseRowBuffers:true,reuseMaterialScratch:true,lightweightFriction:true,reuseTriangleKernel:true,earlyContactPreflight:true};
    const cancelled=advanceSharedAxis(input,req.rotations,req.dt,req.tools,{...options,pruneInactiveWitnesses:true});
    for(let i=0;i<12;i++)if(cancelled.next().done)break;
    cancelled.return();assert.deepEqual(captureSharedAxisReplay(input,fixture.sheath),before);
    const outputs=[false,true].map(pruneInactiveWitnesses=>finish(advanceSharedAxis(input,req.rotations,req.dt,req.tools,{...options,pruneInactiveWitnesses})));
    for(const {state,result} of outputs) {
        assert.ok(state,JSON.stringify(result));assert.ok(result.quality.finite);
        assert.ok(result.certificateBound<=options.forceTolerance);
        assert.ok(result.residual.length<=options.lengthTolerance);
        assert.ok(result.quality.maxPenetration<=options.lengthTolerance);
    }
    const [a,b]=outputs.map(o=>o.state);
    for(const x of new Set([...a.coordinates,...b.coordinates])) {
        const p=sampleSharedAxisPosition(a,x),q=sampleSharedAxisPosition(b,x);
        assert.ok(Math.hypot(...p.map((v,k)=>v-q[k]))<.01,'Local shape changed by more than 0.01 mm');
    }
    assert.deepEqual(captureSharedAxisReplay(input,fixture.sheath),before);
});

import test from 'node:test';import assert from 'node:assert/strict';import {readFileSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis,sampleSharedAxisPosition} from '../src/physics/kirchhoffSharedAxisAppSystem.js';

test('early friction plus zero-reaction release reduces repeated solves on captured vessel cases',async()=>{
    const anatomy=await loadCoupledRuntimeAnatomy();
    try {for(const name of ['anatomy-berenstein-feed-138.67-live-cycle','anatomy-pigtail-wire-withdraw-200.27-incoming']) {
        const fixture=JSON.parse(readFileSync(new URL(`./fixtures/shared-axis/${name}.json`,import.meta.url))),req=fixture.stepRequest,out=[];
        for(const coupledFrictionNewton of [false,true]) {
            const s=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisReplay(s,fixture.sheath);
            const iterator=advanceSharedAxis(s,req.rotations,req.dt,req.tools,{...req.options,coupledFrictionNewton,liveWallNormalLoad:true,
                promoteTrialAssembly:true,projectionMode:'reduced',stagnationFallback:true,wasmMaterial:true,reuseMaterialScratch:true,lightweightFriction:true,
                reuseTriangleKernel:true,earlyContactPreflight:true,reuseConstraintWork:true,reuseMatrixAssembly:true,reuseRowBuffers:true});
            let next;do{next=iterator.next();}while(!next.done);out.push(next.value);
            assert.deepEqual(captureSharedAxisReplay(s,fixture.sheath),before,'published input must not change');
            assert.ok(next.value.state,JSON.stringify(next.value.result));
            assert.ok(next.value.result.certificateBound<=(req.options.forceTolerance??1e-6));
            assert.ok(next.value.result.residual.length<=(req.options.lengthTolerance??1e-5));
            assert.equal(next.value.state.wallFrictionStep,null);assert.equal(next.value.state.dynamicStep,null);
        }
        const a=out[0],b=out[1];
        assert.ok(b.result.iterations<a.result.iterations);assert.ok(b.result.factorizations<a.result.factorizations);
        assert.ok(b.result.fullAssemblies+b.result.residualAssemblies<a.result.fullAssemblies+a.result.residualAssemblies);
        const knots=new Set([...a.state.coordinates,...b.state.coordinates]);
        for(const x of knots){const p=sampleSharedAxisPosition(a.state,x),q=sampleSharedAxisPosition(b.state,x);assert.ok(Math.hypot(...p.map((v,k)=>v-q[k]))<.001);}
    }}finally{anatomy.dispose();}
});

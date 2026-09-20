import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {captureSharedAxisNative,createSharedAxisNative,relaxSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {defineKirchhoffMaterialProfile} from '../src/physics/kirchhoffMaterialProfile.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';

test('stagnation-only globalization preserves ordinary bending and unloading exactly',()=>{
    const beam=defineKirchhoffMaterialProfile({id:'stagnation-beam',sampleEI1:()=>1e6,sampleGJ:()=>1e6/1.3,sampleKappa01:()=>0});
    const states=[false,true].map(stagnationResidualSearch=>{
        const s=createSharedAxisNative({tools:[{id:'wire',insertion:100,type:beam}]});
        const poses=[];
        for(const load of [1,0]) {
            s.loads[s.layout.positions.at(-1)+1]=load;
            const result=relaxSharedAxisNative(s,{stagnationResidualSearch});
            assert.ok(result.converged,JSON.stringify(result));
            assert.equal(result.residualSearchActivation,undefined);
            if(load)assert.ok(Math.abs(s.positions.at(-1)[1]/(100**3/3e6)-1)<.01);
            else assert.ok(Math.abs(s.positions.at(-1)[1])<1e-5);
            const {positions,frames,multipliers}=captureSharedAxisNative(s);
            poses.push({positions,frames,multipliers});
        }
        return poses;
    });
    assert.deepEqual(states[1],states[0]);
});

test('stalled contact solve reduces repeated work while retaining equilibrium bounds and private input',async()=>{
    const anatomy=await loadCoupledRuntimeAnatomy();
    try {
        const fixture=JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/shared-axis/closed-root-wire-745.80-stagnation-residual.json.gz',import.meta.url))));
        const input=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisNative(input),q=fixture.stepRequest;
        let activations=0,rootTrials=0;
        const iterator=advanceSharedAxis(input,q.rotations,q.dt,q.tools,{...q.options,stagnationResidualSearch:true,observeTrial:e=>{
            if(e.kind==='residual-search-activation') {activations++;assert.ok(e.iteration>=15);}
            if(e.kind==='trial'&&e.accept&&e.rootSearch) {
                rootTrials++;
                const merit=m=>Math.max(m.force/q.options.forceTolerance,m.torque/q.options.forceTolerance,m.constraint/q.options.lengthTolerance);
                assert.ok(merit(e.candidate)<=1||merit(e.candidate)<merit(e.base));
            }
        }});
        let next;do{next=iterator.next();}while(!next.done);
        const {state,result}=next.value;
        assert.ok(state,JSON.stringify(result));
        assert.ok(activations>0&&rootTrials>0);
        assert.equal(result.subdivisions,1);
        assert.ok(result.iterations<150,`Reference needed 586, observed ${result.iterations}`);
        assert.ok(result.factorizations<1500,`Reference needed 5576, observed ${result.factorizations}`);
        assert.ok(result.certificateBound<=q.options.forceTolerance);
        assert.ok(result.residual.length<=q.options.lengthTolerance);
        assert.ok(result.quality.finite&&result.quality.maxPenetration<.02);
        assert.deepEqual(captureSharedAxisNative(input),before);
    }finally{anatomy.dispose();}
});

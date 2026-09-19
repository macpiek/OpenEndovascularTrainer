import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis,sampleSharedAxisPosition} from '../src/physics/kirchhoffSharedAxisAppSystem.js';

test('warm contact search reduces repeated factors on captured Pigtail steps with unchanged certification',async()=>{
    const anatomy=await loadCoupledRuntimeAnatomy();
    try{for(const name of ['anatomy-pigtail-contact-burst-incoming','anatomy-pigtail-friction-settling-incoming']){
        const fixture=JSON.parse(readFileSync(new URL(`./fixtures/shared-axis/${name}.json`,import.meta.url))),req=fixture.stepRequest,out=[];
        for(const fast of [false,true]){
            const s=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisReplay(s,fixture.sheath);
            const it=advanceSharedAxis(s,req.rotations,req.dt,req.tools,{...req.options,zeroDualStart:fast,velocityPredictor:fast?1:0});
            let next;do{next=it.next();}while(!next.done);out.push(next.value);
            const {state,result}=next.value;assert.ok(state,JSON.stringify(result));
            assert.ok(result.certificateBound<=req.options.forceTolerance);
            assert.ok(result.residual.length<=req.options.lengthTolerance);
            assert.deepEqual(captureSharedAxisReplay(s,fixture.sheath),before);
            assert.equal(state.dynamicStep,null);assert.equal(state.wallFrictionStep,null);
        }
        const [a,b]=out;
        assert.ok(b.result.factorizations<=a.result.factorizations);
        assert.ok(b.result.fullAssemblies+b.result.residualAssemblies<=a.result.fullAssemblies+a.result.residualAssemblies);
        for(const x of new Set([...a.state.coordinates,...b.state.coordinates])){
            const p=sampleSharedAxisPosition(a.state,x),q=sampleSharedAxisPosition(b.state,x);
            assert.ok(Math.hypot(...p.map((v,k)=>v-q[k]))<.001);
        }
    }}finally{anatomy.dispose();}
});

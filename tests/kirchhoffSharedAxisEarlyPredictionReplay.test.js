import test, {after} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {captureSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';

const anatomy=await loadCoupledRuntimeAnatomy();after(()=>anatomy.dispose());
test('a poor anatomical prediction recovers with substantially fewer factorizations and unchanged acceptance criteria',()=>{
    const fixture=JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/shared-axis/anatomy-wire-569.80-poor-prediction.json.gz',import.meta.url))));
    const req=fixture.stepRequest,outputs=[];
    for(const earlyPredictorFallback of [false,true]) {
        const input=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisNative(input);
        const iterator=advanceSharedAxis(input,req.rotations,req.dt,req.tools,{...req.options,earlyPredictorFallback});
        let next;do{next=iterator.next();}while(!next.done);
        const {state,result}=next.value;
        assert.ok(state,JSON.stringify(result));
        assert.deepEqual(captureSharedAxisNative(input),before,'Attempts cannot mutate the incoming accepted state');
        assert.equal(result.subdivisions,1,'No smaller time/feed steps to conceal failed predictions');
        assert.ok(result.certificateBound<=req.options.forceTolerance);
        assert.ok(result.residual.length<=req.options.lengthTolerance);
        assert.ok(result.quality.finite);
        assert.ok(result.quality.maxPenetration<=req.options.lengthTolerance);
        outputs.push({state,result});
    }
    const [reference,early]=outputs;
    assert.ok(early.result.factorizations<reference.result.factorizations*.7,
        `${early.result.factorizations} vs ${reference.result.factorizations}`);
    for(let i=0;i<reference.state.positions.length;i++)
        assert.ok(Math.hypot(...reference.state.positions[i].map((v,a)=>v-early.state.positions[i][a]))<1e-4,
            'Recovery must return the same equilibrium within 0.1 micrometre');
    assert.equal(early.result.predictorRecovery.early,true);
});

test('catheter withdrawal can rediscover a wire sample across a certified 20 mm wall-free path',()=>{
    const fixture=JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/shared-axis/anatomy-catheter-withdraw-180.80-sign-proof.json.gz',import.meta.url))));
    const req=fixture.stepRequest,input=restoreSharedAxisReplay(fixture,anatomy.field);
    const iterator=advanceSharedAxis(input,req.rotations,req.dt,req.tools,req.options);
    let next;do{next=iterator.next();}while(!next.done);
    const {state,result}=next.value;
    assert.ok(state,JSON.stringify(result));
    assert.equal(result.subdivisions,1);
    assert.ok(result.factorizations<20);
    assert.ok(result.quality.finite);
    assert.ok(result.quality.maxPenetration<=req.options.lengthTolerance);
    assert.ok(result.certificateBound<=req.options.forceTolerance);
});

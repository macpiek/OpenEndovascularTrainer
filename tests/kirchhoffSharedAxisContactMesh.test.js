import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {captureSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';

test('fine mechanics ahead of the catheter recovers the captured coarse-contact obstruction at 659.53 mm',async()=>{
    const input=JSON.parse(gunzipSync(readFileSync(new URL('../reports/full-cycle-60hz-2026-09-19/contact-mesh-10mm/failed-incoming.json.gz',import.meta.url))));
    input.adaptiveMesh.tipRefinementAhead=40;
    const anatomy=await loadCoupledRuntimeAnatomy();
    try {
        const state=restoreSharedAxisReplay(input,anatomy.field),before=captureSharedAxisNative(state),r=input.stepRequest;
        const iterator=advanceSharedAxis(state,r.rotations,r.dt,r.tools,r.options);let next;
        do{next=iterator.next();}while(!next.done);
        const {state:accepted,result}=next.value;
        assert.ok(accepted&&result.converged,JSON.stringify(result));
        assert.equal(result.quality.finite,true);assert.ok(result.quality.maxPenetration<=r.options.lengthTolerance);
        assert.ok(result.certificateBound<=r.options.forceTolerance);
        assert.ok(result.factorizations<100,'Do not regress into the former 1820-factorization failure');
        for(const tool of r.tools)assert.equal(accepted.materials.find(m=>m.spec.id===tool.id).spec.insertion,tool.insertion);
        assert.deepEqual(captureSharedAxisNative(state),before,'Recovery must preserve the incoming physical checkpoint');
    }finally{anatomy.dispose();}
});

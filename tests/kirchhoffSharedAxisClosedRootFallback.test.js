import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {Ray,Vector3,DoubleSide} from 'three';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {captureSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';

test('closed-root buckling recovers without thousands of unpredicted fallback iterations',async()=>{
    const anatomy=await loadCoupledRuntimeAnatomy();
    try {
        const fixture=JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/shared-axis/closed-root-wire-840.40-friction-fallback.json.gz',import.meta.url))));
        const input=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisNative(input),q=fixture.stepRequest;
        const iterator=advanceSharedAxis(input,q.rotations,q.dt,q.tools,{...q.options,earlyPredictorFallback:false});
        let next;do{next=iterator.next();}while(!next.done);
        const {state,result}=next.value;
        assert.ok(state,JSON.stringify(result));assert.deepEqual(captureSharedAxisNative(input),before);
        assert.equal(result.subdivisions,1,'Recover the original full dt');
        assert.ok(result.factorizations<2000,`Observed ${result.factorizations}; captured reference needed 16232`);
        assert.ok(result.iterations<100,`Observed ${result.iterations}; captured reference needed 640`);
        assert.ok(result.certificateBound<=q.options.forceTolerance);
        assert.ok(result.residual.length<=q.options.lengthTolerance);
        assert.ok(result.quality.finite);
        const ray=new Ray();
        for(let i=0;i+1<state.positions.length;i++) {
            if(state.coordinates[i]<fixture.sheath.length)continue;
            const a=new Vector3(...state.positions[i]).add(new Vector3(...state.origin));
            const b=new Vector3(...state.positions[i+1]).add(new Vector3(...state.origin));
            const length=a.distanceTo(b);ray.origin.copy(a);ray.direction.subVectors(b,a).normalize();
            assert.equal(anatomy.geometry.boundsTree.raycastFirst(ray,DoubleSide,0,length),null,`Segment ${i} crosses the wall`);
        }
    }finally{anatomy.dispose();}
});

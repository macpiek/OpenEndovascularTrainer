import test from 'node:test';
import assert from 'node:assert/strict';
import {createPreparedInputCheckpoint} from '../src/physics/preparedInputCheckpoint.js';
import {createCoupledRuntimeFixture,poseFingerprint} from './helpers/coupledRuntimeFixture.js';

test('checkpoint restores typed storage identity, scalar controls and newly created feed fields with reusable buffers',()=>{
    const geometry={},x=new Float64Array([1,2]),input={progress:3,x,geometry},feed={};
    const checkpoint=createPreparedInputCheckpoint();checkpoint.capture([input,feed]);
    x[0]=99;input.x=new Float64Array([7]);input.progress=4;input.created=1;feed.body=input;feed.progress=4;
    checkpoint.restore();assert.equal(input.x,x);assert.deepEqual([...x],[1,2]);assert.equal(input.geometry,geometry);
    assert.equal(input.progress,3);assert.equal('created' in input,false);assert.deepEqual(feed,{});
    input.progress=5;x[1]=6;checkpoint.capture([input,feed]);input.progress=7;x[1]=8;
    checkpoint.restore();assert.equal(input.progress,5);assert.deepEqual([...x],[1,6]);
});

test('real sheath feed and catheter rotation restore presentation, feed history and layout before a reversed command',()=>{
    const f=createCoupledRuntimeFixture({catheterType:'pigtail'}),dt=1/60;
    try {
        const targets=[f.wire.nodeStorage,f.transport,f.transport.performanceStats,f.wireBody,f.catheterBody,
            f.catheter,f.catheter._sheathFeed??={},f.catheter._kirchhoffMaterialOptions,f.catheter._kirchhoffBoundaryOptions];
        const checkpoint=createPreparedInputCheckpoint();checkpoint.capture(targets);
        const before=[poseFingerprint(f.wireBody),poseFingerprint(f.catheterBody)];
        const originalProgress=f.catheter.progress,rotation=f.catheter.rotation;
        f.transport.advance(1,dt);f.wireBody.syncFromRodState(f.wire);
        f.catheter.advance(1,dt,f.transport.progress);f.catheter.rotate(1,dt);f.catheter.syncXpbdBody(f.catheterBody);
        assert.notEqual(f.catheter.progress,originalProgress);assert.notEqual(f.catheter.rotation,rotation);
        checkpoint.restore();
        assert.deepEqual([poseFingerprint(f.wireBody),poseFingerprint(f.catheterBody)],before);
        assert.equal(f.transport.progress,0);assert.equal(f.catheter.progress,originalProgress);assert.equal(f.catheter.rotation,rotation);
        f.transport.advance(1,dt);f.catheter.advance(1,dt,f.transport.progress);f.catheter.syncXpbdBody(f.catheterBody);
        assert.ok(f.catheterBody.velocityX.every(Number.isFinite));assert.ok(f.catheterBody.materialCoordinate.every(Number.isFinite));
        assert.ok(Math.abs(f.transport.progress-44*dt)<1e-12);
        assert.ok(Math.abs(f.catheter.progress-originalProgress-52*dt)<1e-12);
    }finally{f.dispose();}
});

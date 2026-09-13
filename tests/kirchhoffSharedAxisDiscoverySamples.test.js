import test from 'node:test';
import assert from 'node:assert/strict';
import {BufferGeometry,Float32BufferAttribute} from 'three';
import {createSharedAxisNative,assembleSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {createSharedAxisVesselDiscovery} from '../src/physics/kirchhoffSharedAxisVesselWitnesses.js';

test('one discovery sweep retains every already sampled near-wall site instead of only the capsule winner',()=>{
    const geometry=new BufferGeometry().setAttribute('position',new Float32BufferAttribute([
        -100,-100,10,100,-100,10,0,100,10,
        -100,-100,11,100,-100,11,0,100,11,
        -100,-100,12,100,-100,12,0,100,12],3));
    let queries=0;
    const field={fallbackGeometry:geometry,voxelSize:1,queryCapsuleSoA(x,y,z,r,index,out,face,inside,clearance,branch,near,length,count,physical,finite,visit){
        queries++;assert.equal(typeof visit,'function');
        for(const [t,faceIndex,signedGap] of [[0,0,.2],[.5,1,.3],[1,2,2]])visit({faceIndex,signedGap,signedDistance:signedGap+r[0]},t);
        return Object.assign(out,{faceIndex:0,signedGap:.2,signedDistance:.2+r[0],segmentT:0});
    }};
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:15}],wallSamples:[createSharedAxisVesselDiscovery(field,0)]});
    assert.throws(()=>assembleSharedAxisNative(s),/shared-axis-wall-discovery/);
    assert.equal(queries,3);assert.equal(s.pendingVesselRows.size,6);
    for(let e=0;e<3;e++)assert.deepEqual([...s.pendingVesselRows.values()].filter(r=>r.edge===e).map(r=>r.witness.t),[0,.5]);
    geometry.dispose();
});

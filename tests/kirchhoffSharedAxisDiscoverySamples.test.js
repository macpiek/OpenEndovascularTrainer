import test from 'node:test';
import assert from 'node:assert/strict';
import {BufferGeometry,Float32BufferAttribute} from 'three';
import {createSharedAxisNative,assembleSharedAxisNative,extendSharedAxisNativeRows,feedSharedAxisNative,captureSharedAxisNative,restoreSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {createSharedAxisVesselDiscovery} from '../src/physics/kirchhoffSharedAxisVesselWitnesses.js';
import {captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';

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

test('indexed discovery matches the scan through duplicate samples, rollback, withdrawal and reinsertion',()=>{
    const geometry=new BufferGeometry().setAttribute('position',new Float32BufferAttribute([
        -100,-100,10,100,-100,10,0,100,10],3));
    const field={fallbackGeometry:geometry,voxelSize:1,queryCapsuleSoA(x,y,z,r,index,out,face,inside,clearance,branch,near,length,count,physical,finite,visit){
        // Repeated visits must remain one pending row and subsequently one
        // retained row, even after abandoning a trial pose.
        for(const t of [0,.5,.5])visit({faceIndex:0,signedGap:.2,signedDistance:.2+r[0]},t);
        return Object.assign(out,{faceIndex:0,signedGap:.2,signedDistance:.2+r[0],segmentT:0});
    }};
    function run(indexedContacts) {
        const discovery=createSharedAxisVesselDiscovery(field,0,{indexedContacts,queryReuse:false});
        let s=createSharedAxisNative({tools:[{id:'wire',insertion:15}],wallSamples:[discovery]});
        const states=[];
        const discover=()=>{
            if(indexedContacts)s.definitions.some=()=>{throw new Error('Unexpected full contact scan');};
            assert.throws(()=>assembleSharedAxisNative(s),/shared-axis-wall-discovery/);
            const pending=[...s.pendingVesselRows.values()];
            assert.equal(new Set(pending.map(r=>r.id)).size,pending.length);
            extendSharedAxisNativeRows(s,pending);s.pendingVesselRows.clear();
            const before=captureSharedAxisNative(s);
            s.positions[2][1]+=.01;assembleSharedAxisNative(s);restoreSharedAxisNative(s,before);
            assembleSharedAxisNative(s);
            assert.equal(s.pendingVesselRows.size,0);
            assert.deepEqual(s.definitionIds,new Set(s.definitions.map(r=>r.id)));
            states.push(captureSharedAxisReplay(s,{}));
        };
        discover();
        const original=s,removed=[...s.definitionIds].filter(id=>typeof id==='string'&&id.startsWith('vessel/10/15/'));
        s=feedSharedAxisNative(s,{wire:10});
        assert.notEqual(s.definitionIds,original.definitionIds);
        assert.ok(removed.every(id=>!s.definitionIds.has(id)&&original.definitionIds.has(id)));
        assembleSharedAxisNative(s);states.push(captureSharedAxisReplay(s,{}));
        s=feedSharedAxisNative(s,{wire:20});discover();
        assert.ok(removed.every(id=>s.definitionIds.has(id)),'reinsertion must rediscover discarded spatial contacts');
        return states;
    }
    try{assert.deepEqual(run(true),run(false));}finally{geometry.dispose();}
});

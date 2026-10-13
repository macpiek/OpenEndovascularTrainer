import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {createSharedAxisNative,feedSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {createSharedAxisContacts} from '../src/physics/kirchhoffSharedAxisContacts.js';
import {createStentGraftContacts} from '../src/devices/stentGraftContacts.js';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {exposedGraftFaces} from '../src/devices/stentGraftDeployment.js';
import {captureSharedAxisReplay,restoreSharedAxisReplay} from './helpers/sharedAxisReplay.js';

function fabric() {
    const geometry=new THREE.PlaneGeometry(30,10).translate(5,0,0);
    geometry.boundsTree=new MeshBVH(geometry);geometry.computeBoundingBox();
    return {geometry,bounds:geometry.boundingBox,revision:1};
}
const tools=[{id:'wire',insertion:10},{id:'catheter',insertion:10,type:'stentgraft-delivery',radius:2,deliveryExposureMm:4}];

test('cover carries its own graft while the exposed core and other implants retain full contact',()=>{
    const surface={...fabric(),complianceApplied:true,contactPatches:[{coordinate:8,radius:.8,point:[8,0,.8],normal:[0,0,1],depth:.2,width:3}]};
    const state=createSharedAxisNative({tools,spacing:2,samplePosition:s=>[s,0,.2]});
    const energy=surface=>{
        state.chain.gradient.fill(0);
        const sampler=createStentGraftContacts(surface,state);
        return sampler.addPotential(state,false);
    };
    try {
        const full=energy(surface),core=energy({...surface,coveredDeliverySurface:null});
        assert.ok(core>0&&core<full,'the core still loads fabric beyond the withdrawing cover');
        for(const node of [0,1,2])for(let k=0;k<3;k++)assert.equal(state.chain.gradient[state.layout.positions[node]+k],0);
        assert.ok(Math.abs(energy({...surface,coveredDeliverySurface:surface})/full-1)<1e-12,
            'another implant receives both cover and core contact, exactly once');
    } finally {surface.geometry.dispose();}
});

test('cover exclusion does not disable core crossings or cover crossings through another implant',()=>{
    const surface=fabric(),state=createSharedAxisNative({tools,spacing:2,samplePosition:s=>[s,0,3]});
    const query={state,a:[2,0,-3],b:[4,0,-3],radius:2,coordinateA:2,coordinateB:4};
    try {
        const own=createStentGraftContacts({...surface,coveredDeliverySurface:null},state);
        assert.doesNotThrow(()=>own(query));
        assert.throws(()=>own({...query,a:[8,0,-3],b:[10,0,-3],radius:.8,coordinateA:8,coordinateB:10}),e=>e.code==='trial-outside-vessel');
        const other=createStentGraftContacts({...surface,coveredDeliverySurface:surface},state);
        assert.throws(()=>other(query),e=>e.code==='trial-outside-vessel');
    } finally {surface.geometry.dispose();}
});

for(const withOther of [false,true])test(`replay preserves cover contact ownership (${withOther?'other implant':'own implant only'})`,()=>{
    const surface=fabric(),sheath={start:[0,0,3],end:[2,0,3],innerRadius:3,proximalExtension:20};
    let state=createSharedAxisNative({...createSharedAxisContacts({sheath}),tools,spacing:2}),restored;
    try {
        state.wallSamples.push(createStentGraftContacts({...surface,coveredDeliverySurface:withOther?surface:null},state));
        state=feedSharedAxisNative(state,{});
        const saved=captureSharedAxisReplay(state,sheath);
        restored=restoreSharedAxisReplay(saved,null);
        const actual=captureSharedAxisReplay(restored,sheath).graftContactSurface;
        assert.ok(Object.hasOwn(actual,'coveredDeliverySurface'));
        assert.deepEqual(actual.coveredDeliverySurface,saved.graftContactSurface.coveredDeliverySurface);
    } finally {restored?.graftReplayGeometry.dispose();surface.geometry.dispose();}
});

test('each delivery cover excludes only its own unfolding implant from an immutable surface snapshot',()=>{
    const {system,device}=previewFixture('right');
    try {
        system.updateAccess('right',3,null,{deviceId:device.id,release:'sheath'});
        assert.equal(system.mechanicalSurfaceForAccess('right').coveredDeliverySurface,null);
        assert.equal(Object.hasOwn(system.mechanicalSurfaceForAccess('left'),'coveredDeliverySurface'),false);
        assert.ok(system.load('left','limb').ok);
        const other=system.accesses.left.device;other.position=other.target=device.position;
        system.deploy('left');system.updateAccess('left',3,null,{deviceId:other.id,release:'sheath'});
        const right=system.mechanicalSurfaceForAccess('right'),left=system.mechanicalSurfaceForAccess('left');
        assert.equal(right.coveredDeliverySurface.geometry.attributes.position.count,exposedGraftFaces(other).length*3);
        assert.equal(left.coveredDeliverySurface.geometry.attributes.position.count,exposedGraftFaces(device).length*3);
        assert.ok(right.coveredDeliverySurface.geometry.attributes.position.count>0);
        const saved=Array.from(right.coveredDeliverySurface.geometry.attributes.position.array);
        system.updateAccess('left',1/60,null,{deviceId:other.id,release:'sheath'});
        assert.deepEqual(Array.from(right.coveredDeliverySurface.geometry.attributes.position.array),saved);
    } finally {system.dispose();}
});

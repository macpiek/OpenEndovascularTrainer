import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {deliveryMaterialProfile,prepareDeliveryMotion,DELIVERY_COVER_EI,DELIVERY_CORE_EI} from '../src/devices/stentGraftDeliveryMechanics.js';
import {createSharedAxisNative,relaxSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {defineKirchhoffMaterialProfile} from '../src/physics/kirchhoffMaterialProfile.js';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {createStentGraftContacts} from '../src/devices/stentGraftContacts.js';

const beam=defineKirchhoffMaterialProfile({id:'straight-wire-test',sampleEI1:()=>1e5,sampleGJ:()=>1e5});
test('delivery adds bending stiffness to the same guidewire; retraction of the cover softens only exposed material',()=>{
    const deflections=[];
    for(const exposure of [null,0,100]) {
        const tools=[{id:'wire',insertion:100,type:beam}];
        if(exposure!==null)tools.push({id:'catheter',insertion:100,type:'stentgraft-delivery',deliveryExposureMm:exposure,radius:3});
        const state=createSharedAxisNative({tools,spacing:5});
        state.loads[state.layout.positions.at(-1)+1]=.1;
        const result=relaxSharedAxisNative(state,{maxIterations:100,forceTolerance:1e-6});
        assert.ok(result.converged,JSON.stringify(result));
        deflections.push(state.positions.at(-1)[1]);
        const expected=.1*100**3/(3*(1e5+(exposure===null?0:exposure===0?DELIVERY_COVER_EI:DELIVERY_CORE_EI)));
        assert.ok(Math.abs(state.positions.at(-1)[1]/expected-1)<.02);
        state.loads.fill(0);assert.ok(relaxSharedAxisNative(state,{maxIterations:100}).converged);
        assert.ok(Math.abs(state.positions.at(-1)[1])<1e-3);
    }
    assert.ok(deflections[1]<deflections[0]/50,'covered delivery must mechanically brace the wire');
    assert.ok(deflections[2]>deflections[1]*10,'withdrawn cover must not retain its full rigidity');
    const profile=deliveryMaterialProfile(30);
    assert.equal(profile.sample(10).EI1,DELIVERY_CORE_EI);
    assert.equal(profile.sample(40).EI1,DELIVERY_COVER_EI);
    assert.equal(profile.sample(10).kappa01,0);
});

test('motion prepares only the solver input, limits wire overrun, and publishes only matching committed steps',()=>{
    const {system,device:d}=previewFixture();
    try {
        d.phase='loaded';d.position=d.target=30;
        const proxy={progress:30,advance(command,dt,wire,speed){this.progress=Math.max(0,this.progress+command*dt*speed);}};
        prepareDeliveryMotion(d,proxy,1,1,50);
        assert.equal(proxy.progress,38);assert.equal(d.position,30,'trial is not committed');
        proxy.progress=30;assert.equal(d.position,30,'rollback preserves public position');
        system.updateAccess('right',0,null,{deviceId:d.id,mechanicalPosition:38});assert.equal(d.position,30);
        system.updateAccess('right',1,null,{deviceId:d.id+1,mechanicalPosition:38});assert.equal(d.position,30);
        system.updateAccess('right',1,null,{deviceId:d.id,advance:1,mechanicalPosition:38});assert.equal(d.position,38,'no second kinematic advance');
        d.target=0;proxy.progress=38;prepareDeliveryMotion(d,proxy,1,0,50);assert.equal(proxy.progress,13,'remove delivery can request withdrawal');
        d.phase='deploying';proxy.progress=38;prepareDeliveryMotion(d,proxy,1,-1,50);assert.equal(proxy.progress,13,'release still permits delivery withdrawal');
    } finally {system.dispose();}
});

for(const side of ['right','left'])test(`${side}: exposed fabric contacts the wire before full deployment with immutable open-ended snapshots`,()=>{
    const {system,device:d}=previewFixture(side);
    try {
        assert.equal(system.mechanicalSurface,null);
        system.updateAccess(side,3,null,{deviceId:d.id,release:'sheath'});
        const surface=system.mechanicalSurface;
        assert.ok(surface?.geometry.attributes.position.count>0);assert.equal(system.surface,null,'contrast must not seal prematurely');
        assert.ok(d.deployment<1);
        const saved=Array.from(surface.geometry.attributes.position.array),revision=surface.revision;
        const center=d.parts[0].path.sample(10),wall=new THREE.Vector3().fromArray(saved,0);
        const triangleCenter=new THREE.Vector3().fromArray(saved,0).add(new THREE.Vector3().fromArray(saved,3)).add(new THREE.Vector3().fromArray(saved,6)).multiplyScalar(1/3);
        const radial=triangleCenter.clone().sub(d.parts[0].path.nearest(triangleCenter).point).normalize();
        const inside=triangleCenter.clone().addScaledVector(radial,-1),outside=triangleCenter.clone().addScaledVector(radial,1);
        const previous={coordinates:[0,1],positions:[inside.toArray(),inside.toArray()],origin:[0,0,0]};
        const sampler=createStentGraftContacts(surface,previous);
        assert.throws(()=>sampler({state:{origin:[0,0,0]},a:outside.toArray(),b:outside.toArray(),radius:.2,coordinateA:0,coordinateB:1}),e=>e.code==='trial-outside-vessel');
        const inlet=d.parts[0].points[0],direction=center.clone().sub(inlet).normalize();
        const ray=new THREE.Ray(inlet.clone().addScaledVector(direction,-5),direction);
        assert.equal(surface.geometry.boundsTree.raycastFirst(ray,THREE.DoubleSide)?.distance??Infinity,Infinity,'no artificial end cap');
        system.updateAccess(side,1,null,{deviceId:d.id,release:'sheath'});
        assert.ok(system.mechanicalSurface.revision>revision);
        assert.deepEqual(Array.from(surface.geometry.attributes.position.array),saved);
        const frozen=system.mechanicalSurface;system.updateAccess(side,1);assert.equal(system.mechanicalSurface,frozen,'holding release reuses the surface');
    } finally {system.dispose();}
});

test('actual catheter adapter uses the stiff profile, hides its duplicate mesh, and restores a normal catheter',async()=>{
    const {PigtailCatheter}=await import('../src/pigtailCatheter.js');
    const {RodState}=await import('../src/physics/rodState.js');
    const {EndovascularPhysicsWorld,DEFAULT_TOOL_PROFILES}=await import('../src/physics/endovascularPhysicsWorld.js');
    const sheath={start:new THREE.Vector3(),end:new THREE.Vector3(10,0,0)};
    const wire=new RodState(41,5),world=new EndovascularPhysicsWorld();
    const catheter=new PigtailCatheter({wire,segmentLength:5,guidewireLength:200,tailProgressRef:()=>100,vessel:{sheath},retainMaterialTip:true});
    try {
        const body=world.createRod('catheter',41,5,DEFAULT_TOOL_PROFILES.catheter);
        catheter.setType('stentgraft-delivery');catheter.setStiffnessScales({shaftStiffnessScale:1,tipStiffnessScale:1});
        catheter.advance(1,1,100,25);catheter.syncXpbdBody(body);catheter.updateMesh();
        assert.equal(catheter.progress,25);assert.equal(body.radius,3);assert.equal(catheter.mesh.visible,false);
        assert.deepEqual(catheter.getInjectionPorts([]),[]);
        assert.ok(Array.from(body.x).every(Number.isFinite));
        catheter.setType('berenstein');catheter.syncXpbdBody(body);catheter.updateMesh();
        assert.ok(body.radius<1);assert.ok(catheter.mesh.visible);
    } finally {catheter.dispose();}
});

test('owning access gets an inside-lumen constraint during release; the other access keeps two-sided wall contact',()=>{
    const {system,device:d}=previewFixture();
    try {
        system.updateAccess('right',3,null,{deviceId:d.id,release:'sheath'});
        const own=system.mechanicalSurfaceForAccess('right'),other=system.mechanicalSurfaceForAccess('left');
        assert.ok(own.lumenSections.length>0);assert.equal(other.lumenSections.length,0);
        const old=structuredClone(own.lumenSections);d.position-=10;
        system.updateAccess('right',1,null,{deviceId:d.id,release:'sheath'});
        assert.deepEqual(own.lumenSections,old,'snapshot and implant anchoring survive delivery withdrawal');
    } finally {system.dispose();}
});

test('a wire outside an incoming graft is pulled inside rather than repelled to the outside of its sheet',async()=>{
    const {MeshBVH}=await import('three-mesh-bvh');
    const {feedSharedAxisNative}=await import('../src/physics/kirchhoffSharedAxisNative.js');
    const geometry=new THREE.CylinderGeometry(3,3,60,48,1,true).rotateZ(-Math.PI/2).translate(50,0,0);
    geometry.boundsTree=new MeshBVH(geometry);geometry.computeBoundingBox();
    const surface={geometry,bounds:geometry.boundingBox,revision:1,lumenSections:[{start:20,end:80,a:[20,0,0],b:[80,0,0],radiusA:3,radiusB:3}]};
    try {
        let state=createSharedAxisNative({tools:[{id:'wire',insertion:65,type:beam,radius:.2}],spacing:5});
        state.loads[state.layout.positions.at(-1)+1]=5;
        assert.ok(relaxSharedAxisNative(state,{maxIterations:100,forceTolerance:1e-5}).converged);
        assert.ok(state.positions.at(-1)[1]>3.5,'wire begins outside the final fabric');
        state.wallSamples.push(createStentGraftContacts(surface,state));state.graftRevision=1;
        state=feedSharedAxisNative(state,{});state.loads[state.layout.positions.at(-1)+1]=5;
        const result=relaxSharedAxisNative(state,{maxIterations:150,forceTolerance:1e-5});
        assert.ok(result.converged,JSON.stringify(result));
        assert.ok(state.positions.at(-1)[1]<2.9,'own graft confines the wire despite incoming overlap');
        assert.ok(state.graftContactStats.contacts>0);
    } finally {geometry.dispose();}
});

test('failure replay preserves exposed rigidity and the owning lumen constraint',async()=>{
    const {captureSharedAxisReplay,restoreSharedAxisReplay}=await import('./helpers/sharedAxisReplay.js');
    const {createSharedAxisContacts}=await import('../src/physics/kirchhoffSharedAxisContacts.js');
    const {feedSharedAxisNative}=await import('../src/physics/kirchhoffSharedAxisNative.js');
    const {system,device:d}=previewFixture();
    let restored;
    try {
        system.updateAccess('right',3,null,{deviceId:d.id,release:'sheath'});
        const surface=system.mechanicalSurfaceForAccess('right');
        const sheath={start:[0,0,0],end:[10,0,0],innerRadius:3.2,proximalExtension:40};
        let state=createSharedAxisNative({...createSharedAxisContacts({sheath}),tools:[
            {id:'wire',insertion:65,type:'glidewire'},
            {id:'catheter',insertion:50,type:'stentgraft-delivery',radius:3,deliveryExposureMm:20}]});
        state.wallSamples.push(createStentGraftContacts(surface,state));state.graftRevision=surface.revision;
        state=feedSharedAxisNative(state,{});
        const replay=JSON.parse(JSON.stringify(captureSharedAxisReplay(state,sheath)));
        restored=restoreSharedAxisReplay(replay,null);
        assert.equal(restored.materials[1].spec.deliveryExposureMm,20);
        assert.deepEqual(restored.wallSamples.find(s=>s.graftSurface).surface.lumenSections,surface.lumenSections);
    } finally {restored?.graftReplayGeometry?.dispose();system.dispose();}
});

import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {createStentGraftContacts,graftContactResponse} from '../src/devices/stentGraftContacts.js';
import {createSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {defineKirchhoffMaterialProfile} from '../src/physics/kirchhoffMaterialProfile.js';

function gate() {
 const geometry=new THREE.CylinderGeometry(3,3,60,48,1,true).rotateZ(-Math.PI/2).translate(50,0,0);
 geometry.boundsTree=new MeshBVH(geometry);geometry.computeBoundingBox();
 return {geometry,bounds:geometry.boundingBox,revision:1};
}
const type=defineKirchhoffMaterialProfile({id:'cannulation-beam',sampleEI1:()=>1e3,sampleGJ:()=>1e3});

test('fabric response has consistent energy, force and tangent including near-surface recovery',()=>{
 const radius=.4445,h=1e-7;
 for(const distance of [.44,.4,.2,.05,.002]) {
  const r=graftContactResponse(distance,radius),a=graftContactResponse(distance-h,radius),b=graftContactResponse(distance+h,radius);
  assert.ok(r.slope<0&&r.curvature>0);
  assert.ok(Math.abs((b.energy-a.energy)/(2*h)-r.slope)<1e-4*Math.abs(r.slope));
  assert.ok(Math.abs((b.slope-a.slope)/(2*h)-r.curvature)<1e-4*r.curvature);
 }
 assert.deepEqual(graftContactResponse(radius+.1,radius),{energy:0,slope:0,curvature:0});
});

test('wire enters the open gate; a wider catheter follows, slides on the rim and can be withdrawn',()=>{
 const surface=gate();
 let state=createSharedAxisNative({tools:[{id:'wire',insertion:18,type,radius:.4445},{id:'catheter',insertion:10,type,radius:.8333}],spacing:2,samplePosition:x=>[x,2.5,0]});
 let touched=false;
 const solve=(insertions)=>{
  state.wallSamples=[createStentGraftContacts(surface,state)];state.graftRevision=surface.revision;
  const tools=state.materials.map(m=>({...m.spec,rotation:0,insertion:insertions[m.spec.id]??m.spec.insertion}));
  const iterator=advanceSharedAxis(state,{wire:0,catheter:0},1/60,tools,{maxIterations:160,forceTolerance:1e-5,lengthTolerance:1e-5});
  let outcome;do{outcome=iterator.next();}while(!outcome.done);
  const {state:next,result}=outcome.value;
  assert.ok(next&&result.converged,JSON.stringify({insertions,...result}));
  touched||=next.graftContactStats?.contacts>0;
  state=next;
  const ray=new THREE.Ray();
  for(let i=1;i<state.positions.length;i++){
   const a=new THREE.Vector3(...state.positions[i-1]),b=new THREE.Vector3(...state.positions[i]);
   ray.origin.copy(a);ray.direction.copy(b).sub(a).normalize();
   assert.equal(surface.geometry.boundsTree.raycastFirst(ray,THREE.DoubleSide,1e-7,a.distanceTo(b)),null,'accepted shaft cannot cut fabric');
  }
 };
 try {
  for(let x=18.5;x<=65;x+=.5)solve({wire:x});
  assert.ok(state.positions.at(-1)[0]>60,'guidewire crosses the open inlet');
  for(let x=10.5;x<=55;x+=.5)solve({catheter:x});
  assert.ok(touched,'larger catheter must contact the rim, rather than ignoring fabric');
  const tip=state.positions[state.materials.find(m=>m.spec.id==='catheter').last];
  assert.ok(tip[0]>50&&Math.hypot(tip[1],tip[2])<2.2,'catheter is redirected inside the gate');
  for(let x=54.5;x>=10;x-=.5)solve({catheter:x});
  for(let x=64.5;x>=18;x-=.5)solve({wire:x});
 }finally{surface.geometry.dispose();}
});

test('captured left-access rejection recovers with fabric CCD enabled and continues advancing',async()=>{
 const {readFileSync}=await import('node:fs'),{gunzipSync}=await import('node:zlib');
 const {fixture}=await import('./helpers/stentGraftFixture.js');
 const {restoreSharedAxisReplay}=await import('./helpers/sharedAxisReplay.js');
 const {VesselContactField}=await import('../src/physics/collision/vesselContactField.js');
 const {decodeCollisionAsset}=await import('../src/physics/collision/collisionAssetFormat.js');
 const input=JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/shared-axis/stent-graft-left-contact.json.gz',import.meta.url))));
 const f=fixture(),bytes=readFileSync(new URL('../res/Aorta_infrarenal_aneurysm.collision.bin',import.meta.url));
 const field=new VesselContactField(decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength)),{fallbackGeometry:f.geometry,bvhValidationDistance:.02,capsuleBvhValidation:-.1});
 let geometry;
 try {
  let state=restoreSharedAxisReplay(input,field);geometry=state.graftReplayGeometry;
  const surface=state.wallSamples.find(s=>s.graftSurface).surface;
  let rotations=input.stepRequest.rotations;
  for(let step=0;step<40;step++) {
   const sampler=createStentGraftContacts(surface,state);
   const source={...state,wallSamples:[...state.wallSamples.filter(s=>!s.graftSurface),sampler],graftRevision:surface.revision,graftRecovery:sampler.recovery};
   const tools=input.stepRequest.tools.map(t=>({...t,insertion:t.insertion+(t.id==='wire'?Math.max(0,step-20)*.3:0)}));
   const iterator=advanceSharedAxis(source,rotations,input.stepRequest.dt,tools,input.stepRequest.options);let next;
   do{next=iterator.next();}while(!next.done);
   assert.ok(next.value.state&&next.value.result.converged,JSON.stringify({step,...next.value.result}));
   if(step===0)assert.ok(next.value.result.iterations<40,'previously terminal 93-iteration step now converges');
   state=next.value.state;rotations=next.value.rotations;
   assert.ok(state.graftContactStats.maxPenetration<.1,'finite tool radius is supported at the cloth');
   const ray=new THREE.Ray();
   for(let i=1;i<state.positions.length;i++) {
    const a=new THREE.Vector3(...state.positions[i-1]).add(new THREE.Vector3(...state.origin));
    const b=new THREE.Vector3(...state.positions[i]).add(new THREE.Vector3(...state.origin));
    ray.origin.copy(a);ray.direction.copy(b).sub(a).normalize();
    assert.equal(geometry.boundsTree.raycastFirst(ray,THREE.DoubleSide,1e-7,a.distanceTo(b)),null,'no accepted segment crosses fabric');
   }
  }
 }finally{geometry?.dispose();f.dispose();}
});

for(const side of ['right','left'])test(`${side} main body: wire and catheter traverse the actual contralateral gate`,async()=>{
 const {fixture,place,finish}=await import('./helpers/stentGraftFixture.js');
 const f=fixture();
 try {
  place(f.system,side,'body');f.system.deploy(side);finish(f.system,side);
  const body=f.system.accesses[side].device,surface=f.system.surface;
  const entry=body.gate.entry.clone(),part=body.parts[2],normal=entry.clone().sub(part.points.at(-2)).normalize();
  let state=createSharedAxisNative({tools:[{id:'wire',insertion:10,type,radius:.4445},{id:'catheter',insertion:4,type,radius:.8333}],spacing:2,
   samplePosition:x=>entry.clone().addScaledVector(normal,25-x).toArray()});
  for(const id of ['wire','catheter'])for(let insertion=(id==='wire'?10:4)+.5;insertion<=43;insertion+=.5) {
   const sample=createStentGraftContacts(surface,state);
   const source={...state,wallSamples:[sample],graftRevision:surface.revision,graftRecovery:sample.recovery};
   const tools=state.materials.map(m=>({...m.spec,rotation:0,insertion:m.spec.id===id?insertion:m.spec.insertion}));
   const iterator=advanceSharedAxis(source,{wire:0,catheter:0},1/60,tools,{forceTolerance:1e-5,lengthTolerance:1e-5});let next;
   do{next=iterator.next();}while(!next.done);
   assert.ok(next.value.state&&next.value.result.converged,JSON.stringify({side,id,insertion,...next.value.result}));
   state=next.value.state;
  }
  assert.ok(surface.contains(new THREE.Vector3(...state.positions.at(-1))),'both tips finish inside the short leg through its open end');
 }finally{f.dispose();}
});

test('owning circular lumen cannot suppress real cloth contact during pigtail withdrawal',async()=>{
 const {readFileSync}=await import('node:fs'),{gunzipSync}=await import('node:zlib');
 const {fixture}=await import('./helpers/stentGraftFixture.js');
 const {restoreSharedAxisReplay}=await import('./helpers/sharedAxisReplay.js');
 const {VesselContactField}=await import('../src/physics/collision/vesselContactField.js');
 const {decodeCollisionAsset}=await import('../src/physics/collision/collisionAssetFormat.js');
 const input=JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/shared-axis/stent-graft-owning-contact.json.gz',import.meta.url))));
 const f=fixture(),bytes=readFileSync(new URL('../res/Aorta_infrarenal_aneurysm.collision.bin',import.meta.url));
 const field=new VesselContactField(decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength)),{fallbackGeometry:f.geometry,bvhValidationDistance:.02,capsuleBvhValidation:-.1});
 let geometry;
 try {
  let state=restoreSharedAxisReplay(input,field);geometry=state.graftReplayGeometry;
  const surface=state.wallSamples.find(s=>s.graftSurface).surface;
  assert.ok(surface.lumenSections.length>0,'owning-access guidance is active');
  let rotations=input.stepRequest.rotations;
  for(let step=0;step<30;step++) {
   const sampler=createStentGraftContacts(surface,state);
   const source={...state,wallSamples:[...state.wallSamples.filter(s=>!s.graftSurface),sampler],graftRevision:surface.revision,graftRecovery:sampler.recovery};
   const tools=input.stepRequest.tools.map(t=>({...t,insertion:t.insertion-(t.id==='catheter'?Math.max(0,step-10)*.1:0)}));
   const iterator=advanceSharedAxis(source,rotations,input.stepRequest.dt,tools,input.stepRequest.options);let next;
   do{next=iterator.next();}while(!next.done);
   assert.ok(next.value.state&&next.value.result.converged,JSON.stringify({step,...next.value.result}));
   if(!step)assert.ok(next.value.result.iterations<40,'the captured 84-iteration rejection recovers');
   state=next.value.state;rotations=next.value.rotations;
   assert.ok(state.positions.every(p=>p.every(Number.isFinite)));
  }
 }finally{geometry?.dispose();f.dispose();}
});

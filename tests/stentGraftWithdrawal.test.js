import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {deliveryMechanicalExtent,prepareDeliveryMotion,DELIVERY_COVER_EI,deliveryMaterialProfile} from '../src/devices/stentGraftDeliveryMechanics.js';
import {deliveryNoseState,advanceRelease} from '../src/devices/stentGraftDeployment.js';
import {graftLumenAt} from '../src/devices/stentGraftLumenContact.js';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {createSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {createStentGraftContacts} from '../src/devices/stentGraftContacts.js';
const proxy=progress=>({progress,maxLength:1000,setType(){},setStiffnessScales(){},advance(command,dt,wire,speed){this.progress=Math.max(0,this.progress+command*dt*speed);}});
const device=()=>({type:'body',diameter:23,phase:'deployed',position:244.58333333333127,target:244.58333333333127,implantPosition:308,tipRelease:1,sheathWithdrawal:120.2,sheathTravel:136,coverLead:12,noseRetraction:0});

test('nose retraction and handle withdrawal prepare the same physical extent as the committed visible core',()=>{
 const d=device(),catheter=proxy(d.position),before=structuredClone(d);
 for(let i=0;i<700;i++) {
  const position=prepareDeliveryMotion(d,catheter,1/60,0,437,'nose');
  if(i===0)assert.deepEqual(d,before,'preparing a release must not mutate the implant');
  d.position=d.target=position;advanceRelease(d,1/60,'nose');
  assert.ok(Math.abs(catheter.progress-deliveryMechanicalExtent(d).insertion)<1e-8);
 }
 assert.equal(deliveryNoseState(d).remaining,0);
 assert.ok(Math.abs(catheter.progress-136.38333333333128)<1e-8,'old 244.58 mm shaft no longer remains in physics');
 assert.equal(catheter.deliveryExposureMm,0,'collapsed core does not leave a soft phantom span in front of the sleeve');
 assert.equal(deliveryMaterialProfile(catheter.deliveryExposureMm).sample(0).EI1,DELIVERY_COVER_EI);
 const implant=d.implantPosition;
 for(let i=0;i<700;i++) {
  const position=prepareDeliveryMotion(d,catheter,1/60,-1,437);
  d.position=d.target=position;advanceRelease(d,1/60,null);
  assert.ok(Math.abs(catheter.progress-deliveryMechanicalExtent(d).insertion)<1e-8);
  assert.equal(d.implantPosition,implant);
 }
 assert.equal(d.position,0);assert.equal(catheter.progress,0);
});

test('rejected nose motion can be prepared again without consuming release or drifting the handle',()=>{
 const d={...device(),noseRetraction:40},catheter=proxy(deliveryMechanicalExtent({...device(),noseRetraction:40}).insertion);
 const original=structuredClone(d),accepted=catheter.progress;
 const first=prepareDeliveryMotion(d,catheter,1/60,-1,437,'nose'),trial=catheter.progress;
 catheter.progress=accepted; // the app checkpoint restores a rejected candidate
 const second=prepareDeliveryMotion(d,catheter,1/60,-1,437,'nose');
 assert.equal(first,second);assert.equal(catheter.progress,trial);assert.deepEqual(d,original);
});

test('release recovery cannot attract a wire beyond either open end based on its material coordinate',()=>{
 const sections=[{start:0,end:10,a:[0,0,0],b:[10,0,0],radiusA:3,radiusB:3}];
 for(const p of [[20,4,0],[-1,4,0]])assert.equal(graftLumenAt(sections,5,p,.4445),null);
 assert.ok(graftLumenAt(sections,5,[5,4,0],.4445).penetration>0,'incoming cloth recovery remains available inside the section');
});

for(const side of ['right','left'])test(`${side}: complete deployment removes the artificial lumen but preserves cloth CCD`,()=>{
 const {system,device:d}=previewFixture(side);
 try {
  for(let i=0;i<1500&&d.phase!=='deployed';i++)system.updateAccess(side,1/30,null,{deviceId:d.id,release:{sheath:1,tip:true}});
  assert.equal(d.phase,'deployed');
  const surface=system.mechanicalSurfaceForAccess(side);
  assert.equal(surface.lumenSections.length,0);
  // Start inside the threaded branch. Outside-to-inside motion may recover
  // a divider that opened over the delivery wire; outward crossings must fail.
  const center=d.parts[1].path.sample(d.parts[1].path.length*.75);
  const hit=surface.geometry.boundsTree.closestPointToPoint(center,{});
  const a=center.toArray(),b=hit.point.clone().addScaledVector(hit.point.clone().sub(center).normalize(),2).toArray();
  const sampler=createStentGraftContacts(surface,{coordinates:[0,1],positions:[a,a],origin:[0,0,0]});
  assert.throws(()=>sampler({state:{origin:[0,0,0]},a:b,b,radius:.2,coordinateA:0,coordinateB:1}),e=>e.code==='trial-outside-vessel');
 }finally{system.dispose();}
});

test('delivery core can retract and withdraw through an open graft with cloth contacts active',()=>{
 const geometry=new THREE.CylinderGeometry(4,4,60,48,1,true).rotateZ(-Math.PI/2).translate(50,0,0);
 geometry.boundsTree=new MeshBVH(geometry);geometry.computeBoundingBox();
 const surface={geometry,bounds:geometry.boundingBox,revision:1,lumenSections:[]};
 const d={...device(),position:90,target:90,sheathWithdrawal:50,sheathTravel:80},catheter=proxy(90);
 let state=createSharedAxisNative({startCoordinate:-40,boundaryCoordinates:[0],tools:[{id:'wire',insertion:110,type:'glidewire',radius:.4445},{id:'catheter',insertion:90,type:'stentgraft-delivery',radius:3,deliveryExposureMm:38}],spacing:3,samplePosition:x=>[x,1.1,0]}),contact=false;
 try {
  for(let i=0;i<180;i++) {
   const release=i<120?'nose':null,dt=1/20;
   const position=prepareDeliveryMotion(d,catheter,dt,i<120?0:-1,110,release);
   const sampler=createStentGraftContacts(surface,state);
   const source={...state,wallSamples:[sampler],graftRevision:surface.revision,graftRecovery:sampler.recovery};
   const tools=state.materials.map(m=>({...m.spec,rotation:0,insertion:m.spec.id==='catheter'?catheter.progress:110,...(m.spec.id==='catheter'?{deliveryExposureMm:catheter.deliveryExposureMm}:{})}));
   const it=advanceSharedAxis(source,{wire:0,catheter:0},dt,tools,{maxIterations:100,forceTolerance:1e-5,lengthTolerance:1e-5});let next;do{next=it.next();}while(!next.done);
   assert.ok(next.value.state&&next.value.result.converged,JSON.stringify({i,result:next.value.result}));
   state=next.value.state;contact||=state.graftContactStats?.contacts>0;
   d.position=d.target=position;advanceRelease(d,dt,release);
   assert.ok(state.positions.every(p=>p.every(Number.isFinite)));
  }
  assert.ok(contact,'finite-radius cloth contact was exercised');
  assert.equal(state.materials.find(m=>m.spec.id==='catheter').spec.insertion,0);
  assert.ok(state.positions.at(-1)[0]>100,'guidewire remains through the open proximal end');
 }finally{geometry.dispose();}
});


test('the application commit contract keeps the released implant fixed while the physical core retracts',()=>{
 const {system,device:d}=previewFixture();
 const catheter=proxy(d.position);
 try {
  const step=(release,advance=0)=>{
   const position=prepareDeliveryMotion(d,catheter,1/30,advance,600,release);
   system.updateAccess('right',1/30,null,{deviceId:d.id,release,advance,mechanicalPosition:position,mechanicalRotation:0});
   assert.ok(Math.abs(catheter.progress-deliveryMechanicalExtent(d).insertion)<1e-8);
  };
  for(let i=0;i<600&&d.phase!=='deployed';i++)step({sheath:1,tip:true});
  assert.equal(d.phase,'deployed');
  const anchor=d.implantPosition,positions=d.parts.map(p=>Array.from(p.mesh.geometry.attributes.position.array));
  for(let i=0;i<400;i++)step('nose');
  assert.equal(deliveryNoseState(d).remaining,0);
  assert.ok(d.position-catheter.progress>60);
  for(let i=0;i<60;i++)step(null,-1);
  assert.equal(d.implantPosition,anchor);
  assert.deepEqual(d.parts.map(p=>Array.from(p.mesh.geometry.attributes.position.array)),positions);
 }finally{system.dispose();}
});

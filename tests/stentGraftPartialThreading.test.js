import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {packedLayout} from '../src/devices/stentGraftReleaseShape.js';
import {fullyOpenDistance} from '../src/devices/stentGraftDeployment.js';
import {createSharedAxisNative,sharedAxisOuterIntervals,sharedAxisOuterMaterialAt} from '../src/physics/kirchhoffSharedAxisNative.js';

test('folded main body is threaded through the ipsilateral lumen, not between both limbs',()=>{
 const ipsi=packedLayout('body',1),gate=packedLayout('body',2);
 assert.equal(packedLayout('body',2,1,'left').lateral,-gate.lateral);
 assert.equal(ipsi.lateral,0);assert.ok(ipsi.radius>.8);
 assert.ok(gate.lateral-gate.radius>ipsi.radius);
});

test('collision envelope splits cover, exposed core and guidewire at exact material positions',()=>{
 const state=createSharedAxisNative({tools:[{id:'wire',insertion:80},{id:'catheter',insertion:79.9999,type:'stentgraft-delivery',radius:3,deliveryExposureMm:12.3}],spacing:5});
 const catheter=state.materials[1],edge=state.coordinates.findIndex((x,i)=>x<67.6999&&state.coordinates[i+1]>67.6999);
 const intervals=sharedAxisOuterIntervals(state,edge);
 assert.equal(intervals.length,2);assert.equal(intervals[0].material.body.radius,3);assert.equal(intervals[1].material.body.radius,.8);
 assert.ok(Math.abs(state.coordinates[edge]+intervals[0].end*5-67.6999)<1e-9);
 assert.equal(sharedAxisOuterMaterialAt(state,edge,0).body.radius,3);assert.equal(sharedAxisOuterMaterialAt(state,edge,1).body.radius,.8);
 assert.equal(catheter.body.radius,3,'mechanical material is not mutated');
 const last=sharedAxisOuterIntervals(state,catheter.last-1);
 assert.equal(last[0].material.body.radius,.8);assert.equal(last[1].material.spec.id,'wire');
});

for(const side of ['right','left'])test(`${side}: 79.6 mm release has visible ipsilateral contacts before full radial expansion`,()=>{
 const {system,device:d,sources}=previewFixture(side);
 try {
  system.updateAccess(side,79.6/12,null,{deviceId:d.id,release:'sheath'});
  system.updateAccess(side,1,null,{deviceId:d.id,release:'tip'});
  assert.ok(fullyOpenDistance(d)<d.parts[1].releaseOffset,'old contact threshold has not reached the ipsilateral limb');
  const surface=system.mechanicalSurfaceForAccess(side),branch=surface.ownedBranches[0];
  assert.ok(branch&&branch.centers.length>=2);
  assert.deepEqual(branch.positions,Array.from(d.parts[1].mesh.geometry.attributes.position.array.slice(0,branch.positions.length)));
  const original=branch.positions.slice();
  system.updateAccess(side,.1,null,{deviceId:d.id,release:'sheath'});
  const changed=system.mechanicalSurfaceForAccess(side);assert.notEqual(changed.revision,surface.revision);
  assert.deepEqual(branch.positions,original,'previous solver snapshot stays immutable');
  const target=Array.from(d.parts[1].target),oldFold=d.parts[1].folded.at(-1).clone(),exposedFold=d.parts[1].folded[0].clone();
  sources[side].nodes.forEach(p=>p.x+=.5);
  system.updateAccess(side,1/60,null,{deviceId:d.id});
  assert.ok(d.parts[1].folded.at(-1).distanceTo(oldFold)>.49,'covered part follows the delivery system even after tip detachment');
  assert.deepEqual(d.parts[1].folded[0],exposedFold,'exposed fabric no longer follows the detached shaft');
  assert.deepEqual(Array.from(d.parts[1].target),target,'expanded rest shape stays implanted');
 }finally{system.dispose();}
});

for(const side of ['right','left'])test(`${side}: partial ipsilateral cloth reacts and permits withdrawal at 72 percent deployment`,async()=>{
 const {createStentGraftContacts}=await import('../src/devices/stentGraftContacts.js');
 const {advanceSharedAxis}=await import('../src/physics/kirchhoffSharedAxisAppSystem.js');
 const {deliveryMechanicalExtent}=await import('../src/devices/stentGraftDeliveryMechanics.js');
 const {graftBranchContactAt}=await import('../src/devices/stentGraftBranchContact.js');
 const {system,device:d}=previewFixture(side);
 try {
  system.updateAccess(side,79.6/12,null,{deviceId:d.id,release:'sheath'});
  system.updateAccess(side,1,null,{deviceId:d.id,release:'tip'});
  const extent=deliveryMechanicalExtent(d);
  let state=createSharedAxisNative({tools:[{id:'wire',insertion:d.position+20},{id:'catheter',insertion:extent.insertion,type:'stentgraft-delivery',radius:3,deliveryExposureMm:extent.exposedLength}],adaptiveMesh:true,spacing:2,samplePosition:s=>d.deliveryPath.sample(s).toArray()});
  let reacted=false;
  for(let step=0;step<140;step++) {
   const surface=system.mechanicalSurfaceForAccess(side),sample=createStentGraftContacts(surface,state);
   const source={...state,wallSamples:[sample],graftRevision:surface.revision,graftRecovery:sample.recovery};
   const tools=state.materials.map(m=>({...m.spec,rotation:0,insertion:m.spec.id==='catheter'?extent.insertion-Math.min(40,Math.max(0,step-79))*.2:m.spec.insertion}));
   const it=advanceSharedAxis(source,{wire:0,catheter:0},1/60,tools,{maxIterations:160,forceTolerance:1e-4,lengthTolerance:1e-3});let next;do{next=it.next();}while(!next.done);
   assert.ok(next.value.state&&next.value.result.converged,JSON.stringify({step,result:next.value.result}));
   state=next.value.state;reacted||=state.graftContactStats?.contacts>0;
   surface.commitContactPatches(sample.contactPatches??[]);
   system.updateAccess(side,1/60,{nodes:state.positions.map(p=>new THREE.Vector3(...p).add(new THREE.Vector3(...state.origin))),coordinate:i=>state.coordinates[i],catheterMm:0},{deviceId:d.id,mechanicalPosition:tools.find(t=>t.id==='catheter').insertion});
  }
  assert.ok(reacted,'partial cloth must exert forces before the old contact threshold');
  const branches=system.mechanicalSurfaceForAccess(side).ownedBranches;
  let interior=0;
  const samples=[];
  for(let e=0;e<state.positions.length-1;e++) {
   const a=state.positions[e],b=state.positions[e+1],count=Math.max(1,Math.ceil(Math.hypot(...a.map((v,k)=>b[k]-v))));
   for(let j=0;j<count;j++)samples.push({node:j===0,p:a.map((v,k)=>v+(b[k]-v)*j/count),coordinate:state.coordinates[e]+(state.coordinates[e+1]-state.coordinates[e])*j/count});
  }
  for(const {p,coordinate,node} of samples) {
   const part=d.parts[1];
   // After sliding, rod material coordinates no longer identify a fixed
   // section of cloth. Check spatial containment at every rod point.
   const q=graftBranchContactAt(branches,p.map((v,k)=>v+state.origin[k]),state.materials[0].body.radius);
   // Portal reactions fade over 2 mm so the open end remains free to slide.
   const world=p.map((v,k)=>v+state.origin[k]);
   if(q&&[q.axisStart.reduce((v,n,k)=>v+n*(world[k]-q.start[k]),0),q.axisEnd.reduce((v,n,k)=>v+n*(q.end[k]-world[k]),0)].every(distance=>distance>=2)){interior++;assert.ok(!q.outside&&q.gap>(node?-.05:-state.materials[0].body.radius),JSON.stringify({s:coordinate,partStart:d.implantPosition-d.parts[1].releaseOffset,partEnd:d.implantPosition-d.parts[1].releaseOffset-d.parts[1].path.coordinates[branches[0].centers.length-1],q}));}
  }
  assert.ok(interior>=2,'check actual points in the exposed ipsilateral lumen');
 }finally{system.dispose();}
});

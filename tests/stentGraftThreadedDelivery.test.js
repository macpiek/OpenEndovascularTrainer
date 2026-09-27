import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {graftBranchContactAt} from '../src/devices/stentGraftBranchContact.js';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {createStentGraftContacts} from '../src/devices/stentGraftContacts.js';
import {createSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {defineKirchhoffMaterialProfile} from '../src/physics/kirchhoffMaterialProfile.js';
const beam=defineKirchhoffMaterialProfile({id:'ipsilateral-test',sampleEI1:()=>1e3,sampleGJ:()=>1e3});
for(const side of ['right','left'])test(`${side}: stiff delivery assembly recovers to the long lumen with full-radius clearance`,()=>{
 const {system,device:d}=previewFixture(side);
 try {
  for(let i=0;i<700&&d.phase!=='deployed';i++)system.updateAccess(side,1/30,null,{deviceId:d.id,release:{sheath:1,tip:true}});
  const own=system.mechanicalSurfaceForAccess(side),contra=d.parts[2];
  const axis=contra.points[0].clone().sub(contra.points.at(-1)).normalize(),origin=contra.points.at(-1).clone().addScaledVector(axis,-35);
  let state=createSharedAxisNative({startCoordinate:-20,tools:[{id:'wire',insertion:100,type:beam,radius:.4445},{id:'catheter',insertion:65,type:'stentgraft-delivery',radius:3,deliveryExposureMm:40}],adaptiveMesh:true,spacing:2,samplePosition:s=>origin.clone().addScaledVector(axis,s).toArray()});
  for(let i=0;i<180;i++) {
   const sampler=createStentGraftContacts(own,state);
   state={...state,wallSamples:[sampler],graftRevision:own.revision,graftRecovery:sampler.recovery};
   const tools=state.materials.map(m=>({...m.spec,rotation:0}));
   const iterator=advanceSharedAxis(state,{wire:0,catheter:0},1/60,tools,{maxIterations:160,forceTolerance:1e-4,lengthTolerance:1e-3});
   let next;do{next=iterator.next();}while(!next.done);
   const {result}=next.value;assert.ok(next.value.state&&result.converged,JSON.stringify({side,i,result}));
   state=next.value.state;
  }
  const mid=d.parts[1].path.sample(d.parts[1].path.length*.65);
  const candidates=state.positions.filter(p=>Math.abs(new THREE.Vector3(...p).sub(mid).dot(axis))<5);
  assert.ok(candidates.length>0);
  for(const p of candidates) {
   const q=graftBranchContactAt(own.ownedBranches,p,3);
   assert.ok(q&&!q.outside&&q.gap>-.1,JSON.stringify({p,q}));
  }
  for(let i=0;i<30;i++) {
   const sampler=createStentGraftContacts(own,state);
   const source={...state,wallSamples:[sampler],graftRevision:own.revision,graftRecovery:sampler.recovery};
   const tools=state.materials.map(m=>({...m.spec,rotation:0,insertion:m.spec.insertion+(i<15?.2:-.2)}));
   const iterator=advanceSharedAxis(source,{wire:0,catheter:0},1/60,tools,{maxIterations:160,forceTolerance:1e-4,lengthTolerance:1e-3});
   let next;do{next=iterator.next();}while(!next.done);
   assert.ok(next.value.state&&next.value.result.converged,JSON.stringify({side,i,result:next.value.result}));
   state=next.value.state;
  }
 }finally{system.dispose();}
});

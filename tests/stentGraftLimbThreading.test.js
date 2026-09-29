import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {compliantGraftSurface} from '../src/devices/stentGraftCompliance.js';
import {DevicePath} from '../src/devices/stentGraftPaths.js';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {graftBranchContactAt} from '../src/devices/stentGraftBranchContact.js';
import {createSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {createStentGraftContacts} from '../src/devices/stentGraftContacts.js';
import {prepareDeliveryMotion,deliveryMechanicalExtent} from '../src/devices/stentGraftDeliveryMechanics.js';

for(const side of ['left','right'])test(`${side}: added limb retains its own wire during release and relinquishes it after withdrawal`,()=>{
 const {system,device:d}=previewFixture(side,'limb');
 try {
  system.updateAccess(side,4,null,{deviceId:d.id,release:'sheath'});
  const part=d.parts[0],surface=system.mechanicalSurfaceForAccess(side),branch=surface.ownedBranches[0];
  assert.ok(branch,'separately delivered limb must keep its wire on the lumen side of the cloth');
  assert.equal(surface.lumenSections.length,0,'contact uses the moving cloth rather than a target-axis guide');
  assert.ok(branch.transitioning);
  assert.deepEqual(branch.positions,Array.from(part.mesh.geometry.attributes.position.array.slice(0,branch.positions.length)));
  assert.equal(system.mechanicalSurfaceForAccess(side==='left'?'right':'left').ownedBranches.length,0);
  const oldPositions=branch.positions.slice();
  system.updateAccess(side,100,null,{deviceId:d.id,release:'sheath'});
  assert.equal(d.phase,'deployed');assert.equal(d.limbReleased,true);
  assert.deepEqual(branch.positions,oldPositions,'in-flight contact snapshots are immutable');
  assert.equal(system.mechanicalSurfaceForAccess(side).ownedBranches.length,1,'detaching the introducer does not unthread the wire');
  system.removeDelivery(side);
  const entry=d.implantPosition-part.path.length;
  const source=length=>({nodes:[new THREE.Vector3(),new THREE.Vector3(0,length,0)],coordinate:i=>i*length,catheterMm:0});
  system.updateAccess(side,1/60,source(entry-3));
  assert.equal(system.mechanicalSurfaceForAccess(side).ownedBranches.length,0);
  system.updateAccess(side,1/60,source(d.implantPosition+20));
  assert.equal(system.mechanicalSurfaceForAccess(side).ownedBranches.length,0,'later tools can enter either port normally');
 }finally{system.dispose();}
});

for(const side of ['left','right'])test(`${side}: opening an added limb keeps the wire inside and permits delivery withdrawal`,()=>{
 const {system,device:d}=previewFixture(side,'limb');
 try {
  system.updateAccess(side,4,null,{deviceId:d.id,release:'sheath'});
  const extent=deliveryMechanicalExtent(d);
  let state=createSharedAxisNative({tools:[{id:'wire',insertion:d.position+20},{id:'catheter',insertion:extent.insertion,type:'stentgraft-delivery',radius:3,deliveryExposureMm:extent.exposedLength}],adaptiveMesh:true,spacing:2,samplePosition:s=>d.deliveryPath.sample(s).toArray()});
  const checkInside=state=>{
   const branches=system.mechanicalSurfaceForAccess(side).ownedBranches;
   let interior=0;
   for(const p of state.positions) {
    const world=p.map((v,k)=>v+state.origin[k]),q=graftBranchContactAt(branches,world,.4445);
    if(!q)continue;
    const start=q.axisStart.reduce((sum,v,k)=>sum+v*(world[k]-q.start[k]),0),end=q.axisEnd.reduce((sum,v,k)=>sum+v*(q.end[k]-world[k]),0);
    if(start<2||end<2)continue;
    interior++;assert.ok(!q.outside&&q.gap>-.05,JSON.stringify({world,q}));
   }
   assert.ok(interior>=2,'wire remains inside the actual released lumen');
  };
  let contacts=0;
  // Complete the 360 commanded steps, then allow up to two seconds of
  // bounded spring settling, still solving and checking actual lumen contacts.
  for(let step=0;step<480;step++) {
   const surface=system.mechanicalSurfaceForAccess(side),sample=surface?createStentGraftContacts(surface,state):null;
   const source={...state,wallSamples:sample?[sample]:[],graftRevision:surface?.revision??0,graftRecovery:sample?.recovery??[]};
   const exposed=deliveryMechanicalExtent(d);
   const tools=state.materials.map(m=>({...m.spec,rotation:0,...(m.spec.id==='catheter'?{insertion:exposed.insertion,deliveryExposureMm:exposed.exposedLength}:{})}));
   if(step>=180&&step<360)tools.find(m=>m.id==='catheter').insertion-=.2;
   const it=advanceSharedAxis(source,{wire:0,catheter:0},1/60,tools,{maxIterations:160,forceTolerance:1e-4,lengthTolerance:1e-3});let next;do{next=it.next();}while(!next.done);
   assert.ok(next.value.state&&next.value.result.converged,JSON.stringify({step,result:next.value.result}));
   state=next.value.state;contacts+=state.graftContactStats?.contacts??0;
   surface?.commitContactPatches(sample?.contactPatches??[]);
   const catheter=tools.find(m=>m.id==='catheter');
   system.updateAccess(side,1/60,{nodes:state.positions.map(p=>new THREE.Vector3(...p).add(new THREE.Vector3(...state.origin))),coordinate:i=>state.coordinates[i],catheterMm:0},{deviceId:d.id,release:step<180?'sheath':null,mechanicalPosition:catheter.insertion});
   if(step>=180)checkInside(state);
   if(step>=359&&d.phase==='deployed')break;
  }
  assert.equal(d.phase,'deployed',JSON.stringify({cover:d.sheathWithdrawal,openings:d.parts[0].scaffoldRings.map(r=>r.opening)}));
  assert.ok(contacts>0,'opening cloth transfers forces to the wire');
 }finally{system.dispose();}
});

for(const type of ['body','limb'])test(`${type}: preparing a rejected delivery step cannot advance sewn ring springs`,()=>{
 const {system,device:d}=previewFixture('right',type);
 try {
  system.updateAccess('right',4,null,{deviceId:d.id,release:'sheath'});
  const rings=()=>d.parts.map(p=>p.scaffoldRings.map(r=>({freeTime:r.freeTime,opening:r.opening,openingRate:r.openingRate})));
  const before=rings(),withdrawal=d.sheathWithdrawal;
  const proxy={progress:d.position,setType(){},setStiffnessScales(){},advance(command,dt,wire,speed){this.progress+=command*dt*speed;}};
  for(let trial=0;trial<5;trial++)prepareDeliveryMotion(d,proxy,1/60,0,d.position+30,'sheath');
  assert.deepEqual(rings(),before,'trial retries must not alter committed ring states');
  assert.equal(d.sheathWithdrawal,withdrawal);
 }finally{system.dispose();}
});

for(const bodySide of ['right','left'])test(`${bodySide}: connecting a contralateral extension preserves separate lumens for both delivery wires`,()=>{
 const side=bodySide==='right'?'left':'right';
 const {system,device:body}=previewFixture(bodySide);
 try {
  system.updateAccess(bodySide,100,null,{deviceId:body.id,release:{sheath:1,tip:true}});
  assert.equal(body.phase,'deployed');
  // Cannulate the gate before delivering the extension; the original atlas
  // centreline lies between the two outlets and is not a cannulated wire.
  const gate=system.availableGate(side),wire=system.getPath(side),near=wire.nearest(gate.docking);
  const path=new DevicePath([...wire.section(0,Math.max(0,near.s-20),5),gate.docking,
   body.parts[2].points[0],...body.parts[0].points.slice().reverse(),wire.points.at(-1)]);
  system.updateAccess(side,0,{nodes:path.points,coordinate:i=>path.coordinates[i],catheterMm:0});
  assert.ok(system.load(side,'limb').ok);
  assert.ok(system.positionAtTarget(side).ok);
  const limb=system.accesses[side].device;limb.position=limb.target;
  assert.ok(system.deploy(side).ok);assert.equal(limb.parentId,body.id);
  system.updateAccess(side,4,null,{deviceId:limb.id,release:'sheath'});
  const own=system.mechanicalSurfaceForAccess(side),parent=system.mechanicalSurfaceForAccess(bodySide);
  assert.equal(own.ownedBranches.length,1);assert.equal(parent.ownedBranches.length,1);
  assert.deepEqual(own.ownedBranches[0].positions,Array.from(limb.parts[0].mesh.geometry.attributes.position.array.slice(0,own.ownedBranches[0].positions.length)));
  assert.notDeepEqual(own.ownedBranches[0].positions,parent.ownedBranches[0].positions);
  system.updateAccess(side,100,null,{deviceId:limb.id,release:'sheath'});
  assert.equal(limb.phase,'deployed');assert.equal(system.surface.sealed,true);
  assert.equal(system.mechanicalSurfaceForAccess(side).ownedBranches.length,1,'connection and final release retain the extension wire');
  assert.equal(system.mechanicalSurfaceForAccess(bodySide).ownedBranches.length,1);
 }finally{system.dispose();}
});

test('wire inside the opening extension cannot lift fabric as an exterior tool beside the parent',()=>{
 const {system,device:d}=previewFixture('left','limb');
 try {
  system.updateAccess('left',4,null,{deviceId:d.id,release:'sheath'});
  const surface={...system.mechanicalSurfaceForAccess('left'),contains:()=>false};
  const part=d.parts[0],row=5,center=new THREE.Vector3();
  const positions=part.mesh.geometry.attributes.position;
  for(let j=0;j<part.sides;j++)center.add(new THREE.Vector3().fromBufferAttribute(positions,row*part.sides+j));
  center.divideScalar(part.sides);
  const wall=new THREE.Vector3().fromBufferAttribute(positions,row*part.sides);
  const point=wall.clone().addScaledVector(center.clone().sub(wall).normalize(),.2);
  assert.equal(graftBranchContactAt(surface.ownedBranches,point.toArray(),.4445)?.outside,false);
  const vessel=()=>({gap:1,jacobian:[0,0,0,0,0,0]});
  vessel.vesselField={querySphere:()=>({inside:true,signedDistance:.1})};
  const state=createSharedAxisNative({tools:[{id:'wire',insertion:2,radius:.4445}],spacing:1,wallSamples:[vessel],samplePosition:s=>point.clone().addScaledVector(new THREE.Vector3(0,1,0),s*.01).toArray()});
  assert.equal(compliantGraftSurface(surface,state).patches.length,0);
 }finally{system.dispose();}
});

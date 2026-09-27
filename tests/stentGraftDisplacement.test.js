import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {displaceGraftPoint,graftDisplacementSampler} from '../src/devices/stentGraftCompliance.js';

test('local cloth displacement index preserves exact overlapping patch response and boundaries',()=>{
 const patches=Array.from({length:50},(_,i)=>({point:[Math.sin(i)*20,Math.cos(i)*20,i/2],normal:new THREE.Vector3(Math.sin(i*.4),Math.cos(i*.4),.5).normalize().toArray(),width:3+i%3,depth:.1+i%7/10}));
 const sample=graftDisplacementSampler(patches);
 const points=Array.from({length:3000},(_,i)=>new THREE.Vector3(Math.sin(i*.71)*26,Math.cos(i*.37)*26,(i%71)/2-4));
 for(const patch of patches)for(const sign of [-1,1])points.push(new THREE.Vector3(...patch.point).addScaledVector(new THREE.Vector3(...patch.normal),sign*1.5));
 for(const p of points)assert.deepEqual(sample(p.clone()).toArray(),displaceGraftPoint(p.clone(),patches).toArray());
});

test('a threaded wire inside the graft is not classified as an exterior indentation',async()=>{
 const {MeshBVH}=await import('three-mesh-bvh');
 const {createSharedAxisNative}=await import('../src/physics/kirchhoffSharedAxisNative.js');
 const {compliantGraftSurface}=await import('../src/devices/stentGraftCompliance.js');
 const geometry=new THREE.CylinderGeometry(3,3,80,24,1,true).rotateZ(-Math.PI/2).translate(30,0,0);
 geometry.boundsTree=new MeshBVH(geometry);geometry.computeBoundingBox();
 const surface={geometry,bounds:geometry.boundingBox,ownedBranches:[{}],contains:p=>p.x>=-10&&p.x<=70&&Math.hypot(p.y,p.z)<3};
 const state=createSharedAxisNative({tools:[{id:'wire',insertion:60}],spacing:2,samplePosition:s=>[s,2.8,0]});
 state.wallSamples=[{vesselField:{querySphere(){throw Error('interior threaded wire must not request exterior wall indentation');}}}];
 try {const result=compliantGraftSurface(surface,state);assert.equal(result.surface,surface);assert.deepEqual(result.patches,[]);}finally{geometry.dispose();}
});

test('relaxing cloth preserves the guidewire radius even when its axis does not cross the rest sheet',async()=>{
 const {MeshBVH}=await import('three-mesh-bvh');
 const {createSharedAxisNative}=await import('../src/physics/kirchhoffSharedAxisNative.js');
 const {compliantGraftSurface}=await import('../src/devices/stentGraftCompliance.js');
 const {createSharedAxisSegmentContact}=await import('../src/physics/kirchhoffSharedAxisSegmentContact.js');
 const geometry=new THREE.PlaneGeometry(10,10,20,20).rotateY(Math.PI).translate(0,0,.7);
 geometry.boundsTree=new MeshBVH(geometry);geometry.computeBoundingBox();
 const patches=[{point:[0,0,.7],normal:[0,0,-1],depth:.6,width:4}];
 const state=createSharedAxisNative({tools:[{id:'wire',insertion:2,radius:.4445}],spacing:1,samplePosition:s=>[s-1,0,.4445]});
 // The vessel is far enough away to generate no new indentation. The saved
 // indentation must nevertheless remain until the finite-radius wire clears.
 state.wallSamples=[{vesselField:{querySphere:()=>({inside:true,signedDistance:10})}},{graftSurface:true,contactPatches:patches}];
 const surface={geometry,bounds:geometry.boundingBox};let result;
 try {
  result=compliantGraftSurface(surface,state);
  const query=createSharedAxisSegmentContact(result.surface.geometry);
  for(let i=0;i<state.positions.length-1;i++)assert.ok(query.query(state.positions[i],state.positions[i+1],.4445).distance>=.4445-1e-5);
  assert.ok(result.patches.length>0,'retain indentation while the radius still occupies its space');
 }finally{if(result?.surface.geometry!==geometry)result?.surface.geometry.dispose();geometry.dispose();}
});

import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import crypto from 'node:crypto';
import * as THREE from 'three';
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader.js';
import {MeshBVH} from 'three-mesh-bvh';
import {transformAortaGeometry} from '../src/aortaTransform.js';
import {generateVessel} from '../src/vesselGeometry.js';
import {createSharedAxisInsideContinuation} from '../src/physics/kirchhoffSharedAxisInsideContinuation.js';
import {createSharedAxisVesselWitness} from '../src/physics/kirchhoffSharedAxisVesselWitnesses.js';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';
import {VesselContactField} from '../src/physics/collision/vesselContactField.js';
const report=JSON.parse(fs.readFileSync(new URL('../res/Aorta_plain.outlet-closures.json',import.meta.url)));
const root=process.env.OET_ANATOMY_ROOT?new URL(process.env.OET_ANATOMY_ROOT):new URL('../',import.meta.url);
const bytes=fs.readFileSync(new URL('res/Aorta_plain.stl',root));
const g=new STLLoader().parse(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
transformAortaGeometry(g,generateVessel(140,0).vessel);g.boundsTree=new MeshBVH(g);
test.after(()=>g.dispose());

test('closure manifest matches the shipped model and records the wall provenance',()=>{
 assert.equal(crypto.createHash('sha256').update(bytes).digest('hex'),report.outputSha256);
 assert.equal(bytes.readUInt32LE(80),report.triangles);
 if(report.deformation) {
  assert.equal(report.deformation.inputSha256,'5efc1bcb3b6f18f68567ad5edda8cf3f01e6c27a948e80f2b5eefa48468812e3');
  assert.equal(report.deformation.inputTriangles,report.originalTriangles+report.addedTriangles);
  assert.equal(report.deformation.outputSha256,report.surfaceRepair?.inputSha256??report.outputSha256);
  assert.equal(report.deformation.outputTriangles,report.surfaceRepair?.inputTriangles??report.triangles);
  if(report.surfaceRepair) {
   assert.equal(report.surfaceRepair.outputSha256,report.outputSha256);
   assert.equal(report.surfaceRepair.outputTriangles,report.triangles);
  }
 } else {
  const original=Buffer.from(bytes.subarray(0,84+50*report.originalTriangles));original.writeUInt32LE(report.originalTriangles,80);
  assert.equal(crypto.createHash('sha256').update(original).digest('hex'),report.sourceSha256);
 }
 assert.equal(report.caps.length,41);assert.equal(report.terminals.filter(t=>t.closed).length,41);
});

test('every closed outlet blocks axial passage across its center and sampled near-rim interior',()=>{
 const ray=new THREE.Ray();let probes=0;
 for(const cap of report.caps) {
  if(cap.surfaceProbes) {
   assert.equal(cap.surfaceProbes.length,97);
   for(const probe of cap.surfaceProbes) {
    const normal=new THREE.Vector3(...probe.normal);
    assert.ok(Math.abs(normal.length()-1)<1e-6);
    ray.origin.fromArray(probe.point).addScaledVector(normal,-1);ray.direction.copy(normal);
    const hit=g.boundsTree.raycastFirst(ray,THREE.DoubleSide,0,1+cap.thickness+.05);
    assert.ok(hit,`Open deformed outlet ${cap.id}/${probes}`);
    // The finite triangles approximate a curved, transported cap surface.
    assert.ok(hit.distance<=1.05,`Late deformed wall ${cap.id}: ${hit.distance}`);
    probes++;
   }
   continue;
  }
  const center=new THREE.Vector3().fromArray(cap.center),normal=new THREE.Vector3().fromArray(cap.normal);
  const u=new THREE.Vector3(Math.abs(normal.x)<.8?1:0,Math.abs(normal.x)<.8?0:1,0).cross(normal).normalize(),v=normal.clone().cross(u);
  // The minimum measured lumen radius gives a disk wholly inside this cap.
  for(const fraction of [0,.25,.75,.95])for(let k=0;k<(fraction?32:1);k++) {
   const angle=k/32*2*Math.PI,p=center.clone().addScaledVector(u,fraction*cap.minRadius*Math.cos(angle)).addScaledVector(v,fraction*cap.minRadius*Math.sin(angle));
   ray.origin.copy(p).addScaledVector(normal,-1);ray.direction.copy(normal);
   const hit=g.boundsTree.raycastFirst(ray,THREE.DoubleSide,0,1+cap.thickness+.01);
   assert.ok(hit,`Open outlet ${cap.id}/${fraction}/${k}`);
   assert.ok(hit.distance<=1.001,`Late wall ${cap.id}/${fraction}/${k}: ${hit.distance}`);probes++;
  }
 }
 assert.equal(probes,41*97);
});

test('inside continuation cannot relabel an escaped root point as lumen after closure',()=>{
 const root=report.caps.find(c=>c.id===2019),center=new THREE.Vector3().fromArray(root.center),n=new THREE.Vector3().fromArray(root.normal);
 const proof=createSharedAxisInsideContinuation(g),start=center.clone().addScaledVector(n,-2),end=center.clone().addScaledVector(n,2);
 const hit={};g.boundsTree.closestPointToPoint(start,hit);
 proof.remember(0,start.toArray(),hit.distance);
 assert.equal(proof.contains(0,end.toArray()),false);assert.equal(proof.stats.blocked,1);
});

test('the root closure supplies inward wall constraints for both wire and catheter radii',()=>{
 const root=report.caps.find(c=>c.id===2019),center=new THREE.Vector3().fromArray(root.center),n=new THREE.Vector3().fromArray(root.normal);
 const position=center.clone().addScaledVector(n,-.2),hit={};g.boundsTree.closestPointToPoint(position,hit);
 const field={fallbackGeometry:g},definition={kind:'wall',edge:0,witness:{face:hit.faceIndex,t:1},dofs:[0,1,2,3,4,5]};
 const witness=createSharedAxisVesselWitness(field,definition);
 for(const radius of [.889/2,1.667/2]) {
  const contact=witness.evaluate({a:center.clone().addScaledVector(n,-2).toArray(),b:position.toArray(),radius,state:{origin:[0,0,0]}});
  assert.ok(Math.abs(contact.gap-(.2-radius))<1e-4);
  assert.ok(contact.gap<0,'Tool overlap must activate the wall constraint');
  assert.ok(new THREE.Vector3().fromArray(contact.jacobian,3).dot(n)<-.999,'Wall reaction points back into the vessel');
 }
});

test('the rebuilt collision field sees the root cap for both tool diameters',()=>{
 const buffer=fs.readFileSync(new URL('res/Aorta_plain.collision.bin',root));
 const asset=decodeCollisionAsset(buffer.buffer.slice(buffer.byteOffset,buffer.byteOffset+buffer.byteLength));
 assert.equal(asset.metadata.source.stlSha256,report.outputSha256);
 const field=new VesselContactField(asset,{fallbackGeometry:g,bvhValidationDistance:.02,capsuleBvhValidation:-.1});
 const cap=report.caps.find(c=>c.id===2019),normal=new THREE.Vector3().fromArray(cap.normal),p=new THREE.Vector3().fromArray(cap.center).addScaledVector(normal,-.2);
 const tangent=new THREE.Vector3(1,0,0).cross(normal).normalize().multiplyScalar(.1),b=p.clone().add(tangent);
 for(const radius of [.889/2,1.667/2]) {
  const contact=field.queryCapsuleSoA([p.x,b.x],[p.y,b.y],[p.z,b.z],[radius,radius],0,null,-1,true,false,-1,false,.1,1,true,true);
  assert.ok(Math.abs(contact.signedDistance-.2)<1e-4);
  assert.ok(Math.abs(contact.signedGap-(.2-radius))<1e-4);
 }
});

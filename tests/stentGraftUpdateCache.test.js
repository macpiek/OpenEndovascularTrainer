import test from 'node:test';
import assert from 'node:assert/strict';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {updateScaffold} from '../src/devices/stentGraftScaffold.js';
import {exposedGraftFaces,graftFaceExposed} from '../src/devices/stentGraftDeployment.js';

for(const type of ['body','limb'])test(`${type}: delivery cache follows motion, wire deformation, rotation, and hide/show`,()=>{
 const {system,device:d,sources}=previewFixture('right',type,false);
 try {
  system.refreshDelivery('right');const initial=d.deliveryMesh.geometry,nose=d.noseMarker.geometry;
  system.refreshDelivery('right');assert.equal(d.deliveryMesh.geometry,initial);assert.equal(d.noseMarker.geometry,nose);
  d.position-=1;system.refreshDelivery('right');assert.notEqual(d.deliveryMesh.geometry,initial);
  const moving=d.deliveryMesh.geometry;const node=sources.right.nodes.at(-2);node.x+=2;
  system.refreshDelivery('right');assert.notEqual(d.deliveryMesh.geometry,moving,'in-place rod motion invalidates cache');
  const marker=d.rotationMarker.position.clone();d.deliveryRotation+=.3;d.graftRotation+=.3;
  system.refreshDelivery('right');assert.ok(marker.distanceTo(d.rotationMarker.position)>0);
  const pos=d.position;d.position=0;system.refreshDelivery('right');assert.equal(d.noseMarker.visible,false);
  d.position=pos;system.refreshDelivery('right');assert.equal(d.noseMarker.visible,true);assert.equal(d.foldedPreview[0].rings.visible,true);
 }finally{system.dispose();}
});

test('batched collision exposure matches individual face queries through opening and recapture',()=>{
 const {system,device:d}=previewFixture();
 try {
  for(let k=0;k<30;k++) {
   system.updateAccess('right',d.sheathTravel/12/20,null,{deviceId:d.id,release:k<20?'sheath':'resheath'});
   assert.deepEqual(exposedGraftFaces(d),d.contactFaces.filter(f=>graftFaceExposed(d,f)));
  }
 }finally{system.dispose();}
});

test('stationary capture reuses released cloth but moving the latch still changes it',()=>{
 const {system,device:d}=previewFixture();
 try {
  system.updateAccess('right',d.sheathTravel*.45/12,null,{deviceId:d.id,release:'sheath'});
  for(let i=0;i<30;i++)system.updateAccess('right',1/30);
  const part=d.parts[0],p=part.mesh.geometry.attributes.position,version=p.version,pose=p.array.slice();
  for(let i=0;i<10;i++)system.updateAccess('right',1/30);
  assert.equal(p.version,version);assert.deepEqual(p.array,pose);
  system.updateAccess('right',1/30,null,{deviceId:d.id,mechanicalPosition:d.position+.25});
  assert.ok(p.version>version);assert.notDeepEqual(p.array,pose);
 }finally{system.dispose();}
});

test('unchanged scaffold avoids GPU buffer uploads and a cloth edit invalidates it',()=>{
 const {system,device:d}=previewFixture();
 try {
  system.updateAccess('right',3,null,{deviceId:d.id,release:'sheath'});
  for(let i=0;i<40;i++)system.updateAccess('right',1/30);
  const p=d.parts[0],metal=p.rings.geometry.attributes.instanceStart.data,markers=p.markers.geometry.attributes.instanceStart.data;
  const before=metal.version,markerBefore=markers.version,shape=metal.array.slice();
  for(let i=0;i<10;i++)updateScaffold(p,{constrain:false});
  assert.equal(metal.version,before);assert.equal(markers.version,markerBefore);
  assert.deepEqual(metal.array,shape);
  const cloth=p.mesh.geometry.attributes.position;
  cloth.setX(0,cloth.getX(0)+.05);cloth.needsUpdate=true;
  updateScaffold(p,{constrain:false});
  assert.ok(metal.version>before);assert.notDeepEqual(metal.array,shape);
 }finally{system.dispose();}
});

test('sewn assembly reuses private buffers while released geometry changes',()=>{
 const {system,device:d}=previewFixture();
 try {
  system.updateAccess('right',d.sheathTravel*.65/12,null,{deviceId:d.id,release:'sheath'});
  const buffers=d.sewnAssemblyBuffers,pose=d.parts[1].mesh.geometry.attributes.position.array.slice();
  assert.ok(buffers?.positions.length>0);
  for(let i=0;i<5;i++)system.updateAccess('right',1/60,null,{deviceId:d.id,release:'sheath'});
  assert.equal(d.sewnAssemblyBuffers,buffers);
  assert.notDeepEqual(d.parts[1].mesh.geometry.attributes.position.array,pose);
 }finally{system.dispose();}
});

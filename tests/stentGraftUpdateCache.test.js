import test from 'node:test';
import assert from 'node:assert/strict';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
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

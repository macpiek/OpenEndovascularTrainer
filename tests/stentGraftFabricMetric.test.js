import test from 'node:test';
import assert from 'node:assert/strict';
import {Vector3} from 'three';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {constrainSewnAssembly} from '../src/devices/stentGraftAssemblyConstraints.js';
import {sewnFabricProjector} from '../src/devices/stentGraftSewnConstraints.js';

const centers=p=>Array.from({length:p.rows},(_,r)=>{
 const v=new Vector3(),a=p.mesh.geometry.attributes.position;
 for(let j=0;j<p.sides;j++)v.add(new Vector3().fromBufferAttribute(a,r*p.sides+j));
 return v.divideScalar(p.sides);
});
// Same 0.25 mm local tolerance as the existing seam regression; total
// length has a separate 0.3 mm bound, so errors cannot accumulate along a leg.
const strain=p=>{
 const c=centers(p);let length=0,max=0;
 for(let r=1;r<p.rows;r++) {const ds=c[r].distanceTo(c[r-1]);length+=ds;
  if(p.exposure[r]>0&&p.exposure[r-1]>0)max=Math.max(max,ds-(p.path.coordinates[r]-p.path.coordinates[r-1]));}
 return {length,max};
};
test('changing fitted target cannot redefine the rest length of manufactured fabric',()=>{
 const {system,device:d}=previewFixture();
 try {
  for(let i=0;i<130;i++)system.updateAccess('right',d.sheathTravel/12/100,null,{deviceId:d.id,release:'sheath'});
  const p=d.parts[1],before=p.sewnConstraints.cloth.map(c=>c.length);
  for(let i=1;i<p.rows;i++)for(let j=0;j<p.sides;j++)p.target[(i*p.sides+j)*3+1]-=i*3;
  p.sewnConstraintKey=null;sewnFabricProjector(p,true);
  assert.deepEqual(p.sewnConstraints.cloth.map(c=>c.length),before);
 }finally{system.dispose();}
});
for(const side of ['right','left'])test(`${side}: material length holds along the whole ipsilateral leg during release and rotation`,()=>{
 const {system,device:d}=previewFixture(side);
 try {
  let worst=0;
  for(let i=0;i<140;i++) {
   system.updateAccess(side,d.sheathTravel/12/100,null,{deviceId:d.id,release:i<110?'sheath':'resheath',mechanicalRotation:i*.003});
   const p=d.parts[1],s=strain(p);worst=Math.max(worst,s.max);
   assert.ok(s.length<=p.path.length+.3,`total ${s.length} > ${p.path.length}`);
  }
  assert.ok(worst<.25,`local excess ${worst}`);
 }finally{system.dispose();}
});
test('global cloth constraints recover a stretched long leg without moving its sewn mouth independently',()=>{
 const {system,device:d}=previewFixture();
 try {
  for(let i=0;i<130;i++)system.updateAccess('right',d.sheathTravel/12/100,null,{deviceId:d.id,release:'sheath'});
  const p=d.parts[1],a=p.mesh.geometry.attributes.position,y=centers(p)[0].y;
  for(let i=p.sides;i<a.count;i++)a.setY(i,y+(a.getY(i)-y)*1.8);a.needsUpdate=true;
  assert.ok(strain(p).length>p.path.length*1.5);
  constrainSewnAssembly(d);const after=strain(p);
  assert.ok(after.length<p.path.length+.3,JSON.stringify(after));
  assert.ok(after.max<.25,JSON.stringify(after));
 }finally{system.dispose();}
});

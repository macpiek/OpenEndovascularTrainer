import test from 'node:test';
import assert from 'node:assert/strict';
import {Vector3} from 'three';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
const centers=(p,a)=>Array.from({length:p.rows},(_,r)=>{
 const v=new Vector3();for(let j=0;j<p.sides;j++)v.add(new Vector3().fromArray(a,(r*p.sides+j)*3));return v.divideScalar(p.sides);
});
for(const side of ['right','left'])test(`${side}: sewn branch mouth cannot detach during release, rotation and recapture`,()=>{
 const {system,device:d}=previewFixture(side);let checked=0;
 try {
  for(let step=0;step<150;step++) {
   system.updateAccess(side,d.sheathTravel/12/120,null,{deviceId:d.id,release:step<120?'sheath':'resheath',mechanicalRotation:step/120*.25});
   const trunk=d.parts[0].mesh.geometry.attributes.position.array;
   for(let k=1;k<3;k++)if(d.parts[k].exposure[0]>0) {
    const p=d.parts[k],a=p.mesh.geometry.attributes.position.array;
    // Every point of the mouth follows its original material position in the
    // deforming outlet, including the septum. A centroid-only constraint
    // passes even when opposite edges detach in opposite directions.
    for(let j=0;j<p.sides;j++)for(let c=0;c<3;c++) {
     const {indices,weights}=d.sewnJunctionBindings[k-1][j];
     const sewn=weights.reduce((sum,w,n)=>sum+w*trunk[indices[n]+c],0);
     assert.ok(Math.abs(a[j*3+c]-sewn)<.00005,`detached mouth ${k}/${j}, step ${step}`);
    }
    const current=centers(p,a);
    for(let r=1;r<Math.min(p.rows,6);r++)if(p.exposure[r-1]>0) {
     const material=p.path.coordinates[r]-p.path.coordinates[r-1];
     assert.ok(current[r].distanceTo(current[r-1])<=material+.25,'the seam must not pull adjacent fabric rows apart');
    }
    checked++;
    for(const ring of p.scaffoldRings)if(!ring.packed){assert.equal(ring.currentHeight,ring.height);assert.ok(Math.abs(ring.lengthError)<1e-4);}
   }
  }
  assert.ok(checked>100);
 }finally{system.dispose();}
});
test('first branch interval has only a small mesh overlap, not an extra two millimetres of material',()=>{
 const {system,device:d}=previewFixture();
 try {for(const p of d.parts.slice(1)) {
  const c=centers(p,p.target),rest=p.path.coordinates[1]-p.path.coordinates[0];
  assert.ok(c[1].distanceTo(c[0])<rest*1.15,'the first interval must not be doubled by mesh-union overlap');
 }}finally{system.dispose();}
});

test('one coarse deployment command can cross the bifurcation before either limb has built material constraints',()=>{
 const {system,device:d}=previewFixture();
 try {
  system.updateAccess('right',d.sheathTravel*.55/12,null,{deviceId:d.id,release:'sheath'});
  for(const part of d.parts)assert.ok(part.mesh.geometry.attributes.position.array.every(Number.isFinite));
 }finally{system.dispose();}
});

import test from 'node:test';
import assert from 'node:assert/strict';
import {createSacThroughflow} from '../src/contrast/graftSacThroughflow.js';
import {partialFlowFixture} from './helpers/partialContrastFixture.js';
import {createContrastReleaseSurface} from '../src/devices/stentGraftContrastSurface.js';
import {ContrastVolumeRenderer} from '../src/contrast/contrastVolumeRenderer.js';

function tree(volumes=[1,1,1,1]) {
 const entry={mass:new Float64Array(volumes.length),volumes};
 const cells=volumes.map((volume,cellIndex)=>({entry,cellIndex,volume,length:1}));
 return {entry,cells};
}
test('sac throughflow branches at patent outlets, conserves mass and washes out tiny cells',()=>{
 const {entry,cells}=tree([1,1e-5,1,1]);entry.mass[0]=100;
 const links=[{a:0,b:1},{a:1,b:2},{a:1,b:3}],drains=[{a:2,weight:1},{a:3,weight:1}];
 const escaped=[0,0],flow=createSacThroughflow(cells,links,drains,[{inlet:0,q:10}],(d,m)=>escaped[drains.indexOf(d)]+=m);
 assert.ok(Math.abs(flow.outflow[0]-10)<1e-4);
 for(let i=0;i<100;i++)flow.update(.1);
 assert.ok(entry.mass.every(v=>v>=0&&Number.isFinite(v)));
 assert.ok(entry.mass.reduce((a,b)=>a+b,0)<1e-8);
 assert.ok(Math.abs(escaped[0]-50)<1e-6&&Math.abs(escaped[1]-50)<1e-6);
});
test('a disconnected pocket without an outlet is retained rather than numerically faded',()=>{
 const {entry,cells}=tree();entry.mass[0]=30;entry.mass[2]=70;
 const flow=createSacThroughflow(cells,[{a:0,b:1},{a:2,b:3}],[{a:1,weight:1}],[{inlet:0,q:10}],()=>{});
 for(let i=0;i<100;i++)flow.update(.1);
 assert.equal(entry.mass[2],70);assert.equal(entry.mass[3],0);
});
test('sac and lumen history survives repeated geometry revisions without black interpolation frames',()=>{
 const f=partialFlowFixture(),n=f.system.flowNetwork,r=new ContrastVolumeRenderer(f.system);
 let surface=createContrastReleaseSurface([f.device],1);
 try {
  f.system.setStentGraftSurface(surface);
  for(const entry of n.stentGraftRemodeling.trapped.values())for(let i=0;i<entry.mass.length;i++)entry.mass[i]=entry.volumes[i]*.1;
  n.stentGraftRemodeling.trappedIodineMassMg=[...n.stentGraftRemodeling.trapped.values()].reduce((sum,e)=>sum+e.mass.reduce((a,b)=>a+b,0),0);
  for(const e of n.edges)n.depositIodine(e.index,0,1);
  f.system._solverAccumulator=0;r.update({reuseUnchanged:true});
  const texture=r._trappedMesh.userData.texture,before=r._trappedMesh.userData.data.slice();
  assert.ok(before.some(v=>v>0));
  assert.ok(r._trappedMesh.geometry.attributes.flowVolumeBlend.array.every(v=>v===0),
   'native volume replacement must not hide the separately retained sac image');
  for(let i=2;i<8;i++) {
   const next=createContrastReleaseSurface([f.device],i);f.system.setStentGraftSurface(next);surface.dispose();surface=next;
   r.update({reuseUnchanged:true});
   assert.equal(r._trappedMesh.userData.texture,texture,'reuse fixed native sac geometry and its history');
   assert.deepEqual(r._trappedMesh.userData.data,before);
   for(const mesh of r._graftMeshes)assert.deepEqual(mesh.userData.previous,mesh.userData.concentration,'no zero-to-full fade on new cloth');
  }
 }finally{r.dispose();surface.dispose();f.dispose();}
});

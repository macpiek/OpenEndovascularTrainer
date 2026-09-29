import test from 'node:test';
import assert from 'node:assert/strict';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {sewnTopology} from '../src/devices/stentGraftSewnTopology.js';
import {updatePartialSurfacePose} from '../src/devices/stentGraftPartialSurface.js';
import {MAIN_BODY_MODELS} from '../src/devices/stentGraftModels.js';

function verifySurface(d) {
 const topology=sewnTopology(d),edges=new Map(),used=new Set(),triangles=new Set();
 for(const {ids}of topology.faces){
  const triangle=[...ids].sort((a,b)=>a-b).join(',');assert.ok(!triangles.has(triangle),'duplicate fabric triangle');triangles.add(triangle);
  for(let j=0;j<3;j++){
   const a=ids[j],b=ids[(j+1)%3],key=[Math.min(a,b),Math.max(a,b)].join(',');used.add(a);
   const edge=edges.get(key)??{count:0,winding:0};edge.count++;edge.winding+=a<b?1:-1;edges.set(key,edge);
  }
 }
 const ports=new Set(topology.ports.flatMap(({ids})=>ids.map((a,j)=>[a,ids[(j+1)%ids.length]].sort((a,b)=>a-b).join(','))));
 for(const [key,edge]of edges){
  assert.equal(edge.count,ports.has(key)?1:2,`unsewn or overlapping edge ${key}`);
  if(!ports.has(key))assert.equal(edge.winding,0,`inverted stitch ${key}`);
 }
 assert.equal(used.size-edges.size+topology.faces.length,-1,'one Y surface with three open ports');
 const positions=d.sewnMesh.geometry.attributes.position.array,indices=d.sewnMesh.geometry.index.array;
 assert.equal(indices.length,d.contactFaces.length*3);
 updatePartialSurfacePose(d,true);
 d.contactFaces.forEach((f,k)=>f.positions.forEach((v,j)=>assert.ok(Math.abs(v-positions[indices[k*3+Math.floor(j/3)]*3+j%3])<5e-5,'collision and visible cloth disagree')));
 assert.ok(d.parts.every(p=>!p.mesh.visible),'old overlapping cloth must not be drawn');
 return edges;
}
for(const side of ['right','left'])for(const model of MAIN_BODY_MODELS)test(`${side}/${model.id}: shared Y has exactly three open ports during release and recapture`,()=>{
 const {system,device:d}=previewFixture(side,'body',true,model.id);
 try {
  const faces=d.sewnMesh.geometry.index.array.slice();
  for(let i=0;i<32;i++){
   system.updateAccess(side,d.sheathTravel/12/24,null,{deviceId:d.id,release:i<24?'sheath':'resheath',mechanicalRotation:i/32*.3});
   verifySurface(d);assert.deepEqual(d.sewnMesh.geometry.index.array,faces,'release cannot change the sewn topology');
  }
 }finally{system.dispose();}
});

test('deployed collision wall retains open ports and uses the same sewn cloth',async()=>{
 const {StentGraftSurface}=await import('../src/devices/stentGraftSurface.js');
 const {system,device:d}=previewFixture();let surface;
 try {
  for(let i=0;i<35;i++)system.updateAccess('right',d.sheathTravel/12/35,null,{deviceId:d.id,release:'sheath'});
  system.updateAccess('right',2,null,{deviceId:d.id,release:'tip'});
  assert.equal(d.phase,'deployed');
  surface=new StentGraftSurface([d],1);
  const p=d.sewnMesh.geometry.attributes.position,ix=d.sewnMesh.geometry.index;
  const key=values=>values.map(v=>Number(v.toFixed(4))).join(',');
  const triangles=new Set();
  for(let i=0;i<ix.count;i+=3)triangles.add([0,1,2].map(k=>key([p.getX(ix.getX(i+k)),p.getY(ix.getX(i+k)),p.getZ(ix.getX(i+k))])).sort().join('/'));
  const wall=surface.geometry.attributes.position;
  assert.equal(wall.count,ix.count,'the three artificial caps must be removed');
  for(let i=0;i<wall.count;i+=3)assert.ok(triangles.has([0,1,2].map(k=>key([wall.getX(i+k),wall.getY(i+k),wall.getZ(i+k)])).sort().join('/')),'full collision surface must match visible cloth');
 }finally{surface?.dispose();system.dispose();}
});

for(const side of ['right','left'])test(`${side}: anatomical aneurysm mesh keeps the same closed seam through bifurcation release`,async()=>{
 const {fixture,place}=await import('./helpers/stentGraftFixture.js');
 const f=fixture();
 try {
  place(f.system,side,'body');f.system.deploy(side);const d=f.system.accesses[side].device;
  for(let i=0;i<12;i++){
   f.system.updateAccess(side,d.sheathTravel/12/12,null,{deviceId:d.id,release:'sheath'});verifySurface(d);
  }
 }finally{f.dispose();}
});

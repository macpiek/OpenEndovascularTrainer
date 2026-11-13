// Frozen post-retopology renderer versus the next continuity correction.
import fs from 'node:fs';
import {gzipSync,gunzipSync} from 'node:zlib';
import {pathToFileURL} from 'node:url';
import {createHash} from 'node:crypto';
import * as T from 'three';
import {archColumnFixture as currentFixture} from '../tests/helpers/archColumnFixture.js';
import {HybridContrastSystem as CurrentHybrid} from '../src/contrast/hybridContrastSystem.js';
const snapshot='/tmp/arch-physical-seam-before-2026-11-12';
const target=new URL('../tests/fixtures/arch-physical-seam.json.gz',import.meta.url);
// A first capture can use the explicitly frozen development tree. Once the
// fixture exists, current transport can be regenerated without that temporary
// tree: its complete baseline shaders, textures and frames are already stored.
const frozenAvailable=fs.existsSync(`${snapshot}/helper.js`)&&!process.argv.includes('--current-only');
if(!frozenAvailable&&!fs.existsSync(target))throw Error('No frozen baseline: capture the reference tree or restore the checked-in fixture first');
const base=JSON.parse(gunzipSync(fs.readFileSync(frozenAvailable?`${snapshot}/arch-distal.json.gz`:target)));
const {archColumnFixture}=frozenAvailable?await import(pathToFileURL(`${snapshot}/helper.js`)):{archColumnFixture:currentFixture};
const {HybridContrastSystem}=frozenAvailable?await import(pathToFileURL(`${snapshot}/src/contrast/hybridContrastSystem.js`)):{HybridContrastSystem:CurrentHybrid};
const {decodeCollisionAsset}=await import('../src/physics/collision/collisionAssetFormat.js');
const bySource=new Map(base.edges.map((e,i)=>[e.sourceIndex,i]));
const sourceFingerprint=frozenAvailable?JSON.parse(fs.readFileSync(`${snapshot}/fingerprint.json`)):base.provenance.sourceFingerprint;
if(frozenAvailable)for(const file of ['src/contrast/localInjectionSolver.js','src/contrast/outletJetField.js','src/contrast/flowOpticalTopology.js'])sourceFingerprint[file]=createHash('sha256').update(fs.readFileSync(`${snapshot}/${file}`)).digest('hex');
const positions=[];
// Retain actual wall triangles around all three physical ostia. No synthetic
// vessel tubes or wall-label smoothing are used for the image comparison.
for(let i=0;i<base.positions.length;i+=9){let inside=true;for(let j=0;j<9;j+=3)inside&&=base.positions[i+j]>-25&&base.positions[i+j]<70&&base.positions[i+j+1]>0&&base.positions[i+j+1]<105&&base.positions[i+j+2]>-60&&base.positions[i+j+2]<45;if(inside)positions.push(...base.positions.slice(i,i+9));}
const frames=[],data={positions,edges:base.edges,nodes:base.nodes,frames,regions:[],provenance:{reference:'Frozen post-retopology Nov12 renderer, before current continuity correction',sourceFingerprint,description:'Frozen and current transport use identical anatomy, camera, exact catheter outlet, dose, rate and seed. Baseline render textures remain independent of current mass and jet states. Owner probes lie inside both incident lumens.'}};
let original;
if(frozenAvailable){
for(const root of [135,157,230])for(const reverse of [false,true]){
 const descendants=new Set(),queue=[bySource.get(root)];while(queue.length){const index=queue.pop();if(descendants.has(index))continue;descendants.add(index);queue.push(...data.edges[index].childEdgeIndices);}
 const source=base.frames.find(f=>f.kind==='source25'&&Math.abs(f.elapsed-.5)<1e-6);
 frames.push({label:`Ujście${root} · ${reverse?'gęsta gałąź / rozcieńczona aorta':'gęsta aorta / rozcieńczona gałąź'}`,kind:'controlled',root,reverse,time:source.time,elapsed:source.elapsed,boundaryFlows:source.boundaryFlows,mass:data.edges.map((e,index)=>e.volumes.map(v=>e.transportExcluded?0:.3*v*(descendants.has(index)?(reverse?.8:.03):(reverse?.03:.8))))});
}
// Create a valid initial field before the real local jet is sampled.
original=archColumnFixture(data);const cpu=original.cpu();
for(const root of [135,157,230]){const edge=original.network.edges[bySource.get(root)],parent=original.network.edges[edge.parentEdgeIndex];data.regions.push({root,center:edge.start.toArray(),point:root===135?[15.36147308,47.90370941,7.26015627]:root===157?[22.32691764831543,46.666721344,-8.7383394241333]:edge.start.toArray(),halfWidth:.35,owners:[cpu.built.territories[parent.index],cpu.built.territories[edge.index]],cameraSpan:25});}
}else{
 frames.push(...base.referenceFrames);
 Object.assign(data,{regions:base.regions,referenceProjection:base.referenceProjection,referenceShader:base.referenceShader,referenceFrames:base.referenceFrames});
}
// Read the same atlas used by the frozen runtime, retaining full hydraulics.
const bytes=fs.readFileSync(new URL('../res/Aorta_plain.collision.bin',import.meta.url)),asset=decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength)),a=asset.arrays.centerlineSegments,n=asset.arrays.centerlineEdges,stride=asset.metadata.centerline.stride;
const segments=Array.from({length:n.length/2},(_,index)=>({id:index,start:new T.Vector3().fromArray(a,index*stride),end:new T.Vector3().fromArray(a,index*stride+3),radiusStart:a[index*stride+6],radiusEnd:a[index*stride+7],safeRadius:a[index*stride+8],nodeStartId:n[index*2],nodeEndId:n[index*2+1]}));
const scenarios=[{name:'source134',position:[20.797064451630812,51.21208796661758,-14.132795969281586],direction:[-.381571375,.178985373,.906844817],volumeMl:10,steps:[12,30]}];
const source44={name:'source44',position:[18.39336014451009,50.51634444100875,5.364111258641297],direction:[-.3321058793729568,-.5767459806294072,.7463710596705493]};
scenarios.push({...source44,volumeMl:10,steps:[12,30]},{...source44,name:'source44long',volumeMl:20,steps:[54,90]});
async function captureTransport(Hybrid,fixture,{replace=false}={}){
 let at=6;
 for(const scenario of scenarios){
  const port={kind:'berenstein-end',position:new T.Vector3(...scenario.position),direction:new T.Vector3(...scenario.direction).normalize(),radiusMm:.48,areaMm2:Math.PI*.48**2,weight:1,valid:true};
  const system=new Hybrid({centerlineSegments:segments,catheter:{type:'berenstein',getInjectionPorts(out){out.length=0;out.push(port);return out;}},localOptions:{randomSeed:1212}}),net=system.flowNetwork;
  system.startInjection({source:'catheter',volumeMl:scenario.volumeMl,rateMlPerSec:12.1});
  for(let step=1;step<=Math.max(...scenario.steps);step++){
   system.advanceSynchronized(1/60);if(!scenario.steps.includes(step))continue;
   const frame={label:`Bolus ${scenario.name} · ${scenario.volumeMl} ml · ${(step/60).toFixed(2)} s`,kind:'transport',root:null,time:net.time,elapsed:step/60,port:scenario,mass:data.edges.map(e=>Array.from(net.edges[e.sourceIndex].massMg)),boundaryFlows:data.edges.map(e=>[net.getFaceSignedFlowMm3PerS(e.sourceIndex,0),net.getFaceSignedFlowMm3PerS(e.sourceIndex,net.edges[e.sourceIndex].cellCount)])};
   if(replace){if(data.referenceFrames[at]?.label!==frame.label)throw Error('Frozen frame mismatch: '+frame.label);frames[at]=frame;}else frames.push(frame);
   fixture.show(at++);
   frame.transportSummary=Object.fromEntries([135,157,230].map(root=>{
    const pending=[root],seen=new Set();let subtreeMassMg=0,maxConcentration=0;
    while(pending.length){const id=pending.pop();if(seen.has(id))continue;seen.add(id);const edge=net.edges[id];for(let cell=0;cell<edge.cellCount;cell++){subtreeMassMg+=edge.massMg[cell];maxConcentration=Math.max(maxConcentration,edge.massMg[cell]/edge.volumes[cell]/.3);}pending.push(...edge.childEdgeIndices);}
    return[root,{subtreeMassMg,maxConcentration,rootMassMg:Array.from(net.edges[root].massMg).reduce((a,b)=>a+b,0)}];
   }));
   const solver=system.localSolver,adapted={count:solver.count};for(const key of ['iodineMassMg','positionX','positionY','positionZ','jetVelocityX','jetVelocityY','jetVelocityZ','outletDirectionX','outletDirectionY','outletDirectionZ','outletRadiusMm','resolvedOutletRadiusMm','travelDistanceMm','ageSeconds','sourceCellIndex'])adapted[key]=solver[key];adapted.sourceEdgeIndex=Int32Array.from(solver.sourceEdgeIndex,index=>bySource.get(index)??-1);
   adapted.resolveSourceHandoffLocation=(i,out)=>{const location=solver.resolveSourceHandoffLocation(i,out);if(location)location.edgeIndex=bySource.get(location.edgeIndex)??-1;return location;};
   const jet=fixture.surface.projection.outletJet;jet.update(adapted,.3);
   if(jet.fallbackMassByCell.some(v=>v>1e-8))throw Error('Jet fallback requires explicit bulk serialization');
   const u=jet.uniforms;frame.jet={count:u.outletJetCount.value,vectors:Object.fromEntries(['outletJetCenter','outletJetAxis','outletJetShape'].map(key=>[key,u[key].value.map(v=>v.toArray())])),min:u.outletJetMin.value.toArray(),max:u.outletJetMax.value.toArray(),representedMassMg:jet.massMg};
  }
 }
}
if(frozenAvailable)await captureTransport(HybridContrastSystem,original);
const binary=a=>({kind:'array',type:a.constructor.name,data:Buffer.from(a.buffer,a.byteOffset,a.byteLength).toString('base64')});
function encode(value){if(value?.isTexture)return{kind:'texture',width:value.image.width,height:value.image.height,data:binary(value.image.data),format:value.format,type:value.type,internalFormat:value.internalFormat,minFilter:value.minFilter,magFilter:value.magFilter,flipY:value.flipY};if(ArrayBuffer.isView(value))return binary(value);for(const name of ['Vector2','Vector3','Vector4','Matrix3','Matrix4','Color'])if(value?.['is'+name])return{kind:name,data:value.toArray()};if(Array.isArray(value))return value.map(encode);if(value&&typeof value==='object')return Object.fromEntries(Object.entries(value).filter(([,v])=>typeof v!=='function').map(([k,v])=>[k,encode(v)]));return value;}
const dynamic=['columnCurrent','columnPrevious','columnSharedHistory','flowValues','outletJetCount','outletJetCenter','outletJetAxis','outletJetShape','outletJetMin','outletJetMax'];
if(frozenAvailable){
data.referenceProjection={revision:data.provenance.reference,vertexShader:original.surface.material.vertexShader,fragmentShader:original.surface.material.fragmentShader,attributes:Object.fromEntries(Object.entries(original.surface.mesh.geometry.attributes).map(([key,a])=>[key,{array:binary(a.array),itemSize:a.itemSize,normalized:a.normalized}])),index:original.surface.mesh.geometry.index?binary(original.surface.mesh.geometry.index.array):null,uniforms:Object.fromEntries(Object.entries(original.surface.material.uniforms).filter(([key])=>!dynamic.includes(key)).map(([key,u])=>[key,encode(u.value)])),frames:[]};
for(let i=0;i<frames.length;i++){original.show(i);if(frames[i].jet){const jet=frames[i].jet,u=original.surface.material.uniforms;u.outletJetCount.value=jet.count;for(const[key,values]of Object.entries(jet.vectors))values.forEach((v,j)=>u[key].value[j].fromArray(v));u.outletJetMin.value.fromArray(jet.min);u.outletJetMax.value.fromArray(jet.max);}data.referenceProjection.frames.push(Object.fromEntries(dynamic.map(key=>[key,encode(original.surface.material.uniforms[key].value)])));}
data.referenceShader=data.referenceProjection.fragmentShader;
data.referenceFrames=frames.slice();
original.dispose();
}
const current=currentFixture(data);
await captureTransport(CurrentHybrid,current,{replace:true});
data.provenance.currentSourceFingerprint=Object.fromEntries(Object.keys(data.provenance.sourceFingerprint).map(file=>[file,fs.existsSync(file)?createHash('sha256').update(fs.readFileSync(file)).digest('hex'):'not available']));
current.dispose();
const packed=gzipSync(JSON.stringify(data));
if(!process.argv.includes('--dry-run'))fs.writeFileSync(target,packed);
console.log(JSON.stringify({triangles:positions.length/9,frames:frames.length,regions:data.regions,jets:frames.filter(f=>f.jet).map(f=>({label:f.label,count:f.jet.count,mass:f.jet.representedMassMg})),bytes:packed.length}));

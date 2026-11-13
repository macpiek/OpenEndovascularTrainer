// Records actual arch anatomy and transported boluses at production 60 Hz.
// The first build captures the pre-fix scalar field/shader; regeneration keeps it.
import fs from 'node:fs';
import {gzipSync,gunzipSync} from 'node:zlib';
import {execFileSync} from 'node:child_process';
import os from 'node:os';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
import * as T from 'three';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {HybridContrastSystem} from '../src/contrast/hybridContrastSystem.js';
import {arterialWaveform} from '../src/contrast/flowNetwork.js';
import {AnatomicalContrastSurface} from '../src/contrast/anatomicalContrastSurface.js';
import {archColumnFixture} from '../tests/helpers/archColumnFixture.js';
const target=new URL('../tests/fixtures/arch-column.json.gz',import.meta.url);
const anatomy=await loadCoupledRuntimeAnatomy(undefined,'plain');
const a=anatomy.field.centerline,k=anatomy.field.centerlineStride,n=anatomy.field.arrays.centerlineEdges;
const segments=Array.from({length:n.length/2},(_,id)=>({id,start:new T.Vector3().fromArray(a,id*k),end:new T.Vector3().fromArray(a,id*k+3),radiusStart:a[id*k+6],radiusEnd:a[id*k+7],safeRadius:a[id*k+8],nodeStartId:n[id*2],nodeEndId:n[id*2+1]}));
const port={kind:'berenstein-end',position:new T.Vector3(),direction:new T.Vector3(),radiusMm:.48,areaMm2:Math.PI*.48**2,weight:1,valid:true};
const system=new HybridContrastSystem({centerlineSegments:segments,catheter:{type:'berenstein',getInjectionPorts(out){out.length=0;out.push(port);return out;}},localOptions:{randomSeed:1212}});
const net=system.flowNetwork,ostium=net.nodes.get(1982),parent=net.edges[ostium.parentEdgeIndex];
port.position.copy(parent.start).lerp(parent.end,.4);port.direction.copy(parent.axis);
const selected=net.edges.filter(e=>Math.max(e.start.y,e.end.y)>-95&&Math.min(e.start.y,e.end.y)<175&&Math.min(e.start.x,e.end.x)<180&&Math.max(e.start.x,e.end.x)>-150&&Math.min(e.start.z,e.end.z)<100&&Math.max(e.start.z,e.end.z)>-125);
const remap=new Map(selected.map((e,i)=>[e.index,i]));
const edges=selected.map(e=>({sourceIndex:e.index,start:e.start.toArray(),end:e.end.toArray(),startNodeId:e.startNodeId,endNodeId:e.endNodeId,radiusStart:e.radiusStart,radiusEnd:e.radiusEnd,parentEdgeIndex:remap.get(e.parentEdgeIndex)??-1,childEdgeIndices:e.childEdgeIndices.filter(i=>remap.has(i)).map(i=>remap.get(i)),areas:Array.from(e.areas),volumes:Array.from(e.volumes),transportExcluded:e.transportExcluded,lumenMappingExcluded:e.lumenMappingExcluded,renderExcluded:e.renderExcluded,intraluminalAorticConnector:e.intraluminalAorticConnector}));
const g=anatomy.geometry,p=g.attributes.position,positions=[];
for(let i=0;i<(g.index?.count??p.count);i+=3){const ids=[0,1,2].map(j=>g.index?g.index.getX(i+j):i+j);if(ids.every(j=>p.getY(j)>-15&&p.getY(j)<85&&p.getX(j)>-45&&p.getX(j)<80&&p.getZ(j)>-65&&p.getZ(j)<60))for(const j of ids)positions.push(p.getX(j),p.getY(j),p.getZ(j));}
const frames=[];
function record(label,elapsed,kind='transport') {const waveform=arterialWaveform(net.time,net.hemodynamics.heartRateBpm);frames.push({label,kind,elapsed,time:net.time,mass:selected.map(e=>Array.from(e.massMg)),boundaryFlows:selected.map(e=>[net.getFaceSignedFlowMm3PerS(e.index,0,waveform),net.getFaceSignedFlowMm3PerS(e.index,e.cellCount,waveform)])});}
if(!system.startInjection({source:'catheter',volumeMl:10,rateMlPerSec:12.1}).ok)throw Error('Injection unavailable');
for(let frame=1;frame<=90;frame++){system.advanceSynchronized(1/60);if([3,12,30,54,90].includes(frame))record(`Podanie ${frame}/60 s`,frame/60);}
const trunk=net.edges[ostium.childEdgeIndices.find(i=>net.edges[i].radiusStart<10)],main=net.edges[ostium.childEdgeIndices.find(i=>net.edges[i].radiusStart>=10)];
const branch=new Set(),stack=[trunk.index];while(stack.length){const i=stack.pop();if(branch.has(i))continue;branch.add(i);stack.push(...net.edges[i].childEdgeIndices);}
for(const [label,parentValue,branchValue,mainValue,reflux] of [['Gęsta aorta / rozcieńczona gałąź',.85,.04,.6,false],['Refluks gałęzi / napływ krwi',.04,.9,.35,true]]){
 for(const e of net.edges){const c=branch.has(e.index)?branchValue:e.index>parent.index?mainValue:parentValue;for(let i=0;i<e.cellCount;i++)e.massMg[i]=c*.3*e.volumes[i];}
 record(label,0,'controlled');const f=frames.at(-1);f.boundaryFlows=selected.map(e=>{const q=branch.has(e.index)?(reflux?-15000:4000):e.index>parent.index?(reflux?25000:6000):10000;return[q,q];});
}
// A compact fixture preserves the actual graph IDs and optional pre-fix CPU values.
const data={center:ostium.point.toArray(),cameraCenter:[10,35,0],cameraSpan:65,source:port.position.toArray(),parent:remap.get(parent.index),branch:remap.get(trunk.index),main:remap.get(main.index),positions,edges,frames};
const fixture=archColumnFixture(data,{reference:false});
const currentParents=fixture.surface.junctionField.junctions.map(j=>selected[j.edges[0]].index);
data.referenceSharedParents=currentParents;
data.referenceShader=fixture.surface.material.fragmentShader;
for(let i=0;i<frames.length;i++){fixture.show(i);frames[i].referenceFieldValues=Array.from(fixture.surface.current.subarray(0,fixture.surface.field.cellCount+fixture.network.nodes.size));}
if(fs.existsSync(target)){const old=JSON.parse(gunzipSync(fs.readFileSync(target)));data.referenceShader=old.referenceShader;data.referenceSharedParents=old.referenceSharedParents;for(let i=0;i<frames.length;i++)frames[i].referenceFieldValues=old.frames[i]?.referenceFieldValues??frames[i].referenceFieldValues;}
// Rebuild the original projection at the committed pre-fix revision, including
// its old graph components, BVH ownership, ray records, and dynamic textures.
// Restoring only the old shader would silently use repaired topology.
const referenceRevision='0f9c5fc71694958731ddfa211aea3dcec455d11a';
const temporary=fs.mkdtempSync(path.join(os.tmpdir(),'arch-reference-'));
const packed=execFileSync('git',['archive',referenceRevision,'src'],{maxBuffer:128*1024*1024});
execFileSync('tar',['-xf','-','-C',temporary],{input:packed});
fs.writeFileSync(path.join(temporary,'package.json'),'{"type":"module"}');
fs.symlinkSync(path.resolve('node_modules'),path.join(temporary,'node_modules'),'dir');
const helper=fs.readFileSync(new URL('../tests/helpers/archColumnFixture.js',import.meta.url),'utf8').replaceAll('../../src/','./src/');
fs.writeFileSync(path.join(temporary,'helper.js'),helper);
const oldFixtureModule=await import(pathToFileURL(path.join(temporary,'helper.js')));
const original=oldFixtureModule.archColumnFixture({...data,referenceProjection:null},{reference:false});
const binary=a=>({kind:'array',type:a.constructor.name,data:Buffer.from(a.buffer,a.byteOffset,a.byteLength).toString('base64')});
function encode(value){
 if(value?.isTexture)return{kind:'texture',width:value.image.width,height:value.image.height,data:binary(value.image.data),format:value.format,type:value.type,internalFormat:value.internalFormat,minFilter:value.minFilter,magFilter:value.magFilter,flipY:value.flipY};
 if(ArrayBuffer.isView(value))return binary(value);
 for(const name of['Vector2','Vector3','Vector4','Matrix3','Matrix4','Color'])if(value?.['is'+name])return{kind:name,data:value.toArray()};
 if(Array.isArray(value))return value.map(encode);
 if(value&&typeof value==='object')return Object.fromEntries(Object.entries(value).filter(([,v])=>typeof v!=='function').map(([k,v])=>[k,encode(v)]));
 return value;
}
const dynamicNames=['columnCurrent','columnPrevious','columnSharedHistory','flowValues'];
const referenceProjection={revision:referenceRevision,fragmentShader:original.surface.material.fragmentShader,vertexShader:original.surface.material.vertexShader,
 attributes:Object.fromEntries(Object.entries(original.surface.mesh.geometry.attributes).map(([key,a])=>[key,{array:binary(a.array),itemSize:a.itemSize,normalized:a.normalized}])),
 index:original.surface.mesh.geometry.index?binary(original.surface.mesh.geometry.index.array):null,
 uniforms:Object.fromEntries(Object.entries(original.surface.material.uniforms).filter(([key])=>!dynamicNames.includes(key)).map(([key,u])=>[key,encode(u.value)])),frames:[]};
for(let i=0;i<frames.length;i++){
 original.show(i);
 referenceProjection.frames.push(Object.fromEntries(dynamicNames.map(key=>[key,encode(original.surface.material.uniforms[key].value)])));
 frames[i].referenceFieldValues=Array.from(original.surface.current.subarray(0,original.surface.field.cellCount+original.network.nodes.size));
}
data.referenceProjection=referenceProjection;
data.referenceSharedParents=original.surface.junctionField.junctions.map(j=>selected[j.edges[0]].index);
data.referenceShader=referenceProjection.fragmentShader;
data.provenance={referenceRevision,builder:'scripts/build-arch-mixing-preview.mjs',anatomy:'plain runtime collision atlas',transportHz:60,injection:{kind:'Berenstein end port',volumeMl:10,rateMlPerSec:12.1,radiusMm:.48},primaryNode:1982,primaryParent:25,primaryDaughter:27,crop:{min:[-45,-15,-65],max:[80,85,60]},controlledFrameDescription:'Two explicit dense/dilute concentration challenges supplement the five transported frames. They are not clinical measurements.',referenceDescription:'Original revision projection geometry, shader, complete static uniforms and per-frame dynamic texture buffers; no repaired reference topology.'};
original.dispose();fs.rmSync(temporary,{recursive:true,force:true});
fs.writeFileSync(target,gzipSync(JSON.stringify(data)));
console.log(JSON.stringify({edges:edges.length,triangles:positions.length/9,bytes:fs.statSync(target).size,frames:frames.length,referenceSharedParents:data.referenceSharedParents,currentSharedParents:currentParents,referenceRevision,parent:parent.index,trunk:trunk.index,source:port.position.toArray()}));
fixture.dispose();anatomy.dispose();

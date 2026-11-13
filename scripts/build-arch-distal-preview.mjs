// Actual source-134 replay plus a separate antegrade source-25 positive control.
// Transport uses the complete arterial network; only the displayed wall is cropped.
import fs from 'node:fs';
import {gzipSync, gunzipSync} from 'node:zlib';
import * as T from 'three';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {HybridContrastSystem} from '../src/contrast/hybridContrastSystem.js';
import {arterialWaveform} from '../src/contrast/flowNetwork.js';
const target=new URL('../tests/fixtures/arch-distal.json.gz',import.meta.url);
const anatomy=await loadCoupledRuntimeAnatomy(undefined,'plain');
const a=anatomy.field.centerline,k=anatomy.field.centerlineStride,n=anatomy.field.arrays.centerlineEdges;
const segments=Array.from({length:n.length/2},(_,id)=>({id,start:new T.Vector3().fromArray(a,id*k),end:new T.Vector3().fromArray(a,id*k+3),radiusStart:a[id*k+6],radiusEnd:a[id*k+7],safeRadius:a[id*k+8],nodeStartId:n[id*2],nodeEndId:n[id*2+1]}));
const crop={min:[-135,-35,-100],max:[145,235,70]},frames=[],scenarios=[];
let selected,edges,remap,probes,nodes,ostiumRepairs;
for(const scenario of ['source134','source25']){
 const port={kind:'berenstein-end',position:new T.Vector3(20.797064451630812,51.21208796661758,-14.132795969281586),direction:new T.Vector3(-.381571375,.178985373,.906844817).normalize(),radiusMm:.48,areaMm2:Math.PI*.48**2,weight:1,valid:true};
 const system=new HybridContrastSystem({centerlineSegments:segments,catheter:{type:'berenstein',getInjectionPorts(out){out.length=0;out.push(port);return out;}},localOptions:{randomSeed:1212}}),net=system.flowNetwork;
 if(scenario==='source25'){port.position.copy(net.edges[25].start).lerp(net.edges[25].end,.4);port.direction.copy(net.edges[25].axis);}
 selected=net.edges.filter(e=>[0,1,2].every(j=>Math.max(e.start.getComponent(j),e.end.getComponent(j))>crop.min[j]-50&&Math.min(e.start.getComponent(j),e.end.getComponent(j))<crop.max[j]+50));
 remap=new Map(selected.map((e,i)=>[e.index,i]));
 if(!edges){edges=selected.map(e=>({sourceIndex:e.index,start:e.start.toArray(),end:e.end.toArray(),startNodeId:e.startNodeId,endNodeId:e.endNodeId,radiusStart:e.radiusStart,radiusEnd:e.radiusEnd,parentEdgeIndex:remap.get(e.parentEdgeIndex)??-1,childEdgeIndices:e.childEdgeIndices.filter(i=>remap.has(i)).map(i=>remap.get(i)),areas:Array.from(e.areas),volumes:Array.from(e.volumes),transportExcluded:e.transportExcluded,lumenMappingExcluded:e.lumenMappingExcluded,renderExcluded:e.renderExcluded,intraluminalAorticConnector:e.intraluminalAorticConnector,physicalArchOstium:e.physicalArchOstium,retiredArchPrefix:e.retiredArchPrefix,originalArchRootEdgeIndex:e.originalArchRootEdgeIndex}));
  const nodeIds=new Set(selected.flatMap(e=>[e.startNodeId,e.endNodeId]));nodes=[...nodeIds].map(id=>{const node=net.nodes.get(id);return{id,point:node.point.toArray(),parentEdgeIndex:remap.get(node.parentEdgeIndex)??-1,childEdgeIndices:node.childEdgeIndices.filter(i=>remap.has(i)).map(i=>remap.get(i))};});ostiumRepairs=net.archOstiumReattachmentDiagnostics;
  const probeIndices=new Map([[204,'clear brachiocephalic branch'],[205,'clear carotid sleeve'],[270,'supplied left subclavian artery'],[162,'descending aorta'],[163,'short supplied side branch']]);
  for(const root of ostiumRepairs.repairs.map(repair=>repair.rootEdgeIndex))for(const wanted of [20,40,80,120]){
   let e=net.edges[root],d=0;while(e&&d+e.length<wanted){d+=e.length;e=e.childEdgeIndices.map(i=>net.edges[i]).filter(e=>!e.transportExcluded).sort((a,b)=>b.meanFlowMm3PerS-a.meanFlowMm3PerS)[0];}if(e)probeIndices.set(e.index,`root${root} / ${wanted}mm`);
  }
  probes=[...probeIndices].map(([index,label])=>{const e=net.edges[index];return{label,sourceIndex:index,edge:remap.get(index),point:e.start.clone().lerp(e.end,.5).toArray(),radius:(e.radiusStart+e.radiusEnd)/2};});
 }
 if(!system.startInjection({source:'catheter',volumeMl:10,rateMlPerSec:12.1}).ok)throw Error('Injection unavailable');
 for(let step=1;step<=480;step++){
  system.advanceSynchronized(1/60);
  if([3,12,30,54,90,150,240,480].includes(step)){
   const waveform=arterialWaveform(net.time,net.hemodynamics.heartRateBpm),diagnostics=system.getMetrics();
   frames.push({label:`${scenario} · ${(step/60).toFixed(2)} s`,kind:scenario,elapsed:step/60,time:net.time,mass:selected.map(e=>Array.from(e.massMg)),boundaryFlows:selected.map(e=>[net.getFaceSignedFlowMm3PerS(e.index,0,waveform),net.getFaceSignedFlowMm3PerS(e.index,e.cellCount,waveform)]),sourceEdge:diagnostics.lastSourceEdgeIndex,probes:probes.map(p=>{const e=net.edges[p.sourceIndex];return{mass:e.massMg.reduce((s,v)=>s+v,0),concentration:e.massMg.reduce((s,v)=>s+v,0)/e.volumes.reduce((s,v)=>s+v,0)/.3};})});
  }
 }
 const diagnostics=system.getMetrics();scenarios.push({name:scenario,port:port.position.toArray(),direction:port.direction.toArray(),sourceEdge:diagnostics.lastSourceEdgeIndex,sourceDistanceMm:diagnostics.lastSourceDistanceMm});
}
if(scenarios[0].sourceEdge!==134||scenarios[1].sourceEdge!==25)throw Error(`Unexpected source mapping ${JSON.stringify(scenarios)}`);
const positions=[],g=anatomy.geometry,p=g.attributes.position;
for(let i=0;i<(g.index?.count??p.count);i+=3){const ids=[0,1,2].map(j=>g.index?g.index.getX(i+j):i+j);if(ids.every(j=>[p.getX(j),p.getY(j),p.getZ(j)].every((v,axis)=>v>crop.min[axis]&&v<crop.max[axis])))for(const j of ids)positions.push(p.getX(j),p.getY(j),p.getZ(j));}
const data={positions,edges,nodes,frames,probes,scenarios,ostiumRepairs,cameraCenter:[12,62,-8],cameraSpan:65,center:probes[0].point,provenance:{builder:'scripts/build-arch-distal-preview.mjs',anatomy:'plain runtime collision atlas',crop,transportHz:60,injection:{volumeMl:10,rateMlPerSec:12.1},description:'Source134 is the corrected world-space screenshot port. Source25 is a separate physiological positive control. Neither replay forces branch propagation.'}};
// Keep the historical transport graph and its cell arrays independent of
// the corrected graph. Replaying old mass arrays on new cell volumes would
// produce a misleading comparison, even when atlas edge IDs are unchanged.
if(!fs.existsSync(target))throw Error('Keep the historical arch-distal fixture when regenerating; it is the frozen before baseline.');
const stored=JSON.parse(gunzipSync(fs.readFileSync(target)));
const frozen=stored.frozenBefore??stored,oldPositions=frozen.positions??stored.positions;
if(oldPositions.length!==positions.length||oldPositions.some((value,index)=>value!==positions[index]))throw Error('Anatomical wall changed: regenerate a separate independent before geometry rather than sharing positions.');
data.frozenBefore={...frozen,positions:undefined,usesSharedPositions:true};
data.provenance.referenceDescription='Before uses the complete frozen pre-retopology graph, transport frames, projection geometry, and GPU buffers. Only bit-identical anatomical source positions are shared.';
fs.writeFileSync(target,gzipSync(JSON.stringify(data)));anatomy.dispose();
console.log(JSON.stringify({edges:edges.length,triangles:positions.length/9,bytes:fs.statSync(target).size,frames:frames.length,scenarios,probes:probes.map(p=>({edge:p.sourceIndex,label:p.label}))}));

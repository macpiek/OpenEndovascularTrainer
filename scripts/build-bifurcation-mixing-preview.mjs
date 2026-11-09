// npm run preview:contrast:mixing, then /tests/actualBifurcationMixing.browser.html.
// --reuse-transport reuses the generated recording while iterating on reconstruction.
import fs from 'node:fs';
import * as THREE from 'three';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';
import {ContrastFlowNetwork,arterialWaveform} from '../src/contrast/flowNetwork.js';
import {HybridContrastSystem} from '../src/contrast/hybridContrastSystem.js';
import {AnatomicalContrastSurface} from '../src/contrast/anatomicalContrastSurface.js';
import {sampleFlowConcentration} from '../src/contrast/flowConcentrationField.js';
import {measureBifurcationSurface,assertBifurcationMixingRegression} from '../tests/helpers/actualBifurcationMixingRegression.js';
const anatomy=await loadCoupledRuntimeAnatomy(undefined,'baseline');
const {field,geometry:g}=anatomy,a=field.centerline,s=field.centerlineStride,n=field.arrays.centerlineEdges;
const segments=Array.from({length:a.length/s},(_,id)=>({id,start:new THREE.Vector3(...a.slice(id*s,id*s+3)),end:new THREE.Vector3(...a.slice(id*s+3,id*s+6)),radiusStart:a[id*s+6],radiusEnd:a[id*s+7],safeRadius:a[id*s+8],nodeStartId:n[id*2],nodeEndId:n[id*2+1]}));
for(const r of field.metadata.renal?.segments??[])Object.assign(segments[r.index],{renalSide:r.side,renalSector:r.sector,renalHilar:r.hilar??false,renalLateral:r.lateral??false,source:'renal-mesh'});
const cache=new URL('../tests/actualBifurcationMixing.transport.json',import.meta.url);
let recordings;
if(process.argv.includes('--reuse-transport')&&fs.existsSync(cache))recordings=JSON.parse(fs.readFileSync(cache,'utf8'));
else{
 recordings=[];
 for(const [phase,steps] of [[0,[15,75]],[.25,[75,78,81]],[.5,[78]],[.75,[67,70,73]]]){
  const system=new HybridContrastSystem({centerlineSegments:segments,contactField:field,sheath:{...anatomy.vessel.sheath,innerRadius:1,length:70},localOptions:{randomSeed:20260725}});
  const net=system.flowNetwork;net.time=phase*60/net.hemodynamics.heartRateBpm;
  if(!system.startInjection({source:'sheath',volumeMl:20,rateMlPerSec:15}).ok)throw Error('Sheath injection unavailable');
  for(let step=1;step<=Math.max(...steps);step++){
   system.advanceSynchronized(1/60);if(!steps.includes(step))continue;
   const wave=arterialWaveform(net.time,net.hemodynamics.heartRateBpm);
   recordings.push({phase,elapsed:step/60,time:net.time,cells:net.edges.map(e=>Array.from(e.massMg)),boundaryFlows:net.edges.map(e=>[net.getFaceSignedFlowMm3PerS(e.index,0,wave),net.getFaceSignedFlowMm3PerS(e.index,e.cellCount,wave)])});
  }
 }
 fs.writeFileSync(cache,JSON.stringify(recordings));
}
const network=new ContrastFlowNetwork(segments),p=g.attributes.position,positions=[];
for(let i=0;i<(g.index?.count??p.count);i+=3){
 const ids=[0,1,2].map(j=>g.index?g.index.getX(i+j):i+j);
 if(ids.every(k=>p.getY(k)>-340&&p.getY(k)<-260))for(const k of ids)positions.push(p.getX(k),p.getY(k),p.getZ(k));
}
const crop=new THREE.BufferGeometry().setAttribute('position',new THREE.Float32BufferAttribute(positions,3));
let count=0;const offsets=Uint32Array.from(network.edges,e=>{const offset=count;count+=e.cellCount;return offset;});
// This recording serializes scalar reconstruction and the surface shader.
// The volumetric projector owns BVH textures that are intentionally absent
// from this independent concentration/mixing fixture format.
const surfaces=Object.fromEntries(['before','after'].map(name=>[name,new AnatomicalContrastSurface(network,offsets,{fallbackGeometry:crop},{junctionReconstruction:name==='after',projection:'surface'})]));
for(const name of ['position','normal','flowAxis','opticalRadius']){
 const before=surfaces.before.mesh.geometry.attributes[name].array,after=surfaces.after.mesh.geometry.attributes[name].array;
 if(before.length!==after.length||before.some((v,i)=>v!==after[i]))throw Error('Reconstruction must not change anatomy or optical chord');
}
const cases=[];
for(const recording of recordings){
 network.time=recording.time;for(const e of network.edges)e.massMg.set(recording.cells[e.index]);
 network.getFaceSignedFlowMm3PerS=(edge,face)=>recording.boundaryFlows[edge][face===0?0:1];
 const states={};
 for(const [name,surface]of Object.entries(surfaces)){
  surface.update(new Float32Array(count),.3,false);states[name]=Array.from(surface.current);
  if(states[name].some(c=>!Number.isFinite(c)||c<0))throw Error('Invalid reconstruction');
 }
 cases.push({phase:recording.phase,elapsed:recording.elapsed,states});
}
const geometry=surfaces.before.mesh.geometry;geometry.clearGroups();geometry.addGroup(0,geometry.attributes.position.count,0);
// Independent causality challenge on the actual anatomy: iodine exists only
// at the two remote iliac anchors, while the fork and parent blood are clear.
// This catches a spatial interpolator that reads distant anchors too early,
// even when its snapshots look smoother than the original ownership seam.
const surface=surfaces.after,junction=surface.junctionField.junctions.find(j=>j.center.y>-305&&j.center.y<-275&&j.radius>6);
if(!junction)throw Error('Actual aortoiliac junction missing from regression crop');
surface.current.fill(0);const sourceIndices=new Set();
for(const anchor of junction.anchors.slice(1))for(const i of [anchor.stencil[0],anchor.stencil[1],anchor.stencil[3]])if(i>=0){surface.current[i]=1;sourceIndices.add(i);}
surface.junctionField.update(surface.current);
const causalityProbe={sourceCells:[...sourceIndices].filter(i=>i<surface.field.cellCount).length,
 sourceAnchorMinimum:Math.min(...junction.anchors.slice(1).map(a=>sampleFlowConcentration(surface.current,...a.stencil))),
 junctionConcentration:surface.current[junction.nodeSample],
 proximalMaximum:measureBifurcationSurface(geometry.attributes.position.array,surface.mesh.geometry.attributes.flowStencil.array,surface.current).proximalMax};
const result={geometry:geometry.toJSON(),afterStencil:Array.from(surfaces.after.mesh.geometry.attributes.flowStencil.array),vertexShader:surfaces.after.material.vertexShader,fragmentShader:surfaces.after.material.fragmentShader,diagnostics:surfaces.after.diagnostics,cases,causalityProbe};
const regression=assertBifurcationMixingRegression(result);
const output=new URL('../tests/actualBifurcationMixing.fixture.json',import.meta.url);fs.writeFileSync(output,JSON.stringify(result));
console.log(JSON.stringify({bytes:fs.statSync(output).size,diagnostics:result.diagnostics,cases:cases.map(c=>({phase:c.phase,elapsed:c.elapsed})),regression},null,2));
for(const surface of Object.values(surfaces))surface.dispose();crop.dispose();anatomy.dispose();

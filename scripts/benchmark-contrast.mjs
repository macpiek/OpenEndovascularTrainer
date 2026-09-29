import fs from 'node:fs';
import {pathToFileURL,fileURLToPath} from 'node:url';
// Optional arguments: project root, then 'baseline' for the old 30 Hz display cadence.
const root=process.argv[2]??fileURLToPath(new URL('..',import.meta.url)),load=p=>import(pathToFileURL(`${root}/${p}`));
const THREE=await load('node_modules/three/build/three.module.js');
const {decodeCollisionAsset}=await load('src/physics/collision/collisionAssetFormat.js');
const {HybridContrastSystem}=await load('src/contrast/hybridContrastSystem.js');
const {ContrastVolumeRenderer}=await load('src/contrast/contrastVolumeRenderer.js');
const raw=fs.readFileSync(`${root}/res/Aorta_infrarenal_aneurysm.collision.bin`),asset=decodeCollisionAsset(raw.buffer.slice(raw.byteOffset,raw.byteOffset+raw.byteLength)),d=asset.arrays.centerlineSegments,e=asset.arrays.centerlineEdges;
const segments=[];for(let i=0;i<d.length;i+=9)segments.push({id:i/9,start:new THREE.Vector3(...d.slice(i,i+3)),end:new THREE.Vector3(...d.slice(i+3,i+6)),radiusStart:d[i+6],radiusEnd:d[i+7],safeRadius:d[i+8],nodeStartId:e[i/9*2],nodeEndId:e[i/9*2+1]});
const s=new HybridContrastSystem({centerlineSegments:segments,localOptions:{capacity:100}}),r=new ContrastVolumeRenderer(s),net=s.flowNetwork;
const upstream=net.findNearestLocation(new THREE.Vector3(0,-190,0)),times={transport:[],render:[]};
for(let frame=0;frame<240;frame++){
 if(frame<120&&frame%2===0)net.depositIodine(upstream.edgeIndex,upstream.cellIndex,10);
 let t=performance.now();s.update(1/60);times.transport.push(performance.now()-t);
 t=performance.now();if(process.argv[3]!=='baseline'||frame%2===1)r.update({reuseUnchanged:true});times.render.push(performance.now()-t);
}
const stats=a=>{a.sort((a,b)=>a-b);return {median:a[a.length>>1],p95:a[Math.floor(a.length*.95)],total:a.reduce((x,y)=>x+y,0)};};
console.log(JSON.stringify({edges:net.edges.length,vertices:r._flowVertexConcentration.length,transport:stats(times.transport),render:stats(times.render),mass:net.getMassBalanceSnapshot()},null,2));r.dispose();

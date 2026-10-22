/** Build a connected, hollow renal tree and matching precompiled collision field.
 * Source assets stay untouched; publish only after collision validation succeeds.
 * node scripts/extend-renal-arteries.mjs Aorta_plain [output-directory]
 */
import fs from 'node:fs';
import path from 'node:path';
import os from 'node:os';
import crypto from 'node:crypto';
import {spawnSync} from 'node:child_process';
import * as THREE from 'three';
import ManifoldModule from 'manifold-3d';
import {Brush,Evaluator,ADDITION,SUBTRACTION} from 'three-bvh-csg';
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader.js';
import {STLExporter} from 'three/examples/jsm/exporters/STLExporter.js';
import {mergeVertices} from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import {transformAortaGeometry} from '../src/aortaTransform.js';
import {generateVessel} from '../src/vesselGeometry.js';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';
import {extendRenalArteries} from '../src/renalArteries.js';
import {taperedTubeGeometry} from './anatomy/cappedArteryTube.mjs';
const [name='Aorta_plain',output='res']=process.argv.slice(2);
if(!['Aorta_plain','Aorta_infrarenal_aneurysm'].includes(name))throw Error('Unknown source anatomy');
fs.mkdirSync(output,{recursive:true});
const sourcePath=path.resolve(`res/${name}.stl`),source=fs.readFileSync(sourcePath);
const hash=b=>crypto.createHash('sha256').update(b).digest('hex');
const bytes=fs.readFileSync(`res/${name}.collision.bin`);
const asset=decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
const d=asset.arrays.centerlineSegments,e=asset.arrays.centerlineEdges;
const native=Array.from({length:e.length/2},(_,i)=>({id:i,start:new THREE.Vector3().fromArray(d,i*9),end:new THREE.Vector3().fromArray(d,i*9+3),radiusStart:d[i*9+6],radiusEnd:d[i*9+7],safeRadius:d[i*9+8],nodeStartId:e[2*i],nodeEndId:e[2*i+1]}));
const extended=extendRenalArteries(native),added=extended.segments.slice(native.length);
if(extended.kidneys.length!==2)throw Error('Missing renal roots');
const geometry=new STLLoader().parse(source.buffer.slice(source.byteOffset,source.byteOffset+source.byteLength));
const transform=transformAortaGeometry(geometry,generateVessel(140,0).vessel);
const module=await ManifoldModule();module.setup();
function solid(g) {
    g.deleteAttribute('normal');const welded=g.index?g:mergeVertices(g,1e-4);
    const mesh=new module.Mesh({numProp:3,vertProperties:new Float32Array(welded.attributes.position.array),triVerts:new Uint32Array(welded.index.array),tolerance:1e-4});mesh.merge();
    return module.Manifold.ofMesh(mesh);
}
const graph=new Map();for(const s of added){if(!graph.has(s.nodeStartId))graph.set(s.nodeStartId,[]);graph.get(s.nodeStartId).push(s);}
const paths=[];
function trace(s) {
    const points=[{position:s.start.clone(),radius:s.radiusStart}];
    while(true){points.push({position:s.end.clone(),radius:s.radiusEnd});const children=graph.get(s.nodeEndId)??[];if(children.length!==1){for(const child of children)trace(child);break;}s=children[0];}
    paths.push(points);
    return points;
}
for(const root of added.filter(s=>!added.some(p=>p.nodeEndId===s.nodeStartId))) {
    const trunk=trace(root);
    // Open the atlas's capped/tapered stump back to its patent parent lumen.
    const connection=[{position:root.start.clone(),radius:root.radiusStart}];
    let id=root.nodeStartId,total=0;
    while(total<14){const parent=native.find(s=>s.nodeEndId===id);if(!parent)throw Error('Missing parent');
        total+=parent.start.distanceTo(parent.end);parent.radiusStart=Math.max(root.radiusStart,parent.radiusStart);parent.radiusEnd=Math.max(root.radiusStart,parent.radiusEnd);parent.safeRadius=Math.min(parent.radiusStart,parent.radiusEnd);
        connection.unshift({position:parent.start.clone(),radius:root.radiusStart});id=parent.nodeStartId;
    }
    // One sweep through the native stump and the new trunk: no coincident
    // tube caps or spherical collar at the atlas/extension boundary.
    trunk.unshift(...connection.slice(0,-1));
}
function tree(wall) {
    const solids=paths.map(points=>solid(taperedTubeGeometry({axialSpacing:0.5,points:points.map(p=>({position:p.position,radius:p.radius+wall}))})));
    // Rounded bifurcations join tubes without internal septa or end caps.
    for(const points of paths)for(const p of [points[0],points.at(-1)])solids.push(module.Manifold.sphere(p.radius+wall,24).translate(p.position.toArray()));
    const result=module.Manifold.union(solids);for(const s of solids)s.delete();return result;
}
console.log(`${name}: union of ${paths.length} renal branches`);
const outer=tree(.4),inner=tree(0);
function brush(manifold) {
    const m=manifold.getMesh(),g=new THREE.BufferGeometry();
    g.setAttribute('position',new THREE.BufferAttribute(new Float32Array(m.vertProperties),3));g.setIndex(new THREE.BufferAttribute(new Uint32Array(m.triVerts),1));
    const b=new Brush(g);b.updateMatrixWorld();return b;
}
// The atlas includes old overlapping cap seams; the same mesh CSG kernel as
// the existing subclavian repair accepts these while Manifold builds the tree.
geometry.deleteAttribute('normal');const base=new Brush(geometry);base.updateMatrixWorld();
const evaluator=new Evaluator();evaluator.attributes=['position'];evaluator.useGroups=false;
const joined=evaluator.evaluate(base,brush(outer),ADDITION);joined.updateMatrixWorld();
const result=evaluator.evaluate(joined,brush(inner),SUBTRACTION).geometry;
result.translate(...transform.targetCenter.map(v=>-v));result.scale(1/transform.scale,1/transform.scale,1/transform.scale);result.rotateX(Math.PI/2);result.translate(...transform.sourceCenter);
const stl=Buffer.from(new STLExporter().parse(new THREE.Mesh(result),{binary:true}).buffer);
const staging=fs.mkdtempSync(path.join(os.tmpdir(),'oet-renal-build-'));
const prefix=path.join(staging,`${name}_renal`);fs.writeFileSync(prefix+'.stl',stl);
const manifest={kind:'renal-extension',sourcePath,sourceSha256:hash(source),outputSha256:hash(stl),segments:extended.segments.map(s=>({...s,start:s.start.toArray(),end:s.end.toArray(),axis:undefined})),kidneys:extended.kidneys};
fs.writeFileSync(prefix+'.json',JSON.stringify(manifest));
const built=spawnSync(process.execPath,['--max-old-space-size=8192','scripts/build-collision-asset.mjs',prefix+'.stl',prefix+'.collision.bin',prefix+'.collision-report.json',prefix+'.json'],{stdio:'inherit'});
if(built.status!==0)throw Error(`Collision asset generation failed; previous assets retained. Diagnostics: ${staging}`);
for(const suffix of ['.stl','.collision.bin','.json','.collision-report.json'])fs.copyFileSync(prefix+suffix,path.join(output,`${name}_renal${suffix}.next`));
for(const suffix of ['.stl','.collision.bin','.json','.collision-report.json'])fs.renameSync(path.join(output,`${name}_renal${suffix}.next`),path.join(output,`${name}_renal${suffix}`));
fs.rmSync(staging,{recursive:true,force:true});
console.log(`Generated ${path.join(output,name+'_renal')}`);

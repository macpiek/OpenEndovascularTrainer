/** Offline outlet closure. The centerline only seeds candidates; radial BVH
 * sections and an unobstructed outward path distinguish open vessel ends from
 * internal skeleton breaks. Finite-thickness plugs overlap the original wall in the
 * common rendered/collision mesh. Original wall triangles remain unchanged. Never writes the input: explicitly pass an output STL path.
 */
import fs from 'node:fs';
import crypto from 'node:crypto';
import * as THREE from 'three';
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader.js';
import {STLExporter} from 'three/examples/jsm/exporters/STLExporter.js';
import {MeshBVH} from 'three-mesh-bvh';
import {decodeCollisionAsset} from '../../src/physics/collision/collisionAssetFormat.js';
import {transformAortaGeometry} from '../../src/aortaTransform.js';
import {generateVessel} from '../../src/vesselGeometry.js';
const [source,assetPath,output,reportPath]=process.argv.slice(2);
if(!reportPath||source===output)throw Error('Usage: close-anatomy-outlets.mjs INPUT.stl COLLISION.bin OUTPUT.stl REPORT.json (distinct input/output)');
const bytes=fs.readFileSync(source),arrayBuffer=b=>b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength);
const g=new STLLoader().parse(arrayBuffer(bytes)),asset=decodeCollisionAsset(arrayBuffer(fs.readFileSync(assetPath)));
const sourceHash=crypto.createHash('sha256').update(bytes).digest('hex');
if(asset.metadata.source.stlSha256!==sourceHash)throw Error('Collision centerline does not belong to the input STL');
const transform=transformAortaGeometry(g,generateVessel(140,0).vessel);g.boundsTree=new MeshBVH(g);
if(asset.metadata.transform.version!==transform.version||Math.abs(asset.metadata.transform.scale-transform.scale)>1e-7)
 throw Error('Collision centerline transform does not match the input geometry');
const tree=g.boundsTree,data=asset.arrays.centerlineSegments,edges=asset.arrays.centerlineEdges,stride=asset.metadata.centerline.stride;
const nodes=new Map();for(let i=0;i<edges.length;i++){if(!nodes.has(edges[i]))nodes.set(edges[i],[]);nodes.get(edges[i]).push(i);}
const ray=new THREE.Ray(),terminals=[],caps=[];
function section(p,n,u,v,count=64,maxDistance=45) {
 const points=[],radii=[];ray.origin.copy(p);
 for(let k=0;k<count;k++) {
  const theta=k/count*2*Math.PI;ray.direction.copy(u).multiplyScalar(Math.cos(theta)).addScaledVector(v,Math.sin(theta));
  const hit=tree.raycastFirst(ray,THREE.DoubleSide,0,maxDistance);
  if(hit){points.push(hit.point.toArray());radii.push(hit.distance);}
 }
 return {points,radii,count:points.length,min:Math.min(...radii),max:Math.max(...radii)};
}
for(const [id,inc]of nodes) {
 if(inc.length!==1)continue;
 const edge=Math.floor(inc[0]/2),end=inc[0]%2;
 const p=new THREE.Vector3().fromArray(data,edge*stride+end*3),q=new THREE.Vector3().fromArray(data,edge*stride+(1-end)*3),n=p.clone().sub(q).normalize();
 ray.origin.copy(p);ray.direction.copy(n);const hit=tree.raycastFirst(ray,THREE.DoubleSide,0,250),axialWall=hit?.distance??null;
 const record={id,point:p.toArray(),normal:n.toArray(),axialWall,closed:false};terminals.push(record);
 if(hit&&hit.distance<=20){record.reason='existing-wall-ahead';continue;}
 const u=new THREE.Vector3(Math.abs(n.x)<.8?1:0,Math.abs(n.x)<.8?0:1,0).cross(n).normalize(),v=n.clone().cross(u);
 let cap;
 // Some source outlets were cropped on an anatomical axis rather than normal
 // to the final skeleton segment. Prefer a complete section nearest that rim.
 const normals=[n.clone()],axis=n.toArray().map(Math.abs).indexOf(Math.max(...n.toArray().map(Math.abs)));
 if(Math.abs(n.getComponent(axis))>.85)normals.push(new THREE.Vector3().setComponent(axis,Math.sign(n.getComponent(axis))));
 search:for(const setback of [0,.5,1,2,4,8,12,20])for(const normal of normals) {
  const su=new THREE.Vector3(Math.abs(normal.x)<.8?1:0,Math.abs(normal.x)<.8?0:1,0).cross(normal).normalize(),sv=normal.clone().cross(su);
  const center=p.clone().addScaledVector(normal,-setback);ray.origin.copy(center);ray.direction.copy(normal);
  if(tree.raycastFirst(ray,THREE.DoubleSide,0,20))continue;
  const ring=section(center,normal,su,sv,192);
  if(ring.count===192&&ring.min>.3&&ring.max<ring.min*10){cap={center,ring,setback,normal,su,sv};break search;}
 }
 if(cap){n.copy(cap.normal);u.copy(cap.su);v.copy(cap.sv);}
 if(!cap){record.reason='no-enclosed-cross-section';continue;}
 const profile=[0,1,2,4,8,12,20,30,40,60].map(d=>({d,hits:section(p.clone().addScaledVector(n,d),n,u,v,64,cap.ring.max*2).count}));
 const open=!hit||profile.some(x=>x.hits<16&&x.d<=Math.max(10,2*cap.ring.max)&&hit.distance>x.d+2*cap.ring.max);
 record.profile=profile;
 if(!open){record.reason='continuous-or-curved-vessel';continue;}
 const ring=section(cap.center,n,u,v,192);
 if(ring.count!==192)throw Error(`Fine cross-section incomplete for outlet ${id}`);
 const overlap=Math.min(.25,ring.min*.3),thickness=.6,points=ring.points.map(point=>{
  const r=new THREE.Vector3().fromArray(point).sub(cap.center);return cap.center.clone().addScaledVector(r,1+overlap/r.length());
 });
 const positions=[],indices=[],count=points.length;
 for(const offset of [0,thickness])for(const point of points)positions.push(...point.clone().addScaledVector(n,offset).toArray());
 const contour=points.map(point=>{const d=point.clone().sub(cap.center);return new THREE.Vector2(d.dot(u),d.dot(v));});
 const faces=THREE.ShapeUtils.triangulateShape(contour,[]);
 if(faces.length!==count-2)throw Error(`Invalid cap polygon ${id}`);
 for(const [a,b,c]of faces)indices.push(c,b,a,count+a,count+b,count+c);
 for(let a=0;a<count;a++){const b=(a+1)%count;indices.push(a,b,count+b,a,count+b,count+a);}
 caps.push({id,center:cap.center.toArray(),normal:n.toArray(),setback:cap.setback,minRadius:ring.min,maxRadius:ring.max,overlap,thickness,positions,indices});
 record.closed=true;record.reason='open-outlet';
}
console.log(JSON.stringify({terminals:terminals.length,caps:caps.length,root:caps.find(c=>c.maxRadius>10)},(k,v)=>['positions','indices'].includes(k)?undefined:v));
const inverse=new THREE.Matrix4().makeTranslation(...transform.sourceCenter).multiply(new THREE.Matrix4().makeRotationX(Math.PI/2)).multiply(new THREE.Matrix4().makeScale(1/transform.scale,1/transform.scale,1/transform.scale)).multiply(new THREE.Matrix4().makeTranslation(...transform.targetCenter.map(v=>-v)));
const positions=[],indices=[];
for(const cap of caps) {
 const offset=positions.length/3;
 for(let i=0;i<cap.positions.length;i+=3)positions.push(...new THREE.Vector3().fromArray(cap.positions,i).applyMatrix4(inverse).toArray());
 for(const i of cap.indices)indices.push(offset+i);
}
const result=new THREE.BufferGeometry();result.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));result.setIndex(indices);
const exported=new STLExporter().parse(new THREE.Mesh(result),{binary:true});
const extra=Buffer.from(exported.buffer,exported.byteOffset,exported.byteLength),originalTriangles=bytes.readUInt32LE(80);
if(bytes.length!==84+50*originalTriangles)throw Error('Expected binary source STL');
const outputBytes=Buffer.concat([bytes,extra.subarray(84)]);outputBytes.writeUInt32LE(originalTriangles+indices.length/3,80);fs.writeFileSync(output,outputBytes);
const report={source,sourceSha256:crypto.createHash('sha256').update(bytes).digest('hex'),output,outputSha256:crypto.createHash('sha256').update(outputBytes).digest('hex'),
 method:'Finite-thickness closed plugs overlapping original wall; shared rendered/collision triangle mesh, not a Boolean solid union.',
 originalTriangles,addedTriangles:indices.length/3,triangles:originalTriangles+indices.length/3,
 terminals,caps:caps.map(({positions,indices,...cap})=>cap)};fs.writeFileSync(reportPath,JSON.stringify(report,null,2));
console.log(JSON.stringify({output,caps:caps.length,triangles:report.triangles,addedTriangles:report.addedTriangles}));g.dispose();result.dispose();

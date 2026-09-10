import fs from 'node:fs';
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader.js';
import {MeshBVH} from 'three-mesh-bvh';
import {generateVessel} from '../../src/vesselGeometry.js';
import {transformAortaGeometry} from '../../src/aortaTransform.js';
import {VesselContactField,createContactResult} from '../../src/physics/collision/vesselContactField.js';
import {decodeCollisionAsset} from '../../src/physics/collision/collisionAssetFormat.js';
const bytes=fs.readFileSync('res/Aorta_plain.collision.bin');
const asset=decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
// Captured step663, real app STL/BVH. Optional argv[2] changes only the public
// capsule BVH validation threshold (default -.1). No private query mutation.
const threshold=Number(process.argv[2]??-.1);
const stl=fs.readFileSync('res/Aorta_plain.stl');
const geometry=new STLLoader().parse(stl.buffer.slice(stl.byteOffset,stl.byteOffset+stl.byteLength));
transformAortaGeometry(geometry,generateVessel(140,0).vessel);geometry.boundsTree=new MeshBVH(geometry);
const field=new VesselContactField(asset,{fallbackGeometry:geometry,bvhValidationDistance:.02,capsuleBvhValidation:threshold});
const base=[[-3.9944071769714355,-234.25193786621094,9.069884300231934],[-4.901269912719727,-236.12643432617188,11.50005054473877]];
const rejected=[[-3.9944465160369873,-234.25192260742188,9.069734573364258],[-4.901449680328369,-236.12643432617188,11.499829292297363]];
const radius=.8333333134651184,direction=base.map((p,j)=>p.map((v,i)=>rejected[j][i]-v));
function query(points){const c=field.queryCapsuleSoA(...[0,1,2].map(i=>Float64Array.from(points,p=>p[i])),new Float64Array([radius,radius]),0,createContactResult(),-1,false,false,351,false,-1,0,true);return {gap:c.signedGap,t:c.segmentT,branch:c.branchId,source:c.source,face:c.faceIndex,normal:Array.from(c.inward.values)};}
const reports=[];
for(const [label,points] of [['base',base],['rejected',rejected]]){
 const contact=query(points),weights=[1-contact.t,contact.t],predicted=direction.reduce((sum,d,j)=>sum+weights[j]*d.reduce((v,x,i)=>v+x*contact.normal[i],0),0);
 const steps=[1e-4,1e-5,1e-6,1e-7].map(h=>{
  const gradient=points.map((p,j)=>p.map((_,axis)=>{const a=points.map(p=>p.slice()),b=points.map(p=>p.slice());a[j][axis]+=h;b[j][axis]-=h;return(query(a).gap-query(b).gap)/(2*h);}));
  const actual=gradient.reduce((sum,g,j)=>sum+g.reduce((v,x,i)=>v+x*direction[j][i],0),0);
  return {h,gradient,gradientNorm:Math.hypot(...gradient.flat()),predictedFromUnitNormal:predicted,actualDirectionalDerivative:actual};
 });reports.push({label,points,contact,steps});
}
const zBoundary=11.5,transition=(base[1][2]-zBoundary)/(base[1][2]-rejected[1][2]);
const sides=[-1e-6,1e-6].map(delta=>{const f=transition+delta,points=base.map((p,j)=>p.map((v,i)=>v+f*direction[j][i]));return {fraction:f,contact:query(points)};});
console.log(JSON.stringify({capsuleBvhValidation:threshold,reports,gridPlaneTransition:{zBoundary,transition,sides},sdf:{voxelSize:asset.metadata.sdf.voxelSize,origin:asset.metadata.sdf.origin}},null,2));

import fs from 'node:fs';
import crypto from 'node:crypto';
import path from 'node:path';
import * as THREE from 'three';
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader.js';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';
import {transformAortaGeometry} from '../src/aortaTransform.js';
import {generateVessel} from '../src/vesselGeometry.js';
import {INFRARENAL_ANEURYSM,createInfrarenalDeformation,axialSection} from './anatomy/infrarenalAneurysm.mjs';
import {selectInfrarenalSurface,selectInfrarenalCenterline} from './anatomy/infrarenalSelection.mjs';
const sourcePath='res/Aorta_plain.stl',outputPath='res/Aorta_infrarenal_aneurysm.stl';
// The rebuild pipeline stages all outputs until the matching collision asset
// is ready, so the dev server keeps serving a complete previous variant.
const outputDirectory=process.argv[2] || 'res';
fs.mkdirSync(outputDirectory,{recursive:true});
const outputFile=name=>path.join(outputDirectory,name);
const hash=b=>crypto.createHash('sha256').update(b).digest('hex');
const source=fs.readFileSync(sourcePath),sourceSha256=hash(source);
if(sourceSha256!==INFRARENAL_ANEURYSM.sourceSha256)throw Error('Infrarenal landmarks need updating for this source STL');
const arrayBuffer=b=>b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength);
const asset=decodeCollisionAsset(arrayBuffer(fs.readFileSync('res/Aorta_plain.collision.bin')));
if(asset.metadata.source.stlSha256!==sourceSha256)throw Error('Baseline collision asset does not match source');
const data=asset.arrays.centerlineSegments,edges=asset.arrays.centerlineEdges,nodes=new Map();
for(let i=0;i<edges.length/2;i++) for(let k=0;k<2;k++) {
    const id=edges[2*i+k];
    if(!nodes.has(id))nodes.set(id,{point:Array.from(data.slice(9*i+3*k,9*i+3*k+3)),adj:[]});
    nodes.get(id).adj.push(edges[2*i+1-k]);
}
// The uninterrupted trunk between the iliac bifurcation and lower renal origin.
const axis=[nodes.get(1511).point];let previous=1511;
let current=nodes.get(previous).adj.reduce((a,b)=>nodes.get(a).point[1]>nodes.get(b).point[1]?a:b);
if(!nodes.get(previous).adj.includes(current))throw Error('Infrarenal centerline topology changed');
while(current!==1566) {
    axis.push(nodes.get(current).point);
    const next=nodes.get(current).adj.filter(id=>id!==previous);
    if(next.length!==1 || axis.length>100)throw Error('Unexpected branch in infrarenal trunk');
    previous=current;current=next[0];
}
axis.push(nodes.get(current).point);
const geometry=new STLLoader().parse(arrayBuffer(source));
const transform=transformAortaGeometry(geometry,generateVessel().vessel);
const midY=INFRARENAL_ANEURYSM.referenceY;
const provisional=createInfrarenalDeformation({axis,referenceDiameterMm:18});
const referenceDiameterMm=axialSection(geometry.attributes.position.array,midY,provisional.centerAt(midY)).diameter;
const deformation=createInfrarenalDeformation({axis,referenceDiameterMm});
const surfaceSelection=selectInfrarenalSurface(geometry.attributes.position.array,INFRARENAL_ANEURYSM,deformation.centerAt);
const selectedCenterlineSegments=selectInfrarenalCenterline(asset,INFRARENAL_ANEURYSM,deformation.centerAt);
const result=Buffer.from(source),position=geometry.attributes.position,triangles=source.readUInt32LE(80);
let movedVertices=0,changedTriangles=0;
const a=new THREE.Vector3(),b=new THREE.Vector3(),c=new THREE.Vector3();
for(let i=0;i<triangles;i++) {
    let changed=false;const base=84+i*50;
    for(let k=0;k<3;k++) {
        const index=i*3+k,x=position.getX(index),y=position.getY(index),z=position.getZ(index);
        if(!surfaceSelection[index])continue;
        const p=deformation.move(x,y,z);
        if(Math.hypot(p[0]-x,p[1]-y,p[2]-z)<1e-9)continue;
        changed=true;movedVertices++;
        const offset=base+12+k*12,t=transform;
        result.writeFloatLE((p[0]-t.targetCenter[0])/t.scale+t.sourceCenter[0],offset);
        result.writeFloatLE(-(p[2]-t.targetCenter[2])/t.scale+t.sourceCenter[1],offset+4);
        result.writeFloatLE((p[1]-t.targetCenter[1])/t.scale+t.sourceCenter[2],offset+8);
    }
    if(!changed)continue;changedTriangles++;
    for(let k=0;k<3;k++)[a,b,c][k].set(result.readFloatLE(base+12+k*12),result.readFloatLE(base+16+k*12),result.readFloatLE(base+20+k*12));
    b.sub(a).cross(c.sub(a)).normalize();
    for(let k=0;k<3;k++)result.writeFloatLE(b.getComponent(k),base+k*4);
}
const generated=new STLLoader().parse(arrayBuffer(result));transformAortaGeometry(generated,generateVessel().vessel);
const diameterMm=axialSection(generated.attributes.position.array,midY,deformation.centerAt(midY),18,geometry.attributes.position.array).diameter;
if(Math.abs(diameterMm-INFRARENAL_ANEURYSM.targetDiameterMm)>1)throw Error(`Unexpected aneurysm diameter: ${diameterMm}`);
fs.writeFileSync(outputFile(path.basename(outputPath)),result);
const report={...INFRARENAL_ANEURYSM,sourcePath,outputPath,outputSha256:hash(result),transform,axis,
    selection:'connected-infrarenal-slab',selectedCenterlineSegments,
    referenceDiameterMm,measuredDiameterMm:diameterMm,triangles,movedVertices,changedTriangles,
    reference:'https://www.med.umich.edu/1libr/Surgery/VascularSurgery/Illustrations/TypesofAAA.pdf'};
fs.writeFileSync(outputFile('Aorta_infrarenal_aneurysm.json'),JSON.stringify(report,null,2)+'\n');
// The deformation inherits the repaired baseline connectivity. Retain the
// repair provenance, while identifying the actual generated surface.
const repair=JSON.parse(fs.readFileSync('res/Aorta_plain.mesh-repair.json'));
if(repair.outputSha256!==sourceSha256)throw Error('Baseline mesh repair manifest is stale');
fs.writeFileSync(outputFile('Aorta_infrarenal_aneurysm.mesh-repair.json'),JSON.stringify({...repair,
    outputPath,outputSha256:report.outputSha256,
    deformation:{sourceSha256,outputSha256:report.outputSha256,report:'res/Aorta_infrarenal_aneurysm.json'}
},null,2)+'\n');
console.log({outputPath,diameterMm,referenceDiameterMm,changedTriangles,sha256:report.outputSha256});

import fs from 'node:fs';
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader.js';
import {MeshBVH} from 'three-mesh-bvh';
import {generateVessel} from '../../src/vesselGeometry.js';
import {transformAortaGeometry} from '../../src/aortaTransform.js';
import {decodeCollisionAsset} from '../../src/physics/collision/collisionAssetFormat.js';
import {VesselContactField,createContactResult} from '../../src/physics/collision/vesselContactField.js';

// Input: {base:{a:[x,y,z],b:[x,y,z],radii:[r0,r1]},rejected:{...},baseBranch:354}.
// Both real endpoint states are required; interpolation never fabricates a
// private field result. Run from repository root with input path as argv[2].
if(!process.argv[2])throw new Error('Pass JSON with base/rejected capsule endpoints and baseBranch');
const input=JSON.parse(fs.readFileSync(process.argv[2],'utf8'));
for(const state of [input.base,input.rejected])for(const key of ['a','b','radii'])
    if(!Array.isArray(state?.[key])||state[key].length!==(key==='radii'?2:3)||!state[key].every(Number.isFinite))throw new Error(`Invalid ${key}`);
const buffer=path=>{const b=fs.readFileSync(path);return b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength);};
const asset=decodeCollisionAsset(buffer('res/Aorta_plain.collision.bin'));
const geometry=new STLLoader().parse(buffer('res/Aorta_plain.stl'));
const {vessel}=generateVessel(140,0);transformAortaGeometry(geometry,vessel);
geometry.computeVertexNormals();geometry.computeBoundingSphere();geometry.boundsTree=new MeshBVH(geometry);
const fields=[['app',new VesselContactField(asset,{fallbackGeometry:geometry,bvhValidationDistance:.02,capsuleBvhValidation:-.1})],
    ['without-bvh',new VesselContactField(asset,{bvhValidationDistance:.02,capsuleBvhValidation:-.1})]];
const snapshot=c=>({gap:c.signedGap,t:c.segmentT,branch:c.branchId,face:c.faceIndex,source:c.source,
    samples:c.capsuleSampleCount,normal:Array.from(c.inward.values),closest:Array.from(c.closestPoint.values)});
const reports=[];
for(const [name,field] of fields)for(const ArrayType of [Float32Array,Float64Array]) {
    const contact=createContactResult();
    const query=fraction=>{
        const coordinates=['a','b'].map(key=>input.base[key].map((v,i)=>v+fraction*(input.rejected[key][i]-v)));
        const arrays=[0,1,2].map(axis=>new ArrayType(coordinates.map(p=>p[axis])));
        const radii=new ArrayType(input.base.radii.map((v,i)=>v+fraction*(input.rejected.radii[i]-v)));
        const result=field.queryCapsuleSoA(...arrays,radii,0,contact,-1,false,false,input.baseBranch??-1,false,-1,0,input.physicalGap===true);
        return {fraction,...snapshot(result),a:arrays.map(v=>v[0]),b:arrays.map(v=>v[1])};
    };
    const sweep=Array.from({length:33},(_,i)=>query(i/32));
    const transitions=[];
    for(let i=1;i<sweep.length;i++) {
        const signature=c=>`${c.t}:${c.branch}:${c.source}`;
        if(signature(sweep[i-1])===signature(sweep[i]))continue;
        let lo=sweep[i-1],hi=sweep[i];const key=signature(lo);
        for(let k=0;k<35;k++) {
            const mid=query((lo.fraction+hi.fraction)/2);
            if(signature(mid)===key)lo=mid;else hi=mid;
        }
        transitions.push({lo,hi,gapJump:hi.gap-lo.gap,endpointDisplacement:Math.max(...['a','b'].map(key=>Math.hypot(...hi[key].map((v,j)=>v-lo[key][j]))))});
    }
    reports.push({configuration:name,storage:ArrayType.name,sweep,transitions});
}
console.log(JSON.stringify({input,reports},null,2));

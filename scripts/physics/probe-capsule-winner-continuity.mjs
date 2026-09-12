import assert from 'node:assert/strict';
import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {VesselContactField,createContactResult} from '../../src/physics/collision/vesselContactField.js';

// Public asset/geometry inputs only. The sparse grid is an approximate positive
// distance field d=.5+.04*x*y. Its two endpoint estimates exchange order at y=0.
// A real triangle BVH represents a nearby tilted planar wall z=.5+.2*x, so the
// endpoint refinement errors differ. Maximum local discrepancy is sub-voxel.
const brickSize=4,distances=new Uint8Array(brickSize**3);
for(let z=0;z<brickSize;z++)for(let y=0;y<brickSize;y++)for(let x=0;x<brickSize;x++)
    distances[x+brickSize*(y+brickSize*z)]=Math.round((.5+.04*(x-2)*(y-2))/.01);
const asset={metadata:{version:1,decodedBytes:1024,
    centerline:{stride:9,segmentCount:1,nodeCount:2},
    broadPhase:{cellSize:16,origin:[-8,-8,-8],dimensions:[1,1,1]},
    sdf:{voxelSize:1,brickSize,band:4,quantization:.01,distanceQuantization:.01,origin:[-2,-2,-2],dimensions:[1,1,1],brickCount:1},sections:[]},
    arrays:{centerlineSegments:new Float32Array([0,-4,0,0,4,0,.55,.55,0]),centerlineEdges:new Uint32Array([0,1]),
        broadPhaseOffsets:new Uint32Array([0,1]),broadPhaseIds:new Uint32Array([0]),sdfBrickKeys:new Uint32Array([0]),
        sdfDistances:distances,sdfInsideBits:new Uint8Array(8).fill(255)}};
const geometry=new THREE.PlaneGeometry(8,8);
const position=geometry.attributes.position;
for(let i=0;i<position.count;i++)position.setZ(i,.5+.2*position.getX(i));
geometry.boundsTree=new MeshBVH(geometry);
const coarse=new VesselContactField(asset,{capsuleBvhValidation:false});
const refined=new VesselContactField(asset,{fallbackGeometry:geometry,capsuleBvhValidation:true});
const snapshot=c=>({gap:c.signedGap,t:c.segmentT,samples:c.capsuleSampleCount,source:c.source,face:c.faceIndex});
const radius=.5,length=.8,samples=1;
function query(field,epsilon) {
    const x=new Float64Array([-.4,.4]),y=new Float64Array([epsilon,epsilon]),z=new Float64Array([0,0]);
    return snapshot(field.queryCapsuleSoA(x,y,z,new Float64Array([radius,radius]),0,createContactResult(),-1,false,false,-1,false,length,samples));
}
const rows=[];
for(const epsilon of [1e-2,1e-3,1e-4,1e-5,1e-6,1e-7]) {
    const coarseMinus=query(coarse,-epsilon),coarsePlus=query(coarse,epsilon),minus=query(refined,-epsilon),plus=query(refined,epsilon);
    rows.push({epsilon,coarseMinus,coarsePlus,minus,plus,refinedGapJump:Math.abs(plus.gap-minus.gap)});
}
assert.ok(rows.every(row=>row.coarseMinus.t!==row.coarsePlus.t),'coarse winner must switch');
assert.ok(rows.every(row=>row.minus.source==='sparse-sdf-bvh'&&row.plus.source==='sparse-sdf-bvh'),'both winners must be genuinely refined');
assert.ok(rows.every(row=>row.minus.samples===samples&&row.plus.samples===samples),'sample count must remain fixed');
assert.ok(rows.at(-1).refinedGapJump>.1,'a nonvanishing refinement jump must remain as epsilon tends to zero');
assert.ok(Math.abs(rows.at(-1).refinedGapJump-rows[0].refinedGapJump)<1e-5);
console.log(JSON.stringify({fixture:'synthetic sparse-SDF vs real tilted-plane MeshBVH',fixedSampleCount:samples,radius,length,
    conclusion:'Coarse-winner switching produces a nonvanishing refined gap jump in the public capsule query.',rows},null,2));

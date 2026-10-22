import test from 'node:test';
import fs from 'node:fs';
import * as THREE from 'three';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';
import {VesselContactField} from '../src/physics/collision/vesselContactField.js';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';
import {generateVessel} from '../src/vesselGeometry.js';
import assert from 'node:assert/strict';
import {prepareIntroducerExchange,prepareIntroducerMotion} from '../src/devices/introducerSheath.js';
import {sheathLength} from '../src/physics/sheathPath.js';

const short={start:{x:0,y:0,z:0},end:{x:0,y:70,z:0},length:450,radius:1,french:6};
// Curved tube: 300 mm bend radius, 4 mm lumen radius.
function contactAt(p) {
    const dy=p.y-70,dz=p.z-300,r=Math.hypot(dy,dz);
    const cy=70+300*dy/r,cz=300+300*dz/r;
    const dx=-p.x,ny=cy-p.y,nz=cz-p.z,d=Math.hypot(dx,ny,nz);
    return {signedDistance:4-d,normal:d?{x:dx/d,y:ny/d,z:nz/d}:{x:0,y:0,z:1}};
}

for(const length of [450,900]) test(`${length} mm sheath bends and advances beyond the support tip`,()=>{
    const result=prepareIntroducerExchange({sheath:short,length,insertion:length,french:6,
        transport:{progress:100},sampleWire:s=>({x:0,y:s,z:0}),contactAt,
        clearanceAt:p=>contactAt(p).signedDistance});
    assert.equal(sheathLength(result),length);
    assert.ok(result.end.z>150,'the free tip must turn with the curved vessel');
    for(let i=1;i<result.path.length;i++) {
        const a=result.path[i-1],b=result.path[i];
        if(a.s<100)continue;
        assert.ok(Math.abs(Math.hypot(b.x-a.x,b.y-a.y,b.z-a.z)-(b.s-a.s))<1e-6,
            'bending must not shorten the feed or stretch the sheath');
        for(const t of [0,.5,1]) {
            const p={x:a.x+(b.x-a.x)*t,y:a.y+(b.y-a.y)*t,z:a.z+(b.z-a.z)*t};
            assert.ok(contactAt(p).signedDistance>=1.02,'the whole sheath stays inside the lumen');
        }
    }
    const withdrawn=prepareIntroducerMotion({sheath:result,transport:{progress:0},contactAt},-200);
    assert.equal(sheathLength(withdrawn),length-200);
    assert.equal(withdrawn.length,length);
});

test('small manual increments advance a bare sheath around a bend without a wire',()=>{
    let sheath=short;
    for(let i=0;i<320;i++)sheath=prepareIntroducerMotion({sheath,transport:{progress:0},contactAt},.5);
    assert.equal(sheathLength(sheath),230);
    assert.ok(sheath.end.z>25);
    assert.ok(contactAt(sheath.end).signedDistance>=1.02);
});

test('a free tip stops at a closed end instead of tunnelling or accumulating insertion',()=>{
    const capped=p=>p.y>117
        ? {signedDistance:120-p.y,normal:{x:0,y:-1,z:0}}
        : {signedDistance:3,normal:{x:1,y:0,z:0}};
    const options={sheath:short,transport:{progress:0},contactAt:capped};
    const accepted=prepareIntroducerMotion(options,48);
    assert.equal(sheathLength(accepted),118);
    assert.throws(()=>prepareIntroducerMotion({...options,sheath:accepted},2),/ścian/);
    assert.equal(sheathLength(accepted),118,'failed feed leaves the accepted pose intact');
});


test('both native femoral paths accept a 45 cm sheath beyond a 37 cm wire',()=>{
    const bytes=fs.readFileSync(new URL('../res/Aorta_plain_renal.collision.bin',import.meta.url));
    const asset=decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
    const data=asset.arrays.centerlineSegments,links=asset.arrays.centerlineEdges;
    const segments=Array.from({length:links.length/2},(_,id)=>({id,nodeStartId:links[2*id],nodeEndId:links[2*id+1],
        start:new THREE.Vector3().fromArray(data,9*id),end:new THREE.Vector3().fromArray(data,9*id+3),
        radiusStart:data[9*id+6],radiusEnd:data[9*id+7]}));
    const network=new ContrastFlowNetwork(segments),field=new VesselContactField(asset);
    for(const side of ['right','left']) {
        let sheath={...generateVessel().vessel.sheaths[side],length:450};
        const route=[{...sheath.end,s:70}];
        const location=network.findNearestLocation(new THREE.Vector3(sheath.end.x,sheath.end.y,sheath.end.z),{});
        // A deployed support route following the actual femoral-to-aortic tree.
        for(let edge=network.edges[location.edgeIndex];edge;edge=network.edges[edge.parentEdgeIndex]) {
            const p=edge.start,last=route.at(-1),ds=Math.hypot(p.x-last.x,p.y-last.y,p.z-last.z);
            if(ds>1e-6)route.push({x:p.x,y:p.y,z:p.z,s:last.s+ds});
        }
        const sampleWire=s=>{
            let i=1;while(i<route.length-1&&route[i].s<s)i++;
            const a=route[i-1],b=route[i],t=(s-a.s)/(b.s-a.s);
            return{x:a.x+(b.x-a.x)*t,y:a.y+(b.y-a.y)*t,z:a.z+(b.z-a.z)*t};
        };
        for(let step=0;step<760;step++)sheath=prepareIntroducerMotion({sheath,
            transport:{progress:370},sampleWire,contactAt:p=>field.querySphere(p,0)},.5);
        assert.equal(sheathLength(sheath),450,side);
        for(let i=1;i<sheath.path.length;i++) {
            assert.ok(sheath.path[i].s-sheath.path[i-1].s>1e-7,'no degenerate terminal cell from roundoff');
            if(sheath.path[i].s>70)assert.ok(field.querySphere(sheath.path[i],0).signedDistance>=1.049);
        }
        const withdrawn=prepareIntroducerMotion({sheath,transport:{progress:0}},-450);
        assert.equal(sheathLength(withdrawn),0);
    }
});

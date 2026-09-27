import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {fitGraftJunction} from '../src/devices/stentGraftJunction.js';
import {fitExpandedGraft} from '../src/devices/stentGraftExpansion.js';
import {DevicePath} from '../src/devices/stentGraftPaths.js';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';

const free={fit:p=>p};
function tube(x,from,to,radius) {
    const points=Array.from({length:Math.round(to-from)+1},(_,i)=>new THREE.Vector3(x,from+i,0));
    const part={points,rows:points.length,sides:24,radius,target:new Float32Array(points.length*24*3),path:new DevicePath(points)};
    fitExpandedGraft(part,free);return part;
}
test('bifurcation follows a compressed noncircular trunk instead of re-expanding to nominal diameter',()=>{
    const trunk=tube(0,-20,0,14),left=tube(-3.5,0,30,3.5),right=tube(3.5,0,30,3.5);
    for(let k=0;k<trunk.target.length;k+=3){trunk.target[k]*=8/14;trunk.target[k+2]*=5/14;}
    const distal=[left,right].map(p=>Array.from(p.target.slice(20*p.sides*3)));
    fitGraftJunction([trunk,left,right],free);
    for(const [i,part] of [left,right].entries()) {
        const first=part.target.slice(0,part.sides*3);
        for(let k=0;k<first.length;k+=3){
            assert.ok(Math.abs(first[k])<=8.00001,'no shelf beyond actual trunk width');
            assert.ok(Math.abs(first[k+2])<=5.00001,'no shelf beyond actual trunk depth');
        }
        assert.deepEqual(Array.from(part.target.slice(20*part.sides*3)),distal[i],'distal calibre is preserved');
        for(let row=1;row<part.rows;row++)for(let j=0;j<part.sides;j++) {
            const a=new THREE.Vector3().fromArray(part.target,(row*part.sides+j)*3);
            const b=new THREE.Vector3().fromArray(part.target,((row-1)*part.sides+j)*3);
            assert.ok(Math.hypot(a.x-b.x,a.z-b.z)<.8,'no abrupt lateral jump between sections');
        }
    }
});
test('higher continuous rounded stent waves retain thin wire and close each ring',()=>{
    const {system,device}=previewFixture();
    try {
        const part=device.parts[0],segments=part.scaffoldSegments;
        const ring=segments.slice(0,part.sides*12);
        assert.ok(Math.max(...ring.flat().map(p=>p.s))-Math.min(...ring.flat().map(p=>p.s))>=7.99);
        const firstRing=segments.slice(0,part.sides*12);
        for(let i=1;i<firstRing.length;i++)assert.deepEqual(firstRing[i][0],firstRing[i-1][1]);
        assert.equal(firstRing.at(-1)[1].s,firstRing[0][0].s);
        assert.equal(part.rings.userData.wireRadius,.045);
        const slopes=firstRing.slice(0,12).map(([a,b])=>(b.s-a.s)/(b.a-a.a));
        assert.ok(slopes[0]<slopes[5]/5&&slopes.at(-1)<slopes[5]/5,'crowns curve continuously into their adjoining arms');
    }finally{system.dispose();}
});

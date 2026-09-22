import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {fitExpandedGraft,relaxGraftAxis} from '../src/devices/stentGraftExpansion.js';
import {StentGraftWallFit} from '../src/devices/stentGraftWallFit.js';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {rotateGraftPose} from '../src/devices/stentGraftRotation.js';

const radiusAt=(part,i,j=0)=>new THREE.Vector3().fromArray(part.target,(i*part.sides+j)*3).distanceTo(part.points[i]);
test('self-expansion seeks nominal diameter, smooths a local constriction, and recovers when unconstrained',()=>{
    let narrow=true;
    const wall=new StentGraftWallFit({contactField:{querySphere(p,clearance,out){
        const limit=narrow&&Math.abs(p.y)<4?6:30,r=Math.hypot(p.x,p.z);
        out.violation=r+clearance>limit;out.penetration=Math.max(0,r+clearance-limit);
        Object.assign(out.normal,{x:-p.x/Math.max(r,1e-12),y:0,z:-p.z/Math.max(r,1e-12)});return out;
    }}});
    const points=Array.from({length:61},(_,i)=>new THREE.Vector3(0,i*2-60,0));
    const part={points,rows:points.length,sides:24,radius:10,target:new Float32Array(points.length*24*3)};
    fitExpandedGraft(part,wall);
    assert.ok(Math.abs(radiusAt(part,0)-10)<1e-4,'free graft does not expand to fill a 60 mm aneurysm');
    assert.ok(radiusAt(part,30)<=5.301,'wall limits the nominal 20 mm graft');
    assert.ok(radiusAt(part,27)<9.5&&radiusAt(part,27)>5.3,'neighbouring sections share the contact load');
    let jump=0;
    for(let i=1;i<part.rows;i++)jump=Math.max(jump,Math.abs(radiusAt(part,i)-radiusAt(part,i-1)));
    assert.ok(jump<1.5,`no isolated pinched row: maximum radius change ${jump}`);
    narrow=false;fitExpandedGraft(part,wall);
    for(let i=0;i<part.rows;i++)for(let j=0;j<part.sides;j++)assert.ok(Math.abs(radiusAt(part,i,j)-10)<1e-5,'rest radius is recovered rather than preserving an old dent');
});

test('axial relaxation removes local zigzags while preserving landing and branch ends',()=>{
    const points=Array.from({length:31},(_,i)=>new THREE.Vector3(Math.sin(i*.2)*4+(i%2?.8:-.8),i*2,0));
    const energy=()=>points.slice(1,-1).reduce((e,p,j)=>e+p.clone().multiplyScalar(2).sub(points[j]).sub(points[j+2]).lengthSq(),0);
    const before=energy(),ends=[points[0].clone(),points.at(-1).clone()];
    relaxGraftAxis(points,new StentGraftWallFit());
    assert.ok(energy()<before*.03);assert.deepEqual(points[0],ends[0]);assert.deepEqual(points.at(-1),ends[1]);
});

test('large roll preserves round planar sections on a curved free graft and returning to zero restores shape',()=>{
    const {system,device:d}=previewFixture();
    try {
        const before=d.parts[0].target.slice();
        for(const angle of [-166*Math.PI/180,Math.PI/2,0]) {
            rotateGraftPose(d,angle);
            const p=d.parts[0];
            for(let i=0;i<p.rows;i++)for(let j=0;j<p.sides;j++) {
                const radial=new THREE.Vector3().fromArray(p.target,(i*p.sides+j)*3).sub(p.points[i]);
                assert.ok(Math.abs(radial.length()-p.radius)<2e-5,'rolling cannot crumple a free ring');
                assert.ok(Math.abs(radial.dot(p.ringFrames[i].tangent))<2e-5,'all vertices retain one section plane');
            }
        }
        for(let k=0;k<before.length;k++)assert.ok(Math.abs(d.parts[0].target[k]-before[k])<2e-5);
    } finally {system.dispose();}
});

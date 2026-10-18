import test from 'node:test';
import assert from 'node:assert/strict';
import {Quaternion,Vector3} from 'three';
import {projectRotation,quaternionColumns} from '../src/physics/projectiveRodMath.js';
import {projectRotationPolar} from '../src/physics/projectiveRotationPolar.js';

test('polar projection matches the closest proper rotation for distorted near-rotation frames',()=>{
    let seed=217;const random=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/2**32;};
    for(let trial=0;trial<1000;trial++) {
        const axis=new Vector3(random()-.5,random()-.5,random()-.5).normalize();
        const q=new Quaternion().setFromAxisAngle(axis,trial<2?trial*Math.PI:random()*Math.PI*2);
        const columns=quaternionColumns(q).map(c=>c.map(v=>v+(random()-.5)*.4));
        const result=projectRotationPolar(columns),reference=projectRotation(columns);
        for(let c=0;c<3;c++)for(let k=0;k<3;k++)assert.ok(Math.abs(result.columns[c][k]-reference.columns[c][k])<1e-10);
        assert.ok(Math.abs(Math.hypot(...Object.values(result.quaternion))-1)<1e-14);
        const determinant=new Vector3(...result.columns[0]).cross(new Vector3(...result.columns[1])).dot(new Vector3(...result.columns[2]));
        assert.ok(Math.abs(determinant-1)<1e-13);
    }
});

test('reflections, singular and ill-conditioned directors retain the reference Procrustes solution',()=>{
    for(const columns of [[[1,0,0],[0,1,0],[0,0,-1]],[[1,0,0],[1,0,0],[0,0,0]],
        [[1e-12,0,0],[0,1,0],[0,0,1]],[[1e8,0,0],[0,1e8,0],[0,0,1e8]]]) {
        assert.deepEqual(projectRotationPolar(columns),projectRotation(columns));
    }
});

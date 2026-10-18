import test from 'node:test';
import assert from 'node:assert/strict';
import {Quaternion,Vector3} from 'three';
import {projectRotation,quaternionColumns} from '../src/physics/projectiveRodMath.js';
import {createProjectRotationScratch} from '../src/physics/projectiveRotationScratch.js';

test('scratch Jacobi is bit-identical on distorted frames, reflections, singularities and pi rotations',()=>{
    const project=createProjectRotationScratch();let seed=921;
    const random=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/2**32;};
    const frames=[[[1,0,0],[0,1,0],[0,0,-1]],[[1,0,0],[1,0,0],[0,0,0]],[[1e-12,0,0],[0,1,0],[0,0,1]],
        [[0,0,0],[0,0,0],[0,0,0]],[[1e8,0,0],[0,1e8,0],[0,0,1e8]]];
    for(let i=0;i<1000;i++) {
        const q=new Quaternion().setFromAxisAngle(new Vector3(random()-.5,random()-.5,random()-.5).normalize(),i<2?i*Math.PI:random()*Math.PI*2);
        frames.push(quaternionColumns(q).map(c=>c.map(x=>x+(random()-.5)*.4)));
    }
    for(const columns of frames){const saved=structuredClone(columns);assert.deepEqual(project(columns),projectRotation(columns));assert.deepEqual(columns,saved);}
});

test('workspaces and returned rotation targets stay independent across calls',()=>{
    const a=createProjectRotationScratch(),b=createProjectRotationScratch();
    const frame=[[1,.2,.1],[.1,1,-.2],[0,.1,1]],saved=a(frame),copy=structuredClone(saved);
    for(let i=0;i<10;i++)b([[1,0,0],[0,0,1],[0,-1,0]]);
    a([[1,0,0],[0,1,0],[0,0,-1]]);assert.deepEqual(saved,copy);assert.deepEqual(a(frame),saved);
});

import test from 'node:test';
import assert from 'node:assert/strict';
import {prepareSharedAxisActiveBasis,sharedAxisBasisLinearization} from '../src/physics/kirchhoffSharedAxisActiveBasis.js';

test('basis reuse preserves elimination order, fill-in, dependent pivots and trace',()=>{
    for(const count of [9,33,70,180])for(const variant of [0,1,2]) {
        const rows=[];
        for(let j=0;j<count;j++)rows.push({kind:'length',dofs:[j,j+1],jacobian:[-1,1],gap:0,multiplier:Math.sin(j)});
        for(let j=0;j<count;j++)rows.push({kind:'wall',id:`wall/${j}`,dofs:[j,j+1],jacobian:[.25,.75],gap:variant===0?0:.1+(j%3)*.01,multiplier:1+j%5});
        rows.push({...rows.at(-1),id:'dependent',gap:variant===2?.05:rows.at(-1).gap});
        const fixed=new Uint8Array(count+1);fixed[0]=1;const outputs=[];
        for(const reuse of [false,true])for(const repeat of [0,1]) {
            const activeSet=new Uint8Array(rows.length).fill(1),dual=Float64Array.from(rows,r=>r.multiplier),trace=[];
            const result=prepareSharedAxisActiveBasis({rows,fixed,activeSet,dual,trace,basisCache:reuse?sharedAxisBasisLinearization(rows,fixed):Symbol()});
            outputs.push({result,activeSet,dual,trace});
        }
        for(const output of outputs.slice(1))assert.deepEqual(output,outputs[0]);
    }
});

test('basis token ignores reactions/gaps but invalidates changed Jacobians, supports, kind and fixed values',()=>{
    const rows=[{kind:'wall',dofs:[0,1],jacobian:[.5,.5],gap:0,multiplier:1}],fixed=new Uint8Array(3);
    let token=sharedAxisBasisLinearization(rows,fixed);
    rows[0].gap=2;rows[0].multiplier=10;rows[0].geometricHessian=[3];
    assert.equal(sharedAxisBasisLinearization(structuredClone(rows),fixed),token);
    for(const change of [()=>rows[0].jacobian[0]=.6,()=>rows[0].dofs[0]=2,()=>rows[0].kind='length',()=>fixed[0]=1,
        ()=>rows.push({...rows[0]}),()=>rows[0].jacobian[0]=0,()=>rows[0].jacobian[0]=-0]) {
        change();const next=sharedAxisBasisLinearization(rows,fixed);assert.notEqual(next,token);token=next;
    }
});

import {repeatedSharedAxisDual} from '../src/physics/kirchhoffSharedAxisLinear.js';
test('lazy cycle checks match formatted signatures including rounding, negative zero and repeated sets',()=>{
    const map=new Map(),reference=new Set();
    for(let i=0;i<400;i++) {
        const key=String(i%3),dual=new Float64Array([i%5,(i%7)*1e-14,i%2?0:-0,1+(i%4)*1e-11]);
        const signature=key+'/'+Array.from(dual,v=>v.toPrecision(9)).join(',');
        assert.equal(repeatedSharedAxisDual(map,key,dual),reference.has(signature));reference.add(signature);
        dual.fill(99); // Saved detection must own its values.
    }
});

import {BufferGeometry,Float32BufferAttribute} from 'three';
import {createKirchhoffWallWitnessGeometryWorkspace,evaluateKirchhoffWallWitnessGeometry} from '../src/physics/kirchhoffWallWitnessGeometry.js';
test('retained triangle reuse is exact across regions, direct vertex edits and invalid input recovery',()=>{
    const geometry=new BufferGeometry().setAttribute('position',new Float32BufferAttribute([0,0,0,10,0,0,0,10,0],3));
    const fast=createKirchhoffWallWitnessGeometryWorkspace(),ref=createKirchhoffWallWitnessGeometryWorkspace();
    const check=point=>{
        const a=evaluateKirchhoffWallWitnessGeometry({geometry,faceIndex:0,point,reuseTriangle:true},fast);
        const b=evaluateKirchhoffWallWitnessGeometry({geometry,faceIndex:0,point},ref);
        for(const key of ['closestPoint','barycentric','direction','triangleVertices','triangleKey','faceIndex','feature','distance','normalDefined','featureMask'])assert.deepEqual(a[key],b[key],key);
    };
    try {
        for(let i=0;i<500;i++)check([15*Math.sin(i*.7),15*Math.cos(i*.31),i%7===0?0:Math.sin(i)]);
        for(const z of [.1,-2,0]){geometry.attributes.position.setZ(0,z);check([2,3,1]);}
        geometry.attributes.position.setX(1,NaN);
        assert.throws(()=>check([2,2,1]),/finite/);geometry.attributes.position.setX(1,10);check([2,2,1]);
        geometry.attributes.position.setX(1,0);
        assert.throws(()=>check([2,2,1]),/Degenerate/);geometry.attributes.position.setX(1,10);check([2,2,1]);
    }finally{geometry.dispose();}
});

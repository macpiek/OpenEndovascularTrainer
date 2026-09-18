import test from 'node:test';
import assert from 'node:assert/strict';
import {BufferGeometry,Float64BufferAttribute,InterleavedBuffer,InterleavedBufferAttribute} from 'three';
import {createKirchhoffWallWitnessGeometryWorkspace as workspace,evaluateKirchhoffWallWitnessGeometry as evaluate} from '../src/physics/kirchhoffWallWitnessGeometry.js';
import {createSharedAxisVesselWitness} from '../src/physics/kirchhoffSharedAxisVesselWitnesses.js';
import {createSharedAxisNative,captureSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {iterateSharedAxisTimeStep} from '../src/physics/kirchhoffSharedAxisTimeStep.js';
const geometry=vertices=>new BufferGeometry().setAttribute('position',new Float64BufferAttribute(vertices,3));
const values=g=>Object.fromEntries(['closestPoint','barycentric','direction','triangleVertices','triangleKey','faceIndex','feature','distance','normalDefined','featureMask'].map(k=>[k,typeof g[k]==='object'?Array.from(g[k]):g[k]]));
function compare(g,point,faceIndex=0,out=workspace()) {
    const query={geometry:g,faceIndex,point,reuseTriangle:true};
    const reference=values(evaluate(query));
    assert.deepEqual(values(evaluate({...query,reuseTriangleKernel:true},out)),reference);
    return reference;
}

test('cached finite triangle matches every output in all regions, including coplanar and boundary points',()=>{
    const g=geometry([0,0,0,1,0,0,0,1,0]),out=workspace(),features=new Set();
    for(const x of [-2,-1e-16,0,.2,.5,1,1+1e-15,2])for(const y of [-2,0,.3,.5,1,2])for(const z of [0,1e-20,-.3,2])
        features.add(compare(g,[x,y,z],0,out).feature);
    assert.deepEqual([...features].sort(),['edge','face','vertex']);
    for(const reuseTriangleKernel of [false,true,false,true]) {
        const actual=evaluate({geometry:g,faceIndex:0,point:[.2,.3,0],reuseTriangle:true,reuseTriangleKernel},out);
        assert.equal(actual.distance,0);assert.equal(actual.normalDefined,false);
        assert.ok(actual.direction.every(Number.isNaN));
    }
});

test('cached arithmetic is exact for randomized rotated triangles at several scales',()=>{
    let seed=923671;const random=()=>((seed=(Math.imul(seed,1664525)+1013904223)>>>0)/2**32-.5);
    const out=workspace();
    for(const scale of [1e-7,1,1e5])for(let i=0;i<80;i++) {
        const vertices=Array.from({length:9},()=>scale*random()),g=geometry(vertices);
        for(let j=0;j<24;j++)compare(g,Array.from({length:3},()=>2*scale*random()),0,out);
    }
    // A nonzero cross product can coexist with a zero barycentric denominator.
    compare(geometry([0,0,0,1,0,0,1,1e-12,0]),[.5,0,0]);
});

test('shared constants invalidate on raw vertex/index writes, replacements, signed zero and geometry changes',()=>{
    const g=geometry([0,0,0,1,0,0,0,1,0,0,0,2]).setIndex([0,1,2]);
    const a=workspace(),b=workspace(),p=[.4,.2,1];
    compare(g,p,0,a);compare(g,[2,2,1],0,b);const saved=values(b);
    g.attributes.position.array[2]=.15;compare(g,p,0,a);assert.deepEqual(values(b),saved);
    g.index.array[2]=3;compare(g,p,0,b);
    g.setIndex([2,1,0]);compare(g,p,0,a);
    g.setAttribute('position',new Float64BufferAttribute([0,-0,0,1,0,0,0,1,0],3));
    compare(g,p,0,a);g.attributes.position.array[1]=0;compare(g,p,0,a);
    compare(geometry([2,0,0,3,0,0,2,1,0]),p,0,a);
    g.attributes.position.array[0]=NaN;
    assert.throws(()=>evaluate({geometry:g,faceIndex:0,point:p,reuseTriangleKernel:true},a),/finite/);
    g.attributes.position.array.set([0,0,0,1,0,0,2,0,0]);
    assert.throws(()=>evaluate({geometry:g,faceIndex:0,point:p,reuseTriangleKernel:true},a),/Degenerate/);
});

test('indexed and interleaved attributes keep the same index and BVH validation',()=>{
    const g=new BufferGeometry().setAttribute('position',new InterleavedBufferAttribute(
        new InterleavedBuffer(new Float32Array([99,0,0,0,99,1,0,0,99,0,1,0]),4),3,1));
    compare(g,[.2,.3,1]);g.setIndex([2,0,1]);compare(g,[2,2,1]);
    for(const patch of [{faceIndex:-1},{point:[NaN,0,0]},{faceIndex:.5}])
        assert.throws(()=>evaluate({geometry:g,faceIndex:0,point:[0,0,1],reuseTriangleKernel:true,...patch}));
    g.boundsTree={geometry:g,indirect:true};assert.throws(()=>evaluate({geometry:g,faceIndex:0,point:[0,0,1],reuseTriangleKernel:true}),/direct/);
    g.boundsTree={geometry:{},indirect:false};assert.throws(()=>evaluate({geometry:g,faceIndex:0,point:[0,0,1],reuseTriangleKernel:true}),/direct/);
});

test('bounded cache eviction preserves results when a face is revisited',()=>{
    const vertices=[];for(let i=0;i<2100;i++)vertices.push(i,0,0,i+1,0,0,i,1,0);
    const g=geometry(vertices),out=workspace();
    for(let i=0;i<2100;i++)compare(g,[i+.2,.3,1],i,out);
    compare(g,[.2,.3,1],0,out);
});

test('finite-feature gap, Jacobian and Hessian stay exact with and without caller-owned contact buffers',()=>{
    const field={fallbackGeometry:geometry([0,0,0,1,0,0,0,1,0])};
    for(const reuseBuffers of [false,true]) {
        const witness=createSharedAxisVesselWitness(field,{kind:'wall',edge:0,id:'finite',witness:{face:0,t:.3}},{reuseBuffers});
        for(const point of [[.2,.3,1],[2,2,.5],[-2,-1,.5]])for(const needHessian of [false,true]) {
            const input={a:point,b:point,radius:.1,needHessian,reuseGeometry:true};
            const reference=witness.evaluate({...input,state:{}});
            const storage={contact:{jacobian:new Array(6)}};
            const optimized=witness.evaluate({...input,state:{reuseTriangleKernel:true},contactStorage:storage});
            assert.deepEqual(optimized,reference);
            assert.equal(optimized,storage.contact);
        }
        assert.throws(()=>witness.evaluate({a:[0,0,0],b:[0,0,0],radius:.1,state:{reuseTriangleKernel:true}}),/surface/);
    }
});

test('cancellation restores the previous triangle execution mode and accepted physical state',()=>{
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:20}]});
    s.reuseTriangleKernel=false;
    const before=captureSharedAxisNative(s),iterator=iterateSharedAxisTimeStep(s,1/60,{reuseTriangleKernel:true});
    assert.equal(iterator.next().done,false);
    assert.equal(s.reuseTriangleKernel,true);
    iterator.return();
    assert.equal(s.reuseTriangleKernel,false);
    assert.deepEqual(captureSharedAxisNative(s),before);
});

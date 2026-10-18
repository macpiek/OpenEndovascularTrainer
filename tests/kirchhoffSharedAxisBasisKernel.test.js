import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisBasisKernel} from '../src/physics/kirchhoffSharedAxisBasisKernel.js';
import {prepareSharedAxisActiveBasis} from '../src/physics/kirchhoffSharedAxisActiveBasis.js';

test('empty basis does not allocate or require a positive physical dimension',()=>{
    assert.deepEqual(prepareSharedAxisActiveBasis({rows:[],fixed:new Uint8Array(),activeSet:new Uint8Array(),
        dual:new Float64Array(),wasmBasis:true,lazyBasisCoefficients:true}),{converged:true,pivots:0});
});

test('basis memory covers row-heavy layouts and remains fixed while stamp counters wrap',()=>{
    const kernel=createSharedAxisBasisKernel(1,5000),rows=[0,1,4999].map(index=>({
        v:kernel.vector(index),support:kernel.indices(index),supportCount:1,pivot:0,eliminations:[]}));
    const [q,target,last]=rows;q.v[0]=1;q.support[0]=0;last.v[0]=3;last.support[0]=0;
    kernel.store(0,q,1);kernel.store(4999,last,1);kernel.pivotAt[0]=0;
    const buffer=target.v.buffer;
    for(const stamp of [0xfffffffe,1,2,1,3]) {
        target.v[0]=2;target.support[0]=0;target.supportCount=1;
        kernel.eliminate(target,1,stamp);
        assert.equal(target.v[0],0);assert.deepEqual(target.eliminations,[0,2]);
        assert.equal(target.supportCount,1);assert.equal(target.support[0],0);
        assert.equal(target.v.buffer,buffer);assert.equal(last.v[0],3);
    }
});

test('oversized basis workspace falls back before allocation and invalid capacities are rejected',()=>{
    assert.equal(createSharedAxisBasisKernel(4096,4096),null);
    for(const args of [[0,1],[1,0],[-1,4],[1.5,2],[Infinity,1]])
        assert.throws(()=>createSharedAxisBasisKernel(...args),/capacities/);
});

test('updating packed row supports and pivots does not reuse a stale revision',()=>{
    const kernel=createSharedAxisBasisKernel(4,4),q={v:kernel.vector(0),support:kernel.indices(0),supportCount:1,pivot:0};
    const target={v:kernel.vector(1),support:kernel.indices(1),supportCount:1,eliminations:[]};
    for(let revision=1;revision<=4;revision++) {
        const pivot=revision-1;q.v.fill(0);q.v[pivot]=1;q.support[0]=pivot;q.pivot=pivot;
        kernel.store(0,q,revision);kernel.pivotAt.fill(-1);kernel.pivotAt[pivot]=0;
        target.v.fill(-0);target.v[pivot]=revision;target.support[0]=pivot;target.supportCount=1;
        kernel.eliminate(target,1,revision);
        assert.deepEqual(target.eliminations,[0,revision]);assert.equal(target.v[pivot],0);
        for(let i=0;i<4;i++)if(i!==pivot)assert.ok(Object.is(target.v[i],-0));
    }
});

import test from 'node:test';
import assert from 'node:assert/strict';
import {solveCompositeRankRevealing as solve} from '../src/physics/compositeRankRevealingSolve.js';
const multiply=(A,x)=>Array.from({length:x.length},(_,i)=>x.reduce((s,v,j)=>s+A[i*x.length+j]*v,0));
const close=(a,b,t=1e-11)=>assert.ok(Math.abs(a-b)<=t*Math.max(1,Math.abs(a),Math.abs(b)),`${a} != ${b}`);
const checkOriginal=(A,b,r)=>multiply(A,r.solution).forEach((v,i)=>close(v-b[i],r.residual[i]));

test('complete row and column pivoting solves a full-rank nonsymmetric system without changing inputs',()=>{
    const A=[0,2,1,3,0,4,1,-1,0],x=[2,-3,4],b=multiply(A,x),saved=A.slice(),r=solve({matrix:A,rhs:b,size:3});
    assert.equal(r.rank,3);assert.equal(r.compatible,true);r.solution.forEach((v,i)=>close(v,x[i]));
    assert.deepEqual(A,saved);checkOriginal(A,b,r);assert.equal(r.certified,false);assert.equal(r.nonlinearStepAccepted,false);
});

test('compatible nonsymmetric rank-deficient system returns a basic solution and reports its nullity',()=>{
    const A=[1,2,3,2,4,6,0,1,1],b=multiply(A,[2,3,-1]),r=solve({matrix:A,rhs:b,size:3});
    assert.equal(r.rank,2);assert.equal(r.nullity,1);assert.equal(r.compatible,true);
    assert.equal(r.solutionKind,'complete-pivot-basic');assert.equal(r.solution[r.columnPermutation[2]],0);
    checkOriginal(A,b,r);multiply(A,r.solution).forEach((v,i)=>close(v,b[i]));
    // The selected solution may differ from the supplied one along a null
    // vector containing arbitrary coordinates, not necessarily force DOFs.
    assert.notDeepEqual(Array.from(r.solution),[2,3,-1]);
});

test('incompatible singular system keeps its original residual visible and rejects compatibility',()=>{
    const A=[1,2,3,2,4,6,0,1,1],b=[5,11,2],r=solve({matrix:A,rhs:b,size:3});
    assert.equal(r.rank,2);assert.equal(r.compatible,false);assert.ok(r.maximumResidual>.1);checkOriginal(A,b,r);
    assert.equal(r.certified,false);
});

test('equilibration resolves strongly different row and column units',()=>{
    const base=[3,1,-1,2,4,1,1,-2,5],rs=[1e-12,1e8,1e-4],cs=[1e6,1e-9,1e3],
        A=base.map((v,k)=>v*rs[Math.floor(k/3)]*cs[k%3]),x=[2/cs[0],-3/cs[1],4/cs[2]],b=multiply(A,x);
    for(const scales of [{},{rowScales:rs.map(v=>1/v),columnScales:cs.map(v=>1/v)}]) {
        const r=solve({matrix:A,rhs:b,size:3,...scales});assert.equal(r.rank,3);assert.equal(r.compatible,true);
        r.solution.forEach((v,i)=>close(v*cs[i],x[i]*cs[i]));checkOriginal(A,b,r);
    }
});

test('rank cutoff is explicit in the chosen scale and truncation never hides incompatibility',()=>{
    const options={matrix:[1,0,0,1e-14],rhs:[1,1e-14],size:2,rowScales:[1,1],columnScales:[1,1],rankTolerance:1e-12};
    const truncated=solve(options);assert.equal(truncated.rank,1);assert.equal(truncated.compatible,false);
    const full=solve({...options,rankTolerance:1e-16});assert.equal(full.rank,2);assert.equal(full.compatible,true);
});

test('zero systems and invalid inputs are explicit',()=>{
    const zero=solve({matrix:[0,0,0,0],rhs:[0,0],size:2});assert.equal(zero.rank,0);assert.equal(zero.compatible,true);
    assert.equal(solve({matrix:[0,0,0,0],rhs:[0,1],size:2}).compatible,false);
    const valid={matrix:[1,0,0,1],rhs:[1,2],size:2};
    for(const extra of [{size:3},{maxSize:1},{rankTolerance:0},{rankTolerance:NaN},{backwardTolerance:-1},{maxRefinements:9},
        {matrix:[1,0,NaN,1]},{rhs:[Infinity,0]},{rowScales:[1,0]},{columnScales:[1,-1]}])assert.throws(()=>solve({...valid,...extra}),RangeError);
});

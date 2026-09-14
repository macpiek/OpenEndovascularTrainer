import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisLinear,solveSharedAxisLinear,iterateSharedAxisLinear} from '../src/physics/kirchhoffSharedAxisLinear.js';

const close=(a,b,tol=1e-8)=>assert.ok(Math.abs(a-b)<tol*Math.max(1,Math.abs(a),Math.abs(b)),`${a} vs ${b}`);
function independent(count=32) {
    const layout={dofCount:count,band:1},rows=Array.from({length:count},(_,i)=>({
        id:`wall/${i}`,kind:'wall',dofs:[i],jacobian:[1],gap:-1-i/count,multiplier:0}));
    return {layout,rows,chain:{layout,hessian:new Float64Array(count).fill(1)},gradient:new Float64Array(count),fixed:new Uint8Array(count)};
}
function solve(f,options={}) {
    return solveSharedAxisLinear(createSharedAxisLinear(f.layout,f.rows,{lazy:true}),f.chain,{...f,...options});
}
function seeded(seed) {
    const rng=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/2**32;};
    const n=5,layout={dofCount:n,band:n},dofs=Array.from({length:n},(_,i)=>i),target=dofs.map(()=>2*rng()-1);
    const rows=Array.from({length:16},(_,id)=>{
        const jacobian=dofs.map(()=>2*rng()-1);
        return {id,kind:'wall',dofs,jacobian,multiplier:0,gap:-jacobian.reduce((a,v,i)=>a+v*target[i],0)+rng()*.2};
    });
    const hessian=new Float64Array(n*n);for(let i=0;i<n;i++)hessian[i*n]=1;
    return {layout,rows,chain:{layout,hessian},gradient:Float64Array.from(dofs,()=>rng()*2-1),fixed:new Uint8Array(n)};
}
function certify(f,result) {
    assert.equal(result.converged,true,JSON.stringify(result));
    const balance=Float64Array.from(f.gradient,(v,i)=>v+result.increment[i]);
    f.rows.forEach((r,i)=>{
        const lambda=r.multiplier+result.multiplierIncrement[i],gap=r.gap+r.dofs.reduce((s,j,k)=>s+r.jacobian[k]*result.increment[j],0);
        assert.ok(lambda>=-1e-8&&gap>=-1e-8);close(lambda*gap,0);
        r.dofs.forEach((j,k)=>{balance[j]-=lambda*r.jacobian[k];});
    });
    balance.forEach(v=>close(v,0));
}

test('batch activation reduces independent wall LU solves with identical unique primal solution',()=>{
    const f=independent(),snapshot=structuredClone(f),reference=solve(f,{batchActivation:false});
    for(const size of [8,16]) {
        const result=solve(f,{batchActivationSize:size});certify(f,result);
        assert.equal(result.batchFallback,false);assert.equal(result.factorizations,1+Math.ceil(f.rows.length/size));
        assert.equal(reference.factorizations,f.rows.length+1);
        result.increment.forEach((v,i)=>close(v,reference.increment[i]));
        result.multiplierIncrement.forEach((v,i)=>close(v,reference.multiplierIncrement[i]));
    }
    assert.deepEqual(f,snapshot,'Batching must not modify physical inputs');
});

test('seeded feasible QPs agree with single activation and satisfy global stationarity and complementarity',()=>{
    for(let seed=1;seed<=100;seed++) {
        const f=seeded(seed),snapshot=structuredClone(f),reference=solve(f,{batchActivation:false}),batch=solve(f);
        certify(f,reference);certify(f,batch);
        batch.increment.forEach((v,i)=>close(v,reference.increment[i]));
        assert.deepEqual(f,snapshot);
    }
});

test('failed batched factorization retries original rows and counts both searches without stale result views',()=>{
    const f=independent(12),snapshot=structuredClone(f),w=createSharedAxisLinear(f.layout,f.rows);
    const actualSolve=w.lu.solve.bind(w.lu);let calls=0;
    w.lu.solve=(...args)=>{calls++;return calls===2?false:actualSolve(...args);};
    const r=solveSharedAxisLinear(w,f.chain,{...f,compactWorkingSet:false}),reference=solve(f,{batchActivation:false});
    certify(f,r);assert.equal(r.batchFallback,true);assert.equal(r.batchFailure,'band-lu-rejected');
    assert.equal(r.batchFactorizations,2);assert.equal(r.referenceFactorizations,reference.factorizations-1);
    assert.equal(r.workingSetReuses,1,'Reference fallback reuses the already solved initial working set');
    assert.equal(r.factorizations,r.batchFactorizations+r.referenceFactorizations);
    assert.equal(r.activeSetAttempts,r.batchAttempts+r.referenceAttempts);
    assert.equal(calls,r.factorizations);
    r.increment.forEach((v,i)=>close(v,reference.increment[i]));
    r.multiplierIncrement.forEach((v,i)=>close(v,reference.multiplierIncrement[i]));
    assert.deepEqual(f,snapshot);
});

test('infeasible active basis remains explicitly rejected after reference fallback',()=>{
    const layout={dofCount:1,band:1},rows=[
        {kind:'wall',dofs:[0],jacobian:[1],gap:-1,multiplier:1},
        {kind:'wall',dofs:[0],jacobian:[-1],gap:0,multiplier:1}
    ];
    const f={layout,rows,chain:{layout,hessian:new Float64Array([1])},gradient:new Float64Array(1),fixed:new Uint8Array(1)};
    const r=solve(f);assert.equal(r.converged,false);assert.equal(r.batchFallback,true);
    assert.equal(r.failure,'incompatible-active-constraints');assert.equal(r.factorizations,0);assert.equal(r.activeSetAttempts,2);
});

test('cooperative linear solve yields between attempts, excludes pauses from CPU time, and matches synchronous output',async()=>{
    const f=independent(20),snapshot=structuredClone(f),reference=solve(f),w=createSharedAxisLinear(f.layout,f.rows,{lazy:true});
    const iterator=iterateSharedAxisLinear(w,f.chain,f),started=performance.now();let pauses=0,yields=0,result;
    while(true) {
        const next=iterator.next();if(next.done){result=next.value;break;}
        assert.equal(next.value.kind,'linear-active-set');assert.deepEqual(f,snapshot);yields++;
        const begin=performance.now();await new Promise(resolve=>setTimeout(resolve,5));pauses+=performance.now()-begin;
    }
    const elapsed=performance.now()-started;
    assert.equal(yields,result.activeSetAttempts);assert.ok(yields>1);
    assert.ok(elapsed-result.cpuMs>=pauses-.5);certify(f,result);
    result.increment.forEach((v,i)=>close(v,reference.increment[i]));
    result.multiplierIncrement.forEach((v,i)=>close(v,reference.multiplierIncrement[i]));
});

test('interrupting a private linear iterator leaves physical inputs available for a fresh reference solve',()=>{
    const f=independent(),snapshot=structuredClone(f),w=createSharedAxisLinear(f.layout,f.rows,{lazy:true});
    const iterator=iterateSharedAxisLinear(w,f.chain,f);iterator.next();iterator.next();iterator.return();
    assert.deepEqual(f,snapshot);certify(f,solveSharedAxisLinear(w,f.chain,{...f,batchActivation:false}));
});

test('cancellation in the unscaled residual is certified accurately without loosening the threshold',()=>{
    const layout={dofCount:3,band:3},tangent=new Float64Array(15);
    // Exact solution is (1,1,1). Ordinary accumulation of row 0 starts at
    // -1, loses it at 1e16, then incorrectly returns a residual of +1.
    tangent[2]=1e16;tangent[3]=-1e16;tangent[4]=1;tangent[7]=1;tangent[12]=1;
    const chain={layout,tangent},gradient=new Float64Array([-1,-1,-1]),fixed=new Uint8Array(3);
    const r=solveSharedAxisLinear(createSharedAxisLinear(layout,[],{lazy:true}),chain,{rows:[],gradient,fixed,tolerance:1e-10});
    assert.equal(r.converged,true);assert.equal(r.factorizations,1);assert.equal(r.residual,0);
    assert.deepEqual(Array.from(r.increment),[1,1,1]);
});


test('bounded trial direction stops early without mutating state and unrestricted search remains available',()=>{
    const f=independent(32),snapshot=structuredClone(f);
    const bounded=solve(f,{maxActiveSetAttempts:2});
    assert.equal(bounded.converged,false);assert.equal(bounded.failure,'active-set-limit');
    assert.equal(bounded.activeSetAttempts,4);assert.equal(bounded.factorizations,3);assert.equal(bounded.workingSetReuses,1);
    assert.deepEqual(f,snapshot);
    certify(f,solve(f));
    for(const maxActiveSetAttempts of [0,-1,.5,NaN])assert.throws(()=>solve(f,{maxActiveSetAttempts}),RangeError);
});

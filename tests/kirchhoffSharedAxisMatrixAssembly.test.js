import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisLinear,solveSharedAxisLinear,iterateSharedAxisLinear} from '../src/physics/kirchhoffSharedAxisLinear.js';

const physical=r=>({increment:r.increment.slice(),reaction:r.multiplierIncrement.slice(),converged:r.converged,failure:r.failure,
    residual:r.residual,factorizations:r.factorizations,attempts:r.activeSetAttempts,reuses:r.workingSetReuses});
function fixture() {
    const n=9,layout={dofCount:n,band:n},hessian=new Float64Array(n*n);
    for(let i=0;i<n;i++)hessian[i*n]=10+i;
    const rows=Array.from({length:n},(_,i)=>({id:i,kind:i<2?'length':'wall',dofs:[i],jacobian:[1],gap:-.1,multiplier:.03,
        geometricHessian:new Float64Array([.01])}));
    return {layout,chain:{layout,hessian},rows,gradient:new Float64Array(n),fixed:new Uint8Array(n)};
}
test('prepared matrices match every full KKT assembly, including changing masks, supports and nonsymmetric force columns',()=>{
    const f=fixture(),fast=createSharedAxisLinear(f.layout,f.rows),ref=createSharedAxisLinear(f.layout,f.rows);
    for(let step=0;step<40;step++) {
        for(let i=0;i<f.rows.length;i++) {
            const r=f.rows[i];r.gap=i<2||step%3===0?-.1:.2;r.multiplier=(i+step)%2?.01:0;
            r.jacobian[0]=1+step/30;r.extraForceDofs=[(i+step)%9];r.extraForceJacobian=[.002*step];
            r.geometricHessian[0]=.013*(step+1);r.dofs[0]=(i+step)%9;
        }
        f.fixed.fill(0);if(step%5===0){f.fixed[f.rows[0].dofs[0]]=1;f.rows[0].gap=0;}
        f.chain.hessian.fill(0);for(let i=0;i<9;i++)f.chain.hessian[i*9]=15+i+step;
        f.chain.tangent=step%2?new Float64Array(9*17):null;
        if(f.chain.tangent)for(let i=0;i<9;i++)for(let j=0;j<9;j++)f.chain.tangent[i*17+j-i+8]=i===j?30+i:(i-j)*.001;
        f.gradient.fill(step*.001);
        const snapshots=[],out=[];
        for(const [reuseMatrixAssembly,w] of [[false,ref],[true,fast]]) {
            const assemblies=[],original=w.lu.solve;
            w.lu.solve=function(...args){assemblies.push({matrix:args[0].slice(),rhs:args[1].slice(),scales:args[2].slice()});return original.apply(this,args);};
            const trace=[];
            try{out.push({...physical(solveSharedAxisLinear(w,f.chain,{...f,compactWorkingSet:false,reuseMatrixAssembly,trace})),trace});}
            finally{w.lu.solve=original;}
            snapshots.push(assemblies);
        }
        assert.deepEqual(snapshots[1],snapshots[0],`every matrix/RHS/scaling in step ${step}`);
        assert.deepEqual(out[1],out[0]);
    }
});
test('compact active layouts retain exact directions with overlapping geometric Hessians and changing force supports',()=>{
    const f=fixture(),w=createSharedAxisLinear(f.layout,f.rows,{lazy:true});
    for(let trial=0;trial<24;trial++) {
        for(let i=0;i<f.rows.length;i++) {
            const r=f.rows[i];r.dofs=[i,(i+1)%9].sort((a,b)=>a-b);r.jacobian=[.6,.4];
            r.geometricHessian=new Float64Array([.02,.004,.004,.01]);
            r.gap=i<2?-.01:(i+trial)%3===0?.03:-.05;r.multiplier=(i+trial)%2?.01:0;
            r.extraForceDofs=[(i+trial)%9];r.extraForceJacobian=[.001];
        }
        const before=structuredClone(f),outputs=[];
        for(const reuseMatrixAssembly of [false,true]) {
            const trace=[],r=solveSharedAxisLinear(reuseMatrixAssembly?w:createSharedAxisLinear(f.layout,f.rows,{lazy:true}),f.chain,{...f,reuseMatrixAssembly,trace});
            outputs.push({...physical(r),trace});
        }
        assert.deepEqual(outputs[1],outputs[0]);assert.deepEqual(f,before);
    }
});
test('cancellation releases common numeric assembly; a new call rereads in-place Hessians and reactions',()=>{
    const f=fixture(),w=createSharedAxisLinear(f.layout,f.rows);
    f.rows.forEach(r=>{r.kind='wall';r.multiplier=0;});
    const it=iterateSharedAxisLinear(w,f.chain,{...f,compactWorkingSet:false,reuseMatrixAssembly:true,batchActivation:false});
    assert.equal(it.next().done,false);assert.equal(it.next().done,false);it.return();
    f.chain.hessian[0]=45;f.rows[0].geometricHessian[0]=.3;f.rows[0].multiplier=.02;
    const outputs=[false,true].map(reuseMatrixAssembly=>physical(solveSharedAxisLinear(reuseMatrixAssembly?w:createSharedAxisLinear(f.layout,f.rows),f.chain,{...f,compactWorkingSet:false,reuseMatrixAssembly})));
    assert.deepEqual(outputs[1],outputs[0]);
});

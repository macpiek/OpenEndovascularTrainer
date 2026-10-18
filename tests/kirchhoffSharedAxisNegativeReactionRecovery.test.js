import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisLinear,solveSharedAxisLinear,iterateSharedAxisLinear} from '../src/physics/kirchhoffSharedAxisLinear.js';

function fixture(extraWall=false) {
    const epsilon=1e-7,layout={dofCount:2,band:2};
    const rows=[{kind:'wall',id:'loose',dofs:[0,1],jacobian:[.64,.768],gap:-1,multiplier:1},
        {kind:'wall',id:'tight',dofs:[0,1],jacobian:[.64+epsilon,.768-epsilon],gap:-1.0001,multiplier:1}];
    if(extraWall)rows.push({kind:'wall',id:'later-contact',dofs:[0,1],jacobian:[0,1],gap:-1.1,multiplier:0});
    return {layout,rows,chain:{layout,hessian:Float64Array.from([1,0,1,0])},
        gradient:Float64Array.from([-(1.28+epsilon),-(1.536-epsilon)]),fixed:new Uint8Array(2)};
}
const workspace=f=>createSharedAxisLinear(f.layout,f.rows,{lazy:true});
const run=(f,o={},w=workspace(f))=>solveSharedAxisLinear(w,f.chain,{...f,tolerance:1e-8,recoverNegativeContact:true,...o});
function certificate(f,r) {
    assert.ok(r.converged,JSON.stringify(r));assert.ok(r.residual<=1e-8);
    const force=Float64Array.from(r.increment,(x,i)=>x+f.gradient[i]);
    f.rows.forEach((row,i)=>{
        const reaction=row.multiplier+r.multiplierIncrement[i];
        const gap=row.gap+row.jacobian.reduce((sum,j,k)=>sum+j*r.increment[row.dofs[k]],0);
        assert.ok(reaction>=-1e-8,`Negative wall reaction ${reaction}`);
        assert.ok(gap>=-1e-8,`Released wall crossed ${gap}`);
        assert.ok(Math.abs(reaction*gap)<1e-8,'Complementarity must hold on every original row');
        row.dofs.forEach((p,k)=>{force[p]-=row.jacobian[k]*r.multiplierIncrement[i];});
    });
    assert.ok(force.every(v=>Math.abs(v)<1e-8),'Original force balance');
}

for(const wasmLinearAssembly of [false,true])test(`an uncertified negative reaction proposes only a pivot; the final direction retains the original certificate (WASM=${wasmLinearAssembly})`,()=>{
    const f=fixture(),before=structuredClone(f),options={wasmLinearAssembly,reuseConstraintWork:true,reuseMatrixAssembly:true,deferActiveBasis:true};
    const rejected=run(f,{...options,recoverNegativeContact:false});
    assert.equal(rejected.converged,false);assert.ok(rejected.residual>1e-8);
    assert.ok(Math.min(...rejected.multiplierIncrement)<-1e9,'Ill-conditioned opposing contact reactions');
    const r=run(f,options);certificate(f,r);assert.ok(r.negativeReactionRecoveries>0);
    assert.deepEqual(f,before,'Recovery only changes the private working set');
    const n=f.rows[1].jacobian,scale=-f.rows[1].gap/n.reduce((s,x)=>s+x*x,0);
    for(let i=0;i<2;i++)assert.ok(Math.abs(r.increment[i]-scale*n[i])<1e-9,'Closest feasible point, not the failed enormous displacement');
});

test('a different inactive wall still enters after a failed contact is released',()=>{
    const f=fixture(true),trace=[],r=run(f,{trace});certificate(f,r);
    assert.ok(trace.some(e=>e.kind==='negative-reaction-recovery'));
    assert.ok(trace.some(e=>e.change==='later-contact'||e.activate?.includes('later-contact')));
    assert.ok(Math.abs(r.increment[1]-1.1)<1e-8);
});

test('cancelling after a recovery pivot does not retain an uncertified direction or mutate reactions',()=>{
    const f=fixture(),before=structuredClone(f),trace=[],w=workspace(f);
    const iterator=iterateSharedAxisLinear(w,f.chain,{...f,tolerance:1e-8,recoverNegativeContact:true,trace});
    let yielded=false;
    for(let i=0;i<100;i++) {
        const next=iterator.next();assert.equal(next.done,false);
        if(trace.some(e=>e.kind==='negative-reaction-recovery')){yielded=true;break;}
    }
    assert.ok(yielded);iterator.return();assert.deepEqual(f,before);
    const other=fixture(true);certificate(other,run(other));
    certificate(f,run(f,{},w));assert.deepEqual(f,before);
});

test('incompatible walls and rejected LU factors never become successful through recovery',()=>{
    const f=fixture();f.rows=[{kind:'wall',dofs:[0],jacobian:[1],gap:-1,multiplier:1},
        {kind:'wall',dofs:[0],jacobian:[-1],gap:-1,multiplier:1}];
    const impossible=run(f);assert.equal(impossible.converged,false);assert.equal(impossible.negativeReactionRecoveries,0);
    const singular=fixture();singular.chain.hessian.fill(0);singular.rows=[];
    const rejected=run(singular);assert.equal(rejected.converged,false);assert.equal(rejected.failure,'band-lu-rejected');
    assert.equal(rejected.negativeReactionRecoveries,0);
});

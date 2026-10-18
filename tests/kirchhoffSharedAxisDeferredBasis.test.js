import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisLayout,createSharedAxisLinear,solveSharedAxisLinear,iterateSharedAxisLinear} from '../src/physics/kirchhoffSharedAxisLinear.js';

function fixture(gaps,jacobians) {
    const layout=createSharedAxisLayout([['wire']]),p=layout.positions[1];
    const rows=gaps.map((gap,i)=>({id:`wall${i}`,kind:'wall',dofs:[p],jacobian:[jacobians[i]],gap,multiplier:1}));
    const chain={layout,hessian:new Float64Array(layout.dofCount*layout.band)};
    for(let i=0;i<layout.dofCount;i++)chain.hessian[i*layout.band]=1;
    const fixed=new Uint8Array(layout.dofCount).fill(1);fixed[p]=0;
    const gradient=new Float64Array(layout.dofCount);gradient[p]=-jacobians.reduce((a,b)=>a+b,0);
    return {layout,rows,chain,fixed,gradient,p};
}
for(const batchActivation of [true,false])test(`deferred rank repair retains tighter parallel contact and counts rejected LU (batch ${batchActivation})`,()=>{
    const f=fixture([-1,-2],[1,1]),outputs=[];
    for(const deferActiveBasis of [false,true]) {
        const trace=[],r=solveSharedAxisLinear(createSharedAxisLinear(f.layout,f.rows),f.chain,{...f,tolerance:1e-9,deferActiveBasis,batchActivation,lazyBasisCoefficients:true,trace});
        assert.equal(r.converged,true);assert.ok(Math.abs(r.increment[f.p]-2)<1e-9);
        assert.ok(Math.abs(1+r.multiplierIncrement[0])<1e-9);assert.ok(Math.abs(1+r.multiplierIncrement[1]-2)<1e-9);
        assert.ok(trace.some(e=>e.kind==='basis-pivot'));outputs.push(r);
    }
    assert.deepEqual(outputs[1].increment,outputs[0].increment);
    assert.deepEqual(outputs[1].multiplierIncrement,outputs[0].multiplierIncrement);
    assert.ok(outputs[1].factorizations>outputs[0].factorizations,'Singular trial factorization must remain in work accounting');
});

test('deferred basis still rejects incompatible walls, and cancellation cannot change physical rows',()=>{
    const f=fixture([-1,0],[1,-1]),before=structuredClone(f.rows),w=createSharedAxisLinear(f.layout,f.rows);
    const options={...f,tolerance:1e-9,deferActiveBasis:true,lazyBasisCoefficients:true};
    const it=iterateSharedAxisLinear(w,f.chain,options);it.next();it.next();it.return();
    assert.deepEqual(f.rows,before);
    const r=solveSharedAxisLinear(w,f.chain,options);
    assert.equal(r.converged,false);assert.equal(r.failure,'incompatible-active-constraints');assert.deepEqual(f.rows,before);
});

test('deferred rank repair keeps current Jacobians across interleaved solves and in-place geometry updates',()=>{
    const a=fixture([-1,-2],[1,1]),b=fixture([-3,-1],[1,1]);
    // Reusing a fixed-mask identity must not make different Jacobians share
    // numerical rank data, including when a generator resumes after a yield.
    b.fixed=a.fixed;
    const options=f=>({...f,tolerance:1e-9,deferActiveBasis:true,reuseConstraintWork:true,
        reuseMatrixAssembly:true,lazyBasisCoefficients:true,wasmBasis:true});
    const drain=it=>{let next;do{next=it.next();}while(!next.done);return next.value;};
    for(let pass=0;pass<4;pass++) {
        b.rows[1].jacobian[0]=pass%2?2:1;b.rows[1].gap=-4;
        b.gradient[b.p]=-b.rows.reduce((sum,r)=>sum+r.jacobian[0],0);
        const reference=solveSharedAxisLinear(createSharedAxisLinear(b.layout,b.rows),b.chain,
            {...options(b),deferActiveBasis:false,reuseConstraintWork:false});
        const it=iterateSharedAxisLinear(createSharedAxisLinear(b.layout,b.rows),b.chain,options(b));
        assert.equal(it.next().done,false);
        const other=solveSharedAxisLinear(createSharedAxisLinear(a.layout,a.rows),a.chain,options(a));
        assert.equal(other.converged,true);
        const actual=drain(it);
        assert.equal(actual.converged,true);assert.ok(actual.residual<=1e-9);
        assert.deepEqual(actual.increment,reference.increment);
        assert.deepEqual(actual.multiplierIncrement,reference.multiplierIncrement);
    }
});

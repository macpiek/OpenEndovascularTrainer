import test from 'node:test';
import assert from 'node:assert/strict';
import {stepSharedAxis,iterateSharedAxisTimeStep} from '../src/physics/kirchhoffSharedAxisTimeStep.js';
import {createSharedAxisModifiedNewton} from '../src/physics/kirchhoffSharedAxisModifiedNewton.js';
import {createSharedAxisLinear,solveSharedAxisLinear} from '../src/physics/kirchhoffSharedAxisLinear.js';
import {createSharedAxisNative,applySharedAxisNativeIncrement,iterateSharedAxisNative,relaxSharedAxisNative,captureSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';

const close=(a,b,tol=1e-8)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>assert.ok(Math.abs(v-b[i])<=tol,`${i}: ${v} vs ${b[i]}`));};
function linearFixture() {
    const layout={dofCount:3,band:1},chain={layout,hessian:new Float64Array([2,2,2])};
    const rows=[{id:'length',kind:'length',dofs:[0],jacobian:[1],gap:.2,multiplier:.5},
        {id:'loaded',kind:'wall',dofs:[1],jacobian:[1],gap:.1,multiplier:2},
        {id:'open',kind:'wall',dofs:[2],jacobian:[1],gap:100,multiplier:0}];
    const options={rows,gradient:new Float64Array([3,4,1]),fixed:new Uint8Array(3),reuseWorkingSet:false};
    return {layout,chain,rows,options,w:createSharedAxisLinear(layout,rows,{lazy:true})};
}
test('frozen matrix handles a new RHS without refactorization and owns its storage',()=>{
    const f=linearFixture(),cache=createSharedAxisModifiedNewton();
    try {
        assert.ok(solveSharedAxisLinear(f.w,f.chain,{...f.options,modifiedNewtonContext:cache}).converged);
        cache.seal(f.rows,f.options.fixed);cache.arm(true);
        f.rows[0].gap=.1;f.rows[1].gap=.05;f.rows[1].multiplier=3;f.options.gradient.set([1,2,3]);
        // Overwrite the ordinary workspace and its arena between reuse attempts.
        f.chain.hessian.fill(7);solveSharedAxisLinear(f.w,f.chain,f.options);f.chain.hessian.fill(2);
        assert.equal(cache.canReuse(f.rows,f.options.fixed,1e-8),true);
        const actual=cache.solve(f.rows,f.options.gradient,1e-8);
        const reference=solveSharedAxisLinear(createSharedAxisLinear(f.layout,f.rows,{lazy:true}),f.chain,f.options);
        assert.ok(actual.converged);assert.equal(actual.factorizations,0);
        close(actual.increment,reference.increment);close(actual.multiplierIncrement,reference.multiplierIncrement);
        cache.arm(true);f.rows[1].multiplier=0;assert.equal(cache.canReuse(f.rows,f.options.fixed,1e-8),false);
        f.rows[1].multiplier=3;f.rows[1].id='different-face';assert.equal(cache.canReuse(f.rows,f.options.fixed,1e-8),false);
        f.rows[1].id='loaded';f.options.fixed[0]=1;assert.equal(cache.canReuse(f.rows,f.options.fixed,1e-8),false);
        f.options.fixed[0]=0;
        cache.getLU(createSharedAxisLinear(f.layout,f.rows));cache.seal(f.rows,f.options.fixed);cache.arm(true);
        assert.equal(cache.canReuse(f.rows,f.options.fixed,1e-8),false,'a new workspace must record its own dual mapping');
    } finally {cache.dispose();}
});

test('frozen directions that release a loaded contact are rejected for a fresh solve',()=>{
    const f=linearFixture(),cache=createSharedAxisModifiedNewton();
    try {
        solveSharedAxisLinear(f.w,f.chain,{...f.options,modifiedNewtonContext:cache});cache.seal(f.rows,f.options.fixed);cache.arm(true);
        f.options.gradient[1]=-100;
        assert.equal(cache.solve(f.rows,f.options.gradient,1e-8).converged,false);
    } finally {cache.dispose();}
});

function rod() {
    const s=createSharedAxisNative({startCoordinate:-10,tools:[{id:'wire',type:'straight',insertion:40}]});
    const dx=new Float64Array(s.layout.dofCount);
    s.layout.positions.forEach((p,i)=>{if(i>1)dx[p+1]=.02*Math.sin(i);});
    applySharedAxisNativeIncrement(s,dx,new Float64Array(s.multipliers.length));return s;
}
test('modified Newton retains nonlinear acceptance and cancellation releases private progress',()=>{
    const reference=rod(),actual=rod(),options={promoteTrialAssembly:true,reuseConstraintWork:true,reuseMatrixAssembly:true};
    const a=relaxSharedAxisNative(reference,options),b=relaxSharedAxisNative(actual,{...options,modifiedNewton:true});
    assert.ok(a.converged);assert.ok(b.converged,JSON.stringify(b));assert.ok(b.modifiedAttempts>0);
    assert.ok(b.residual.force<=1e-6&&b.residual.torque<=1e-6&&b.residual.length<=1e-5);
    close(actual.positions.flat(),reference.positions.flat(),1e-4);
    const cancelled=rod(),before=captureSharedAxisNative(cancelled);
    const iterator=iterateSharedAxisNative(cancelled,{...options,modifiedNewton:true});iterator.next();iterator.next();iterator.return();
    assert.deepEqual(captureSharedAxisNative(cancelled),before);
    assert.ok(relaxSharedAxisNative(cancelled,{...options,modifiedNewton:true}).converged);
});

test('a rejected experimental timestep retries full Newton and cancellation rolls back material history',()=>{
    const s=rod();let rejected=0;
    const r=stepSharedAxis(s,1/60,{modifiedNewton:true,promoteTrialAssembly:true,
        observeTrial:event=>{if(event.kind==='direction'&&event.method===-1){rejected++;throw new Error('synthetic frozen-direction rejection');}}});
    assert.ok(rejected>0);
    assert.ok(r.converged,JSON.stringify(r));assert.equal(r.modifiedNewtonRecovery?.converged,true);
    const other=rod(),before=captureSharedAxisNative(other),history=other.wallFrictionHistory;
    const iterator=iterateSharedAxisTimeStep(other,1/60,{modifiedNewton:true,promoteTrialAssembly:true});
    let progressed=false;
    for(let i=0;i<500;i++) {
        const next=iterator.next();assert.equal(next.done,false,'cancel before publication');
        if(JSON.stringify(other.positions)!==JSON.stringify(before.positions)){progressed=true;break;}
    }
    assert.ok(progressed);iterator.return();
    assert.deepEqual(captureSharedAxisNative(other),before);assert.equal(other.wallFrictionHistory,history);
    assert.ok(stepSharedAxis(other,1/60,{modifiedNewton:true,promoteTrialAssembly:true}).converged);
});

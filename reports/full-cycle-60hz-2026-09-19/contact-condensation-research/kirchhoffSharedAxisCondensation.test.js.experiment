import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisLinear,solveSharedAxisLinear} from '../src/physics/kirchhoffSharedAxisLinear.js';
import {measureUncondensedSharedAxisResidual,canCondenseSharedAxisContact} from '../src/physics/kirchhoffSharedAxisCondensation.js';

test('parallel compliant contacts condense to one primal equation and preserve load sharing',()=>{
    for(const count of [1,8,64])for(const compliance of [.01,1e-6])
    for(const reuseMatrixAssembly of [false,true])for(const wasmLinearAssembly of [false,true]) {
        const layout={dofCount:1,band:1},chain={layout,hessian:new Float64Array([10])};
        const rows=Array.from({length:count},()=>({kind:'wall',dofs:[0],jacobian:[1],gap:.1,multiplier:0,compliance}));
        const work=createSharedAxisLinear(layout,rows),options={rows,gradient:new Float64Array([20]),fixed:new Uint8Array(1),
            reuseMatrixAssembly,wasmLinearAssembly,batchActivationSize:64,condenseCompliantContacts:true};
        const result=solveSharedAxisLinear(work,chain,options);
        assert.ok(result.converged,JSON.stringify(result));assert.equal(result.condensedContacts,count);
        assert.equal(work.peakActiveEntries,1);
        const x=result.increment[0],expected=(-count*.1-compliance*20)/(count+compliance*10);
        assert.ok(Math.abs(x-expected)<1e-10);
        const target=(20+10*x)/count;
        for(const lambda of result.multiplierIncrement)assert.ok(Math.abs(lambda-target)<1e-8);
        assert.ok(measureUncondensedSharedAxisResidual(chain,options,new Uint8Array(count).fill(1),result.increment,result.multiplierIncrement)<1e-8);
    }
});

function coupledFixture(compliance,fixedEntrance,denseTangent) {
    const layout={dofCount:4,band:3},hessian=Float64Array.from([12,0,0,11,.2,0,10,.2,0,9,.2,0]);
    const chain={layout,hessian},dx=Float64Array.from([fixedEntrance?0:-.1,.02,.02,.03]);
    if(denseTangent) {
        chain.tangent=new Float64Array(20);
        for(let i=0;i<4;i++)for(let j=Math.max(0,i-2);j<=Math.min(3,i+2);j++)
            chain.tangent[i*5+j-i+2]=hessian[Math.max(i,j)*3+Math.abs(i-j)]+(i===0&&j===2?.03:0);
    }
    const rows=[
        {kind:'wall',dofs:[0,1],jacobian:[.6,.4],compliance,multiplier:.7,extraForceDofs:[2],extraForceJacobian:[.15],geometricHessian:Float64Array.from([.1,.02,.02,.1])},
        {kind:'wall',dofs:[2,3],jacobian:[.3,.7],compliance:compliance*2,multiplier:1.2,extraForceDofs:[1],extraForceJacobian:[-.1],geometricHessian:Float64Array.from([.07,0,0,.07])},
        {kind:'length',dofs:[1,2],jacobian:[1,-1],multiplier:.5},
        {kind:'wall',dofs:[3],jacobian:[1],multiplier:.4},
        {kind:'wall',dofs:[0],jacobian:[1],compliance,multiplier:1}
    ];
    const targets=[2,3,-.5,0,0],gradient=new Float64Array(4),fixed=Uint8Array.from([+fixedEntrance,0,0,0]);
    for(let i=0;i<4;i++)for(let j=Math.max(0,i-2);j<=Math.min(3,i+2);j++)
        gradient[i]-=(chain.tangent?chain.tangent[i*5+j-i+2]:hessian[Math.max(i,j)*3+Math.abs(i-j)])*dx[j];
    rows.forEach((r,index)=>{
        r.gap=-r.dofs.reduce((sum,p,k)=>sum+r.jacobian[k]*dx[p],0)-(r.compliance??0)*targets[index]+(index>=3?.2:0);
        const dl=targets[index]-r.multiplier,sign=r.kind==='wall'?-1:1;
        r.dofs.forEach((p,k)=>{gradient[p]-=sign*r.jacobian[k]*dl;});
        r.extraForceDofs?.forEach((p,k)=>{gradient[p]-=r.extraForceJacobian[k]*dl;});
        if(r.geometricHessian)r.dofs.forEach((p,i)=>r.dofs.forEach((q,j)=>{gradient[p]-=r.geometricHessian[i*r.dofs.length+j]*dx[q];}));
    });
    return {layout,chain,rows,dx,targets,options:{rows,gradient,fixed,tolerance:1e-8}};
}

test('condensation preserves hard constraints, fixed supports, geometric tangents and nonsymmetric friction',()=>{
    for(const compliance of [.01,1e-6])for(const fixedEntrance of [false,true])for(const denseTangent of [false,true])
    for(const reuseMatrixAssembly of [false,true])for(const wasmLinearAssembly of [false,true]) {
        const {layout,chain,rows,dx,targets,options}=coupledFixture(compliance,fixedEntrance,denseTangent);
        const reference=solveSharedAxisLinear(createSharedAxisLinear(layout,rows),chain,options);
        const work=createSharedAxisLinear(layout,rows),result=solveSharedAxisLinear(work,chain,{...options,reuseMatrixAssembly,wasmLinearAssembly,condenseCompliantContacts:true});
        assert.ok(reference.converged&&result.converged,JSON.stringify({reference,result}));
        assert.equal(result.condensedContacts,2);
        for(let i=0;i<4;i++)assert.ok(Math.abs(result.increment[i]-dx[i])<1e-8);
        for(let i=0;i<rows.length;i++)assert.ok(Math.abs(rows[i].multiplier+result.multiplierIncrement[i]-targets[i])<1e-7);
        const active=Uint8Array.from(rows,(_,i)=>+(i<3));
        assert.ok(measureUncondensedSharedAxisResidual(chain,options,active,result.increment,result.multiplierIncrement)<=options.tolerance);
        const corrupted=result.multiplierIncrement.slice();corrupted[0]+=.1;
        assert.ok(measureUncondensedSharedAxisResidual(chain,options,active,result.increment,corrupted)>.01);
    }
});

test('fixed compliant indentation recovers its support reaction without a dual unknown',()=>{
    const layout={dofCount:1,band:1},chain={layout,hessian:new Float64Array([1])};
    const rows=[{kind:'wall',dofs:[0],jacobian:[1],gap:-.02,multiplier:0,compliance:.001}];
    const work=createSharedAxisLinear(layout,rows);
    const result=solveSharedAxisLinear(work,chain,{rows,gradient:new Float64Array(1),fixed:new Uint8Array([1]),reuseMatrixAssembly:true,condenseCompliantContacts:true});
    assert.ok(result.converged);assert.equal(result.condensedContacts,1);
    assert.equal(Math.abs(result.increment[0]),0);assert.equal(result.multiplierIncrement[0],20);
});

test('cancellation from near-rigid compliance fails the original-equation check and retries full LU',()=>{
    const layout={dofCount:1,band:1},chain={layout,hessian:new Float64Array([10])};
    const rows=[{kind:'wall',dofs:[0],jacobian:[1],gap:.1,multiplier:0,compliance:1e-18}];
    const result=solveSharedAxisLinear(createSharedAxisLinear(layout,rows),chain,{rows,
        gradient:new Float64Array([20]),fixed:new Uint8Array(1),condenseCompliantContacts:true});
    assert.ok(result.converged);assert.equal(result.condensationFallback,true);
    assert.equal(result.condensationStats.fallbacks,1);
    assert.equal(result.factorizations,3,'Initial working set, condensed attempt and original retry all count');
    assert.ok(result.condensationResidual>1e-8);
    assert.ok(Math.abs(result.increment[0]+.1)<1e-12);
    assert.ok(Math.abs(result.multiplierIncrement[0]-19)<1e-10);
});

test('rigid rows and nonlocal force columns remain in the full block system',()=>{
    const layout={dofCount:8,band:3};
    assert.equal(canCondenseSharedAxisContact({kind:'wall',compliance:0,dofs:[0],jacobian:[1]},layout),false);
    assert.equal(canCondenseSharedAxisContact({kind:'length',compliance:1,dofs:[0],jacobian:[1]},layout),false);
    assert.equal(canCondenseSharedAxisContact({kind:'wall',compliance:.001,dofs:[0,1],extraForceDofs:[7]},layout),false);
});

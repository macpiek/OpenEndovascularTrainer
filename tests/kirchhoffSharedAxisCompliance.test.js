import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisLinear,solveSharedAxisLinear} from '../src/physics/kirchhoffSharedAxisLinear.js';
import {createSharedAxisNative,captureSharedAxisNative,extendSharedAxisNativeRows} from '../src/physics/kirchhoffSharedAxisNative.js';
import {iterateSharedAxisTimeStep,stepSharedAxis} from '../src/physics/kirchhoffSharedAxisTimeStep.js';

test('compliant parallel contacts share load and match the analytic spring equilibrium',()=>{
    for(const count of [1,2,8,40])for(const compliance of [0,.01])
    for(const compactWorkingSet of [false,true])for(const reuseMatrixAssembly of [false,true])for(const wasmLinearAssembly of [false,true]) {
        const layout={dofCount:1,band:1},chain={layout,hessian:new Float64Array([10])};
        const rows=Array.from({length:count},()=>({kind:'wall',dofs:[0],jacobian:[1],gap:.1,multiplier:0,compliance}));
        const result=solveSharedAxisLinear(createSharedAxisLinear(layout,rows),chain,{rows,
            gradient:new Float64Array([20]),fixed:new Uint8Array(1),compactWorkingSet,reuseMatrixAssembly,wasmLinearAssembly,
            simultaneousContactRelease:true,batchActivationSize:64});
        assert.ok(result.converged,JSON.stringify(result));
        const x=result.increment[0],expected=(-count*.1-compliance*20)/(count+compliance*10);
        assert.ok(Math.abs(x-expected)<1e-9,`${x} versus ${expected}`);
        const reactions=Array.from(result.multiplierIncrement);
        assert.ok(Math.abs(10*x+20-reactions.reduce((a,b)=>a+b,0))<1e-9);
        for(const reaction of reactions) {
            assert.ok(reaction>=-1e-9);
            assert.ok(Math.abs(reaction*(.1+x+compliance*reaction))<1e-8);
            if(compliance)assert.ok(Math.abs(reaction-(-x-.1)/compliance)<1e-8);
        }
    }
});

test('a fixed penetrating point can carry a compliant wall reaction',()=>{
    const layout={dofCount:1,band:1},chain={layout,hessian:new Float64Array([1])};
    const rows=[{kind:'wall',dofs:[0],jacobian:[1],gap:-.02,multiplier:0,compliance:.001}];
    const result=solveSharedAxisLinear(createSharedAxisLinear(layout,rows),chain,{rows,
        gradient:new Float64Array(1),fixed:new Uint8Array([1]),reuseMatrixAssembly:true});
    assert.ok(result.converged,JSON.stringify(result));
    assert.equal(Math.abs(result.increment[0]),0);
    assert.ok(Math.abs(result.multiplierIncrement[0]-20)<1e-9);
});

test('simultaneous release solves all separating loaded contacts without changing equilibrium',()=>{
    const n=24,layout={dofCount:n,band:1},chain={layout,hessian:new Float64Array(n).fill(1)};
    for(const compliance of [0,.001]) {
        const rows=Array.from({length:n},(_,i)=>({kind:'wall',dofs:[i],jacobian:[1],gap:1,multiplier:2,compliance}));
        const options={rows,gradient:new Float64Array(n).fill(-2),fixed:new Uint8Array(n),batchActivationSize:64};
        const reference=solveSharedAxisLinear(createSharedAxisLinear(layout,rows),chain,options);
        const actual=solveSharedAxisLinear(createSharedAxisLinear(layout,rows),chain,{...options,simultaneousContactRelease:true});
        assert.ok(reference.converged&&actual.converged);
        assert.equal(actual.factorizations,2);
        assert.ok(actual.factorizations<reference.factorizations);
        for(let i=0;i<n;i++) {
            assert.ok(Math.abs(actual.increment[i])<1e-9);
            assert.ok(Math.abs(actual.multiplierIncrement[i]+2)<1e-9);
        }
    }
});

test('compliance is scoped to the private timestep and cancellation restores its previous value',()=>{
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:20}]});
    s.wallCompliance=.003;s.velocities=s.positions.map(()=>[0,2,0]);
    const before=captureSharedAxisNative(s),iterator=iterateSharedAxisTimeStep(s,1/60,{wallCompliance:1e-6});
    assert.equal(iterator.next().done,false);assert.equal(s.wallCompliance,1e-6);
    iterator.return();assert.equal(s.wallCompliance,.003);
    assert.deepEqual(captureSharedAxisNative(s),before);
    for(const wallCompliance of [-1,NaN,Infinity])assert.throws(()=>stepSharedAxis(s,1/60,{wallCompliance}),RangeError);
});

test('a complete timestep reports geometric penetration separately from compliant equilibrium',()=>{
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:20}]});
    s.fixed.fill(1);
    const edge=s.positions.length-2,dofs=[s.layout.positions[edge],s.layout.positions[edge+1]].flatMap(p=>[p,p+1,p+2]);
    extendSharedAxisNativeRows(s,[{kind:'wall',id:'test-plane',edge,dofs,witness:{face:0,t:1},
        evaluate:({b})=>({gap:b[1]-.02,jacobian:[0,0,0,0,1,0]})}]);
    const result=stepSharedAxis(s,1/60,{wallCompliance:.001,reuseRowBuffers:true,reuseConstraintWork:true,
        reuseMatrixAssembly:true,wasmLinearAssembly:true,projectionMode:'reduced',reuseFrictionAssembly:true});
    assert.ok(result.converged,JSON.stringify(result));
    assert.ok(Math.abs(s.multipliers.at(-1)-20)<1e-9);
    assert.ok(result.residual.length<1e-9);
    assert.equal(result.quality.maxPenetration,.02,'Do not hide penetration inside the compliant gap');
    assert.equal(s.acceptedWallGaps.get('test-plane'),-.02);
    assert.equal(s.wallCompliance,undefined);
});

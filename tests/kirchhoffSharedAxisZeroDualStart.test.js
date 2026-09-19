import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisLinear,solveSharedAxisLinear} from '../src/physics/kirchhoffSharedAxisLinear.js';

test('zero working dual releases unloaded walls together while retaining incoming physical forces',()=>{
    const n=24,layout={dofCount:n,band:1},chain={layout,hessian:new Float64Array(n).fill(1)};
    const rows=Array.from({length:n},(_,i)=>({id:`wall/${i}`,kind:'wall',dofs:[i],jacobian:[1],gap:0,multiplier:i+1}));
    const options={rows,gradient:Float64Array.from(rows,r=>-1-r.multiplier),fixed:new Uint8Array(n),batchRelease:true};
    const before=structuredClone(options);
    const run=zeroDualStart=>solveSharedAxisLinear(createSharedAxisLinear(layout,rows,{lazy:true}),chain,{...options,zeroDualStart});
    const a=run(false),b=run(true);
    assert.ok(a.converged&&b.converged);assert.ok(b.factorizations<=2);assert.ok(a.factorizations>=n);
    for(let i=0;i<n;i++){
        assert.ok(Math.abs(b.increment[i]-1)<1e-10);
        assert.ok(Math.abs(rows[i].multiplier+b.multiplierIncrement[i])<1e-10);
        assert.ok(Math.abs(a.increment[i]-b.increment[i])<1e-10);
    }
    assert.deepEqual(options,before);
});

test('failed zero-dual trial falls back to the original physical dual and certifies its result',()=>{
    const layout={dofCount:2,band:1},chain={layout,hessian:new Float64Array([1,1])};
    const rows=[0,1].map(i=>({id:`wall/${i}`,kind:'wall',dofs:[i],jacobian:[1],gap:0,multiplier:i+1}));
    const options={rows,gradient:new Float64Array([-2,-3]),fixed:new Uint8Array(2),batchRelease:true,zeroDualStart:true,compactWorkingSet:false};
    const w=createSharedAxisLinear(layout,rows),original=w.lu.solve.bind(w.lu);let calls=0;
    w.lu.solve=(...args)=>++calls===1?false:original(...args);
    const r=solveSharedAxisLinear(w,chain,options);
    assert.ok(r.converged);assert.equal(r.batchFallback,true);assert.equal(r.factorizations,calls);
    assert.ok(r.increment.every(v=>Math.abs(v-1)<1e-10));
    assert.ok(r.multiplierIncrement.every((v,i)=>Math.abs(v+rows[i].multiplier)<1e-10));
});

test('loaded feasible contact systems retain primal equilibrium and complementarity across starting duals',()=>{
    for(let seed=1;seed<=60;seed++){
        let state=seed;const random=()=>((state=(Math.imul(state,1664525)+1013904223)>>>0)/2**32);
        const n=3,layout={dofCount:n,band:n},hessian=new Float64Array(n*n);for(let i=0;i<n;i++)hessian[i*n]=1;
        const physical=Float64Array.from({length:n},()=>random()*2-1),gradient=physical.slice();
        const rows=Array.from({length:10},(_,id)=>{
            const jacobian=Array.from({length:n},()=>random()*2-1),multiplier=id<2?random():0;
            jacobian.forEach((v,i)=>{gradient[i]-=multiplier*v;});
            return {id,kind:'wall',dofs:[0,1,2],jacobian,multiplier,gap:random()*.2};
        });
        const options={rows,gradient,fixed:new Uint8Array(n),batchRelease:true},chain={layout,hessian};
        const run=zeroDualStart=>solveSharedAxisLinear(createSharedAxisLinear(layout,rows,{lazy:true}),chain,{...options,zeroDualStart});
        const a=run(false),b=run(true);assert.ok(a.converged&&b.converged,`seed ${seed}`);
        const balance=Float64Array.from(physical,(v,i)=>v+b.increment[i]);
        rows.forEach((r,i)=>{
            const lambda=r.multiplier+b.multiplierIncrement[i],gap=r.gap+r.jacobian.reduce((sum,v,k)=>sum+v*b.increment[k],0);
            assert.ok(lambda>=-1e-8&&gap>=-1e-8);assert.ok(Math.abs(lambda*gap)<1e-8);
            r.jacobian.forEach((v,k)=>{balance[k]-=lambda*v;});
        });
        for(let i=0;i<n;i++){assert.ok(Math.abs(balance[i])<1e-8);assert.ok(Math.abs(a.increment[i]-b.increment[i])<1e-8);}
    }
});

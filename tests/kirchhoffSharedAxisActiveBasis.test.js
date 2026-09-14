import test from 'node:test';
import assert from 'node:assert/strict';
import { prepareSharedAxisActiveBasis } from '../src/physics/kirchhoffSharedAxisActiveBasis.js';
import { createSharedAxisLayout, createSharedAxisLinear, solveSharedAxisLinear } from '../src/physics/kirchhoffSharedAxisLinear.js';

function force(rows,dual,count) {
    const f=new Float64Array(count);
    rows.forEach((r,i)=>r.dofs.forEach((d,k)=>{f[d]+=(r.kind==='wall'?-1:1)*dual[i]*r.jacobian[k];}));
    return f;
}
function checkForcePreservingPivot(rows,count) {
    const copy=structuredClone(rows),dual=Float64Array.from(rows,r=>r.multiplier);
    const before=force(rows,dual,count),activeSet=new Uint8Array(rows.length).fill(1),trace=[];
    const result=prepareSharedAxisActiveBasis({rows,dual,activeSet,fixed:new Uint8Array(count),trace});
    assert.ok(result.converged,JSON.stringify(result));assert.ok(result.pivots>0);
    const after=force(rows,dual,count);
    after.forEach((v,i)=>assert.ok(Math.abs(v-before[i])<1e-10));
    rows.forEach((r,i)=>{if(r.kind==='wall')assert.ok(dual[i]>=0);if(!activeSet[i])assert.equal(dual[i],0);});
    assert.deepEqual(rows,copy,'Preparing a working basis must not overwrite physical reactions');
    return {activeSet,dual};
}

test('warm-start reduction preserves endpoint forces and their moment for interior sample contacts',()=>{
    const dofs=[0,1,2,3,4,5];
    for(const t of [.25,2/3]) {
        const rows=[[1,0,0],[0,1,0],[0,0,1],[1,1,1]].map((n,i)=>({
            kind:'wall',dofs,jacobian:[...n.map(v=>(1-t)*v),...n.map(v=>t*v)],gap:[.1,.2,.3,.1][i],multiplier:i+1
        }));
        const {activeSet}=checkForcePreservingPivot(rows,6);
        assert.equal(activeSet.reduce((sum,v)=>sum+v,0),3);
    }
});

test('basis reduction includes signed length reactions and contacts at different positions of one segment',()=>{
    const dofs=[0,1,2,3,4,5];
    const rows=[{kind:'length',dofs,jacobian:[-1,0,0,1,0,0],gap:0,multiplier:-2},
        ...[0,.5,1].map((t,i)=>({kind:'wall',dofs,jacobian:[1-t,0,0,t,0,0],gap:.1+.1*t,multiplier:i+1}))];
    const {activeSet}=checkForcePreservingPivot(rows,6);
    assert.equal(activeSet[0],1);assert.equal(activeSet.reduce((sum,v)=>sum+v,0),2);
});

test('dependent parallel contacts retain the tighter inequality and solve the full global balance',()=>{
    const layout=createSharedAxisLayout([['wire']]),p=layout.positions[1],dofs=[0,1,2,p,p+1,p+2];
    const rows=[-1,-2].map(gap=>({kind:'wall',dofs,jacobian:[0,0,0,1,0,0],gap,multiplier:1}));
    const chain={layout,hessian:new Float64Array(layout.dofCount*layout.band)};
    for(let i=0;i<layout.dofCount;i++)chain.hessian[i*layout.band]=1;
    const fixed=new Uint8Array(layout.dofCount).fill(1);fixed[p]=0;
    const gradient=force(rows,rows.map(r=>r.multiplier),layout.dofCount);
    const result=solveSharedAxisLinear(createSharedAxisLinear(layout,rows),chain,{rows,gradient,fixed});
    assert.ok(result.converged,JSON.stringify(result));assert.ok(Math.abs(result.increment[p]-2)<1e-9);
    const reactions=rows.map((r,i)=>r.multiplier+result.multiplierIncrement[i]);
    assert.ok(Math.abs(reactions[0])<1e-9);assert.ok(Math.abs(reactions[1]-2)<1e-9);
    rows.forEach((r,i)=>assert.ok(r.gap+result.increment[p]>=-1e-9&&Math.abs((r.gap+result.increment[p])*reactions[i])<1e-9));
});

test('infeasible opposing inequalities fail explicitly instead of discarding one wall',()=>{
    const rows=[{kind:'wall',dofs:[0],jacobian:[1],gap:-1,multiplier:1},
        {kind:'wall',dofs:[0],jacobian:[-1],gap:0,multiplier:1}];
    const dual=new Float64Array([1,1]),activeSet=new Uint8Array([1,1]);
    const result=prepareSharedAxisActiveBasis({rows,dual,activeSet,fixed:new Uint8Array(1)});
    assert.equal(result.converged,false);assert.equal(result.failure,'incompatible-active-constraints');
    assert.deepEqual(Array.from(dual),[1,1]);assert.deepEqual(Array.from(activeSet),[1,1]);
});

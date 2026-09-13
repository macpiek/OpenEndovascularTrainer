import test from 'node:test';
import assert from 'node:assert/strict';
import {Quaternion,Vector3} from 'three';
import {createSharedAxisLayout,createSharedAxisLinear,solveSharedAxisLinear} from '../src/physics/kirchhoffSharedAxisLinear.js';
import {createSharedAxisNative,assembleSharedAxisNative,applySharedAxisNativeIncrement} from '../src/physics/kirchhoffSharedAxisNative.js';
import {prepareSharedAxisDynamicStep,sharedAxisRotationalInertia} from '../src/physics/kirchhoffSharedAxisDynamics.js';
import {nativeHingeTorqueTangent} from '../src/physics/kirchhoffSharedAxisMaterialTangent.js';
import {prepareSharedAxisActiveBasis} from '../src/physics/kirchhoffSharedAxisActiveBasis.js';

const close=(a,b,tolerance=1e-9)=>assert.ok(Math.abs(a-b)<=tolerance*Math.max(1,Math.abs(a),Math.abs(b)),`${a} vs ${b}`);
function random(seed) {return ()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/2**32;};}
function force(rows,dual,n) {
    const out=new Float64Array(n);
    rows.forEach((r,i)=>r.dofs.forEach((p,k)=>{out[p]+=(r.kind==='wall'?-1:1)*dual[i]*r.jacobian[k];}));
    return out;
}

test('compact KKT matches full linear solve when a loaded wall releases but its geometric Hessian remains',()=>{
    const layout=createSharedAxisLayout([['wire']]),p=layout.positions[1],dofs=[0,1,2,p,p+1,p+2];
    const geometricHessian=new Float64Array(36);geometricHessian[3*6+3]=3;
    const rows=[
        {id:'length',kind:'length',dofs,jacobian:[0,0,0,0,1,0],gap:-.3,multiplier:.4},
        {id:'released-curved-wall',kind:'wall',dofs,jacobian:[0,0,0,1,0,0],gap:10,multiplier:2,geometricHessian},
        {id:'inactive-wall',kind:'wall',dofs,jacobian:[0,0,0,-1,0,0],gap:8,multiplier:0},
        {id:'active-wall',kind:'wall',dofs,jacobian:[0,0,0,0,0,1],gap:-.5,multiplier:.2}
    ];
    const chain={layout,hessian:new Float64Array(layout.dofCount*layout.band)};
    for(let i=0;i<layout.dofCount;i++)chain.hessian[i*layout.band]=2;
    const fixed=new Uint8Array(layout.dofCount).fill(1);fixed.fill(0,p,p+3);
    const gradient=force(rows,rows.map(r=>r.multiplier),layout.dofCount);
    [1,.8,-.6].forEach((v,k)=>{gradient[p+k]+=v;});
    const results=[true,false].map(compactWorkingSet=>{
        const w=createSharedAxisLinear(layout,rows,{lazy:compactWorkingSet});
        const r=solveSharedAxisLinear(w,chain,{rows,gradient,fixed,compactWorkingSet});
        assert.equal(r.converged,true,JSON.stringify(r));
        return {increment:r.increment.slice(),multiplierIncrement:r.multiplierIncrement.slice()};
    });
    for(const field of ['increment','multiplierIncrement'])results[0][field].forEach((v,i)=>close(v,results[1][field][i]));
    close(results[0].increment[p],-.2); // H = 2 + 3, even after the wall releases.
    close(results[0].increment[p+1],.3);close(results[0].increment[p+2],.5);
    close(results[0].multiplierIncrement[1],-2);
});

test('gradient-only assembly preserves complete material, kinetic and contact residuals and invalidates tangent',()=>{
    const rng=random(0x601c);
    for(const tangentMode of ['newton','gauss-newton']) {
        const s=createSharedAxisNative({tools:[{id:'wire',insertion:30},{id:'catheter',insertion:22.5,type:'berenstein'}],
            wallSample:({b})=>{
                const hessian=new Float64Array(36);hessian[4*6+4]=-1;
                return {gap:10-.5*b[1]**2,jacobian:[0,0,0,0,-b[1],0],hessian};
            }});
        s.velocities=s.positions.map(()=>[rng()-.5,rng()-.5,rng()-.5]);
        prepareSharedAxisDynamicStep(s,1/120);
        const dx=Float64Array.from({length:s.layout.dofCount},()=>.01*(rng()-.5));
        applySharedAxisNativeIncrement(s,dx,new Float64Array(s.multipliers.length));
        s.multipliers.forEach((_,i)=>{s.multipliers[i]=s.definitions[i].kind==='length'?rng()-.5:rng();});
        const full=assembleSharedAxisNative(s,{tangentMode}),gradient=s.chain.gradient.slice();
        assert.equal(s.chain.hessianValid,true);
        const fast=assembleSharedAxisNative(s,{tangentMode,withTangent:false});
        assert.equal(s.chain.hessianValid,false);
        for(const key of ['energy','force','torque','constraint'])close(fast[key],full[key],1e-11);
        s.chain.gradient.forEach((v,i)=>close(v,gradient[i],1e-11));
        fast.rows.forEach((r,i)=>{close(r.gap,full.rows[i].gap);assert.deepEqual(r.jacobian,full.rows[i].jacobian);});
        assembleSharedAxisNative(s,{tangentMode});assert.equal(s.chain.hessianValid,true);
    }
});

test('closed-form rotational inertia matches the general native material tangent over the short arc',()=>{
    const rng=random(0xc0111de),axis=()=>new Vector3(rng()-.5,rng()-.5,rng()-.5).normalize();
    const angles=[0,1e-12,1e-6,.249999,.25,.250001,Math.PI-1e-7,
        ...Array.from({length:80},()=>rng()*(Math.PI-1e-5))];
    for(const angle of angles) {
        const previous=new Quaternion().setFromAxisAngle(axis(),rng()*2*Math.PI);
        const current=previous.clone().multiply(new Quaternion().setFromAxisAngle(axis(),angle));
        if(rng()<.5)current.set(-current.x,-current.y,-current.z,-current.w);
        const weight=10**(-2+5*rng());
        const general=nativeHingeTorqueTangent(previous,current,{x:0,y:0,z:0},[1/weight,1/weight,1/weight]);
        const fast=sharedAxisRotationalInertia(previous,current,weight);
        const gradientOnly=sharedAxisRotationalInertia(previous,current,weight,false);
        close(fast.energy,general.energy,1e-9);close(gradientOnly.energy,fast.energy,1e-12);
        for(let i=0;i<3;i++) {
            close(fast.torque[i],general.torque[i+3],1e-8);close(gradientOnly.torque[i],fast.torque[i],1e-12);
            for(let j=0;j<3;j++)close(fast.jacobian[i*3+j],general.jacobian[(i+3)*6+j+3],1e-8);
        }
        assert.equal(gradientOnly.jacobian,undefined);
    }
});

test('seeded dependent wall bases preserve free generalized force without modifying physical reactions',()=>{
    const rng=random(0xb4515);
    for(let trial=0;trial<40;trial++) {
        const n=12,free=9,dofs=Array.from({length:n},(_,i)=>i),rows=[];
        // Independent local coordinates followed by randomized nonnegative
        // combinations model coincident/interior finite-face witnesses.
        for(let i=0;i<free;i++)rows.push({id:`base-${i}`,kind:i<2?'length':'wall',dofs,
            jacobian:dofs.map(j=>j===i?1:j>=free?rng()-.5:0),gap:0,multiplier:i<2?rng()-.5:.1+2*rng()});
        for(let i=0;i<7;i++) {
            const coefficients=Array.from({length:free},()=>rng());
            rows.push({id:`dependent-${i}`,kind:'wall',dofs,gap:0,multiplier:.1+rng(),
                jacobian:dofs.map(j=>j<free?coefficients[j]:rng()-.5)});
        }
        const fixed=new Uint8Array(n);fixed.fill(1,free);
        const dual=Float64Array.from(rows,r=>r.multiplier),before=force(rows,dual,n),snapshot=structuredClone(rows);
        const activeSet=new Uint8Array(rows.length).fill(1);
        const r=prepareSharedAxisActiveBasis({rows,fixed,dual,activeSet});
        assert.equal(r.converged,true,JSON.stringify(r));assert.ok(r.pivots>=7);
        const after=force(rows,dual,n);for(let i=0;i<free;i++)close(after[i],before[i],1e-8);
        rows.forEach((row,i)=>{if(row.kind==='wall')assert.ok(dual[i]>=0);if(!activeSet[i])assert.equal(dual[i],0);});
        assert.deepEqual(rows,snapshot);
    }
});

test('seeded redundant unilateral systems match the independently known constrained minimizer',()=>{
    const rng=random(0x600d),layout=createSharedAxisLayout([['wire']]),p=layout.positions[1];
    for(let trial=0;trial<30;trial++) {
        const x=.2+rng(),y=.1+rng(),rows=[];
        for(let i=0;i<9;i++) {
            const a=i===0?1:i===1?0:rng(),b=i===1?1:i===0?0:rng();
            rows.push({id:`wall-${i}`,kind:'wall',dofs:[p,p+1],jacobian:[a,b],gap:-a*x-b*y,multiplier:.1+rng()});
        }
        const chain={layout,hessian:new Float64Array(layout.dofCount*layout.band)};
        for(let i=0;i<layout.dofCount;i++)chain.hessian[i*layout.band]=1;
        const fixed=new Uint8Array(layout.dofCount).fill(1);fixed[p]=fixed[p+1]=0;
        const gradient=force(rows,rows.map(r=>r.multiplier),layout.dofCount);
        const result=solveSharedAxisLinear(createSharedAxisLinear(layout,rows,{lazy:true}),chain,{rows,gradient,fixed});
        assert.equal(result.converged,true,JSON.stringify(result));
        close(result.increment[p],x,1e-8);close(result.increment[p+1],y,1e-8);
        rows.forEach((row,i)=>{
            const lambda=row.multiplier+result.multiplierIncrement[i],gap=row.gap+row.jacobian[0]*result.increment[p]+row.jacobian[1]*result.increment[p+1];
            assert.ok(lambda>=-1e-8);assert.ok(gap>=-1e-8);close(lambda*gap,0,1e-8);
        });
    }
});

test('sparse basis tracks fill and repeated cancellations across a 100-node connected chain',()=>{
    const rng=random(901),nodes=100,n=3*nodes,rows=[];
    const points=Array.from({length:nodes},(_,i)=>[i*5,4*Math.sin(i*.1),3*Math.cos(i*.07)]);
    const fixed=new Uint8Array(n);fixed.fill(1,0,6);
    for(let edge=0;edge<nodes-1;edge++) {
        const d=points[edge+1].map((v,k)=>v-points[edge][k]),length=Math.hypot(...d),t=d.map(v=>v/length);
        rows.push({kind:'length',dofs:Array.from({length:6},(_,i)=>3*edge+i),jacobian:[...t.map(v=>-v),...t],gap:0,multiplier:rng()-.5});
    }
    for(let edge=2;edge<nodes-1;edge++) {
        const t=.2+.6*rng(),angle=2*Math.PI*rng(),normal=[.1*rng(),Math.cos(angle),Math.sin(angle)];
        const dofs=Array.from({length:6},(_,i)=>3*edge+i),jacobian=[...normal.map(v=>(1-t)*v),...normal.map(v=>t*v)];
        rows.push({kind:'wall',dofs,jacobian,gap:0,multiplier:.1+rng()});
        if(edge%10===0)rows.push({kind:'wall',dofs,jacobian:jacobian.map(v=>2*v),gap:0,multiplier:.1+rng()});
    }
    const dual=Float64Array.from(rows,r=>r.multiplier),activeSet=new Uint8Array(rows.length).fill(1),before=force(rows,dual,n);
    const result=prepareSharedAxisActiveBasis({rows,fixed,dual,activeSet});
    assert.equal(result.converged,true);assert.equal(result.pivots,9);
    const after=force(rows,dual,n);
    for(let i=6;i<n;i++)close(after[i],before[i],1e-10);
    rows.forEach((r,i)=>{if(r.kind==='wall')assert.ok(dual[i]>=0);if(!activeSet[i])assert.equal(dual[i],0);});
});

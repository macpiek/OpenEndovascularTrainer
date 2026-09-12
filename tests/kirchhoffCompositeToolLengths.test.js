import assert from 'node:assert/strict';
import test from 'node:test';
import { createCompositeChainLayout } from '../src/physics/kirchhoffCompositeChain.js';
import { createCompositeToolLengthWorkspace as create, evaluateCompositeToolLengths as evaluate } from '../src/physics/kirchhoffCompositeToolLengths.js';

const close = (a,b,t=1e-9) => assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t*(1+Math.max(Math.abs(a),Math.abs(b))), `${a} != ${b}`);
const basis = [[1,0,0],[0,.8,.6],[0,-.6,.8]];
function fixture() {
    const layout=createCompositeChainLayout([['wire','catheter'],['wire','catheter'],['wire']]);
    const modes=[1,2].map((node,i)=>({node,basis:basis.map(v=>[...v]),relativeDofs:[3*i,3*i+1,3*i+2]}));
    const q=[[0,0,0],[2,.3,-.1],[4,.2,.4],[6,.1,.5]], rho=Float64Array.from([.08,.03,-.02,.04,-.05,.01]);
    const restLengths=new Map([['wire',Float64Array.from([1.95,2.07,1.99])],['catheter',Float64Array.from([2.02,2.1,NaN])]]);
    const multipliers=Float64Array.from([.7,-.4,-.6,.3,1.1]);
    const positions=(positions=q,offset=rho)=>{
        const wire=positions.map(p=>[...p]);
        modes.forEach(m=>m.basis.forEach((b,j)=>b.forEach((v,k)=>wire[m.node][k]+=v*offset[m.relativeDofs[j]])));
        return new Map([['wire',wire],['catheter',positions.map(p=>[...p])]]);
    };
    const workspace=create({layout,modes});
    return {layout,modes,q,rho,restLengths,multipliers,positions,workspace};
}
function physicalWork(f,q=f.q,rho=f.rho) {
    const p=f.positions(q,rho); let value=0, index=0;
    for(let e=0;e<f.layout.nodeCount-1;e++)for(const id of f.layout.edgeToolIds[e]) {
        const x=p.get(id); value+=f.multipliers[index++]*(Math.hypot(...x[e+1].map((v,k)=>v-x[e][k]))-f.restLengths.get(id)[e]);
    }
    return value;
}
function assemble(f,q=f.q,rho=f.rho) {
    evaluate({toolPositions:f.positions(q,rho),restLengths:f.restLengths,multipliers:f.multipliers,tolerance:1e-8},f.workspace);
    const n=f.layout.dofCount+f.rho.length, g=Float64Array.from([...f.workspace.commonGradient,...f.workspace.relativeGradient]),H=new Float64Array(n*n);
    for(const row of f.workspace.rows) {
        const ids=[...row.commonDofs,...Array.from(row.relativeDofs,d=>d+f.layout.dofCount)];
        ids.forEach((r,i)=>ids.forEach((c,j)=>H[n*r+c]+=row.geometricTangent[ids.length*i+j]));
    }
    return {g,H,n};
}

test('independent physical material rest lengths expose the axial obstruction of transverse-only motion',()=>{
    const layout=createCompositeChainLayout([['wire','catheter'],['wire','catheter']]);
    const q=[[0,0,0],[2,0,0],[4,0,0]], lateral=.0405;
    const restLengths=new Map([['wire',[2,2]],['catheter',[2,2]]]);
    const workspace=create({layout}), multipliers=new Float64Array(4);
    const wire=q.map((p,i)=>[p[0],i===1?lateral:0,0]);
    evaluate({toolPositions:new Map([['wire',wire],['catheter',q]]),restLengths,multipliers,tolerance:1e-8},workspace);
    assert.equal(workspace.converged,false);
    close(workspace.rows[0].residual,.000410020470803,1e-12);
    assert.equal(workspace.rows[1].residual,0);
    const axial=Math.sqrt(4-lateral*lateral)-2;
    wire[1][0]+=axial; wire[2][0]+=2*axial;
    evaluate({toolPositions:new Map([['wire',wire],['catheter',q]]),restLengths,multipliers,tolerance:1e-12},workspace);
    assert.equal(workspace.converged,true); close(wire[2][0]-q[2][0],-.000820209093,1e-12);
    assert.equal(workspace.certified,false);
    assert.throws(()=>evaluate({toolPositions:new Map(),restLengths,multipliers,tolerance:0},workspace),/positive/);
    assert.equal(workspace.converged,false);assert.equal(workspace.operatorReady,false);
});

test('signed reactions and the full stress tangent match independent physical constraint-work finite differences',()=>{
    const f=fixture(),base=assemble(f),h=2e-6;
    for(let col=0;col<base.n;col++) {
        const qp=f.q.map(p=>[...p]),qm=f.q.map(p=>[...p]),rp=f.rho.slice(),rm=f.rho.slice();
        if(col>=f.layout.dofCount){rp[col-f.layout.dofCount]+=h;rm[col-f.layout.dofCount]-=h;}
        else {
            const node=Array.from(f.layout.positions).findIndex(d=>col>=d&&col<d+3);
            if(node<0){close(base.g[col],0);for(let row=0;row<base.n;row++)close(base.H[row*base.n+col],0);continue;}
            const k=col-f.layout.positions[node];qp[node][k]+=h;qm[node][k]-=h;
        }
        close(base.g[col],(physicalWork(f,qp,rp)-physicalWork(f,qm,rm))/(2*h),2e-9);
        const plus=assemble(f,qp,rp),minus=assemble(f,qm,rm);
        for(let row=0;row<base.n;row++)close(base.H[row*base.n+col],(plus.g[row]-minus.g[row])/(2*h),3e-9);
    }
});

test('length Jacobians differentiate actual tool geometry and keep each material metric independent',()=>{
    const f=fixture(),h=2e-6;assemble(f);
    const saved=f.workspace.rows.map(row=>({g:row.jacobian.slice(),residual:row.residual,ids:[...row.commonDofs,...Array.from(row.relativeDofs,d=>d+f.layout.dofCount)]}));
    for(let col=0;col<f.rho.length;col++) {
        const p=f.rho.slice(),m=f.rho.slice();p[col]+=h;m[col]-=h;
        assemble(f,f.q,p);const plus=f.workspace.rows.map(r=>r.residual);
        assemble(f,f.q,m);const minus=f.workspace.rows.map(r=>r.residual);
        saved.forEach((row,j)=>{const i=row.ids.indexOf(f.layout.dofCount+col);close((plus[j]-minus[j])/(2*h),i<0?0:row.g[i],1e-9);});
    }
    f.restLengths.get('wire')[0]+=.2;assemble(f);
    f.workspace.rows.forEach((r,i)=>close(r.residual,saved[i].residual-(i===0?.2:0)));
});

test('each edge reaction has zero resultant and moment, and mapped relative forces are counted once',()=>{
    const f=fixture();assemble(f);const positions=f.positions();
    for(const row of f.workspace.rows) {
        const p=positions.get(row.toolId),forces=[row.jacobian.slice(0,3),row.jacobian.slice(3,6)].map(v=>Array.from(v,x=>x*row.multiplier));
        const moment=[0,0,0];
        for(let k=0;k<3;k++)close(forces[0][k]+forces[1][k],0);
        for(let e=0;e<2;e++){const x=p[row.edge+e],F=forces[e];moment[0]+=x[1]*F[2]-x[2]*F[1];moment[1]+=x[2]*F[0]-x[0]*F[2];moment[2]+=x[0]*F[1]-x[1]*F[0];}
        moment.forEach(v=>close(v,0));
        assert.deepEqual(row.forceColumn,row.jacobian);
        if(row.toolId==='catheter')assert.equal(row.relativeDofs.length,0);
    }
    const copy=assemble(f);f.multipliers.fill(0);const zero=assemble(f);
    assert.ok(copy.g.some(v=>Math.abs(v)>.1));assert.ok(zero.g.every(v=>v===0)&&zero.H.every(v=>v===0));
    assert.equal(f.workspace.rows.length,5);
});

test('frozen pullback owns bases and indices; missing material history or invalid topology is explicit',()=>{
    const f=fixture(),old=assemble(f);f.modes[0].basis[0][0]=9;f.modes[0].relativeDofs[0]=99;
    // Supply unchanged physical positions; perturbing caller-owned descriptors
    // cannot mutate the workspace derivative or geometric tangent.
    const positions=new Map([['wire',f.q.map(p=>[...p])],['catheter',f.q.map(p=>[...p])]]);
    evaluate({toolPositions:positions,restLengths:f.restLengths,multipliers:f.multipliers,tolerance:1e-8},f.workspace);
    assert.deepEqual(f.workspace.rows[0].relativeDofs,Int32Array.from([0,1,2]));
    assert.equal(f.workspace.rows[0].vectors[6][0],1);
    assert.ok(old.g.every(Number.isFinite));
    assert.throws(()=>evaluate({toolPositions:positions,restLengths:new Map([['catheter',[2,2,2]]]),multipliers:f.multipliers,tolerance:1e-8},f.workspace),/independent rest/);
    assert.throws(()=>create({layout:f.layout,modes:[{node:2,basis,relativeDofs:[0,1,2]},{node:1,basis,relativeDofs:[3,4,5]}]}),/Ordered/);
    assert.throws(()=>create({layout:f.layout,modes:[{node:1,basis:[[1,0,0],[1,0,0]],relativeDofs:[0,1]}]}),/orthonormal/);
});

test('gradient-only trials retain exact length reactions but never supply stale stress tangents to a direction',()=>{
    const f=fixture();assemble(f);const full=structuredClone(f.workspace);
    f.workspace.rows.forEach(r=>r.geometricTangent.fill(NaN));
    const w=evaluate({toolPositions:f.positions(),restLengths:f.restLengths,multipliers:f.multipliers,tolerance:1e-8,order:'gradient'},f.workspace);
    assert.equal(w.hessianValid,false);assert.equal(w.operatorReady,true);assert.deepEqual(w.commonGradient,full.commonGradient);assert.deepEqual(w.relativeGradient,full.relativeGradient);
    w.rows.forEach((r,i)=>{assert.equal(r.residual,full.rows[i].residual);assert.deepEqual(r.jacobian,full.rows[i].jacobian);
        assert.ok(r.geometricTangent.every(Number.isNaN));assert.equal(r.geometricTangentValid,false);});
    evaluate({toolPositions:f.positions(),restLengths:f.restLengths,multipliers:f.multipliers,tolerance:1e-8},w);
    assert.equal(w.hessianValid,true);w.rows.forEach((r,i)=>{assert.equal(r.geometricTangentValid,true);assert.deepEqual(r.geometricTangent,full.rows[i].geometricTangent);});
});

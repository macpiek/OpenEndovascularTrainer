import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {createCompositeContactPullback as create,createCompositeContactPullbackFactory as factory,pullbackCompositeContact as pullback} from '../src/physics/kirchhoffCompositeContactPullback.js';
import {evaluateKirchhoffLumenSegmentContact} from '../src/physics/kirchhoffLumenContact.js';
import {createCompositeLumenSideGeometryWorkspace,differentiateCompositeLumenSideContact} from '../src/physics/kirchhoffCompositeLumenSideGeometry.js';

const close=(a,b,t=2e-9)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t*(1+Math.max(Math.abs(a),Math.abs(b))),`${a} != ${b}`);

test('normal chart sharing preserves physical scatter while every sample owns its numeric arrays',()=>{
    const f=fixture(),source=structuredClone({layout:f.layout,modes:f.modes,points:f.points}),make=factory(f),raw=geometry(f.physical());
    const a=pullback(raw,make(f.points)),b=pullback(raw,make(f.points)),cold=pullback(raw,create(f));
    for(const key of ['gapJacobian','normalForceColumn','normalDerivative']){assert.deepEqual(a[key],cold[key]);assert.deepEqual(b[key],cold[key]);assert.notEqual(a[key].buffer,b[key].buffer);}
    assert.equal(make.matches(source),true);
    f.layout.positions[1]+=1;f.modes[1].basis[0][0]+=.3;assert.equal(make.matches(f),false);
    assert.deepEqual(pullback(raw,make(source.points)).normalDerivative,cold.normalDerivative);
    assert.throws(()=>make([{toolId:'missing',node:1}]),/present tool/);
});
test('normal support plans are bounded, ordered, and isolated from published metadata and numeric mutations',()=>{
    const f=fixture(),make=factory(f),raw=geometry(f.physical()),a=make(f.points),expected=structuredClone(pullback(raw,create(f)));
    pullback(raw,a);a.commonDofs.fill(999);a.relativeDofs.fill(999);a.points[0].node=0;a.normalDerivative.fill(999);
    const b=pullback(raw,make(f.points));assert.deepEqual(b,expected);
    assert.equal(make.diagnostics.builds,1);assert.equal(make.diagnostics.hits,1);
    const reversed=f.points.toReversed();
    assert.deepEqual(pullback(raw,make(reversed)),pullback(raw,create({...f,points:reversed})));
    assert.equal(make.diagnostics.builds,2);
    const points=[0,1,2].flatMap(node=>['wire','catheter'].map(toolId=>({toolId,node})));
    for(const left of points)for(const right of points)make([left,right]);
    assert.equal(make.diagnostics.retainedPlans,make.diagnostics.capacity);
    const builds=make.diagnostics.builds;
    assert.deepEqual(pullback(raw,make(f.points)),expected);assert.equal(make.diagnostics.builds,builds+1);
    assert.throws(()=>make([{toolId:'wire',node:NaN}]),/present tool/);
    assert.deepEqual(pullback(raw,b),expected);
});
function fixture() {
    const layout=createCompositeChainLayout([['wire','catheter'],['wire','catheter'],['wire','catheter']]);
    const modes=Array.from({length:4},(_,node)=>({node,basis:[[.8,.6,0],[-.6,.8,0],[0,0,1]],relativeDofs:[3*node,3*node+1,3*node+2]}));
    const points=[{toolId:'wire',node:1},{toolId:'wire',node:2},{toolId:'catheter',node:1},{toolId:'catheter',node:2}];
    const q=[[0,0,0],[2,.1,-.2],[4,.3,.2],[6,.2,.1]],rho=Float64Array.from({length:12},(_,i)=>.08*Math.sin(i+.7));
    const workspace=create({layout,modes,points});
    function physical(positions=q,relative=rho) {
        return points.flatMap(p=>positions[p.node].map((v,k)=>v+(p.toolId==='wire'?modes[p.node].basis.reduce((sum,b,j)=>sum+b[k]*relative[3*p.node+j],0):0)));
    }
    return {layout,modes,points,q,rho,workspace,physical};
}
const weights=[.7,.3,-.3,-.7],K=[[2,.2,0],[.2,3,.1],[0,.1,1]];

test('gradient pullback retains independent physical G/B and cannot publish a stale full normal derivative',()=>{
    const f=fixture(),raw=geometry(f.physical()),full=structuredClone(pullback(raw,f.workspace));
    const gradient={...raw,hessianValid:false};delete gradient.normalDerivative;
    const r=pullback(gradient,f.workspace,{order:'gradient'});
    assert.equal(r.operatorReady,true);assert.equal(r.hessianValid,false);assert.ok(r.normalDerivative.every(Number.isNaN));
    for(const key of ['gapJacobian','normalForceColumn','forceColumn'])assert.deepEqual(r[key],full[key]);
    assert.throws(()=>pullback(gradient,f.workspace),/fresh physical normal derivative/);
    assert.equal(f.workspace.operatorReady,false);assert.ok(f.workspace.normalDerivative.every(Number.isNaN));
    const restored=pullback(raw,f.workspace);assert.equal(restored.hessianValid,true);assert.deepEqual(restored.normalDerivative,full.normalDerivative);
});
// A synthetic non-distance gap and separately normalized force direction.
// Its DB is intentionally nonsymmetric, as it can be for a discrete SDF.
// This independent physical function has no knowledge of joint indices.
function geometry(x) {
    const v=[0,0,0];for(let p=0;p<4;p++)for(let k=0;k<3;k++)v[k]+=weights[p]*x[3*p+k];
    const f=K.map(row=>row.reduce((sum,k,j)=>sum+k*v[j],0)),length=Math.hypot(...f);
    const gap=.6-.5*v.reduce((sum,v,k)=>sum+v*f[k],0),gapJacobian=new Float64Array(12),normalForceColumn=new Float64Array(12),normalDerivative=new Float64Array(144);
    const fK=K.map((_,j)=>f.reduce((sum,v,k)=>sum+v*K[k][j],0));
    for(let i=0;i<12;i++) {
        gapJacobian[i]=-weights[Math.floor(i/3)]*f[i%3];normalForceColumn[i]=gapJacobian[i]/length;
        for(let j=0;j<12;j++)normalDerivative[12*i+j]=-weights[Math.floor(i/3)]*weights[Math.floor(j/3)]*
            (K[i%3][j%3]/length-f[i%3]*fK[j%3]/length**3);
    }
    return {supported:true,gap,gapJacobian,normalForceColumn,normalDerivative,forceColumn:Float64Array.from(normalForceColumn,v=>-v)};
}
function displaced(f,col,delta) {
    const q=f.q.map(p=>[...p]),rho=f.rho.slice(),w=f.workspace;
    if(col>=w.commonDofs.length)rho[w.relativeDofs[col-w.commonDofs.length]]+=delta;
    else {const dof=w.commonDofs[col],node=Array.from(f.layout.positions).findIndex(d=>dof>=d&&dof<d+3);q[node][dof-f.layout.positions[node]]+=delta;}
    return f.physical(q,rho);
}
function independentColumn(f,physicalColumn) {
    const common=new Map(),relative=new Map();
    f.points.forEach((p,i)=>{
        for(let k=0;k<3;k++){const d=f.layout.positions[p.node]+k;common.set(d,(common.get(d)??0)+physicalColumn[3*i+k]);}
        if(p.toolId==='wire')for(let a=0;a<3;a++){const d=3*p.node+a,b=f.modes[p.node].basis[a];relative.set(d,(relative.get(d)??0)+b.reduce((sum,v,k)=>sum+v*physicalColumn[3*i+k],0));}
    });
    return Float64Array.from([...f.workspace.commonDofs].map(d=>common.get(d)??0).concat([...f.workspace.relativeDofs].map(d=>relative.get(d)??0)));
}

test('physical wire/catheter endpoint ordering pulls back once and retains distinct G, B and nonsymmetric DB',()=>{
    const f=fixture(),raw=geometry(f.physical()),w=pullback(raw,f.workspace),g=independentColumn(f,raw.gapJacobian),b=independentColumn(f,raw.normalForceColumn);
    assert.equal(w.dofCount,12);assert.equal(w.commonDofs.length,6);assert.equal(w.relativeDofs.length,6);assert.equal(w.gap,raw.gap);
    w.gapJacobian.forEach((v,i)=>close(v,g[i]));w.normalForceColumn.forEach((v,i)=>close(v,b[i]));
    assert.ok(w.gapJacobian.some((v,i)=>Math.abs(v-w.normalForceColumn[i])>.1));
    assert.ok(w.normalDerivative.some((v,i)=>Math.abs(v-w.normalDerivative[(i%12)*12+Math.floor(i/12)])>.01));
    w.forceColumn.forEach((v,i)=>assert.equal(v,-w.normalForceColumn[i]));
    assert.equal(w.certified,false);
});

test('gap and physical normal-force derivatives match independent finite changes of common and relative positions',()=>{
    const f=fixture(),w=pullback(geometry(f.physical()),f.workspace),G=w.gapJacobian.slice(),DB=w.normalDerivative.slice(),h=2e-6;
    for(let col=0;col<w.dofCount;col++) {
        const plus=geometry(displaced(f,col,h)),minus=geometry(displaced(f,col,-h));
        close(G[col],(plus.gap-minus.gap)/(2*h),2e-9);
        const bp=independentColumn(f,plus.normalForceColumn),bm=independentColumn(f,minus.normalForceColumn);
        for(let row=0;row<w.dofCount;row++)close(DB[row*w.dofCount+col],(bp[row]-bm[row])/(2*h),3e-9);
    }
});

test('common action/reaction cancels under rigid translation and virtual work is preserved without adding relative force twice',()=>{
    const f=fixture(),raw=geometry(f.physical()),w=pullback(raw,f.workspace),d=Float64Array.from({length:w.dofCount},(_,i)=>.2*Math.cos(i));
    const x0=f.physical(),x1=x0.slice();
    for(let j=0;j<w.dofCount;j++){const moved=displaced(f,j,d[j]);for(let i=0;i<x1.length;i++)x1[i]+=moved[i]-x0[i];}
    const physicalWork=raw.normalForceColumn.reduce((sum,v,i)=>sum+v*(x1[i]-x0[i]),0);
    close(w.normalForceColumn.reduce((sum,v,i)=>sum+v*d[i],0),physicalWork,1e-12);
    for(let k=0;k<3;k++) {
        let force=0;for(let i=k;i<w.commonDofs.length;i+=3)force+=w.normalForceColumn[i];close(force,0,1e-12);
        // The rigid common translation nullspace includes relative force
        // rows of DB; adding rho as another world force would destroy it.
        for(let row=0;row<w.dofCount;row++){let value=0;for(let col=k;col<w.commonDofs.length;col+=3)value+=w.normalDerivative[row*w.dofCount+col];close(value,0,1e-12);}
    }
});

test('frozen mapping owns its basis and source ordering while a failed refresh invalidates borrowed numeric outputs',()=>{
    const f=fixture(),raw=geometry(f.physical()),first=pullback(raw,f.workspace),saved=structuredClone(first);
    f.modes[1].basis[0].fill(9);f.points.reverse();const repeated=pullback(raw,f.workspace);
    assert.equal(first,repeated);assert.deepEqual(repeated,saved);
    assert.throws(()=>pullback({...raw,supported:false},f.workspace),/supported/);
    assert.equal(first.operatorReady,false);assert.equal(first.supported,false);assert.ok(first.normalDerivative.every(Number.isNaN));
    assert.deepEqual(pullback(raw,f.workspace),saved);
    assert.throws(()=>pullback({...raw,forceColumn:raw.normalForceColumn},f.workspace),/negative/);
});

test('contact topology rejects absent tools and long correspondence instead of silently growing a global band',()=>{
    const f=fixture();
    assert.throws(()=>create({...f,points:[{toolId:'missing',node:1}]}),/present tool/);
    assert.throws(()=>create({...f,points:[{toolId:'wire',node:0},{toolId:'catheter',node:3}]}),/rebuild its chart/);
    const local=create({...f,points:[{toolId:'catheter',node:1},{toolId:'catheter',node:2}]});
    assert.equal(local.relativeDofs.length,0);assert.equal(local.commonDofs.length,6);
    const reduced=create({...f,modes:[],points:f.points});assert.equal(reduced.relativeDofs.length,0);assert.equal(reduced.certified,false);
});

test('original lumen side detection and its complete physical derivative enter common/rho without additional detection',()=>{
    const f=fixture();f.q[1]=[0,0,0];f.q[2]=[2,0,0];
    f.rho.set([.19,-.08,.01],3);f.rho.set([-.222,.204,.015],6);
    let queries=0;
    const detect=x=>{
        queries++;const input={innerStart:x.slice(0,3),innerEnd:x.slice(3,6),outerStart:x.slice(6,9),outerEnd:x.slice(9,12),
            lumenRadius:.5,innerRadius:.46,innerMaterialSegmentId:'wire-edge',outerMaterialSegmentId:'cat-edge',quadrature:[.25,.5,.75]};
        return {input,contact:evaluateKirchhoffLumenSegmentContact(input).side};
    };
    const original=detect(f.physical()),geometryWorkspace=createCompositeLumenSideGeometryWorkspace();
    assert.equal(original.contact.innerT,.25);assert.ok(original.contact.gap<0);
    const physical=differentiateCompositeLumenSideContact(original,geometryWorkspace);
    assert.equal(physical.supported,true);
    const joint=pullback(physical,f.workspace),G=joint.gapJacobian.slice(),DB=joint.normalDerivative.slice();
    assert.equal(queries,1);assert.equal(joint.gap,original.contact.gap);
    const h=1e-6;
    for(let col=0;col<joint.dofCount;col++) {
        const plus=detect(displaced(f,col,h)),minus=detect(displaced(f,col,-h));
        assert.equal(plus.contact.innerT,.25);assert.equal(minus.contact.innerT,.25);
        close(G[col],(plus.contact.gap-minus.contact.gap)/(2*h),2e-9);
        const gradients=c=>[...c.gradients.inner.flat(),...c.gradients.outer.flat()];
        const bp=independentColumn(f,gradients(plus.contact)),bm=independentColumn(f,gradients(minus.contact));
        for(let row=0;row<joint.dofCount;row++)close(DB[row*joint.dofCount+col],(bp[row]-bm[row])/(2*h),2e-7);
    }
    assert.equal(queries,1+2*joint.dofCount);
    assert.equal(physical.selectionCertified,false);assert.equal(joint.certified,false);
});

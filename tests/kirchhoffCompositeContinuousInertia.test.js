import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeContinuousGeometry as geometry,evaluateCompositeContinuousGeometry as sample,createCompositeContinuousMaterialVelocity as velocityHistory} from '../src/physics/kirchhoffCompositeContinuousGeometry.js';
import {createCompositeMaterialInertiaEdge as create} from '../src/physics/kirchhoffCompositeMaterialInertia.js';
import {createCompositeJointMaterialHistory as history} from '../src/physics/kirchhoffCompositeJointMaterialHistory.js';

const close=(a,b,t=1e-9)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}`);
const vec=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,k)=>close(v,b[k],t));};
const polynomial=(values,interval=[0,1])=>({fractions:interval,bernsteinVelocities:values,interpretation:'quintic-bernstein-material-velocity'});
function fixture() {
    const x=[0,1,2.4,4],edge=geometry({coordinates:x}).edges[1],previous=x.map(s=>[s,.2*s**3,.03*s*s]),dt=.02,
        positions=previous.map((p,i)=>p.map((v,k)=>v+.001*(i+1)*(k+1))),tool={id:'wire',massPerMaterialLength:.13,
            materialMap:{sStart:30,dsDx:1.2,dsDt:[-.6,-.2]},oldVelocityPieces:[polynomial(Array.from({length:6},(_,j)=>[.01*j,.002*j*j,-.003*j]))]},
        input={coordinates:edge.coordinates,continuousGeometry:edge,previousPositions:previous,dt,tool};
    return {x,edge,previous,dt,positions,tool,input};
}

test('the material inertia factory integrates the same continuous geometry velocity and both physical momentum balances',()=>{
    const f=fixture(),op=create(f.input),r=op.evaluate(f.positions);
    assert.equal(op.scope,'prepared-one-material-continuous-quintic-inertia');assert.equal(op.nodeIndices.length,4);assert.equal(op.quadraturePoints,6);
    assert.equal(r.gradient.length,12);assert.equal(r.hessian.length,144);
    for(const s of r.tools[0].samples) {
        const q=sample(f.edge,{positions:f.positions,previousPositions:f.previous,fraction:s.fraction,dt:f.dt,materialMap:f.tool.materialMap});
        vec(s.velocity,q.materialVelocity,3e-14);
    }
    for(let k=0;k<3;k++) {
        close(r.tools[0].momentum[k]-r.tools[0].oldMomentum[k],r.momentumIncrement[k],1e-14);
        close(r.gradient.filter((_,j)=>j%3===k).reduce((s,v)=>s+v,0)*f.dt,r.momentumIncrement[k],5e-14);
    }
});

test('Gauss-6 integrates degree-ten old velocity energy exactly, including a discontinuous old material boundary',()=>{
    const f=fixture(),controls=Array.from({length:6},(_,j)=>[j===5?1:0,0,0]);f.tool.materialMap.dsDt=0;
    for(const pieces of [[polynomial(controls)],[polynomial(controls,[0,.37]),polynomial(controls.map(v=>v.map(a=>2*a)),[.37,1])]]) {
        f.tool.oldVelocityPieces=pieces;const op=create(f.input),r=op.evaluate(f.previous),factor=pieces.length===1?1:.37+4*.63,
            momentumFactor=pieces.length===1?1:.37+2*.63;
        close(r.energy,.5*op.mass*factor/11,3e-16);close(r.oldKineticEnergy,r.energy,0);close(r.kineticEnergy,0,0);
        close(r.tools[0].oldMomentum[0],op.mass*momentumFactor/6,3e-16);
        assert.equal(op.quadraturePoints,6*pieces.length);
    }
});

test('continuous energy, gradient and prepared tangent differentiate all local position columns with feed',()=>{
    const f=fixture(),op=create(f.input),r=structuredClone(op.evaluate(f.positions)),h=1e-6;
    for(let column=0;column<12;column++) {
        const plus=structuredClone(f.positions),minus=structuredClone(f.positions);plus[Math.floor(column/3)][column%3]+=h;minus[Math.floor(column/3)][column%3]-=h;
        const a=structuredClone(op.evaluate(plus)),b=op.evaluate(minus);
        close((a.energy-b.energy)/(2*h),r.gradient[column],2e-8);
        for(let row=0;row<12;row++)close((a.gradient[row]-b.gradient[row])/(2*h),r.hessian[row*12+column],2e-7);
    }
    assert.deepEqual(op.evaluate(f.positions).hessian,r.hessian);
});

test('restricting the continuous field to an affine rod reproduces existing energy, gradient and tangent under exact pullback',()=>{
    const f=fixture(),[a,b]=f.edge.coordinates,old=[[a,.2*a,-.1*a],[b,.2*b,-.1*b]],p=old.map((v,i)=>v.map((x,k)=>x+.002*(i+1)*(k+1))),
        at=(ends,x)=>ends[0].map((v,k)=>v+(x-a)/(b-a)*(ends[1][k]-v)),endpointVelocities=[[.03,.02,-.01],[.01,-.02,.04]],
        bernstein=Array.from({length:6},(_,j)=>endpointVelocities[0].map((v,k)=>v+j/5*(endpointVelocities[1][k]-v)));
    f.input.previousPositions=f.x.map(x=>at(old,x));f.tool.oldVelocityPieces=[polynomial(bernstein)];
    const c=create(f.input).evaluate(f.x.map(x=>at(p,x))),
        affine=create({coordinates:[a,b],previousPositions:old,dt:f.dt,tool:{...f.tool,oldVelocityPieces:undefined,oldMaterialVelocities:endpointVelocities}}).evaluate(p),
        T=f.x.map(x=>[1-(x-a)/(b-a),(x-a)/(b-a)]);
    close(c.energy,affine.energy,2e-14);vec(c.momentumIncrement,affine.momentumIncrement,4e-14);
    for(let i=0;i<6;i++) {
        close(T.reduce((s,t,j)=>s+t[Math.floor(i/3)]*c.gradient[3*j+i%3],0),affine.gradient[i],3e-12);
        for(let k=0;k<6;k++) {
            let value=0;for(let j=0;j<4;j++)for(let l=0;l<4;l++)value+=T[j][Math.floor(i/3)]*c.hessian[(3*j+i%3)*12+3*l+k%3]*T[l][Math.floor(k/3)];
            close(value,affine.hessian[i*6+k],2e-10);
        }
    }
});

test('continuous preparation owns history; gradient evaluation and failed retry revoke all tangent validity',()=>{
    const f=fixture(),op=create(f.input),r=structuredClone(op.evaluate(f.positions));
    f.previous[0][0]=900;f.tool.oldVelocityPieces[0].bernsteinVelocities[0][0]=900;f.tool.materialMap.dsDt[0]=900;
    assert.deepEqual(op.evaluate(f.positions).gradient,r.gradient);
    const gradient=op.evaluate(f.positions,{order:'gradient'});assert.equal(gradient.hessianValid,false);assert.ok(gradient.hessian.every(Number.isNaN));vec(gradient.gradient,r.gradient,0);
    const bad=structuredClone(f.positions);bad[3][2]=Infinity;assert.throws(()=>op.evaluate(bad),/finite/);
    assert.equal(gradient.hessianValid,false);assert.ok(gradient.gradient.every(Number.isNaN));assert.ok(Number.isNaN(gradient.energy));
    assert.deepEqual(op.evaluate(f.positions).hessian,r.hessian);assert.deepEqual(op.evaluate(f.positions).gradient,r.gradient);
});

test('continuous inertia refuses gaps, ambiguous endpoint histories and undeclared polynomial fields',()=>{
    for(const change of [f=>f.tool.oldVelocityPieces[0].fractions[0]=.1,f=>f.tool.oldVelocityPieces[0].fractions[1]=.9,
        f=>f.tool.oldVelocityPieces[0].bernsteinVelocities.pop(),f=>f.tool.oldVelocityPieces[0].interpretation='mesh-velocity',
        f=>f.tool.oldVelocityPieces[0].bernsteinVelocities[5][2]=NaN,f=>f.tool.oldMaterialVelocities=[[0,0,0],[0,0,0]],
        f=>f.input.coordinates=[0,1],f=>f.input.continuousGeometry={...f.edge}]) {
        const f=fixture();change(f);assert.throws(()=>create(f.input));
    }
});

test('captured velocity controls retain the complete accepted material field and survive label restriction',()=>{
    const f=fixture(),record=velocityHistory(f.edge,{id:'wire',positions:f.positions,previousPositions:f.previous,dt:f.dt,materialMap:f.tool.materialMap}),
        h=history({materialVelocities:[{edge:1,tools:[record]}]}),span=record.sEnd-record.sStart;
    for(const fraction of [0,.13,.39,.57,.81,1]) {
        const expected=sample(f.edge,{positions:f.positions,previousPositions:f.previous,dt:f.dt,fraction,materialMap:f.tool.materialMap}).materialVelocity;
        vec(h.sample('wire',record.sStart+span*fraction),expected,5e-14);
    }
    const prepared=h.prepare({coordinates:[0,1],inertiaEdges:[{tools:[{id:'wire',materialMap:{sStart:record.sStart+.2*span,dsDx:.6*span,dsDt:99}}]}]}),p=prepared.inertiaEdges[0].tools[0];
    assert.equal(p.oldMaterialVelocities,undefined);assert.equal(p.compatibleWithAffineEdgeOperator,false);assert.equal(p.requiresContinuousInertia,true);
    assert.equal(p.pieces[0].samples.length,6);
    const restricted=history({materialVelocities:[{edge:8,tools:[{id:'wire',sStart:p.pieces[0].sStart,sEnd:p.pieces[0].sEnd,
        bernsteinVelocities:p.pieces[0].bernsteinVelocities,interpretation:'quintic-bernstein-material-velocity'}]}]});
    for(const fraction of [0,.2,.6,1]) {
        const s=p.pieces[0].sStart+(p.pieces[0].sEnd-p.pieces[0].sStart)*fraction;
        vec(restricted.sample('wire',s),h.sample('wire',s),3e-14);
    }
    assert.throws(()=>record.bernsteinVelocities[0][0]=99,TypeError);
});

test('independent opposite feeds carry both accepted continuous fields across old nodes and into the next inertia preparation',()=>{
    const x=[0,1,2.4,4],g=geometry({coordinates:x}),dt=.02,records=g.edges.map(e=>({edge:e.edge,tools:[]})),own=new Map();
    for(const id of ['wire','catheter']) {
        const sign=id==='wire'?1:-1,origin=id==='wire'?20:100,metric=id==='wire'?1.2:.8,
            previous=x.map(s=>[s,sign*.03*s**3,sign*.01*s*s]),positions=previous.map((v,j)=>v.map((a,k)=>a+sign*.002*(j+1)*(k+1)));
        own.set(id,{positions,previous,origin,metric,sign});
        for(const e of g.edges)records[e.edge].tools.push(velocityHistory(e,{id,positions:e.nodeIndices.map(j=>positions[j]),previousPositions:e.nodeIndices.map(j=>previous[j]),dt,
            materialMap:{sStart:origin+metric*e.coordinates[0],dsDx:metric,dsDt:-sign*.3}}));
    }
    const external=({toolId,s})=>{
        const t=own.get(toolId),start=s<t.origin?t.origin-10:t.origin+4*t.metric;
        return {id:toolId,sStart:start,sEnd:start+10,velocities:[[0,0,0],[0,0,0]],interpretation:'physical-material-velocity'};
    };
    const h=history({materialVelocities:records,reservoir:external}),maps=g.edges.map(e=>({tools:[...own].map(([id,t])=>({id,
        materialMap:{sStart:t.origin+t.metric*e.coordinates[0]-t.sign*.15,dsDx:t.metric,dsDt:-t.sign*.3}}))})),prepared=h.prepare({coordinates:x,inertiaEdges:maps});
    assert.ok(prepared.requiredCuts.length>=2);
    for(const [id,t] of own) {
        const e=g.edges[1],p=prepared.inertiaEdges[1].tools.find(t=>t.id===id);assert.equal(p.pieces.length,2);
        const op=create({coordinates:e.coordinates,continuousGeometry:e,previousPositions:e.nodeIndices.map(j=>t.positions[j]),dt,
            tool:{id,massPerMaterialLength:.13,materialMap:p.materialMap,oldVelocityPieces:p.pieces}}),r=op.evaluate(e.nodeIndices.map(j=>t.positions[j]));
        const expected=[0,0,0];
        for(const s of r.tools[0].samples) {
            const oldX=(s.s-t.origin)/t.metric,source=g.edges.find(e=>oldX>=e.coordinates[0]&&oldX<=e.coordinates[1]),fraction=(oldX-source.coordinates[0])/(source.coordinates[1]-source.coordinates[0]);
            const v=sample(source,{positions:source.nodeIndices.map(j=>t.positions[j]),previousPositions:source.nodeIndices.map(j=>t.previous[j]),fraction,dt,
                materialMap:{sStart:t.origin+t.metric*source.coordinates[0],dsDx:t.metric,dsDt:-t.sign*.3}}).materialVelocity;
            vec(s.oldMaterialVelocity,v,1e-13);v.forEach((a,k)=>expected[k]+=s.massWeight*a);
        }
        vec(r.tools[0].oldMomentum,expected,2e-14);
        // The source is already expressed at current labels; supplying huge
        // map rates to preparation must not advect that query a second time.
        const again=h.prepare({coordinates:x,inertiaEdges:maps.map(e=>({tools:e.tools.map(t=>({...t,materialMap:{...t.materialMap,dsDt:999}}))}))});
        assert.deepEqual(again.inertiaEdges[1].tools.find(t=>t.id===id).pieces,p.pieces);
    }
});

test('mixed polynomial and affine reservoir history preserves degree-ten energy and refuses endpoint-only downgrade',()=>{
    const polynomialRecord={id:'wire',sStart:0,sEnd:1,interpretation:'quintic-bernstein-material-velocity',bernsteinVelocities:Array.from({length:6},(_,j)=>[j===5?1:0,0,0])},
        h=history({materialVelocities:[{edge:4,tools:[polynomialRecord]}],reservoir:()=>({id:'wire',sStart:-1,sEnd:0,interpretation:'physical-material-velocity',velocities:[[2,0,0],[2,0,0]]})}),
        maps=[{tools:[{id:'wire',materialMap:{sStart:-1,dsDx:1,dsDt:0}}]}],p=h.prepare({coordinates:[0,2],inertiaEdges:maps}).inertiaEdges[0].tools[0],
        edge=geometry({coordinates:[0,2]}).edges[0],positions=[[0,0,0],[2,0,0]],input={coordinates:[0,2],previousPositions:positions,dt:.1,
            tool:{id:'wire',materialMap:p.materialMap,massPerMaterialLength:1,oldVelocityPieces:p.pieces}},r=create({...input,continuousGeometry:edge}).evaluate(positions);
    close(r.energy,.5*(4+1/11),2e-15);close(r.tools[0].oldMomentum[0],2+1/6,2e-15);
    assert.equal(p.oldMaterialVelocities,undefined);assert.equal(p.requiresContinuousInertia,true);
    assert.throws(()=>create(input),/OLD MATERIAL|old material/i);
    assert.throws(()=>h.sample('wire',0),{code:'ambiguous-material-history-trace'});
    vec(h.sample('wire',0,{trace:'left'}),[2,0,0],0);vec(h.sample('wire',0,{trace:'right'}),[0,0,0],0);
});

import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeContinuousGeometry as compile,sampleCompositeContinuousBasis as basis,
    evaluateCompositeContinuousGeometry as evaluate,compositeContinuousBezierControls as controls} from '../src/physics/kirchhoffCompositeContinuousGeometry.js';

const close=(a,b,t=1e-10)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}`);
const vec=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,k)=>close(v,b[k],t));};
const local=(edge,p)=>edge.nodeIndices.map(j=>p[j]);

test('shared quintic geometry removes artificial position, curvature and advected bending-rate jumps on nonuniform nodes',()=>{
    for(const x of [[0,1,2,3,4],[0,.6,1.8,3.1,5]]) {
        const g=compile({coordinates:x}),old=x.map(s=>[s,.2*s**3,.07*s*s]),p=old.map(([a,b,c],i)=>[a+.001*i,b+.002*i*i,c-.001*i]),dt=1/120;
        for(let j=1;j<x.length-1;j++) {
            const sample=(e,f)=>evaluate(e,{positions:local(e,p),previousPositions:local(e,old),fraction:f,dt,
                materialMap:{sStart:40+2*e.coordinates[0],dsDx:2,dsDt:-4}});
            const left=sample(g.edges[j-1],1),right=sample(g.edges[j],0);
            for(const key of ['position','positionDx','positionDxx','tangent','tangentDx','positionDt','positionDxDt','materialVelocity','bendingOmega'])vec(left[key],right[key],3e-11);
            vec(left.position,p[j],2e-13);close(left.materialLabel,right.materialLabel,0);
            assert.equal(left.angularVelocity,null,'bending motion does not infer own material spin');
        }
        assert.ok(g.edges.every(e=>e.nodeIndices.length<=4));
    }
});

test('quadratic curves and analytic tangent derivatives are reproduced throughout nonuniform edges',()=>{
    const x=[0,.7,2.1,4],g=compile({coordinates:x}),p=x.map(s=>[s,.3*s*s-.2*s,1+.1*s*s]);
    for(const e of g.edges)for(const f of [0,.13,.5,.81,1]) {
        const s=e.coordinates[0]+f*(e.coordinates[1]-e.coordinates[0]),r=evaluate(e,{positions:local(e,p),fraction:f});
        vec(r.position,[s,.3*s*s-.2*s,1+.1*s*s],3e-14);
        vec(r.positionDx,[1,.6*s-.2,.2*s],4e-14);vec(r.positionDxx,[0,.6,.2],2e-13);
    }
});

test('explicit physical interfaces keep separate one-sided derivatives and do not borrow nodes across a material region',()=>{
    const x=[0,1,2,3,4],g=compile({coordinates:x,interfaces:[2]}),p=[[0,0,0],[1,0,0],[2,0,0],[3,1,0],[4,4,0]],
        left=evaluate(g.edges[1],{positions:local(g.edges[1],p),fraction:1}),right=evaluate(g.edges[2],{positions:local(g.edges[2],p),fraction:0});
    vec(left.position,right.position,0);vec(left.positionDxx,[0,0,0],1e-14);vec(right.positionDxx,[0,2,0],1e-14);
    assert.deepEqual(g.edges[1].nodeIndices,[0,1,2]);assert.deepEqual(g.edges[2].nodeIndices,[2,3,4]);
    const kink=compile({coordinates:[0,1,2],interfaces:[1]}),q=[[0,0,0],[1,0,0],[1,1,0]];
    vec(evaluate(kink.edges[0],{positions:local(kink.edges[0],q),fraction:1}).tangent,[1,0,0],0);
    vec(evaluate(kink.edges[1],{positions:local(kink.edges[1],q),fraction:0}).tangent,[0,1,0],0);
});

test('Bernstein controls and analytic basis describe one curve and its physical position Jacobian',()=>{
    const x=[0,1,2.4,4],e=compile({coordinates:x}).edges[1],p=local(e,x.map(s=>[s,.2*s**3,.1*s*s])),b=controls(e,p),f=.37,
        r=evaluate(e,{positions:p,fraction:f}),N=basis(e,f),choose=[1,5,10,10,5,1];
    const direct=[0,1,2].map(k=>b.reduce((sum,v,i)=>sum+choose[i]*f**i*(1-f)**(5-i)*v[k],0));
    vec(r.position,direct,2e-14);
    const h=1e-6;
    for(let j=0;j<p.length;j++)for(let k=0;k<3;k++) {
        const plus=structuredClone(p),minus=structuredClone(p);plus[j][k]+=h;minus[j][k]-=h;
        const a=evaluate(e,{positions:plus,fraction:f}),c=evaluate(e,{positions:minus,fraction:f});
        close((a.position[k]-c.position[k])/(2*h),N.weights[j],2e-9);
        close((a.positionDx[k]-c.positionDx[k])/(2*h),N.first[j],2e-9);
        close((a.positionDxx[k]-c.positionDxx[k])/(2*h),N.second[j],3e-9);
    }
    N.weights.fill(999);b[0].fill(999);assert.deepEqual(evaluate(e,{positions:p,fraction:f}).position,r.position);
});

test('rigid translation and rotation preserve material velocity and bending angular motion in the corresponding observer',()=>{
    const x=[0,1,2.2,4],e=compile({coordinates:x}).edges[1],p=local(e,x.map(s=>[s,.2*s*s,.07*s**3])),old=p.map(v=>v.map((a,k)=>a-.003*(k+1))),
        input={positions:p,previousPositions:old,fraction:.61,dt:.01,materialMap:{sStart:5,dsDx:1.3,dsDt:[-.2,-.4]}},r=evaluate(e,input),
        R=([a,b,c])=>[c,a,b],shift=[100,-30,70],point=v=>R(v).map((a,k)=>a+shift[k]),rotated=evaluate(e,{...input,positions:p.map(point),previousPositions:old.map(point)});
    vec(rotated.position,point(r.position),3e-14);
    for(const key of ['positionDx','positionDxx','materialVelocity','bendingOmega'])vec(rotated[key],R(r[key]),3e-12);
});

test('continuous geometry validates its source, physical supports and finite motion without extrapolation',()=>{
    for(const input of [{coordinates:[0,0,1]},{coordinates:[0,Infinity]},{coordinates:[0,1],interfaces:[0]},
        {coordinates:[0,1,2],interfaces:[1,1]},{coordinates:[0,1,2],interfaces:[.5]}])assert.throws(()=>compile(input));
    const coordinates=[0,1,2],g=compile({coordinates}),e=g.edges[0],p=[[0,0,0],[1,0,0],[2,0,0]];coordinates[1]=99;
    assert.throws(()=>basis({...e},.5),/compiled/);assert.throws(()=>basis(e,1.01),/extrapolation/);
    assert.throws(()=>evaluate(e,{positions:p.map(()=>[0,0,0]),fraction:.5}),/tangent/);
    assert.throws(()=>evaluate(e,{positions:p,fraction:.5,previousPositions:p,dt:.01,materialMap:{sStart:0,dsDx:0,dsDt:1}}),/metric/);
    assert.throws(()=>{e.nodeIndices[0]=99;},TypeError);vec(evaluate(e,{positions:p,fraction:.5}).position,[.5,0,0],2e-15);
});

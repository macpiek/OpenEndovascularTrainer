import test from 'node:test';
import assert from 'node:assert/strict';
import {Quaternion,Vector3} from 'three';
import {createSharedAxisNative,captureSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {iterateSharedAxisXpbd,xpbdVectorIncrement} from '../src/physics/sharedAxisXpbdExperimental.js';

const finish=it=>{let n;do{n=it.next();}while(!n.done);return n.value;};
const create=extra=>createSharedAxisNative({tools:[{id:'catheter',type:'straight',insertion:60}],...extra});
const close=(a,b,e=1e-9)=>assert.ok(Math.abs(a-b)<=e,`${a} vs ${b}`);
const I=Float64Array.from([1,0,0,0,1,0,0,0,1]);

test('XPBD local block matches analytical compliant spring and accumulates lambda only once',()=>{
    for(const dt of [1/30,1/60,1/120]) {
        const x=[1,2,3],alpha=[.01,.02,.03],lambda=[0,0,0];
        const delta=xpbdVectorIncrement(x,lambda,alpha,dt,[I],[.5]);
        delta.forEach((v,k)=>close(v,-x[k]/(.5+alpha[k]/dt**2)));
        const next=x.map((v,k)=>v+.5*delta[k]);
        const repeat=xpbdVectorIncrement(next,delta,alpha,dt,[I],[.5]);
        repeat.forEach(x=>close(x,0));
    }
});

test('XPBD unloaded straight rod remains at rest and does not claim Newton certification',()=>{
    const s=create(),before=captureSharedAxisNative(s),result=finish(iterateSharedAxisXpbd(s,1/60));
    assert.equal(result.converged,true,JSON.stringify(result));
    assert.equal(result.certificateBound,undefined);assert.equal(result.xpbd.equilibriumCertified,false);
    s.positions.forEach((p,i)=>p.forEach((v,k)=>close(v,before.positions[i][k])));
    assert.equal(result.factorizations,0);
});

test('XPBD cancellation and rejected geometry restore the complete incoming pose and dynamics',()=>{
    const s=create({tools:[{id:'catheter',type:'berenstein',insertion:60}]}),before=captureSharedAxisNative(s);
    s.velocities=s.positions.map(()=>[0,.1,0]);const velocities=s.velocities;
    s.wallFrictionHistory=[{id:'original'}];const history=s.wallFrictionHistory;
    const it=iterateSharedAxisXpbd(s,1/60);assert.equal(it.next().done,false);it.return();
    assert.deepEqual(captureSharedAxisNative(s),before);assert.equal(s.velocities,velocities);assert.equal(s.wallFrictionHistory,history);
    const result=finish(iterateSharedAxisXpbd(s,1/60,{xpbd:{iterations:1,maxShear:1e-14,maxLengthError:1e-14}}));
    assert.equal(result.converged,false);assert.deepEqual(captureSharedAxisNative(s),before);assert.equal(s.dynamicStep,null);
});

test('intrinsic bend responds to rotating the manufactured material frame',()=>{
    const poses=[];
    for(const angle of [0,Math.PI/2]) {
        const s=create({tools:[{id:'catheter',type:'berenstein',insertion:60,shaftStiffness:2,tipStiffness:2}]});
        const rotation=new Quaternion().setFromAxisAngle(new Vector3(1,0,0),angle);
        for(const {body,last} of s.materials)for(let e=0;e<last;e++) {
            const q=new Quaternion(body.orientationX[e],body.orientationY[e],body.orientationZ[e],body.orientationW[e]).premultiply(rotation);
            for(const k of ['X','Y','Z','W'])body['orientation'+k][e]=q[k.toLowerCase()];
        }
        // Loose gates measure covariance of the method, not clinical accuracy.
        for(let step=0;step<10;step++)assert.equal(finish(iterateSharedAxisXpbd(s,1/60,
            {xpbd:{iterations:64,maxShear:.01,maxLengthError:.01}})).converged,true);
        poses.push(s.positions.at(-1));
    }
    assert.ok(Math.hypot(poses[0][1],poses[0][2])>.01);
    const rotated=new Vector3(...poses[0]).applyAxisAngle(new Vector3(1,0,0),Math.PI/2).toArray();
    rotated.forEach((v,k)=>close(v,poses[1][k],1e-7));
});

test('hard wall remains unilateral and fixed inlet positions cannot move',()=>{
    const s=create({wallSample:({b})=>({gap:.5-b[1],jacobian:[0,0,0,0,-1,0]})});
    const before=s.positions.slice(0,2).map(p=>p.slice());
    s.velocities=s.positions.map((_,i)=>[0,i>1?60:0,0]);
    const r=finish(iterateSharedAxisXpbd(s,1/60,{xpbd:{iterations:128,maxShear:.1,maxLengthError:.1}}));
    assert.equal(r.converged,true,JSON.stringify(r));assert.ok(r.quality.maxPenetration<=1e-5);
    assert.deepEqual(s.positions.slice(0,2),before);assert.ok(s.multipliers.every(x=>x>=0));
    assert.ok(Math.max(...s.multipliers)>0);
});

test('invalid timestep and iteration budget are rejected before changing state',()=>{
    const s=create(),before=captureSharedAxisNative(s);
    for(const [dt,xpbd] of [[0,{}],[NaN,{}],[1/60,{iterations:1.5}]])
        assert.throws(()=>finish(iterateSharedAxisXpbd(s,dt,{xpbd})),/Invalid XPBD/);
    assert.deepEqual(captureSharedAxisNative(s),before);
});

test('hybrid global closure measures its correction without disguising it as force convergence',()=>{
    const s=create({tools:[{id:'catheter',type:'berenstein',insertion:60}]});
    const before=s.positions.slice(0,2).map(p=>p.slice());
    const r=finish(iterateSharedAxisXpbd(s,1/60,{xpbd:{iterations:8,hardClosure:true}}));
    assert.equal(r.converged,true,JSON.stringify(r));assert.ok(r.xpbd.closureCorrectionMm>0);
    assert.ok(r.xpbd.preClosureShear>r.xpbd.maxShear);assert.ok(r.xpbd.maxShear<=1e-5);
    assert.ok(r.xpbd.maxLengthErrorMm<=1e-5);assert.equal(r.xpbd.equilibriumCertified,false);
    assert.deepEqual(s.positions.slice(0,2),before);
});

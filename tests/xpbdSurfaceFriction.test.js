import test from 'node:test';
import assert from 'node:assert/strict';
import {Quaternion,Vector3} from 'three';
import {createSharedAxisNative,extendSharedAxisNativeRows,captureSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {prepareSharedAxisDynamicStep} from '../src/physics/kirchhoffSharedAxisDynamics.js';
import {createXpbdSurfaceFriction} from '../src/physics/xpbdSurfaceFriction.js';
import {iterateSharedAxisTimeStep} from '../src/physics/kirchhoffSharedAxisTimeStep.js';
import {iterateSharedAxisXpbd} from '../src/physics/sharedAxisXpbdExperimental.js';

const finish=it=>{let r;do{r=it.next();}while(!r.done);return r.value;};
const close=(a,b,e=1e-8)=>assert.ok(Math.abs(a-b)<=e,`${a} vs ${b}`);
function fixture(mu=.5) {
    const s=createSharedAxisNative({tools:[{id:'wire',type:'straight',insertion:20,wallStaticFriction:mu,wallKineticFriction:mu*.6}]});
    const e=2;
    extendSharedAxisNativeRows(s,[{kind:'wall',edge:e,id:'feed-wall',witness:{face:2,t:1},
        dofs:[s.layout.positions[e],s.layout.positions[e+1]].flatMap(i=>[i,i+1,i+2]),
        evaluate:({b})=>({gap:b[1],jacobian:[0,0,0,0,1,0]})}]);
    s.loads[s.layout.positions[e+1]+1]=-100;return s;
}

test('coupled surface friction matches reference stick, slide and elastic history without stretching the rod',()=>{
    for(const mu of [.006,.5]) {
        const reference=fixture(mu),candidate=fixture(mu),modes=new Set();
        for(let step=0;step<30;step++) {
            const options={feedById:{wire:step<15?.0002:.1}};
            assert.equal(finish(iterateSharedAxisTimeStep(reference,1/60,options)).converged,true);
            const result=finish(iterateSharedAxisXpbd(candidate,1/60,{...options,xpbd:{iterations:32,globalChain:true,chainSurfaceFriction:true}}));
            assert.equal(result.converged,true,JSON.stringify(result));
            reference.positions.forEach((p,i)=>p.forEach((v,k)=>close(v,candidate.positions[i][k],1e-6)));
            const a=reference.wallFrictionHistory,b=candidate.wallFrictionHistory;assert.equal(b.length,a.length);
            a.forEach((h,i)=>{assert.equal(b[i].id,h.id);assert.equal(b[i].mode,h.mode);h.elastic.forEach((v,k)=>close(v,b[i].elastic[k],1e-7));modes.add(h.mode);});
            assert.ok(result.xpbd.maxLengthErrorMm<1e-8);
        }
        if(mu===.5)assert.deepEqual([...modes].sort(),['slide','stick']);
    }
});

test('surface-slip rows include the finite contact radius and differentiate independent material rotations',()=>{
    const s=fixture(),dt=1/60;prepareSharedAxisDynamicStep(s,dt);
    const frames=s.dynamicStep.frames.map(a=>a.map(f=>({q:new Quaternion(...f.old)})));
    const adapter=createXpbdSurfaceFriction(s,frames,s.dynamicStep,dt,{wire:.001});
    const index=s.definitions.length-1,rows=s.definitions.map(r=>({...r,jacobian:[0,0,0,0,1,0]}));
    const lambda=new Float64Array(rows.length);lambda[index]=100*dt*dt;adapter.refresh(rows,lambda);
    const frameDofs=[Array.from({length:frames[0].length},(_,e)=>s.positions.length*3+3*e)],impulses=new Map();
    const expected=adapter.constraints(frameDofs,impulses),q=frames[0][2].q.clone(),h=1e-6;
    for(const r of expected)for(let j=0;j<r.dofs.length;j++) {
        const dof=r.dofs[j];
        const sample=sign=>{
            if(dof<s.positions.length*3)s.positions[Math.floor(dof/3)][dof%3]+=sign*h;
            else {const axis=new Vector3().setComponent((dof-s.positions.length*3)%3,1);frames[0][2].q.copy(q).premultiply(new Quaternion().setFromAxisAngle(axis,sign*h));}
            const result=adapter.constraints(frameDofs,impulses).find(x=>x.id===r.id).gap;
            if(dof<s.positions.length*3)s.positions[Math.floor(dof/3)][dof%3]-=sign*h;else frames[0][2].q.copy(q);
            return result;
        };
        close((sample(1)-sample(-1))/(2*h),r.jacobian[j],2e-8);
    }
    assert.ok(expected.some(r=>Math.hypot(...r.jacobian.slice(6))>.1),'Rotation must move the surface point');
});

test('cancelling a surface-friction step restores the accepted slip history and pose',()=>{
    const s=fixture();s.wallFrictionHistory=[{id:'wire/feed-wall',owner:'wire',face:2,siteCoordinate:15,elastic:[.001,0,0],mode:'stick'}];
    const pose=captureSharedAxisNative(s),history=s.wallFrictionHistory;
    const it=iterateSharedAxisXpbd(s,1/60,{xpbd:{iterations:32,globalChain:true,chainSurfaceFriction:true},feedById:{wire:.1}});
    assert.equal(it.next().done,false);assert.equal(it.next().done,false);it.return();
    assert.deepEqual(captureSharedAxisNative(s),pose);assert.equal(s.wallFrictionHistory,history);assert.equal(s.dynamicStep,null);
});

test('global XPBD matches the Newton solution on a curved loaded wall',()=>{
    const create=()=>{
        const s=createSharedAxisNative({tools:[{id:'wire',type:'straight',insertion:20,shaftStiffness:2,tipStiffness:2}]}),e=2;
        extendSharedAxisNativeRows(s,[{kind:'wall',edge:e,id:'cylinder',witness:{face:2,t:1},
            dofs:[s.layout.positions[e],s.layout.positions[e+1]].flatMap(i=>[i,i+1,i+2]),
            evaluate:({b})=>{
                const y=b[1]+.15,z=b[2],radius=Math.hypot(y,z),n=[0,y/radius,z/radius],hessian=new Float64Array(36);
                for(let i=1;i<3;i++)for(let j=1;j<3;j++)hessian[(3+i)*6+3+j]=-((i===j?1:0)-n[i]*n[j])/radius;
                return {gap:.2-radius,jacobian:[0,0,0,...n.map(v=>-v)],hessian};
            }}]);
        s.loads[s.layout.positions[e+1]+1]=100;s.loads[s.layout.positions[e+1]+2]=5;return s;
    };
    const a=create(),b=create();let loaded=false;
    for(let step=0;step<20;step++) {
        assert.equal(finish(iterateSharedAxisTimeStep(a,1/60)).converged,true);
        const result=finish(iterateSharedAxisXpbd(b,1/60,{xpbd:{iterations:128,globalChain:true,chainSurfaceFriction:true,chainTrace:true}}));
        assert.equal(result.converged,true,JSON.stringify(result));
        a.positions.forEach((p,i)=>p.forEach((v,k)=>close(v,b.positions[i][k],1e-6)));
        loaded ||= b.multipliers.at(-1)>0;
    }
    assert.ok(loaded,'The test must exercise a loaded curved contact');
});

import test from 'node:test';
import assert from 'node:assert/strict';
import { Vector3, Quaternion } from 'three';
import { EndovascularPhysicsWorld } from '../src/physics/endovascularPhysicsWorld.js';
import { solveKirchhoffCoupledSystem, applyKirchhoffCoupledCorrection } from '../src/physics/kirchhoffCoupledSystem.js';
import { captureKirchhoffCoupledTrialState, restoreKirchhoffCoupledTrialState } from '../src/physics/kirchhoffCoupledTrialState.js';
import { createKirchhoffWallReactionLedger as create, captureKirchhoffWallReaction as capture,
    recordKirchhoffWallReaction as record, buildKirchhoffWallReactionRelease as release,
    commitKirchhoffWallReactionRelease as commit, captureKirchhoffWallSurfaceReaction as captureSurface,
    recordKirchhoffWallSurfaceReaction as recordSurface } from '../src/physics/kirchhoffWallReactionLedger.js';

const close=(a,b)=>assert.ok(Math.abs(a-b)<1e-10,`${a} != ${b}`);
function fixture() {
    const body=new EndovascularPhysicsWorld().createRod('catheter',4,2,{});
    const gradients=[{side:0,dof:6,value:.25},{side:0,dof:7,value:.5},
        {side:0,dof:12,value:.75},{side:0,dof:13,value:1.5}];
    return {body,gradients};
}

test('two wall features retain independent applied reactions and pre-apply moments',()=>{
    const {body,gradients}=fixture(),a=create(body),b=create(body);
    const frozen=capture(body,gradients,0);
    const point=new Vector3(body.x[1],body.y[1],body.z[1]);
    body.y[1]+=3;
    record(a,frozen,.5);
    close(a.lambda,.5);close(b.lambda,0);assert.equal(b.wrenches.length,0);
    const expected=point.cross(new Vector3(.25,.5,0)).multiplyScalar(.5);
    const wrench=a.wrenches.find(w=>w.node===1);
    close(wrench.mx,expected.x);close(wrench.my,expected.y);close(wrench.mz,expected.z);
    frozen[0].fx=100;close(wrench.fx,.125);
});

for(const scale of [1,.5,.125])test(`wall reaction release shares global correction scale ${scale}`,()=>{
    const {body,gradients}=fixture(),ledger=create(body);
    record(ledger,capture(body,gradients,0),.2);
    const row=release(ledger,0), component={bodies:[body],kirchhoffContacts:[]};
    const before=ledger.wrenches.map(v=>({...v}));
    const result=solveKirchhoffCoupledSystem(component,1/120,{additionalRows:[row],activeCondensation:true});
    assert.ok(result.diagnostics.converged,JSON.stringify(result.diagnostics));
    close(result.additionalIncrement[0],-1);
    result.scale=scale;
    applyKirchhoffCoupledCorrection(component,result);
    commit(row,scale*result.additionalIncrement[0]);
    close(ledger.lambda,.2*(1-scale));
    ledger.wrenches.forEach((w,i)=>{for(const k of ['fx','fy','fz','mx','my','mz'])close(w[k],before[i][k]*(1-scale));});
    close(body.wallProjectionX[1],-.2*.25*body.inverseMass[1]*scale);
    close(body.wallProjectionY[2],-.2*1.5*body.inverseMass[2]*scale);
    assert.equal(ledger.retiring,scale!==1);
});

test('release preserves world force and moment after translation and rotation of its material frame',()=>{
    const {body,gradients}=fixture(),ledger=create(body);
    record(ledger,capture(body,gradients,0),.4);
    const expectedF=new Vector3(),expectedM=new Vector3();
    for(const w of ledger.wrenches){expectedF.add(new Vector3(w.fx,w.fy,w.fz));expectedM.add(new Vector3(w.mx,w.my,w.mz));}
    const q=new Quaternion().setFromAxisAngle(new Vector3(1,2,3).normalize(),.7);
    for(let n=0;n<body.count;n++) {body.x[n]+=1;body.y[n]+=2;body.z[n]-=.5;
        if(n<body.segmentCount){body.orientationX[n]=q.x;body.orientationY[n]=q.y;body.orientationZ[n]=q.z;body.orientationW[n]=q.w;}}
    const row=release(ledger,0),forces=Array.from({length:body.count},()=>new Vector3()),torques=forces.map(()=>new Vector3());
    for(const g of row.gradients){const n=Math.floor(g.dof/6),axis=g.dof%6;
        const v=axis<3?forces[n]:torques[n];v.setComponent(axis%3,v.getComponent(axis%3)+g.value);}
    const actualF=new Vector3(),actualM=new Vector3();
    for(let n=0;n<body.count;n++){actualF.add(forces[n]);actualM.add(new Vector3(body.x[n],body.y[n],body.z[n]).cross(forces[n]));
        if(n<body.segmentCount)actualM.add(torques[n].applyQuaternion(q));}
    close(actualF.distanceTo(expectedF),0);close(actualM.distanceTo(expectedM),0);
    body.activeStart=2;
    assert.throws(()=>release(ledger,0),/material support/);
});

test('common nonlinear rollback restores old wall reaction and removes newly discovered witness',()=>{
    const {body,gradients}=fixture(),ledger=create(body);
    record(ledger,capture(body,gradients,0),.2);
    const witnesses=new Map([['face-a',ledger]]);
    const component={bodies:[body],kirchhoffContacts:[],wallWitnesses:witnesses};
    const before=ledger.wrenches.map(v=>({...v}));
    const snapshot=captureKirchhoffCoupledTrialState(component);
    const row=release(ledger,0);
    commit(row,-.5);
    const other=create(body);record(other,capture(body,gradients,0),.3);
    witnesses.set('face-b',other);
    restoreKirchhoffCoupledTrialState(snapshot);
    assert.equal(component.wallWitnesses,witnesses);
    assert.equal(witnesses.get('face-a'),ledger);assert.equal(witnesses.has('face-b'),false);
    close(ledger.lambda,.2);assert.deepEqual(ledger.wrenches,before);
    assert.equal(ledger.retiring,false);close(body.wallProjectionY[2],0);
});

test('signed surface force and radius torque share release without becoming normal projection',()=>{
    const {body}=fixture(),ledger=create(body);
    const q=new Quaternion().setFromAxisAngle(new Vector3(0,1,0),.7);
    body.orientationX[1]=q.x;body.orientationY[1]=q.y;body.orientationZ[1]=q.z;body.orientationW[1]=q.w;
    const normal=[{side:0,dof:8,value:1}];
    const tangent=[{side:0,dof:7,value:1},{side:0,dof:9,value:.5}];
    record(ledger,capture(body,normal,0),2);
    const frozen=captureSurface(body,tangent,0);
    const expectedMoment=new Vector3(.5,0,0).applyQuaternion(q)
        .add(new Vector3(body.x[1],body.y[1],body.z[1]).cross(new Vector3(0,1,0)));
    close(frozen[0].mx,expectedMoment.x);close(frozen[0].my,expectedMoment.y);close(frozen[0].mz,expectedMoment.z);
    recordSurface(ledger,frozen,-.3);ledger.tangentLambda[0]=-.3;
    close(ledger.lambda,2);
    const row=release(ledger,0), before=ledger.wrenches.map(w=>({...w}));
    assert.ok(row.gradients.some(g=>g.dof===7&&g.value===-.3));
    assert.ok(row.gradients.some(g=>g.dof>=9&&g.dof<=11&&g.value!==0));
    commit(row,-.5);
    close(ledger.lambda,1);close(ledger.tangentLambda[0],-.15);
    for(let i=0;i<before.length;i++)for(const k of ['fx','fy','fz','mx','my','mz'])close(ledger.wrenches[i][k],before[i][k]*.5);
    close(body.wallProjectionY[1],0);
    close(body.wallProjectionZ[1],-body.inverseMass[1]);
    assert.throws(()=>capture(body,tangent,0),/translations/,'normal capture remains translation-only');
});

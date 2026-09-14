import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisNative,extendSharedAxisNativeRows,assembleSharedAxisNative,captureSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {prepareSharedAxisDynamicStep} from '../src/physics/kirchhoffSharedAxisDynamics.js';
import {prepareSharedAxisWallFriction,captureSharedAxisWallFriction} from '../src/physics/kirchhoffSharedAxisWallFriction.js';
import {stepSharedAxis,iterateSharedAxisTimeStep} from '../src/physics/kirchhoffSharedAxisTimeStep.js';

function fixture(overlap=false) {
    const tools=[{id:'wire',insertion:20,wallStaticFriction:.2,wallKineticFriction:.1}];
    if(overlap)tools.push({id:'catheter',insertion:20,wallStaticFriction:.2,wallKineticFriction:.1});
    const s=createSharedAxisNative({tools});
    for(let e=1;e+1<s.positions.length;e++) {
        extendSharedAxisNativeRows(s,[{kind:'wall',edge:e,id:`floor/${e}`,witness:{face:0,t:1},
            dofs:[s.layout.positions[e],s.layout.positions[e+1]].flatMap(p=>[p,p+1,p+2]),
            evaluate:({b})=>({gap:b[1],jacobian:[0,0,0,0,1,0]})}]);
        s.multipliers[s.multipliers.length-1]=100;
        s.loads[s.layout.positions[e+1]+1]=-20;
    }
    return s;
}

test('native global reaction column includes wall traction and surface torque without a transpose',()=>{
    const s=fixture(),dt=1/60;prepareSharedAxisDynamicStep(s,dt);
    prepareSharedAxisWallFriction(s,{feedById:{wire:.1},liveNormalLoad:true});
    const base=assembleSharedAxisNative(s),index=s.multipliers.length-1,row=base.rows[index];
    assert.ok(row.extraForceDofs.length>0);
    const expected=new Float64Array(s.layout.dofCount);
    row.dofs.forEach((p,k)=>expected[p]-=row.jacobian[k]);
    row.extraForceDofs.forEach((p,k)=>expected[p]+=row.extraForceJacobian[k]);
    const h=1e-4;s.multipliers[index]+=h;assembleSharedAxisNative(s);const plus=s.chain.gradient.slice();
    s.multipliers[index]-=2*h;assembleSharedAxisNative(s);const minus=s.chain.gradient.slice();
    for(let i=0;i<expected.length;i++)assert.ok(Math.abs((plus[i]-minus[i])/(2*h)-expected[i])<1e-8);
    assert.deepEqual(row.jacobian,[0,0,0,0,1,0]);
});

test('simultaneous normal/friction update reaches the same certified equilibrium for one or both materials',()=>{
    for(const overlap of [false,true]) {
        const states=[];
        for(const liveWallNormalLoad of [false,true]) {
            const s=fixture(overlap),feedById=overlap?{wire:.1,catheter:.1}:{wire:.1};
            const r=stepSharedAxis(s,1/60,{feedById,liveWallNormalLoad});
            assert.ok(r.converged,JSON.stringify(r));assert.ok(r.certificateBound<=1e-6);assert.equal(r.interToolRows,0);
            assert.ok(s.wallFrictionHistory.length>0);states.push({s,r});
        }
        states[0].s.positions.forEach((p,i)=>p.forEach((v,k)=>assert.ok(Math.abs(v-states[1].s.positions[i][k])<1e-6)));
        assert.ok(states[1].r.frictionIterations<=states[0].r.frictionIterations);
    }
});

test('cancelling live normal-load solve restores physical history and leaves no pending force columns',()=>{
    const s=fixture(),pose=captureSharedAxisNative(s),history=captureSharedAxisWallFriction(s);
    const it=iterateSharedAxisTimeStep(s,1/60,{feedById:{wire:.1},liveWallNormalLoad:true});
    it.next();it.next();it.return();
    assert.deepEqual(captureSharedAxisNative(s),pose);assert.deepEqual(captureSharedAxisWallFriction(s),history);
    assert.equal(s.wallFrictionStep,null);assert.equal(s.dynamicStep,null);
});

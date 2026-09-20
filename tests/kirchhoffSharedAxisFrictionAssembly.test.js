import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisNative,extendSharedAxisNativeRows,assembleSharedAxisNative,refreshSharedAxisFrictionMeasure,captureSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {prepareSharedAxisDynamicStep} from '../src/physics/kirchhoffSharedAxisDynamics.js';
import {prepareSharedAxisWallFriction,refreshSharedAxisWallFriction,captureSharedAxisWallFriction} from '../src/physics/kirchhoffSharedAxisWallFriction.js';
import {iterateSharedAxisTimeStep} from '../src/physics/kirchhoffSharedAxisTimeStep.js';

function rod() {
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:20,wallStaticFriction:.5,wallKineticFriction:.3}]});
    const evaluate=()=>({gap:0,jacobian:[0,.6,0,0,.4,0]});
    evaluate.sharedAxisGeometryOnly=true;
    extendSharedAxisNativeRows(s,[{kind:'wall',edge:2,id:'wall',witness:{face:1,t:.4},
        dofs:[s.layout.positions[2],s.layout.positions[3]].flatMap(p=>[p,p+1,p+2]),evaluate}]);
    s.multipliers[s.multipliers.length-1]=100;
    return s;
}

test('friction-only refresh preserves full force assembly exactly after a stick/slide change',()=>{
    for(const lightweightFriction of [false,true]) {
        const s=rod();prepareSharedAxisDynamicStep(s,1/60);
        prepareSharedAxisWallFriction(s,{feedById:{wire:.002},liveNormalLoad:true,lightweightFriction});
        s.multipliers[s.multipliers.length-1]=10;
        const base=assembleSharedAxisNative(s,{withTangent:false,retainFrictionBase:true});
        assert.ok(refreshSharedAxisWallFriction(s).forceChange>0);
        const actual=refreshSharedAxisFrictionMeasure(s,base),gradient=s.chain.gradient.slice();
        assert.ok(actual);assert.equal(s.chain.hessianValid,false);
        const expected=assembleSharedAxisNative(s);
        for(const k of ['energy','force','torque','constraint'])assert.equal(actual[k],expected[k],k);
        assert.deepEqual(gradient,s.chain.gradient);
        assert.equal(s.chain.hessianValid,true);
    }
});

test('changed pose, reactions, load, topology or an undeclared evaluator reject reuse before mutation',()=>{
    for(const change of [s=>{s.geometryKey=Symbol();},s=>{s.multipliers[0]++;},s=>{s.loads[0]++;},
        s=>{s.dynamicStep={};},s=>{s.materials=s.materials.slice();},
        s=>{s.pendingVesselRows=new Map([['new',{}]]);},
        s=>{delete s.definitions.at(-1).evaluate.sharedAxisGeometryOnly;}]) {
        const s=rod();prepareSharedAxisDynamicStep(s,1/60);
        prepareSharedAxisWallFriction(s,{feedById:{wire:.1},liveNormalLoad:true});
        const base=assembleSharedAxisNative(s,{retainFrictionBase:true});change(s);
        const gradient=s.chain.gradient.slice(),valid=s.chain.hessianValid;
        assert.equal(refreshSharedAxisFrictionMeasure(s,base),null);
        assert.deepEqual(s.chain.gradient,gradient);assert.equal(s.chain.hessianValid,valid);
    }
});

test('cancellation after a friction refresh restores all committed state',()=>{
    const s=rod(),before=captureSharedAxisNative(s),friction=captureSharedAxisWallFriction(s),velocities=s.velocities;
    let refreshed=false;
    const it=iterateSharedAxisTimeStep(s,1/60,{feedById:{wire:.1},liveWallNormalLoad:true,
        coupledFrictionNewton:true,promoteTrialAssembly:true,reuseFrictionAssembly:true,
        observeTrial:e=>{if(e.kind==='friction-refresh')refreshed=true;}});
    for(let i=0;i<500&&!refreshed;i++)assert.equal(it.next().done,false);
    assert.ok(refreshed);it.return();
    assert.deepEqual(captureSharedAxisNative(s),before);
    assert.deepEqual(captureSharedAxisWallFriction(s),friction);
    assert.equal(s.velocities,velocities);assert.equal(s.dynamicStep,null);
});

import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisNative,extendSharedAxisNativeRows,applySharedAxisNativeIncrement} from '../src/physics/kirchhoffSharedAxisNative.js';
import {prepareSharedAxisDynamicStep} from '../src/physics/kirchhoffSharedAxisDynamics.js';
import {sharedAxisFrictionPotential,prepareSharedAxisWallFriction,assembleSharedAxisWallFriction,refreshSharedAxisWallFriction,commitSharedAxisWallFriction,captureSharedAxisWallFriction,restoreSharedAxisWallFriction} from '../src/physics/kirchhoffSharedAxisWallFriction.js';

test('force-only friction matches the full law at zero load, stick, slide and return boundaries',()=>{
    const law={stiffness:1000,normalLoad:10,muStatic:.5,muKinetic:.3};
    const out={};
    for(const mode of ['stick','slide'])for(const normalLoad of [0,10,20])for(const muStatic of [0,.5])
    for(const slip of [[0,0,0],[.003,0,0],[.01,-.02,.003],[1e-10,0,0]]) {
        const params={...law,mode,normalLoad,muStatic,muKinetic:muStatic?law.muKinetic:0};
        const full=sharedAxisFrictionPotential(slip,params),light=sharedAxisFrictionPotential(slip,params,false,out);
        assert.equal(light.hessian,null);assert.equal(light.energy,full.energy);
        assert.deepEqual(light.traction,full.traction);assert.equal(light.mode,full.mode);
    }
    assert.throws(()=>sharedAxisFrictionPotential([NaN,0,0],law,false),RangeError);
    assert.throws(()=>sharedAxisFrictionPotential([0,0,0],{...law,normalLoad:-1},false),RangeError);
});

function fixture(lightweightFriction,liveNormalLoad,feed) {
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:35,wallStaticFriction:.5,wallKineticFriction:.3},
        {id:'catheter',insertion:15,wallStaticFriction:.7,wallKineticFriction:.4}]});
    for(const [index,e] of [2,s.coordinates.length-3].entries()) {
        const t=.4,n=index?[.6,0,.8]:[0,1,0];
        extendSharedAxisNativeRows(s,[{kind:'wall',edge:e,id:'site-'+index,witness:{face:index,t},
            dofs:[s.layout.positions[e],s.layout.positions[e+1]].flatMap(i=>[i,i+1,i+2]),
            evaluate:()=>({gap:0,jacobian:[...n.map(v=>(1-t)*v),...n.map(v=>t*v)]})}]);
        s.multipliers[s.multipliers.length-1]=100*(index+1);
    }
    prepareSharedAxisDynamicStep(s,1/120);
    prepareSharedAxisWallFriction(s,{feedById:{wire:feed,catheter:feed/2},liveNormalLoad,lightweightFriction});
    assert.equal(s.wallFrictionStep.records.length,2);
    applySharedAxisNativeIncrement(s,Float64Array.from({length:s.layout.dofCount},(_,i)=>.0003*Math.sin(i)),new Float64Array(s.multipliers.length));
    return s;
}
function sample(s,full,exact) {
    s.chain.gradient.fill(0);s.chain.hessian.fill(0);
    s.chain.tangent=exact?new Float64Array(s.layout.dofCount*(2*s.layout.band-1)).fill(full?0:17):null;
    const energy=assembleSharedAxisWallFriction(s,full);
    return {energy,g:s.chain.gradient.slice(),H:(s.chain.tangent??s.chain.hessian).slice(),columns:structuredClone(s.wallFrictionStep.normalForceColumns)};
}

for(const live of [false,true])test(`buffered friction preserves every force, both tangents, refresh and history (live=${live})`,()=>{
    for(const feed of [.002,.1]) {
        const states=[fixture(false,live,feed),fixture(true,live,feed)];
        for(const full of [true,false,true])for(const exact of [true,false])for(const factor of [1,0,-1,2]) {
            for(const s of states)for(const r of s.wallFrictionStep.records)if(live)s.multipliers[r.rowIndex]=factor*r.normalLoad;
            assert.deepEqual(sample(states[1],full,exact),sample(states[0],full,exact));
        }
        const snapshots=states.map(s=>captureSharedAxisWallFriction(s));assert.deepEqual(snapshots[1],snapshots[0]);
        states.forEach((s,i)=>restoreSharedAxisWallFriction(s,snapshots[i]));
        assert.equal(states[1].wallFrictionStep.lightweightFriction,true);
        assert.deepEqual(refreshSharedAxisWallFriction(states[1]),refreshSharedAxisWallFriction(states[0]));
        assert.deepEqual(refreshSharedAxisWallFriction(states[1]),refreshSharedAxisWallFriction(states[0]));
        states.forEach(s=>commitSharedAxisWallFriction(s));
        assert.deepEqual(states[1].wallFrictionHistory,states[0].wallFrictionHistory);
    }
});

test('published live-load columns survive a later buffered contact and force-only assembly',()=>{
    const s=fixture(true,true,.1);sample(s,true,true);
    const columns=s.wallFrictionStep.normalForceColumns,saved=structuredClone(columns);
    assert.equal(columns.length,2);
    s.positions[2][0]+=.0001;sample(s,false,true);
    assert.deepEqual(columns,saved,'Published columns must own their dofs and force derivatives');
    assert.deepEqual(s.wallFrictionStep.normalForceColumns,[]);
});

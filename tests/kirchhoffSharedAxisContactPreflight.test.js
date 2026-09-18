import test from 'node:test';
import assert from 'node:assert/strict';
import {BufferGeometry,Float32BufferAttribute} from 'three';
import {createSharedAxisNative,assembleSharedAxisNative,extendSharedAxisNativeRows,captureSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {createSharedAxisConstraintRowPool} from '../src/physics/kirchhoffSharedAxisConstraintRows.js';
import {createSharedAxisVesselDiscovery} from '../src/physics/kirchhoffSharedAxisVesselWitnesses.js';
import {prepareSharedAxisDynamicStep} from '../src/physics/kirchhoffSharedAxisDynamics.js';
import {prepareSharedAxisWallFriction,commitSharedAxisWallFriction} from '../src/physics/kirchhoffSharedAxisWallFriction.js';

function fixture(extra=null) {
    const geometry=new BufferGeometry().setAttribute('position',new Float32BufferAttribute([-100,-100,10,100,-100,10,0,100,10],3));
    const calls=[];
    const field={fallbackGeometry:geometry,voxelSize:1,queryCapsuleSoA(x,y,z,r,index,out,face,inside,clearance,branch,near,length,count,physical,finite,visit){
        calls.push(x[0]);visit({faceIndex:0,signedGap:.2,signedDistance:.2+r[0]},.5);
        return Object.assign(out,{faceIndex:0,signedGap:.2,signedDistance:.2+r[0],segmentT:.5});
    }};
    const discovery=createSharedAxisVesselDiscovery(field,0,{queryReuse:false});
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:15}],wallSamples:extra?[...extra.before,discovery,...extra.after]:[discovery]});
    s.cacheMechanicalAssembly=true;s.loads[s.layout.dofCount-1]=.123;
    let materialReads=0;
    const body=s.materials[0].body,rest=body.restRotation1;
    Object.defineProperty(body,'restRotation1',{get(){materialReads++;return rest;}});
    const pool=createSharedAxisConstraintRowPool(),run=earlyContactPreflight=>assembleSharedAxisNative(s,{earlyContactPreflight,rowStorage:pool.acquire(),reuseConstraintWork:true});
    return {s,run,calls,geometry,get materialReads(){return materialReads;}};
}
const measure=(s,r)=>({energy:r.energy,force:r.force,torque:r.torque,constraint:r.constraint,
    g:s.chain.gradient.slice(),H:s.chain.tangent.slice(),rows:r.rows.map(v=>({gap:v.gap,jacobian:v.jacobian.slice(),H:v.geometricHessian?.slice()}))});

test('discovery preflight skips material only on a complete restart and never duplicates geometry queries',()=>{
    const outputs=[];
    for(const early of [false,true]) {
        const f=fixture(),{s,run,calls}=f,before=captureSharedAxisNative(s);
        try {
            assert.throws(()=>run(early),/shared-axis-wall-discovery/);
            assert.deepEqual(calls,[0,5,10]);assert.equal(s.pendingVesselRows.size,3);
            assert.deepEqual(captureSharedAxisNative(s),before);
            assert.equal(f.materialReads===0,early,'Rejected topology must avoid material assembly only in the preflight path');
            const ids=[...s.pendingVesselRows.keys()];
            extendSharedAxisNativeRows(s,[...s.pendingVesselRows.values()]);s.pendingVesselRows.clear();
            const first=measure(s,run(early));assert.equal(calls.length,3,'Prepared discovery output survives the row-only restart');
            s.positions[2][1]+=.01;s.geometryKey=Symbol('trial');
            const second=measure(s,run(early));assert.equal(calls.length,6,'One query per edge, including the successful preflight');
            outputs.push({ids,first,second});
        }finally{f.geometry.dispose();}
    }
    assert.deepEqual(outputs[1],outputs[0]);
});

for(const placement of ['before','after'])test(`geometry failure ${placement} discovery preserves query order, partial pending rows and error priority`,()=>{
    const outputs=[];
    for(const early of [false,true]) {
        const bad=()=>{throw Object.assign(new Error('surface boundary'),{code:'trial-outside-vessel'});};
        bad.sharedAxisGeometryOnly=true;
        const f=fixture({before:placement==='before'?[bad]:[],after:placement==='after'?[bad]:[]});
        try {
            let failure;try{f.run(early);}catch(error){failure={message:error.message,code:error.code};}
            assert.ok(f.materialReads>0,'Geometry errors must retain the original material-before-error behavior');
            outputs.push({failure,calls:f.calls,pending:[...(f.s.pendingVesselRows?.keys()??[])]});
        }finally{f.geometry.dispose();}
    }
    assert.deepEqual(outputs[1],outputs[0]);
    assert.equal(outputs[0].calls.length,placement==='before'?0:1);
});

test('unmarked custom samplers retain the original assembly order',()=>{
    const custom=({state})=>{assert.ok(state.chain.tangent,'Custom callbacks may depend on material assembly');return {gap:1,jacobian:[0,0,0,0,0,0]};};
    const f=fixture({before:[custom],after:[]});
    try{assert.throws(()=>f.run(true),/shared-axis-wall-discovery/);assert.ok(f.materialReads>0);}finally{f.geometry.dispose();}
});

test('prepared geometry preserves defensive copies for a marked sampler with reused output',()=>{
    const outputs=[];
    for(const early of [false,true]) {
        const contact={gap:0,jacobian:new Array(6)},sample=({edge})=>{contact.gap=1+edge;contact.jacobian.fill(edge*.1);return contact;};
        sample.sharedAxisGeometryOnly=true;
        const f=fixture({before:[sample],after:[]});
        try {
            assert.throws(()=>f.run(early),/shared-axis-wall-discovery/);
            extendSharedAxisNativeRows(f.s,[...f.s.pendingVesselRows.values()]);f.s.pendingVesselRows.clear();
            f.s.geometryKey=Symbol('new');outputs.push(measure(f.s,f.run(early)));
        }finally{f.geometry.dispose();}
    }
    assert.deepEqual(outputs[1],outputs[0]);
});

test('an early discovery restart invalidates the previous friction certificate before publication',()=>{
    const f=fixture();
    try {
        prepareSharedAxisDynamicStep(f.s,1/60);
        prepareSharedAxisWallFriction(f.s,{liveNormalLoad:true});
        assert.equal(f.s.wallFrictionStep.certified,true);
        assert.throws(()=>f.run(true),/shared-axis-wall-discovery/);
        assert.equal(f.s.wallFrictionStep.certified,false);
        assert.equal(f.s.wallFrictionStep.certificate,null);
        assert.deepEqual(f.s.wallFrictionStep.normalForceColumns,[]);
        assert.throws(()=>commitSharedAxisWallFriction(f.s),/converged/);
    }finally{f.geometry.dispose();}
});

import test from 'node:test';
import assert from 'node:assert/strict';
import {quaternionExp} from '../src/physics/discreteKirchhoffRod.js';
import {nativeHingeTorqueTangent,assembleSharedAxisMaterialTangent} from '../src/physics/kirchhoffSharedAxisMaterialTangent.js';
import {createSharedAxisNative,applySharedAxisNativeIncrement,captureSharedAxisNative,restoreSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';

test('material scratch preserves exact torques and derivatives across changing poses, rest frames and stiffness',()=>{
    const scratch={},q0=quaternionExp({x:.2,y:-.1,z:.3});
    const rests=[{x:0,y:0,z:0},{x:1e-9,y:-2e-9,z:3e-9},{x:.4,y:-.2,z:.1}];
    for(let i=0;i<60;i++) {
        const q1=quaternionExp({x:2.5*Math.sin(i),y:.2*Math.cos(i),z:.3});
        const rest=rests[i%rests.length],compliance=[.003+i*.001,.4,4];
        for(const withTangent of [false,true])assert.deepEqual(
            nativeHingeTorqueTangent(q0,q1,rest,compliance,withTangent,null,null,scratch),
            nativeHingeTorqueTangent(q0,q1,rest,compliance,withTangent));
    }
});

test('a promoted material snapshot owns its gradient after scratch is overwritten by another trial',()=>{
    const scratch={},preparation={store:true},q0=quaternionExp({x:.1,y:0,z:0});
    const q1=quaternionExp({x:.2,y:.3,z:.4}),rest={x:.03,y:0,z:.02},compliance=[.01,.04,.2];
    const expected=nativeHingeTorqueTangent(q0,q1,rest,compliance);
    nativeHingeTorqueTangent(q0,q1,rest,compliance,false,preparation,null,scratch);
    const saved=structuredClone(preparation.value);
    nativeHingeTorqueTangent(q1,q0,{x:.4,y:0,z:0},compliance,true,null,null,scratch);
    assert.deepEqual(preparation.value,saved);
    assert.deepEqual(nativeHingeTorqueTangent(q0,q1,rest,compliance,true,{reuse:preparation.value},null,scratch),expected);
});

for(const wasmMaterial of [false,true])test(`material assembly scratch stays exact across tools, restored poses and changed profiles (WASM=${wasmMaterial})`,()=>{
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:35},{id:'catheter',insertion:25,type:'pigtail'}],
        samplePosition:x=>[100*Math.sin(x/100),100*(1-Math.cos(x/100)),.01*x]});
    const snapshot=captureSharedAxisNative(s),delta=Float64Array.from({length:s.layout.dofCount},(_,i)=>.003*Math.sin(1.2*i));
    for(let pose=0;pose<4;pose++) {
        restoreSharedAxisNative(s,snapshot);
        if(pose%2)applySharedAxisNativeIncrement(s,delta,new Float64Array(s.multipliers.length));
        if(pose===2)for(const {body} of s.materials){body.restRotation1[1]+=.02;body.kirchhoffBendCompliance1[1]*=2;}
        for(const withTangent of [false,true]) {
            const energy=assembleSharedAxisMaterialTangent(s,withTangent,null,wasmMaterial,false);
            const gradient=s.chain.gradient.slice(),tangent=s.chain.tangent.slice();
            assert.equal(assembleSharedAxisMaterialTangent(s,withTangent,null,wasmMaterial,true),energy);
            assert.deepEqual(s.chain.gradient,gradient);assert.deepEqual(s.chain.tangent,tangent);
        }
    }
});

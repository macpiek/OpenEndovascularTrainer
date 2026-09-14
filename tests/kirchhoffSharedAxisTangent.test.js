import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { nativeHingeTorqueTangent } from '../src/physics/kirchhoffSharedAxisMaterialTangent.js';
import { createSharedAxisNative, assembleSharedAxisNative, applySharedAxisNativeIncrement, captureSharedAxisNative, restoreSharedAxisNative } from '../src/physics/kirchhoffSharedAxisNative.js';

test('native torque tangent matches central differences for curved anisotropic material at small and large rotations',()=>{
    for(const angle of [.001,.3,1.2]) {
        const q0=new THREE.Quaternion().setFromEuler(new THREE.Euler(.2,-.3,.5));
        const q1=q0.clone().multiply(new THREE.Quaternion().setFromEuler(new THREE.Euler(angle,.07,-.02)));
        const rest={x:.03,y:-.02,z:.01},c=[.2,.3,.4],base=nativeHingeTorqueTangent(q0,q1,rest,c),h=1e-6;
        for(let col=0;col<6;col++) {
            const axis=new THREE.Vector3();axis.setComponent(col%3,1);
            const sample=sign=>{
                const pair=[q0.clone(),q1.clone()];pair[col<3?0:1].multiply(new THREE.Quaternion().setFromAxisAngle(axis,sign*h));
                return nativeHingeTorqueTangent(...pair,rest,c).torque;
            };
            const plus=sample(1),minus=sample(-1);
            for(let row=0;row<6;row++)assert.ok(Math.abs((plus[row]-minus[row])/(2*h)-base.jacobian[row*6+col])<1e-7,`${angle}/${row}/${col}`);
        }
    }
});

test('shared spatial material tangent matches the derivative of the native physical residual',()=>{
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:30},{id:'catheter',insertion:25}]});
    const dx=Float64Array.from({length:s.layout.dofCount},(_,i)=>.005*Math.sin(i));
    const zero=new Float64Array(s.multipliers.length);applySharedAxisNativeIncrement(s,dx,zero);
    assembleSharedAxisNative(s);const H=s.chain.tangent.slice(),half=s.layout.band-1,width=2*half+1,snapshot=captureSharedAxisNative(s),h=1e-6;
    for(let col=0;col<dx.length;col++) {
        dx.fill(0);dx[col]=1;
        const sample=sign=>{restoreSharedAxisNative(s,snapshot);applySharedAxisNativeIncrement(s,dx,zero,sign*h);assembleSharedAxisNative(s);return s.chain.gradient.slice();};
        const plus=sample(1),minus=sample(-1);
        for(let row=0;row<dx.length;row++) {
            const expected=Math.abs(col-row)<=half?H[row*width+col-row+half]:0,actual=(plus[row]-minus[row])/(2*h);
            assert.ok(Math.abs(actual-expected)<1e-5*Math.max(1,Math.abs(expected)),`${row}/${col}: ${actual} vs ${expected}`);
        }
    }
});

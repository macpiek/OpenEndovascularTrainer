import test from 'node:test';
import assert from 'node:assert/strict';
import {evaluateSharedAxisBendLimit} from '../src/physics/kirchhoffSharedAxisBendLimit.js';
import {createSharedAxisNative,feedSharedAxisNative,relaxSharedAxisNative,assembleSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {defineKirchhoffMaterialProfile} from '../src/physics/kirchhoffMaterialProfile.js';
test('bend gap, exact gradient and Hessian agree with independent finite differences',()=>{
    const state={positions:[[0,0,0],[4,1,.2],[7,4,.7]],coordinates:[0,5,10]},base=evaluateSharedAxisBendLimit({state,edge:0},Math.PI/4),h=1e-5;
    for(let i=0;i<9;i++) {
        const p=state.positions[Math.floor(i/3)],k=i%3;p[k]+=h;
        const plus=evaluateSharedAxisBendLimit({state,edge:0},Math.PI/4);p[k]-=2*h;
        const minus=evaluateSharedAxisBendLimit({state,edge:0},Math.PI/4);p[k]+=h;
        assert.ok(Math.abs((plus.gap-minus.gap)/(2*h)-base.jacobian[i])<1e-8);
        for(let j=0;j<9;j++)assert.ok(Math.abs((plus.jacobian[j]-minus.jacobian[j])/(2*h)-base.hessian[j*9+i])<1e-8);
    }
});
test('common bend limit blocks intrinsic curvature beyond 45 degrees and survives feed without duplicates',()=>{
    const type=defineKirchhoffMaterialProfile({id:'sharp',sampleEI1:()=>1000,sampleGJ:()=>1000,sampleKappa01:()=>.22});
    let s=createSharedAxisNative({tools:[{id:'catheter',insertion:20,type}],maxBendAngle:Math.PI/4});
    for(const insertion of [20,21,25]) {
        if(insertion!==20)s=feedSharedAxisNative(s,{catheter:insertion});
        const result=relaxSharedAxisNative(s);assert.ok(result.converged,JSON.stringify(result));
        const rows=assembleSharedAxisNative(s).rows.filter(r=>r.subtype==='bend-limit');
        assert.equal(rows.length,s.positions.length-2);
        assert.ok(rows.some(r=>r.multiplier>0));assert.ok(rows.every(r=>r.gap>=-1e-5));
    }
});

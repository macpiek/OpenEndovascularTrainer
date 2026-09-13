import assert from 'node:assert/strict';
import test from 'node:test';
import {createSharedAxisNative,sharedAxisOuterMaterialAt,captureSharedAxisNative,restoreSharedAxisNative,
    applySharedAxisNativeIncrement,extendSharedAxisNativeRows} from '../src/physics/kirchhoffSharedAxisNative.js';
import {assembleSharedAxisConstraintRows,sharedAxisConstraintEdgeGeometry,sharedAxisPositionDofMask}
    from '../src/physics/kirchhoffSharedAxisConstraintRows.js';
import {assembleSharedAxisConstraintRowsReference} from './helpers/sharedAxisConstraintRowsReference.js';

function fixture() {
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:45},{id:'catheter',insertion:30}],maxBendAngle:Math.PI/4,
        samplePosition:x=>[x,2*Math.sin(x/30),.2*Math.cos(x/17)],wallSample:({a,b,needHessian})=>({
            gap:.3-.2*a[1]-.8*b[1],jacobian:[0,-.2,0,0,-.8,0],
            hessian:needHessian?Float64Array.from({length:36},(_,i)=>Math.sin(i*.3)*.01):undefined})});
    s.multipliers.forEach((_,i)=>s.multipliers[i]=i%4===0?0:Math.sin(i*.7)*3);
    return s;
}
function compare(s,withTangent=true) {
    const initial=Float64Array.from(s.chain.gradient,(_,i)=>Math.cos(i*.2));s.chain.gradient.set(initial);
    const expected=assembleSharedAxisConstraintRowsReference(s,{withTangent}),gradient=s.chain.gradient.slice();
    s.chain.gradient.set(initial);
    const actual=assembleSharedAxisConstraintRows(s,{withTangent,outerMaterialAt:sharedAxisOuterMaterialAt});
    assert.deepEqual(actual,expected);assert.deepEqual(s.chain.gradient,gradient);return actual;
}

test('optimized rows retain bit-identical gaps, reactions and exact geometric Hessians',()=>{
    const s=fixture();compare(s);compare(s,false);s.cacheMechanicalAssembly=true;compare(s);compare(s);
});

test('pose cache handles trial rollback and multiplier-only changes without changing retained rows',()=>{
    const s=fixture(),saved=captureSharedAxisNative(s),rows=compare(s),oldRows=rows.map(r=>({...r,
        dofs:r.dofs.slice(),jacobian:r.jacobian.slice(),geometricHessian:r.geometricHessian?.slice()}));
    const geometry=sharedAxisConstraintEdgeGeometry(s);
    s.multipliers.forEach((v,i)=>s.multipliers[i]=v+2);compare(s);
    assert.equal(sharedAxisConstraintEdgeGeometry(s),geometry);assert.deepEqual(rows,oldRows);
    applySharedAxisNativeIncrement(s,Float64Array.from(s.chain.gradient,(_,i)=>.001*Math.sin(i)),new Float64Array(s.multipliers.length));
    assert.notEqual(sharedAxisConstraintEdgeGeometry(s),geometry);compare(s);
    restoreSharedAxisNative(s,saved);compare(s);
    assert.deepEqual(sharedAxisConstraintEdgeGeometry(s).lengths,geometry.lengths);
});

test('new contact rows reuse pose geometry but honor new reactions and Hessian demand',()=>{
    const s=fixture();compare(s);const geometry=sharedAxisConstraintEdgeGeometry(s),edge=2;
    let calls=0;
    extendSharedAxisNativeRows(s,[{kind:'wall',edge,id:'new',dofs:[s.layout.positions[edge],s.layout.positions[edge+1]].flatMap(p=>[p,p+1,p+2]),
        evaluate:({needHessian})=>{calls++;return {gap:1,jacobian:[0,0,-.3,0,0,-.7],hessian:needHessian?new Float64Array(36).fill(.02):undefined};}}]);
    s.cacheMechanicalAssembly=true;compare(s,false);s.multipliers[s.multipliers.length-1]=3;
    const rows=compare(s);assert.equal(calls,2);assert.equal(sharedAxisConstraintEdgeGeometry(s),geometry);
    assert.equal(rows.at(-1).geometricHessian[0],-.06);
});

test('position mask is reusable and classifies every material spin separately',()=>{
    const s=fixture(),mask=sharedAxisPositionDofMask(s.layout),old=new Set(Array.from(s.layout.positions).flatMap(p=>[p,p+1,p+2]));
    assert.equal(mask,sharedAxisPositionDofMask(s.layout));
    for(let i=0;i<s.layout.dofCount;i++)assert.equal(!!mask[i],old.has(i));
});

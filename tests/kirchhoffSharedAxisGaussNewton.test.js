import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisNative,applySharedAxisNativeIncrement,captureSharedAxisNative,restoreSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {assembleSharedAxisGaussNewton} from '../src/physics/kirchhoffSharedAxisGaussNewton.js';
import {referenceSharedAxisGaussNewton} from './helpers/sharedAxisGaussNewtonReference.js';

const close=(a,b,tolerance=2e-10)=>assert.ok(Math.abs(a-b)<=tolerance*Math.max(1,Math.abs(a),Math.abs(b)),`${a} versus ${b}`);
function fixture(tools) {
    const s=createSharedAxisNative({tools,samplePosition:x=>[100*Math.sin(x/100),100*(1-Math.cos(x/100)),.01*x]});
    const dx=Float64Array.from({length:s.layout.dofCount},(_,i)=>.012*Math.sin(i*1.23));
    applySharedAxisNativeIncrement(s,dx,new Float64Array(s.multipliers.length));
    for(let i=0;i<s.loads.length;i++)s.loads[i]=.03*Math.cos(i);
    return s;
}
const cases=[
    [{id:'wire',insertion:45}],
    [{id:'catheter',insertion:45,type:'pigtail'}],
    [{id:'wire',insertion:55,shaftStiffness:39,tipStiffness:30.7},{id:'catheter',insertion:35,type:'berenstein',shaftStiffness:58.1,tipStiffness:87}],
    [{id:'wire',insertion:45},{id:'catheter',insertion:44.9999,type:'pigtail'}],
    [{id:'wire',insertion:45},{id:'catheter',insertion:45.0001,type:'pigtail'}]
];
for(const [index,tools] of cases.entries())test(`direct Gauss-Newton pullback matches existing native row assembly (coverage ${index})`,()=>{
    const s=fixture(tools),saved=captureSharedAxisNative(s),referenceEnergy=referenceSharedAxisGaussNewton(s),referenceGradient=s.chain.gradient.slice(),referenceH=s.chain.hessian.slice();
    const energy=assembleSharedAxisGaussNewton(s);
    close(energy,referenceEnergy);s.chain.gradient.forEach((v,i)=>close(v,referenceGradient[i]));s.chain.hessian.forEach((v,i)=>close(v,referenceH[i]));
    assert.equal(s.chain.tangent,null);assert.deepEqual(captureSharedAxisNative(s),saved);
    const gradient=s.chain.gradient.slice();assert.equal(assembleSharedAxisGaussNewton(s,false),energy);assert.deepEqual(s.chain.gradient,gradient);assert.ok(s.chain.hessian.every(v=>v===0));
});

test('direct native material gradient agrees with energy finite differences including fractional tips',()=>{
    const s=fixture(cases[3]),snapshot=captureSharedAxisNative(s),zero=new Float64Array(s.multipliers.length),dx=new Float64Array(s.layout.dofCount),h=1e-6;
    assembleSharedAxisGaussNewton(s);const gradient=s.chain.gradient.slice();
    for(let col=0;col<dx.length;col++) {
        dx.fill(0);dx[col]=1;
        const sample=sign=>{restoreSharedAxisNative(s,snapshot);applySharedAxisNativeIncrement(s,dx,zero,sign*h);return assembleSharedAxisGaussNewton(s);};
        close((sample(1)-sample(-1))/(2*h),gradient[col]+s.loads[col],2e-5);
    }
});

for(const edges of [80,100])test(`long shared overlap retains every native GN matrix coefficient (${edges} edges)`,()=>{
    const s=fixture([{id:'wire',insertion:edges*5,shaftStiffness:39,tipStiffness:30.7},
        {id:'catheter',insertion:edges*5-.0001,type:'pigtail',shaftStiffness:58.1,tipStiffness:87}]);
    const expectedEnergy=referenceSharedAxisGaussNewton(s),g=s.chain.gradient.slice(),H=s.chain.hessian.slice();
    close(assembleSharedAxisGaussNewton(s),expectedEnergy);
    s.chain.gradient.forEach((v,i)=>close(v,g[i]));s.chain.hessian.forEach((v,i)=>close(v,H[i]));
});

import test from 'node:test';
import assert from 'node:assert/strict';
import {Quaternion,Vector3} from 'three';
import {nativeHingeTorqueTangent,assembleSharedAxisMaterialTangent} from '../src/physics/kirchhoffSharedAxisMaterialTangent.js';
import {createSharedAxisNative,applySharedAxisNativeIncrement,captureSharedAxisNative,restoreSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';

const close=(a,b,tol=2e-6)=>assert.ok(Math.abs(a-b)<=tol*Math.max(1,Math.abs(a),Math.abs(b)),`${a} vs ${b}`);
const random=seed=>()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/2**32;};

test('reused native derivative scratch retains anisotropic torque and energy derivatives across random large rotations',()=>{
    const rng=random(0x7a96e17),axis=()=>new Vector3(rng()-.5,rng()-.5,rng()-.5).normalize();
    for(let sample=0;sample<16;sample++) {
        const q0=new Quaternion().setFromAxisAngle(axis(),2*Math.PI*rng());
        const q1=q0.clone().multiply(new Quaternion().setFromAxisAngle(axis(),.05+2.5*rng()));
        const rest={x:.1*(rng()-.5),y:.1*(rng()-.5),z:.1*(rng()-.5)},compliance=[.003,.4,4];
        const base=nativeHingeTorqueTangent(q0,q1,rest,compliance),h=1e-6;
        for(let col=0;col<6;col++) {
            const direction=new Vector3();direction.setComponent(col%3,1);
            const shifted=sign=>{
                const frames=[q0.clone(),q1.clone()];frames[col<3?0:1].multiply(new Quaternion().setFromAxisAngle(direction,sign*h));
                return nativeHingeTorqueTangent(frames[0],frames[1],rest,compliance);
            };
            const plus=shifted(1),minus=shifted(-1);
            close((plus.energy-minus.energy)/(2*h),base.torque[col]);
            for(let row=0;row<6;row++)close((plus.torque[row]-minus.torque[row])/(2*h),base.jacobian[row*6+col]);
        }
    }
});

function fixture() {
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:35},{id:'catheter',insertion:25,type:'pigtail'}],
        samplePosition:x=>[100*Math.sin(x/100),100*(1-Math.cos(x/100)),.01*x]});
    const dx=Float64Array.from({length:s.layout.dofCount},(_,i)=>.003*Math.sin(1.2*i));
    applySharedAxisNativeIncrement(s,dx,new Float64Array(s.multipliers.length));return s;
}

test('reused pullback scratch preserves complete physical energy gradient and both Newton tangent triangles',()=>{
    const s=fixture(),baseEnergy=assembleSharedAxisMaterialTangent(s),gradient=s.chain.gradient.slice(),H=s.chain.tangent.slice();
    const snapshot=captureSharedAxisNative(s),n=s.layout.dofCount,half=s.layout.band-1,width=2*half+1,h=1e-6;
    const direction=new Float64Array(n),zero=new Float64Array(s.multipliers.length);let maximumAsymmetry=0;
    assert.ok(Number.isFinite(baseEnergy));
    for(let col=0;col<n;col++) {
        direction.fill(0);direction[col]=1;
        const sample=sign=>{
            restoreSharedAxisNative(s,snapshot);applySharedAxisNativeIncrement(s,direction,zero,sign*h);
            const energy=assembleSharedAxisMaterialTangent(s);return {energy,gradient:s.chain.gradient.slice()};
        };
        const plus=sample(1),minus=sample(-1);close((plus.energy-minus.energy)/(2*h),gradient[col],1e-5);
        for(let row=0;row<n;row++) {
            const expected=Math.abs(col-row)<=half?H[row*width+col-row+half]:0;
            close((plus.gradient[row]-minus.gradient[row])/(2*h),expected,2e-5);
            if(Math.abs(col-row)<=half)maximumAsymmetry=Math.max(maximumAsymmetry,Math.abs(expected-H[col*width+row-col+half]));
        }
    }
    assert.ok(maximumAsymmetry>1e-5,'Off-equilibrium moving-frame tangent must not be silently symmetrized');
});

test('gradient-only material assembly resets its output and matches full assembly without changing the physical state',()=>{
    const s=fixture();s.loads.forEach((_,i)=>{s.loads[i]=.01*Math.cos(i);});
    const snapshot=captureSharedAxisNative(s),fullEnergy=assembleSharedAxisMaterialTangent(s),gradient=s.chain.gradient.slice();
    const fastEnergy=assembleSharedAxisMaterialTangent(s,false);
    assert.equal(fastEnergy,fullEnergy);assert.deepEqual(s.chain.gradient,gradient);
    assert.ok(s.chain.tangent.every(v=>v===0));assert.deepEqual(captureSharedAxisNative(s),snapshot);
    assembleSharedAxisMaterialTangent(s);assert.deepEqual(s.chain.gradient,gradient);
});

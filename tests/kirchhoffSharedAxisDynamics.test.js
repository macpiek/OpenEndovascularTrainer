import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisNative,relaxSharedAxisNative,assembleSharedAxisNative,captureSharedAxisNative,restoreSharedAxisNative,applySharedAxisNativeIncrement} from '../src/physics/kirchhoffSharedAxisNative.js';
import {prepareSharedAxisDynamicStep,completeSharedAxisDynamicStep} from '../src/physics/kirchhoffSharedAxisDynamics.js';

test('shared inertia counts both materials once and transports a free translation with native damping',()=>{
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:20},{id:'catheter',insertion:10,type:'straight'}]});
    // Use the native straight wire for both mass channels in this translation oracle.
    s.fixed.fill(0);s.velocities=s.positions.map(()=>[1,2,3]);
    const before=s.positions.map(p=>p.slice()),dt=1/120;
    prepareSharedAxisDynamicStep(s,dt);
    assert.ok(Math.abs(s.dynamicStep.masses.reduce((a,b)=>a+b,0)-(20/5+1.4*10/5))<1e-12);
    const r=relaxSharedAxisNative(s);assert.ok(r.converged,JSON.stringify(r));
    // Center-of-mass translation is independent of the internal native shape forces.
    for(let k=0;k<3;k++) {
        const displacement=s.positions.reduce((v,p,i)=>v+s.dynamicStep.masses[i]*(p[k]-before[i][k]),0)/6.8;
        assert.ok(Math.abs(displacement-(k+1)*dt*.98)<1e-7);
    }
    completeSharedAxisDynamicStep(s);assert.equal(s.dynamicStep,null);
});

test('kinetic potential gradient and tangent agree with finite differences of the full shared residual',()=>{
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:20},{id:'catheter',insertion:15}]});
    prepareSharedAxisDynamicStep(s,1/120);
    const dx=Float64Array.from({length:s.layout.dofCount},(_,i)=>.002*Math.sin(i)),zero=new Float64Array(s.multipliers.length);
    applySharedAxisNativeIncrement(s,dx,zero);assembleSharedAxisNative(s);
    const H=s.chain.tangent.slice(),gradient=s.chain.gradient.slice(),half=s.layout.band-1,width=2*half+1,saved=captureSharedAxisNative(s),h=1e-6;
    for(let col=0;col<dx.length;col++) {
        dx.fill(0);dx[col]=1;
        const sample=sign=>{restoreSharedAxisNative(s,saved);applySharedAxisNativeIncrement(s,dx,zero,sign*h);const r=assembleSharedAxisNative(s);return {g:s.chain.gradient.slice(),energy:r.energy};};
        const a=sample(1),b=sample(-1);
        assert.ok(Math.abs((a.energy-b.energy)/(2*h)-gradient[col])<1e-4*Math.max(1,Math.abs(gradient[col])));
        for(let row=0;row<dx.length;row++) {
            const expected=Math.abs(col-row)<=half?H[row*width+col-row+half]:0,actual=(a.g[row]-b.g[row])/(2*h);
            assert.ok(Math.abs(actual-expected)<2e-5*Math.max(1,Math.abs(expected)),`${row}/${col}: ${actual} vs ${expected}`);
        }
    }
});

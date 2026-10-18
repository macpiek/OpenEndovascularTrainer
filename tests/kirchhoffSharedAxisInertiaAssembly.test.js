import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisNative,applySharedAxisNativeIncrement,captureSharedAxisNative,restoreSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {prepareSharedAxisDynamicStep,assembleSharedAxisInertia} from '../src/physics/kirchhoffSharedAxisDynamics.js';
import {assembleSharedAxisInertia as reference} from './helpers/sharedAxisDynamicsReference.js';
import {sharedAxisMaterialKernelWorkspace,sharedAxisMaterialInertiaWorkspace} from '../src/physics/kirchhoffSharedAxisMaterialKernel.js';

const close=(a,b,tolerance=2e-12)=>assert.ok(Math.abs(a-b)<=tolerance*Math.max(1,Math.abs(a),Math.abs(b)),`${a} versus ${b}`);
function fixture(edges=8,fractional=0) {
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:edges*5,mass:1},{id:'catheter',insertion:edges*5+fractional,mass:1.75,type:'pigtail'}],
        samplePosition:x=>[100*Math.sin(x/100),100*(1-Math.cos(x/100)),.01*x]});
    s.velocities=s.positions.map((_,i)=>[Math.sin(i),Math.cos(i),.1*i]);
    s.angularVelocities=Object.fromEntries(s.materials.map(m=>[m.spec.id,Array.from({length:m.last},(_,i)=>[8*Math.sin(i),-3*Math.cos(i),4*Math.sin(i*.7)])]));
    prepareSharedAxisDynamicStep(s,1/60);
    applySharedAxisNativeIncrement(s,Float64Array.from({length:s.layout.dofCount},(_,i)=>.03*Math.sin(i*1.23)),new Float64Array(s.multipliers.length));
    return s;
}
function reset(s,full=true,seed=false) {
    const n=s.layout.dofCount;
    s.chain.gradient.set(Float64Array.from({length:n},(_,i)=>seed ? .1*Math.sin(i) : 0));
    s.chain.hessian.fill(seed ? .3 : 0);
    s.chain.tangent=full?new Float64Array(n*(2*s.layout.band-1)).fill(seed ? .2 : 0):null;
}
for(const fractional of [0,-.0001,.0001])for(const mode of ['newton','gauss-newton','gradient-only'])test(`scratch inertia matches every reference coefficient (${mode}, fractional ${fractional})`,()=>{
    const s=fixture(8,fractional),saved=captureSharedAxisNative(s),dynamic=structuredClone(s.dynamicStep),full=mode!=='gauss-newton',tangent=mode!=='gradient-only';
    reset(s,full,true);const expected=reference(s,tangent),gradient=s.chain.gradient.slice(),H=(s.chain.tangent??s.chain.hessian).slice();
    reset(s,full,true);close(assembleSharedAxisInertia(s,tangent),expected);
    s.chain.gradient.forEach((v,i)=>close(v,gradient[i]));(s.chain.tangent??s.chain.hessian).forEach((v,i)=>close(v,H[i]));
    assert.deepEqual(captureSharedAxisNative(s),saved);assert.deepEqual(s.dynamicStep,dynamic);
});

test('scratch kinetic energy gradient and both moving-frame tangent triangles match finite differences',()=>{
    const s=fixture(4,-.0001),saved=captureSharedAxisNative(s),n=s.layout.dofCount,half=s.layout.band-1,width=2*half+1,h=1e-6;
    reset(s);assembleSharedAxisInertia(s);const g=s.chain.gradient.slice(),H=s.chain.tangent.slice(),dx=new Float64Array(n),zero=new Float64Array(s.multipliers.length);
    let asymmetry=0;
    for(let col=0;col<n;col++) {
        dx.fill(0);dx[col]=1;
        const sample=sign=>{restoreSharedAxisNative(s,saved);applySharedAxisNativeIncrement(s,dx,zero,sign*h);reset(s);return {energy:assembleSharedAxisInertia(s),gradient:s.chain.gradient.slice()};};
        const plus=sample(1),minus=sample(-1);close((plus.energy-minus.energy)/(2*h),g[col],2e-5);
        for(let row=0;row<n;row++) {
            const expected=Math.abs(col-row)<=half?H[row*width+col-row+half]:0;
            close((plus.gradient[row]-minus.gradient[row])/(2*h),expected,2e-5);
            if(Math.abs(col-row)<=half)asymmetry=Math.max(asymmetry,Math.abs(expected-H[col*width+row-col+half]));
        }
    }
    assert.ok(asymmetry>1,'Moving material frames produce a nonsymmetric residual derivative away from equilibrium');
});

for(const edges of [80,100])test(`long overlap retains native inertia energy and full tangent (${edges} edges)`,()=>{
    const s=fixture(edges,-.0001);reset(s);const energy=reference(s),g=s.chain.gradient.slice(),H=s.chain.tangent.slice();
    reset(s);close(assembleSharedAxisInertia(s),energy);g.forEach((v,i)=>close(v,s.chain.gradient[i]));H.forEach((v,i)=>close(v,s.chain.tangent[i]));
});

test('batched inertia retains every Float64 coefficient, signed zero and the preexisting material tangent',()=>{
    for(const edges of [2,8,100])for(const fractional of [0,-.0001,.0001]) {
        const s=fixture(edges,fractional),before=captureSharedAxisNative(s),dynamic=structuredClone(s.dynamicStep);
        reset(s,true,true);
        const energy=assembleSharedAxisInertia(s),gradient=s.chain.gradient.slice(),H=s.chain.tangent.slice();
        const w=sharedAxisMaterialKernelWorkspace(s.chain,s.materials.reduce((sum,m)=>sum+Math.max(0,m.last-1),0));
        for(let repeat=0;repeat<3;repeat++) {
            reset(s,true,true);w.tangent.set(s.chain.tangent);s.chain.tangent=w.tangent;
            assert.equal(sharedAxisMaterialInertiaWorkspace(s.chain),w);
            assert.equal(assembleSharedAxisInertia(s,true,true),energy);
            assert.deepEqual(s.chain.gradient,gradient);assert.deepEqual(s.chain.tangent,H);
            assert.equal(s.chain.tangent,w.tangent,'No global matrix copy or replacement');
        }
        assert.deepEqual(captureSharedAxisNative(s),before);assert.deepEqual(s.dynamicStep,dynamic);
        reset(s);assert.equal(sharedAxisMaterialInertiaWorkspace(s.chain),null,'A caller-owned tangent never aliases the kernel');
        assert.throws(()=>w.assembleInertia(w.records.length+1),/capacity/);
    }
});

test('inertia batches keep separate chain memories and preserve JS-only gradient and Gauss-Newton paths',()=>{
    const a=fixture(5),b=fixture(9);
    const run=s=>{
        reset(s,true,true);
        const w=sharedAxisMaterialKernelWorkspace(s.chain,s.materials.reduce((sum,m)=>sum+Math.max(0,m.last-1),0));
        w.tangent.set(s.chain.tangent);s.chain.tangent=w.tangent;
        assembleSharedAxisInertia(s,true,true);return s.chain.tangent.slice();
    };
    const saved=run(a);run(b);assert.deepEqual(a.chain.tangent,saved);
    assert.notEqual(a.chain.tangent.buffer,b.chain.tangent.buffer);
    for(const [full,tangent] of [[false,true],[true,false]]) {
        reset(a,full,true);const expected=assembleSharedAxisInertia(a,tangent),g=a.chain.gradient.slice(),H=(a.chain.tangent??a.chain.hessian).slice();
        reset(a,full,true);assert.equal(assembleSharedAxisInertia(a,tangent,true),expected);
        assert.deepEqual(a.chain.gradient,g);assert.deepEqual(a.chain.tangent??a.chain.hessian,H);
    }
});

test('inertia capacity is refreshed when the material layout changes without changing matrix dimensions',()=>{
    const s=fixture(5),chain=s.chain,hinges=s.materials.reduce((sum,m)=>sum+Math.max(0,m.last-1),0);
    const original=sharedAxisMaterialKernelWorkspace(chain,hinges);
    chain.tangent=original.tangent;
    const spins=chain.layout.spins;
    chain.layout.spins=new Map([...spins,['new-material',[]]]);
    assert.equal(sharedAxisMaterialInertiaWorkspace(chain),null,'The old matrix has insufficient record capacity');
    const resized=sharedAxisMaterialKernelWorkspace(chain,hinges);
    assert.notEqual(resized,original);
    assert.equal(resized.records.length,original.records.length+1);
    assert.equal(original.tangent.length,resized.tangent.length);
    chain.tangent=resized.tangent;
    assert.equal(sharedAxisMaterialInertiaWorkspace(chain),resized);
    assert.equal(sharedAxisMaterialKernelWorkspace(chain,hinges),resized);
    chain.layout.spins=spins;
    assert.equal(sharedAxisMaterialInertiaWorkspace(chain),null,'Shrinking the layout also invalidates the workspace');
});

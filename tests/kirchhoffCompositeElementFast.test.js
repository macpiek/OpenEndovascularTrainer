import assert from 'node:assert/strict';
import test from 'node:test';
import * as oracle from '../src/physics/kirchhoffCompositeElement.js';
import * as fast from '../src/physics/kirchhoffCompositeElementFast.js';
import * as chain from '../src/physics/kirchhoffCompositeChain.js';
import {loadCompositeFastChainOverlay} from './helpers/compositeFastChainOverlay.js';

const close=(a,b,t=2e-12)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t*(1+Math.max(Math.abs(a),Math.abs(b))),`${a} != ${b}`);
const outputs=['strain','jacobian','moments','gradient','hessian','toolEnergy','referenceTwists'];
function compare(a,b,t=2e-12) {
    close(a.energy,b.energy,t);
    for(const key of outputs) {assert.equal(a[key].length,b[key].length);a[key].forEach((v,i)=>close(v,b[key][i],t));}
}
function fixture(seed=1,count=2) {
    const random=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296;};
    const rest=[[0,0,0],[2,.2,-.1],[3.8,.6,.3]];
    return {positions:rest.map(p=>p.map(v=>v+(random()-.5)*.2)),reference:oracle.captureCompositeReferenceFrames(rest),
        referenceLength:1.7+random(),referenceTwist:2*Math.PI,
        tools:Array.from({length:count},(_,i)=>({dsDx:.6+random(),angles:[random()-.5,random()-.5],referenceTwist:i?undefined:-4*Math.PI,
            material:oracle.compileCompositeMaterial({stiffness:[[3,.3,.2],[.3,5,-.1],[.2,-.1,2]],
                intrinsic:[.05*random(),-.1*random(),.3*random()],energyOffset:.1+random()})}))};
}
function move(args,dof,amount) {
    if(dof<9)args.positions[Math.floor(dof/3)][dof%3]+=amount;
    else args.tools[Math.floor((dof-9)/2)].angles[(dof-9)%2]+=amount;
}

test('fast stencil preserves every response field on 3D anisotropic one/two-tool elements with dsDx and winding anchors',()=>{
    for(let count=1;count<=2;count++) {
        const a=oracle.createCompositeElementWorkspace(count),b=fast.createCompositeElementWorkspace(count);
        for(let seed=1;seed<=24;seed++) {
            const args=fixture(seed,count),before=structuredClone(args);
            compare(oracle.evaluateCompositeElement(args,a),fast.evaluateCompositeElement(args,b));
            assert.deepEqual(args,before);
        }
    }
});

test('fast exact gradient and every strain Jacobian column match independent finite differences',()=>{
    for(const count of [1,2]) {
        const args=fixture(421,count),w=fast.createCompositeElementWorkspace(count);
        fast.evaluateCompositeElement(args,w);const gradient=w.gradient.slice(),jacobian=w.jacobian.slice(),n=w.dofCount;
        for(let i=0;i<n;i++) {
            const h=1e-6;
            move(args,i,h);fast.evaluateCompositeElement(args,w);const plus=w.energy,ps=w.strain.slice();
            move(args,i,-2*h);fast.evaluateCompositeElement(args,w);const minus=w.energy,ms=w.strain.slice();
            move(args,i,h);close(gradient[i],(plus-minus)/(2*h),2e-7);
            for(let row=0;row<3*count;row++)close(jacobian[row*n+i],(ps[row]-ms[row])/(2*h),2e-8);
        }
        // Frozen reference frames induce a position derivative of torsion;
        // omitting this derivative would not pass this witness.
        assert.ok(Math.max(...jacobian.slice(2*n,2*n+9).map(Math.abs))>1e-3);
    }
});

test('fast GN is reconstructed independently from complete J and anisotropic K, including bend/twist cross terms',()=>{
    const args=fixture(27),w=fast.evaluateCompositeElement(args),n=w.dofCount;
    for(let i=0;i<n;i++)for(let j=0;j<n;j++) {
        let value=0;
        for(let t=0;t<2;t++)for(let a=0;a<3;a++)for(let b=0;b<3;b++)value+=args.referenceLength*args.tools[t].dsDx*
            w.jacobian[(3*t+a)*n+i]*args.tools[t].material.stiffness[3*a+b]*w.jacobian[(3*t+b)*n+j];
        close(w.hessian[i*n+j],value);close(w.hessian[i*n+j],w.hessian[j*n+i]);
    }
});

test('winding uses Math.round semantics, energy offsets preserve reactions, and material spins stay independent',()=>{
    const p=[[0,0,0],[2,0,0],[4,0,0]],material=oracle.compileCompositeMaterial({EI1:2,GJ:3});
    const args={positions:p,reference:oracle.captureCompositeReferenceFrames(p),referenceLength:2,
        tools:[{angles:[0,.2],material},{angles:[-.1,-.3],material}]};
    for(const anchor of [-3*Math.PI,-Math.PI,Math.PI,3*Math.PI]) {
        args.referenceTwist=anchor;
        compare(oracle.evaluateCompositeElement(args),fast.evaluateCompositeElement(args));
    }
    args.referenceTwist=0;
    const baseline=fast.evaluateCompositeElement(args),energy=baseline.energy,gradient=baseline.gradient.slice(),hessian=baseline.hessian.slice();
    args.tools[0].material=oracle.compileCompositeMaterial({EI1:2,GJ:3,energyOffset:.37});
    const changed=fast.evaluateCompositeElement(args);
    close(changed.energy-energy,2*.37);
    changed.gradient.forEach((v,i)=>close(v,gradient[i]));changed.hessian.forEach((v,i)=>close(v,hessian[i]));
    assert.equal(changed.hessian[9*13+11],0);
});

test('fast output buffers and memory are stable across calls; invalid geometry or nonfinite tensors cannot pass',()=>{
    const args=fixture(38),w=fast.createCompositeElementWorkspace(2),arrays=outputs.map(k=>w[k]),memory=w.kernel.memory.buffer;
    for(let i=0;i<30;i++)fast.evaluateCompositeElement(args,w);
    outputs.forEach((k,i)=>assert.equal(w[k],arrays[i]));assert.equal(w.kernel.memory.buffer,memory);
    assert.notEqual(fast.createCompositeElementWorkspace(2).kernel.memory.buffer,memory);
    for(const module of [oracle,fast]) {
        assert.throws(()=>module.evaluateCompositeElement({...args,referenceLength:0}),/positive/);
        assert.throws(()=>module.evaluateCompositeElement({...args,positions:[[0,0,0],[0,0,0],[1,0,0]]}),/collapse/);
        assert.throws(()=>module.evaluateCompositeElement({...args,referenceTwist:Infinity,tools:[{...args.tools[0],referenceTwist:undefined}]}),/anchor/);
        const broken=fixture(38);broken.tools[0].material.stiffness[0]=Infinity;
        assert.throws(()=>module.evaluateCompositeElement(broken),/Nonfinite/);
    }
});

test('whole-chain overlay preserves energy, band assembly, primal response and support reactions across mixed sections',async()=>{
    const candidate=(await loadCompositeFastChainOverlay()).module;
    for(const count of [6,32,65]) {
        const coordinates=Array.from({length:count},(_,i)=>i*2),rest=coordinates.map(x=>[x,.2*Math.sin(x/12),.2*Math.cos(x/17)]);
        const positions=rest.map((p,i)=>p.map((v,j)=>v+.03*Math.sin(i+j)));
        const edgeIds=Array.from({length:count-1},(_,i)=>i<2?['wire']:i<count-3?['wire','catheter']:['catheter']);
        const layout=chain.createCompositeChainLayout(edgeIds),a=chain.createCompositeChainWorkspace(layout,{elementBackend:'javascript'}),b=candidate.createCompositeChainWorkspace(layout);
        assert.equal(a.elementBackend,'javascript');assert.equal(b.elementBackend,'wasm');
        const data={positions,coordinates,reference:oracle.captureCompositeReferenceFrames(rest),tools:[0,1].map(i=>({id:i?'catheter':'wire',
            angles:Float64Array.from(coordinates.slice(1),x=>.02*x+.3*i),referenceTwists:new Float64Array(count-1).fill(i?2*Math.PI:0),
            dsDx:x=>1+.1*Math.sin(x/10+i),material:oracle.compileCompositeMaterial({EI:[[3,.2],[.2,5]],GJ:2,kappa0:[.01,-.02],energyOffset:.2})}))};
        chain.assembleCompositeChain(data,a);candidate.assembleCompositeChain(data,b);
        close(a.energy,b.energy);a.gradient.forEach((v,i)=>close(v,b.gradient[i]));a.hessian.forEach((v,i)=>close(v,b.hessian[i]));
        const diagonal=new Float64Array(layout.dofCount).fill(2),fixed=new Uint8Array(layout.dofCount),load=new Float64Array(layout.dofCount);
        fixed.fill(1,0,5);load[layout.positions[count-1]+1]=-.01;
        const ra=chain.solveCompositeChainIncrement(a,{diagonal,fixed,extraGradient:load,tolerance:1e-10});
        const rb=candidate.solveCompositeChainIncrement(b,{diagonal,fixed,extraGradient:load,tolerance:1e-10});
        assert.equal(ra.converged,true);assert.equal(rb.converged,true);
        for(const key of ['increment','residual','reactions'])ra[key].forEach((v,i)=>close(v,rb[key][i]));
    }
});

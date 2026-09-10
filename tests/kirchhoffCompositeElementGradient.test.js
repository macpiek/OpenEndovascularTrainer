import assert from 'node:assert/strict';
import test from 'node:test';
import * as oracle from '../src/physics/kirchhoffCompositeElement.js';
import * as exact from '../src/physics/kirchhoffCompositeElementExact.js';
import * as chain from '../src/physics/kirchhoffCompositeChain.js';
const close=(a,b,t=2e-12)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t*(1+Math.max(Math.abs(a),Math.abs(b))),`${a} != ${b}`);
const vectorClose=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const fields=['strain','jacobian','moments','gradient','toolEnergy','referenceTwists'];
const bytes=a=>Buffer.from(a.buffer,a.byteOffset,a.byteLength);
function fixture(seed=1,count=2) {
    const random=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296;};
    const rest=[[0,0,0],[2,.2,-.1],[3.8,.6,.3]];
    return {positions:rest.map(p=>p.map(v=>v+(random()-.5)*.2)),reference:oracle.captureCompositeReferenceFrames(rest),referenceLength:1.7+random(),referenceTwist:2*Math.PI,
        tools:Array.from({length:count},(_,i)=>({dsDx:.6+random(),angles:[random()-.5,random()-.5],referenceTwist:i?undefined:-4*Math.PI,
            material:oracle.compileCompositeMaterial({stiffness:[[3,.3,.2],[.3,5,-.1],[.2,-.1,2]],intrinsic:[.05*random(),-.1*random(),.3*random()],energyOffset:.1+random()})}))};
}
const compare=(a,b)=>{close(a.energy,b.energy);for(const field of fields)vectorClose(a[field],b[field]);};
function move(args,dof,delta) {if(dof<9)args.positions[Math.floor(dof/3)][dof%3]+=delta;else args.tools[Math.floor((dof-9)/2)].angles[(dof-9)%2]+=delta;}

test('gradient export matches FULL Exact and original JS E/g/all non-H diagnostics for 3D anisotropic one/two-material elements',()=>{
    for(const count of [1,2]) {
        const full=exact.createCompositeElementWorkspace(count),first=exact.createCompositeElementWorkspace(count),js=oracle.createCompositeElementWorkspace(count);
        for(let seed=1;seed<=20;seed++) {
            const args=fixture(seed,count),before=structuredClone(args);
            exact.evaluateCompositeElement(args,first,{order:'gradient'});
            compare(first,exact.evaluateCompositeElement(args,full));compare(first,oracle.evaluateCompositeElement(args,js));
            assert.equal(first.hessianValid,false);assert.equal(first.evaluationOrder,'gradient');assert.equal(full.hessianValid,true);
            assert.deepEqual(args,before);
        }
    }
});

test('gradient-only exact forces independently differentiate energy with unchanged tolerance',()=>{
    const args=fixture(421),w=exact.createCompositeElementWorkspace(2);exact.evaluateCompositeElement(args,w,{order:'gradient'});
    const gradient=w.gradient.slice(),h=1e-6;
    for(let i=0;i<w.dofCount;i++) {
        const plus=structuredClone(args),minus=structuredClone(args);move(plus,i,h);move(minus,i,-h);
        const a=exact.evaluateCompositeElement(plus,w,{order:'gradient'}).energy,b=exact.evaluateCompositeElement(minus,w,{order:'gradient'}).energy;
        close(gradient[i],(a-b)/(2*h),2e-7);
    }
});

test('gradient mode neither reads nor writes stale Hessian bytes and marks them invalid until a successful full evaluation',()=>{
    const args=fixture(17),w=exact.createCompositeElementWorkspace(2);
    exact.evaluateCompositeElement(args,w);const before=Buffer.from(bytes(w.hessian)),buffer=w.hessian;
    args.positions[2][1]+=.03;exact.evaluateCompositeElement(args,w,{order:'gradient'});
    assert.deepEqual(bytes(w.hessian),before);assert.equal(w.hessian,buffer);assert.equal(w.hessianValid,false);
    w.hessian.fill(NaN);exact.evaluateCompositeElement(args,w,{order:'gradient'});
    assert.ok(w.hessian.every(Number.isNaN));assert.equal(w.hessianValid,false);
    exact.evaluateCompositeElement(args,w);assert.equal(w.hessianValid,true);assert.ok(w.hessian.every(Number.isFinite));
    assert.notDeepEqual(bytes(w.hessian),before);
    assert.throws(()=>exact.evaluateCompositeElement(args,w,{order:'bad'}),/order/);assert.equal(w.hessianValid,false);
    exact.evaluateCompositeElement(args,w);assert.equal(w.hessianValid,true);
    const broken=fixture(17);broken.positions[1]=[...broken.positions[0]];
    assert.throws(()=>exact.evaluateCompositeElement(broken,w),/collapse/);assert.equal(w.hessianValid,false);
});

test('energy offsets and anchored atan2 branch crossing retain exact gradient/reference twist without a Hessian',()=>{
    const p=[[0,0,0],[2,0,0],[4,0,0]],base={positions:p,reference:oracle.captureCompositeReferenceFrames(p),referenceLength:2,referenceTwist:Math.PI,
        tools:[{angles:[.1,.2],dsDx:.7,material:oracle.compileCompositeMaterial({stiffness:[[3,.3,.2],[.3,5,-.1],[.2,-.1,2]],intrinsic:[.1,-.2,.3],energyOffset:.37})}]};
    const w=exact.createCompositeElementWorkspace(1);
    for(const epsilon of [-1e-7,0,1e-7]) {
        const args=structuredClone(base);args.reference[1].director=[0,-Math.cos(epsilon),Math.sin(epsilon)];
        const first=exact.evaluateCompositeElement(args,w,{order:'gradient'});compare(first,oracle.evaluateCompositeElement(args));
        close(first.referenceTwists[0],Math.PI-epsilon);assert.equal(first.hessianValid,false);
    }
});
function chainFixture(count=8) {
    const coordinates=Array.from({length:count},(_,i)=>2*i),rest=coordinates.map(x=>[x,.2*Math.sin(x/12),.2*Math.cos(x/17)]);
    const layout=chain.createCompositeChainLayout(Array.from({length:count-1},(_,edge)=>edge<2?['wire']:edge<count-3?['wire','catheter']:['catheter']));
    const data={positions:rest.map((p,i)=>p.map((v,j)=>v+.03*Math.sin(i+j))),coordinates,reference:oracle.captureCompositeReferenceFrames(rest),
        tools:[0,1].map(i=>({id:i?'catheter':'wire',angles:Float64Array.from(coordinates.slice(1),x=>.02*x+.3*i),dsDx:1.1+.1*i,
            referenceTwists:new Float64Array(count-2).fill(i?2*Math.PI:0),material:oracle.compileCompositeMaterial({EI:[[3,.2],[.2,5]],GJ:2,kappa0:[.01,-.02],energyOffset:.2})}))};
    return {layout,data};
}

test('whole-chain gradient assembly preserves E/g/winding, leaves H untouched, and the Chain solver rejects it',()=>{
    const {layout,data}=chainFixture(),w=chain.createCompositeChainWorkspace(layout,{elementBackend:'wasm-exact'}),full=chain.createCompositeChainWorkspace(layout,{elementBackend:'wasm-exact'}),js=chain.createCompositeChainWorkspace(layout,{elementBackend:'javascript'});
    chain.assembleCompositeChain(data,w);assert.equal(w.hessianValid,true);const before=Buffer.from(bytes(w.hessian));
    data.positions[3][1]+=.04;
    chain.assembleCompositeChain(data,w,{order:'gradient'});chain.assembleCompositeChain(data,full);chain.assembleCompositeChain(data,js);
    for(const other of [full,js]){close(w.energy,other.energy);vectorClose(w.gradient,other.gradient);for(const[id,v]of w.evaluatedReferenceTwists)
        v.forEach((x,i)=>Number.isNaN(x)?assert.ok(Number.isNaN(other.evaluatedReferenceTwists.get(id)[i])):close(x,other.evaluatedReferenceTwists.get(id)[i]));}
    assert.equal(w.hessianValid,false);assert.deepEqual(bytes(w.hessian),before);
    assert.throws(()=>chain.solveCompositeChainIncrement(w,{diagonal:new Float64Array(layout.dofCount).fill(1e6)}),/fresh full Hessian/);
    chain.assembleCompositeChain(data,w);assert.equal(w.hessianValid,true);assert.deepEqual(bytes(w.hessian),bytes(full.hessian));
    assert.equal(chain.solveCompositeChainIncrement(w,{diagonal:new Float64Array(layout.dofCount).fill(1e6)}).converged,true);
    assert.throws(()=>chain.assembleCompositeChain(data,js,{order:'gradient'}),/wasm-exact/);assert.equal(js.hessianValid,false);
    chain.assembleCompositeChain(data,js);assert.equal(js.hessianValid,true);
    const broken={...data,coordinates:[...data.coordinates]};broken.coordinates[1]=broken.coordinates[0];
    assert.throws(()=>chain.assembleCompositeChain(broken,w),/coordinates/);assert.equal(w.hessianValid,false);
});

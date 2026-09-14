import assert from 'node:assert/strict';
import test from 'node:test';
import * as THREE from 'three';
import { defineKirchhoffMaterialProfile } from '../src/physics/kirchhoffMaterialProfile.js';
import { assembleKirchhoffDirect } from '../src/physics/kirchhoffDirectSolver.js';
import { createSharedAxisNative, assembleSharedAxisNative, relaxSharedAxisNative, feedSharedAxisNative,
    rotateSharedAxisNative, captureSharedAxisNative, restoreSharedAxisNative, applySharedAxisNativeIncrement } from '../src/physics/kirchhoffSharedAxisNative.js';

const beam = (EI = 1e6, curvature = 0) => defineKirchhoffMaterialProfile({ id: `beam-${EI}-${curvature}`,
    sampleEI1: () => EI, sampleGJ: () => EI / 1.3, sampleKappa01: () => curvature });
const q = (b, e) => new THREE.Quaternion(b.orientationX[e], b.orientationY[e], b.orientationZ[e], b.orientationW[e]);
function accepted(s, options) {
    const result = relaxSharedAxisNative(s, options);
    assert.ok(result.converged, JSON.stringify(result));
    assert.ok(result.residual.force <= 1e-6 && result.residual.torque <= 1e-6 && result.residual.length <= 1e-5);
    return result;
}
function straightPair(insertion = 100) { return createSharedAxisNative({ tools: [
    { id: 'wire', insertion, type: beam() }, { id: 'catheter', insertion, type: beam() }] }); }

test('one authoritative axis; independent spins and one length equation per spatial edge', () => {
    const s = straightPair();
    assert.equal(s.layout.dofCount, 3 * 21 + 2 * 20);
    assert.equal(s.definitions.length, 20);
    assert.ok(s.definitions.every(r => r.kind === 'length'));
    assert.equal(s.interToolRows, 0);
    accepted(s);
    for (const { body } of s.materials) for (let n = 0; n <= body.segmentCount; n++)
        assert.deepEqual([body.x[n], body.y[n], body.z[n]], s.positions[n]);
});

test('native energy pullback has the correct spatial and independent spin derivatives', () => {
    const s = createSharedAxisNative({ tools: [{ id: 'wire', insertion:25, type:beam(1000,.012) },
        { id:'catheter', insertion:20, type:beam(2000,-.008) }] });
    const dx = Float64Array.from({length:s.layout.dofCount}, (_,i)=>.003*Math.sin(i*1.7));
    applySharedAxisNativeIncrement(s, dx, new Float64Array(s.multipliers.length));
    assembleSharedAxisNative(s); const gradient=s.chain.gradient.slice(), snapshot=captureSharedAxisNative(s);
    for(let i=0;i<dx.length;i++) {
        dx.fill(0);dx[i]=1; const h=1e-6;
        restoreSharedAxisNative(s,snapshot);applySharedAxisNativeIncrement(s,dx,new Float64Array(s.multipliers.length),h);
        const plus=assembleSharedAxisNative(s).energy;
        restoreSharedAxisNative(s,snapshot);applySharedAxisNativeIncrement(s,dx,new Float64Array(s.multipliers.length),-h);
        const minus=assembleSharedAxisNative(s).energy;
        assert.ok(Math.abs((plus-minus)/(2*h)-gradient[i])<2e-6*Math.max(1,Math.abs(gradient[i])),`dof ${i}`);
    }
    restoreSharedAxisNative(s,snapshot);
    let nativeEnergy=0;
    for(const {body} of s.materials) {
        const a=assembleKirchhoffDirect(body,1);
        for(let r=6;r<a.rowCount;r++)if(r%6<3)nativeEnergy+=.5*a.strain[r]**2/a.alpha[r];
    }
    assert.equal(assembleSharedAxisNative(s).energy,nativeEnergy);
});

test('external cantilever oracle: shared rigidity is the sum; unloading recovers the straight shape', () => {
    const tips=[];
    for(const ids of [['wire'],['wire','catheter']]) {
        const s=createSharedAxisNative({tools:ids.map(id=>({id,insertion:100,type:beam()}))});
        s.loads[s.layout.positions.at(-1)+1]=1;accepted(s);
        const y=s.positions.at(-1)[1], expected=100**3/(3*ids.length*1e6);
        assert.ok(Math.abs(y/expected-1)<.01,`${y} vs ${expected}`);tips.push(y);
        s.loads.fill(0);accepted(s);assert.ok(Math.abs(s.positions.at(-1)[1])<1e-5);
    }
    assert.ok(Math.abs(tips[0]/tips[1]-2)<1e-4);
});

test('a stiff straight wire counteracts the native curved catheter energy', () => {
    const cat={id:'catheter',insertion:60,type:beam(1e5,.005)};
    const alone=createSharedAxisNative({tools:[cat]});accepted(alone);
    const together=createSharedAxisNative({tools:[{id:'wire',insertion:60,type:beam(9e5)},cat]});accepted(together);
    const bend=s=>q(s.materials.find(t=>t.spec.id==='catheter').body,5).angleTo(q(s.materials.find(t=>t.spec.id==='catheter').body,6));
    assert.ok(Math.abs(bend(together)/bend(alone)-.1)<.002);
    assert.ok(Math.abs(together.positions.at(-1)[1])<Math.abs(alone.positions.at(-1)[1])*.11);
});

test('frictionless independent rotation: catheter torque does not rotate the wire', () => {
    const s=straightPair(40), wire=captureSharedAxisNative(s).frames[0];
    rotateSharedAxisNative(s,'catheter',.3);accepted(s);
    const [w,c]=s.materials;
    for(let e=0;e<w.last;e++)assert.ok(q(w.body,e).angleTo(new THREE.Quaternion(...wire.map(a=>a[e])))<1e-7);
    for(let e=0;e<c.last;e++)assert.ok(q(c.body,e).angleTo(q(c.body,0))<1e-7);
    const before=q(c.body,0);rotateSharedAxisNative(s,'wire',-.2);accepted(s);
    assert.ok(q(c.body,0).angleTo(before)<1e-7);
});

test('independent feed/withdrawal crosses mesh nodes and either tool may extend beyond the other', () => {
    let s=createSharedAxisNative({tools:[{id:'wire',insertion:100,shaftStiffness:39,tipStiffness:30.7},
        {id:'catheter',insertion:50,shaftStiffness:58.1,tipStiffness:87}]});accepted(s);
    rotateSharedAxisNative(s,'catheter',.1);accepted(s);
    for(const insertion of [51,54.9,55,55.1,75,100,110,90,30]) {
        const saved=captureSharedAxisNative(s),c=feedSharedAxisNative(s,{catheter:insertion});
        assert.deepEqual(captureSharedAxisNative(s),saved,'candidate must not change accepted geometry/history');
        assert.equal(c.materials.find(t=>t.spec.id==='wire').spec.insertion,100);
        assert.ok(q(c.materials[1].body,0).angleTo(q(s.materials[1].body,0))<1e-7);
        for(const {body,spec,last} of c.materials) {
            assert.equal(body.materialCoordinate[last],1000);
            for(let n=0;n<=last;n++)assert.ok(Math.abs(body.materialCoordinate[n]-(1000+c.coordinates[n]-spec.insertion))<1e-8);
        }
        accepted(c);s=c;
    }
    for(const wire of [90,80,110]) { const c=feedSharedAxisNative(s,{wire});accepted(c);s=c;assert.equal(s.materials[1].spec.insertion,30); }
});

test('planar wall acts on the outer catheter radius in overlap and wire radius beyond it', () => {
    const owners=[];
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:50,type:beam(),radius:.4},
        {id:'catheter',insertion:25,type:beam(),radius:.8}],wallSample:({b,radius,owner})=>{
            owners.push([owner,radius]);return {gap:1-radius-b[1],jacobian:[0,0,0,0,-1,0]};
        }});
    s.loads[s.layout.positions.at(-1)+1]=50;accepted(s);
    assert.ok(owners.some(([id,r])=>id==='catheter'&&r===.8));assert.ok(owners.some(([id,r])=>id==='wire'&&r===.4));
    for(const row of assembleSharedAxisNative(s).rows.filter(r=>r.kind==='wall'))assert.ok(row.gap>=-1e-5&&row.multiplier>=0);
    assert.ok(s.multipliers.some((v,i)=>s.definitions[i].kind==='wall'&&v>0));
    s.loads.fill(0);accepted(s);assert.ok(Math.abs(s.positions.at(-1)[1])<1e-5);
});

test('nonconverged solve is atomic and never commits a partial relaxation', () => {
    const s=straightPair();s.loads[s.layout.positions.at(-1)+1]=100;
    const before=captureSharedAxisNative(s),result=relaxSharedAxisNative(s,{maxIterations:1});
    assert.equal(result.converged,false);assert.deepEqual(captureSharedAxisNative(s),before);assert.equal(s.acceptedSolves,0);
});

test('a failed geometry callback restores the complete accepted state', () => {
    let calls=0;
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:30,type:beam()}],wallSample:()=>{
        if(++calls>6)throw new Error('synthetic geometry failure');
        return {gap:10,jacobian:[0,0,0,0,1,0]};
    }});
    s.loads[s.layout.positions.at(-1)+1]=10;
    const before=captureSharedAxisNative(s),result=relaxSharedAxisNative(s);
    assert.equal(result.converged,false);assert.match(result.error,/synthetic geometry failure/);
    assert.deepEqual(captureSharedAxisNative(s),before);
});

test('invalid and degenerate geometry is rejected explicitly', () => {
    assert.throws(()=>createSharedAxisNative({tools:[{id:'wire',insertion:20}],samplePosition:()=>[0,0,0]}),/nondegenerate/);
    assert.throws(()=>relaxSharedAxisNative(straightPair(),{forceTolerance:0}),/convergence/);
});

test('partial-overlap cantilever agrees with the piecewise rigidity integral', () => {
    const s=createSharedAxisNative({spacing:2.5,tools:[{id:'wire',insertion:100,type:beam()},
        {id:'catheter',insertion:50,type:beam()}]});
    s.loads[s.layout.positions.at(-1)+1]=1;accepted(s);
    // y(L) = F integral((L-x)^2/EI(x), x=0..L).
    const expected=(100**3-50**3)/(3*2e6)+50**3/(3*1e6);
    assert.ok(Math.abs(s.positions.at(-1)[1]/expected-1)<.02);
});

test('long wire / short overlap remains converged after feed and curved-material rotation', () => {
    let s=createSharedAxisNative({tools:[{id:'wire',insertion:309,shaftStiffness:39,tipStiffness:30.7},
        {id:'catheter',insertion:100,shaftStiffness:58.1,tipStiffness:87}]});accepted(s);
    for(let step=1;step<=12;step++) {s=feedSharedAxisNative(s,{wire:309+step*.25,catheter:100+step*.25});accepted(s);}
    rotateSharedAxisNative(s,'catheter',.005);accepted(s);
});

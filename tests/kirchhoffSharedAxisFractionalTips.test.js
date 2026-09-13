import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisNative,assembleSharedAxisNative,extendSharedAxisNativeRows,captureSharedAxisNative,restoreSharedAxisNative,applySharedAxisNativeIncrement,sharedAxisOuterMaterialAt} from '../src/physics/kirchhoffSharedAxisNative.js';
import {prepareSharedAxisDynamicStep} from '../src/physics/kirchhoffSharedAxisDynamics.js';
import {createSharedAxisContacts} from '../src/physics/kirchhoffSharedAxisContacts.js';
import {prepareSharedAxisWallFriction} from '../src/physics/kirchhoffSharedAxisWallFriction.js';
const close=(a,b,tol=1e-10)=>assert.ok(Math.abs(a-b)<=tol,`${a} versus ${b}`);
const tools=catheter=>[{id:'wire',insertion:20,radius:.4,wallStaticFriction:.01,wallKineticFriction:.01},{id:'catheter',insertion:catheter,radius:.8,wallStaticFriction:.01,wallKineticFriction:.01}];

test('fractional coverage preserves both physical tips and material profiles without a submicron spatial cell',()=>{
    for(const tip of [19.9999,20.0001]) {
        const s=createSharedAxisNative({tools:tools(tip)});
        assert.ok(s.coordinates.slice(1).every((x,i)=>x-s.coordinates[i]>4.99));
        for(const m of s.materials) {
            close(m.coordinates.at(-1),m.spec.insertion,0);close(m.body.materialCoordinate[m.last],1000,0);
            close(m.body.x[m.last],m.spec.insertion);
            close(m.body.restLength.reduce((a,b)=>a+b,0),m.spec.insertion);
        }
        prepareSharedAxisDynamicStep(s,1/60);
        close(s.dynamicStep.masses.reduce((a,b)=>a+b,0),(20+1.4*tip)/5);
        const firstMoment=s.positions.reduce((sum,p,i)=>sum+p[0]*s.dynamicStep.masses[i],0);
        close(firstMoment,(20**2+1.4*tip**2)/10);
    }
});

test('fractional material energy and dynamic tangent agree with finite differences on the common axis',()=>{
    const s=createSharedAxisNative({tools:tools(19.9999)});prepareSharedAxisDynamicStep(s,1/120);
    const zero=new Float64Array(s.multipliers.length),dx=Float64Array.from({length:s.layout.dofCount},(_,i)=>.002*Math.sin(i));
    applySharedAxisNativeIncrement(s,dx,zero);assembleSharedAxisNative(s);
    const saved=captureSharedAxisNative(s),gradient=s.chain.gradient.slice(),H=s.chain.tangent.slice(),half=s.layout.band-1,width=2*half+1,h=1e-6;
    for(let col=0;col<dx.length;col++) {
        dx.fill(0);dx[col]=1;
        const sample=sign=>{restoreSharedAxisNative(s,saved);applySharedAxisNativeIncrement(s,dx,zero,sign*h);const a=assembleSharedAxisNative(s);return {energy:a.energy,g:s.chain.gradient.slice()};};
        const a=sample(1),b=sample(-1);close((a.energy-b.energy)/(2*h),gradient[col],1e-4*Math.max(1,Math.abs(gradient[col])));
        for(let row=0;row<dx.length;row++) {
            const expected=Math.abs(row-col)<=half?H[row*width+col-row+half]:0;
            close((a.g[row]-b.g[row])/(2*h),expected,2e-5*Math.max(1,Math.abs(expected)));
        }
    }
});

test('capsule queries split the outer catheter from the actual wire-only fractional sliver',()=>{
    const calls=[],field={voxelSize:1,queryCapsuleSoA(x,y,z,r,index,out){calls.push({a:x[0],b:x[1],radius:r[0]});return Object.assign(out,{signedDistance:10,signedGap:9,segmentT:.5,faceIndex:0});}};
    const sheath={start:[0,0,0],end:[10,0,0],innerRadius:.9,proximalExtension:40};
    const s=createSharedAxisNative({...createSharedAxisContacts({sheath,contactField:field}),tools:tools(19.9999)});assembleSharedAxisNative(s);
    const last=calls.slice(-2);close(last[0].a,15);close(last[0].b,19.9999);close(last[0].radius,.8);
    close(last[1].a,19.9999);close(last[1].b,20);close(last[1].radius,.4);
    const e=s.coordinates.length-2,t=(s.materials[1].endFraction+1)/2;
    assert.equal(sharedAxisOuterMaterialAt(s,e,t).spec.id,'wire');
    extendSharedAxisNativeRows(s,[{kind:'wall',edge:e,id:'wire-only',witness:{face:0,t},dofs:[s.layout.positions[e],s.layout.positions[e+1]].flatMap(i=>[i,i+1,i+2]),evaluate:()=>({gap:0,jacobian:[0,1-t,0,0,t,0]})}]);
    s.multipliers[s.multipliers.length-1]=100;prepareSharedAxisDynamicStep(s,1/60);prepareSharedAxisWallFriction(s,{feedById:{wire:.01}});
    assert.equal(s.wallFrictionStep.records[0].material.spec.id,'wire');
});

test('the larger catheter tip keeps its own sheath constraint inside a fractional cell',()=>{
    const sheath={start:[0,0,0],end:[30,0,0],innerRadius:.9,proximalExtension:40};
    const s=createSharedAxisNative({...createSharedAxisContacts({sheath}),tools:tools(19.9999)});s.positions.at(-1)[1]=.2;
    const rows=assembleSharedAxisNative(s).rows.filter(r=>r.edge===s.coordinates.length-2&&r.kind==='wall');
    const catheter=rows.find(r=>r.sampleT!==undefined),wire=rows.find(r=>r.sampleT===undefined);
    assert.ok(catheter.gap<-.099);assert.ok(wire.gap>.29);
});

test('a fractional prepared checkpoint restores its rebased world origin and exact physical equations',async()=>{
    const {feedSharedAxisNative}=await import('../src/physics/kirchhoffSharedAxisNative.js');
    const {captureSharedAxisReplay,restoreSharedAxisReplay}=await import('./helpers/sharedAxisReplay.js');
    const sheath={start:[101,-37,20],end:[131,-37,20],innerRadius:.9,proximalExtension:40};
    const initial=createSharedAxisNative({...createSharedAxisContacts({sheath,localCoordinates:true}),tools:tools(19.9999)});
    const s=feedSharedAxisNative(initial,{catheter:20.0001});
    assert.notDeepEqual(s.origin,sheath.start,'This fixture must exercise a genuinely rebased origin');
    prepareSharedAxisDynamicStep(s,1/120);prepareSharedAxisWallFriction(s,{feedById:{catheter:.0002}});
    const dx=Float64Array.from({length:s.layout.dofCount},(_,i)=>.0001*Math.sin(i));
    applySharedAxisNativeIncrement(s,dx,new Float64Array(s.multipliers.length));
    const checkpoint=JSON.parse(JSON.stringify(captureSharedAxisReplay(s,sheath))),restored=restoreSharedAxisReplay(checkpoint,null);
    assert.deepEqual(restored.origin,s.origin);assert.equal(restored.fractionalTipThreshold,s.fractionalTipThreshold);
    for(let i=0;i<s.materials.length;i++) {
        assert.deepEqual(restored.materials[i].coordinates,s.materials[i].coordinates);
        assert.equal(restored.materials[i].endFraction,s.materials[i].endFraction);
    }
    const a=assembleSharedAxisNative(s),b=assembleSharedAxisNative(restored);
    assert.equal(a.energy,b.energy);assert.deepEqual(a.rows,b.rows);
    assert.deepEqual(restored.chain.gradient,s.chain.gradient);assert.deepEqual(restored.chain.tangent,s.chain.tangent);
});

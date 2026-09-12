import assert from 'node:assert/strict';
import test from 'node:test';
import {closestSegmentSegment} from '../src/physics/kirchhoffLumenContact.js';
import {createCompositeExternalCapsuleGeometryWorkspace as workspace,evaluateCompositeExternalCapsuleContact as evaluate} from '../src/physics/kirchhoffCompositeExternalCapsuleGeometry.js';
const keys=['innerStart','innerEnd','outerStart','outerEnd'],N=12,
    dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),sub=(a,b)=>a.map((v,i)=>v-b[i]),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],
    close=(a,b,t=1e-8)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t*Math.max(1,Math.abs(a),Math.abs(b)),`${a} != ${b}`);
const same=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
function fixture(kind='interior') {
    const input={innerStart:[-1,0,.3],innerEnd:[1,0,.3],outerStart:[0,-1,0],outerEnd:[0,1,0],innerRadius:.1,outerRadius:.2};
    if(kind==='inner-clamp')Object.assign(input,{innerStart:[.4,0,.3],innerEnd:[1.3,.1,.35]});
    if(kind==='outer-clamp')Object.assign(input,{innerStart:[-1,1.3,.3],innerEnd:[1,1.4,.35]});
    if(kind==='both-clamp')Object.assign(input,{innerStart:[.4,1.3,.3],innerEnd:[1.3,1.4,.35]});
    if(kind==='parallel')Object.assign(input,{innerStart:[.3,.2,.1],innerEnd:[1.3,.2,.1],outerStart:[0,0,0],outerEnd:[2,0,0]});
    return input;
}
const sample=(input,order='full',out=workspace())=>evaluate({input,order},out);
function perturb(input,j,h) {const p=structuredClone(input);p[keys[Math.floor(j/3)]][j%3]+=h;return p;}
for(const kind of ['interior','inner-clamp','outer-clamp','both-clamp','parallel'])test(`${kind}: original native capsule values and all moving-foot B/DB/gap derivatives`,()=>{
    const input=fixture(kind),out=sample(input),raw=closestSegmentSegment(...keys.map(k=>input[k])),h=2e-6;
    assert.equal(out.supported,true,out.reason);same(out.innerPoint,raw.firstPoint,1e-14);same(out.outerPoint,raw.secondPoint,1e-14);
    close(out.innerT,raw.firstT,1e-14);close(out.outerT,raw.secondT,1e-14);close(out.gap,raw.distance-.3,1e-14);
    for(let j=0;j<N;j++) {
        let plus=sample(perturb(input,j,h)),minus=sample(perturb(input,j,-h)),step=2*h,tol=5e-8;
        // At exact parallelism the original deterministic witness is at A0.
        // Retain that endpoint member rather than average the clamp members.
        if(kind==='parallel') {if(plus.innerT===out.innerT){minus=out;step=h;}else {plus=out;step=h;assert.equal(minus.innerT,out.innerT);}tol=3e-5;}
        assert.equal(plus.supported,true);assert.equal(minus.supported,true);
        close(out.gapJacobian[j],(plus.gap-minus.gap)/step,tol);
        close(out.innerTGradient[j],(plus.innerT-minus.innerT)/step,tol);close(out.outerTGradient[j],(plus.outerT-minus.outerT)/step,tol);
        for(let i=0;i<N;i++)close(out.normalDerivative[N*i+j],(plus.normalForceColumn[i]-minus.normalForceColumn[i])/step,tol);
    }
    const force=[0,0,0],moment=[0,0,0],direction=Array.from({length:N},(_,i)=>Math.sin(i+.4));
    keys.forEach((key,i)=>{const f=Array.from(out.normalForceColumn.slice(3*i,3*i+3)),m=cross(input[key],f);f.forEach((v,k)=>force[k]+=v);m.forEach((v,k)=>moment[k]+=v);});
    same(force,[0,0,0],1e-14);same(moment,[0,0,0],1e-14);same(out.forceColumn,Array.from(out.normalForceColumn,v=>-v),0);
    // Away from the deliberately selected nearparallel fallback chart, the
    // closest-point stationarity makes physical pressure exactly conjugate.
    close(dot(out.gapJacobian,direction),dot(out.normalForceColumn,direction),1e-12);
});

test('exact clamp equality selects a one-sided endpoint derivative, and open distal ownership uses the original beyond-mouth test',()=>{
    const input=fixture('inner-clamp');input.innerStart=[0,0,.3];const out=sample(input),h=1e-6;
    assert.equal(out.innerT,0);assert.ok(out.innerTGradient.every(v=>v===0));
    for(let j=0;j<N;j++) {
        let next,sign;for(const s of [1,-1]){const p=sample(perturb(input,j,s*h));if(p.innerT===0){next=p;sign=s;break;}}
        assert.ok(next);for(let i=0;i<N;i++)close(out.normalDerivative[N*i+j],(next.normalForceColumn[i]-out.normalForceColumn[i])/(sign*h),3e-5);
    }
    const beyond={innerStart:[2.1,.2,0],innerEnd:[3,.3,0],outerStart:[0,0,0],outerEnd:[2,0,0],innerRadius:.1,outerRadius:.3,openDistalB:true};
    assert.equal(sample(beyond).openDistalExcluded,true);assert.equal(sample({...beyond,openDistalB:false}).openDistalExcluded,false);
    const atMouth={...beyond,innerStart:[2,.2,0],innerEnd:[2,.3,0]};assert.equal(sample(atMouth).openDistalExcluded,false);
});

test('owned derivative validity resets and exact degeneracy stays explicit',()=>{
    const input=fixture(),ws=workspace(),full=structuredClone(sample(input,'full',ws)),old=structuredClone(input),gradient=sample(input,'gradient',ws);
    same(gradient.normalForceColumn,full.normalForceColumn,0);same(gradient.gapJacobian,full.gapJacobian,0);assert.equal(gradient.hessianValid,false);
    assert.ok(gradient.normalDerivative.every(Number.isNaN));assert.ok(gradient.innerTGradient.every(Number.isNaN));assert.deepEqual(input,old);
    const witness=sample(input,'witness',ws);same(witness.innerTGradient,full.innerTGradient,0);assert.ok(witness.normalDerivative.every(Number.isNaN));
    same(sample(input,'full',ws).normalDerivative,full.normalDerivative,0);
    assert.equal(sample({...input,innerEnd:input.innerStart},'full',ws).reason,'degenerate-native-edge');
    assert.equal(sample({...input,innerStart:[-1,0,0],innerEnd:[1,0,0]},'full',ws).reason,'coincident-native-capsule-witness');
});

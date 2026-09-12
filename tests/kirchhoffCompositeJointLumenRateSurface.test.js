import assert from 'node:assert/strict';
import test from 'node:test';
import {evaluateKirchhoffLumenSegmentContact as detect} from '../src/physics/kirchhoffLumenContact.js';
import {createCompositeLumenSideGeometryWorkspace, differentiateCompositeLumenSideContact} from '../src/physics/kirchhoffCompositeLumenSideGeometry.js';
import {createCompositeLumenTipGeometryWorkspace, differentiateCompositeLumenTipContact} from '../src/physics/kirchhoffCompositeLumenTipGeometry.js';
import {createCompositeJointLumenRateSurfaceWorkspace, evaluateCompositeJointLumenRateSurface} from '../src/physics/kirchhoffCompositeJointLumenRateSurface.js';

import {createCompositeExternalCapsuleGeometryWorkspace,evaluateCompositeExternalCapsuleContact} from '../src/physics/kirchhoffCompositeExternalCapsuleGeometry.js';

const N=14,names=['innerStart','innerEnd','outerStart','outerEnd'],dt=.013;
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),add=(a,b)=>a.map((v,i)=>v+b[i]),sub=(a,b)=>a.map((v,i)=>v-b[i]),scale=(v,a)=>v.map(x=>x*a),
    unit=v=>scale(v,1/Math.hypot(...v)),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const close=(a,b,t=3e-8,message='')=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t*Math.max(1,Math.abs(a),Math.abs(b)),`${message}: ${a} != ${b}`);
const same=(a,b,t,message)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t,`${message ?? ''}[${i}]`));};

function fixture(kind) {
    const input={innerStart:[.3,.2,.04],innerEnd:[1.2,.25,.1],outerStart:[0,0,0],outerEnd:[2,0,0],
        lumenRadius:.5,innerRadius:.16,openDistal:false,portalFilletRadius:0,quadrature:[.37],activationDistance:100,
        innerMaterialSegmentId:'wire0',outerMaterialSegmentId:'cat0'};
    if(kind==='fillet'||kind==='rim')Object.assign(input,{innerStart:kind==='fillet'?[1.9,.38,0]:[1.8,.4,.05],
        innerEnd:kind==='fillet'?[2.1,.38,0]:[2.3,.7,.12],openDistal:true,portalFilletRadius:.15,quadrature:[.25]});
    if(kind==='endpoint0'||kind==='endpoint1') {
        const x=kind==='endpoint0'?0:2;
        Object.assign(input,{innerStart:[x,.2,.04],innerEnd:[x+.3,.25,.1],quadrature:[0],endpointDerivative:'clamped-one-sided'});
    }
    if(kind==='external')Object.assign(input,{innerStart:[.4,-.6,.35],innerEnd:[1.5,.7,.45],outerStart:[0,0,0],outerEnd:[2,0,0],outerRadius:.24,openDistalB:false});
    const tools=['wire','catheter'].map((id,i)=>{
        const positions=names.slice(2*i,2*i+2).map(name=>input[name].slice()),previousPositions=positions.map((p,end)=>p.map((v,k)=>v-.008*Math.sin(1+3*i+2*end+k))),
            tangent=unit(sub(previousPositions[1],previousPositions[0])),seed=[0,0,1],director=unit(sub(seed,scale(tangent,dot(seed,tangent))));
        return {id,edgeId:`${id}0`,edge:0,positions,previousPositions,coordinates:i?[7,10]:[2,3.3],angle:i?-.41:.33,previousAngle:i?-.54:.21,
            reference:{tangent,director},materialMap:{sStart:i?20:40,dsDx:i?.83:1.12,dsDtEnds:i?[-.06,-.02]:[.11,.16]}};
    });
    return {input,tools,kind};
}
function pressure(f) {
    if(f.kind==='external')return evaluateCompositeExternalCapsuleContact({input:f.input,order:'full'},createCompositeExternalCapsuleGeometryWorkspace());
    const result=detect(f.input),side=f.kind==='side'||f.kind.startsWith('endpoint'),contact=side?result.side:f.kind==='fillet'?result.fillet:result.portal.contact;
    assert.ok(contact,`Original ${f.kind} contact required`);
    const g=side?differentiateCompositeLumenSideContact({input:f.input,contact},createCompositeLumenSideGeometryWorkspace()):
        differentiateCompositeLumenTipContact({input:f.input,contact},createCompositeLumenTipGeometryWorkspace());
    assert.ok(g.supported,g.reason);return g;
}
function evaluate(f,order='full',ws=createCompositeJointLumenRateSurfaceWorkspace()) {
    return evaluateCompositeJointLumenRateSurface({geometry:pressure(f),input:f.input,tools:f.tools,dt,order},ws);
}
function perturb(f,column,amount) {
    const p=structuredClone(f),tool=Math.floor(column/7),local=column%7;
    if(local===6)p.tools[tool].angle+=amount;
    else {const end=Math.floor(local/3),axis=local%3;p.input[names[tool*2+end]][axis]+=amount;p.tools[tool].positions[end][axis]+=amount;}
    return p;
}

for(const kind of ['side','fillet','rim','external'])test(`${kind}: all 14 own position/spin columns include moving pressure normal, witness and tangent in DB and slip`,()=>{
    const f=fixture(kind),out=evaluate(f),h=1e-6;
    assert.equal(out.motion.finiteStepSlipKnown,false);assert.equal(out.motion.rateMode,'backward-euler-grid');
    assert.equal(out.motion.slipModel,'implicit-backward-euler-surface-rate');assert.equal(out.motion.includesHingeTransport,false);
    assert.equal(out.identity.witness,'common-inner-capsule-point-along-physical-normal');
    assert.ok(out.increment.some(v=>Math.abs(v)>1e-3));
    let movingNormal=0,movingPoint=0,movingFraction=0;
    for(let j=0;j<N;j++) {
        const plus=evaluate(perturb(f,j,h)),minus=evaluate(perturb(f,j,-h));
        for(let i=0;i<2*N;i++)close(out.DforceMap[i*N+j],(plus.forceMap[i]-minus.forceMap[i])/(2*h),7e-8,`${kind} DB ${i},${j}`);
        for(let c=0;c<2;c++)close(out.slipJacobian[c*N+j],(plus.increment[c]-minus.increment[c])/(2*h),7e-8,`${kind} slip ${c},${j}`);
        for(let k=0;k<3;k++) {
            close(out.currentQueryJacobian[(2+k)*N+j],(plus.point[k]-minus.point[k])/(2*h),7e-8,`${kind} point ${k},${j}`);
            close(out.currentQueryJacobian[(5+k)*N+j],(plus.normal[k]-minus.normal[k])/(2*h),7e-8,`${kind} normal ${k},${j}`);
            close(out.currentQueryJacobian[(8+k)*N+j],(plus.tangent[k]-minus.tangent[k])/(2*h),7e-8,`${kind} tangent ${k},${j}`);
            movingNormal=Math.max(movingNormal,Math.abs(out.currentQueryJacobian[(5+k)*N+j]));
            movingPoint=Math.max(movingPoint,Math.abs(out.currentQueryJacobian[(2+k)*N+j]));
        }
        for(let i=0;i<2;i++)movingFraction=Math.max(movingFraction,Math.abs(out.currentQueryJacobian[i*N+j]));
    }
    assert.ok(movingNormal>.1&&movingPoint>.1);if(kind!=='fillet')assert.ok(movingFraction>.1);
    const saved=structuredClone(f),ws=createCompositeJointLumenRateSurfaceWorkspace(),full=structuredClone(evaluate(f,'full',ws)),value=evaluate(f,'value',ws);
    same(value.forceMap,full.forceMap,0);same(value.increment,full.increment,0);assert.equal(value.DforceMapValid,false);assert.ok(value.DforceMap.every(Number.isNaN));
    assert.deepEqual(f,saved);same(evaluate(f,'full',ws).DforceMap,full.DforceMap,0);
});

for(const kind of ['endpoint0','endpoint1'])test(`${kind}: all 14 columns match the declared clamped one-sided member of the original detector`,()=>{
    const f=fixture(kind),out=evaluate(f),h=2e-10,end=kind==='endpoint0'?0:1;
    assert.equal(pressure(f).outerT,end);
    // The original detector allows only a 1e-9 endpoint tolerance. Pick the
    // clamped side of every coordinate direction inside that actual tolerance;
    // central differences would mix its distinct interior derivative member.
    for(let j=0;j<N;j++) {
        let p,sign;
        for(const candidate of [1,-1]) {const trial=perturb(f,j,candidate*h);if(pressure(trial).outerT===end){p=evaluate(trial);sign=candidate;break;}}
        assert.ok(p,`${kind} clamped direction ${j}`);
        for(let i=0;i<2*N;i++)close(out.DforceMap[i*N+j],(p.forceMap[i]-out.forceMap[i])/(sign*h),3e-5,`${kind} DB ${i},${j}`);
        for(let c=0;c<2;c++)close(out.slipJacobian[c*N+j],(p.increment[c]-out.increment[c])/(sign*h),3e-5,`${kind} slip ${c},${j}`);
    }
});

test('all native pair branches transfer equal force and full spatial moment at one witness, with exact material-rate virtual power',()=>{
    for(const kind of ['side','endpoint0','endpoint1','fillet','rim','external']) {
        const f=fixture(kind),out=evaluate(f),Ft=[.7,-.43],force=out.physicalForce,
            worldForce=add(scale(Array.from(force.axes[0]),Ft[0]),scale(Array.from(force.axes[1]),Ft[1])),totalForce=[0,0,0],totalMoment=[0,0,0],
            generalized=Array.from({length:N},(_,i)=>dot(Array.from(out.forceMap.slice(2*i,2*i+2)),Ft));
        let separateSpin=0,connectionWitness=0;
        f.tools.forEach((tool,i)=>{
            const local=generalized.slice(7*i,7*i+7),r=force.tools[i],tangent=Array.from(r.tangent),spin=local[6],
                connection=Array.from({length:6},(_,j)=>dot(tangent,[r.omegaMap[j],r.omegaMap[7+j],r.omegaMap[14+j]])),
                physicalForces=[0,1].map(end=>[0,1,2].map(k=>local[3*end+k]-spin*connection[3*end+k])),
                applied=scale(worldForce,i===0?1:-1),sum=add(...physicalForces),moment=add(add(cross(tool.positions[0],physicalForces[0]),cross(tool.positions[1],physicalForces[1])),scale(tangent,spin));
            same(sum,applied,1e-12,`${kind} resultant`);same(moment,cross(Array.from(out.point),applied),1e-12,`${kind} wrench`);
            sum.forEach((v,k)=>totalForce[k]+=v);moment.forEach((v,k)=>totalMoment[k]+=v);
            separateSpin=Math.max(separateSpin,Math.abs(spin));connectionWitness=Math.max(connectionWitness,Math.hypot(...connection));
        });
        same(totalForce,[0,0,0],1e-12);same(totalMoment,[0,0,0],1e-12);assert.ok(separateSpin>.001);assert.ok(connectionWitness>.001);
        close(dot(generalized,Array.from(out.motion.rates)),dot(Ft,Array.from(out.increment,v=>v/dt)),1e-12,`${kind} material power`);
        const zeroFeed=structuredClone(f);zeroFeed.tools.forEach(t=>t.materialMap.dsDtEnds=[0,0]);
        assert.ok(Math.hypot(...sub(Array.from(out.increment),Array.from(evaluate(zeroFeed).increment)))>1e-4,'independent actual material feed contributes to slip');
    }
});

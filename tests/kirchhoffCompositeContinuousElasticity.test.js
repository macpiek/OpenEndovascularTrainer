import test from 'node:test';
import assert from 'node:assert/strict';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeContinuousGeometry} from '../src/physics/kirchhoffCompositeContinuousGeometry.js';
import {createCompositeContinuousFrame as prepare,createCompositeContinuousFrameWorkspace as workspace,
    evaluateCompositeContinuousFrame as frameAt,evaluateCompositeContinuousStrain as strainAt,evaluateCompositeContinuousStrains as strainsAt,
    evaluateCompositeContinuousElasticDensities as densitiesAt} from '../src/physics/kirchhoffCompositeContinuousFrame.js';
import {createCompositeContinuousElasticEdge as elastic} from '../src/physics/kirchhoffCompositeContinuousElasticity.js';

const close=(a,b,t=1e-9)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}; tolerance ${t}`);
const same=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,j)=>close(v,b[j],t));};
const dot=(a,b)=>a.reduce((s,v,k)=>s+v*b[k],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],
    rotate=(v,t,a)=>v.map((x,k)=>Math.cos(a)*x+Math.sin(a)*cross(t,v)[k]+(1-Math.cos(a))*dot(t,v)*t[k]);
function fixture({count=5,edge=1,bend=.04,step=.05}={}) {
    const coordinates=Array.from({length:count},(_,j)=>j),previousPositions=coordinates.map(x=>[x,bend*x*x,0]),previousAngles=coordinates.slice(1).map((_,j)=>step*j),
        source={geometry:createCompositeContinuousGeometry({coordinates}),edge,toolId:'wire',previousPositions,previousAngles,
            reference:captureCompositeReferenceFrames(previousPositions,[0,0,1]),referenceTwists:new Array(count-2).fill(0)},plan=prepare(source),
        input={positions:plan.positionNodeIndices.map(j=>previousPositions[j].slice()),angles:plan.angleEdgeIndices.map(j=>previousAngles[j]),coordinate:edge+.37,dsDx:1.3};
    return {source,plan,input};
}
function perturb(input,j,h) {const p=3*input.positions.length;if(j<p)input.positions[Math.floor(j/3)][j%3]+=h;else input.angles[j-p]+=h;}
const material={stiffness:[[2,.1,.05],[.1,3,-.07],[.05,-.07,1]],intrinsic:[.012,-.02,.03]};

test('direct scalar energy derivatives retain material and geometric stiffness for distinct anisotropic samples',()=>{
    const f=fixture(),coordinates=[1.1,1.7],materials=[compileCompositeMaterial(material),compileCompositeMaterial({...material,intrinsic:[.4,-.6,.2],energyOffset:1.2})],
        input={...f.input,coordinates,materials},pool=workspace(),densities=densitiesAt(input,f.plan,pool),strains=strainsAt(input,f.plan,pool);
    densities.forEach((r,sample)=>{
        const m=materials[sample],s=strains[sample],N=s.configurationDofs,error=s.strain.map((v,j)=>v-m.intrinsic[j]),
            moment=error.map((_,i)=>m.stiffness[3*i]*error[0]+m.stiffness[3*i+1]*error[1]+m.stiffness[3*i+2]*error[2]);
        close(r.energy,.5*dot(error,moment)+m.energyOffset,1e-14);
        for(let i=0;i<N;i++) {
            close(r.gradient[i],moment.reduce((sum,v,k)=>sum+v*s.jacobian[k*N+i],0),1e-13);
            for(let j=0;j<N;j++) {
                let expected=0;for(let k=0;k<3;k++) {
                    expected+=moment[k]*s.hessian[(k*N+i)*N+j];
                    for(let l=0;l<3;l++)expected+=s.jacobian[k*N+i]*m.stiffness[3*k+l]*s.jacobian[l*N+j];
                }
                close(r.hessian[i*N+j],expected,2e-12);
            }
        }
    });
    const first=densitiesAt({...input,order:'value'},f.plan,pool);first.forEach((r,j)=>{same(r.gradient,densities[j].gradient,0);assert.equal(r.hessian,null);});
    assert.throws(()=>densitiesAt({...input,materials:[]},f.plan,pool),/explicit material/);
    assert.throws(()=>densitiesAt({...input,materials:[{...materials[0],intrinsic:[NaN,0,0]},materials[1]]},f.plan,pool),/finite/);
    assert.deepEqual(densitiesAt(input,f.plan,pool),densities);
});

test('analytic Darboux strain uses the surface frame and its exact configuration gradient/Hessian match finite differences',()=>{
    const f=fixture(),w=workspace(f.plan),r=strainAt(f.input,f.plan,w),surface=frameAt(f.input,f.plan,w),N=r.configurationDofs,D=N+1,h=1e-6,
        omega=[0,1,2].map(k=>surface.angularRateMap[k*D+N]);
    same(r.strain,surface.directors.map(d=>dot(d,omega)/f.input.dsDx),2e-14);
    for(let j=0;j<N;j++) {
        const plus=structuredClone(f.input),minus=structuredClone(f.input);perturb(plus,j,h);perturb(minus,j,-h);
        const a=strainAt({...plus,order:'value'},f.plan,w),b=strainAt({...minus,order:'value'},f.plan,w);
        for(let k=0;k<3;k++) {
            close((a.strain[k]-b.strain[k])/(2*h),r.jacobian[k*N+j],2e-8);
            for(let i=0;i<N;i++)close((a.jacobian[k*N+i]-b.jacobian[k*N+i])/(2*h),r.hessian[(k*N+i)*N+j],3e-7);
        }
    }
});

test('batched strains are bitwise equal to separate evaluations while one native build and bounded arena serve different frames',()=>{
    const pool=workspace(),f=fixture(),coordinates=[1.1,1.3,1.7,1.9],before=pool.diagnostics,
        batch=strainsAt({...f.input,coordinates},f.plan,pool),after=pool.diagnostics;
    assert.equal(after.nativeBuilds-before.nativeBuilds,1);assert.equal(after.samples-before.samples,coordinates.length);
    coordinates.forEach((coordinate,j)=>assert.deepEqual(batch[j],strainAt({...f.input,coordinate},f.plan,workspace(f.plan))));
    const large=fixture({count:12,edge:5,bend:.003}),r=strainAt(large.input,large.plan,pool),maximum=pool.diagnostics;
    assert.equal(r.configurationDofs,29);assert.ok(maximum.retainedBytes<=2048*(2*(1+29)+5)*8+2048*13+29*29*8+65536,'first derivatives, adjoints and one output Hessian, with bounded WASM page rounding');
    const again=strainAt(f.input,f.plan,pool);same(again.strain,batch[0].strain.map((_,j)=>strainAt(f.input,f.plan).strain[j]),0);
    assert.equal(pool.diagnostics.arenaAllocations,maximum.arenaAllocations);assert.equal(pool.diagnostics.retainedBytes,maximum.retainedBytes);
    assert.throws(()=>strainsAt({...f.input,coordinates:[1.3,NaN]},f.plan,pool),/finite/);
    assert.deepEqual(strainAt(large.input,large.plan,pool).hessian,r.hessian);
});

test('independent native frame gauges preserve material strain and all exact derivatives',()=>{
    const f=fixture(),r=strainAt(f.input,f.plan),gauges=f.source.previousAngles.map((_,j)=>.13*j+(j%2?2:-2)*Math.PI);
    f.source.reference.forEach((ref,j)=>ref.director=rotate(ref.director,ref.tangent,gauges[j]));
    f.source.previousAngles=f.source.previousAngles.map((v,j)=>v-gauges[j]);f.source.referenceTwists=f.source.referenceTwists.map((v,j)=>v+gauges[j+1]-gauges[j]);
    f.input.angles=f.plan.angleEdgeIndices.map((j,k)=>f.input.angles[k]-gauges[j]);f.plan=prepare(f.source);
    const changed=strainAt(f.input,f.plan);same(changed.strain,r.strain,3e-14);same(changed.jacobian,r.jacobian,3e-13);same(changed.hessian,r.hessian,2e-12);
});

test('continuous bending energy matches the analytic integral for a quadratic centerline and material metric scaling',()=>{
    const bend=.08,EI=2.7,f=fixture({bend,step:0}),F=x=>EI*bend/(2*f.input.dsDx)*(Math.atan(2*bend*x)+(2*bend*x)/(1+(2*bend*x)**2)),
        edge=elastic({frame:f.plan,dsDx:f.input.dsDx,material:{EI1:EI,EI2:4,GJ:1}}),r=edge.evaluate(f.input);
    close(r.energy,F(2)-F(1),2e-12);assert.ok(r.quadrature.converged);assert.equal(r.quadrature.rigorousErrorBound,false);
    const other=elastic({frame:f.plan,dsDx:2*f.input.dsDx,material:{EI1:EI,EI2:4,GJ:1}}).evaluate(f.input);
    close(other.energy,.5*r.energy,1e-12);same(other.gradient,Array.from(r.gradient,v=>.5*v),1e-10);
    const rest=elastic({frame:f.plan,dsDx:f.input.dsDx,materialAt:({coordinate:x})=>({EI1:EI,EI2:4,GJ:1,intrinsic:[2*bend/(1+(2*bend*x)**2)/f.input.dsDx,0,0]})}).evaluate(f.input);
    close(rest.energy,0,1e-25);rest.gradient.forEach(v=>close(v,0,2e-13));
});

test('integrated exact elastic forces/tangents match energy and gradient finite differences including geometric stiffness',()=>{
    const f=fixture({count:4,bend:.025}),edge=elastic({frame:f.plan,dsDx:f.input.dsDx,material,quadrature:{energy:1e-10,gradient:1e-9,hessian:1e-8}}),
        r=structuredClone(edge.evaluate(f.input)),N=r.configurationDofs,h=1e-6;
    for(let j=0;j<N;j++) {
        const plus=structuredClone(f.input),minus=structuredClone(f.input);perturb(plus,j,h);perturb(minus,j,-h);
        const a=structuredClone(edge.evaluate(plus,{order:'gradient'})),b=edge.evaluate(minus,{order:'gradient'});
        close((a.energy-b.energy)/(2*h),r.gradient[j],2e-8);
        for(let i=0;i<N;i++)close((a.gradient[i]-b.gradient[i])/(2*h),r.hessian[i*N+j],2e-7);
    }
    const net=[0,1,2].map(k=>Array.from({length:f.input.positions.length},(_,j)=>r.gradient[3*j+k]).reduce((s,v)=>s+v,0));same(net,[0,0,0],2e-14);
});

test('quadrature splits declared material discontinuities and never returns partial force or a stale Hessian on failure',()=>{
    const f=fixture({bend:0,step:0}),split=1.27,edge=elastic({frame:f.plan,dsDx:f.input.dsDx,materialBreaks:[split],
        materialAt:({coordinate})=>({EI1:2,GJ:1,energyOffset:coordinate<split?2:5})}),r=edge.evaluate(f.input);
    close(r.energy,f.input.dsDx*(.27*2+.73*5),1e-13);
    const poisoned=structuredClone(f.input);poisoned.positions[0][0]=NaN;assert.throws(()=>edge.evaluate(poisoned),/finite/);assert.equal(r.operatorReady,false);assert.ok(r.hessian.every(Number.isNaN));
    const retry=edge.evaluate(f.input,{order:'gradient'});assert.equal(retry.hessianValid,false);assert.ok(retry.hessian.every(Number.isNaN));
    close(edge.evaluate(f.input).energy,f.input.dsDx*(.27*2+.73*5),1e-13);
    const short=elastic({frame:f.plan,dsDx:f.input.dsDx,material,quadrature:{maxEvaluations:1}});
    assert.throws(()=>short.evaluate(f.input),e=>e.code==='continuous-elastic-quadrature-budget');
    const curved=fixture({bend:.2,step:.3}),strict=elastic({frame:curved.plan,dsDx:1,material,quadrature:{energy:1e-30,gradient:1e-30,hessian:1e-30,maxDepth:0}});
    assert.throws(()=>strict.evaluate(curved.input),e=>e.code==='continuous-elastic-quadrature-budget');
});

import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeContinuousGeometry as geometry,evaluateCompositeContinuousGeometry as curve} from '../src/physics/kirchhoffCompositeContinuousGeometry.js';
import {captureCompositeReferenceFrames} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeContinuousFrame as prepare,createCompositeContinuousFrameWorkspace as workspace,evaluateCompositeContinuousFrame as evaluate} from '../src/physics/kirchhoffCompositeContinuousFrame.js';
import {evaluateCompositeContinuousSurfaceMotion as surface} from '../src/physics/kirchhoffCompositeContinuousSurfaceMotion.js';

const add=(a,b)=>a.map((v,k)=>v+b[k]),sub=(a,b)=>a.map((v,k)=>v-b[k]),scale=(a,s)=>a.map(v=>v*s),
    dot=(a,b)=>a.reduce((s,v,k)=>s+v*b[k],0),cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],unit=a=>scale(a,1/Math.hypot(...a));
const close=(a,b,t=1e-9)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b} (${t})`);
const same=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,k)=>close(v,b[k],t));};
function rotate(v,axis,angle){const a=unit(axis),c=Math.cos(angle),s=Math.sin(angle);return add(add(scale(v,c),scale(cross(a,v),s)),scale(a,dot(a,v)*(1-c)));}
function transport(d,a,b){const v=cross(a,b),w=cross(v,d);return add(add(d,w),scale(cross(v,w),1/(1+dot(a,b))));}
const phase=(a,b,t)=>Math.atan2(dot(t,cross(a,b)),dot(a,b));
function fixture({straight=false,edge=1,toolId='wire',x=[0,1,2.3,3.5,5],angleStep=.07}={}) {
    const g=geometry({coordinates:x}),old=x.map(s=>[s,straight?0:.04*s*s,straight?0:.003*s**3]),
        source={geometry:g,edge,toolId,previousPositions:old,previousAngles:x.slice(1).map((_,j)=>angleStep*j),
            reference:captureCompositeReferenceFrames(old,[0,1,0]),referenceTwists:new Array(x.length-2).fill(0)},
        plan=prepare(source),current=old.map((v,j)=>v.map((x,k)=>x+(straight?0:.001*(j+1)*(k+1)))),
        angles=source.previousAngles.map((v,j)=>v+(straight?0:.003*(j+1))),
        input={positions:plan.positionNodeIndices.map(j=>current[j]),angles:plan.angleEdgeIndices.map(j=>angles[j]),
            coordinate:x[edge]+.37*(x[edge+1]-x[edge]),positionRates:plan.positionNodeIndices.map(j=>[.03*j,.02,-.01*j]),
            angleRates:plan.angleEdgeIndices.map(j=>.2-.03*j),materialMap:{sStart:20+x[edge],dsDx:1.2,dsDt:[-.3,-.1]},
            contact:{point:[1.5,.3,-.1],axes:[[1,0,0],[0,0,1]]},wall:{velocity:[.01,-.02,.03]}};
    return {g,source,plan,input,current,angles,scratch:workspace(plan)};
}
function perturb(input,j,h,N) {
    if(j===N){input.coordinate+=h;return;}
    const count=3*input.positions.length;
    if(j<count)input.positions[Math.floor(j/3)][j%3]+=h;else input.angles[j-count]+=h;
}

test('continuous physical directors and material surface rates match across artificial geometry and angle-support boundaries',()=>{
    const f=fixture(),x=f.g.coordinates,at=x[2];
    function atEdge(edge,coordinate) {
        const plan=prepare({...f.source,edge});
        return surface({...f.input,positions:plan.positionNodeIndices.map(j=>f.current[j]),angles:plan.angleEdgeIndices.map(j=>f.angles[j]),coordinate,
            positionRates:plan.positionNodeIndices.map(j=>[.03*j,.02,-.01*j]),angleRates:plan.angleEdgeIndices.map(j=>.2-.03*j),
            materialMap:{sStart:20+x[edge],dsDx:1.2,dsDt:-.3}},plan);
    }
    const left=atEdge(1,at),right=atEdge(2,at);
    for(const key of ['position','velocity','omega','surfaceVelocity','slipRate'])same(left[key],right[key],5e-11);
    left.directors.forEach((d,j)=>same(d,right.directors[j],2e-13));
    const midpoint=x[1]+(x[2]-x[1])/2,h=1e-7,a=atEdge(1,midpoint-h),b=atEdge(1,midpoint+h);
    same(a.omega,b.omega,1e-6);same(a.surfaceVelocity,b.surfaceVelocity,1e-6);
    const shape=curve(f.g.edges[1],{positions:f.g.edges[1].nodeIndices.map(j=>f.current[j]),fraction:.37});
    same(evaluate(f.input,f.plan,f.scratch).position,shape.position,3e-14);
});

test('all frame and angular-rate-map columns, including physical coordinate derivatives, match independent finite differences',context=>{
    const f=fixture(),r=evaluate(f.input,f.plan,f.scratch),N=r.configurationDofs,D=N+1,h=1e-6;let max=0;
    for(let j=0;j<D;j++) {
        const plus=structuredClone(f.input),minus=structuredClone(f.input);perturb(plus,j,h,N);perturb(minus,j,-h,N);
        const a=evaluate({...plus,order:'value'},f.plan,f.scratch),b=evaluate({...minus,order:'value'},f.plan,f.scratch);
        for(let axis=0;axis<9;axis++)close((a.directors.flat()[axis]-b.directors.flat()[axis])/(2*h),r.frameJacobian[axis*D+j],2e-8);
        for(let entry=0;entry<3*D;entry++) {
            const error=Math.abs((a.angularRateMap[entry]-b.angularRateMap[entry])/(2*h)-r.angularRateMapDerivative[entry*D+j]);
            max=Math.max(max,error);assert.ok(error<3e-7,`angular derivative ${entry},${j}: ${error}`);
        }
    }
    context.diagnostic(JSON.stringify({configurationDofs:N,maxAngularDerivativeError:max,operations:r.operations,arenaBytes:r.workspaceBytes}));
});

test('surface B and its complete configuration/foot/point derivatives match rate variations and finite differences',()=>{
    const f=fixture(),r=surface(f.input,f.plan,f.scratch),N=r.configurationDofs,h=1e-6;
    for(let j=0;j<N+4;j++) {
        const plus=structuredClone(f.input),minus=structuredClone(f.input);
        if(j<=N){perturb(plus,j,h,N);perturb(minus,j,-h,N);}else {plus.contact.point[j-N-1]+=h;minus.contact.point[j-N-1]-=h;}
        const a=surface({...plus,order:'value'},f.plan,f.scratch),b=surface({...minus,order:'value'},f.plan,f.scratch);
        for(let entry=0;entry<2*N;entry++)close((a.forceMap[entry]-b.forceMap[entry])/(2*h),j<N?r.configurationDerivative[entry*N+j]:r.queryDerivative[entry*10+j-N],3e-7);
        for(let c=0;c<2;c++)close((a.slipRate[c]-b.slipRate[c])/(2*h),j<N?r.rateConfigurationDerivative[c*N+j]:r.rateQueryDerivative[c*10+j-N],3e-7);
    }
    const force=[.3,-.8],generalized=r.rates.map((_,j)=>dot(force,Array.from(r.forceMap.slice(2*j,2*j+2))));
    close(dot(generalized,r.rates)+dot(force,r.prescribedRate),dot(force,r.slipRate),1e-14);
    for(let j=0;j<N;j++) {
        const plus=structuredClone(f.input),minus=structuredClone(f.input),count=3*plus.positions.length;
        if(j<count){plus.positionRates[Math.floor(j/3)][j%3]+=h;minus.positionRates[Math.floor(j/3)][j%3]-=h;}
        else{plus.angleRates[j-count]+=h;minus.angleRates[j-count]-=h;}
        const a=surface({...plus,order:'value'},f.plan,f.scratch),b=surface({...minus,order:'value'},f.plan,f.scratch);
        for(let c=0;c<2;c++)close((a.slipRate[c]-b.slipRate[c])/(2*h),r.forceMap[2*j+c],4e-9);
    }
});

test('opposite tools retain different levers, feeds and spins and give conjugate equal/opposite contact work',()=>{
    const w=fixture({straight:true}),c=fixture({straight:true,toolId:'catheter'});c.source.previousPositions.forEach(p=>p[1]-=.4);
    c.plan=prepare(c.source);c.input.positions=c.plan.positionNodeIndices.map(j=>c.source.previousPositions[j].slice());c.scratch=workspace(c.plan);
    w.input.materialMap.dsDt=-.6;c.input.materialMap.dsDt=.3;w.input.angleRates.fill(2);c.input.angleRates.fill(-3);
    const a=surface(w.input,w.plan,w.scratch),b=surface(c.input,c.plan,c.scratch),Ft=[.4,-.2],
        rates=[...a.rates,...b.rates],B=[...a.forceMap,...Array.from(b.forceMap,v=>-v)],slip=sub(a.slipRate,b.slipRate),prescribed=sub(a.prescribedRate,b.prescribedRate),
        loads=rates.map((_,j)=>Ft[0]*B[2*j]+Ft[1]*B[2*j+1]);
    assert.notDeepEqual(a.lever,b.lever);assert.notDeepEqual(a.omega,b.omega);
    close(dot(loads,rates)+dot(Ft,prescribed),dot(Ft,slip),3e-14);
});

test('common instantaneous rigid motion yields the same physical velocity at the world witness for two curved tools',()=>{
    const omega=[.4,-.3,.7],translation=[.2,-.1,.05],point=[1.5,.3,-.1],responses=[];
    for(const toolId of ['wire','catheter']) {
        const f=fixture({toolId});if(toolId==='catheter')f.source.previousPositions.forEach(p=>p[1]-=.4);
        f.plan=prepare(f.source);f.scratch=workspace(f.plan);
        const positions=f.plan.positionNodeIndices.map(j=>f.source.previousPositions[j]),angles=f.plan.angleEdgeIndices.map(j=>f.source.previousAngles[j]),
            input={...f.input,positions,angles,positionRates:positions.map(p=>add(translation,cross(omega,p))),
                angleRates:f.plan.angleEdgeIndices.map(j=>dot(omega,f.source.reference[j].tangent)),materialMap:{...f.input.materialMap,dsDt:0},wall:{velocity:[0,0,0]}};
        const r=surface(input,f.plan,f.scratch);same(r.omega,omega,3e-13);same(r.surfaceVelocity,add(translation,cross(omega,point)),3e-13);responses.push(r);
    }
    same(responses[0].slipRate,responses[1].slipRate,4e-13);
});

test('independent native frame gauges and large common unwrapped angle lifts leave physical fields and force maps unchanged',()=>{
    const f=fixture(),r=surface(f.input,f.plan,f.scratch),gauges=f.source.previousAngles.map((_,j)=>(j%2?1:-1)*2*Math.PI+.11*j);
    f.source.reference.forEach((ref,j)=>ref.director=rotate(ref.director,ref.tangent,gauges[j]));
    f.source.previousAngles=f.source.previousAngles.map((v,j)=>v-gauges[j]);f.source.referenceTwists=f.source.referenceTwists.map((v,j)=>v+gauges[j+1]-gauges[j]);
    f.input.angles=f.plan.angleEdgeIndices.map((j,k)=>f.input.angles[k]-gauges[j]);f.plan=prepare(f.source);f.scratch=workspace(f.plan);
    const changed=surface(f.input,f.plan,f.scratch);
    for(const key of ['forceMap','configurationDerivative','slipRate','omega'])same(changed[key],r[key],4e-11);
    const s=fixture({straight:true,angleStep:0});s.input.positions=s.plan.positionNodeIndices.map(j=>s.source.previousPositions[j]);s.input.angles.fill(4*Math.PI);
    delete s.input.positionRates;delete s.input.angleRates;s.input.rateMode='backward-euler-grid';s.input.dt=.1;s.input.materialMap.dsDt=0;
    same(surface(s.input,s.plan,s.scratch).omega,[40*Math.PI,0,0],2e-12);
});

test('resolved spatial winding exceeds a full revolution without being replaced by modulo angles; unresolved intervals request refinement',()=>{
    const x=Array.from({length:11},(_,j)=>j),f=fixture({straight:true,x,angleStep:.7}),plans=x.slice(1).map((_,edge)=>prepare({...f.source,edge}));
    let last=0,unwrapped=0;
    for(let i=0;i<=180;i++) {
        const coordinate=.5+9*i/180,edge=Math.min(9,Math.floor(coordinate)),plan=plans[edge],
            r=evaluate({positions:plan.positionNodeIndices.map(j=>f.source.previousPositions[j]),angles:plan.angleEdgeIndices.map(j=>f.source.previousAngles[j]),coordinate,order:'value'},plan),
            raw=Math.atan2(r.directors[0][2],r.directors[0][1]),value=raw+2*Math.PI*Math.round((last-raw)/(2*Math.PI));
        assert.ok(value>=last-1e-12);last=unwrapped=value;
    }
    close(unwrapped,6.3,3e-13);
    const bad=fixture();bad.input.angles[1]+=2*Math.PI;
    assert.throws(()=>evaluate(bad.input,bad.plan,bad.scratch),e=>e.code==='continuous-frame-refinement-required'&&e.details.reason==='unresolved-current-spatial-winding');
    bad.source.previousAngles[1]+=2*Math.PI;assert.throws(()=>prepare(bad.source),e=>e.details?.reason==='unresolved-accepted-spatial-winding');
});

test('observer rotation and finite common body rotation preserve the physical material frame reconstruction',()=>{
    const f=fixture(),original=evaluate(f.input,f.plan,f.scratch),axis=unit([.3,-.2,.8]),angle=.73,shift=[.4,-.1,.7],R=v=>rotate(v,axis,angle),point=v=>add(R(v),shift),observer=fixture();
    observer.source.previousPositions=observer.source.previousPositions.map(point);observer.source.reference.forEach(r=>{r.tangent=R(r.tangent);r.director=R(r.director);});
    observer.input.positions=observer.input.positions.map(point);observer.plan=prepare(observer.source);
    const changed=evaluate(observer.input,observer.plan);original.directors.forEach((d,j)=>same(changed.directors[j],R(d),5e-14));same(changed.position,point(original.position),4e-14);
    const body=fixture(),old=evaluate({...body.input,positions:body.plan.positionNodeIndices.map(j=>body.source.previousPositions[j]),angles:body.plan.angleEdgeIndices.map(j=>body.source.previousAngles[j])},body.plan,body.scratch);
    body.input.positions=body.plan.positionNodeIndices.map(j=>point(body.source.previousPositions[j]));
    body.input.angles=body.plan.angleEdgeIndices.map(j=>{
        const ref=body.source.reference[j],t=R(ref.tangent),carried=transport(ref.director,ref.tangent,t);
        return body.source.previousAngles[j]+phase(carried,R(ref.director),t);
    });
    const moved=evaluate(body.input,body.plan,body.scratch);old.directors.forEach((d,j)=>same(moved.directors[j],R(d),4e-14));
});

test('backward Euler rate derivatives include the physical B/dt term and value mode keeps exact B without second derivatives',()=>{
    const f=fixture();f.input.rateMode='backward-euler-grid';f.input.dt=.02;delete f.input.positionRates;delete f.input.angleRates;
    const r=surface(f.input,f.plan,f.scratch),N=r.configurationDofs,h=1e-6;
    for(let j=0;j<N;j++) {
        const plus=structuredClone(f.input),minus=structuredClone(f.input);perturb(plus,j,h,N);perturb(minus,j,-h,N);
        const a=surface({...plus,order:'value'},f.plan,f.scratch),b=surface({...minus,order:'value'},f.plan,f.scratch);
        for(let c=0;c<2;c++)close((a.slipRate[c]-b.slipRate[c])/(2*h),r.rateConfigurationDerivative[c*N+j],3e-7);
    }
    const value=surface({...f.input,order:'value'},f.plan,f.scratch);assert.deepEqual(value.forceMap,r.forceMap);assert.deepEqual(value.slipRate,r.slipRate);
    assert.equal(value.derivativeValid,false);assert.equal(value.configurationDerivative,null);assert.equal(value.finiteStepSlipKnown,false);
});

test('prepared sources and returned derivatives are owned; failures do not poison later frame evaluations',()=>{
    const f=fixture(),r=evaluate(f.input,f.plan,f.scratch),before=structuredClone(f.input);
    f.source.previousPositions[0][0]=99;f.source.reference[0].director[0]=99;f.source.previousAngles[0]=99;f.source.referenceTwists[0]=99;
    assert.deepEqual(evaluate(f.input,f.plan,f.scratch).angularRateMap,r.angularRateMap);
    const bad=structuredClone(f.input);bad.positions[0][0]=NaN;assert.throws(()=>evaluate(bad,f.plan,f.scratch),/finite/);
    assert.deepEqual(evaluate(f.input,f.plan,f.scratch).angularRateMapDerivative,r.angularRateMapDerivative);assert.deepEqual(f.input,before);
    r.frameJacobian.fill(999);assert.notDeepEqual(evaluate(f.input,f.plan,f.scratch).frameJacobian,r.frameJacobian);
    assert.throws(()=>evaluate(f.input,{...f.plan},f.scratch),/workspace/);assert.throws(()=>evaluate({...f.input,coordinate:-1},f.plan,f.scratch),/leaves/);
    assert.throws(()=>surface({...f.input,rateMode:'unknown'},f.plan,f.scratch),/rates/);
});

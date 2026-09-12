import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeContinuousGeometry,evaluateCompositeContinuousGeometry,sampleCompositeContinuousBasis} from '../src/physics/kirchhoffCompositeContinuousGeometry.js';
import {captureCompositeReferenceFrames} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeContinuousFrame} from '../src/physics/kirchhoffCompositeContinuousFrame.js';
import {createCompositeContinuousWallSurface as create,evaluateCompositeContinuousWallSurface as evaluate} from '../src/physics/kirchhoffCompositeContinuousWallSurface.js';

const dot=(a,b)=>a.reduce((sum,v,k)=>sum+v*b[k],0),unit=v=>v.map(x=>x/Math.hypot(...v)),
    close=(a,b,t=2e-8)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b} (${t})`),
    same=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,j)=>close(v,b[j],t));};
function fixture(kind='sphere') {
    const coordinates=Array.from({length:10},(_,j)=>.7*j+.013*j*j),geometry=createCompositeContinuousGeometry({coordinates}),edge=4,fraction=.39,
        previousPositions=coordinates.map(x=>[x,.021*x*x,.002*x*x*x]),previousAngles=coordinates.slice(1).map((_,j)=>.043*j),
        frame=createCompositeContinuousFrame({geometry,edge,toolId:'wire',previousPositions,previousAngles,
            reference:captureCompositeReferenceFrames(previousPositions,[0,1,0]),referenceTwists:new Array(coordinates.length-2).fill(0)}),
        own=geometry.edges[edge],basis=sampleCompositeContinuousBasis(own,fraction),
        site={owner:'wire',edge,fraction,nodeIndices:own.nodeIndices.slice(),positionWeights:basis.weights.slice(),supported:true,hessianValid:true,
            position:[0,0,0],normal:[0,0,0],closestPoint:[0,0,0],signedDistance:0,pointGapGradient:[0,0,0],pointNormalDerivative:new Array(9).fill(0)},
        input={positions:frame.positionNodeIndices.map(j=>previousPositions[j].map((v,k)=>v+.0012*(j+1)*(k+1))),
            angles:frame.angleEdgeIndices.map(j=>previousAngles[j]+.002*(j+1)),materialMap:{sStart:17,dsDx:1.13,dsDt:[-.2,.1]},dt:.02};
    function refresh(pose) {
        const positions=own.nodeIndices.map(node=>pose.positions[frame.positionNodeIndices.indexOf(node)]),point=evaluateCompositeContinuousGeometry(own,{positions,fraction}).position;
        let normal,distance,gradient,derivative;
        if(kind==='plane') {
            normal=unit([.1,.6,.8]);distance=.7+dot(normal,point);gradient=normal.slice();derivative=new Array(9).fill(0);
        } else {
            const radial=point.map((v,k)=>v-[3,-2,1][k]),r=Math.hypot(...radial),direction=radial.map(v=>v/r),factor=kind==='scaled-sphere'?1.7:1;
            normal=direction.map(v=>-v);distance=factor*(5-r);gradient=normal.map(v=>factor*v);
            derivative=Array.from({length:9},(_,j)=>-((Math.floor(j/3)===j%3?1:0)-direction[Math.floor(j/3)]*direction[j%3])/r);
        }
        Object.assign(site,{position:Array.from(point),normal,closestPoint:point.map((v,k)=>v-distance*normal[k]),signedDistance:distance,
            gap:distance-.1,pointGapGradient:gradient,pointNormalDerivative:derivative,supported:true,hessianValid:true});
    }
    refresh(input);const plan=create({frame,site,seedAxis:[.3,.8,-.2],wallVelocity:[.01,-.02,.03]});
    return {frame,site,input,plan,refresh};
}
function perturb(input,j,h){const count=input.positions.length*3;if(j<count)input.positions[Math.floor(j/3)][j%3]+=h;else input.angles[j-count]+=h;}

for(const kind of ['plane','sphere','scaled-sphere'])test(`C2 ${kind} wall B, DB and implicit slip include the complete moving witness and tangent chart`,context=>{
    const f=fixture(kind),r=evaluate(f.input,f.plan),N=r.configurationDofs,h=1e-6;let maxB=0,maxSlip=0,maxQuery=0;
    assert.equal(N,29);assert.equal(f.frame.positionNodeIndices.length,8);assert.equal(f.frame.angleEdgeIndices.length,5);
    for(let l=0;l<N;l++) {
        const plus=structuredClone(f.input),minus=structuredClone(f.input);perturb(plus,l,h);perturb(minus,l,-h);
        f.refresh(plus);const a=evaluate({...plus,order:'gradient'},f.plan);f.refresh(minus);const b=evaluate({...minus,order:'gradient'},f.plan);
        for(let row=0;row<2*N;row++) {
            const fd=(a.forceMap[row]-b.forceMap[row])/(2*h),actual=r.configurationDerivative[row*N+l];maxB=Math.max(maxB,Math.abs(fd-actual));close(fd,actual,8e-7);
        }
        for(let c=0;c<2;c++) {
            const fd=(a.slipIncrement[c]-b.slipIncrement[c])/(2*h),actual=r.slipDerivative[c*N+l];maxSlip=Math.max(maxSlip,Math.abs(fd-actual));close(fd,actual,4e-8);
        }
        const qa=[0,...a.contactPoint,...a.axes.flat()],qb=[0,...b.contactPoint,...b.axes.flat()];
        qa.forEach((v,q)=>{const fd=(v-qb[q])/(2*h),actual=r.queryConfigurationDerivative[q*N+l];maxQuery=Math.max(maxQuery,Math.abs(fd-actual));close(fd,actual,4e-8);});
    }
    const remote=f.frame.configurationColumns.findIndex(c=>c.kind==='position'&&!f.site.nodeIndices.includes(c.node));assert.ok(remote>=0);
    assert.ok(Array.from(r.forceMap.slice(2*remote,2*remote+6)).some(v=>Math.abs(v)>1e-8),'Remote physical frame support must retain real reaction columns');
    assert.equal(r.finiteStepSlipKnown,false);assert.equal(r.contactCertified,false);assert.equal(r.slipModel,'implicit-backward-euler-surface-rate');
    context.diagnostic(JSON.stringify({configurationDofs:N,maxB,maxSlip,maxQuery}));
});

test('wall reaction has exact conjugate power with independent feed, spin, moving wall and backward Euler grid rates',()=>{
    const f=fixture(),r=evaluate(f.input,f.plan),Ft=[.3,-.7],force=r.rates.map((_,j)=>-dot(Ft,Array.from(r.forceMap.slice(2*j,2*j+2))));
    close(dot(force,r.rates)-dot(Ft,r.prescribedRate),-dot(Ft,r.slipRate),1e-13);
    same(r.slipIncrement,r.slipRate.map(v=>f.input.dt*v),0);
    f.site.hessianValid=false;f.site.pointNormalDerivative.fill(NaN);f.site.pointGapGradient.fill(NaN);
    const gradient=evaluate({...f.input,order:'gradient'},f.plan);
    assert.deepEqual(gradient.forceMap,r.forceMap);assert.deepEqual(gradient.slipIncrement,r.slipIncrement);
    assert.equal(gradient.configurationDerivative,null);assert.equal(gradient.slipDerivative,null);assert.equal(gradient.derivativeValid,false);
    assert.throws(()=>evaluate(f.input,f.plan),/matching derivative order/);
});

test('wall surface rejects stale queries, changed support and singular charts without poisoning later complete derivatives',()=>{
    const f=fixture(),r=evaluate(f.input,f.plan),changed=structuredClone(f.input);
    const local=f.frame.positionNodeIndices.indexOf(f.site.nodeIndices[1]);changed.positions[local][0]+=.01;
    assert.throws(()=>evaluate(changed,f.plan),/stale/);
    f.site.closestPoint[0]+=.01;assert.throws(()=>evaluate(f.input,f.plan),/witness/);f.refresh(f.input);
    f.site.supported=false;assert.throws(()=>evaluate(f.input,f.plan),/supported wall query/);f.refresh(f.input);
    f.site.positionWeights[0]+=.01;assert.throws(()=>evaluate(f.input,f.plan),/support changed/);f.site.positionWeights[0]-=.01;
    // Restore exact coefficients: subtracting need not undo the original sum bitwise.
    const fresh=fixture();f.site.positionWeights=fresh.site.positionWeights.slice();
    assert.deepEqual(evaluate(f.input,f.plan).configurationDerivative,r.configurationDerivative);
    const singular=create({frame:f.frame,site:f.site,seedAxis:f.site.normal});assert.throws(()=>evaluate(f.input,singular),/tangent chart/);
    assert.throws(()=>evaluate({...f.input,dt:0},f.plan),/positive dt/);
    assert.throws(()=>evaluate(f.input,{...f.plan}),/compiled/);
    r.forceMap.fill(999);assert.notDeepEqual(evaluate(f.input,f.plan).forceMap,r.forceMap);
});

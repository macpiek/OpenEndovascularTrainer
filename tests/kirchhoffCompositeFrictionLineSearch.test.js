import assert from 'node:assert/strict';
import test from 'node:test';
import {findCompositeFrictionConeCrossing as crossing,proposeCompositeFrictionBacktrack as propose} from '../src/physics/kirchhoffCompositeFrictionLineSearch.js';
import {measureCompositeFriction} from '../src/physics/kirchhoffCompositeFriction.js';
import {advanceCompositeJointTimeStep as advance,createCompositeJointTimeStepWorkspace} from '../src/physics/kirchhoffCompositeJointTimeStep.js';
import {relativeSearchFixture,relativeSearchOptions} from './fixtures/compositeRelativeSearch.js';

const close=(a,b,t=1e-10)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}`);
const point=(Fn,x,y=0)=>({Fn,traction:[x,y]});
test('affine cone exit follows changing normal load, the opposite boundary and a disabled axis without mutating forces',()=>{
    for(const mu of [[.3,.6],[.3,0]]) {
        const start=point(2,.6),end=point(1,-.9),before=structuredClone({start,end,mu});
        close(crossing({start,end,mu}),2/3);assert.deepEqual({start,end,mu},before);
    }
    // Equal normal/tangential increments give A=0: a linear intersection.
    close(crossing({start:point(1,-.5),end:point(.25,-1.25),mu:[1,1]}),1/3);
    assert.equal(crossing({start:point(1,0),end:point(1,0,.1),mu:[1,0]}),null);
});

test('two-axis anisotropic intersections stay on the original force cone over widely different force scales',()=>{
    for(const scale of [1e-150,1,1e150]) {
        const mu=[.2,.7],start=point(2*scale,.01*scale,-.2*scale),end=point(.8*scale,.8*scale,-scale),t=crossing({start,end,mu});
        assert.ok(t>0&&t<1);
        const cone=u=>Math.hypot(...start.traction.map((v,k)=>(v+(end.traction[k]-v)*u)/mu[k]/scale))-(start.Fn+(end.Fn-start.Fn)*u)/scale;
        close(cone(t),0,2e-14);assert.ok(cone(t-1e-5)<0&&cone(t+1e-5)>0);
    }
});

test('no numerical hint invents an exit for open, apex, negative-load or unresolved paths',()=>{
    for(const input of [
        {start:point(1,0),end:point(1,.2),mu:[1,1]},
        {start:point(0,0),end:point(1,1),mu:[1,1]},
        {start:point(-1,0),end:point(1,2),mu:[1,1]},
        {start:point(1,0),end:point(0,2),mu:[1,1]},
        {start:point(1,0),end:point(1,2),mu:[0,0]},
        {start:point(1,0),end:point(1,1e300),mu:[1e-300,1]},
    ])assert.equal(crossing(input),null);
    assert.throws(()=>crossing({start:point(NaN,0),end:point(1,2),mu:[1,1]}),/Finite/);
});

test('backtracking matches the same law and samples, safeguards contraction and combines multiple crossings',()=>{
    const sample=(id,Fn,x,mu=[1,1])=>({...point(Fn,x),sampleId:id,mu});
    const start=[sample('a',1,0),sample('b',1,0)],end=[sample('a',1,2),sample('b',1,4)];
    close(propose(start,end,.5),.125);
    assert.equal(propose(start,[sample('x',1,2),sample('b',1,4,[2,1])],1),null);
    assert.equal(propose([sample('a',1,0)],[sample('a',1,100)],1),null);
    assert.equal(propose(start,[],1),null);
    assert.throws(()=>propose(start,end,0),/trial step/);
});

function verify(f,r) {
    assert.equal(r.accepted,true,JSON.stringify({status:r.status,error:r.error,diagnostics:r.diagnostics}));
    const c=r.diagnostics.certificate,p=r.state.toolPositions;
    assert.ok(c.force<=1e-7&&c.torque<=1e-8&&c.length<=1e-8&&c.boundary<=1e-9);
    assert.equal(c.converged,true);
    for(const s of c.friction.samples) {
        assert.ok(s.Fn>=0);
        if(s.slipRequired===false){assert.equal(s.Fn,0);assert.deepEqual(s.traction,[0,0]);}
        else assert.equal(measureCompositeFriction({traction:s.traction,slip:s.slip,normalForce:s.Fn,mu:s.mu,
            slipTolerance:1e-8,coneTolerance:1e-9,workTolerance:1e-9}).converged,true);
    }
    // Independent straight-segment capsule gap at EVERY declared sample,
    // not the line-search cone or a subset of loaded records.
    let i=0;
    for(const pair of f.contacts.pairs)for(const u of pair.quadrature) {
        const a=p.get('catheter')[pair.outerEdge],b=p.get('catheter')[pair.outerEdge+1],v=b.map((x,k)=>x-a[k]);
        const w=p.get('wire')[pair.innerEdge].map((x,k)=>x+u*(p.get('wire')[pair.innerEdge+1][k]-x)-a[k]);
        const t=w.reduce((sum,x,k)=>sum+x*v[k],0)/v.reduce((sum,x)=>sum+x*x,0);
        assert.ok(t>0&&t<1);
        const gap=pair.lumenRadius-pair.innerRadius-Math.hypot(...w.map((x,k)=>x-t*v[k])),Fn=r.state.lumenContactState.normalForces[i++];
        assert.ok(gap>=-1e-8&&Fn>=0&&Math.abs(Fn*gap)<=1e-9);
    }
    for(const [id,positions] of p)for(let e=0;e<positions.length-1;e++)close(Math.hypot(...positions[e+1].map((v,k)=>v-positions[e][k])),r.state.restLengths.get(id)[e],1e-8);
    const net=[0,0,0];
    for(const balance of r.balances.values())for(let k=0;k<3;k++){
        close(balance.momentumRate[k],balance.appliedForce[k]+balance.boundaryForce[k]+balance.contactForce[k],1e-7);net[k]+=balance.contactForce[k];
    }
    net.forEach(x=>close(x,0,1e-12));
}
function equivalent(a,b) {
    for(const [id,p] of a.state.toolPositions)p.forEach((v,i)=>v.forEach((x,k)=>close(x,b.state.toolPositions.get(id)[i][k],1e-9)));
    for(const [id,angles] of a.state.angles)angles.forEach((x,i)=>close(x,b.state.angles.get(id)[i],1e-9));
    // The two trajectories may stop at different points within the original
    // physical tolerances, especially for two-axis sliding. Compare forces
    // at the original 1e-7 force scale; verify() checks every physical gate.
    for(const [key,field] of [['lumenContactState','normalForces'],['lumenFrictionState','tractions']])a.state[key][field].forEach((x,i)=>close(x,b.state[key][field][i],1e-7));
    assert.equal(a.state.time,b.state.time);assert.equal(a.state.step,b.state.step);
}

test('loaded short/deep shafts cross stick-slide in a bounded common solve while retaining the original physical root',()=>{
    for(const spec of [{n:17,axial:.1,lateral:0,k:5},{n:65,axial:.1,lateral:0,k:5},{n:17,axial:-.1,lateral:0,k:5},
        {n:17,axial:.1,lateral:.02,k:5},{n:17,axial:.1,lateral:0,k:50},{n:17,axial:.1,lateral:0,k:500}]) {
        const f=relativeSearchFixture({n:spec.n,lumen:true,offset:spec.n===17?.04075:.0406,wireSlope:spec.n===17?-.0005:-.0002});
        f.contacts.friction.forcePerLength=spec.k;
        const before=structuredClone(f.state),options=relativeSearchOptions(f,f.state,{force:[spec.axial,.4,spec.lateral],forceNode:0});
        const old=advance(f.state,{...options,globalization:'newton'}),r=advance(f.state,{...options,budget:{directions:4,evaluations:12}});
        verify(f,old);verify(f,r);equivalent(old,r);assert.deepEqual(f.state,before);
        assert.ok(r.diagnostics.frictionConeAccepted>0);assert.ok(r.diagnostics.evaluations<old.diagnostics.evaluations);
        assert.equal(r.diagnostics.numericalDirections,0);
    }
});

test('fresh-certificate budget failure publishes nothing and retry keeps the same histories through the next dt',()=>{
    const f=relativeSearchFixture({n:17,lumen:true,offset:.04075,wireSlope:-.0005}),before=structuredClone(f.state),workspace=createCompositeJointTimeStepWorkspace(f.state);
    const options=relativeSearchOptions(f,f.state,{force:[.1,.4,0],forceNode:0,workspace});
    const failed=advance(f.state,{...options,budget:{evaluations:8}});
    assert.equal(failed.accepted,false);assert.equal(failed.status,'evaluation-budget');assert.equal(failed.state,f.state);assert.deepEqual(f.state,before);
    assert.equal(failed.diagnostics.certificate.converged,true,'Even a converged trial must fund the independent final query');
    const retry=advance(f.state,options),cold=advance(f.state,{...options,workspace:undefined});verify(f,retry);verify(f,cold);assert.deepEqual(retry.state,cold.state);
    const next=relativeSearchOptions(f,retry.state,{force:[.1,.4,0],forceNode:0,workspace});
    const a=advance(retry.state,next),b=advance(cold.state,{...next,workspace:undefined,globalization:'newton'});verify(f,a);verify(f,b);equivalent(a,b);
    assert.equal(a.state.step,2);assert.equal(a.state.time,2/120);
});

test('an open lumen keeps the original trial path, exact zero reactions and every declared normal query',()=>{
    const f=relativeSearchFixture({n:17,lumen:true}),options=relativeSearchOptions(f,f.state),a=advance(f.state,options),b=advance(f.state,{...options,globalization:'newton'});
    verify(f,a);verify(f,b);assert.deepEqual(a.state,b.state);
    for(const key of ['evaluations','directions','contactQueries','acceptedAlphas'])assert.deepEqual(a.diagnostics[key],b.diagnostics[key]);
    assert.equal(a.diagnostics.frictionConeTrials,0);
});

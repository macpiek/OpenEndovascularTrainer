import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createContactResult} from '../src/physics/collision/vesselContactField.js';
import {measureCompositeFriction} from '../src/physics/kirchhoffCompositeFriction.js';
import {createCompositeJointMaterialHistory} from '../src/physics/kirchhoffCompositeJointMaterialHistory.js';
import {createCompositeJointTimeStepState,createCompositeJointTimeStepWorkspace,advanceCompositeJointTimeStep} from '../src/physics/kirchhoffCompositeJointTimeStep.js';

const dt=1/120;
const close=(a,b,t=1e-8)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b} (${t})`);
const same=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const accepted=r=>assert.equal(r.accepted,true,JSON.stringify({status:r.status,error:r.error,diagnostics:r.diagnostics}));
function fixture() {
    const coordinates=[0,2,4],slope=.002,positions=coordinates.map(x=>[x,.16001+slope*x,0]),dsDx=Math.hypot(1,slope);
    const layout=createCompositeChainLayout([['wire'],['wire']]);
    const tools=[{id:'wire',dsDx,reference:captureCompositeReferenceFrames(positions),referenceTwists:new Float64Array(1),
        material:compileCompositeMaterial({stiffness:[[2,0,0],[0,3,0],[0,0,1]],intrinsic:[0,0,0]})}];
    const state=createCompositeJointTimeStepState({layout,coordinates,positions,relative:[],modes:[],
        angles:new Map([['wire',new Float64Array(2)]]),tools,restLengths:new Map([['wire',new Float64Array(2).fill(2*dsDx)]]),materialCoordinate:'reference-arclength'});
    const field={calls:0,queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out=createContactResult()) {
        this.calls++;const t=ay<by?0:1,p=t===0?[ax,ay,az]:[bx,by,bz];
        out.signedDistance=p[1];out.signedGap=p[1]-radius;out.segmentT=t;out.capsuleSampleCount=2;
        out.inward.values.set([0,1,0]);out.closestPoint.values.set([p[0],0,p[2]]);out.source='analytic-plane';out.faceIndex=0;return out;
    }};
    const wall={mode:'wall-coulomb',chartId:'full-wall-friction-dt',contactMode:'capsule',field,forcePerLength:1,
        plane:{normal:[0,1,0],offset:0},contactOwners:{edges:[{edge:0,wall:{owner:'wire',radius:.16,materialSegmentId:'wire:0'}},{edge:1,wall:null}]},
        friction:{law:'coulomb',mu:[.006,.006],forcePerLength:50,materialPath:'linear-affine-maps',motion:'stationary-material',source:'analytic-plane',tangentBasis:'projected-own-tangent'}};
    return {state,wall};
}
function options(f,state,{feed=.3,force=-.4,spin=.03,k=50,workspace,budget}={}) {
    const dsDx=state.tools[0].dsDx;
    const edges=state.layout.edgeToolIds.map((ids,edge)=>({tools:ids.map(id=>({id,massPerMaterialLength:.13,
        materialMap:{sStart:20+state.coordinates[edge]*dsDx+(state.time+dt)*feed,dsDx,dsDt:feed}}))}));
    const history=createCompositeJointMaterialHistory({materialVelocities:state.materialVelocities,
        reservoir:({toolId})=>({id:toolId,sStart:0,sEnd:100,velocities:[[0,0,0],[0,0,0]],interpretation:'physical-material-velocity'})});
    const sampled=history.prepare({coordinates:state.coordinates,inertiaEdges:edges});
    const inertiaEdges=edges.map((entry,edge)=>({tools:entry.tools.map((t,index)=>{
        const h=sampled.inertiaEdges[edge].tools[index];return {...t,...(h.pieces.length===1?{oldMaterialVelocities:h.oldMaterialVelocities}:
            {oldVelocityPieces:h.pieces.map(p=>({fractions:p.fractions,oldMaterialVelocities:p.oldMaterialVelocities}))})};
    })}));
    return {dt,torsionMode:'quasi-static',contacts:'none',wall:{...f.wall,friction:{...f.wall.friction,forcePerLength:k}},workspace,budget,
        inertia:{previousPositions:structuredClone(state.toolPositions),inertiaEdges},
        boundaries:{positions:[{toolId:'wire',node:2,value:f.state.toolPositions.get('wire')[2]}],spins:[{toolId:'wire',edge:0,value:spin*(state.time+dt)}]},
        loads:{forces:[{toolId:'wire',node:0,value:[0,force,0]}],torques:[{toolId:'wire',edge:1,value:.001}]}};
}
const advance=(f,state,extra)=>advanceCompositeJointTimeStep(state,options(f,state,extra));
test('a numerical wall restart preserves incoming material state and must still close both contact laws',()=>{
    const f=fixture(),before=structuredClone(f.state),input=options(f,f.state),
        first=advanceCompositeJointTimeStep(f.state,{...input,budget:{directions:1}});
    assert.equal(first.accepted,false);assert.equal(first.status,'direction-budget');
    const seed=first.diagnostics.numericalRestart;
    const r=advanceCompositeJointTimeStep(f.state,{...input,initialGuess:{...seed.initialGuess,angles:new Map(seed.initialGuess.angles)},initialWallReactions:seed.initialWallReactions});
    accepted(r);verify(f,r);assert.deepEqual(f.state,before);assert.equal(r.state.time,dt);assert.equal(r.state.step,1);
    assert.throws(()=>advanceCompositeJointTimeStep(f.state,{...input,initialWallReactions:{normalForces:[NaN,0,0],tractions:[0,0]}}),/finite/);
});
function verify(f,r) {
    const p=r.state.toolPositions.get('wire'),Fn=r.state.wallContactState.normalForces[0],Ft=Array.from(r.state.wallFrictionState.tractions);
    const normal=r.diagnostics.certificate.wall,friction=r.diagnostics.certificate.wallFriction,s=friction.samples[0];
    assert.equal(normal.converged,true);assert.equal(friction.converged,true);assert.equal(friction.samples.length,1);
    assert.ok(Fn>=0);assert.ok(p[0][1]-.16>=-1e-8);assert.ok(Math.abs(Fn*(p[0][1]-.16))<=1e-9);
    same(s.traction,Ft,1e-14);close(s.Fn,Fn,1e-14);
    assert.equal(measureCompositeFriction({traction:Ft,slip:s.slip,normalForce:Fn,mu:f.wall.friction.mu,
        slipTolerance:1e-8,coneTolerance:1e-9,workTolerance:1e-9}).converged,true);
    for(let edge=0;edge<2;edge++)close(Math.hypot(...p[edge+1].map((v,k)=>v-p[edge][k])),r.state.restLengths.get('wire')[edge]);
    const tangent=[p[1][0]-p[0][0],0,p[1][2]-p[0][2]],length=Math.hypot(...tangent),axial=tangent.map(v=>v/length),circ=[-axial[2],0,axial[0]];
    const contact=axial.map((v,k)=>Ft[0]*v+Ft[1]*circ[k]+(k===1?Fn:0)),balance=r.balances.get('wire');
    same(balance.contactForce,contact,1e-12);same(balance.momentumRate,balance.appliedForce.map((v,k)=>v+balance.boundaryForce[k]+contact[k]),1e-7);
    assert.equal(r.diagnostics.contactQueries,r.diagnostics.evaluations);assert.equal(r.diagnostics.wallQueries,r.diagnostics.evaluations);
    assert.equal(r.diagnostics.wallFrictionRows,2);return s;
}

test('full joint dt solves stationary vessel friction with own feed, spin and physical external reaction',()=>{
    const f=fixture(),before=structuredClone(f.state),r=advance(f,f.state);accepted(r);const s=verify(f,r);
    assert.deepEqual(f.state,before);assert.ok(s.Fn>0);assert.ok(s.slip[0]<-1e-4);assert.ok(s.traction[0]>0);assert.ok(Math.abs(s.traction[1])>1e-9);
    close(r.state.time,dt,0);assert.equal(r.state.step,1);
});

test('second wall-friction dt retains own material history and cannot discard loaded wall traction',()=>{
    const f=fixture(),a=advance(f,f.state);accepted(a);verify(f,a);const before=structuredClone(a.state),b=advance(f,a.state);accepted(b);verify(f,b);
    assert.deepEqual(a.state,before);close(b.state.time,2*dt,0);assert.equal(b.state.step,2);
    assert.throws(()=>advanceCompositeJointTimeStep(b.state,{...options(f,b.state),wall:{...f.wall,mode:'wall-normal',friction:'none'}}),/cannot discard/);
});

test('wall Coulomb roots are independent of numerical penalty and match cold/reused complete dt',()=>{
    const f=fixture(),workspace=createCompositeJointTimeStepWorkspace(f.state),base=advance(f,f.state);accepted(base);
    for(const k of [5,50,500]){
        const a=advance(f,f.state,{k}),b=advance(f,f.state,{k,workspace});accepted(a);accepted(b);verify(f,b);assert.deepEqual(a.state,b.state);
        same(a.state.positions.flat(),base.state.positions.flat(),1e-8);same(Array.from(a.state.wallFrictionState.tractions),Array.from(base.state.wallFrictionState.tractions),1e-8);
    }
});

test('late wall-friction failure preserves state, winding, traction and time; retry matches cold',()=>{
    const f=fixture(),workspace=createCompositeJointTimeStepWorkspace(f.state),a=advance(f,f.state,{workspace});accepted(a);
    const cold=advance(f,a.state);accepted(cold);const before=structuredClone(a.state);
    for(const budget of [{evaluations:cold.diagnostics.evaluations-1},{contactQueries:cold.diagnostics.contactQueries-1}]){
        const failed=advance(f,a.state,{workspace,budget});assert.equal(failed.accepted,false);assert.deepEqual(a.state,before);assert.equal(failed.state,a.state);
        const retry=advance(f,a.state,{workspace});accepted(retry);verify(f,retry);assert.deepEqual(retry.state,cold.state);
    }
});

test('unknown incoming surface orientation beyond the proximal material endpoint cannot be silently extrapolated',()=>{
    const f=fixture(),before=structuredClone(f.state),r=advance(f,f.state,{feed:-.3});
    assert.equal(r.accepted,false);assert.equal(r.status,'surface-material-transport-required');
    assert.equal(r.state,f.state);assert.deepEqual(f.state,before);assert.equal(r.state.time,0);
});

test('vessel friction unloads through signed private normal trials and can then be explicitly disabled',()=>{
    const f=fixture(),workspace=createCompositeJointTimeStepWorkspace(f.state);let state=f.state,r,minPrivate=Infinity;
    for(const force of [-.4,-.4,0,.4]) {r=advance(f,state,{workspace,force});accepted(r);verify(f,r);minPrivate=Math.min(minPrivate,r.diagnostics.minimumPrivateNormalForce);state=r.state;}
    assert.ok(state.wallContactState.normalForces.every(v=>v===0));assert.ok(state.wallFrictionState.tractions.every(v=>v===0));
    assert.ok(minPrivate<0);assert.ok(r.diagnostics.certificate.wall.minGap>0);
    const clear=advanceCompositeJointTimeStep(state,{...options(f,state,{force:.4}),wall:{...f.wall,mode:'wall-normal',friction:'none'}});
    accepted(clear);assert.equal(clear.state.wallFrictionState,undefined);
    assert.equal(clear.state.wallContactState.preserveSampleReactions,true);
    const next=advanceCompositeJointTimeStep(clear.state,{...options(f,clear.state,{force:.4}),wall:{...f.wall,mode:'wall-normal',friction:'none'}});
    accepted(next);assert.equal(next.state.wallFrictionState,undefined);
    assert.equal(next.state.wallContactState.preserveSampleReactions,true);
});

test('a physically disabled Coulomb axis has literal zero traction while the other axis retains its own law',()=>{
    for(const mu of [[0,.006],[.006,0],[0,0]]) {
        const f=fixture();f.wall.friction.mu=mu;const a=advance(f,f.state);accepted(a);verify(f,a);
        const b=advance(f,a.state);accepted(b);verify(f,b);
        assert.ok(b.state.wallContactState.normalForces[0]>0);
        for(let c=0;c<2;c++)if(mu[c]===0)assert.equal(b.state.wallFrictionState.tractions[c],0);
        else assert.ok(Math.abs(b.state.wallFrictionState.tractions[c])>1e-9);
    }
});

// Two exposed surfaces in the same common/relative system. This isolates wall
// material ownership; lumen coupling is covered by JointFrictionTimeStep.
function twoOwnerFixture() {
    const f=fixture(),coordinates=[0,2,4],layout=createCompositeChainLayout([['wire','catheter'],['wire','catheter']]);
    const points=new Map([['wire',coordinates.map(x=>[x,.16001+.000002*x,2])],['catheter',coordinates.map(x=>[x,.80001+.002*x,0])]]);
    const tools=[...points].map(([id,p])=>({id,dsDx:Math.hypot(...p[1].map((v,k)=>(v-p[0][k])/2)),
        reference:captureCompositeReferenceFrames(p),referenceTwists:new Float64Array(1),
        material:compileCompositeMaterial({stiffness:[[2,0,0],[0,3,0],[0,0,1]],intrinsic:[0,0,0]})}));
    const cat=points.get('catheter'),wire=points.get('wire');
    f.state=createCompositeJointTimeStepState({layout,coordinates,positions:cat,
        relative:wire.flatMap((p,i)=>p.map((v,k)=>v-cat[i][k])),modes:coordinates.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]})),
        angles:new Map(tools.map(t=>[t.id,new Float64Array(2)])),tools,
        restLengths:new Map(tools.map(t=>[t.id,new Float64Array(2).fill(2*t.dsDx)])),materialCoordinate:'reference-arclength'});
    f.wall.contactOwners.edges=[{edge:0,wall:{owner:'catheter',radius:.8,materialSegmentId:'catheter:0'}},
        {edge:1,wall:{owner:'wire',radius:.16,materialSegmentId:'wire:1'}}];
    delete f.wall.friction.mu;
    f.wall.friction.muByOwner=[{owner:'wire',mu:[.006,.004]},{owner:'catheter',mu:[.002,.001]}];
    return f;
}
function twoOwnerOptions(f,state,extra={}) {
    const value=options(f,state,extra),rates={wire:.3,catheter:.15},byId=new Map(state.tools.map(t=>[t.id,t]));
    const edges=state.layout.edgeToolIds.map((ids,edge)=>({tools:ids.map(id=>({id,massPerMaterialLength:.13,
        materialMap:{sStart:20+state.coordinates[edge]*byId.get(id).dsDx+(state.time+dt)*rates[id],dsDx:byId.get(id).dsDx,dsDt:rates[id]}}))}));
    const history=createCompositeJointMaterialHistory({materialVelocities:state.materialVelocities,
        reservoir:({toolId})=>({id:toolId,sStart:0,sEnd:100,velocities:[[0,0,0],[0,0,0]],interpretation:'physical-material-velocity'})});
    const sampled=history.prepare({coordinates:state.coordinates,inertiaEdges:edges});
    value.inertia.inertiaEdges=edges.map((entry,edge)=>({tools:entry.tools.map((t,index)=>{
        const h=sampled.inertiaEdges[edge].tools[index];return {...t,...(h.pieces.length===1?{oldMaterialVelocities:h.oldMaterialVelocities}:
            {oldVelocityPieces:h.pieces.map(p=>({fractions:p.fractions,oldMaterialVelocities:p.oldMaterialVelocities}))})};
    })}));
    value.boundaries={positions:state.tools.map(t=>({toolId:t.id,node:2,value:f.state.toolPositions.get(t.id)[2]})),
        spins:state.tools.map(t=>({toolId:t.id,edge:0,value:(t.id==='wire'?.03:-.02)*(state.time+dt)}))};
    value.loads={forces:[{toolId:'catheter',node:0,value:[0,-.4,0]},{toolId:'wire',node:1,value:[0,-.4,0]}],
        torques:[{toolId:'wire',edge:1,value:.001},{toolId:'catheter',edge:1,value:-.001}]};
    return value;
}
function verifyOwners(f,r) {
    accepted(r);const samples=r.diagnostics.certificate.wallFriction.samples;
    assert.equal(r.diagnostics.certificate.wall.converged,true);assert.equal(samples.length,2);
    assert.equal(r.diagnostics.wallQueries,2*r.diagnostics.evaluations);
    assert.equal(r.diagnostics.contactQueries,r.diagnostics.wallQueries);
    assert.equal(r.diagnostics.wallFrictionRows,4);
    for(const s of samples) {
        const mu=f.wall.friction.mu??f.wall.friction.muByOwner.find(t=>t.owner===s.owner).mu;
        same(s.mu,mu,0);assert.ok(s.Fn>0);assert.ok(s.slip[0]<-1e-4);
        assert.ok(s.traction[0]>0);assert.ok(Math.abs(s.traction[1])>1e-10);
        const support=Math.hypot(...s.slip.map((v,k)=>v*mu[k]));
        same(s.traction,s.slip.map((v,k)=>-s.Fn*mu[k]**2*v/support),1e-8);
        assert.equal(measureCompositeFriction({traction:s.traction,slip:s.slip,normalForce:s.Fn,mu,
            slipTolerance:1e-8,coneTolerance:1e-9,workTolerance:1e-9}).converged,true);
        const owner=f.wall.contactOwners.edges.flatMap(e=>(e.walls??(e.wall?[e.wall]:[])).map(w=>({edge:e.edge,wall:w}))).find(e=>e.wall.owner===s.owner),p=r.state.toolPositions.get(s.owner),edge=owner.edge;
        assert.ok(Math.min(p[edge][1],p[edge+1][1])-owner.wall.radius>=-1e-8);
        const tangent=[p[edge+1][0]-p[edge][0],0,p[edge+1][2]-p[edge][2]],length=Math.hypot(...tangent),axial=tangent.map(v=>v/length),circ=[-axial[2],0,axial[0]];
        const force=axial.map((v,k)=>s.traction[0]*v+s.traction[1]*circ[k]+(k===1?s.Fn:0)),b=r.balances.get(s.owner);
        same(b.contactForce,force,1e-12);same(b.momentumRate,b.appliedForce.map((v,k)=>v+b.boundaryForce[k]+force[k]),1e-7);
        for(let e=0;e<2;e++)close(Math.hypot(...p[e+1].map((v,k)=>v-p[e][k])),r.state.restLengths.get(s.owner)[e]);
    }
}

test('one whole dt applies different wall laws to both exposed tools with independent feed and spin',()=>{
    const f=twoOwnerFixture(),before=structuredClone(f.state),workspace=createCompositeJointTimeStepWorkspace(f.state);
    const first=advanceCompositeJointTimeStep(f.state,twoOwnerOptions(f,f.state,{workspace}));verifyOwners(f,first);
    assert.deepEqual(f.state,before);assert.equal(first.state.wallFrictionState.mu,undefined);
    for(const k of [5,50,500]) {
        const cold=advanceCompositeJointTimeStep(f.state,twoOwnerOptions(f,f.state,{k}));verifyOwners(f,cold);
        same(cold.state.positions.flat(),first.state.positions.flat(),1e-8);
        same(Array.from(cold.state.relative),Array.from(first.state.relative),1e-8);
        same(Array.from(cold.state.wallFrictionState.tractions),Array.from(first.state.wallFrictionState.tractions),1e-8);
    }
    const state=first.state,owned=structuredClone(state),cold=advanceCompositeJointTimeStep(state,twoOwnerOptions(f,state));verifyOwners(f,cold);
    for(const budget of [{evaluations:cold.diagnostics.evaluations-1},{contactQueries:cold.diagnostics.contactQueries-1}]) {
        const failed=advanceCompositeJointTimeStep(state,twoOwnerOptions(f,state,{workspace,budget}));
        assert.equal(failed.accepted,false);assert.equal(failed.state,state);assert.deepEqual(state,owned);
        const retry=advanceCompositeJointTimeStep(state,twoOwnerOptions(f,state,{workspace}));verifyOwners(f,retry);assert.deepEqual(retry.state,cold.state);
    }
    close(cold.state.time,2*dt,0);assert.equal(cold.state.step,2);
});

test('identical per-owner wall coefficients give the same physical whole-step root as the existing common law',()=>{
    const f=twoOwnerFixture(),mu=[.004,.003];f.wall.friction.muByOwner.forEach(e=>e.mu=mu.slice());
    const perOwner=advanceCompositeJointTimeStep(f.state,twoOwnerOptions(f,f.state));verifyOwners(f,perOwner);
    delete f.wall.friction.muByOwner;f.wall.friction.mu=mu;
    const common=advanceCompositeJointTimeStep(f.state,twoOwnerOptions(f,f.state));verifyOwners(f,common);
    const a=structuredClone(perOwner.state),b=structuredClone(common.state);delete a.wallFrictionState;delete b.wallFrictionState;
    assert.deepEqual(a,b);assert.deepEqual(perOwner.balances,common.balances);
    assert.deepEqual(perOwner.state.wallFrictionState.tractions,common.state.wallFrictionState.tractions);
    assert.equal(perOwner.diagnostics.evaluations,common.diagnostics.evaluations);
});

test('a shared chart edge solves both exposed wall reactions without merging the two surfaces',()=>{
    const f=twoOwnerFixture();
    f.wall.contactOwners.edges=[{edge:0,walls:[f.wall.contactOwners.edges[0].wall,{owner:'wire',radius:.16,materialSegmentId:'wire:0'}]},{edge:1,wall:null}];
    const workspace=createCompositeJointTimeStepWorkspace(f.state),step=(state,extra={})=>{
        const o=twoOwnerOptions(f,state,{workspace,...extra});o.loads.forces[1].node=0;return advanceCompositeJointTimeStep(state,o);
    };
    const a=step(f.state);verifyOwners(f,a);const before=structuredClone(a.state),cold=step(a.state,{workspace:undefined});verifyOwners(f,cold);
    const failed=step(a.state,{budget:{contactQueries:cold.diagnostics.contactQueries-1}});assert.equal(failed.accepted,false);assert.deepEqual(a.state,before);
    const b=step(a.state);verifyOwners(f,b);assert.deepEqual(b.state,cold.state);
    assert.ok(b.diagnostics.certificate.wallFriction.samples.every(s=>s.edge===0));
});

function staticKineticFixture() {
    const f=fixture();
    for(const p of f.state.positions)p[1]-=.00001;
    for(const p of f.state.toolPositions.get('wire'))p[1]-=.00001;
    delete f.wall.friction.mu;Object.assign(f.wall.friction,{law:'coulomb-static-kinetic',muStatic:[.006,.006],muKinetic:[.002,.002]});
    return f;
}
function staticKineticOptions(f,state,{axial=.0015,incoming,...extra}={}) {
    const o=options(f,state,{feed:0,spin:0,...extra});o.boundaries.positions=[];
    o.loads={forces:[{toolId:'wire',node:0,value:[axial,-.4,0]}],torques:[]};
    if(incoming!==undefined||!state.wallFrictionState)o.wall.friction.incomingSurfaceMotion=[{owner:'wire',edge:0,velocity:incoming??[0,0],interpretation:'physical-tangential-surface-velocity'}];
    return o;
}
const advanceModes=(f,state,extra)=>advanceCompositeJointTimeStep(state,staticKineticOptions(f,state,extra));
function verifyModes(r) {
    accepted(r);assert.equal(r.diagnostics.certificate.converged,true);assert.equal(r.diagnostics.wallFrictionModes.accepted,true);
    const sample=r.diagnostics.certificate.wallFriction.samples[0],b=r.balances.get('wire');
    same(b.momentumRate,b.appliedForce.map((v,k)=>v+b.boundaryForce[k]+b.contactForce[k]),1e-7);
    assert.ok(sample.Fn>0);assert.equal(r.state.wallFrictionState.law,'coulomb-static-kinetic');
    return sample;
}
test('common mechanical static friction holds between kinetic and static limits, then breaks away and keeps sliding with kinetic friction',()=>{
    const f=staticKineticFixture(),before=structuredClone(f.state),held=advanceModes(f,f.state);const staticSample=verifyModes(held);
    assert.equal(staticSample.mode,'static');assert.ok(Math.hypot(...staticSample.slip)<1e-8);
    assert.ok(Math.abs(staticSample.traction[0])>.002*staticSample.Fn);assert.ok(Math.abs(staticSample.traction[0])<.006*staticSample.Fn);
    assert.equal(held.state.wallFrictionState.modeHistory[0].nextMode,'static');assert.deepEqual(f.state,before);
    const released=advanceModes(f,held.state,{axial:.008}),sliding=verifyModes(released);
    assert.equal(released.diagnostics.wallFrictionModeChanges,1);assert.equal(sliding.mode,'kinetic');assert.ok(sliding.slip[0]>1e-8);
    close(sliding.traction[0],-.002*sliding.Fn,1e-9);
    const continued=advanceModes(f,released.state),moving=verifyModes(continued);
    assert.equal(moving.mode,'kinetic');assert.ok(moving.slip[0]>1e-8);assert.equal(continued.diagnostics.wallFrictionModeChanges,undefined);
    close(moving.traction[0],-.002*moving.Fn,1e-9);
});
test('two-branch roots and breakaway remain independent of numerical penalty and preserve rejected state with exact retry',()=>{
    const f=staticKineticFixture(),workspace=createCompositeJointTimeStepWorkspace(f.state),cold=advanceModes(f,f.state,{axial:.008});verifyModes(cold);
    for(const k of [5,50,500]) {
        const r=advanceModes(f,f.state,{axial:.008,k,workspace});verifyModes(r);assert.equal(r.diagnostics.wallFrictionModeChanges,1);
        same(r.state.positions.flat(),cold.state.positions.flat(),1e-8);same(Array.from(r.state.wallFrictionState.tractions),Array.from(cold.state.wallFrictionState.tractions),1e-8);
    }
    const before=structuredClone(f.state);
    for(const budget of [{directions:cold.diagnostics.directions-1},{evaluations:cold.diagnostics.evaluations-1},{contactQueries:cold.diagnostics.contactQueries-1}]) {
        const failed=advanceModes(f,f.state,{axial:.008,workspace,budget});assert.equal(failed.accepted,false);assert.equal(failed.state,f.state);assert.deepEqual(f.state,before);
        const retry=advanceModes(f,f.state,{axial:.008,workspace});verifyModes(retry);assert.deepEqual(retry.state,cold.state);
    }
});
test('a spatial wall slot cannot invent resting material motion or inherit another material label stop',()=>{
    const f=staticKineticFixture(),o=staticKineticOptions(f,f.state);delete o.wall.friction.incomingSurfaceMotion;
    const missing=advanceCompositeJointTimeStep(f.state,o);assert.equal(missing.accepted,false);assert.match(missing.error,/incoming surface motion/);
    const a=advanceModes(f,f.state);verifyModes(a);
    const other=structuredClone(a.state);other.wallFrictionState.modeHistory[0].label+=.1;
    const stale=advanceModes(f,other);assert.equal(stale.accepted,false);assert.match(stale.error,/another label/);
});

test('kinetic motion stops under zero axial drive and regains the larger static holding range on the next dt',()=>{
    const f=staticKineticFixture();let r=advanceModes(f,f.state,{axial:.008});verifyModes(r);
    let steps=0;
    while(r.state.wallFrictionState.modeHistory[0].nextMode!=='static'&&steps<30) {
        r=advanceModes(f,r.state,{axial:0});verifyModes(r);steps++;
    }
    assert.ok(steps>0&&steps<30);assert.equal(r.state.wallFrictionState.modeHistory[0].nextMode,'static');
    const held=advanceModes(f,r.state),sample=verifyModes(held);assert.equal(sample.mode,'static');
    assert.ok(Math.hypot(...sample.slip)<1e-8);assert.ok(Math.abs(sample.traction[0])>.002*sample.Fn);
    assert.equal(held.diagnostics.wallFrictionModeChanges,undefined);
});

test('equal coefficients in the two-branch law reproduce ordinary Coulomb state and reaction without extra solves',()=>{
    const f=staticKineticFixture();f.wall.friction.muKinetic=f.wall.friction.muStatic.slice();
    const two=advanceModes(f,f.state,{axial:.008});verifyModes(two);
    f.wall.friction.law='coulomb';f.wall.friction.mu=f.wall.friction.muStatic;delete f.wall.friction.muStatic;delete f.wall.friction.muKinetic;
    const one=advanceModes(f,f.state,{axial:.008});accepted(one);
    const a=structuredClone(two.state),b=structuredClone(one.state);delete a.wallFrictionState;delete b.wallFrictionState;
    assert.deepEqual(a,b);assert.deepEqual(two.state.wallFrictionState.tractions,one.state.wallFrictionState.tractions);
    assert.deepEqual(two.balances,one.balances);assert.equal(two.diagnostics.directions,one.diagnostics.directions);assert.equal(two.diagnostics.evaluations,one.diagnostics.evaluations);
});

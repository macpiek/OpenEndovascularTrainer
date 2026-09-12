import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {evaluateKirchhoffLumenSegmentContact} from '../src/physics/kirchhoffLumenContact.js';
import {measureCompositeFriction} from '../src/physics/kirchhoffCompositeFriction.js';
import {createCompositeJointMaterialHistory} from '../src/physics/kirchhoffCompositeJointMaterialHistory.js';
import {createCompositeJointTimeStepState,createCompositeJointTimeStepWorkspace,advanceCompositeJointTimeStep} from '../src/physics/kirchhoffCompositeJointTimeStep.js';

const dt=1/120;
const close=(a,b,t=1e-8)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b} (tol ${t})`);
const same=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const accepted=r=>assert.equal(r.accepted,true,JSON.stringify({status:r.status,error:r.error,diagnostics:r.diagnostics}));

function fixture({slope=-.002,offset=.341}={}) {
    const coordinates=[0,2,4],positions=coordinates.map(x=>[x,0,0]);
    const wire=coordinates.map(x=>[x+.1,offset+slope*x,0]);
    const layout=createCompositeChainLayout([['wire','catheter'],['wire','catheter']]);
    const modes=coordinates.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]}));
    const angles=new Map(),restLengths=new Map();
    const tools=['wire','catheter'].map(id=>{
        const p=id==='wire'?wire:positions,dsDx=id==='wire'?Math.hypot(1,slope):1;
        angles.set(id,new Float64Array(2));restLengths.set(id,new Float64Array(2).fill(2*dsDx));
        return {id,dsDx,reference:captureCompositeReferenceFrames(p),referenceTwists:new Float64Array(1),
            material:compileCompositeMaterial({stiffness:(id==='wire'?[2,3,1]:[8,11,4]).map((v,i)=>Array.from({length:3},(_,j)=>i===j?v:0)),intrinsic:[0,0,0]})};
    });
    const state=createCompositeJointTimeStepState({layout,coordinates,positions,
        relative:wire.flatMap((p,i)=>p.map((v,k)=>v-positions[i][k])),modes,angles,tools,restLengths,materialCoordinate:'reference-arclength'});
    const contacts={mode:'lumen-coulomb',chartId:'full-friction-dt',forcePerLength:1,
        friction:{law:'coulomb',mu:[.015,.006],forcePerLength:50,materialPath:'linear-affine-maps'},
        pairs:[{id:'own-side-0',innerToolId:'wire',outerToolId:'catheter',innerEdge:0,outerEdge:0,
            innerMaterialSegmentId:'wire:0',outerMaterialSegmentId:'catheter:0',lumenRadius:.5,innerRadius:.16,
            quadrature:[.25],openDistal:false,portalFilletRadius:0}]};
    return {state,contacts};
}
function options(f,state,{feed={wire:-.3,catheter:.1},force=.4,k=50,workspace,budget}={}) {
    const mappedEdges=state.layout.edgeToolIds.map((ids,edge)=>({tools:ids.map(id=>{
        const dsDx=state.tools.find(t=>t.id===id).dsDx,dsDt=feed[id];
        const previous=state.materialVelocities?.[edge]?.tools.find(t=>t.id===id);
        return {id,massPerMaterialLength:id==='wire'?.13:.24,
            materialMap:{sStart:(previous?.sStart??20+state.coordinates[edge]*dsDx)+dt*dsDt,dsDx,dsDt}};
    })}));
    // The fixture explicitly starts its external material reservoir at rest.
    // Previously accepted internal velocity fields are sampled at CURRENT
    // labels, preserving every old edge boundary during the second dt.
    const history=createCompositeJointMaterialHistory({materialVelocities:state.materialVelocities,
        reservoir:({toolId})=>({id:toolId,sStart:0,sEnd:100,velocities:[[0,0,0],[0,0,0]],interpretation:'physical-material-velocity'})});
    const sampled=history.prepare({coordinates:state.coordinates,inertiaEdges:mappedEdges});
    const inertiaEdges=mappedEdges.map((entry,edge)=>({tools:entry.tools.map((t,index)=>{
        const h=sampled.inertiaEdges[edge].tools[index];
        return {...t,...(h.pieces.length===1?{oldMaterialVelocities:h.oldMaterialVelocities}:
            {oldVelocityPieces:h.pieces.map(p=>({fractions:p.fractions,oldMaterialVelocities:p.oldMaterialVelocities}))})};
    })}));
    return {dt,torsionMode:'quasi-static',contacts:{...f.contacts,friction:{...f.contacts.friction,forcePerLength:k}},
        workspace,budget,
        boundaries:{positions:['wire','catheter'].map(toolId=>({toolId,node:2,value:f.state.toolPositions.get(toolId)[2]})),
            spins:[{toolId:'wire',edge:0,value:.03*(state.time+dt)},{toolId:'catheter',edge:0,value:-.01*(state.time+dt)}]},
        loads:{forces:[{toolId:'wire',node:0,value:[0,force,0]}],torques:[{toolId:'wire',edge:1,value:.001}]},
        inertia:{previousPositions:structuredClone(state.toolPositions),inertiaEdges}};
}
function advance(f,state,extra) {return advanceCompositeJointTimeStep(state,options(f,state,extra));}
function verifyPhysical(f,r,externalContactResultant=[0,0,0]) {
    const pair=f.contacts.pairs[0],p=r.state.toolPositions;
    const raw=evaluateKirchhoffLumenSegmentContact({...pair,innerStart:p.get('wire')[0],innerEnd:p.get('wire')[1],
        outerStart:p.get('catheter')[0],outerEnd:p.get('catheter')[1]});
    const Fn=r.state.lumenContactState.normalForces[0],Ft=r.state.lumenFrictionState.tractions;
    assert.ok(Fn>=0);assert.ok(raw.side.gap>=-1e-8);assert.ok(Math.abs(Fn*raw.side.gap)<=1e-9);
    const proof=r.diagnostics.certificate.friction;assert.equal(proof.converged,true);
    assert.equal(proof.samples.length,1);const s=proof.samples[0];
    same(s.traction,Array.from(Ft),1e-14);close(s.Fn,Fn,1e-14);
    if(s.slipRequired===false){
        // No physical slip is fabricated for an exactly unloaded open
        // contact. Independently check the raw gap and accepted force state.
        assert.equal(s.slip,null);assert.ok(raw.side.gap>0);assert.ok(Fn===0);
        assert.ok(Array.from(Ft).every(v=>v===0));assert.deepEqual(s.equationResidual,[0,0]);
        assert.equal(s.zeroConeProof,'strict-open-inactive-normal-deltaFn-zero-implies-deltaFt-zero');
        assert.equal(s.work,0);assert.equal(s.coneViolation,0);
    } else {
        const original=measureCompositeFriction({traction:s.traction,slip:s.slip,normalForce:Fn,mu:pair.mu??f.contacts.friction.mu,
            slipTolerance:1e-8,coneTolerance:1e-9,workTolerance:1e-9});
        assert.equal(original.converged,true,JSON.stringify(original));
    }
    for(const [id,points] of p)for(let edge=0;edge<2;edge++)close(Math.hypot(...points[edge+1].map((v,k)=>v-points[edge][k])),r.state.restLengths.get(id)[edge],1e-8);
    for(const b of r.balances.values())same(b.momentumRate,b.appliedForce.map((v,k)=>v+b.boundaryForce[k]+b.contactForce[k]),1e-7);
    same(r.balances.get('wire').contactForce.map((v,k)=>v+r.balances.get('catheter').contactForce[k]),externalContactResultant,1e-12);
    assert.equal(r.diagnostics.contactQueries,r.diagnostics.evaluations+1+(r.diagnostics.wallQueries??0));
    assert.equal(r.diagnostics.frictionPreparationQueries,1);
    return s;
}

test('one full 120 Hz joint step solves normal force and two Coulomb components with independent material feeds and spins',()=>{
    const f=fixture(),before=structuredClone(f.state),r=advance(f,f.state);accepted(r);
    const s=verifyPhysical(f,r);assert.deepEqual(f.state,before);
    assert.ok(s.Fn>0);assert.ok(Math.hypot(...s.traction)>0);assert.ok(s.slip[0]>1e-4);assert.ok(s.traction[0]<0);
    assert.equal(r.diagnostics.frictionRows,2);close(r.state.time,dt,0);assert.equal(r.state.step,1);
    assert.notEqual(r.state.angles.get('wire')[0],r.state.angles.get('catheter')[0]);
});

test('two loaded friction dt retain actual own material history and cannot silently disable their tangential load',()=>{
    const f=fixture(),a=advance(f,f.state);accepted(a);verifyPhysical(f,a);
    const before=structuredClone(a.state),b=advance(f,a.state);accepted(b);verifyPhysical(f,b);
    assert.deepEqual(a.state,before);assert.equal(b.state.step,2);close(b.state.time,2*dt,0);
    assert.throws(()=>advanceCompositeJointTimeStep(b.state,{...options(f,b.state),contacts:{...f.contacts,mode:'lumen-normal',friction:'none'}}),/cannot discard/);
});

test('original friction solution is unchanged by numerical penalty and by reused joint workspaces',()=>{
    const f=fixture(),workspace=createCompositeJointTimeStepWorkspace(f.state),baseline=advance(f,f.state);accepted(baseline);verifyPhysical(f,baseline);
    for(const k of [5,500]) {
        const cold=advance(f,f.state,{k}),warm=advance(f,f.state,{k,workspace});accepted(cold);accepted(warm);verifyPhysical(f,warm);
        for(const name of ['state','perTool','boundaryForces','contactForces','spinReactions','balances'])assert.deepEqual(warm[name],cold[name],name);
        for(const [id,points] of cold.state.toolPositions)points.forEach((p,i)=>same(p,baseline.state.toolPositions.get(id)[i],1e-8));
        same(cold.state.lumenContactState.normalForces,baseline.state.lumenContactState.normalForces,1e-7);
        same(cold.state.lumenFrictionState.tractions,baseline.state.lumenFrictionState.tractions,1e-8);
    }
});

test('late rejection restores normal/tangential history, geometry, spins and time and retry matches a cold physical step',()=>{
    const f=fixture(),workspace=createCompositeJointTimeStepWorkspace(f.state),a=advance(f,f.state,{workspace});accepted(a);
    const before=structuredClone(a.state),cold=advance(f,a.state);accepted(cold);
    for(const budget of [{evaluations:cold.diagnostics.evaluations-1},{contactQueries:0}]) {
        const failed=advance(f,a.state,{workspace,budget});assert.equal(failed.accepted,false);assert.equal(failed.state,a.state);assert.deepEqual(a.state,before);
        const retried=advance(f,a.state,{workspace});accepted(retried);verifyPhysical(f,retried);assert.deepEqual(retried.state,cold.state);
    }
});

test('reversing material feeds reverses axial friction while independent spin friction and both momentum balances remain active',()=>{
    const f=fixture(),r=advance(f,f.state,{feed:{wire:.3,catheter:-.1}});accepted(r);
    const s=verifyPhysical(f,r);assert.ok(s.Fn>0);assert.ok(s.slip[0]<-1e-4);assert.ok(s.traction[0]>0);
    assert.ok(Math.abs(s.slip[1])>1e-5);assert.ok(s.traction[1]*s.slip[1]<0);
});

test('unloading normal contact releases both Coulomb forces through the original equations without retaining stale friction',()=>{
    const f=fixture(),workspace=createCompositeJointTimeStepWorkspace(f.state);let state=f.state,result;
    for(const force of [.4,.4,0,-.4]) {result=advance(f,state,{force,workspace});accepted(result);verifyPhysical(f,result);state=result.state;}
    assert.ok(state.lumenContactState.normalForces.every(v=>v===0));
    assert.ok(state.lumenFrictionState.tractions.every(v=>v===0));
    assert.ok(result.diagnostics.certificate.contact.minGap>1e-4);
    assert.ok(result.diagnostics.minimumPrivateNormalForce<-.1);
    const cleared=advanceCompositeJointTimeStep(state,{...options(f,state,{force:-.4}),contacts:{...f.contacts,mode:'lumen-normal',friction:'none'}});
    accepted(cleared);assert.equal(cleared.state.lumenFrictionState,undefined);
});

test('Coulomb retains every declared interior pressure and surface sample independently of the mechanical edge count',()=>{
    const f=fixture(),before=structuredClone(f.state),input=options(f,f.state);
    input.contacts.pairs=input.contacts.pairs.map(p=>({...p,quadrature:[.25,.5,.75]}));
    const r=advanceCompositeJointTimeStep(f.state,input);accepted(r);
    assert.equal(r.diagnostics.lumenRows,3);assert.equal(r.diagnostics.frictionRows,6);
    assert.equal(r.diagnostics.lumenGauge.redundantSamples,0);assert.deepEqual(r.diagnostics.lumenGauge.transfers,[]);
    assert.equal(r.diagnostics.certificate.friction.samples.length,3);
    assert.equal(r.state.lumenContactState.normalForces.length,3);assert.equal(r.state.lumenFrictionState.tractions.length,6);
    r.diagnostics.certificate.friction.samples.forEach(s=>{
        if(s.slipRequired===false){assert.equal(s.Fn,0);assert.deepEqual(s.traction,[0,0]);}
        else assert.equal(measureCompositeFriction({traction:s.traction,slip:s.slip,normalForce:s.Fn,mu:s.mu,
            slipTolerance:1e-8,coneTolerance:1e-9,workTolerance:1e-9}).converged,true);
    });
    assert.deepEqual(f.state,before);
});

test('catheter vessel support and both components of wire-catheter friction enter the same physical step and retain their histories',()=>{
    const f=fixture(),workspace=createCompositeJointTimeStepWorkspace(f.state);
    const field={queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out){
        const t=ay===by?.5:ay>by?0:1,x=(1-t)*ax+t*bx,y=(1-t)*ay+t*by,z=(1-t)*az+t*bz;
        out.signedDistance=.8-y;out.signedGap=out.signedDistance-radius;out.segmentT=t;
        out.inward.values.set([0,-1,0]);out.closestPoint.values.set([x,.8,z]);
        out.faceIndex=1;out.branchId=0;out.source='analytic-plane';out.capsuleSampleCount=2;return out;
    }};
    const wall={mode:'wall-normal',friction:'none',contactMode:'envelope',chartId:'upper-catheter-support',field,forcePerLength:1,
        contactOwners:{edges:[0,1].map(edge=>({edge,wall:{owner:'catheter',radius:.8}}))}};
    let state=f.state;
    for(let step=0;step<2;step++) {
        const before=structuredClone(state),input={...options(f,state,{workspace}),wall};
        const r=advanceCompositeJointTimeStep(state,input);accepted(r);
        const wallForce=r.state.wallContactState.normalForces.reduce((sum,v)=>sum+v,0);
        assert.ok(wallForce>0);assert.ok(r.state.lumenContactState.normalForces[0]>0);
        verifyPhysical(f,r,[0,-wallForce,0]);assert.deepEqual(state,before);
        assert.ok(r.state.toolPositions.get('catheter').every(p=>p[1]<=1e-8));
        assert.ok(r.diagnostics.certificate.wall.converged);
        const rejected=advanceCompositeJointTimeStep(state,{...input,budget:{evaluations:r.diagnostics.evaluations-1}});
        assert.equal(rejected.accepted,false);assert.equal(rejected.state,state);assert.deepEqual(state,before);
        const retry=advanceCompositeJointTimeStep(state,input);accepted(retry);assert.deepEqual(retry.state,r.state);
        state=r.state;
    }
});

for(const twoBranch of [false,true])test(`lumen Coulomb and vessel ${twoBranch?'static/kinetic':'Coulomb'} solve together with two independent material histories and atomic retry`,()=>{
    const f=fixture(),angle=-.01,c=Math.cos(angle),s=Math.sin(angle),rotate=p=>[c*p[0]-s*p[1],s*p[0]+c*p[1],p[2]];
    // This original capsule has a strict selected endpoint. A flat capsule
    // can switch its loaded sample and needs a separate traction transport.
    const positions=f.state.positions.map(rotate),wire=f.state.toolPositions.get('wire').map(rotate);
    f.state=createCompositeJointTimeStepState({...f.state,positions,relative:wire.flatMap((p,i)=>p.map((v,k)=>v-positions[i][k])),
        tools:f.state.tools.map(t=>({...t,reference:captureCompositeReferenceFrames(f.state.toolPositions.get(t.id).map(rotate))}))});
    const workspace=createCompositeJointTimeStepWorkspace(f.state);
    const field={queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out){
        const t=ay>=by?0:1,p=t===0?[ax,ay,az]:[bx,by,bz];
        out.signedDistance=.8-p[1];out.signedGap=out.signedDistance-radius;out.segmentT=t;
        out.inward.values.set([0,-1,0]);out.closestPoint.values.set([p[0],.8,p[2]]);
        out.faceIndex=1;out.source='analytic-plane';out.capsuleSampleCount=2;return out;
    }};
    const wall={mode:'wall-coulomb',contactMode:'capsule',chartId:'upper-catheter-coulomb',field,forcePerLength:1,plane:{normal:[0,-1,0],offset:-.8},
        friction:{law:'coulomb',mu:[.006,.006],forcePerLength:50,materialPath:'linear-affine-maps',motion:'stationary-material',source:'analytic-plane',tangentBasis:'projected-own-tangent'},
        contactOwners:{edges:[{edge:0,wall:{owner:'catheter',radius:.8,materialSegmentId:'catheter:0'}},{edge:1,wall:null}]}};
    const feed=twoBranch?{wire:-.3,catheter:0}:undefined;
    let kineticReference;
    if(twoBranch) {
        kineticReference=advanceCompositeJointTimeStep(f.state,{...options(f,f.state,{feed}),wall});accepted(kineticReference);
        delete wall.friction.mu;Object.assign(wall.friction,{law:'coulomb-static-kinetic',muStatic:[.012,.012],muKinetic:[.006,.006],
            incomingSurfaceMotion:[{owner:'catheter',edge:0,velocity:[0,0],interpretation:'physical-tangential-surface-velocity'}]});
    }
    let state=f.state;
    for(let step=0;step<2;step++){
        if(twoBranch&&step>0)delete wall.friction.incomingSurfaceMotion;
        const before=structuredClone(state),input={...options(f,state,{workspace,feed}),wall},r=advanceCompositeJointTimeStep(state,input);accepted(r);
        const Fn=r.state.wallContactState.normalForces[0],Ft=r.state.wallFrictionState.tractions,p=r.state.toolPositions.get('catheter');
        const chord=[p[1][0]-p[0][0],0,p[1][2]-p[0][2]],length=Math.hypot(...chord),axial=chord.map(v=>v/length),circ=[axial[2],0,-axial[0]];
        const reaction=axial.map((v,k)=>Ft[0]*v+Ft[1]*circ[k]+(k===1?-Fn:0));
        assert.ok(Fn>0);assert.ok(Math.hypot(...Ft)>0);verifyPhysical(f,r,reaction);assert.deepEqual(state,before);
        const s=r.diagnostics.certificate.wallFriction.samples[0];assert.equal(r.diagnostics.certificate.wallFriction.converged,true);
        assert.equal(measureCompositeFriction({traction:Ft,slip:s.slip,normalForce:Fn,mu:s.mu,
            slipTolerance:1e-8,coneTolerance:1e-9,workTolerance:1e-9}).converged,true);
        if(twoBranch) {
            assert.equal(r.diagnostics.wallFrictionModes.accepted,true);assert.equal(s.mode,'kinetic');
            if(step===0) {
                assert.equal(r.diagnostics.wallFrictionModeChanges,1);
                same(r.state.positions.flat(),kineticReference.state.positions.flat(),1e-8);
                same(Array.from(r.state.relative),Array.from(kineticReference.state.relative),1e-8);
                same(Array.from(r.state.lumenFrictionState.tractions),Array.from(kineticReference.state.lumenFrictionState.tractions),1e-8);
                same(Array.from(Ft),Array.from(kineticReference.state.wallFrictionState.tractions),1e-8);
            }
        }
        const failed=advanceCompositeJointTimeStep(state,{...input,budget:{evaluations:r.diagnostics.evaluations-1}});
        assert.equal(failed.accepted,false);assert.equal(failed.state,state);assert.deepEqual(state,before);
        const retry=advanceCompositeJointTimeStep(state,input);accepted(retry);assert.deepEqual(retry.state,r.state);state=r.state;
    }
});

function nodalUpperWall() {
    const selected=[];
    const field={queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out){
        const t=ay>=by?0:1,p=t===0?[ax,ay,az]:[bx,by,bz];
        out.signedDistance=.8-p[1];out.signedGap=out.signedDistance-radius;out.segmentT=t;
        out.inward.values.set([0,-1,0]);out.closestPoint.values.set([p[0],.8,p[2]]);
        out.faceIndex=1;out.source='analytic-plane';out.capsuleSampleCount=2;
        if(ax!==bx||ay!==by||az!==bz)selected.push(t);return out;
    }};
    return {mode:'wall-coulomb',contactMode:'nodal-endpoints',pressureDiscretization:'nodal-endpoints-one-sided-surface',
        pressureSites:[{owner:'catheter',node:0,edge:0,trace:'right'},{owner:'catheter',node:1,edge:0,trace:'left'},{owner:'catheter',node:2,edge:1,trace:'left'}],
        chartId:'nodal-upper-catheter-coulomb',field,selected,forcePerLength:1,plane:{normal:[0,-1,0],offset:-.8},
        friction:{law:'coulomb',mu:[.006,.006],forcePerLength:50,materialPath:'linear-affine-maps',motion:'stationary-material',source:'analytic-plane',tangentBasis:'projected-own-tangent'},
        contactOwners:{edges:[0,1].map(edge=>({edge,wall:{owner:'catheter',radius:.8,materialSegmentId:`catheter:${edge}`}}))}};
}
function verifyNodalWall(f,r,wall) {
    const normal=r.diagnostics.certificate.wall,friction=r.diagnostics.certificate.wallFriction,p=r.state.toolPositions.get('catheter');
    assert.equal(normal.converged,true);assert.equal(friction.converged,true);assert.equal(normal.originalInequalities.length,6);
    assert.ok(normal.originalInequalities.every(s=>s.gap>=-1e-8));assert.equal(r.state.wallContactState.normalForces.length,3);
    assert.equal(r.state.wallFrictionState.tractions.length,6);const reaction=[0,0,0];
    for(const s of friction.samples){
        const {edge,node,traction:Ft,Fn}=s,chord=[p[edge+1][0]-p[edge][0],0,p[edge+1][2]-p[edge][2]],length=Math.hypot(...chord);
        const axial=chord.map(v=>v/length),circ=[axial[2],0,-axial[0]];
        for(let k=0;k<3;k++)reaction[k]+=Ft[0]*axial[k]+Ft[1]*circ[k]+(k===1?-Fn:0);
        assert.ok(Fn>=0);assert.ok(Math.abs(Fn*p[node][1])<=1e-9);
        assert.equal(measureCompositeFriction({traction:Ft,slip:s.slip,normalForce:Fn,mu:wall.friction.mu,
            slipTolerance:1e-8,coneTolerance:1e-9,workTolerance:1e-9}).converged,true);
    }
    verifyPhysical(f,r,reaction);assert.equal(r.diagnostics.wallFrictionRows,6);assert.equal(r.diagnostics.wallQueries,5*r.diagnostics.evaluations);
    return friction.samples;
}

test('explicit nodal pressure advances a loaded flat catheter with lumen and vessel Coulomb through original capsule sample changes',()=>{
    const f=fixture(),wall=nodalUpperWall(),workspace=createCompositeJointTimeStepWorkspace(f.state);let state=f.state;
    for(let step=0;step<2;step++){
        // Nodal surface history keeps its declared own-edge trace. Catheter
        // spin and mechanical motion remain free; crossing material history
        // during catheter feed requires the separate transport extension.
        const before=structuredClone(state),input={...options(f,state,{workspace,feed:{wire:-.3,catheter:0}}),wall};
        const r=advanceCompositeJointTimeStep(state,input);accepted(r);verifyNodalWall(f,r,wall);assert.deepEqual(state,before);
        assert.ok(r.state.wallContactState.normalForces.some(v=>v>0));assert.ok(r.state.wallFrictionState.tractions.some(v=>Math.abs(v)>1e-9));
        assert.equal(r.diagnostics.wallRowRebuilds,0);assert.equal(r.diagnostics.invalidTrials??0,0);assert.equal(r.diagnostics.wallChartDiscoveries,0);
        const failed=advanceCompositeJointTimeStep(state,{...input,budget:{evaluations:r.diagnostics.evaluations-1}});
        assert.equal(failed.accepted,false);assert.equal(failed.state,state);assert.deepEqual(state,before);
        const retry=advanceCompositeJointTimeStep(state,input);accepted(retry);assert.deepEqual(retry.state,r.state);state=r.state;
    }
    assert.ok(wall.selected.includes(0)&&wall.selected.includes(1));
});

test('nodal vessel/lumen Coulomb solution survives penalty changes, cold/reused solves and late query-budget rejection',()=>{
    const f=fixture(),wall=nodalUpperWall(),workspace=createCompositeJointTimeStepWorkspace(f.state),feed={wire:-.3,catheter:0};
    const run=(state,{k=50,workspace,budget}={})=>advanceCompositeJointTimeStep(state,{...options(f,state,{k,workspace,budget,feed}),
        wall:{...wall,friction:{...wall.friction,forcePerLength:k}}});
    const base=run(f.state);accepted(base);verifyNodalWall(f,base,wall);
    for(const k of [5,50,500]) {
        const cold=run(f.state,{k}),reused=run(f.state,{k,workspace});accepted(cold);accepted(reused);verifyNodalWall(f,reused,wall);
        assert.deepEqual(cold.state,reused.state);same(cold.state.positions.flat(),base.state.positions.flat(),1e-8);
        same(Array.from(cold.state.wallContactState.normalForces),Array.from(base.state.wallContactState.normalForces),1e-7);
        same(Array.from(cold.state.wallFrictionState.tractions),Array.from(base.state.wallFrictionState.tractions),1e-8);
        const before=structuredClone(cold.state),next=run(cold.state,{k});accepted(next);
        const rejected=run(cold.state,{k,workspace,budget:{contactQueries:next.diagnostics.contactQueries-1}});
        assert.equal(rejected.accepted,false);assert.equal(rejected.status,'contact-query-budget');assert.deepEqual(cold.state,before);
        const retry=run(cold.state,{k,workspace});accepted(retry);assert.deepEqual(retry.state,next.state);verifyNodalWall(f,retry,wall);
    }
});

test('loaded nodal surface trace changes and unavailable material entry cannot silently change pressure or winding',()=>{
    const f=fixture(),wall=nodalUpperWall(),feed={wire:-.3,catheter:0},input={...options(f,f.state,{feed}),wall};
    const a=advanceCompositeJointTimeStep(f.state,input);accepted(a);const before=structuredClone(a.state),base={...options(f,a.state,{feed}),wall};
    const pressureSites=wall.pressureSites.map((s,i)=>i===1?{...s,edge:1,trace:'right'}:s);
    assert.throws(()=>advanceCompositeJointTimeStep(a.state,{...base,wall:{...wall,pressureSites}}),/provenance changed/);assert.deepEqual(a.state,before);
    const entry=advanceCompositeJointTimeStep(a.state,{...options(f,a.state,{feed:{wire:-.3,catheter:-.1}}),wall});
    assert.equal(entry.accepted,false);assert.equal(entry.status,'surface-material-transport-required');assert.equal(entry.state,a.state);assert.deepEqual(a.state,before);
});

function prescribeLoadedCatheter(f,state,wall,{dy=0,workspace,budget}={}) {
    const input={...options(f,state,{feed:{wire:-.3,catheter:0},workspace,budget}),wall};
    input.boundaries.positions=input.boundaries.positions.filter(b=>b.toolId!=='catheter');
    for(let node=0;node<3;node++)input.boundaries.positions.push({toolId:'catheter',node,
        value:state.toolPositions.get('catheter')[node].map((v,k)=>v+(k===1?dy:0))});
    input.boundaries.spins=input.boundaries.spins.filter(b=>b.toolId!=='catheter');
    for(let edge=0;edge<2;edge++)input.boundaries.spins.push({toolId:'catheter',edge,value:state.angles.get('catheter')[edge]});
    return input;
}

test('loaded nodal wall forces back-substitute to exact zero at freshly open prescribed physical positions',()=>{
    const f=fixture(),wall=nodalUpperWall(),feed={wire:-.3,catheter:0};
    const loaded=advanceCompositeJointTimeStep(f.state,{...options(f,f.state,{feed}),wall});accepted(loaded);
    const before=structuredClone(loaded.state),positive=loaded.state.wallContactState.normalForces.filter(Fn=>Fn>0).length;
    assert.equal(positive,2);assert.ok(loaded.state.wallFrictionState.tractions.some(Ft=>Ft!==0));
    let reference;
    // These are numerical NCP/Coulomb scales, not physical stiffnesses. Even
    // when Fn-k*g selects the former active branch, g>0 at a prescribed
    // point has the unique original unilateral solution Fn=Ft=0.
    for(const [normalK,frictionK] of [[.1,5],[1,50],[10,500]]) {
        const currentWall={...wall,forcePerLength:normalK,friction:{...wall.friction,forcePerLength:frictionK}};
        const r=advanceCompositeJointTimeStep(loaded.state,prescribeLoadedCatheter(f,loaded.state,currentWall,{dy:-1e-4}));accepted(r);
        verifyNodalWall(f,r,currentWall);assert.deepEqual(loaded.state,before);
        assert.ok(r.diagnostics.certificate.wall.originalInequalities.every(s=>s.gap>0));
        assert.deepEqual(Array.from(r.state.wallContactState.normalForces),[0,0,0]);
        assert.deepEqual(Array.from(r.state.wallFrictionState.tractions),[0,0,0,0,0,0]);
        assert.equal(r.diagnostics.wallKnownOpenBacksubstitutions,positive);assert.equal(r.diagnostics.wallKnownOpenRowRefreshes,1);
        assert.deepEqual(r.diagnostics.suppressedPrescribedLengthRows,[1,3]);assert.equal(r.diagnostics.invalidTrials??0,0);
        if(reference)assert.deepEqual(r.state,reference);else reference=r.state;
        close(r.state.time,loaded.state.time+dt,0);assert.equal(r.state.step,loaded.state.step+1);
    }
});

test('exactly closed prescribed wall sites retain their accepted pressure gauge and untouched histories',()=>{
    const f=fixture(),wall=nodalUpperWall(),feed={wire:-.3,catheter:0};
    const loaded=advanceCompositeJointTimeStep(f.state,{...options(f,f.state,{feed}),wall});accepted(loaded);
    const before=structuredClone(loaded.state),r=advanceCompositeJointTimeStep(loaded.state,prescribeLoadedCatheter(f,loaded.state,wall));accepted(r);
    verifyNodalWall(f,r,wall);assert.deepEqual(loaded.state,before);
    assert.ok(r.diagnostics.certificate.wall.samples.every(s=>s.gap===0));
    assert.deepEqual(r.state.wallContactState.normalForces,loaded.state.wallContactState.normalForces);
    assert.equal(r.diagnostics.wallKnownOpenBacksubstitutions,0);assert.equal(r.diagnostics.wallKnownOpenRowRefreshes,0);
    assert.equal(r.state.wallContactState.normalForces[2],0);
    assert.equal(r.state.wallFrictionState.tractions[4],0);assert.equal(r.state.wallFrictionState.tractions[5],0);
    assert.deepEqual(r.diagnostics.suppressedPrescribedLengthRows,[1,3]);
});

test('known-open private back-substitution rolls back on preparation, direction and late query budgets and reuses identically',()=>{
    const f=fixture(),wall=nodalUpperWall(),feed={wire:-.3,catheter:0},workspace=createCompositeJointTimeStepWorkspace(f.state);
    const loaded=advanceCompositeJointTimeStep(f.state,{...options(f,f.state,{feed,workspace}),wall});accepted(loaded);
    const before=structuredClone(loaded.state),input=prescribeLoadedCatheter(f,loaded.state,wall,{dy:-1e-4,workspace});
    const cold=advanceCompositeJointTimeStep(loaded.state,{...input,workspace:undefined});accepted(cold);
    const warm=advanceCompositeJointTimeStep(loaded.state,input);accepted(warm);assert.deepEqual(warm.state,cold.state);
    for(const [budget,status,prepared] of [
        [{contactQueries:2},'contact-query-budget',false],
        [{directions:0},'direction-budget',true],
        [{contactQueries:cold.diagnostics.contactQueries-1},'contact-query-budget',true],
        [{evaluations:cold.diagnostics.evaluations-1},'evaluation-budget',true],
    ]) {
        const rejected=advanceCompositeJointTimeStep(loaded.state,{...input,budget});
        assert.equal(rejected.accepted,false);assert.equal(rejected.status,status);assert.equal(rejected.state,loaded.state);
        assert.equal(rejected.diagnostics.wallKnownOpenBacksubstitutions>0,prepared);assert.deepEqual(loaded.state,before);
        const retry=advanceCompositeJointTimeStep(loaded.state,input);accepted(retry);assert.deepEqual(retry.state,cold.state);
        verifyNodalWall(f,retry,wall);
    }
});

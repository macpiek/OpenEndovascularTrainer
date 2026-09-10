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
        return {id,massPerMaterialLength:id==='wire'?.13:.24,
            materialMap:{sStart:20+state.coordinates[edge]*dsDx+(state.time+dt)*dsDt,dsDx,dsDt}};
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
    const original=measureCompositeFriction({traction:s.traction,slip:s.slip,normalForce:Fn,mu:pair.mu??f.contacts.friction.mu,
        slipTolerance:1e-8,coneTolerance:1e-9,workTolerance:1e-9});
    assert.equal(original.converged,true,JSON.stringify(original));
    for(const [id,points] of p)for(let edge=0;edge<2;edge++)close(Math.hypot(...points[edge+1].map((v,k)=>v-points[edge][k])),r.state.restLengths.get(id)[edge],1e-8);
    for(const b of r.balances.values())same(b.momentumRate,b.appliedForce.map((v,k)=>v+b.boundaryForce[k]+b.contactForce[k]),1e-7);
    same(r.balances.get('wire').contactForce.map((v,k)=>v+r.balances.get('catheter').contactForce[k]),externalContactResultant,1e-12);
    assert.equal(r.diagnostics.contactQueries,r.diagnostics.evaluations+1+(r.diagnostics.wallQueries??0));
    assert.equal(r.diagnostics.frictionPreparationQueries,1);
    return s;
}


const f=fixture(),workspace=createCompositeJointTimeStepWorkspace(f.state),before=structuredClone(f.state);
    const field={queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out){
        const t=ay>=by?0:1,p=t===0?[ax,ay,az]:[bx,by,bz];
        out.signedDistance=.8-p[1];out.signedGap=out.signedDistance-radius;out.segmentT=t;
        out.inward.values.set([0,-1,0]);out.closestPoint.values.set([p[0],.8,p[2]]);
        out.faceIndex=1;out.source='analytic-plane';out.capsuleSampleCount=2;return out;
    }};
    const wall={mode:'wall-coulomb',contactMode:'capsule',chartId:'upper-catheter-coulomb',field,forcePerLength:1,plane:{normal:[0,-1,0],offset:-.8},
        friction:{law:'coulomb',mu:[.006,.006],forcePerLength:50,materialPath:'linear-affine-maps',motion:'stationary-material',source:'analytic-plane',tangentBasis:'projected-own-tangent'},
        contactOwners:{edges:[{edge:0,wall:{owner:'catheter',radius:.8,materialSegmentId:'catheter:0'}},{edge:1,wall:null}]}};

const input={...options(f,f.state,{workspace}),wall},r=advanceCompositeJointTimeStep(f.state,input);
assert.equal(r.accepted,false);assert.deepEqual(f.state,before);console.log(JSON.stringify({scope:'loaded flat catheter: original capsule wall point switch without traction transport; unresolved, immutable source state',status:r.status,error:r.error,diagnostics:r.diagnostics},null,2));

import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeJointTimeStepState,createCompositeJointTimeStepWorkspace,advanceCompositeJointTimeStep} from '../src/physics/kirchhoffCompositeJointTimeStep.js';
import {createCompositeJointMaterialHistory} from '../src/physics/kirchhoffCompositeJointMaterialHistory.js';

const dt=1/120,close=(a,b,t=1e-8)=>assert.ok(Number.isFinite(a)&&Math.abs(a-b)<=t,`${a} != ${b}`);
const accepted=r=>assert.equal(r.accepted,true,JSON.stringify({status:r.status,error:r.error,diagnostics:r.diagnostics}));
const span=(id,sStart,sEnd,v)=>({id,sStart,sEnd,velocities:[[v,0,0],[v,0,0]],interpretation:'physical-material-velocity'});
function fixture() {
    const coordinates=[0,2,4],positions=coordinates.map(x=>[x,0,0]),wire=coordinates.map(x=>[x,.1,0]);
    const layout=createCompositeChainLayout([['wire','catheter'],['wire','catheter']]);
    const tools=['wire','catheter'].map(id=>({id,dsDx:1,reference:captureCompositeReferenceFrames(id==='wire'?wire:positions),referenceTwists:new Float64Array(1),material:compileCompositeMaterial({EI1:2,GJ:1})}));
    const state=createCompositeJointTimeStepState({layout,coordinates,positions,relative:wire.flatMap((p,i)=>p.map((v,k)=>v-positions[i][k])),
        modes:coordinates.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]})),tools,
        angles:new Map(tools.map(t=>[t.id,new Float64Array(2)])),restLengths:new Map(tools.map(t=>[t.id,new Float64Array([2,2])])),materialCoordinate:'reference-arclength'});
    const materialVelocities=[{edge:0,tools:[span('wire',0,1.5,1),span('catheter',10,12,-.75)]},
        {edge:1,tools:[span('wire',1.5,5,3),span('catheter',12,14,-.75)]}];
    const history=createCompositeJointMaterialHistory({materialVelocities,reservoir:({toolId})=>toolId==='catheter'?span('catheter',9,10,-.75):null});
    const maps=state.layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,materialMap:{sStart:coordinates[e]+(id==='wire'?.25:9.875),dsDx:1,dsDt:(id==='wire'?.25:-.125)/dt}}))}));
    return {state,history,maps};
}
function prepare(state,history,maps) {
    const prepared=history.prepare({coordinates:state.coordinates,inertiaEdges:maps});
    const inertiaEdges=prepared.inertiaEdges.map(entry=>({tools:entry.tools.map(t=>({id:t.id,massPerMaterialLength:.1,materialMap:t.materialMap,
        ...(t.compatibleWithAffineEdgeOperator?{oldMaterialVelocities:t.oldMaterialVelocities}:{oldVelocityPieces:t.pieces.map(p=>({fractions:p.fractions,oldMaterialVelocities:p.oldMaterialVelocities}))})}))}));
    return {prepared,input:{dt,torsionMode:'quasi-static',contacts:'none',inertia:{previousPositions:structuredClone(state.toolPositions),inertiaEdges},
        boundaries:{positions:[],spins:[{toolId:'wire',edge:0,value:0},{toolId:'catheter',edge:0,value:0}]}}};
}

test('opposite material feeds cross old velocity boundaries and integrate their exact momentum in one unchanged joint chart',()=>{
    const f=fixture(),p=prepare(f.state,f.history,f.maps),before=structuredClone(f.state),r=advanceCompositeJointTimeStep(f.state,p.input);
    accepted(r);assert.deepEqual(f.state,before);assert.equal(r.state.layout.nodeCount,3);assert.equal(r.state.relative.length,9);
    assert.ok(p.prepared.requiredCuts.some(c=>c.toolId==='wire'&&c.s===1.5));
    assert.ok(p.input.inertia.inertiaEdges[0].tools.find(t=>t.id==='wire').oldVelocityPieces);
    // Exact integration over CURRENT material labels [.25,4.25]:
    // 1.25 mm at v=1 and 2.75 mm at v=3, independently of Gauss locations.
    close(r.perTool.get('wire').oldMomentum[0],.1*(1.25+2.75*3),1e-14);
    close(r.perTool.get('catheter').oldMomentum[0],.1*4*(-.75),1e-14);
    for(const [id,p0] of f.state.toolPositions) {
        const v=id==='wire'?2.375:-.75,u=-(id==='wire'?.25:-.125)/dt;
        r.state.toolPositions.get(id).forEach((p,i)=>p.forEach((x,k)=>close(x,p0[i][k]+(k===0?dt*(v-u):0),1e-9)));
        for(const entry of r.state.materialVelocities)entry.tools.find(t=>t.id===id).velocities.forEach(p=>{close(p[0],v,1e-8);close(p[1],0,1e-10);close(p[2],0,1e-10);});
        r.balances.get(id).residual.forEach(v=>close(v,0,2e-7));
    }
    const nextHistory=createCompositeJointMaterialHistory({materialVelocities:r.state.materialVelocities});
    const heldMaps=f.maps.map(entry=>({tools:entry.tools.map(t=>({...t,materialMap:{...t.materialMap,dsDt:0}}))}));
    const next=advanceCompositeJointTimeStep(r.state,prepare(r.state,nextHistory,heldMaps).input);accepted(next);assert.equal(next.state.step,2);
    for(const [id,b] of next.balances){b.residual.forEach(v=>close(v,0,2e-7));close(next.perTool.get(id).oldMomentum[0],r.perTool.get(id).momentum[0],1e-10);}
});

test('piecewise prepared inputs retain exact cold/reuse results and do not leak failed or changed histories',()=>{
    const f=fixture(),p=prepare(f.state,f.history,f.maps),workspace=createCompositeJointTimeStepWorkspace(f.state);
    const cold=advanceCompositeJointTimeStep(f.state,p.input),warm=advanceCompositeJointTimeStep(f.state,{...p.input,workspace});accepted(cold);accepted(warm);
    assert.deepEqual(warm.state,cold.state);assert.deepEqual(warm.balances,cold.balances);
    const before=structuredClone(f.state),oldInput=structuredClone(p.input),late=advanceCompositeJointTimeStep(f.state,{...p.input,workspace,budget:{evaluations:cold.diagnostics.evaluations-1}});
    assert.equal(late.accepted,false);assert.equal(late.state,f.state);assert.deepEqual(f.state,before);assert.deepEqual(p.input,oldInput);
    const retry=advanceCompositeJointTimeStep(f.state,{...p.input,workspace});accepted(retry);assert.deepEqual(retry.state,cold.state);
    const bad=structuredClone(p.input),tool=bad.inertia.inertiaEdges[0].tools.find(t=>t.oldVelocityPieces);tool.oldVelocityPieces[0].fractions[1]-=.01;
    assert.throws(()=>advanceCompositeJointTimeStep(f.state,{...bad,workspace}),/exactly cover/);assert.deepEqual(f.state,before);
    const ambiguous=structuredClone(p.input);ambiguous.inertia.inertiaEdges[0].tools.find(t=>t.oldVelocityPieces).oldMaterialVelocities=[[0,0,0],[0,0,0]];
    assert.throws(()=>advanceCompositeJointTimeStep(f.state,ambiguous),/unambiguous/);
    const final=advanceCompositeJointTimeStep(f.state,{...p.input,workspace});accepted(final);assert.deepEqual(final.state,cold.state);
});

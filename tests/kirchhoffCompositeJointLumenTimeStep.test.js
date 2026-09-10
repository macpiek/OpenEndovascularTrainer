import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {evaluateKirchhoffLumenSegmentContact} from '../src/physics/kirchhoffLumenContact.js';
import {createCompositeJointTimeStepState,createCompositeJointTimeStepWorkspace,advanceCompositeJointTimeStep} from '../src/physics/kirchhoffCompositeJointTimeStep.js';

const close=(a,b,t=1e-8)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}`);
const same=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const options={dt:.1,torsionMode:'quasi-static',tolerances:{force:1e-7,torque:1e-8,length:1e-8,boundary:1e-9,
    linearForce:5e-10,linearTorque:5e-10,linearConstraint:5e-11,lumenGap:1e-8,lumenNcp:1e-8,lumenWork:1e-9}};
function fixture({y=.34,slope=-.02,xOffset=.1}={}) {
    const coordinates=[0,2,4],positions=coordinates.map(x=>[x,0,0]),wire=coordinates.map(x=>[x+xOffset,y+slope*x,0]);
    const layout=createCompositeChainLayout([['wire','catheter'],['wire','catheter']]),modes=coordinates.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]}));
    const angles=new Map(),restLengths=new Map(),tools=['wire','catheter'].map(id=>{
        const dsDx=id==='wire'?Math.hypot(1,slope):1;angles.set(id,new Float64Array(2));restLengths.set(id,new Float64Array(2).fill(2*dsDx));
        return {id,dsDx,reference:captureCompositeReferenceFrames(id==='wire'?wire:positions),referenceTwists:new Float64Array(1),
            material:compileCompositeMaterial({stiffness:(id==='wire'?[2,3,1]:[8,11,4]).map((v,i)=>Array.from({length:3},(_,j)=>i===j?v:0)),intrinsic:[0,0,0]})};
    });
    const state=createCompositeJointTimeStepState({layout,coordinates,positions,relative:wire.flatMap((p,i)=>p.map((v,k)=>v-positions[i][k])),modes,angles,tools,restLengths,materialCoordinate:'reference-arclength'});
    const contacts={mode:'lumen-normal',friction:'none',chartId:'fixture-fixed-chart',forcePerLength:1,
        pairs:[{id:'first-pair',innerToolId:'wire',outerToolId:'catheter',innerEdge:0,outerEdge:0,innerMaterialSegmentId:'wire:e0',outerMaterialSegmentId:'catheter:e0',
            lumenRadius:.5,innerRadius:.16,quadrature:[.25,.5,.75],openDistal:false,portalFilletRadius:0}]};
    const boundaries={positions:[{toolId:'wire',node:2,value:wire[2]},{toolId:'catheter',node:2,value:positions[2]}],spins:[{toolId:'wire',edge:0,value:0},{toolId:'catheter',edge:0,value:0}]};
    return {state,contacts,boundaries};
}
function inertiaFor(state) {
    return {previousPositions:structuredClone(state.toolPositions),inertiaEdges:state.layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,massPerMaterialLength:id==='wire'?.13:.24,
        materialMap:{sStart:20+state.coordinates[e]*state.tools.find(t=>t.id===id).dsDx,dsDx:state.tools.find(t=>t.id===id).dsDx,dsDt:0},
        oldMaterialVelocities:structuredClone(state.materialVelocities?.[e].tools.find(t=>t.id===id).velocities??[[0,0,0],[0,0,0]])}))}))};
}
function advance(f,state,force,extra={}) {return advanceCompositeJointTimeStep(state,{...options,contacts:f.contacts,boundaries:f.boundaries,inertia:inertiaFor(state),
    loads:{forces:[{toolId:'wire',node:0,value:[0,force,0]}],torques:[{toolId:'wire',edge:1,value:.001}]},...extra});}
function accepted(r) {assert.equal(r.accepted,true,JSON.stringify({status:r.status,error:r.error,diagnostics:r.diagnostics}));}
function rawContact(f,state,samples=f.contacts.pairs[0].quadrature) {
    const p=state.toolPositions,c=f.contacts.pairs[0];
    return evaluateKirchhoffLumenSegmentContact({...c,quadrature:samples,innerStart:p.get('wire')[0],innerEnd:p.get('wire')[1],outerStart:p.get('catheter')[0],outerEnd:p.get('catheter')[1]});
}
function physicalProof(f,r) {
    const raw=rawContact(f,r.state),proof=r.diagnostics.certificate.contact,Fn=r.state.lumenContactState.normalForces;
    assert.equal(raw.samples.length,f.contacts.pairs[0].quadrature.length);
    close(proof.minGap,Math.min(...raw.samples.map(s=>s.gap)),1e-14);
    raw.samples.forEach((s,i)=>{assert.ok(s.gap>=-1e-8);assert.ok(Fn[i]>=0);close(proof.samples[i].gap,s.gap,1e-14);assert.ok(Math.abs(Fn[i]*s.gap)<=1e-9);});
    for(const [id,p] of r.state.toolPositions)for(let e=0;e<2;e++)close(Math.hypot(...p[e+1].map((v,k)=>v-p[e][k])),r.state.restLengths.get(id)[e],1e-8);
    for(const balance of r.balances.values())same(balance.momentumRate,balance.appliedForce.map((v,k)=>v+balance.boundaryForce[k]+balance.contactForce[k]),1e-7);
    same(r.balances.get('wire').contactForce,r.balances.get('catheter').contactForce.map(v=>-v),1e-14);
    assert.equal(r.diagnostics.contactQueries,r.diagnostics.evaluations*f.contacts.pairs[0].quadrature.length+(r.diagnostics.lumenGauge?.preparationQueries??0));
}

test('all original singleton lumen inequalities activate in the one shared solve and drive both physical materials',()=>{
    const f=fixture(),before=structuredClone(f.state),r=advance(f,f.state,.4);accepted(r);physicalProof(f,r);
    assert.deepEqual(f.state,before);assert.equal(r.diagnostics.lumenSamples,3);assert.equal(r.diagnostics.certificate.contact.activeCount,1);
    assert.ok(r.state.lumenContactState.normalForces[0]>.25);assert.equal(r.state.lumenContactState.normalForces[1],0);assert.equal(r.state.lumenContactState.normalForces[2],0);
    assert.ok(r.state.toolPositions.get('catheter')[0][1]>.01);
    const free=advance(f,f.state,.4,{contacts:'none'});accepted(free);assert.ok(Math.abs(free.state.toolPositions.get('catheter')[0][1])<1e-12);
    assert.equal(r.state.time,.1);assert.equal(r.state.step,1);
});

test('two loaded dt retain own physical velocities and release reaches exact zero without clamping signed Newton trials',()=>{
    const f=fixture(),a=advance(f,f.state,.4);accepted(a);const b=advance(f,a.state,.4);accepted(b);physicalProof(f,b);
    assert.ok(b.state.lumenContactState.normalForces[0]>a.state.lumenContactState.normalForces[0]);
    const before=structuredClone(b.state),rejected=advance(f,b.state,0,{budget:{directions:1}});
    assert.equal(rejected.accepted,false);assert.equal(rejected.status,'direction-budget');assert.equal(rejected.state,b.state);assert.deepEqual(b.state,before);
    assert.ok(rejected.diagnostics.minimumPrivateNormalForce<-.01);
    const released=advance(f,b.state,0);accepted(released);physicalProof(f,released);
    assert.ok(released.diagnostics.minimumPrivateNormalForce<-.01);assert.ok(released.diagnostics.certificate.contact.minGap>1e-3);
    assert.equal(released.state.lumenContactState.normalForces.every(v=>v===0),true);assert.equal(released.state.step,3);
    const again=advance(f,b.state,0);accepted(again);assert.deepEqual(again.state,released.state);
    for(const entry of released.state.materialVelocities)for(const t of entry.tools) {
        const p=released.state.toolPositions.get(t.id),old=b.state.toolPositions.get(t.id);
        t.velocities.forEach((v,end)=>same(v,p[entry.edge+end].map((x,k)=>(x-old[entry.edge+end][k])/.1),1e-12));
    }
    assert.throws(()=>advance(f,b.state,0,{contacts:'none'}),/cannot discard/);
});

test('normal forces and accepted physical response are independent of NCP numerical scaling',()=>{
    const f=fixture(),solutions=[];
    for(const forcePerLength of [.1,1,10]) {const r=advance(f,f.state,.4,{contacts:{...f.contacts,forcePerLength}});accepted(r);physicalProof(f,r);solutions.push(r);}
    for(const r of solutions.slice(1)) {
        for(const [id,p] of r.state.toolPositions)p.forEach((v,i)=>same(v,solutions[0].state.toolPositions.get(id)[i],1e-8));
        same(r.state.lumenContactState.normalForces,solutions[0].state.lumenContactState.normalForces,1e-7);
        assert.equal(r.state.lumenContactState.signature,solutions[0].state.lumenContactState.signature);
    }
    const continued=advance(f,solutions[0].state,.4,{contacts:{...f.contacts,forcePerLength:10}});accepted(continued);physicalProof(f,continued);
});

test('cold and reused workspaces preserve all normal forces, physical results and late-rejection histories',()=>{
    const f=fixture(),workspace=createCompositeJointTimeStepWorkspace(f.state);let state=f.state;
    for(const force of [.4,.4,0]) {
        const cold=advance(f,state,force),warm=advance(f,state,force,{workspace});accepted(cold);accepted(warm);
        for(const k of ['state','perTool','boundaryForces','contactForces','spinReactions','balances'])assert.deepEqual(warm[k],cold[k],k);
        assert.deepEqual(warm.diagnostics.certificate,cold.diagnostics.certificate);
        const before=structuredClone(state),late=advance(f,state,force,{workspace,budget:{evaluations:cold.diagnostics.evaluations-1}});
        assert.equal(late.accepted,false);assert.equal(late.status,'evaluation-budget');assert.equal(late.state,state);assert.deepEqual(state,before);
        state=warm.state;
    }
    assert.equal(workspace.diagnostics.directionBuilds,1);assert.equal(workspace.diagnostics.assembly.structureBuilds,1);
    assert.equal(workspace.diagnostics.lumen.chartBuilds,1);assert.equal(workspace.diagnostics.lumen.chartHits,5);
    assert.equal(workspace.diagnostics.lumen.support.builds,1);assert.equal(workspace.diagnostics.lumen.support.hits,17);
});

test('zero radial normal is eliminated only when its original gap is strictly open and Fn is exactly zero',()=>{
    const f=fixture({y:0,slope:0}),r=advance(f,f.state,0,{loads:{forces:[],torques:[]}});accepted(r);physicalProof(f,r);
    assert.equal(r.diagnostics.certificate.contact.eliminatedCount,3);assert.equal(r.diagnostics.directions,0);
    const closed={...f.contacts,pairs:f.contacts.pairs.map(p=>({...p,lumenRadius:p.innerRadius}))};
    const failure=advance(f,f.state,0,{contacts:closed});assert.equal(failure.accepted,false);assert.equal(failure.status,'unsupported-lumen-contact');
    const bad=createCompositeJointTimeStepState(r.state);bad.lumenContactState.normalForces[0]=.01;
    const withForce=advance(f,bad,0);assert.equal(withForce.accepted,false);assert.equal(withForce.status,'unsupported-lumen-contact');
});

test('query budgets, endpoint support, duplicate samples and stale material provenance fail with unchanged retry state',()=>{
    const f=fixture(),before=structuredClone(f.state),workspace=createCompositeJointTimeStepWorkspace(f.state);
    const limited=advance(f,f.state,.4,{workspace,budget:{contactQueries:2}});
    assert.equal(limited.accepted,false);assert.equal(limited.status,'contact-query-budget');assert.equal(limited.diagnostics.contactQueries,2);assert.equal(limited.state,f.state);assert.deepEqual(f.state,before);
    const badPair=structuredClone(f.contacts);badPair.pairs[0].quadrature=[.25,.25];assert.throws(()=>advance(f,f.state,.4,{contacts:badPair}),/unique quadrature/);
    const duplicate=structuredClone(f.contacts);duplicate.pairs.push({...duplicate.pairs[0],id:'another-name'});assert.throws(()=>advance(f,f.state,.4,{contacts:duplicate}),/Duplicate semantic/);
    const endpoint=fixture({xOffset:0});endpoint.contacts.pairs[0].quadrature=[0];
    const failure=advance(endpoint,endpoint.state,.4);assert.equal(failure.accepted,false);assert.equal(failure.status,'unsupported-lumen-contact');
    const portal=structuredClone(f.contacts);portal.pairs[0].openDistal=true;assert.throws(()=>advance(f,f.state,.4,{contacts:portal}),/openDistal:false/);
    const good=advance(f,f.state,.4,{workspace});accepted(good);
    const stale=structuredClone(f.contacts);stale.pairs[0].innerMaterialSegmentId='different-material';assert.throws(()=>advance(f,good.state,.4,{workspace,contacts:stale}),/provenance changed/);
    const recharted=createCompositeJointTimeStepState(good.state);recharted.coordinates.forEach((v,i)=>recharted.coordinates[i]=2*v);recharted.tools.forEach(t=>t.dsDx/=2);
    // Same physical geometry/rest metrics/material labels and chartId; only
    // the actual chart changes. It still needs an explicit history transfer.
    assert.throws(()=>advance(f,recharted,.4),/provenance changed/);
    const rotated=createCompositeJointTimeStepState(good.state),basis=[[0,1,0],[-1,0,0],[0,0,1]];
    rotated.modes.forEach(m=>{
        const d=rotated.toolPositions.get('wire')[m.node].map((v,k)=>v-rotated.positions[m.node][k]);
        m.basis=structuredClone(basis);m.relativeDofs.forEach((dof,a)=>rotated.relative[dof]=basis[a].reduce((s,v,k)=>s+v*d[k],0));
    });
    assert.throws(()=>advance(f,rotated,.4),/provenance changed/);
    const retried=advance(f,f.state,.4,{workspace});accepted(retried);assert.deepEqual(retried.state,good.state);
});

test('accepted nodal normal loads equal independent detector work and close both material impulse balances',()=>{
    const f=fixture(),r=advance(f,f.state,.4);accepted(r);
    const expected=new Map([['wire',Array.from({length:3},()=>[0,0,0])],['catheter',Array.from({length:3},()=>[0,0,0])]]);
    f.contacts.pairs[0].quadrature.forEach((s,i)=>{
        const raw=rawContact(f,r.state,[s]).side,Fn=r.state.lumenContactState.normalForces[i];
        for(const [id,name] of [['wire','inner'],['catheter','outer']])raw.gradients[name].forEach((g,node)=>g.forEach((v,k)=>expected.get(id)[node][k]+=Fn*v));
    });
    for(const [id,p] of expected)p.forEach((v,i)=>same(v,r.contactForces.get(id)[i],1e-14));
    const velocity=new Map([['wire',[[.07,-.04,.02],[-.02,.03,.01],[0,0,0]]],['catheter',[[-.01,.02,.03],[.04,-.02,.01],[0,0,0]]]]);
    let power=0;for(const [id,p] of expected)p.forEach((v,i)=>v.forEach((x,k)=>power+=x*velocity.get(id)[i][k]));
    const virtualWork=epsilon=>{
        const trial={toolPositions:new Map([...r.state.toolPositions].map(([id,p])=>[id,p.map((v,i)=>v.map((x,k)=>x+epsilon*velocity.get(id)[i][k]))]))};
        return f.contacts.pairs[0].quadrature.reduce((sum,s,i)=>sum+r.state.lumenContactState.normalForces[i]*rawContact(f,trial,[s]).side.gap,0);
    };
    close(power,(virtualWork(1e-6)-virtualWork(-1e-6))/(2e-6),1e-10);
    for(const [id,b] of r.balances)same(r.perTool.get(id).momentumIncrement,b.appliedForce.map((v,k)=>options.dt*(v+b.boundaryForce[k]+b.contactForce[k])),1e-8);
});

test('independent wire feed and handle rotation preserve catheter boundary and own material velocity histories under normal contact',()=>{
    const f=fixture(),a=advance(f,f.state,.4);accepted(a);
    const boundaries=structuredClone(f.boundaries),feed=.001;
    boundaries.positions[0].value[0]+=feed;boundaries.positions[0].value[1]-=.02*feed;boundaries.spins[0].value=.1;
    const r=advance(f,a.state,.4,{boundaries});accepted(r);physicalProof(f,r);
    same(r.state.toolPositions.get('wire')[2],boundaries.positions[0].value,1e-9);
    assert.deepEqual(r.state.toolPositions.get('catheter')[2],f.boundaries.positions[1].value);
    close(r.state.angles.get('wire')[0],.1,0);close(r.state.angles.get('catheter')[0],0,0);
    const wireTip=r.state.materialVelocities[1].tools.find(t=>t.id==='wire').velocities[1];
    same(wireTip,[feed/.1,-.02*feed/.1,0],1e-10);
    same(r.state.materialVelocities[1].tools.find(t=>t.id==='catheter').velocities[1],[0,0,0],1e-14);
});

test('the same physical parameters and original gates accept loaded normal-contact steps at 120Hz',()=>{
    const f=fixture({y:.35});let state=f.state;
    for(const force of [.4,.4,0]) {
        const r=advance(f,state,force,{dt:1/120});accepted(r);physicalProof(f,r);
        if(state===f.state)assert.ok(r.state.lumenContactState.normalForces[0]>0);
        close(r.state.time-state.time,1/120,1e-15);state=r.state;
    }
});

function parallelFixture() {
    const f=fixture({y:.25,slope:0,xOffset:.25});f.contacts.pairs[0].innerRadius=.25;
    const seed=advance(f,f.state,0,{loads:{forces:[],torques:[]}});accepted(seed);
    const state=createCompositeJointTimeStepState(seed.state);state.lumenContactState.normalForces.set([1,1,1]);
    const loads={forces:[{toolId:'wire',node:0,value:[0,1.6,0]},{toolId:'wire',node:1,value:[0,1.5,0]},
        {toolId:'catheter',node:0,value:[0,-1.125,0]},{toolId:'catheter',node:1,value:[0,-1.875,0]}],torques:[]};
    return {...f,state,loads};
}

test('parallel full-side pressure uses the exact two-extreme envelope and accepts a full nonlinear dt with all original gates',()=>{
    const f=parallelFixture(),before=structuredClone(f.state),workspace=createCompositeJointTimeStepWorkspace(f.state);
    const r=advance(f,f.state,0,{loads:f.loads}),warm=advance(f,f.state,0,{loads:f.loads,workspace});accepted(r);accepted(warm);
    assert.deepEqual(warm.state,r.state);assert.deepEqual(warm.balances,r.balances);assert.deepEqual(warm.diagnostics.certificate,r.diagnostics.certificate);assert.deepEqual(f.state,before);
    assert.equal(r.diagnostics.lumenSamples,3);assert.equal(r.diagnostics.lumenRows,2);assert.equal(r.diagnostics.lumenGauge.transfers.length,1);
    assert.deepEqual(r.diagnostics.lumenGauge.transfers[0].forces,[.5,.5]);assert.equal(r.diagnostics.lumenGauge.preparationQueries,3);
    assert.equal(r.state.lumenContactState.normalForces[1],0);assert.ok(r.state.lumenContactState.normalForces[0]>1.6);assert.ok(r.state.lumenContactState.normalForces[2]>1.46);
    assert.ok(r.diagnostics.directions>0);assert.equal(r.diagnostics.factorizations,r.diagnostics.directions);
    physicalProof(f,r);
    const limited=advance(f,f.state,0,{loads:f.loads,workspace,budget:{contactQueries:2}});
    assert.equal(limited.accepted,false);assert.equal(limited.status,'contact-query-budget');assert.deepEqual(f.state,before);assert.equal(limited.state,f.state);
    const late=advance(f,f.state,0,{loads:f.loads,workspace,budget:{evaluations:r.diagnostics.evaluations-1}});
    assert.equal(late.accepted,false);assert.deepEqual(f.state,before);assert.equal(late.state,f.state);
    const retry=advance(f,f.state,0,{loads:f.loads,workspace});accepted(retry);assert.deepEqual(retry.state,r.state);
    const next=advance(f,r.state,0,{loads:f.loads,workspace});accepted(next);physicalProof(f,next);assert.equal(next.diagnostics.lumenGauge.transfers.length,0);
});

import {createCompositeJointLumenRows} from '../src/physics/kirchhoffCompositeJointLumenRows.js';
function rowsFor(f,history=null) {return createCompositeJointLumenRows({...f.state,contacts:f.contacts,history,tolerances:options.tolerances});}
function historyFor(rows,Fn) {return {signature:rows.signature,sampleIds:rows.samples.map(s=>s.sampleId),normalForces:Float64Array.from(Fn),friction:'none'};}
function physicalColumns(f,positions=f.state.toolPositions) {
    return f.contacts.pairs[0].quadrature.map(s=>rawContact(f,{toolPositions:positions},[s]).side.gradients).map(g=>[...g.inner,...g.outer].flat());
}
const weighted=(columns,weights)=>Array.from({length:12},(_,j)=>columns.reduce((sum,column,i)=>sum+weights[i]*column[j],0));
function wrench(F,points,origin) {
    return [0,1].map(tool=>{const force=[0,0,0],moment=[0,0,0];for(let i=2*tool;i<2*tool+2;i++){
        const f=F.slice(3*i,3*i+3),r=points[i].map((v,k)=>v-origin[k]),m=[r[1]*f[2]-r[2]*f[1],r[2]*f[0]-r[0]*f[2],r[0]*f[1]-r[1]*f[0]];
        for(let k=0;k<3;k++){force[k]+=f[k];moment[k]+=m[k];}
    }return {force,moment};});
}

test('nonzero interior gauge preserves each physical nodal load, wrench and work but rebuilds endpoint stress tangents',()=>{
    const f=parallelFixture(),initial=rowsFor(f),history=historyFor(initial,[1,2,3]),rows=rowsFor(f,history),before=structuredClone(history);
    let queries=0;rows.prepareGauge({toolPositions:f.state.toolPositions,consumeQuery(){queries++;}});
    assert.deepEqual(Array.from(rows.normalForces),[2,0,4]);assert.deepEqual(history,before);assert.equal(queries,3);
    const columns=physicalColumns(f),F0=weighted(columns,[1,2,3]),F1=weighted(columns,Array.from(rows.normalForces));assert.deepEqual(F1,F0);
    const points=[...f.state.toolPositions.get('wire').slice(0,2),...f.state.toolPositions.get('catheter').slice(0,2)],delta=Array.from({length:12},(_,i)=>(i-5)*.125);
    assert.deepEqual(wrench(F1,points,[-.5,.75,1.25]),wrench(F0,points,[-.5,.75,1.25]));
    assert.equal(F1.reduce((s,v,i)=>s+v*delta[i],0),F0.reduce((s,v,i)=>s+v*delta[i],0));
    const previousDB=Array.from({length:144},(_,j)=>rows.samples.reduce((sum,s,i)=>sum+[1,2,3][i]*s.geometry.normalDerivative[j],0));
    const currentDB=Array.from({length:144},(_,j)=>rows.samples.reduce((sum,s,i)=>sum+rows.normalForces[i]*s.geometry.normalDerivative[j],0));
    assert.ok(Math.max(...previousDB.map((v,i)=>Math.abs(v-currentDB[i])))>=.5);
    const cg=new Float64Array(f.state.layout.dofCount),rg=new Float64Array(f.state.relative.length);
    rows.refresh({toolPositions:f.state.toolPositions,commonResidual:cg,relativeResidual:rg,order:'full',consumeQuery(){queries++;}});
    assert.equal(queries,6);assert.equal(rows.rows.length,2);
    for(const index of rows.rowSampleIndices){const sample=rows.samples[index];sample.row.geometricTangent.forEach((v,j)=>close(v,-rows.normalForces[index]*sample.pulled.normalDerivative[j],0));}
    assert.throws(()=>rows.prepareGauge({toolPositions:f.state.toolPositions,consumeQuery(){}}),/already prepared/);
});

test('the affine envelope is valid for varying normals, but a nonzero interior force without identical columns rejects',()=>{
    const f=parallelFixture(),positions=structuredClone(f.state.toolPositions);positions.set('wire',[[.25,.15,-.4],[2.25,.15,.4],[4.25,.15,1.2]]);
    const initial=rowsFor(f),loaded=historyFor(initial,[0,1,0]),before=structuredClone(loaded),bad=rowsFor(f,loaded);
    assert.throws(()=>bad.prepareGauge({toolPositions:positions,consumeQuery(){}}),/lacks an exact affine force-column transfer/);assert.deepEqual(loaded,before);
    const rows=rowsFor(f);rows.prepareGauge({toolPositions:positions,consumeQuery(){}});
    const cg=new Float64Array(f.state.layout.dofCount),rg=new Float64Array(f.state.relative.length);
    const proof=rows.refresh({toolPositions:positions,commonResidual:cg,relativeResidual:rg,order:'full',consumeQuery(){}});
    assert.equal(proof.converged,true);assert.equal(proof.samples.length,3);close(proof.samples[1].gap,.1,1e-14);
    assert.ok(proof.samples[1].gap>(proof.samples[0].gap+proof.samples[2].gap)/2);assert.equal(rows.rows.length,2);
    // Opposite normals: the exact interior radial-zero sample is still queried
    // and has a strictly positive original gap, requiring no invented normal.
    positions.set('wire',[[.25,-.5,0],[2.25,.5,0],[4.25,1.5,0]]);
    const opposed=rowsFor(f);opposed.prepareGauge({toolPositions:positions,consumeQuery(){}});
    const open=opposed.refresh({toolPositions:positions,commonResidual:cg.fill(0),relativeResidual:rg.fill(0),order:'full',consumeQuery(){}});
    assert.equal(open.converged,true);close(open.samples[1].gap,.25,0);assert.equal(open.samples[1].eliminated,true);
});

test('envelope extremes follow geometric s rather than input order and never bypass an unsupported declared sample',()=>{
    const f=parallelFixture();f.contacts.pairs[0].quadrature=[.5,.75,.25];
    const initial=rowsFor(f),rows=rowsFor(f,historyFor(initial,[2,3,1]));rows.prepareGauge({toolPositions:f.state.toolPositions,consumeQuery(){}});
    assert.deepEqual(rows.rowSampleIndices,[1,2]);assert.deepEqual(Array.from(rows.normalForces),[0,4,2]);
    const single=fixture();single.contacts.pairs[0].quadrature=[.5];const one=rowsFor(single);assert.equal(one.rows.length,1);
    const endpoint=fixture({xOffset:0});endpoint.contacts.pairs[0].quadrature=[.5,.75,0];
    const r=advance(endpoint,endpoint.state,.4);assert.equal(r.accepted,false);assert.equal(r.status,'unsupported-lumen-contact');
});

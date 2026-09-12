import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeJointTimeStepState,createCompositeJointTimeStepWorkspace,advanceCompositeJointTimeStep} from '../src/physics/kirchhoffCompositeJointTimeStep.js';
import {VesselContactField,createContactResult} from '../src/physics/collision/vesselContactField.js';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';

const dt=1/120;
const close=(a,b,t=1e-8)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}`);
const vector=(a,b,t)=>a.forEach((v,i)=>close(v,b[i],t));
const accepted=r=>assert.equal(r.accepted,true,JSON.stringify({status:r.status,error:r.error,diagnostics:r.diagnostics}));
function stateFor(catheter,wire=catheter.map(p=>p.map((v,i)=>v+(i===1?.25:0)))) {
    const n=catheter.length,coordinates=catheter.map((_,i)=>i*Math.hypot(...catheter[1].map((v,k)=>v-catheter[0][k])));
    const layout=createCompositeChainLayout(Array.from({length:n-1},()=>['wire','catheter']));
    const modes=catheter.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]}));
    const angles=new Map(),restLengths=new Map();
    const tools=['wire','catheter'].map(id=>{
        angles.set(id,new Float64Array(n-1));restLengths.set(id,new Float64Array(n-1).fill(coordinates[1]));
        return {id,dsDx:1,reference:captureCompositeReferenceFrames(id==='wire'?wire:catheter),referenceTwists:new Float64Array(n-2),
            material:compileCompositeMaterial({EI1:.001,GJ:.001})};
    });
    return createCompositeJointTimeStepState({layout,coordinates,positions:catheter,relative:wire.flatMap((p,i)=>p.map((v,k)=>v-catheter[i][k])),modes,angles,tools,restLengths,materialCoordinate:'reference-arclength'});
}
function inertiaFor(state,velocity=null) {
    return {previousPositions:structuredClone(state.toolPositions),inertiaEdges:state.layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,massPerMaterialLength:.01,
        materialMap:{sStart:state.coordinates[e],dsDx:1,dsDt:0},oldMaterialVelocities:velocity?.[id]
            ?[Array.from(velocity[id]),Array.from(velocity[id])]:structuredClone(state.materialVelocities?.[e].tools.find(t=>t.id===id).velocities??[[0,0,0],[0,0,0]])}))}))};
}
function inputs(state,wall,velocity=null) {
    return {dt,torsionMode:'quasi-static',contacts:'none',wall,inertia:inertiaFor(state,velocity),
        boundaries:{positions:[],spins:[{toolId:'wire',edge:0,value:0},{toolId:'catheter',edge:0,value:0}]}};
}
function planeFixture() {
    const state=stateFor([0,2,4].map(x=>[x,.82+.06*x,0])),queries=[];
    const field={queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out){
        queries.push({a:[ax,ay,az],b:[bx,by,bz],radius});const t=ay===by?.5:ay<by?0:1;
        out.signedDistance=(1-t)*ay+t*by;out.signedGap=out.signedDistance-radius;out.segmentT=t;
        out.inward.values.set([0,1,0]);out.closestPoint.values.set([(1-t)*ax+t*bx,0,(1-t)*az+t*bz]);
        out.faceIndex=1;out.branchId=0;out.source='analytic-plane';out.capsuleSampleCount=2;return out;
    }};
    const wall={mode:'wall-normal',friction:'none',chartId:'fixed-plane',field,forcePerLength:1,
        contactOwners:{edges:[0,1].map(edge=>({edge,wall:{owner:'catheter',radius:.8}}))}};
    return {state,wall,queries};
}
function physicalProof(r) {
    assert.ok(r.diagnostics.certificate.wall.converged);
    for(const [id,p] of r.state.toolPositions)for(let e=0;e<p.length-1;e++)close(Math.hypot(...p[e+1].map((v,k)=>v-p[e][k])),r.state.restLengths.get(id)[e]);
    for(const b of r.balances.values())vector(b.momentumRate,b.appliedForce.map((v,k)=>v+b.boundaryForce[k]+b.contactForce[k]),2e-7);
    assert.ok(r.state.wallContactState.normalForces.every(v=>v>=0));
}

test('wall normals stop the actual catheter axis in the shared step while an unloaded wire keeps its own motion',()=>{
    const f=planeFixture(),before=structuredClone(f.state),r=advanceCompositeJointTimeStep(f.state,inputs(f.state,f.wall,{wire:[1,0,0],catheter:[0,-3,0]}));
    accepted(r);physicalProof(r);assert.deepEqual(f.state,before);
    close(r.state.toolPositions.get('catheter')[0][1],.8);
    assert.ok(r.state.toolPositions.get('catheter').every(p=>p[1]>=.8-1e-8));
    r.state.toolPositions.get('wire').forEach((p,i)=>vector(p,f.state.toolPositions.get('wire')[i].map((v,k)=>v+(k===0?dt:0)),1e-9));
    vector(r.balances.get('wire').contactForce,[0,0,0],0);
    assert.ok(r.balances.get('catheter').contactForce[1]>0);
    assert.equal(f.queries.length,r.diagnostics.wallQueries);assert.equal(r.diagnostics.contactQueries,f.queries.length);
    assert.ok(f.queries.every(q=>q.radius===.8));assert.equal(r.state.step,1);close(r.state.time,dt,0);
});

test('wall histories survive the next dt and cold/reuse release, late rejection and retry preserve owned accepted state',()=>{
    const f=planeFixture(),workspace=createCompositeJointTimeStepWorkspace(f.state);
    const first=advanceCompositeJointTimeStep(f.state,{...inputs(f.state,f.wall,{wire:[0,0,0],catheter:[0,-3,0]}),workspace});accepted(first);
    const config=inputs(first.state,f.wall),cold=advanceCompositeJointTimeStep(first.state,config),warm=advanceCompositeJointTimeStep(first.state,{...config,workspace});
    accepted(cold);accepted(warm);physicalProof(warm);assert.deepEqual(warm.state,cold.state);assert.deepEqual(warm.balances,cold.balances);
    const before=structuredClone(first.state),late=advanceCompositeJointTimeStep(first.state,{...config,workspace,budget:{evaluations:cold.diagnostics.evaluations-1}});
    assert.equal(late.accepted,false);assert.equal(late.status,'evaluation-budget');assert.equal(late.state,first.state);assert.deepEqual(first.state,before);
    const retry=advanceCompositeJointTimeStep(first.state,{...config,workspace});accepted(retry);assert.deepEqual(retry.state,warm.state);
    const release=advanceCompositeJointTimeStep(warm.state,inputs(warm.state,f.wall,{wire:[0,0,0],catheter:[0,1,0]}));accepted(release);physicalProof(release);
    assert.ok(release.state.wallContactState.normalForces.every(v=>v===0));assert.ok(release.state.toolPositions.get('catheter')[0][1]>.8);
    assert.throws(()=>advanceCompositeJointTimeStep(first.state,{...config,wall:'none'}),/cannot discard/);
});

test('original wall queries share the contact budget and rejected physical attempts leave incoming state untouched',()=>{
    const f=planeFixture(),before=structuredClone(f.state),r=advanceCompositeJointTimeStep(f.state,{...inputs(f.state,f.wall),budget:{contactQueries:1}});
    assert.equal(r.accepted,false);assert.equal(r.status,'contact-query-budget');assert.equal(r.diagnostics.contactQueries,1);assert.equal(r.state,f.state);assert.deepEqual(f.state,before);
});

test('wire pressure, lumen reactions and vessel support solve together and balance each material independently',()=>{
    const f=planeFixture(),cat=[0,2,4].map(x=>[x,.8+.06*x,0]),h=Math.hypot(1,.06),t=[1/h,.06/h,0],normal=[-.06/h,1/h,0];
    const wire=cat.map(p=>p.map((v,k)=>v+.1*t[k]-.34*normal[k])),state=stateFor(cat,wire);
    const contacts={mode:'lumen-normal',friction:'none',chartId:'two-loaded-interfaces',forcePerLength:1,
        pairs:[{id:'first',innerToolId:'wire',outerToolId:'catheter',innerEdge:0,outerEdge:0,innerMaterialSegmentId:'wire0',outerMaterialSegmentId:'catheter0',
            lumenRadius:.5,innerRadius:.16,quadrature:[.25,.5,.75],openDistal:false,portalFilletRadius:0}]};
    const config={...inputs(state,f.wall),contacts,loads:{forces:[{toolId:'wire',node:0,value:[0,-.4,0]}],torques:[]}};
    const before=structuredClone(state),r=advanceCompositeJointTimeStep(state,config);accepted(r);physicalProof(r);assert.deepEqual(state,before);
    assert.ok(r.diagnostics.certificate.contact.converged);assert.ok(r.state.lumenContactState.normalForces.some(v=>v>0));
    assert.ok(r.state.wallContactState.normalForces.some(v=>v>0));
    const resultant=[0,0,0];for(const b of r.balances.values())b.contactForce.forEach((v,k)=>resultant[k]+=v);
    const wallForce=r.state.wallContactState.normalForces.reduce((sum,v)=>sum+v,0);
    vector(resultant,[0,wallForce,0],1e-12);
    assert.equal(r.diagnostics.contactQueries,r.diagnostics.wallQueries+3*r.diagnostics.evaluations);
});

test('a flat capsule switching its loaded sample stays explicitly unsupported and cannot commit a partial step',()=>{
    const f=planeFixture(),state=stateFor([0,2,4].map(x=>[x,.82,0])),before=structuredClone(state);
    const r=advanceCompositeJointTimeStep(state,inputs(state,f.wall,{wire:[0,0,0],catheter:[0,-10,0]}));
    assert.equal(r.accepted,false);assert.equal(r.state,state);assert.deepEqual(state,before);
    assert.match(r.diagnostics.lastInvalidTrial,/source\/sample\/feature changed/);
});

test('the endpoint envelope resolves the same flat capsule load and subsequent hold/release in full physical steps',()=>{
    const f=planeFixture(),state=stateFor([0,2,4].map(x=>[x,.82,0])),wall={...f.wall,contactMode:'envelope'},workspace=createCompositeJointTimeStepWorkspace(state);
    const config=inputs(state,wall,{wire:[0,0,0],catheter:[0,-10,0]}),before=structuredClone(state);
    const first=advanceCompositeJointTimeStep(state,{...config,workspace});accepted(first);physicalProof(first);assert.deepEqual(state,before);
    first.state.toolPositions.get('catheter').forEach(p=>close(p[1],.8));
    first.state.toolPositions.get('wire').forEach((p,i)=>vector(p,state.toolPositions.get('wire')[i],1e-9));
    close(first.balances.get('catheter').contactForce[1],.01*4*(10-.02/dt)/dt,1e-7);
    assert.equal(first.diagnostics.certificate.wall.contactMode,'envelope');
    assert.equal(first.diagnostics.certificate.wall.samples.length,6);
    assert.equal(first.diagnostics.certificate.wall.retainedRows,3);
    assert.ok(first.diagnostics.certificate.wall.samples.every(s=>s.role!=='capsule'||s.Fn===0));
    assert.equal(first.diagnostics.wallQueries,5*first.diagnostics.evaluations);
    const cold=advanceCompositeJointTimeStep(state,config);accepted(cold);assert.deepEqual(cold.state,first.state);assert.deepEqual(cold.balances,first.balances);
    const late=advanceCompositeJointTimeStep(state,{...config,workspace,budget:{evaluations:first.diagnostics.evaluations-1}});
    assert.equal(late.accepted,false);assert.equal(late.state,state);assert.deepEqual(state,before);
    const retry=advanceCompositeJointTimeStep(state,{...config,workspace});accepted(retry);assert.deepEqual(retry.state,first.state);
    const next=advanceCompositeJointTimeStep(first.state,{...inputs(first.state,wall),workspace});accepted(next);physicalProof(next);
    next.state.toolPositions.get('catheter').forEach(p=>close(p[1],.8));
    const released=advanceCompositeJointTimeStep(next.state,{...inputs(next.state,wall,{wire:[0,0,0],catheter:[0,1,0]}),workspace});
    accepted(released);physicalProof(released);released.state.toolPositions.get('catheter').forEach(p=>close(p[1],.8+dt));
    assert.ok(released.state.wallContactState.normalForces.every(v=>v===0));assert.equal(released.state.step,3);
    assert.throws(()=>advanceCompositeJointTimeStep(first.state,inputs(first.state,f.wall)),/provenance changed/);
});

function sdfFixture() {
    const bytes=fs.readFileSync(new URL('../res/Aorta_plain.collision.bin',import.meta.url));
    const field=new VesselContactField(decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength)));
    const a=[65.00287246704102,-462.00980948623305,-79.56869888305664],b=[64.95548751831055,-461.7916869276393,-79.65612350463867];
    const cat=[a,b,b.map((v,i)=>2*v-a[i])],wire=cat.map(p=>p.map((v,i)=>v+(i===2?.1:0))),state=stateFor(cat,wire);
    const wall={mode:'wall-normal',friction:'none',chartId:'real-Aorta-P1',field,forcePerLength:1,
        contactOwners:{edges:[{edge:0,wall:{owner:'catheter',radius:.4445}},{edge:1,wall:null}]}};
    return {state,wall};
}

test('real Aorta P1 admits both SDF normal reactions in the whole joint dt with the separate wire unchanged',()=>{
    const f=sdfFixture(),before=structuredClone(f.state),workspace=createCompositeJointTimeStepWorkspace(f.state);
    const r=advanceCompositeJointTimeStep(f.state,{...inputs(f.state,f.wall),workspace});accepted(r);physicalProof(r);assert.deepEqual(f.state,before);
    assert.equal(r.state.wallContactState.records.filter(v=>v.seam!==null).length,1);
    assert.equal(r.state.wallContactState.normalForces[0],0);assert.ok(r.state.wallContactState.normalForces[1]>0);assert.ok(r.state.wallContactState.normalForces[2]>0);
    f.state.toolPositions.get('wire').forEach((p,i)=>vector(r.state.toolPositions.get('wire')[i],p,1e-9));
    const p=r.state.toolPositions.get('catheter'),raw=f.wall.field.queryCapsuleCoordinates(...p[0],...p[1],.4445,createContactResult());
    assert.ok(raw.signedGap>=-1e-8);assert.ok(r.diagnostics.wallChartDiscoveries>0);assert.ok(r.diagnostics.wallRowRebuilds>0);
    const retryState=structuredClone(f.state),failed=advanceCompositeJointTimeStep(f.state,{...inputs(f.state,f.wall),workspace,budget:{directions:1}});
    assert.equal(failed.accepted,false);assert.equal(failed.state,f.state);assert.deepEqual(f.state,retryState);
    const again=advanceCompositeJointTimeStep(f.state,{...inputs(f.state,f.wall),workspace});accepted(again);assert.deepEqual(again.state,r.state);
    const next=advanceCompositeJointTimeStep(r.state,{...inputs(r.state,f.wall),workspace});accepted(next);physicalProof(next);assert.equal(next.state.step,2);
});

import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeJointTimeStepState,createCompositeJointTimeStepWorkspace,advanceCompositeJointTimeStep} from '../src/physics/kirchhoffCompositeJointTimeStep.js';
import {evaluateKirchhoffLumenSegmentContact} from '../src/physics/kirchhoffLumenContact.js';
import {createCompositeJointLumenRows} from '../src/physics/kirchhoffCompositeJointLumenRows.js';

const dt=1/120,zero=[0,0,0];
const close=(a,b,t=1e-8)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b} (tol ${t})`);
const near=(a,b,t=1e-8)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
function fixture(feature) {
    const coordinates=[0,2,4],catheter=[[0,0,0],[2,0,0],[4,0,0]];
    const wire=feature==='distal-fillet'?[[1.9,.38,0],[2.1,.38,0],[2.3,.38,0]]:[[1.97,.505,0],[2.03,.511,0],[2.09,.517,0]];
    const layout=createCompositeChainLayout([['wire','catheter'],['wire']]);
    const positions=[...catheter.slice(0,2),wire[2]],modes=[0,1].map(node=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]}));
    const relative=wire.slice(0,2).flatMap((p,i)=>p.map((v,k)=>v-positions[i][k])),angles=new Map(),restLengths=new Map();
    const tools=['wire','catheter'].map(id=>{
        const p=id==='wire'?wire:catheter,dsDx=Math.hypot(...p[1].map((v,k)=>v-p[0][k]))/2;
        angles.set(id,new Float64Array(2));restLengths.set(id,new Float64Array(2).fill(2*dsDx));
        return {id,dsDx,reference:captureCompositeReferenceFrames(p),referenceTwists:new Float64Array(1),
            material:compileCompositeMaterial({stiffness:[[2,0,0],[0,3,0],[0,0,1]],intrinsic:zero})};
    });
    const state=createCompositeJointTimeStepState({layout,coordinates,positions,relative,modes,angles,tools,restLengths,materialCoordinate:'reference-arclength'});
    const pair={id:'tip',feature,innerToolId:'wire',outerToolId:'catheter',innerEdge:0,outerEdge:0,
        innerMaterialSegmentId:'wire:0',outerMaterialSegmentId:'catheter:tip',lumenRadius:.5,innerRadius:.16,openDistal:true,portalFilletRadius:.15};
    if(feature==='distal-fillet')pair.quadrature=[.25];
    const contacts={mode:'lumen-normal',friction:'none',chartId:'actual-tip-fixed-chart',forcePerLength:1,pairs:[pair]};
    const boundaries={positions:catheter.slice(0,2).map((value,node)=>({toolId:'catheter',node,value})),
        spins:[{toolId:'wire',edge:0,value:0},{toolId:'catheter',edge:0,value:0}]};
    return {state,contacts,boundaries};
}
function inertia(state,velocity) {
    return {previousPositions:structuredClone(state.toolPositions),inertiaEdges:state.layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,
        massPerMaterialLength:id==='wire'?.13:.24,materialMap:{sStart:20+state.coordinates[e]*state.tools.find(t=>t.id===id).dsDx,
            dsDx:state.tools.find(t=>t.id===id).dsDx,dsDt:0},
        oldMaterialVelocities:velocity?[velocity[id],velocity[id]].map(v=>v.slice()):structuredClone(state.materialVelocities?.[e].tools.find(t=>t.id===id).velocities??[zero,zero])}))}))};
}
function advance(f,state=f.state,extra={}) {
    return advanceCompositeJointTimeStep(state,{dt,torsionMode:'quasi-static',contacts:f.contacts,boundaries:f.boundaries,inertia:inertia(state),...extra});
}
function accepted(r){assert.equal(r.accepted,true,JSON.stringify({status:r.status,error:r.error,diagnostics:r.diagnostics}));}
function original(f,state) {
    const pair=f.contacts.pairs[0],p=state.toolPositions;
    return evaluateKirchhoffLumenSegmentContact({...pair,activationDistance:Number.MAX_VALUE,quadrature:pair.quadrature??[.5],
        innerStart:p.get('wire')[0],innerEnd:p.get('wire')[1],outerStart:p.get('catheter')[0],outerEnd:p.get('catheter')[1]});
}
function prove(f,r) {
    const raw=original(f,r.state),feature=f.contacts.pairs[0].feature,record=feature==='distal-fillet'?raw.fillet:raw.portal.contact;
    const proof=r.diagnostics.certificate.contact,Fn=r.state.lumenContactState.normalForces[0];
    if(record){close(proof.samples[0].gap,record.gap,1e-14);assert.ok(record.gap>=-1e-8);assert.ok(Math.abs(Fn*record.gap)<=1e-9);}
    else {assert.equal(raw.portal.crosses,false);assert.equal(proof.samples[0].applicable,false);assert.equal(proof.samples[0].gap,null);assert.ok(Fn===0);}
    assert.ok(Fn>=0);
    // These bounded fixtures must also stay clear of the other original
    // detector features; a passing declared row must not hide their overlap.
    for(const other of [raw.side,raw.fillet,raw.portal.contact])if(other)assert.ok(other.gap>=-1e-8,`${other.kind}: ${other.gap}`);
    assert.equal(proof.retainedRows,1);assert.equal(proof.redundantSamples,0);
    for(const [id,p] of r.state.toolPositions)for(let e=0;e<2;e++)if(r.state.layout.edgeToolIds[e].includes(id))
        close(Math.hypot(...p[e+1].map((v,k)=>v-p[e][k])),r.state.restLengths.get(id)[e]);
    for(const b of r.balances.values())near(b.momentumRate,b.appliedForce.map((v,k)=>v+b.boundaryForce[k]+b.contactForce[k]),1e-7);
    const force=[0,0,0],moment=[0,0,0];
    for(const [id,loads] of r.contactForces)loads.forEach((f,node)=>{
        f.forEach((v,k)=>force[k]+=v);cross(r.state.toolPositions.get(id)[node],f).forEach((v,k)=>moment[k]+=v);
    });
    near(force,zero,1e-12);near(moment,zero,1e-11);
    close(Math.hypot(...r.balances.get('wire').contactForce),Fn,1e-10);
    assert.equal(r.diagnostics.contactQueries,r.diagnostics.evaluations);
    return {raw,record,Fn};
}

for(const feature of ['distal-fillet','distal-rim'])test(`${feature} acts in one whole dt on the actual catheter tip with balanced physical force and moment`,()=>{
    const f=fixture(feature),before=structuredClone(f.state),r=advance(f);accepted(r);const p=prove(f,r);
    assert.ok(p.Fn>0);assert.deepEqual(f.state,before);assert.equal(r.state.time,dt);assert.equal(r.state.step,1);
    if(feature==='distal-fillet')assert.ok(Math.hypot(...r.contactForces.get('catheter')[0])>1e-4);
    else {assert.ok(r.diagnostics.certificate.contact.samples[0].forceScale>1);assert.notEqual(p.record.innerT,.5);}
});

for(const feature of ['distal-fillet','distal-rim'])test(`${feature} keeps owned history through a loaded next dt, cold/reuse, late rejection and release`,()=>{
    const f=fixture(feature),workspace=createCompositeJointTimeStepWorkspace(f.state),a=advance(f,f.state,{workspace});accepted(a);prove(f,a);
    const press=feature==='distal-fillet'?[-3.5,3.5,0]:[0,5,0],loads={forces:[{toolId:'wire',node:0,value:press}]};
    const cold=advance(f,a.state,{loads}),b=advance(f,a.state,{loads,workspace});accepted(cold);accepted(b);prove(f,b);
    assert.ok(b.state.lumenContactState.normalForces[0]>0);
    for(const k of ['state','perTool','boundaryForces','contactForces','spinReactions','balances'])assert.deepEqual(b[k],cold[k],k);
    const before=structuredClone(a.state),late=advance(f,a.state,{loads,workspace,budget:{evaluations:b.diagnostics.evaluations-1}});
    assert.equal(late.accepted,false);assert.equal(late.state,a.state);assert.deepEqual(a.state,before);
    const retry=advance(f,a.state,{loads,workspace});accepted(retry);assert.deepEqual(retry.state,b.state);
    const release=advance(f,b.state,{workspace,loads:{forces:[{toolId:'wire',node:0,value:press.map(v=>-v)}]}});accepted(release);prove(f,release);
    assert.ok(release.state.lumenContactState.normalForces[0]===0);assert.ok(release.diagnostics.certificate.contact.minGap>0);
    assert.equal(release.state.step,3);close(release.state.time,3*dt,1e-14);
});

function normalRows(f,history=null) {
    return createCompositeJointLumenRows({...f.state,contacts:f.contacts,history,
        tolerances:{force:1e-7,lumenGap:1e-8,lumenNcp:1e-8,lumenWork:1e-9,linearConstraint:5e-11}});
}
function residuals(state) {return {commonResidual:new Float64Array(state.layout.dofCount),relativeResidual:new Float64Array(state.relative.length),order:'full',consumeQuery(){}};}

test('each declared fillet sample keeps its own normal row; failed later queries publish no partial force or stale certificate',()=>{
    const f=fixture('distal-fillet'),a=advance(f);accepted(a);
    f.contacts.pairs[0].quadrature=[.25,.5];const rows=normalRows(f),physical=a.state.toolPositions;
    assert.equal(rows.rows.length,2);assert.equal(rows.gauge.redundantSamples,0);
    rows.normalForces[0]=a.state.lumenContactState.normalForces[0];
    rows.prepareGauge({toolPositions:physical,consumeQuery(){}});
    assert.ok(rows.refresh({toolPositions:physical,...residuals(f.state)}).converged);const history=rows.commit();
    const moved=structuredClone(physical);moved.get('wire')[1][0]=2.6;
    const output=residuals(f.state);output.commonResidual.fill(2);output.relativeResidual.fill(-3);
    const before={commonResidual:output.commonResidual.slice(),relativeResidual:output.relativeResidual.slice()};
    let queries=0;output.consumeQuery=()=>queries++;
    assert.throws(()=>rows.refresh({toolPositions:moved,...output}),/left its original/);assert.equal(queries,2);
    assert.deepEqual(output.commonResidual,before.commonResidual);assert.deepEqual(output.relativeResidual,before.relativeResidual);
    assert.ok([...rows.nodalForces.values()].every(p=>p.every(v=>v.every(x=>x===0))));
    assert.ok(rows.rows.every(row=>!row.geometricTangentValid&&Number.isNaN(row.residual)));
    assert.throws(()=>rows.commit(),/uncertified/);assert.deepEqual(rows.normalForces,history.normalForces);
    assert.ok(rows.refresh({toolPositions:physical,...residuals(f.state)}).converged);assert.deepEqual(rows.commit(),history);
});

test('only an unloaded original rim may leave its crossing and re-enter without an invented gap or a discarded force',()=>{
    const f=fixture('distal-rim'),rows=normalRows(f),absent=structuredClone(f.state.toolPositions);
    absent.get('wire').forEach(p=>p[0]+=.2);rows.prepareGauge({toolPositions:absent,consumeQuery(){}});
    const proof=rows.refresh({toolPositions:absent,...residuals(f.state)});assert.ok(proof.converged);
    assert.equal(proof.samples[0].gap,null);assert.equal(proof.samples[0].applicable,false);const history=rows.commit();
    rows.normalForces[0]=1;assert.throws(()=>rows.refresh({toolPositions:absent,...residuals(f.state)}),/left its original/);
    assert.throws(()=>rows.commit(),/uncertified/);assert.equal(rows.normalForces[0],1);
    rows.normalForces.set(history.normalForces);const entering=structuredClone(f.state.toolPositions);entering.get('wire').forEach(p=>p[1]-=.1);
    const renewed=rows.refresh({toolPositions:entering,...residuals(f.state)});assert.ok(renewed.converged);
    assert.equal(renewed.samples[0].applicable,true);assert.ok(renewed.minGap>0);assert.deepEqual(rows.commit().sampleIds,history.sampleIds);
    const bad=structuredClone(f.contacts);bad.pairs[0].quadrature=[.5];assert.throws(()=>normalRows({...f,contacts:bad}),/exact moving crossing/);
    bad.pairs[0].quadrature=undefined;bad.pairs[0].openDistal=false;assert.throws(()=>normalRows({...f,contacts:bad}),/open distal/);
});

test('a certified normal law cannot commit changed forces or changed physical geometry under its old proof',()=>{
    const f=fixture('distal-rim'),rows=normalRows(f),physical=structuredClone(f.state.toolPositions);
    physical.get('wire').forEach(p=>p[1]-=.018);rows.normalForces[0]=1;
    rows.prepareGauge({toolPositions:physical,consumeQuery(){}});
    let queries=0;const input={toolPositions:physical,...residuals(f.state),consumeQuery(){queries++;}};
    assert.ok(rows.refresh(input).converged);const history=rows.commit();assert.equal(queries,1);
    for(const bad of [-1e-30,NaN,2]) {
        rows.normalForces[0]=bad;assert.throws(()=>rows.commit(),/changed after their last certified refresh/);
        rows.normalForces[0]=1;
    }
    const y=physical.get('wire')[0][1];physical.get('wire')[0][1]+=.01;
    assert.throws(()=>rows.commit(),/changed after their last certified refresh/);
    physical.get('wire')[0][1]=y;assert.deepEqual(rows.commit(),history);assert.equal(queries,1);
    // A normal refresh is sufficient to obtain a fresh proof after a change;
    // commit itself neither queries nor silently reconstructs the old forces.
    rows.normalForces[0]=2;assert.ok(rows.refresh(input).converged);assert.equal(rows.commit().normalForces[0],2);assert.equal(queries,2);
    physical.get('wire').forEach(p=>p[1]+=.01);
    const invalid=rows.refresh(input);assert.equal(invalid.converged,false);assert.ok(invalid.minGap<0);
    invalid.converged=true;invalid.minGap=0;invalid.penetration=invalid.ncp=invalid.complementarity=0;
    assert.throws(()=>rows.commit(),/uncertified/);assert.equal(queries,3);
});

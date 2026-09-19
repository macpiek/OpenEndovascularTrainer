import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {Quaternion,Vector3} from 'three';
import {createSharedAxisAppSystem,sampleSharedAxisPosition,advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {EndovascularPhysicsWorld,DEFAULT_TOOL_PROFILES} from '../src/physics/endovascularPhysicsWorld.js';
import {createCoupledSolverSelection,resolveAppCoupledSolver} from '../src/physics/coupledSolverSelection.js';
import {createFixedStepTransaction} from '../src/physics/fixedStepTransaction.js';
import {createPreparedInputCheckpoint} from '../src/physics/preparedInputCheckpoint.js';

const dt=1/120;
function fixture({workSliceMs=0,origin=[127,-83,29],adaptiveMesh=null,projectiveDynamics=false,coupledFrictionNewton=false,predictiveNewton=true,onRejectedStep=null,physicsOptions}={}) {
    const controls={reads:0,queries:0,throwQuery:false,failQueries:0};
    const world=new EndovascularPhysicsWorld({fixedDt:dt});
    const tools=['wire','catheter'].map(id=>{
        const body=world.createRod(id,13,5,DEFAULT_TOOL_PROFILES[id==='wire'?'guidewire':id]);
        return {id,body,insertion:0,rotation:0,type:id==='wire'?'glidewire':'straight',shaftStiffness:1,tipStiffness:1,
            nodeCoordinates:Array.from({length:13},(_,i)=>-40+5*i)};
    });
    world.contactField={voxelSize:1,queryCapsuleSoA(x,y,z,r,edge,out){
        controls.queries++;
        if(controls.failQueries>0){controls.failQueries--;throw new Error('synthetic one-off contact query failure');}
        if(controls.throwQuery)throw new Error('synthetic contact query failed');
        Object.assign(out,{signedDistance:100,signedGap:100-r[edge],segmentT:.5,faceIndex:0});return out;
    }};
    const system=createSharedAxisAppSystem({workSliceMs,adaptiveMesh,projectiveDynamics,coupledFrictionNewton,predictiveNewton,onRejectedStep,physicsOptions,
        readTools(){controls.reads++;return tools.map(t=>({...t,nodeCoordinates:t.nodeCoordinates.slice()}));},
        readSheath:()=>({start:origin,end:[origin[0]+10,origin[1],origin[2]],innerRadius:2,proximalExtension:40})});
    world.wholeStepSystem=system;
    return {system,world,tools,controls,origin};
}
function finish(f,{maxSlices=1000,onPending=()=>{}}={}) {
    for(let i=0;i<maxSlices;i++) {
        const r=f.system.step(f.world,dt);
        if(r.accepted)return r;
        assert.equal(r.status,'shared-axis-pending',JSON.stringify(r));onPending(r);
    }
    assert.fail('Shared-axis provider did not complete its bounded synthetic timestep');
}
const positions=body=>Array.from({length:body.count},(_,i)=>[body.x[i],body.y[i],body.z[i]]);
const frame=body=>new Quaternion(body.orientationX[0],body.orientationY[0],body.orientationZ[0],body.orientationW[0]);
const close=(a,b,tol=1e-6)=>assert.ok(Math.abs(a-b)<tol,`${a} vs ${b}`);

test('adaptive provider publishes actual mechanical nodes atomically and clears mesh diagnostics on reset',()=>{
    const f=fixture({adaptiveMesh:true});finish(f);
    assert.equal(f.system.diagnostics.solver,'shared-axis-adaptive');
    assert.equal(f.system.diagnostics.mesh.adaptive,true);
    assert.equal(f.system.diagnostics.mesh.nodes,f.tools[0].body.jointStateView.coordinates.length);
    const views=f.tools.map(t=>t.body.jointStateView);
    f.tools[0].insertion=3;f.tools[1].insertion=1;
    finish(f,{onPending:()=>f.tools.forEach((t,i)=>assert.equal(t.body.jointStateView,views[i]))});
    assert.deepEqual(f.tools.map(t=>t.body.jointStateView.coordinates.at(-1)),[3,1]);
    f.system.reset();assert.equal(f.system.diagnostics.mesh,null);
    assert.ok(f.tools.every(t=>t.body.jointStateView===null));
});

test('adaptive tolerance changes preserve a pending step and wake an unchanged sleeping state',()=>{
    const f=fixture({adaptiveMesh:true});finish(f);
    const initializations=f.system.diagnostics.initializations;
    f.tools[0].insertion=3;f.tools[1].insertion=1;
    assert.equal(f.system.step(f.world,dt).pending,true);
    f.system.setAdaptiveShapeTolerance(.45);
    finish(f);
    assert.equal(f.system.diagnostics.mesh.shapeTolerance,.15,'pending step retains its original budget');
    finish(f);
    assert.equal(f.system.diagnostics.mesh.shapeTolerance,.45);
    assert.deepEqual(f.tools.map(t=>t.body.jointStateView.coordinates.at(-1)),[3,1]);
    let result;
    for(let i=0;i<100;i++){result=finish(f);if(result.status==='sleeping')break;}
    assert.equal(result.status,'sleeping');
    f.system.setAdaptiveShapeTolerance(.01);
    assert.equal(finish(f).status,'converged','a new budget invalidates sleep');
    assert.equal(f.system.diagnostics.mesh.shapeTolerance,.01);
    assert.equal(f.system.diagnostics.initializations,initializations,'changing the budget does not reset tools');
    assert.throws(()=>f.system.setAdaptiveShapeTolerance(NaN),/Invalid/);
    assert.throws(()=>f.system.setAdaptiveShapeTolerance(0),/Invalid/);
    assert.equal(fixture().system.setAdaptiveShapeTolerance(.5),false,'reference mesh is unchanged');
});

test('contact margin applies after a pending solve and wakes a sleeping adaptive provider',()=>{
    const f=fixture({adaptiveMesh:true});finish(f);
    f.tools[0].insertion=3;
    assert.equal(f.system.step(f.world,dt).pending,true);
    f.system.setAdaptiveContactMargin(0);
    finish(f);
    assert.equal(f.system.diagnostics.mesh.contactMargin,1);
    finish(f);
    assert.equal(f.system.diagnostics.mesh.contactMargin,0);
    assert.equal(f.tools[0].body.jointStateView.coordinates.at(-1),3);
    let result;
    for(let i=0;i<100;i++){result=finish(f);if(result.status==='sleeping')break;}
    assert.equal(result.status,'sleeping');
    f.system.setAdaptiveContactMargin(.25);
    assert.equal(finish(f).status,'converged');
    assert.equal(f.system.diagnostics.mesh.contactMargin,.25);
    assert.equal(f.system.diagnostics.initializations,1);
    assert.throws(()=>f.system.setAdaptiveContactMargin(-1),/Invalid/);
    assert.throws(()=>f.system.setAdaptiveContactMargin(Infinity),/Invalid/);
    assert.equal(fixture().system.setAdaptiveContactMargin(0),false);
});

test('arc-loss and spacing controls are frozen during a pending step and wake sleeping physics',()=>{
    const f=fixture({adaptiveMesh:true});finish(f);
    f.tools[0].insertion=3;
    assert.equal(f.system.step(f.world,dt).pending,true);
    f.system.setAdaptiveMaxArcLoss(.05);
    f.system.setAdaptiveMaxSpacing(50);
    finish(f);
    assert.equal(f.system.diagnostics.mesh.maxArcLoss,.002);
    assert.equal(f.system.diagnostics.mesh.maxAllowedSpacing,20);
    finish(f);
    assert.equal(f.system.diagnostics.mesh.maxArcLoss,.05);
    assert.equal(f.system.diagnostics.mesh.maxAllowedSpacing,50);
    assert.equal(f.tools[0].body.jointStateView.coordinates.at(-1),3);
    for(const [setter,value,field] of [
        ['setAdaptiveMaxArcLoss',0,'maxArcLoss'],['setAdaptiveMaxSpacing',5,'maxAllowedSpacing']
    ]) {
        let result;
        for(let i=0;i<100;i++){result=finish(f);if(result.status==='sleeping')break;}
        assert.equal(result.status,'sleeping');
        f.system[setter](value);
        assert.equal(finish(f).status,'converged');
        assert.equal(f.system.diagnostics.mesh[field],value);
    }
    assert.equal(f.system.diagnostics.initializations,1);
    assert.throws(()=>f.system.setAdaptiveMaxSpacing(4),/spacing/);
    assert.throws(()=>f.system.setAdaptiveMaxSpacing(NaN),/Invalid/);
    assert.throws(()=>f.system.setAdaptiveMaxArcLoss(1),/Invalid/);
    assert.equal(fixture().system.setAdaptiveMaxArcLoss(.1),false);
    assert.equal(fixture().system.setAdaptiveMaxSpacing(50),false);
});

test('shared-axis selection installs its whole-step provider without the split coupled kernel',()=>{
    const f=fixture(),selection=createCoupledSolverSelection('shared-axis',{wholeStepSystem:f.system});
    assert.equal(resolveAppCoupledSolver('?coupledSolver=shared-axis'),'shared-axis');
    assert.equal(resolveAppCoupledSolver(''),'shared-axis');
    assert.equal(selection.wholeStepSystem,f.system);assert.equal(selection.coupledSystem,null);
    assert.equal(selection.getReport(f.world).installed,true);
    assert.throws(()=>createCoupledSolverSelection('shared-axis'),/whole-step/);
});

test('pending global solves publish both tools atomically and freeze prepared controls',()=>{
    const f=fixture(),before=f.tools.map(t=>positions(t.body));
    f.tools[0].insertion=3;f.tools[1].insertion=1;
    const first=f.system.step(f.world,dt);assert.equal(first.status,'shared-axis-pending');
    assert.equal(first.pending,true);
    assert.equal(f.controls.reads,1);
    // These commands belong to the following timestep, not this pending one.
    f.tools[0].insertion=4;f.tools[1].insertion=2;
    const result=finish(f,{onPending:()=>{
        f.tools.forEach((t,i)=>assert.deepEqual(positions(t.body),before[i]));
        assert.ok(f.tools.every(t=>!t.body.jointStateView));
    }});
    assert.equal(result.status,'converged');assert.equal(f.controls.reads,1);
    assert.equal(result.diagnostics.last.wallNormalFallbacks,0);
    assert.equal(result.diagnostics.last.quality.finite,true);
    assert.deepEqual(f.tools.map(t=>t.body.jointStateView.coordinates.at(-1)),[3,1]);
    const saved=f.tools.map(t=>({view:t.body.jointStateView,points:positions(t.body)}));
    finish(f,{onPending:()=>f.tools.forEach((t,i)=>{
        assert.equal(t.body.jointStateView,saved[i].view);assert.deepEqual(positions(t.body),saved[i].points);
    })});
    assert.deepEqual(f.tools.map(t=>t.body.jointStateView.coordinates.at(-1)),[4,2]);
    assert.equal(f.controls.reads,2);
});

test('World consumes one prepared dt only after cooperative completion',()=>{
    const f=fixture();let preparations=0;
    assert.equal(f.world.advance(dt,()=>{preparations++;f.tools[0].insertion=.5;}),0);
    assert.equal(f.world.lastStepResult.pending,true,'World preserves the explicit cooperative progress contract');
    assert.equal(f.world.stepCount,0);close(f.world.accumulator,dt);
    for(let attempts=0;!f.world.lastStepResult.accepted&&attempts<1000;attempts++)
        f.world.advance(0,()=>{preparations++;});
    assert.equal(f.world.lastStepResult.accepted,true);assert.equal(f.world.stepCount,1);
    assert.equal(preparations,1);assert.equal(f.controls.reads,1);close(f.world.accumulator,0);
});

test('independent feed and proximal spin share exactly one overlap curve in world coordinates',()=>{
    const f=fixture();finish(f);
    const before=f.tools.map(t=>frame(t.body));
    f.tools[0].insertion=2;f.tools[1].insertion=.75;
    f.tools[0].rotation=.1;f.tools[1].rotation=-.07;
    finish(f);
    for(let i=0;i<2;i++) {
        const t=f.tools[i],expected=new Quaternion().setFromAxisAngle(new Vector3(1,0,0),t.rotation).multiply(before[i]);
        close(Math.abs(frame(t.body).dot(expected)),1,1e-6);
        assert.equal(t.body.jointStateView.coordinates.at(-1),t.insertion);
        const view=t.body.jointStateView;
        close(view.positions[0],f.origin[0]-40);close(view.positions[1],f.origin[1]);close(view.positions[2],f.origin[2]);
        assert.ok(positions(t.body).flat().every(Number.isFinite));
    }
    const [wire,catheter]=f.tools.map(t=>t.body.jointStateView);
    for(let i=0;i<catheter.coordinates.length;i++) {
        const j=Array.from(wire.coordinates).indexOf(catheter.coordinates[i]);assert.ok(j>=0);
        assert.deepEqual(catheter.positions.slice(i*3,i*3+3),wire.positions.slice(j*3,j*3+3));
    }
    f.tools[1].insertion=.5;finish(f);
    assert.deepEqual(f.tools.map(t=>t.body.jointStateView.coordinates.at(-1)),[2,.5]);
});

test('sampling translates local coordinates once and extrapolates the exposed proximal reservoir',()=>{
    const s={origin:[100,-20,300],coordinates:[-5,0,5],positions:[[-5,0,0],[0,0,0],[3,4,0]]};
    assert.deepEqual(sampleSharedAxisPosition(s,-10),[90,-20,300]);
    assert.deepEqual(sampleSharedAxisPosition(s,2.5),[101.5,-18,300]);
    assert.deepEqual(sampleSharedAxisPosition(s,10),[106,-12,300]);
});

test('reset abandons private work and clears both published curves and solver counters',()=>{
    const f=fixture();finish(f);
    f.tools[0].insertion=3;assert.equal(f.system.step(f.world,dt).status,'shared-axis-pending');
    const before=f.tools.map(t=>positions(t.body));f.system.reset();
    f.tools.forEach((t,i)=>{assert.equal(t.body.jointStateView,null);assert.deepEqual(positions(t.body),before[i]);});
    for(const key of ['initializations','acceptedSteps','pendingSlices','failedSteps'])assert.equal(f.system.diagnostics[key],0);
    assert.equal(f.system.diagnostics.last,null);
    f.tools[0].insertion=.25;finish(f);
    assert.equal(f.tools[0].body.jointStateView.coordinates.at(-1),.25);assert.equal(f.system.diagnostics.initializations,1);
});

test('missing anatomy rejects without preparing tools or publishing geometry',()=>{
    const f=fixture();f.world.contactField=null;
    const result=f.system.step(f.world,dt);
    assert.equal(result.accepted,false);assert.equal(result.status,'geometry-not-ready');assert.equal(f.controls.reads,0);
    assert.ok(f.tools.every(t=>!t.body.jointStateView));assert.equal(f.system.diagnostics.initializations,0);
});

test('unchanged settled commands sleep with stable diagnostics and parameter changes wake the solver',()=>{
    const f=fixture({workSliceMs:1000});let result;
    for(let step=0;step<500;step++) {
        result=finish(f);if(result.status==='sleeping')break;
    }
    assert.equal(result.status,'sleeping');
    const saved=structuredClone(result.diagnostics),view=f.tools[0].body.jointStateView;
    const queries=f.controls.queries,accepted=f.system.diagnostics.acceptedSteps;
    assert.equal(f.system.step(f.world,dt).status,'sleeping');
    assert.equal(f.tools[0].body.jointStateView,view);assert.equal(f.controls.queries,queries);
    for(const [index,key,value] of [[1,'shaftStiffness',5],[1,'tipStiffness',8],[1,'type','berenstein'],[0,'mass',2]]) {
        f.tools[index][key]=value;
        assert.equal(finish(f).status,'converged',`Changed ${key} must be solved`);
    }
    assert.equal(f.system.diagnostics.acceptedSteps,accepted+4);
    assert.deepEqual(result.diagnostics,saved,'An earlier result must not mutate after later steps');
});

test('a throwing geometry callback preserves both published bodies and reset permits recovery',()=>{
    const f=fixture();finish(f);
    const saved=f.tools.map(t=>({view:t.body.jointStateView,points:positions(t.body)}));
    f.controls.throwQuery=true;f.tools[0].insertion=11;
    const failed=f.system.step(f.world,dt);
    // Numerical geometry errors are reported by the native transaction;
    // any pending slices before the final rejection are likewise private.
    let result=failed;
    for(let i=0;result.status==='shared-axis-pending'&&i<1000;i++)result=f.system.step(f.world,dt);
    assert.equal(result.accepted,false);assert.notEqual(result.status,'shared-axis-pending');
    f.tools.forEach((t,i)=>{assert.equal(t.body.jointStateView,saved[i].view);assert.deepEqual(positions(t.body),saved[i].points);});
    f.system.reset();f.controls.throwQuery=false;f.tools[0].insertion=1;
    assert.equal(finish(f).accepted,true);
});

test('terminal failure is latched, rolls back input, and a changed command resumes without resetting the accepted solver',()=>{
    const f=fixture();f.tools[0].insertion=11;finish(f);
    const saved=f.tools.map(t=>({view:t.body.jointStateView,points:positions(t.body),insertion:t.insertion}));
    const checkpoint=createPreparedInputCheckpoint();let command=1,preparations=0,rollbacks=0;
    const transaction=createFixedStepTransaction({world:f.world,
        prepare:()=>{checkpoint.capture([...f.tools,...f.tools.map(t=>t.body)]);preparations++;
            f.tools[0].insertion+=command;f.tools[0].body.x[0]+=command;return {command};},
        recovery:{readKey:()=>command,rollback:()=>{checkpoint.restore();rollbacks++;}}});
    f.controls.throwQuery=true;
    let result;
    for(let i=0;i<1000;i++){
        transaction.beginFrame();result=transaction.attempt(dt);
        if(result.terminal)break;
        assert.equal(result.pending,true);
        assert.throws(()=>f.world.abandonFailedWholeStep(),/terminally rejected/);
    }
    assert.equal(result.terminal,true);assert.equal(transaction.pending,false);assert.equal(transaction.blocked,true);
    assert.equal(rollbacks,1);assert.equal(preparations,1);assert.equal(f.world.stepCount,0);close(f.world.accumulator,0);
    const queries=f.controls.queries,failures=f.system.diagnostics.failedSteps;
    for(let i=0;i<120;i++){transaction.beginFrame();assert.equal(transaction.canAttempt(),false);assert.equal(transaction.attempt(dt).attempted,false);}
    assert.equal(f.controls.queries,queries);assert.equal(f.system.diagnostics.failedSteps,failures);
    f.tools.forEach((t,i)=>{assert.equal(t.insertion,saved[i].insertion);assert.deepEqual(positions(t.body),saved[i].points);assert.equal(t.body.jointStateView,saved[i].view);});
    // Even direct callers cannot restart the exact rejected target.
    f.tools[0].insertion=12;
    assert.equal(f.system.step(f.world,dt).terminal,true);assert.equal(f.controls.queries,queries);
    f.tools[0].insertion=11;
    f.controls.throwQuery=false;command=-1;
    for(let i=0;i<1000;i++){transaction.beginFrame();result=transaction.attempt(dt);if(result.accepted)break;assert.equal(result.pending,true);}
    assert.equal(result.accepted,true);assert.equal(transaction.blocked,false);assert.equal(f.world.stepCount,1);close(f.world.accumulator,0);
    assert.equal(f.system.diagnostics.initializations,1,'Recovery must retain the accepted native state');
    assert.equal(f.tools[0].body.jointStateView.coordinates.at(-1),10);assert.equal(preparations,2);
});

test('catheter material stiffness changes the solved transient shape, not only the command key',()=>{
    const shapes=[];
    for(const stiffness of [.2,20]) {
        const f=fixture({workSliceMs:1000});
        f.tools[1].type='berenstein';f.tools[1].tipStiffness=stiffness;
        // Two centimeters expose the bend beyond the 10 mm synthetic sheath.
        for(let step=1;step<=20;step++) {f.tools[1].insertion=step;finish(f);}
        shapes.push(Array.from(f.tools[1].body.jointStateView.positions));
    }
    const difference=Math.max(...shapes[0].map((v,i)=>Math.abs(v-shapes[1][i])));
    assert.ok(difference>1e-3,`The actual catheter shape must depend on stiffness (${difference} mm)`);
});

test('whole-step diagnostics sum wall-load fallbacks and their cost across rejected and accepted subdivisions',()=>{
    // Exercise the actual orchestration function with deterministic timestep
    // outcomes. A fallback in an earlier substep must not disappear when the
    // final accepted substep needs none.
    const attempts=[
        {converged:false,wallNormalFallback:{attempted:true,converged:false}},
        {converged:true,wallNormalFallback:{attempted:true,converged:true}},
        {converged:true}
    ];
    let calls=0;
    const advance=vm.runInNewContext(`(${advanceSharedAxis.toString()})`,{
        profile:t=>t,angleDifference:(a,b)=>a-b,
        feedSharedAxisNative:(s,feeds)=>({...s,materials:s.materials.map(m=>({...m,spec:{...m.spec,insertion:feeds[m.spec.id]}}))}),
        rotateSharedAxisNative:()=>{},
        iterateSharedAxisTimeStep:function*() {
            const index=calls++,scale=index+1;yield {kind:'controlled-step'};
            return {...attempts[index],iterations:scale,factorizations:2*scale,backtracks:scale,
                frictionIterations:scale,geometryRestarts:0,timings:{assemblyMs:scale,linearMs:2*scale,frictionMs:3*scale}};
        }
    });
    const state={materials:[{spec:{id:'wire',insertion:0}},{spec:{id:'catheter',insertion:0}}]};
    const iterator=advance(state,{wire:0,catheter:0},1/60,[{id:'wire',insertion:1,rotation:.1},{id:'catheter',insertion:.5,rotation:0}]);
    let next;do{next=iterator.next();}while(!next.done);
    assert.ok(next.value.state);assert.equal(calls,3);
    const r=next.value.result;
    assert.equal(r.subdivisions,2);assert.equal(r.substepAttempts,3);assert.equal(r.wallNormalFallbacks,2);
    assert.equal(r.wallNormalFallback,undefined,'Last substep itself did not fallback');
    assert.equal(r.iterations,6);assert.equal(r.factorizations,12);assert.equal(r.timings.assemblyMs,6);
    assert.equal(r.timings.linearMs,12);assert.equal(r.timings.frictionMs,18);
});


test('rejection records the accepted state and frozen command; replay survives JSON and later recovery',()=>{
    const events=[];const f=fixture({onRejectedStep:r=>events.push(r)});f.tools[0].insertion=11;finish(f);
    assert.equal(f.system.getLastFailure(),null,'No snapshot on successful steps');
    f.tools[0].insertion=12;f.controls.throwQuery=true;
    let failed;
    for(let i=0;i<1000;i++) {failed=f.system.step(f.world,dt);if(failed.terminal)break;}
    assert.equal(failed.terminal,true);
    assert.equal(events.length,1);
    for(let i=0;i<10;i++)assert.equal(f.system.step(f.world,dt).terminal,true);
    assert.equal(events.length,1,'Cached rejection does not produce a record per frame');
    assert.equal(events[0].failure.recovered,false);
    const report=JSON.parse(JSON.stringify(f.system.getLastFailure()));
    assert.equal(report.failure.captureError,undefined);
    assert.equal(report.tools[0].insertion,11);
    assert.equal(report.stepRequest.tools[0].insertion,12);
    assert.equal(report.stepRequest.dt,dt);
    assert.ok(report.acceptedWallGaps,'Contact retention state must be included');
    const restored=restoreSharedAxisReplay(report,f.world.contactField);
    const req=report.stepRequest;
    const iterator=advanceSharedAxis(restored,req.rotations,req.dt,req.tools,req.options);
    let next;do{next=iterator.next();}while(!next.done);
    assert.equal(next.value.state,undefined);
    assert.equal(next.value.result.status,report.failure.result.status);
    assert.deepEqual(JSON.parse(JSON.stringify(next.value.result.attempts)),report.failure.result.attempts);
    assert.deepEqual(captureSharedAxisReplay(restored,report.sheath),captureSharedAxisReplay(restoreSharedAxisReplay(report,f.world.contactField),report.sheath),
        'Failed replay must preserve its incoming physical state');
    // Caller mutations, recovery and reset must not erase or mutate the evidence.
    f.system.getLastFailure().positions[0][0]=999;
    f.controls.throwQuery=false;f.tools[0].insertion=10;finish(f);f.system.reset();
    assert.deepEqual(JSON.parse(JSON.stringify(f.system.getLastFailure())),report);
});


test('PD provider publishes only complete steps and keeps its solver identity and actual DOF count',()=>{
    const f=fixture({adaptiveMesh:true,projectiveDynamics:true});finish(f);
    assert.equal(f.system.id,'shared-axis-projective');assert.equal(f.system.diagnostics.modifiedNewton,false);
    assert.ok(f.system.diagnostics.last.pd);assert.equal(f.system.diagnostics.mesh.dofs,f.system.diagnostics.last.pd.dofs);
    const views=f.tools.map(t=>t.body.jointStateView);
    f.tools[0].insertion=3;f.tools[1].insertion=1;
    finish(f,{onPending:()=>f.tools.forEach((t,i)=>assert.equal(t.body.jointStateView,views[i]))});
    assert.deepEqual(f.tools.map(t=>t.body.jointStateView.coordinates.at(-1)),[3,1]);
    f.system.reset();assert.equal(f.system.diagnostics.last,null);assert.ok(f.tools.every(t=>!t.body.jointStateView));
});


test('fast Newton provider is explicit, forwards its counters and is disabled for projective dynamics',()=>{
    const ordinary=fixture({adaptiveMesh:true});assert.equal(ordinary.system.diagnostics.coupledFrictionNewton,false);
    const fast=fixture({adaptiveMesh:true,coupledFrictionNewton:true});finish(fast);
    assert.equal(fast.system.diagnostics.coupledFrictionNewton,true);
    assert.ok(Number.isFinite(fast.system.diagnostics.last.coupledFrictionRefreshes));
    assert.ok(Number.isFinite(fast.system.diagnostics.last.retainedDiscoveryTrials));
    assert.equal(fixture({projectiveDynamics:true,coupledFrictionNewton:true}).system.diagnostics.coupledFrictionNewton,false);
});

test('predictive Newton can be disabled while keeping fast friction enabled',()=>{
    const fast=fixture({adaptiveMesh:true,coupledFrictionNewton:true});finish(fast);
    assert.equal(fast.system.diagnostics.predictiveNewton,true);
    const previous=fixture({adaptiveMesh:true,coupledFrictionNewton:true,predictiveNewton:false});finish(previous);
    assert.equal(previous.system.diagnostics.coupledFrictionNewton,true);
    assert.equal(previous.system.diagnostics.predictiveNewton,false);
    assert.equal(fixture().system.diagnostics.predictiveNewton,false);
    assert.equal(fixture({projectiveDynamics:true,coupledFrictionNewton:true}).system.diagnostics.predictiveNewton,false);
});


test('recovered substep rejections emit the original replay once, before publishing the accepted state',()=>{
    const events=[];const f=fixture({onRejectedStep:r=>events.push(r),physicsOptions:{liveWallNormalLoad:false}});
    f.tools[0].insertion=11;finish(f);
    f.tools[0].insertion=12;f.controls.failQueries=1;finish(f);
    assert.equal(events.length,1);
    assert.equal(events[0].failure.recovered,true);
    assert.ok(events[0].failure.result.attempts.some(a=>a.converged===false));
    assert.equal(events[0].tools[0].insertion,11);
    assert.equal(events[0].stepRequest.tools[0].insertion,12);
    assert.equal(f.tools[0].body.jointStateView.coordinates.at(-1),12);
    f.tools[0].insertion=13;f.controls.failQueries=1;finish(f);
    assert.equal(events.length,2);
    assert.notEqual(events[0].failure.id,events[1].failure.id);
});

test('a failed archive observer cannot prevent recovery or physical publication',()=>{
    let called=0;const f=fixture({onRejectedStep:()=>{called++;throw new Error('archive offline');},physicsOptions:{liveWallNormalLoad:false}});
    f.tools[0].insertion=11;finish(f);
    f.tools[0].insertion=12;f.controls.failQueries=1;
    assert.equal(finish(f).accepted,true);
    assert.equal(called,1);
    assert.equal(f.tools[0].body.jointStateView.coordinates.at(-1),12);
});

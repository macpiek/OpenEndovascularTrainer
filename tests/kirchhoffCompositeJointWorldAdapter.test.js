import assert from 'node:assert/strict';
import test from 'node:test';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeJointTimeStepState,advanceCompositeJointTimeStep} from '../src/physics/kirchhoffCompositeJointTimeStep.js';
import {createCompositeJointWorldAdapter} from '../src/physics/kirchhoffCompositeJointWorldAdapter.js';
import {createContactResult} from '../src/physics/collision/vesselContactField.js';
import {getCompositeJointWallFrictionEdgeId} from '../src/physics/kirchhoffCompositeJointWallFrictionRows.js';
import {createCompositeJointSurfacePoseHistory,prepareCompositeJointSurfacePosePath} from '../src/physics/kirchhoffCompositeJointSurfacePoseHistory.js';

const dt=1/120,close=(a,b,t=1e-10)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}`);
const accepted=r=>assert.equal(r.accepted,true,JSON.stringify(r));
const copyViews=world=>world.bodies.map(b=>Object.fromEntries(Object.entries(b).filter(([,v])=>ArrayBuffer.isView(v)).map(([k,v])=>[k,v.slice()])));
function fixture({publishFailure=false,publicationOrigin}={}) {
    const coordinates=[0,2,4],positions=coordinates.map(x=>[x,0,0]),wire=coordinates.map(x=>[x+.1,.341-.002*x,0]);
    const layout=createCompositeChainLayout([['wire','catheter'],['wire','catheter']]),angles=new Map(),restLengths=new Map();
    const tools=['wire','catheter'].map(id=>{
        const p=id==='wire'?wire:positions,dsDx=id==='wire'?Math.hypot(1,.002):1;
        angles.set(id,new Float64Array(2));restLengths.set(id,new Float64Array(2).fill(2*dsDx));
        return {id,dsDx,reference:captureCompositeReferenceFrames(p),referenceTwists:new Float64Array(1),material:compileCompositeMaterial({EI1:id==='wire'?2:8,EI2:id==='wire'?3:11,GJ:id==='wire'?1:4})};
    });
    const initial=createCompositeJointTimeStepState({layout,coordinates,positions,angles,restLengths,tools,materialCoordinate:'reference-arclength',
        modes:coordinates.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]})),relative:wire.flatMap((p,i)=>p.map((v,k)=>v-positions[i][k]))});
    const contacts={mode:'lumen-coulomb',chartId:'actual-world-adapter',forcePerLength:1,
        friction:{law:'coulomb',mu:[.015,.006],forcePerLength:50,materialPath:'linear-affine-maps'},
        pairs:[{id:'side-0',innerToolId:'wire',outerToolId:'catheter',innerEdge:0,outerEdge:0,innerMaterialSegmentId:'wire:0',outerMaterialSegmentId:'catheter:0',
            lumenRadius:.5,innerRadius:.16,quadrature:[.25],openDistal:false,portalFilletRadius:0}]};
    const control={prepares:0,initializes:0,lastInput:null,failPublication:publishFailure};
    let bindings;
    function options(state){
        return {dt,torsionMode:'quasi-static',contacts,wall:control.wall??'none',globalization:control.globalization??'adaptive',
            inertia:{previousPositions:structuredClone(state.toolPositions),inertiaEdges:state.layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>{
                const dsDx=state.tools.find(t=>t.id===id).dsDx;
                return {id,massPerMaterialLength:id==='wire'?.13:.24,materialMap:{sStart:20+state.coordinates[e]*dsDx,dsDx,dsDt:0},
                    oldMaterialVelocities:structuredClone(state.materialVelocities?.[e]?.tools.find(t=>t.id===id).velocities??[[0,0,0],[0,0,0]])};
            })}))},
            boundaries:{positions:['wire','catheter'].map(toolId=>({toolId,node:2,value:initial.toolPositions.get(toolId)[2].slice()})),
                spins:[{toolId:'wire',edge:0,value:.03*(state.time+dt)},{toolId:'catheter',edge:0,value:-.01*(state.time+dt)}]},
            loads:{forces:[{toolId:'wire',node:0,value:[0,.4,0]}],torques:[{toolId:'wire',edge:1,value:.001}]}};
    }
    const adapter=createCompositeJointWorldAdapter({initialize(){control.initializes++;return {state:initial,bindings,publicationOrigin};},
        prepareStep({state}){control.prepares++;return control.lastInput=options(state);}});
    const world=new EndovascularPhysicsWorld({fixedDt:dt,wholeStepSystem:adapter});
    bindings=['wire','catheter'].map((toolId,i)=>{
        let body=world.createRod(i===0?'guidewire':'catheter',3,2);
        initial.toolPositions.get(toolId).forEach((p,n)=>body.setNodePosition(n,...p));body.copyCurrentToPrevious();
        if(i===1&&publishFailure){
            body=new Proxy(body,{defineProperty(target,key,descriptor){
                if(key==='jointStateView'&&control.failPublication){control.failPublication=false;throw new Error('test second-body publication failure');}
                return Reflect.defineProperty(target,key,descriptor);
            }});world.bodies[i]=body;
        }
        return {body,toolId,nodes:[0,1,2].map(node=>({node,jointNode:node,trace:node===0?'right':'left'})),edges:[0,1].map(edge=>({edge,jointEdge:edge}))};
    });
    return {world,adapter,initial,bindings,control,options};
}
function verifyPublished(f) {
    const state=f.adapter.snapshot();
    for(const {body,toolId,nodes,edges} of f.bindings){
        const view=body.jointStateView;
        assert.equal(view.time,state.time);assert.equal(view.step,state.step);assert.equal(view.coarseBodyArraysArePhysicsState,false);
        assert.equal(view.materialAngularVelocity,null);assert.equal(view.angularInertia,'quasi-static');
        for(const r of nodes)for(let k=0;k<3;k++)assert.equal(body[['x','y','z'][k]][r.node],Math.fround(state.toolPositions.get(toolId)[r.jointNode][k]));
        assert.deepEqual(view.positions,Float64Array.from(state.toolPositions.get(toolId).flat()));
        assert.deepEqual(view.unwrappedAngles,state.angles.get(toolId));
        for(const {edge,jointEdge} of edges){
            const q=['orientationX','orientationY','orientationZ','orientationW'].map(k=>body[k][edge]);close(Math.hypot(...q),1,3e-14);
            const [x,y,z,w]=q,tangent=[2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)],expected=state.tools.find(t=>t.id===toolId).reference[jointEdge].tangent;
            tangent.forEach((v,k)=>close(v,expected[k],3e-14));
        }
    }
    const result=f.world.lastStepResult;assert.equal(result.diagnostics.certificate.converged,true);
    assert.equal(result.diagnostics.certificate.friction.converged,true);
    for(const balance of result.balances.values())balance.momentumRate.forEach((v,k)=>close(v,balance.appliedForce[k]+balance.boundaryForce[k]+balance.contactForce[k],1e-7));
    return state;
}

test('actual World runs the production shared Coulomb timestep for two dt and publishes both own material states',()=>{
    const f=fixture(),before=structuredClone(f.initial),direct1=advanceCompositeJointTimeStep(f.initial,f.options(f.initial));accepted(direct1);
    Object.defineProperty(f.world.bodies[0],'linearDamping',{get(){assert.fail('Old damping must not run');}});
    Object.defineProperty(f.world.bodies[0],'_splitPhysicalMotion',{set(){assert.fail('Old predictor must not run');}});
    let preparations=0;assert.equal(f.world.advance(dt,()=>preparations++),1);const first=verifyPublished(f);
    assert.deepEqual(first,direct1.state);assert.equal(f.world.stepCount,1);assert.equal(f.world.accumulator,0);
    const direct2=advanceCompositeJointTimeStep(first,f.options(first));accepted(direct2);
    assert.equal(f.world.advance(dt,()=>preparations++),1);assert.deepEqual(verifyPublished(f),direct2.state);
    assert.equal(f.control.initializes,1);assert.equal(f.control.prepares,2);assert.equal(preparations,2);assert.deepEqual(f.initial,before);
    assert.equal(f.adapter.diagnostics.publications,2);assert.equal(f.world.getStats().mode,'whole-step');
    assert.equal(f.world.getStats().wholeStepSystem,f.adapter.id);
});

test('ordinary accepted steps are identical with adaptive and original Newton search and never activate regularization',()=>{
    const f=fixture();let state=f.initial;
    for(let step=0;step<3;step++){
        const adaptive=advanceCompositeJointTimeStep(state,f.options(state)),newton=advanceCompositeJointTimeStep(state,{...f.options(state),globalization:'newton'});accepted(adaptive);accepted(newton);
        assert.deepEqual(adaptive.state,newton.state);assert.deepEqual(adaptive.balances,newton.balances);assert.deepEqual(adaptive.diagnostics.certificate,newton.diagnostics.certificate);
        assert.deepEqual(adaptive.diagnostics.regularizationActivations,[]);assert.equal(adaptive.diagnostics.numericalDirections,0);
        for(const k of ['directions','evaluations','linearSolves','contactQueries'])assert.equal(adaptive.diagnostics[k],newton.diagnostics[k]);
        state=adaptive.state;
    }
});

test('failed numerical budgets retain one prepared command and no geometry/history/time, then direct retry is identical',()=>{
    const f=fixture(),before=copyViews(f.world);f.adapter.setBudget({directions:0});let prepared=0;
    assert.equal(f.world.advance(dt,()=>prepared++),0);assert.equal(f.world.lastStepResult.status,'direction-budget');
    assert.deepEqual(copyViews(f.world),before);assert.equal(f.adapter.snapshot().step,0);assert.equal(f.world.stepCount,0);assert.equal(f.world.accumulator,dt);
    const expected=advanceCompositeJointTimeStep(f.initial,f.options(f.initial));accepted(expected);
    // A caller cannot modify an already prepared force, BC or velocity by
    // changing the source arrays while this same physical dt is pending.
    f.control.lastInput.loads.forces[0].value[1]=400;f.control.lastInput.inertia.inertiaEdges[0].tools[0].oldMaterialVelocities[0][0]=100;
    f.adapter.setBudget(null);const retry=f.world.stepFixed();accepted(retry);
    assert.equal(retry.consumedPendingDt,true);assert.deepEqual(verifyPublished(f),expected.state);
    assert.equal(prepared,1);assert.equal(f.control.prepares,1);assert.equal(f.world.accumulator,0);assert.equal(f.world.stepCount,1);
});

test('a second-body publication exception restores all first-body arrays and views before retry',()=>{
    const f=fixture({publishFailure:true}),before=copyViews(f.world);
    assert.equal(f.world.advance(dt),0);assert.equal(f.world.lastStepResult.accepted,false);assert.match(f.world.lastStepResult.message,/second-body publication/);
    assert.deepEqual(copyViews(f.world),before);assert.ok(f.world.bodies.every(b=>b.jointStateView===undefined));
    assert.equal(f.adapter.snapshot().step,0);assert.equal(f.adapter.diagnostics.publications,0);assert.equal(f.world.accumulator,dt);
    const expected=advanceCompositeJointTimeStep(f.initial,f.options(f.initial));accepted(expected);
    assert.equal(f.world.advance(0),1);assert.deepEqual(verifyPublished(f),expected.state);assert.equal(f.control.prepares,1);
});

test('accepted Float64 physics and complete views are independent of subsequent Float32 view corruption',()=>{
    const f=fixture();accepted(f.world.stepFixed());const first=f.adapter.snapshot(),expected=advanceCompositeJointTimeStep(first,f.options(first));accepted(expected);
    for(const b of f.world.bodies){b.x.fill(1e6);b.velocityY.fill(1e6);b.orientationW.fill(0);b.jointStateView.positions.fill(1e6);b.jointStateView.unwrappedAngles.fill(1e6);}
    accepted(f.world.stepFixed());assert.deepEqual(verifyPublished(f),expected.state);
    const snapshot=f.adapter.snapshot();snapshot.positions[0][0]=999;assert.notEqual(f.adapter.snapshot().positions[0][0],999);
});

test('unmapped World constraints reject explicitly before advancing the shared model',()=>{
    for(const [collection,status] of [['sheaths','joint-world-sheath-adapter-required'],['containments','joint-world-containment-adapter-required'],['toolContacts','joint-world-external-contact-adapter-required']]){
        const f=fixture(),before=copyViews(f.world);f.world[collection].push({enabled:true});
        assert.equal(f.world.advance(dt),0);assert.equal(f.world.lastStepResult.status,status);
        assert.equal(f.adapter.snapshot().step,0);assert.deepEqual(copyViews(f.world),before);assert.equal(f.adapter.diagnostics.publications,0);
    }
});

test('the same World field and a vacuous wall certificate cannot stand in for actual surface coverage',()=>{
    const f=fixture(),before=copyViews(f.world);let queries=0;
    const field={queryCapsuleCoordinates(){queries++;throw new Error('No undeclared wall coverage may reach a query');}};
    f.world.contactField=field;
    f.control.wall={mode:'wall-normal',friction:'none',field,chartId:'empty-world-wall',forcePerLength:1,
        contactOwners:{edges:[{edge:0,wall:null},{edge:1,wall:null}]}};
    assert.equal(f.world.advance(dt),0);assert.equal(f.world.lastStepResult.status,'joint-world-wall-adapter-required');
    assert.equal(queries,0);assert.equal(f.world.stepCount,0);assert.equal(f.world.accumulator,dt);
    assert.deepEqual(copyViews(f.world),before);assert.equal(f.adapter.diagnostics.publications,0);
});

test('active-range/storage changes require transfer and reset releases owned state, views and pending preparation',()=>{
    const f=fixture();accepted(f.world.stepFixed());const state=f.adapter.snapshot(),views=copyViews(f.world);
    f.world.bodies[0].activeEnd=1;assert.equal(f.world.advance(dt),0);assert.equal(f.world.lastStepResult.status,'joint-world-remap-required');
    assert.deepEqual(f.adapter.snapshot(),state);assert.deepEqual(copyViews(f.world),views);
    f.world.resetSimulationState();assert.equal(f.adapter.snapshot(),null);assert.equal(f.adapter.diagnostics.pending,false);assert.ok(f.world.bodies.every(b=>b.jointStateView===undefined));
    assert.equal(f.world.accumulator,0);assert.equal(f.world.stepCount,0);
});

function addPreparedWall(f,{height=.501,state=f.initial,originalField}={}){
    const tool=state.tools.find(t=>t.id==='wire'),positions=state.toolPositions.get('wire'),materialSegmentId='world-path-wire:0',edgeId=getCompositeJointWallFrictionEdgeId(materialSegmentId);
    const labels=[20,20+state.coordinates[1]*tool.dsDx],history=createCompositeJointSurfacePoseHistory({toolId:'wire',
        nodes:positions.slice(0,2).map((position,node)=>({id:node,node,position})),
        edges:[{edgeId,edge:0,materialSegmentId,source:'accepted',nodeIds:[0,1],coordinates:state.coordinates.slice(0,2),labels,reference:tool.reference[0],angle:state.angles.get('wire')[0]}]});
    const path=prepareCompositeJointSurfacePosePath({history,targetEdgeId:edgeId,dt,currentMaps:[{edgeId,labels,dsDt:0}],
        configurationColumns:[...[0,1].flatMap(node=>[0,1,2].map(component=>({kind:'position',toolId:'wire',node,component}))),{kind:'angle',toolId:'wire',edge:0}],
        nodeBindings:[0,1].map(node=>({nodeId:node,offset:[0,0,0],terms:[0,1,2].map(k=>({column:3*node+k,weights:[0,1,2].map(axis=>axis===k?1:0)}))})),
        angleBindings:[{edgeId,offset:0,terms:[{column:6,weight:1}]}]});
    const field=originalField??{calls:0,queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out=createContactResult()){
        this.calls++;const t=ay>=by?0:1,p=t===0?[ax,ay,az]:[bx,by,bz];out.signedDistance=height-p[1];out.signedGap=out.signedDistance-radius;
        out.segmentT=t;out.capsuleSampleCount=2;out.inward.values.set([0,-1,0]);out.closestPoint.values.set([p[0],height,p[2]]);out.source='analytic-plane';out.faceIndex=0;return out;
    }};
    f.control.wall={mode:'wall-coulomb',chartId:'actual-world-path',contactMode:'nodal-endpoints',pressureDiscretization:'nodal-endpoints-one-sided-surface',
        pressureSites:[{owner:'wire',node:0,edge:0,trace:'right'},{owner:'wire',node:1,edge:0,trace:'left'}],field,forcePerLength:1,
        plane:{normal:[0,-1,0],offset:-height},contactOwners:{edges:[{edge:0,wall:{owner:'wire',radius:.16,materialSegmentId}},{edge:1,wall:null}]},
        friction:{law:'coulomb',mu:[.006,.006],forcePerLength:50,materialPath:'linear-affine-maps',motion:'stationary-material',source:'analytic-plane',tangentBasis:'projected-own-tangent'},
        surfacePosePaths:[{owner:'wire',edge:0,path}]};
    return {path,field};
}

test('World preparation retains genuine immutable wall pose paths and owns their mutable declarations across retry',()=>{
    const f=fixture(),{path,field}=addPreparedWall(f),direct=advanceCompositeJointTimeStep(f.initial,f.options(f.initial));accepted(direct);
    assert.ok(direct.state.wallContactState.normalForces.some(v=>v>0));
    f.adapter.setBudget({directions:0});assert.equal(f.world.advance(dt),0);assert.equal(f.world.lastStepResult.status,'direction-budget');
    assert.ok(Object.isFrozen(path));
    // These edits affect only caller-owned records. The genuine prepared
    // handle, original field, and copied semantic declaration remain pending.
    f.control.lastInput.wall.surfacePosePaths[0].owner='catheter';f.control.lastInput.wall.surfacePosePaths.length=0;
    f.adapter.setBudget(null);assert.equal(f.world.advance(0),1);assert.deepEqual(verifyPublished(f),direct.state);
    assert.equal(f.world.lastStepResult.diagnostics.certificate.wallFriction.converged,true);assert.ok(field.calls>0);assert.equal(f.control.prepares,1);
});

test('a structurally copied wall pose path has no preparation provenance and cannot enter World physics',()=>{
    const f=fixture();addPreparedWall(f);f.control.wall.surfacePosePaths[0].path=structuredClone(f.control.wall.surfacePosePaths[0].path);
    const before=copyViews(f.world);assert.equal(f.world.advance(dt),0);assert.equal(f.world.lastStepResult.status,'joint-world-preparation-error');
    assert.match(f.world.lastStepResult.message,/prepared own surface pose path/);assert.deepEqual(copyViews(f.world),before);assert.equal(f.world.stepCount,0);
});

test('unregularized Newton retains the original wall penetration failure without committing a partial World step',()=>{
    const f=fixture();addPreparedWall(f,{height:.49});f.control.globalization='newton';const before=copyViews(f.world);
    assert.equal(f.world.advance(dt),0);assert.equal(f.world.lastStepResult.status,'original-linear-equations');
    assert.equal(f.world.stepCount,0);assert.equal(f.world.accumulator,dt);assert.equal(f.adapter.snapshot().step,0);
    assert.deepEqual(copyViews(f.world),before);assert.equal(f.adapter.diagnostics.publications,0);
});

test('adaptive search resolves initial wall penetration with the unchanged full physical certificate and next-dt history',context=>{
    const f=fixture();addPreparedWall(f,{height:.49});const direct=advanceCompositeJointTimeStep(f.initial,f.options(f.initial));accepted(direct);
    assert.equal(f.world.advance(dt),1);const first=verifyPublished(f);assert.deepEqual(first,direct.state);
    const diagnostics=f.world.lastStepResult.diagnostics;
    assert.ok(diagnostics.tinyNewtonStepsAvoided>0);assert.ok(diagnostics.numericalDirections>0);
    assert.equal(diagnostics.regularizationActivations[0].reason,'tiny-newton-step');
    assert.ok(diagnostics.certificate.wall.converged);assert.ok(diagnostics.certificate.wallFriction.converged);
    assert.ok(diagnostics.certificate.force<=1e-7);assert.ok(diagnostics.certificate.torque<=1e-8);assert.ok(diagnostics.certificate.length<=1e-8);
    assert.ok(diagnostics.certificate.wall.originalInequalities.every(s=>s.gap>=-1e-8));
    addPreparedWall(f,{height:.49,state:first,originalField:f.control.wall.field});const second=advanceCompositeJointTimeStep(first,f.options(first));accepted(second);
    assert.equal(f.world.advance(dt),1);assert.deepEqual(verifyPublished(f),second.state);assert.equal(f.world.stepCount,2);assert.equal(f.world.accumulator,0);
    context.diagnostic(JSON.stringify({directions:diagnostics.directions,evaluations:diagnostics.evaluations,activations:diagnostics.regularizationActivations,force:diagnostics.certificate.force}));
});

test('budgets during stabilized wall search restore both bodies and prepared history; exact retry consumes dt once',()=>{
    const cold=fixture();addPreparedWall(cold,{height:.49});const expected=advanceCompositeJointTimeStep(cold.initial,cold.options(cold.initial));accepted(expected);
    const d=expected.diagnostics;
    for(const budget of [{directions:d.directions-1},{linearSolves:d.linearSolves-1},{evaluations:d.evaluations-1},{contactQueries:d.contactQueries-1}]){
        const f=fixture();addPreparedWall(f,{height:.49,originalField:cold.control.wall.field});const before=copyViews(f.world);f.adapter.setBudget(budget);
        assert.equal(f.world.advance(dt),0);assert.match(f.world.lastStepResult.status,/budget/);
        assert.deepEqual(copyViews(f.world),before);assert.equal(f.adapter.snapshot().step,0);assert.equal(f.world.accumulator,dt);
        assert.ok(f.world.lastStepResult.diagnostics.regularizationActivations.length>0);assert.ok(f.world.bodies.every(b=>b.jointStateView===undefined));
        f.adapter.setBudget(null);assert.equal(f.world.advance(0),1);assert.deepEqual(verifyPublished(f),expected.state);
        assert.equal(f.control.prepares,1);assert.equal(f.world.stepCount,1);assert.equal(f.world.accumulator,0);
    }
});


test('local physics publishes world positions without translating velocities or its owned state',()=>{
    const origin=[1032,-425,87],f=fixture({publicationOrigin:origin}),baseline=fixture();
    accepted(f.adapter.step(f.world,dt));accepted(baseline.adapter.step(baseline.world,dt));
    assert.deepEqual(f.adapter.snapshot(),baseline.adapter.snapshot());
    for(let b=0;b<f.bindings.length;b++){
        const body=f.bindings[b].body,plain=baseline.bindings[b].body;
        for(let k=0;k<3;k++)for(let n=0;n<3;n++){
            const value=f.adapter.snapshot().toolPositions.get(f.bindings[b].toolId)[n][k];
            assert.equal(body[['x','y','z'][k]][n],Math.fround(value+origin[k]));
            assert.equal(body[['previousX','previousY','previousZ'][k]][n],Math.fround(f.initial.toolPositions.get(f.bindings[b].toolId)[n][k]+origin[k]));
            assert.equal(body[['velocityX','velocityY','velocityZ'][k]][n],plain[['velocityX','velocityY','velocityZ'][k]][n]);
        }
        body.jointStateView.positions.forEach((v,i)=>assert.equal(v,plain.jointStateView.positions[i]+origin[i%3]));
        assert.deepEqual(body.jointStateView.edgeMaterialVelocities,plain.jointStateView.edgeMaterialVelocities);
        assert.deepEqual(body.jointStateView.orientations,plain.jointStateView.orientations);
    }
    accepted(f.adapter.step(f.world,dt));accepted(baseline.adapter.step(baseline.world,dt));
    assert.deepEqual(f.adapter.snapshot(),baseline.adapter.snapshot());
});

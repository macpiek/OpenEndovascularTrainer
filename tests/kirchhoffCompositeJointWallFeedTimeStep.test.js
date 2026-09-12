import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createContactResult} from '../src/physics/collision/vesselContactField.js';
import {measureCompositeFriction} from '../src/physics/kirchhoffCompositeFriction.js';
import {createCompositeJointMaterialHistory} from '../src/physics/kirchhoffCompositeJointMaterialHistory.js';
import {createCompositeJointSurfacePoseHistory,prepareCompositeJointSurfacePosePath} from '../src/physics/kirchhoffCompositeJointSurfacePoseHistory.js';
import {getCompositeJointWallFrictionEdgeId} from '../src/physics/kirchhoffCompositeJointWallFrictionRows.js';
import {createCompositeJointTimeStepState,createCompositeJointTimeStepWorkspace,advanceCompositeJointTimeStep} from '../src/physics/kirchhoffCompositeJointTimeStep.js';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {createCompositeJointWorldAdapter} from '../src/physics/kirchhoffCompositeJointWorldAdapter.js';
import {prepareCompositeJointWorldWall} from '../src/physics/kirchhoffCompositeJointWorldWall.js';
const close=(a,b,t=1e-8)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b} (${t})`);
const same=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,j)=>close(v,b[j],t));};
const accepted=r=>assert.equal(r.accepted,true,JSON.stringify({status:r.status,error:r.error,diagnostics:r.diagnostics}));
const id='catheter',semantic=edge=>`catheter:${edge}`;
function fixture() {
    const coordinates=[0,2,4],slope=.002,positions=coordinates.map(x=>[x,.16001+slope*x,0]),dsDx=Math.hypot(1,slope);
    const layout=createCompositeChainLayout([[id],[id]]),reference=captureCompositeReferenceFrames(positions);
    const state=createCompositeJointTimeStepState({layout,coordinates,positions,relative:[],modes:[],angles:new Map([[id,new Float64Array(2)]]),
        tools:[{id,dsDx,reference,referenceTwists:new Float64Array(1),material:compileCompositeMaterial({stiffness:[[2,0,0],[0,3,0],[0,0,1]],intrinsic:[0,0,0]})}],
        restLengths:new Map([[id,new Float64Array(2).fill(2*dsDx)]]),materialCoordinate:'reference-arclength'});
    const field={calls:0,queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out=createContactResult()){
        this.calls++;const t=ay<by?0:1,p=t===0?[ax,ay,az]:[bx,by,bz];out.signedDistance=p[1];out.signedGap=p[1]-radius;out.segmentT=t;out.capsuleSampleCount=2;
        out.inward.values.set([0,1,0]);out.closestPoint.values.set([p[0],0,p[2]]);out.source='analytic-plane';out.faceIndex=0;return out;
    }};
    const wall={mode:'wall-coulomb',chartId:'production-proximal-catheter-feed',contactMode:'nodal-endpoints',pressureDiscretization:'nodal-endpoints-one-sided-surface',
        pressureSites:[{owner:id,node:0,edge:0,trace:'right'},{owner:id,node:1,edge:0,trace:'left'}],field,forcePerLength:1,plane:{normal:[0,1,0],offset:0},
        contactOwners:{edges:[{edge:0,wall:{owner:id,radius:.16,materialSegmentId:semantic(0)}},{edge:1,wall:null}]},
        friction:{law:'coulomb',mu:[.006,.006],forcePerLength:50,materialPath:'linear-affine-maps',motion:'stationary-material',source:'analytic-plane',tangentBasis:'projected-own-tangent'}};
    // Explicit straight entrance port: current exterior endpoint = 2*q0-q1,
    // with its own angle bound to the entrance spin. Store its accepted pose;
    // a later dt consumes that pose, never a candidate extrapolation.
    const ports=new WeakMap();ports.set(state,{position:[-2,.15601,0],reference:structuredClone(reference[0]),angle:0});
    return {state,wall,ports};
}
function rememberAcceptedPort(f,state) {
    const p=state.toolPositions.get(id);f.ports.set(state,{position:p[0].map((v,k)=>2*v-p[1][k]),reference:structuredClone(state.tools[0].reference[0]),angle:state.angles.get(id)[0]});
}
function options(f,state,{dt=1/120,feed=-.3,spin=.03,force=-.4,k=50,workspace,budget,withPath=true}={}) {
    const segmentId=f.materialSegmentId??semantic,edgeId=edge=>getCompositeJointWallFrictionEdgeId(segmentId(edge));
    const dsDx=state.tools[0].dsDx,oldStart=state.wallFrictionState?.currentMaps.find(m=>m.id===id&&m.edge===0)?.currentMap.sStart??20;
    const edges=state.layout.edgeToolIds.map((ids,edge)=>({tools:ids.map(id=>({id,massPerMaterialLength:.13,
        materialMap:{sStart:oldStart+state.coordinates[edge]*dsDx+dt*feed,dsDx,dsDt:feed}}))}));
    const velocityHistory=createCompositeJointMaterialHistory({materialVelocities:state.materialVelocities,
        reservoir:({toolId})=>({id:toolId,sStart:0,sEnd:100,velocities:[[0,0,0],[0,0,0]],interpretation:'physical-material-velocity'})});
    const sampled=velocityHistory.prepare({coordinates:state.coordinates,inertiaEdges:edges});
    const inertiaEdges=edges.map((e,edge)=>({tools:e.tools.map((t,index)=>{const h=sampled.inertiaEdges[edge].tools[index];return {...t,...(h.pieces.length===1?{oldMaterialVelocities:h.oldMaterialVelocities}:
        {oldVelocityPieces:h.pieces.map(p=>({fractions:p.fractions,oldMaterialVelocities:p.oldMaterialVelocities}))})};})}));
    const wall={...f.wall,friction:{...f.wall.friction,forcePerLength:k}};
    if(withPath){
        const port=f.ports.get(state);assert.ok(port,'An accepted external port pose is required');
        const p=state.toolPositions.get(id),nodes=[{id:'external',position:port.position},...p.map((position,node)=>({id:node,node,position}))];
        const acceptedEdges=[0,1].map(edge=>({edgeId:edgeId(edge),edge,materialSegmentId:segmentId(edge),source:'accepted',nodeIds:[edge,edge+1],coordinates:state.coordinates.slice(edge,edge+2),
            labels:[oldStart+state.coordinates[edge]*dsDx,oldStart+state.coordinates[edge+1]*dsDx],reference:state.tools[0].reference[edge],angle:state.angles.get(id)[edge]}));
        const sourceEdges=[{edgeId:'catheter:external-port',materialSegmentId:'external:catheter:0',source:'reservoir',nodeIds:['external',0],coordinates:[-2,0],labels:[oldStart-2*dsDx,oldStart],reference:port.reference,angle:port.angle},...acceptedEdges];
        const history=createCompositeJointSurfacePoseHistory({toolId:id,nodes,edges:sourceEdges,reservoirIdentity:'explicit-straight-entrance-port',
            hinges:[{leftEdgeId:sourceEdges[0].edgeId,rightEdgeId:edgeId(0),referenceTwist:0},{leftEdgeId:edgeId(0),rightEdgeId:edgeId(1),referenceTwist:state.tools[0].referenceTwists[0]}]});
        const configurationColumns=[...p.flatMap((_,node)=>[0,1,2].map(component=>({kind:'position',toolId:id,node,component}))),...[0,1].map(edge=>({kind:'angle',toolId:id,edge}))];
        const nodeBindings=nodes.map(n=>n.node===undefined?{nodeId:n.id,offset:[0,0,0],terms:[0,1,2].flatMap(k=>[{column:k,weights:[0,1,2].map(j=>j===k?2:0)},{column:3+k,weights:[0,1,2].map(j=>j===k?-1:0)}])}:
            {nodeId:n.id,offset:[0,0,0],terms:[0,1,2].map(k=>({column:3*n.node+k,weights:[0,1,2].map(j=>j===k?1:0)}))});
        const path=prepareCompositeJointSurfacePosePath({history,targetEdgeId:edgeId(0),dt,configurationColumns,nodeBindings,
            angleBindings:sourceEdges.map(e=>({edgeId:e.edgeId,offset:0,terms:[{column:9+(e.edge??0),weight:1}]})),
            currentMaps:sourceEdges.map(e=>({edgeId:e.edgeId,labels:e.labels.map(s=>s+dt*feed),dsDt:feed}))});
        wall.surfacePosePaths=[{owner:id,edge:0,path}];
    }
    return {dt,torsionMode:'quasi-static',contacts:'none',wall,workspace,budget,inertia:{previousPositions:structuredClone(state.toolPositions),inertiaEdges},
        boundaries:{positions:[{toolId:id,node:2,value:f.state.toolPositions.get(id)[2]}],spins:[{toolId:id,edge:0,value:spin*(state.time+dt)}]},
        loads:{forces:[{toolId:id,node:0,value:[0,force,0]}],torques:[{toolId:id,edge:1,value:.001}]}};
}
function advance(f,state,extra) {
    const calls=f.wall.field.calls,r=advanceCompositeJointTimeStep(state,options(f,state,extra));
    assert.equal(f.wall.field.calls-calls,r.diagnostics.wallQueries);
    if(r.accepted)rememberAcceptedPort(f,r.state);return r;
}
function verify(f,r) {
    const p=r.state.toolPositions.get(id),normal=r.diagnostics.certificate.wall,friction=r.diagnostics.certificate.wallFriction;
    assert.equal(normal.converged,true);assert.equal(friction.converged,true);assert.equal(friction.samples.length,2);assert.equal(r.diagnostics.wallFrictionRows,4);
    assert.equal(r.diagnostics.certificate.converged,true);
    for(const [key,tolerance] of [['force',1e-7],['torque',1e-8],['length',1e-8],['boundary',1e-9]])assert.ok(r.diagnostics.certificate[key]<=tolerance);
    assert.equal(normal.contactMode,'nodal-endpoints');assert.equal(normal.originalInequalities.length,3);
    assert.ok(normal.originalInequalities.every(s=>s.gap>=-1e-8));assert.equal(normal.originalInequalities.find(s=>s.role==='capsule').pressureDof,false);
    friction.samples.forEach((s,i)=>{const Fn=r.state.wallContactState.normalForces[i],Ft=Array.from(r.state.wallFrictionState.tractions.slice(2*i,2*i+2));
        assert.ok(Fn>=0);close(s.Fn,Fn,0);same(s.traction,Ft,0);assert.ok(p[i][1]-.16>=-1e-8);assert.ok(Math.abs(Fn*(p[i][1]-.16))<=1e-9);
        assert.equal(measureCompositeFriction({traction:Ft,slip:s.slip,normalForce:Fn,mu:f.wall.friction.mu,slipTolerance:1e-8,coneTolerance:1e-9,workTolerance:1e-9}).converged,true);
    });
    for(let edge=0;edge<2;edge++)close(Math.hypot(...p[edge+1].map((v,k)=>v-p[edge][k])),r.state.restLengths.get(id)[edge]);
    const balance=r.balances.get(id);same(balance.momentumRate,balance.appliedForce.map((v,k)=>v+balance.boundaryForce[k]+balance.contactForce[k]),1e-7);
    const chord=[p[1][0]-p[0][0],0,p[1][2]-p[0][2]],length=Math.hypot(...chord),axis=chord.map(v=>v/length),circ=[-axis[2],0,axis[0]];
    const expected=[0,1,2].map(k=>friction.samples.reduce((sum,s)=>sum+axis[k]*s.traction[0]+circ[k]*s.traction[1]+(k===1?s.Fn:0),0));
    same(balance.contactForce,expected,1e-12);
    assert.equal(r.diagnostics.contactQueries,r.diagnostics.wallQueries);assert.equal(r.diagnostics.wallQueries,3*r.diagnostics.evaluations);
    return friction.samples[0];
}

test('production joint step accepts proximal catheter feed, independent spin and two original nodal Coulomb laws with free DOFs',context=>{
    const f=fixture(),before=structuredClone(f.state),config=options(f,f.state);assert.equal(config.boundaries.positions.length,1);assert.equal(config.boundaries.spins.length,1);
    assert.equal(config.wall.surfacePosePaths[0].path.configurationDofs,11);
    assert.equal(f.state.layout.dofCount,11);const a=advance(f,f.state);accepted(a);const first=verify(f,a);assert.deepEqual(f.state,before);
    assert.ok(first.Fn>0);assert.ok(first.slip[0]>1e-4);assert.ok(first.traction[0]<0);assert.ok(Math.abs(first.traction[1])>1e-9);
    assert.ok(Math.abs(a.state.toolPositions.get(id)[0][1]-before.toolPositions.get(id)[0][1])>1e-7);assert.ok(Math.abs(a.state.angles.get(id)[1])>1e-5);
    const prior=structuredClone(a.state),b=advance(f,a.state,{dt:1/150});accepted(b);const second=verify(f,b);assert.deepEqual(a.state,prior);
    assert.equal(b.state.step,2);close(b.state.time,1/120+1/150,0);assert.equal(b.state.wallFrictionState.signature,a.state.wallFrictionState.signature);assert.ok(second.Fn>0);
    context.diagnostic(JSON.stringify({first:first,second:second,evaluations:[a.diagnostics.evaluations,b.diagnostics.evaluations],directions:[a.diagnostics.directions,b.diagnostics.directions]}));
});

test('production second-dt late rollback preserves accepted pose, angular/material history and Fn/Ft; reused retry equals cold',context=>{
    const f=fixture(),workspace=createCompositeJointTimeStepWorkspace(f.state),first=advance(f,f.state,{workspace});accepted(first);verify(f,first);
    const before=structuredClone(first.state),portBefore=structuredClone(f.ports.get(first.state)),cold=advance(f,first.state,{dt:1/150});accepted(cold);verify(f,cold);
    const failures=[];
    for(const budget of [{evaluations:cold.diagnostics.evaluations-1},{contactQueries:cold.diagnostics.contactQueries-1}]){
        const config=options(f,first.state,{dt:1/150,workspace,budget}),inertiaBefore=structuredClone(config.inertia),path=config.wall.surfacePosePaths[0].path;
        const failed=advanceCompositeJointTimeStep(first.state,config);assert.equal(failed.accepted,false);assert.equal(failed.state,first.state);assert.deepEqual(first.state,before);
        assert.deepEqual(config.inertia,inertiaBefore);assert.equal(config.wall.surfacePosePaths[0].path,path);assert.deepEqual(f.ports.get(first.state),portBefore);
        assert.ok(['evaluation-budget','contact-query-budget'].includes(failed.status));assert.ok(failed.diagnostics.directions>0);failures.push({status:failed.status,evaluations:failed.diagnostics.evaluations,queries:failed.diagnostics.contactQueries});
        const retry=advance(f,first.state,{dt:1/150,workspace});accepted(retry);verify(f,retry);assert.deepEqual(retry.state,cold.state);assert.deepEqual(retry.balances,cold.balances);
        assert.deepEqual(retry.contactForces,cold.contactForces);assert.deepEqual(retry.spinReactions,cold.spinReactions);assert.deepEqual(retry.diagnostics.certificate,cold.diagnostics.certificate);
    }
    context.diagnostic(JSON.stringify({coldEvaluations:cold.diagnostics.evaluations,coldQueries:cold.diagnostics.contactQueries,failures}));
});

test('production proximal entering labels without explicit pose remain unsupported and cannot erase loaded friction history',()=>{
    const f=fixture(),workspace=createCompositeJointTimeStepWorkspace(f.state),before=structuredClone(f.state);
    const rejected=advance(f,f.state,{workspace,withPath:false});assert.equal(rejected.accepted,false);assert.equal(rejected.status,'surface-material-transport-required');assert.equal(rejected.state,f.state);assert.deepEqual(f.state,before);
    const retry=advance(f,f.state,{workspace}),cold=advance(f,f.state);accepted(retry);accepted(cold);verify(f,retry);assert.deepEqual(retry.state,cold.state);
    const loaded=structuredClone(retry.state);
    assert.throws(()=>advance(f,retry.state,{withPath:false}),/history changed/);assert.deepEqual(retry.state,loaded);
    const config=options(f,retry.state);assert.throws(()=>advanceCompositeJointTimeStep(retry.state,{...config,wall:{...config.wall,mode:'wall-normal',friction:'none'}}),/cannot discard/);
    assert.deepEqual(retry.state,loaded);
});

test('production axial feed and own prescribed spin remain independent controls at the loaded proximal wall',()=>{
    const f=fixture(),baseline=advance(f,f.state);accepted(baseline);const a=verify(f,baseline);
    const slower=advance(f,f.state,{feed:-.15}),reversedSpin=advance(f,f.state,{spin:-.03});accepted(slower);accepted(reversedSpin);
    const b=verify(f,slower),c=verify(f,reversedSpin);
    assert.ok(b.slip[0]>0&&b.slip[0]<.6*a.slip[0]);assert.ok(c.slip[0]>.9*a.slip[0]);assert.ok(a.slip[1]*c.slip[1]<0);
    assert.ok(a.traction[1]*c.traction[1]<0);assert.ok([a,b,c].every(s=>s.Fn>0&&s.traction[0]<0&&s.work<0));
    close(reversedSpin.state.angles.get(id)[0],-.03/120,1e-15);
});

test('production positive-feed solutions preserve original roots across penalties and cold/reused workspaces',()=>{
    const f=fixture(),workspace=createCompositeJointTimeStepWorkspace(f.state),base=advance(f,f.state);accepted(base);
    for(const k of [5,50,500]){
        const cold=advance(f,f.state,{k}),warm=advance(f,f.state,{k,workspace});accepted(cold);accepted(warm);verify(f,warm);assert.deepEqual(warm.state,cold.state);
        same(warm.state.positions.flat(),base.state.positions.flat(),1e-8);same(Array.from(warm.state.wallContactState.normalForces),Array.from(base.state.wallContactState.normalForces),1e-8);
        same(Array.from(warm.state.wallFrictionState.tractions),Array.from(base.state.wallFrictionState.tractions),1e-8);
        const next=advance(f,warm.state,{dt:1/150,k,workspace});accepted(next);verify(f,next);assert.equal(next.state.wallFrictionState.signature,warm.state.wallFrictionState.signature);
    }
});

test('production zero-feed noninteger metric preserves exact proximal/distal label traces across two dt',()=>{
    const f=fixture(),metric=f.state.tools[0].dsDx;assert.notEqual(((20+2*metric)-20)/(2*metric),1);
    const workspace=createCompositeJointTimeStepWorkspace(f.state),first=advance(f,f.state,{feed:0,workspace});accepted(first);verify(f,first);
    const second=advance(f,first.state,{feed:0,dt:1/150,workspace});accepted(second);verify(f,second);assert.equal(second.state.step,2);
    assert.ok(second.state.wallContactState.normalForces[0]>0);assert.ok(Math.abs(second.state.wallFrictionState.tractions[1])>1e-9);
});

test('actual World commits proximal catheter feed and retains its prepared path through a second-dt rejection',()=>{
    const f=fixture(),initialPort=structuredClone(f.ports.get(f.state));let bindings,preparations=0;
    const system=createCompositeJointWorldAdapter({initialize:()=>({state:f.state,bindings}),prepareStep({state,dt}){
        preparations++;
        // This fixture's fixed straight entrance relation owns the exterior
        // pose. Evaluate it only from accepted state, never a trial candidate.
        if(state.step===0)f.ports.set(state,initialPort);else rememberAcceptedPort(f,state);
        return options(f,state,{dt});
    }});
    const world=new EndovascularPhysicsWorld({fixedDt:1/120,wholeStepSystem:system}),body=world.createRod(id,3,2);
    f.state.toolPositions.get(id).forEach((p,n)=>body.setNodePosition(n,...p));body.copyCurrentToPrevious();
    bindings=[{toolId:id,body,nodes:[0,1,2].map(node=>({node,jointNode:node,trace:node===2?'left':'right'})),edges:[0,1].map(edge=>({edge,jointEdge:edge}))}];
    const direct1=advance(f,f.state);accepted(direct1);verify(f,direct1);
    assert.equal(world.advance(1/120),1);assert.deepEqual(system.snapshot(),direct1.state);
    const prior=system.snapshot(),view=body.jointStateView,oldX=body.x.slice(),oldY=body.y.slice();
    const direct2=advance(f,direct1.state);accepted(direct2);verify(f,direct2);
    system.setBudget({directions:0});assert.equal(world.advance(1/120),0);assert.equal(world.lastStepResult.status,'direction-budget');
    assert.deepEqual(system.snapshot(),prior);assert.equal(body.jointStateView,view);assert.deepEqual(body.x,oldX);assert.deepEqual(body.y,oldY);
    assert.equal(world.accumulator,1/120);assert.equal(world.stepCount,1);
    system.setBudget(null);assert.equal(world.advance(0),1);assert.deepEqual(system.snapshot(),direct2.state);
    assert.equal(preparations,2);assert.equal(world.stepCount,2);assert.equal(world.accumulator,0);
    assert.equal(world.lastStepResult.diagnostics.certificate.wallFriction.converged,true);
    assert.deepEqual(Array.from(body.jointStateView.positions),direct2.state.toolPositions.get(id).flat());
    assert.deepEqual(Array.from(body.jointStateView.unwrappedAngles),Array.from(direct2.state.angles.get(id)));
});

test('source-derived actual World wall retains the explicit proximal feed path, radii and loaded history across two dt and retry',()=>{
    const f=fixture(),initialPort=structuredClone(f.ports.get(f.state));let bindings,preparations=0,lastInput;
    f.materialSegmentId=edge=>JSON.stringify(['world-wall',id,id,edge,edge]);
    const system=createCompositeJointWorldAdapter({worldWall:{plane:f.wall.plane},initialize:()=>({state:f.state,bindings}),prepareStep({state,dt}){
        preparations++;if(state.step===0)f.ports.set(state,initialPort);else rememberAcceptedPort(f,state);
        const o=options(f,state,{dt});return lastInput={...o,wall:'none',worldWallSurfacePosePaths:o.wall.surfacePosePaths};
    }});
    const world=new EndovascularPhysicsWorld({fixedDt:1/120,contactField:f.wall.field,wholeStepSystem:system}),body=world.createRod(id,3,2);
    f.state.toolPositions.get(id).forEach((p,n)=>body.setNodePosition(n,...p));body.nodeRadius.fill(.16);body.setCollisionRange(0,0);
    body.wallStaticFriction=body.wallKineticFriction=.006;body.copyCurrentToPrevious();
    bindings=[{toolId:id,body,nodes:[0,1,2].map(node=>({node,jointNode:node,trace:node===2?'left':'right'})),edges:[0,1].map(edge=>({edge,jointEdge:edge}))}];
    const direct=state=>{const o=options(f,state),{wall}=prepareCompositeJointWorldWall({world,state,bindings,plane:f.wall.plane,surfacePosePaths:o.wall.surfacePosePaths});
        const r=advanceCompositeJointTimeStep(state,{...o,wall});accepted(r);rememberAcceptedPort(f,r.state);return r;};
    const a=direct(f.state);assert.equal(world.advance(1/120),1);assert.deepEqual(system.snapshot(),a.state);
    const sample=world.lastStepResult.diagnostics.certificate.wallFriction.samples[0];
    assert.ok(sample.Fn>0);assert.ok(sample.slip[0]>1e-4);assert.ok(sample.traction[0]<0);
    assert.equal(world.lastStepResult.diagnostics.worldAdapter.wallSource.originalCapsules,1);
    assert.equal(world.lastStepResult.diagnostics.worldAdapter.wallSource.jointCapsules,1);
    const b=direct(a.state),before=system.snapshot();system.setBudget({contactQueries:b.diagnostics.contactQueries-1});
    assert.equal(world.advance(1/120),0);assert.deepEqual(system.snapshot(),before);
    lastInput.worldWallSurfacePosePaths[0].owner='wire';lastInput.worldWallSurfacePosePaths.length=0;
    system.setBudget(null);assert.equal(world.advance(0),1);assert.deepEqual(system.snapshot(),b.state);assert.equal(preparations,2);
    assert.equal(world.accumulator,0);assert.equal(world.stepCount,2);
    const acceptedState=system.snapshot(),p=acceptedState.toolPositions.get(id),radius=body.nodeRadius[0];
    assert.ok(Math.min(p[0][1],p[1][1])-radius>=-1e-8);assert.equal(world.lastStepResult.diagnostics.certificate.wallFriction.converged,true);
});

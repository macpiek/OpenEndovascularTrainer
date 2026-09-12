import test from 'node:test';
import assert from 'node:assert/strict';
import {BufferGeometry,Float32BufferAttribute,Vector3} from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {createContactResult} from '../src/physics/collision/vesselContactField.js';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeJointTimeStepState,advanceCompositeJointTimeStep,createCompositeJointTimeStepWorkspace} from '../src/physics/kirchhoffCompositeJointTimeStep.js';
import {prepareCompositeJointWorldWall,assertCompositeJointWorldWall} from '../src/physics/kirchhoffCompositeJointWorldWall.js';
import {createCompositeJointContinuousWallRows} from '../src/physics/kirchhoffCompositeJointContinuousWallRows.js';

const dt=1/120,ids=['wire','catheter'],zero=[0,0,0],close=(a,b,t=1e-8)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}`);
function fixture({voxelSize=1,coordinates=[0,1,2],curved=false}={}) {
    const mesh=new BufferGeometry();mesh.setAttribute('position',new Float32BufferAttribute([[-50,0,-50],[50,0,-50],[0,0,50]].flat(),3));mesh.boundsTree=new MeshBVH(mesh);
    const point=new Vector3(),target={point:new Vector3(),distance:Infinity,faceIndex:-1},field={voxelSize,fallbackGeometry:mesh,calls:0};
    field.querySphere=(p,radius,out=createContactResult())=>{field.calls++;mesh.boundsTree.closestPointToPoint(point.fromArray(p),target);
        const sign=p[1]>=0?1:-1;out.source='sparse-sdf-bvh';out.faceIndex=target.faceIndex;out.signedDistance=sign*target.distance;out.signedGap=out.signedDistance-radius;
        out.closestPoint.values.set(target.point.toArray());out.inward.values.set(p.map((v,k)=>sign*(v-target.point.getComponent(k))/target.distance));return out;};
    field.queryCapsuleCoordinates=(ax,ay,az,bx,by,bz,radius,out=createContactResult())=>field.querySphere([ax,ay,az],radius,out);
    const n=coordinates.length,layout=createCompositeChainLayout(coordinates.slice(1).map(()=>ids)),positions=coordinates.map(x=>[x,curved?.225+.2*(x-1.5)**2:.25,0]),
        wire=positions.map(p=>[p[0],p[1],.125]),modes=coordinates.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]})),tools=ids.map(id=>({id,dsDx:1,
            reference:captureCompositeReferenceFrames(id==='wire'?wire:positions),referenceTwists:new Float64Array(n-2),material:compileCompositeMaterial({EI1:2,EI2:3,GJ:1})})),
        state=createCompositeJointTimeStepState({layout,coordinates,positions,tools,modes,relative:modes.flatMap(()=>[0,0,.125]),angles:new Map(ids.map(id=>[id,new Float64Array(n-1)])),
            restLengths:new Map(ids.map(id=>[id,positions.slice(1).map((p,i)=>Math.hypot(...p.map((v,k)=>v-positions[i][k])))])),
            inertiaGeometryByTool:new Map(ids.map(id=>[id,{}])),elasticityGeometry:'continuous-material-frame',lengthGeometry:curved?'native-chords':'continuous-arclength',materialCoordinate:'reference-arclength'}),
        world=new EndovascularPhysicsWorld({contactField:field,fixedDt:dt}),bindings=ids.map(toolId=>({toolId,body:world.createRod(toolId==='wire'?'guidewire':'catheter',n,1),
            nodes:coordinates.map((_,node)=>({node,jointNode:node}))}));
    bindings.forEach(b=>{b.body.nodeRadius.fill(.25);b.body.wallStaticFriction=b.body.wallKineticFriction=0;b.body.setCollisionRange(0,n-2);});
    const source=(current=state,inertia=options(current,'none').inertia)=>prepareCompositeJointWorldWall({world,state:current,bindings,contactMode:'continuous-samples',inertia});
    return {state,world,bindings,field,source};
}
function options(state,wall) {
    return {dt,torsionMode:'quasi-static',contacts:'none',wall,
        inertia:{previousPositions:structuredClone(state.toolPositions),inertiaEdges:state.layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,massPerMaterialLength:.13,
            materialMap:{sStart:20+state.coordinates[e],dsDx:1,dsDt:0},oldMaterialVelocities:[[id==='wire'?.2:-.1,0,0],[id==='wire'?.2:-.1,0,0]]}))}))},
        boundaries:{positions:[],spins:ids.map(toolId=>({toolId,edge:0,value:toolId==='wire'?.1:-.2}))},
        loads:{forces:ids.flatMap(toolId=>Array.from({length:state.layout.nodeCount},(_,node)=>({toolId,node,value:[0,toolId==='wire'?-1:-.4,0]}))),torques:[]}};
}

test('normal contacts at actual material samples solve both C2 tools together with signed reactions, independent feed/spin and exact retry',()=>{
    const f=fixture(),p=f.source(),input=options(f.state,p.wall),workspace=createCompositeJointTimeStepWorkspace(f.state),original=structuredClone(f.state),
        limited=advanceCompositeJointTimeStep(f.state,{...input,workspace,budget:{directions:0}});
    assert.equal(limited.accepted,false);assert.deepEqual(f.state,original);
    const r=advanceCompositeJointTimeStep(f.state,{...input,workspace});assert.equal(r.accepted,true,JSON.stringify({status:r.status,error:r.error,certificate:r.diagnostics?.certificate}));
    assert.equal(r.diagnostics.certificate.wall.coverage,'declared-original-resolution-samples');assert.equal(r.diagnostics.certificate.wall.continuumClearanceCertified,false);
    for(const id of ids)for(let node=0;node<3;node++){
        close(r.state.toolPositions.get(id)[node][0],node+dt*(id==='wire'?.2:-.1),1e-9);close(r.state.toolPositions.get(id)[node][1],.25,1e-9);
        close(r.contactForces.get(id)[node][1],id==='wire'?1:.4,1e-7);
    }
    for(const b of r.balances.values())b.residual.forEach(v=>close(v,0,1e-7));
    const prepared=f.source(r.state),second=advanceCompositeJointTimeStep(r.state,{...options(r.state,prepared.wall),workspace});
    assert.equal(second.accepted,true,JSON.stringify({status:second.status,error:second.error}));assert.ok(second.state.wallContactState.records.some(r=>r.force>0));
    assert.deepEqual(f.state,original);assert.throws(()=>advanceCompositeJointTimeStep(second.state,{...options(second.state,prepared.wall),wall:'none'}),/cannot discard/);
    f.field.fallbackGeometry.dispose();
});

test('source preserves every inherited interior sample and its true C2 penetration when all source nodes are open',()=>{
    const f=fixture({voxelSize:.125,coordinates:[0,1,2,3],curved:true}),prepared=f.source();
    assert.ok(f.state.toolPositions.get('wire').every(p=>p[1]>.25));
    assert.equal(prepared.proof.continuumClearanceCertified,false);assert.equal(prepared.wall.samplingIntervals.length,6);
    assert.ok(prepared.wall.pressureSites.some(s=>s.owner==='wire'&&s.edge===1&&s.fraction===.5));
    const manager=createCompositeJointContinuousWallRows({layout:f.state.layout,coordinates:f.state.coordinates,modes:f.state.modes,wall:prepared.wall,
        geometryByTool:f.state.inertiaGeometryByTool,tolerances:{force:1e-7,wallGap:1e-8,wallNcp:1e-8,wallWork:1e-9,linearConstraint:1e-10}}),
        certificate=manager.refresh({toolPositions:f.state.toolPositions,commonResidual:new Float64Array(f.state.layout.dofCount),relativeResidual:new Float64Array(f.state.relative.length),order:'full'});
    assert.equal(certificate.converged,false);assert.ok(certificate.samples.some(s=>s.edge===1&&s.fraction===.5&&s.gap<-.02));assert.throws(()=>manager.commit(),/freshly certified/);
    const proofArgs={world:f.world,state:f.state,bindings:f.bindings,...prepared};assert.equal(assertCompositeJointWorldWall(proofArgs),prepared.proof);
    f.bindings[0].body.wallStaticFriction=.01;assert.throws(()=>assertCompositeJointWorldWall(proofArgs),/changed/);
    f.field.fallbackGeometry.dispose();
});

for(const scenario of ['static','sliding','breakaway','spin'])test(`actual sampled C2 wall static/kinetic source: ${scenario}, owned rates and retry`,()=>{
    const f=fixture(),input=options(f.state,'none');input.boundaries.spins.forEach(b=>b.value=scenario==='spin'?(b.toolId==='wire'?.0005:-.0003):0);
    for(const b of f.bindings) {
        b.body.wallStaticFriction=.05;b.body.wallKineticFriction=.02;
        b.body.velocityX.fill(scenario==='sliding'?(b.toolId==='wire'?.2:-.1):0);const v=b.body.velocityX[0];
        for(const edge of input.inertia.inertiaEdges){const t=edge.tools.find(t=>t.id===b.toolId);t.oldMaterialVelocities=[[v,0,0],[v,0,0]];}
    }
    if(scenario==='static'||scenario==='breakaway')input.loads.forces.forEach(f=>f.value[0]=scenario==='static'?.001:.08);
    const p=f.source(f.state,input.inertia);input.wall=p.wall;
    assert.equal(p.wall.mode,'wall-coulomb');assert.deepEqual(p.wall.friction.muByOwner[0].muStatic,[.05,.05]);assert.equal(p.wall.friction.finiteStepSlipKnown,false);
    const before=structuredClone(f.state),workspace=createCompositeJointTimeStepWorkspace(f.state),limited=advanceCompositeJointTimeStep(f.state,{...input,workspace,budget:{directions:0}});
    assert.equal(limited.accepted,false);assert.deepEqual(f.state,before);
    const result=advanceCompositeJointTimeStep(f.state,{...input,workspace});
    assert.equal(result.accepted,true,JSON.stringify({status:result.status,error:result.error,certificate:result.diagnostics?.certificate,modes:result.diagnostics?.wallFrictionModes}));
    const certificate=result.diagnostics.certificate.wallFriction;assert.equal(certificate.finiteStepSlipKnown,false);assert.ok(certificate.samples.every(s=>s.converged));
    for(const s of certificate.samples)if(!s.unloaded){assert.ok(s.work<=1e-9);assert.ok(s.coneViolation<=1e-9);assert.ok(s.workGap<=1e-9);}
    if(scenario==='static')assert.ok(certificate.samples.every(s=>s.slip.every(v=>Math.abs(v)<1e-8)));
    if(scenario==='sliding')assert.ok(certificate.samples.every(s=>s.mode==='kinetic'));
    if(scenario==='breakaway')assert.ok(result.diagnostics.wallFrictionModeChanges>0);
    if(scenario==='spin')assert.ok([...result.spinReactions.values()].some(v=>v.some(x=>Math.abs(x)>1e-7)));
    for(const b of result.balances.values())b.residual.forEach(v=>close(v,0,2e-7));assert.equal(result.state.continuousRateHistory.kind,'continuous-backward-euler-material-rate');
    const secondInput=options(result.state,'none');secondInput.boundaries.spins=input.boundaries.spins;secondInput.loads=input.loads;
    // Prepared translational inertia keeps the accepted rate history while
    // friction owns the exact preceding implicit angular reconstruction.
    secondInput.inertia.inertiaEdges.forEach((edge,e)=>edge.tools.forEach(t=>{delete t.oldMaterialVelocities;const old=result.state.materialVelocities[e].tools.find(r=>r.id===t.id);t.oldVelocityPieces=[{fractions:[0,1],interpretation:old.interpretation,bernsteinVelocities:old.bernsteinVelocities.map(v=>v.slice())}];}));
    secondInput.wall=f.source(result.state,secondInput.inertia).wall;const second=advanceCompositeJointTimeStep(result.state,{...secondInput,workspace});
    assert.equal(second.accepted,true,JSON.stringify({status:second.status,error:second.error,certificate:second.diagnostics?.certificate}));
    assert.deepEqual(f.state,before);f.field.fallbackGeometry.dispose();
});

test('first exposed C2 wall contact after an accepted free step imports its own moving material history',()=>{
    const f=fixture();for(const b of f.bindings){b.body.setCollisionRange(0,-1);b.body.wallStaticFriction=.05;b.body.wallKineticFriction=.02;}
    const free=options(f.state,f.source().wall);free.loads.forces=[];free.boundaries.spins.forEach(b=>b.value=0);
    const first=advanceCompositeJointTimeStep(f.state,free);assert.equal(first.accepted,true,first.error);assert.equal(first.state.wallFrictionState,undefined);
    assert.equal(first.state.continuousRateHistory.kind,'continuous-backward-euler-material-rate');
    f.bindings.forEach(b=>b.body.setCollisionRange(0,1));const loaded=options(first.state,'none');loaded.boundaries.spins.forEach(b=>b.value=0);
    loaded.inertia.inertiaEdges.forEach((edge,e)=>edge.tools.forEach(t=>{delete t.oldMaterialVelocities;const old=first.state.materialVelocities[e].tools.find(r=>r.id===t.id);
        t.oldVelocityPieces=[{fractions:[0,1],interpretation:old.interpretation,bernsteinVelocities:old.bernsteinVelocities.map(v=>v.slice())}];}));
    const source=f.source(first.state,loaded.inertia);loaded.wall=source.wall;assert.deepEqual(source.wall.friction.incomingSurfaceMotion,[]);
    const result=advanceCompositeJointTimeStep(first.state,loaded);assert.equal(result.accepted,true,JSON.stringify({status:result.status,error:result.error}));
    assert.ok(result.diagnostics.certificate.wallFriction.samples.every(s=>s.mode==='kinetic'&&s.Fn>0));f.field.fallbackGeometry.dispose();
});

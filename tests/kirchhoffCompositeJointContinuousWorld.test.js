import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeJointTimeStepState,createCompositeJointTimeStepWorkspace,advanceCompositeJointTimeStep,iterateCompositeJointTimeStep} from '../src/physics/kirchhoffCompositeJointTimeStep.js';
import {createCompositeJointWorldAdapter} from '../src/physics/kirchhoffCompositeJointWorldAdapter.js';
import {createCompositeJointMaterialHistory} from '../src/physics/kirchhoffCompositeJointMaterialHistory.js';
import {evaluateCompositeContinuousGeometry} from '../src/physics/kirchhoffCompositeContinuousGeometry.js';
import {getCompositeJointRenderPath} from '../src/compositeJointRenderPath.js';
import {updateSmoothTubeGeometry} from '../src/smoothTubeGeometry.js';

const dt=1/120,ids=['wire','catheter'],close=(a,b,t=1e-10)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}`);
function fixture(cooperative=false) {
    const coordinates=[0,1,2,3,4,5],positions=coordinates.map(x=>[x,0,0]),
        own=id=>positions.map(p=>p.map((v,k)=>v+(id==='wire'&&k===1?.03:0))),
        tools=ids.map(id=>({id,dsDx:1,reference:captureCompositeReferenceFrames(own(id),[0,0,1]),referenceTwists:new Float64Array(4),
            material:compileCompositeMaterial({EI1:id==='wire'?2:9,EI2:id==='wire'?3:12,GJ:id==='wire'?1:4})})),
        state=createCompositeJointTimeStepState({layout:createCompositeChainLayout(coordinates.slice(1).map(()=>ids)),coordinates,positions,tools,
            modes:coordinates.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]})),relative:coordinates.flatMap(()=>[0,.03,0]),
            angles:new Map(ids.map(id=>[id,new Float64Array(5)])),restLengths:new Map(ids.map(id=>[id,new Float64Array(5).fill(1)])),
            inertiaGeometryByTool:new Map(ids.map(id=>[id,{}])),elasticityGeometry:'continuous-material-frame',lengthGeometry:'continuous-arclength',materialCoordinate:'reference-arclength'});
    function options(current) {
        const inertia={dt,previousPositions:structuredClone(current.toolPositions),inertiaEdges:current.layout.edgeToolIds.map((tools,e)=>({tools:tools.map(id=>({id,
            massPerMaterialLength:id==='wire'?.13:.24,materialMap:{sStart:20+current.coordinates[e],dsDx:1,dsDt:0},oldMaterialVelocities:[[0,0,0],[0,0,0]]}))}))};
        if(current.step) {
            const prepared=createCompositeJointMaterialHistory({materialVelocities:current.materialVelocities}).prepare({coordinates:current.coordinates,inertiaEdges:inertia.inertiaEdges});
            for(let e=0;e<inertia.inertiaEdges.length;e++)for(const t of inertia.inertiaEdges[e].tools){delete t.oldMaterialVelocities;t.oldVelocityPieces=prepared.inertiaEdges[e].tools.find(v=>v.id===t.id).pieces;}
        }
        return {dt,torsionMode:'quasi-static',contacts:'none',inertia,
            boundaries:{positions:ids.flatMap(toolId=>[0,1].map(node=>({toolId,node,value:state.toolPositions.get(toolId)[node]}))),spins:ids.map(toolId=>({toolId,edge:0,value:0}))},
            loads:{forces:[{toolId:'wire',node:5,value:[0,.004,.002]},{toolId:'catheter',node:5,value:[0,-.003,-.002]}]}};
    }
    let bindings;
    const adapter=createCompositeJointWorldAdapter({cooperative,initialize:()=>({state,bindings}),prepareStep:({state})=>options(state)}),world=new EndovascularPhysicsWorld({fixedDt:dt,wholeStepSystem:adapter});
    bindings=ids.map(toolId=>{const body=world.createRod(toolId==='wire'?'guidewire':toolId,6,1);own(toolId).forEach((p,n)=>body.setNodePosition(n,...p));body.copyCurrentToPrevious();
        return {body,toolId,nodes:coordinates.map((_,node)=>({node,jointNode:node,trace:node===5?'left':'right'})),edges:coordinates.slice(1).map((_,edge)=>({edge,jointEdge:edge}))};});
    return {state,adapter,world,bindings,options};
}

test('continuous elasticity/length joint steps publish polynomial geometry and velocities through actual World, retain own history and rollback',()=>{
    const f=fixture();let previous=f.state;
    for(let step=0;step<2;step++) {
        const expected=advanceCompositeJointTimeStep(previous,f.options(previous));assert.equal(expected.accepted,true,expected.status);
        if(step===1) {
            const views=f.bindings.map(b=>b.body.jointStateView),before=f.adapter.snapshot();f.adapter.setBudget({directions:0});
            assert.equal(f.world.advance(dt),0);assert.equal(f.world.lastStepResult.status,'direction-budget');
            assert.deepEqual(f.adapter.snapshot(),before);f.bindings.forEach((b,j)=>assert.equal(b.body.jointStateView,views[j]));f.adapter.setBudget(null);
        }
        assert.equal(f.world.advance(step===1?0:dt),1,JSON.stringify(f.world.lastStepResult));
        const accepted=f.adapter.snapshot();assert.deepEqual(accepted,expected.state);assert.equal(f.world.accumulator,0);
        for(const {body,toolId} of f.bindings) {
            const view=body.jointStateView,path=getCompositeJointRenderPath(view);assert.equal(getCompositeJointRenderPath(view),path);
            assert.equal(view.continuousCurve.controls.length,90);assert.equal(view.continuousCurve.materialVelocityControls.length,90);
            for(let e=0;e<5;e++) {
                const record=accepted.materialVelocities[e].tools.find(t=>t.id===toolId);
                assert.deepEqual(Array.from(view.continuousCurve.materialVelocityControls.slice(18*e,18*(e+1))),record.bernsteinVelocities.flat());
                for(const [end,j] of [[0,0],[1,5]])for(let k=0;k<3;k++)close(view.edgeMaterialVelocities[6*e+3*end+k],record.bernsteinVelocities[j][k]);
                for(const fraction of [0,.17,.5,.83,1]) {
                    const geometry=accepted.inertiaGeometryByTool.get(toolId).edges[e],sample=evaluateCompositeContinuousGeometry(geometry,{fraction,positions:geometry.nodeIndices.map(n=>accepted.toolPositions.get(toolId)[n])}),u=(e+fraction)/5;
                    path.getPointAt(u,new THREE.Vector3()).toArray().forEach((v,k)=>close(v,sample.position[k]));
                    path.getTangentAt(u,new THREE.Vector3()).toArray().forEach((v,k)=>close(v,sample.tangent[k]));
                }
            }
            // Rendering/publication cannot overwrite the next physical state.
            view.continuousCurve.controls.fill(999);view.continuousCurve.materialVelocityControls.fill(999);
        }
        assert.deepEqual(f.adapter.snapshot(),expected.state);previous=accepted;
    }
    f.world.resetSimulationState();f.bindings.forEach(b=>assert.equal(b.body.jointStateView,undefined));
});

test('suspended numerical work publishes only the accepted dt, matches sync physics, and can be cancelled on reset',()=>{
    const f=fixture(true),expected=advanceCompositeJointTimeStep(f.state,f.options(f.state));
    assert.equal(expected.accepted,true,expected.status);
    const source=structuredClone(f.state.toolPositions),body=f.bindings[0].body,x=body.x.slice();
    assert.equal(f.world.advance(dt),0);assert.equal(f.world.lastStepResult.status,'computing');
    assert.throws(()=>f.adapter.setBudget({directions:1}),/suspended/);
    let attempts=0;
    while(f.world.lastStepResult.status==='computing'){
        assert.equal(f.world.stepCount,0);assert.deepEqual(body.x,x);assert.equal(body.jointStateView,undefined);
        assert.deepEqual(f.adapter.snapshot().toolPositions,source);
        assert.ok(attempts++<100);f.world.advance(0);
    }
    assert.equal(f.world.lastStepResult.accepted,true,JSON.stringify(f.world.lastStepResult));
    assert.ok(attempts>2);assert.equal(f.world.stepCount,1);assert.equal(f.world.accumulator,0);
    assert.equal(f.adapter.diagnostics.preparations,1);assert.deepEqual(f.adapter.snapshot(),expected.state);
    f.world.advance(dt);assert.equal(f.world.lastStepResult.status,'computing');
    f.world.resetSimulationState();assert.equal(body.jointStateView,undefined);
    assert.equal(f.adapter.snapshot(),null);assert.equal(f.world.advance(dt),0);
    assert.equal(f.world.lastStepResult.status,'computing');f.world.resetSimulationState();
});

test('iterator cancellation releases its busy workspace and cooperative rejection does not rerun unchanged input',()=>{
    const f=fixture(true),workspace=createCompositeJointTimeStepWorkspace(f.state),options={...f.options(f.state),workspace};
    const iterator=iterateCompositeJointTimeStep(f.state,options);assert.equal(iterator.next().done,false);
    assert.throws(()=>advanceCompositeJointTimeStep(f.state,options),/busy/);
    iterator.return();assert.equal(advanceCompositeJointTimeStep(f.state,options).accepted,true);
    f.adapter.setBudget({directions:0});f.world.advance(dt);
    for(let i=0;i<20&&f.world.lastStepResult.status==='computing';i++)f.world.advance(0);
    assert.equal(f.world.lastStepResult.status,'direction-budget');
    const calls=f.adapter.diagnostics.workspace.calls,rejections=f.adapter.diagnostics.rejected;
    f.world.advance(0);f.world.advance(0);
    assert.equal(f.adapter.diagnostics.workspace.calls,calls);assert.equal(f.adapter.diagnostics.rejected,rejections);
    f.adapter.setBudget(null);f.world.advance(0);assert.equal(f.world.lastStepResult.status,'computing');
    f.world.resetSimulationState();
});

test('tube rings follow the exact published quintic and tangent without a replacement display spline; topology is reused',()=>{
    const controls=Float64Array.from([[0,0,0],[.2,.8,0],[.4,-.2,.1],[.6,.3,.2],[.8,.1,.1],[1,0,0]].flat()),
        view={continuousCurve:{kind:'quintic-bernstein',coordinates:new Float64Array([3,7]),controls}},path=getCompositeJointRenderPath(view),
        geometry=updateSmoothTubeGeometry(null,null,{path,radius:.1,radialSegments:8,samplesPerSegment:16}),array=geometry.attributes.position.array;
    assert.equal(geometry.parameters.path,path);
    for(let ring=0;ring<=16;ring++) {
        const center=new THREE.Vector3();for(let radial=0;radial<8;radial++)center.add(new THREE.Vector3().fromArray(array,3*(ring*9+radial)));center.multiplyScalar(1/8);
        close(center.distanceTo(path.getPointAt(ring/16,new THREE.Vector3())),0,5e-8);
        close(geometry.tangents[ring].distanceTo(path.getTangentAt(ring/16,new THREE.Vector3())),0,1e-14);
    }
    assert.equal(updateSmoothTubeGeometry(geometry,null,{path,radius:.15,radialSegments:8,samplesPerSegment:16}),geometry);
    assert.equal(geometry.attributes.position.array,array);geometry.dispose();
    assert.throws(()=>path.getPointAt(-.1,new THREE.Vector3()),/leaves/);assert.equal(getCompositeJointRenderPath(null),null);
});

test('native Joint rendering retains every union node and uses the same physical piecewise segments',()=>{
    const view={coordinates:new Float64Array([0,1,3]),positions:new Float64Array([0,0,0,1,1,0,3,0,0])},path=getCompositeJointRenderPath(view);
    assert.equal(path.pointCount,3);assert.equal(getCompositeJointRenderPath(view),path);
    assert.deepEqual(path.getPointAt(1/3,new THREE.Vector3()).toArray(),[1,1,0]);
    assert.deepEqual(path.getPointAt(2/3,new THREE.Vector3()).toArray(),[2,.5,0]);
    close(path.getTangentAt(.8,new THREE.Vector3()).dot(new THREE.Vector3(2,-1,0).normalize()),1);
    assert.throws(()=>getCompositeJointRenderPath({...view,positions:new Float64Array(3)}),/Invalid/);
});

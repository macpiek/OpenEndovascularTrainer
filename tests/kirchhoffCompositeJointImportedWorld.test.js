import assert from 'node:assert/strict';
import test from 'node:test';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createBishopFrame,quaternionExp,multiplyQuaternions} from '../src/physics/discreteKirchhoffRod.js';
import {importCompositeJointWorld} from '../src/physics/kirchhoffCompositeJointWorldImport.js';
import {createCompositeJointWorldAdapter} from '../src/physics/kirchhoffCompositeJointWorldAdapter.js';
import {advanceCompositeJointTimeStep} from '../src/physics/kirchhoffCompositeJointTimeStep.js';

const dt=1/120,accepted=r=>assert.equal(r.accepted,true,JSON.stringify(r));
const obj=v=>({x:v[0],y:v[1],z:v[2]});
const close=(a,b,t=1e-8)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}`);
function fixture(){
    let imported,initializations=0,preparations=0,lastInput;
    const adapter=createCompositeJointWorldAdapter({initialize(){initializations++;return imported;},prepareStep({state}){preparations++;return lastInput=options(state);}});
    const world=new EndovascularPhysicsWorld({fixedDt:dt,wholeStepSystem:adapter});
    const descriptors=[['wire','guidewire',[0,5,10,15,20],5,4*Math.PI,.13],['catheter','catheter',[0,4,8,12],4,-6*Math.PI,.24]].map(([toolId,id,coordinates,spacing,angle,density])=>{
        const body=world.createRod(id,coordinates.length+2,spacing);body.activeStart=1;body.activeEnd=coordinates.length;
        coordinates.forEach((x,i)=>{body.setNodePosition(i+1,x,toolId==='wire'?.341:0,0);if(i+1<coordinates.length)body.restLength[i+1]=coordinates[i+1]-x;});
        const reference=captureCompositeReferenceFrames(coordinates.map((_,i)=>[body.x[i+1],body.y[i+1],body.z[i+1]]));
        const angles=reference.map(()=>angle);
        reference.forEach((f,i)=>{
            const q=multiplyQuaternions(quaternionExp(obj(f.tangent.map(v=>v*angle))),createBishopFrame(obj(f.tangent),obj(f.director)));
            for(const key of ['x','y','z','w'])body[`orientation${key.toUpperCase()}`][i+1]=q[key];
        });
        body.copyCurrentToPrevious();
        const material=compileCompositeMaterial({EI1:toolId==='wire'?2:8,EI2:toolId==='wire'?3:11,GJ:toolId==='wire'?1:4});
        return {body,toolId,nodeCoordinates:coordinates,reference,angles,referenceTwists:reference.slice(1).map(()=>0),winding:'explicit-unwrapped',
            velocityInterpretation:'physical-material-velocity',materialLabels:coordinates.map(x=>20+x),material:{dsDx:1,massPerMaterialLength:density,materialAt:()=>material}};
    });
    imported=importCompositeJointWorld({tools:descriptors});
    function options(state){
        return {dt,torsionMode:'quasi-static',wall:'none',
            contacts:{mode:'lumen-coulomb',chartId:'imported-world',forcePerLength:1,
                friction:{law:'coulomb',mu:[.015,.006],forcePerLength:50,materialPath:'linear-affine-maps'},
                pairs:[{id:'proximal-side',innerToolId:'wire',outerToolId:'catheter',innerEdge:0,outerEdge:0,innerMaterialSegmentId:'wire:0',outerMaterialSegmentId:'catheter:0',
                    lumenRadius:.5,innerRadius:.16,quadrature:[.25],openDistal:false,portalFilletRadius:0}]},
            inertia:{previousPositions:structuredClone(state.toolPositions),inertiaEdges:state.layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({
                id,massPerMaterialLength:descriptors.find(t=>t.toolId===id).material.massPerMaterialLength,
                materialMap:{sStart:20+state.coordinates[e],dsDx:1,dsDt:0},
                oldMaterialVelocities:structuredClone(state.materialVelocities[e].tools.find(t=>t.id===id).velocities)
            }))}))},
            boundaries:{positions:descriptors.map(t=>{const node=imported.mappings.get(t.toolId).nodes.at(-1).jointNode;return {toolId:t.toolId,node,value:imported.state.toolPositions.get(t.toolId)[node].slice()};}),
                spins:descriptors.map(t=>({toolId:t.toolId,edge:0,value:t.angles[0]+(t.toolId==='wire'?.03:-.01)*(state.time+dt)}))},
            loads:{forces:[{toolId:'wire',node:0,value:[0,.4,0]}],torques:[{toolId:'wire',edge:1,value:.001}]}};
    }
    return {world,adapter,descriptors,imported,options,get counts(){return {initializations,preparations};},get lastInput(){return lastInput;}};
}
function verify(f){
    const state=f.adapter.snapshot(),result=f.world.lastStepResult;
    assert.equal(result.diagnostics.certificate.converged,true);assert.equal(result.diagnostics.certificate.friction.converged,true);
    for(const b of f.imported.bindings){
        const view=b.body.jointStateView,mapping=f.imported.mappings.get(b.toolId);
        assert.equal(view.time,state.time);assert.equal(view.coarseBodyArraysArePhysicsState,false);
        assert.ok(view.edges.length>b.edges.length,'Refined own edge poses must survive publication');
        assert.deepEqual(Array.from(view.nodes),mapping.jointNodes.map(r=>r.jointNode));
        assert.deepEqual(Array.from(view.edges),mapping.jointEdges.map(r=>r.jointEdge));
        assert.deepEqual(Array.from(view.positions),Array.from(view.nodes,n=>state.toolPositions.get(b.toolId)[n]).flat());
        assert.deepEqual(Array.from(view.unwrappedAngles),Array.from(view.edges,e=>state.angles.get(b.toolId)[e]));
        for(const r of b.nodes)for(let k=0;k<3;k++)assert.equal(b.body[['x','y','z'][k]][r.node],Math.fround(state.toolPositions.get(b.toolId)[r.jointNode][k]));
        for(const r of b.edges){
            const offset=Array.from(view.edges).indexOf(r.jointEdge)*4;
            const q=['orientationX','orientationY','orientationZ','orientationW'].map(key=>b.body[key][r.edge]);
            const target=Array.from(view.orientations.slice(offset,offset+4)),sign=q.reduce((sum,v,k)=>sum+v*target[k],0)<0?-1:1;
            q.forEach((v,k)=>close(v,sign*target[k],2e-14));
        }
    }
    for(const balance of result.balances.values())balance.momentumRate.forEach((v,k)=>close(v,balance.appliedForce[k]+balance.boundaryForce[k]+balance.contactForce[k],1e-7));
    return state;
}

test('actual differently spaced World rods import once, solve two shared dt and retain all union subedge poses',()=>{
    const f=fixture();assert.deepEqual(Array.from(f.imported.state.coordinates),[0,4,5,8,10,12,15,20]);
    const before=structuredClone(f.imported.state.toolPositions);
    const direct1=advanceCompositeJointTimeStep(f.imported.state,f.options(f.imported.state));accepted(direct1);
    assert.equal(f.world.advance(dt),1);const first=verify(f);assert.deepEqual(first,direct1.state);
    assert.ok(first.angles.get('wire')[0]>4*Math.PI);assert.ok(first.angles.get('catheter')[0]<-6*Math.PI);
    const direct2=advanceCompositeJointTimeStep(first,f.options(first));accepted(direct2);
    // Rendering views cannot replace the richer union state on the next dt.
    for(const body of f.world.bodies){body.x.fill(999);body.orientationW.fill(0);body.jointStateView.positions.fill(-999);body.jointStateView.unwrappedAngles.fill(0);}
    assert.equal(f.world.advance(dt),1);assert.deepEqual(verify(f),direct2.state);
    assert.deepEqual(f.counts,{initializations:1,preparations:2});assert.deepEqual(f.imported.state.toolPositions,before);
    assert.equal(f.world.accumulator,0);assert.equal(f.world.stepCount,2);
});

test('imported World union retains original command and complete material history across budget rejection and retry',()=>{
    const f=fixture(),original=f.world.bodies.map(b=>({x:b.x.slice(),y:b.y.slice(),z:b.z.slice()}));
    const expected=advanceCompositeJointTimeStep(f.imported.state,f.options(f.imported.state));accepted(expected);
    f.adapter.setBudget({directions:0});assert.equal(f.world.advance(dt),0);assert.equal(f.world.lastStepResult.status,'direction-budget');
    assert.equal(f.world.accumulator,dt);assert.equal(f.world.stepCount,0);assert.deepEqual(f.adapter.snapshot().materialVelocities,f.imported.state.materialVelocities);
    f.world.bodies.forEach((b,i)=>{for(const key of ['x','y','z'])assert.deepEqual(b[key],original[i][key]);assert.equal(b.jointStateView,undefined);});
    f.lastInput.loads.forces[0].value[1]=99;f.lastInput.inertia.inertiaEdges[0].tools[0].oldMaterialVelocities[0][0]=99;
    f.adapter.setBudget(null);assert.equal(f.world.advance(0),1);assert.deepEqual(verify(f),expected.state);
    assert.deepEqual(f.counts,{initializations:1,preparations:1});assert.equal(f.world.accumulator,0);
});

import assert from 'node:assert/strict';
import test from 'node:test';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {createCompositeAppToolSource,initializeCompositeAppState,prepareCompositeAppInputs} from '../src/physics/kirchhoffCompositeAppInputs.js';
import {createCompositeJointWorldAdapter} from '../src/physics/kirchhoffCompositeJointWorldAdapter.js';

function fixture() {
    const world=new EndovascularPhysicsWorld(),coordinates=[0,1,2,3],sources=['wire','catheter'].map(toolId=>{
        const wire=toolId==='wire',body=world.createRod(wire?'guidewire':'catheter',4,1);
        coordinates.forEach((x,n)=>{body.setNodePosition(n,x,wire?.15:0,0);body.materialCoordinate[n]=x;});
        body.captureKirchhoffRestConfiguration({captureRestRotation:false});body.nodeRadius.fill(wire?.1:.4);
        return createCompositeAppToolSource({body,toolId,nodeCoordinates:coordinates,materialLabels:coordinates,unwrappedAngles:[0,0,0],referenceWindingTurns:[0,0],massPerMaterialLength:.1,
            profile:{type:wire?'glidewire':'pigtail',tipMaterialCoordinate:80,shaftStiffnessScale:1,tipStiffnessScale:1}});
    });
    world.addContainment(sources[0].body,sources[1].body,{innerRadius:.3,axialFriction:.03,torsionalFriction:.07,openProximal:true,openDistal:false,enforceDistalPortal:false,portalFilletRadius:0,startNode:0,endNode:0,containedLength:1});
    world.addToolContact(sources[0].body,sources[1].body,{friction:.08,openDistalB:false,startSegmentA:2,endSegmentA:2,startSegmentB:0,endSegmentB:0});
    const initial=initializeCompositeAppState({tools:sources,geometry:'native-discrete-rod'}),adapter=createCompositeJointWorldAdapter({worldContainment:true,
        initialize:()=>initial,prepareStep:({state,dt})=>{
            const p=prepareCompositeAppInputs({state,dt,commands:['wire','catheter'].map(toolId=>({toolId,labelShift:0,feedVelocity:0,spinIncrement:0}))});
            return {...p.options,preparedState:p.state};
        }});
    world.wholeStepSystem=adapter;return {world,adapter,sources};
}

test('actual containment and external contacts share one World block with distinct source coefficients, including separated physical edges',()=>{
    const f=fixture();
    for(let i=0;i<2;i++) {
        const r=f.world.stepFixed();assert.equal(r.accepted,true,JSON.stringify({status:r.status,message:r.message,error:r.error}));
        const c=r.diagnostics.certificate;assert.equal(c.contact.converged,true);assert.equal(c.friction.converged,true);
        assert.ok(c.contact.samples.some(s=>s.feature==='external-capsule'));
        const coefficients=new Set(c.friction.samples.map(s=>JSON.stringify(s.mu)));
        assert.deepEqual([...coefficients].sort(),['[0.03,0.07]','[0.08,0.08]']);
        for(const tool of f.sources)assert.ok(tool.body.jointStateView.positions.every(Number.isFinite));
    }
    assert.equal(f.adapter.snapshot().step,2);assert.equal(f.adapter.diagnostics.publications,2);
});

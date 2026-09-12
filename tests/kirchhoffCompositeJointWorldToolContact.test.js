import assert from 'node:assert/strict';
import test from 'node:test';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {createCompositeAppToolSource,initializeCompositeAppState,prepareCompositeAppInputs} from '../src/physics/kirchhoffCompositeAppInputs.js';
import {prepareCompositeJointWorldToolContact as prepare,assertCompositeJointWorldToolContact as verify} from '../src/physics/kirchhoffCompositeJointWorldToolContact.js';
import {advanceCompositeJointTimeStep} from '../src/physics/kirchhoffCompositeJointTimeStep.js';

function fixture({y=.7,offset=.5,unequal=true}={}) {
    const world=new EndovascularPhysicsWorld(),sources=['wire','catheter'].map(toolId=>{
        const wire=toolId==='wire',x=unequal?(wire?[0,5,10,15,20]:[0,4,8,12]):[0,1,2],body=world.createRod(wire?'guidewire':'catheter',x.length,x[1]);
        x.forEach((v,node)=>{body.setNodePosition(node,v+(wire?offset:0),wire?y:0,0);body.materialCoordinate[node]=v;});
        body.captureKirchhoffRestConfiguration({captureRestRotation:false});body.nodeRadius.fill(wire?.1:.4);
        return createCompositeAppToolSource({body,toolId,nodeCoordinates:x,materialLabels:x,unwrappedAngles:x.slice(1).map(()=>0),referenceWindingTurns:x.slice(2).map(()=>0),
            massPerMaterialLength:.1,profile:{type:wire?'glidewire':'pigtail',tipMaterialCoordinate:80,shaftStiffnessScale:1,tipStiffnessScale:1}});
    }),imported=initializeCompositeAppState({tools:sources,geometry:'native-discrete-rod'});
    return {world,state:imported.state,bindings:imported.bindings,sources};
}

test('original external windows expand to owned 5/4 mm union children with parent radii, actual friction and open-distal exclusions',()=>{
    const f=fixture(),tool=f.world.addToolContact(f.sources[0].body,f.sources[1].body,{friction:.08,openDistalB:true,startSegmentA:2,endSegmentA:3,startSegmentB:1,endSegmentB:2}),
        result=prepare(f),wireMap=new Map(f.bindings[0].nodes.map(n=>[n.node,n.jointNode])),catMap=new Map(f.bindings[1].nodes.map(n=>[n.node,n.jointNode]));
    assert.equal(verify({...f,...result}),result.proof);assert.ok(result.contacts.pairs.length>0);assert.ok(result.inactivePairs.some(e=>e.reason==='open-distal'));
    assert.equal(result.contacts.mode,'lumen-coulomb');assert.equal(result.contacts.friction.rateMode,'backward-euler-grid');assert.equal(result.contacts.friction.finiteStepSlipKnown,false);
    for(const p of result.contacts.pairs) {
        assert.equal(p.feature,'external-capsule');assert.ok(p.innerEdge>=wireMap.get(2)&&p.innerEdge<wireMap.get(4));assert.ok(p.outerEdge>=catMap.get(1)&&p.outerEdge<catMap.get(3));
        assert.equal(p.innerRadius,f.sources[0].body.nodeRadius[0]);assert.equal(p.outerRadius,f.sources[1].body.nodeRadius[0]);
        assert.deepEqual(result.contacts.friction.muByPair[p.id],[.08,.08]);if(p.openDistalB)assert.equal(p.outerEdge,catMap.get(3)-1);
    }
    assert.equal(result.proof.includesLegacyPostPass,false);assert.equal(result.proof.continuumClearanceCertified,false);
    tool.endSegmentA=2;assert.throws(()=>verify({...f,...result}),/changed/);assert.ok(prepare(f).contacts.pairs.length<result.contacts.pairs.length);
    tool.enabled=false;assert.equal(prepare(f).contacts,'none');
});

test('the original connected lumen branch retains ownership at the native distal mouth, without exempting a remote outside branch',()=>{
    const f=fixture({y:.15,offset:0}),wire=f.sources[0].body,catheter=f.sources[1].body;
    f.world.addContainment(wire,catheter,{innerRadius:.3,axialFriction:0,torsionalFriction:0,openProximal:true,openDistal:true,enforceDistalPortal:true,
        portalFilletRadius:.15,startNode:0,endNode:2,containedLength:12});
    f.world.addToolContact(wire,catheter,{friction:0,openDistalB:true,startSegmentA:2,endSegmentA:3,startSegmentB:2,endSegmentB:2});
    const result=prepare(f);assert.ok(result.exclusions.some(e=>e.reason==='lumen-ownership'));assert.ok(result.inactivePairs.some(e=>e.reason==='open-distal'));
    const outside=fixture({y:.7,offset:0});outside.world.addContainment(outside.sources[0].body,outside.sources[1].body,{innerRadius:.3,openDistal:true,enforceDistalPortal:true,containedLength:12,endNode:2});
    outside.world.addToolContact(outside.sources[0].body,outside.sources[1].body,{friction:0,openDistalB:true,startSegmentA:2,endSegmentA:3,startSegmentB:2,endSegmentB:2});
    assert.ok(!prepare(outside).exclusions.some(e=>e.reason==='lumen-ownership'));
});

test('all source-window gaps remain represented and every actual constraint keeps its own Coulomb coefficient',()=>{
    const f=fixture({y:20,offset:0});
    for(const friction of [.03,.12])f.world.addToolContact(f.sources[0].body,f.sources[1].body,{friction,startSegmentA:0,endSegmentA:0,startSegmentB:0,endSegmentB:0});
    const result=prepare(f);assert.ok(result.contacts.pairs.length>2);const coefficients=new Set(Object.values(result.contacts.friction.muByPair).map(v=>v[0]));
    assert.deepEqual([...coefficients],[.03,.12]);assert.equal(result.exclusions.length,0);
    result.contacts.pairs[0].outerRadius+=.1;assert.throws(()=>verify({...f,...result}),/changed/);
});

test('a small actual World external source runs inside the native shared timestep',()=>{
    const f=fixture({y:.55,offset:.2,unequal:false});f.world.addToolContact(f.sources[0].body,f.sources[1].body,{friction:0,startSegmentA:0,endSegmentA:0,startSegmentB:0,endSegmentB:0});
    const source=prepare(f),prepared=prepareCompositeAppInputs({state:f.state,dt:1/120,commands:['wire','catheter'].map(toolId=>({toolId,labelShift:0,feedVelocity:0,spinIncrement:0}))}),
        result=advanceCompositeJointTimeStep(prepared.state,{...prepared.options,contacts:source.contacts,wall:'none'});
    assert.equal(result.accepted,true,JSON.stringify({status:result.status,error:result.error}));assert.ok(result.state.toolPositions.get('wire').every(p=>p.every(Number.isFinite)));
});

test('actual external source friction carries a loaded reaction with independent material feed and handle spin',()=>{
    const f=fixture({y:Math.fround(.1)+Math.fround(.4),offset:.2,unequal:false}),dt=1/120;
    f.world.addToolContact(f.sources[0].body,f.sources[1].body,{friction:.08,startSegmentA:0,endSegmentA:0,startSegmentB:0,endSegmentB:0});
    const reservoir=({toolId,s})=>({id:toolId,sStart:s<0?-1:2,sEnd:s<0?0:3,velocities:[[0,0,0],[0,0,0]],interpretation:'physical-material-velocity'}),
        prepared=prepareCompositeAppInputs({state:f.state,dt,reservoir,commands:[{toolId:'wire',labelShift:-.04*dt,feedVelocity:.04,spinIncrement:.002},
        {toolId:'catheter',labelShift:.03*dt,feedVelocity:-.03,spinIncrement:-.001}]}),source=prepare({...f,state:prepared.state}),
        result=advanceCompositeJointTimeStep(prepared.state,{...prepared.options,contacts:source.contacts,wall:'none',
            boundaries:{...prepared.options.boundaries,positions:[]},loads:{forces:[{toolId:'wire',node:0,value:[0,-.01,0]},{toolId:'catheter',node:0,value:[0,.01,0]}]}});
    assert.equal(result.accepted,true,JSON.stringify({status:result.status,error:result.error,diagnostics:result.diagnostics}));
    assert.ok(result.state.lumenContactState.normalForces.some(v=>v>1e-5));assert.ok(result.state.lumenFrictionState.tractions.some(v=>Math.abs(v)>1e-7));
    const proof=result.diagnostics.certificate.friction;assert.equal(proof.converged,true);assert.equal(proof.finiteStepSlipKnown,false);assert.equal(proof.rateMode,'backward-euler-grid');
    assert.ok(proof.samples.some(s=>s.slip.some(v=>Math.abs(v)>1e-5)));assert.deepEqual(proof.samples[0].mu,[.08,.08]);
    const total=[0,0,0];for(const forces of result.contactForces.values())for(const f of forces)f.forEach((v,k)=>total[k]+=v);
    assert.ok(total.every(v=>Math.abs(v)<1e-10));for(const b of result.balances.values())assert.ok(b.residual.every(v=>Math.abs(v)<1e-7));
});

import assert from 'node:assert/strict';
import test from 'node:test';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeJointTimeStepState,advanceCompositeJointTimeStep} from '../src/physics/kirchhoffCompositeJointTimeStep.js';
import {prepareCompositeJointWorldContainment as prepare,assertCompositeJointWorldContainment as verify} from '../src/physics/kirchhoffCompositeJointWorldContainment.js';
import {evaluateKirchhoffLumenSegmentContact} from '../src/physics/kirchhoffLumenContact.js';
import {createCompositeLumenSideGeometryWorkspace,differentiateCompositeLumenSideContact} from '../src/physics/kirchhoffCompositeLumenSideGeometry.js';
import {createCompositeLumenTipGeometryWorkspace,differentiateCompositeLumenTipContact} from '../src/physics/kirchhoffCompositeLumenTipGeometry.js';
import {createCompositeAppToolSource,initializeCompositeAppState,prepareCompositeAppInputs} from '../src/physics/kirchhoffCompositeAppInputs.js';

import {createCompositeJointLumenRows} from '../src/physics/kirchhoffCompositeJointLumenRows.js';

const ids=['wire','catheter'],close=(a,b,t=2e-7)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}`);
function fixture() {
    const world=new EndovascularPhysicsWorld(),wire=world.createRod('guidewire',3,1),catheter=world.createRod('catheter',3,1),coordinates=[0,1,2],
        positions=coordinates.map(x=>[x,0,0]),ownWire=coordinates.map(x=>[x,.15,0]),layout=createCompositeChainLayout([ids,ids]),
        modes=coordinates.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]})),
        tools=ids.map(id=>({id,dsDx:1,reference:captureCompositeReferenceFrames(id==='wire'?ownWire:positions),referenceTwists:[0],material:compileCompositeMaterial({EI1:2,EI2:2,GJ:1})})),
        state=createCompositeJointTimeStepState({layout,coordinates,positions,tools,modes,relative:modes.flatMap(()=>[0,.15,0]),
            angles:new Map(ids.map(id=>[id,[0,0]])),restLengths:new Map(ids.map(id=>[id,[1,1]])),elasticityGeometry:'native-discrete-rod',materialCoordinate:'reference-arclength'}),
        bindings=[{toolId:'wire',body:wire,nodes:coordinates.map((_,node)=>({node,jointNode:node}))},{toolId:'catheter',body:catheter,nodes:coordinates.map((_,node)=>({node,jointNode:node}))}];
    wire.nodeRadius.fill(.1);catheter.nodeRadius.fill(.4);
    for(const b of bindings)coordinates.forEach((_,j)=>{const p=state.toolPositions.get(b.toolId)[j];b.body.x[j]=p[0];b.body.y[j]=p[1];b.body.z[j]=p[2];});
    const containment=world.addContainment(wire,catheter,{innerRadius:.3,axialFriction:0,torsionalFriction:0,openProximal:true,openDistal:true,searchWindow:2,
        portalFilletRadius:.15,enforceDistalPortal:true,startNode:0,endNode:2,outerStartNode:0,containedLength:2});
    const inertia={previousPositions:structuredClone(state.toolPositions),inertiaEdges:coordinates.slice(1).map((_,edge)=>({tools:ids.map(id=>({id,massPerMaterialLength:.1,
        materialMap:{sStart:20+edge,dsDx:1,dsDt:0},oldMaterialVelocities:[[0,0,0],[0,0,0]]}))}))};
    return {world,state,bindings,containment,inertia};
}

test('actual native World containment preserves source radius/window/open tip and runs both tools through the common timestep without a postpass',()=>{
    const f=fixture(),prepared=prepare(f),before=structuredClone(f.state);
    assert.equal(verify({...f,...prepared}),prepared.proof);assert.equal(prepared.proof.includesLegacyPostPass,false);assert.equal(prepared.proof.continuumClearanceCertified,false);
    assert.ok(prepared.contacts.pairs.some(p=>p.feature==='side'));assert.ok(prepared.contacts.pairs.some(p=>p.feature==='distal-fillet'));assert.ok(prepared.contacts.pairs.some(p=>p.feature==='distal-rim'));
    assert.ok(prepared.contacts.pairs.every(p=>p.lumenRadius===.3&&p.innerRadius===f.bindings[0].body.nodeRadius[0]&&p.portalFilletRadius===.15&&p.endpointDerivative==='clamped-one-sided'));
    const input={dt:1/120,torsionMode:'quasi-static',wall:'none',contacts:prepared.contacts,inertia:f.inertia,boundaries:{positions:[],spins:ids.map(toolId=>({toolId,edge:0,value:0}))}},
        result=advanceCompositeJointTimeStep(f.state,input);
    assert.equal(result.accepted,true,JSON.stringify({status:result.status,error:result.error}));assert.deepEqual(f.state,before);
    for(const p of result.state.toolPositions.values())assert.ok(p.every(v=>v.every(Number.isFinite)));
    for(const b of result.balances.values())b.residual.forEach(v=>close(v,0,1e-10));
    f.containment.containedLength=.2;assert.throws(()=>verify({...f,...prepared}),/changed/);
    const shorter=prepare(f);assert.ok(shorter.samples.length<prepared.samples.length);assert.ok(shorter.exclusions.some(e=>e.reason==='outside-contained-material-span'));
    f.containment.axialFriction=.03;f.containment.torsionalFriction=.07;
    const friction=prepare(f);assert.equal(friction.contacts.mode,'lumen-coulomb');assert.deepEqual(friction.contacts.friction.mu,[.03,.07]);
    assert.equal(friction.contacts.friction.rateMode,'backward-euler-grid');assert.equal(friction.contacts.friction.finiteStepSlipKnown,false);
    assert.equal(verify({...f,...friction}),friction.proof);
    const other=f.world.addContainment(f.bindings[0].body,f.bindings[1].body,{axialFriction:.08,torsionalFriction:.07});
    assert.throws(()=>prepare(f),e=>e.code==='joint-world-containment-adapter-required'&&e.details.required==='per-pair-native-Coulomb-coefficients');other.enabled=false;
    f.containment.enabled=false;assert.equal(prepare(f).contacts,'none');
});

test('native endpoint projection selects the clamped generalized derivative at both ends and keeps strict default behavior',()=>{
    for(const end of [0,1]) {
        const input={innerStart:[end,.2,0],innerEnd:[end+.3,.2,0],outerStart:[0,0,0],outerEnd:[1,0,0],innerMaterialSegmentId:'wire',outerMaterialSegmentId:'catheter',
            lumenRadius:.4,innerRadius:.1,quadrature:[0],openDistal:false,portalFilletRadius:0},raw=evaluateKirchhoffLumenSegmentContact(input),scratch=createCompositeLumenSideGeometryWorkspace();
        assert.equal(differentiateCompositeLumenSideContact({input,contact:raw.side},scratch).supported,false);
        const full=differentiateCompositeLumenSideContact({input:{...input,endpointDerivative:'clamped-one-sided'},contact:raw.side},scratch);
        assert.equal(full.supported,true,full.reason);assert.ok(full.outerTGradient.every(v=>v===0));
        const H=full.normalDerivative.slice(),B=full.normalForceColumn.slice(),h=1e-6;
        // Move radially: projection stays exactly on the selected endpoint,
        // so this checks its generalized branch without crossing features.
        const plus=structuredClone(input),minus=structuredClone(input);plus.innerStart[1]+=h;minus.innerStart[1]-=h;
        const a=evaluateKirchhoffLumenSegmentContact(plus).side,b=evaluateKirchhoffLumenSegmentContact(minus).side;
        close((a.gap-b.gap)/(2*h),B[1],1e-9);
        const da=differentiateCompositeLumenSideContact({input:{...plus,endpointDerivative:'clamped-one-sided'},contact:a},createCompositeLumenSideGeometryWorkspace()),
            db=differentiateCompositeLumenSideContact({input:{...minus,endpointDerivative:'clamped-one-sided'},contact:b},createCompositeLumenSideGeometryWorkspace());
        for(let row=0;row<12;row++)close((da.normalForceColumn[row]-db.normalForceColumn[row])/(2*h),H[12*row+1],1e-9);
    }
});

test('source containment transfers a loaded normal reaction between both native tools in the common solve',()=>{
    const f=fixture(),clearance=f.containment.innerRadius-f.bindings[0].body.nodeRadius[0];
    for(let node=0;node<3;node++){f.state.relative[3*node+1]=clearance;f.state.toolPositions.get('wire')[node][1]=clearance;}
    f.inertia.previousPositions=structuredClone(f.state.toolPositions);
    const source=prepare(f),result=advanceCompositeJointTimeStep(f.state,{dt:1/120,torsionMode:'quasi-static',wall:'none',contacts:source.contacts,inertia:f.inertia,
        boundaries:{positions:[],spins:ids.map(toolId=>({toolId,edge:0,value:0}))},loads:{forces:[{toolId:'wire',node:1,value:[0,.01,0]},{toolId:'catheter',node:1,value:[0,-.01,0]}]}});
    assert.equal(result.accepted,true,JSON.stringify({status:result.status,error:result.error}));
    assert.ok(result.state.lumenContactState.normalForces.some(v=>v>0));
    const total=[0,0,0];for(const [id,forces] of result.contactForces)for(const f of forces)f.forEach((v,k)=>total[k]+=v);
    total.forEach(v=>close(v,0,1e-10));for(const b of result.balances.values())b.residual.forEach(v=>close(v,0,1e-7));
});

test('actual positive source friction solves loaded native side contacts with independent tool feed and spin',()=>{
    const f=fixture(),clearance=f.containment.innerRadius-f.bindings[0].body.nodeRadius[0],dt=1/120,slope=-.01,stretch=Math.hypot(1,slope);
    Object.assign(f.containment,{axialFriction:.03,torsionalFriction:.07,openDistal:false,endNode:0,containedLength:1});
    // One initial touching pressure sample and separated remaining samples
    // avoid an indeterminate four-load distribution on exactly parallel rods.
    for(let node=0;node<3;node++){const y=clearance+slope*(node-.125);f.state.relative[3*node+1]=y;f.state.toolPositions.get('wire')[node][1]=y;}
    const wire=f.state.tools.find(t=>t.id==='wire');wire.reference=captureCompositeReferenceFrames(f.state.toolPositions.get('wire'));wire.dsDx=stretch;f.state.restLengths.get('wire').fill(stretch);
    f.inertia.previousPositions=structuredClone(f.state.toolPositions);
    for(const [index,edge] of f.inertia.inertiaEdges.entries())for(const t of edge.tools){const feed=t.id==='wire'?.04:-.03,s=t.id==='wire'?stretch:1;t.materialMap.dsDt=feed;t.materialMap.dsDx=s;t.materialMap.sStart=20+index*s;
        const velocity=[-feed/s,t.id==='wire'?-feed*slope/s:0,0];t.oldMaterialVelocities=[velocity.slice(),velocity.slice()];}
    const source=prepare(f),before=structuredClone(f.state),result=advanceCompositeJointTimeStep(f.state,{dt,torsionMode:'quasi-static',wall:'none',contacts:source.contacts,inertia:f.inertia,
        boundaries:{positions:[],spins:[{toolId:'wire',edge:0,value:.002},{toolId:'catheter',edge:0,value:-.001}]},
        loads:{forces:[{toolId:'wire',node:0,value:[0,.01,0]},{toolId:'catheter',node:0,value:[0,-.01,0]}]}});
    assert.equal(result.accepted,true,JSON.stringify({status:result.status,error:result.error,diagnostics:result.diagnostics}));assert.deepEqual(f.state,before);
    assert.ok(result.state.lumenContactState.normalForces.some(v=>v>0));assert.ok(result.state.lumenFrictionState.tractions.some(v=>Math.abs(v)>1e-7));
    const proof=result.diagnostics.certificate.friction;assert.equal(proof.converged,true);assert.equal(proof.rateMode,'backward-euler-grid');assert.equal(proof.finiteStepSlipKnown,false);
    assert.ok(proof.samples.some(s=>s.slip.some(v=>Math.abs(v)>1e-5)));
    const total=[0,0,0];for(const forces of result.contactForces.values())for(const f of forces)f.forEach((v,k)=>total[k]+=v);
    total.forEach(v=>close(v,0,1e-10));for(const b of result.balances.values())b.residual.forEach(v=>close(v,0,1e-7));
});

test('an exact distal crossing at a native endpoint has an explicit clamped rim member with finite physical reactions',()=>{
    const input={innerStart:[0,.2,0],innerEnd:[1,.2,0],outerStart:[0,0,0],outerEnd:[1,0,0],innerMaterialSegmentId:'wire',outerMaterialSegmentId:'catheter',
        lumenRadius:.4,innerRadius:.1,quadrature:[.5],openDistal:true,portalFilletRadius:.15,activationDistance:100},raw=evaluateKirchhoffLumenSegmentContact(input),
        scratch=createCompositeLumenTipGeometryWorkspace();
    assert.equal(raw.portal.crosses,true);assert.equal(raw.portal.innerT,1);
    assert.equal(differentiateCompositeLumenTipContact({input,contact:raw.portal.contact},scratch).supported,false);
    const full=differentiateCompositeLumenTipContact({input:{...input,endpointDerivative:'clamped-one-sided'},contact:raw.portal.contact},scratch);
    assert.equal(full.supported,true,full.reason);assert.ok(full.normalForceColumn.every(Number.isFinite));assert.ok(full.normalDerivative.every(Number.isFinite));assert.ok(full.innerTGradient.every(v=>v===0));
});

test('actual 5 mm wire and 4 mm catheter app grids preserve original material sample resolution through union children',()=>{
    const world=new EndovascularPhysicsWorld(),sources=ids.map(toolId=>{
        const wire=toolId==='wire',x=wire?[0,5,10,15,20]:[0,4,8,12],body=world.createRod(wire?'guidewire':'catheter',x.length,x[1]);
        x.forEach((v,node)=>{body.setNodePosition(node,v,wire?.15:0,0);body.materialCoordinate[node]=v;});body.captureKirchhoffRestConfiguration({captureRestRotation:false});body.nodeRadius.fill(wire?.1:.4);
        return createCompositeAppToolSource({body,toolId,nodeCoordinates:x,materialLabels:x,unwrappedAngles:x.slice(1).map(()=>0),referenceWindingTurns:x.slice(2).map(()=>0),
            massPerMaterialLength:.1,profile:{type:wire?'glidewire':'pigtail',tipMaterialCoordinate:80,shaftStiffnessScale:1,tipStiffnessScale:1}});
    }),imported=initializeCompositeAppState({tools:sources,geometry:'native-discrete-rod'}),args={world,state:imported.state,bindings:imported.bindings};
    world.addContainment(sources[0].body,sources[1].body,{innerRadius:.3,axialFriction:0,torsionalFriction:0,openProximal:true,openDistal:true,searchWindow:2,
        portalFilletRadius:.15,enforceDistalPortal:true,startNode:0,endNode:2,outerStartNode:0,containedLength:12});
    assert.ok(imported.bindings.some(b=>b.nodes.some((n,j)=>j&&n.jointNode>b.nodes[j-1].jointNode+1)));
    const prepared=prepare(args);assert.equal(verify({...args,...prepared}),prepared.proof);
    const inherited=prepared.samples.map(s=>5*(s.sourceInnerEdge+s.sourceFraction));
    assert.deepEqual(inherited,[.625,1.875,3.125,4.375,5.625,6.875,8.125,9.375,10.625,11.875]);
    assert.equal(new Set(inherited).size,inherited.length);assert.ok(prepared.contacts.pairs.every(p=>p.innerRadius===sources[0].body.nodeRadius[0]));
    assert.ok(prepared.contacts.pairs.some(p=>p.innerEdge!==p.outerEdge||typeof p.innerMaterialSegmentId==='string'));
    const options=prepareCompositeAppInputs({state:imported.state,dt:1/120,commands:ids.map(toolId=>({toolId,labelShift:0,feedVelocity:0,spinIncrement:0}))}),
        result=advanceCompositeJointTimeStep(options.state,{...options.options,contacts:prepared.contacts,wall:'none'});
    assert.equal(result.accepted,true,JSON.stringify({status:result.status,error:result.error}));
});


test('only supplementary aperture side samples become absent with their original crossing; primary samples and loaded reactions stay enforced',()=>{
    const f=fixture(),source=prepare(f),contacts={...source.contacts,mode:'lumen-normal',friction:'none'},tolerances={force:1e-7,linearConstraint:1e-10,lumenGap:1e-8,lumenNcp:1e-8,lumenWork:1e-9},
        make=c=>createCompositeJointLumenRows({layout:f.state.layout,coordinates:f.state.coordinates,modes:f.state.modes,contacts:c,tolerances,preserveSampleReactions:true}),
        manager=make(contacts),points=structuredClone(f.state.toolPositions),refresh=m=>m.refresh({toolPositions:points,commonResidual:new Float64Array(f.state.layout.dofCount),relativeResidual:new Float64Array(f.state.relative.length),order:'full',consumeQuery(){}});
    manager.prepareGauge({toolPositions:f.state.toolPositions,consumeQuery(){}});
    const auxiliary=manager.samples.find(s=>s.conditionalPortal&&s.s===0&&s.input.outerSegmentIndex===1);
    assert.ok(auxiliary,'Actual source records the supplementary original tip sample');
    points.get('wire')[1][0]=.9998;points.get('wire')[2][0]=1.99;
    const absent=refresh(manager);assert.equal(absent.converged,true);
    const proof=absent.samples.find(s=>s.sampleId===auxiliary.sampleId);
    assert.equal(proof.applicable,false);assert.equal(proof.gap,null);assert.equal(proof.Fn,0);assert.equal(proof.eliminated,true);
    assert.ok(manager.samples.some(s=>!s.conditionalPortal&&s.feature==='side'&&s.applicable));
    // Changing the declaration into a primary side row restores its strict
    // support guard. No primary material sample is silently discarded.
    const primary=make({...contacts,pairs:contacts.pairs.map(({conditionalPortalSamples,...p})=>p)});
    primary.prepareGauge({toolPositions:f.state.toolPositions,consumeQuery(){}});
    assert.throws(()=>refresh(primary),/left its original side/);
    points.get('wire')[1][0]=1;points.get('wire')[2][0]=2;refresh(manager);
    assert.equal(auxiliary.applicable,true);assert.equal(auxiliary.eliminated,false);
    manager.normalForces[auxiliary.index]=.01;points.get('wire')[1][0]=.9998;points.get('wire')[2][0]=1.99;
    assert.throws(()=>refresh(manager),/left its original side/,'A nonzero reaction cannot disappear at the aperture');
});

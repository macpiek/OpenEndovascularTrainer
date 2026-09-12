import test from 'node:test';
import assert from 'node:assert/strict';
import {EndovascularPhysicsWorld,DEFAULT_TOOL_PROFILES} from '../src/physics/endovascularPhysicsWorld.js';
import {applyKirchhoffMaterialProfile} from '../src/physics/applyKirchhoffMaterialProfile.js';
import {kirchhoffMaterialProfile} from '../src/physics/kirchhoffMaterialProfile.js';
import {createCompositeAppToolSource,initializeCompositeAppState,prepareCompositeAppInputs,alignCompositeAppInitialMaterialFrames} from '../src/physics/kirchhoffCompositeAppInputs.js';
import {advanceCompositeJointTimeStep} from '../src/physics/kirchhoffCompositeJointTimeStep.js';
import {createProximalMaterialFrame} from '../src/physics/kirchhoffOrientationBoundary.js';
import {materialFrameDirectors} from '../src/physics/discreteKirchhoffRod.js';

function fixture() {
    const world=new EndovascularPhysicsWorld(),tools=[];
    for(const [toolId,id,type,offset,tipLabel] of [['wire','guidewire','glidewire',.03,0],['catheter','catheter','pigtail',0,80]]) {
        const body=world.createRod(id,4,4,DEFAULT_TOOL_PROFILES[id]),x=[-12,-8,-4,0];
        x.forEach((v,i)=>{body.setNodePosition(i,v,offset,0);body.materialCoordinate[i]=v;});
        body.captureKirchhoffRestConfiguration({captureRestRotation:false});
        applyKirchhoffMaterialProfile(body,type,{tipCoordinate:tipLabel,shaftStiffnessScale:1.5,tipStiffnessScale:.7});
        tools.push(createCompositeAppToolSource({body,toolId,nodeCoordinates:x,materialLabels:x,unwrappedAngles:[0,0,0],
            referenceWindingTurns:[0,0],massPerMaterialLength:body.mass/4,
            profile:{type,tipMaterialCoordinate:tipLabel,shaftStiffnessScale:1.5,tipStiffnessScale:.7}}));
    }
    return {world,tools};
}
const zeroCommands = state=>state.tools.map(t=>({toolId:t.id,labelShift:0,feedVelocity:0,spinIncrement:0}));

test('native first fractional insertion keeps the physical tip history at its rounded chart endpoint',()=>{
    const world=new EndovascularPhysicsWorld(),progress=.13333333333333333,x=[-10,-5,0,progress],tools=[];
    for(const [toolId,type] of [['wire','glidewire'],['catheter','berenstein']]) {
        const body=world.createRod(toolId==='wire'?'guidewire':'catheter',x.length,5);
        x.forEach((v,i)=>body.setNodePosition(i,v,0,0));
        body.captureKirchhoffRestConfiguration({captureRestRotation:false});
        x.slice(0,-1).forEach((v,i)=>{body.restLength[i]=x[i+1]-v;});
        tools.push(createCompositeAppToolSource({body,toolId,nodeCoordinates:x,materialLabels:x.map(v=>v-progress),
            unwrappedAngles:[0,0,0],referenceWindingTurns:[0,0],massPerMaterialLength:1,
            profile:{type,tipMaterialCoordinate:0,shaftStiffnessScale:1,tipStiffnessScale:1}}));
    }
    const {state}=initializeCompositeAppState({tools,geometry:'native-discrete-rod'}),before=structuredClone(state.materialVelocities),
        prepared=prepareCompositeAppInputs({state,dt:1/120,commands:zeroCommands(state)});
    assert.deepEqual(state.materialVelocities,before);
    for(const own of prepared.options.inertia.inertiaEdges.at(-1).tools) {
        assert.equal(own.oldVelocityPieces.length,1);
        assert.deepEqual(own.oldVelocityPieces[0].oldMaterialVelocities,[[0,0,0],[0,0,0]]);
    }
});

test('actual app bodies import once into newest continuous model with owned profiles, material velocities and independent geometry',()=>{
    const f=fixture(),before=f.tools.map(t=>({x:t.body.x.slice(),q:t.body.orientationX.slice()})),r=initializeCompositeAppState(f),s=r.state;
    assert.equal(s.elasticityGeometry,'continuous-material-frame');assert.equal(s.lengthGeometry,'continuous-arclength');
    assert.ok(s.inertiaGeometryByTool instanceof Map);assert.equal(s.modes.length,4);
    for(const source of f.tools) {
        const id=source.toolId,tool=s.tools.find(t=>t.id===id);
        assert.deepEqual(s.toolPositions.get(id),Array.from(source.body.x,(x,i)=>[x,source.body.y[i],source.body.z[i]]));
        assert.deepEqual(Array.from(s.angles.get(id)),source.angles);
        assert.equal(tool.massPerMaterialLength,source.body.mass/4);
        const at=tool.materialAt({coordinate:-4}),p=kirchhoffMaterialProfile(source.appMaterialProfile.type).sample(source.appMaterialProfile.tipMaterialCoordinate+4),
            scale=id==='wire'?.7:1.5;
        assert.ok(Math.abs(at.stiffness[0]-p.EI1*scale)<1e-9);
        assert.deepEqual(Array.from(at.intrinsic),[p.kappa01,p.kappa02,p.tau0]);
        assert.equal(s.materialVelocities[0].tools.find(t=>t.id===id).sStart,-12);
    }
    assert.deepEqual(f.tools.map(t=>({x:t.body.x.slice(),q:t.body.orientationX.slice()})),before);
    f.tools[0].body.velocityX[0]=99;assert.equal(s.materialVelocities[0].tools[0].velocities[0][0],0);
});

test('preparation retains accepted polynomial velocity history and unwrapped phases across actual newest whole steps',()=>{
    const s=initializeCompositeAppState(fixture()).state,dt=1/120,first=prepareCompositeAppInputs({state:s,dt,commands:zeroCommands(s)}),
        r=advanceCompositeJointTimeStep(first.state,first.options);
    assert.equal(r.accepted,true,JSON.stringify({status:r.status,message:r.error,diagnostics:r.diagnostics}));
    assert.equal(r.state.step,1);
    const second=prepareCompositeAppInputs({state:r.state,dt,commands:zeroCommands(r.state)});
    assert.equal(second.options.inertia.inertiaEdges[0].tools[0].oldVelocityPieces[0].interpretation,'quintic-bernstein-material-velocity');
    assert.equal(second.options.inertia.inertiaEdges[0].tools[0].oldVelocityPieces[0].bernsteinVelocities.length,6);
    const next=advanceCompositeJointTimeStep(second.state,second.options);assert.equal(next.accepted,true,JSON.stringify(next.diagnostics));
    assert.equal(next.state.step,2);assert.equal(next.state.time,2*dt);
    r.state.angles.get('wire')[0]=8*Math.PI+.3;
    const commanded=prepareCompositeAppInputs({state:r.state,dt,commands:zeroCommands(r.state).map(c=>({...c,spinIncrement:c.toolId==='wire'?.02:-.03}))});
    assert.equal(commanded.options.boundaries.spins[0].value,8*Math.PI+.32);
    assert.equal(commanded.options.boundaries.spins[1].value,-.03);
});

test('independent feed uses exact own current labels, explicit inlet history and immutable profile refresh',()=>{
    const s=initializeCompositeAppState(fixture()).state,before=s.tools.map(t=>t.appMaterialProfile.materialOrigin),dt=1/120,
        commands=zeroCommands(s).map(c=>({...c,labelShift:c.toolId==='wire'?-.01:-.02,feedVelocity:c.toolId==='wire'?1.2:2.4})),
        reservoir=({toolId})=>({id:toolId,sStart:-13,sEnd:-12,velocities:[[1,2,3],[1,2,3]],interpretation:'physical-material-velocity'});
    assert.throws(()=>prepareCompositeAppInputs({state:s,dt,commands}),error=>error.code==='missing-material-history');
    const result=prepareCompositeAppInputs({state:s,dt,commands,reservoir});
    assert.deepEqual(s.tools.map(t=>t.appMaterialProfile.materialOrigin),before);
    for(const t of result.options.inertia.inertiaEdges[0].tools) {
        const command=commands.find(c=>c.toolId===t.id);
        assert.equal(t.materialMap.sStart,-12+command.labelShift);assert.equal(t.materialMap.dsDt,-command.feedVelocity);
        assert.equal(t.oldVelocityPieces.length,2);assert.deepEqual(t.oldVelocityPieces[0].oldMaterialVelocities,[[1,2,3],[1,2,3]]);
        const material=result.state.tools.find(own=>own.id===t.id);assert.equal(material.appMaterialProfile.materialOrigin,-12+command.labelShift);
    }
    assert.ok(result.history.requiredCuts.length>0);
});

test('missing lift, unknown source type and implicit merged tool commands reject rather than invent state',()=>{
    const f=fixture(),source=f.tools[0];
    const args={body:source.body,toolId:'wire',nodeCoordinates:source.nodeCoordinates,materialLabels:source.materialLabels,
        unwrappedAngles:[0,0,0],referenceWindingTurns:[0,0],massPerMaterialLength:.25,profile:source.appMaterialProfile};
    assert.throws(()=>createCompositeAppToolSource({...args,unwrappedAngles:undefined}),/Known unwrapped/);
    assert.throws(()=>createCompositeAppToolSource({...args,referenceWindingTurns:[.5,0]}),/explicit integers/);
    assert.throws(()=>createCompositeAppToolSource({...args,profile:{...args.profile,type:'unknown'}}),/supported/);
    const s=initializeCompositeAppState(f).state;
    assert.throws(()=>prepareCompositeAppInputs({state:s,dt:1/120,commands:zeroCommands(s).slice(0,1)}),/every physical tool/);
});

test('fresh frame alignment repairs actual non-axis Float32 source tangents once while preserving roll, rest and explicit winding',()=>{
    const world=new EndovascularPhysicsWorld(),body=world.createRod('guidewire',3,4,DEFAULT_TOOL_PROFILES.guidewire),
        raw=[.7,.4,.5],norm=Math.hypot(...raw),direction=raw.map(v=>v/norm),base=[1024.14,-2001.25,300.42],x=[-8,-4,0],
        q=createProximalMaterialFrame({x:direction[0],y:direction[1],z:direction[2]},.37,{x:0,y:0,z:1});
    x.forEach((coordinate,i)=>{body.setNodePosition(i,...direction.map((v,k)=>base[k]+coordinate*v));body.materialCoordinate[i]=coordinate;});
    for(let e=0;e<body.segmentCount;e++)for(const k of ['x','y','z','w']) {
        body[`orientation${k.toUpperCase()}`][e]=q[k];body[`previousOrientation${k.toUpperCase()}`][e]=q[k];
    }
    applyKirchhoffMaterialProfile(body,'glidewire',{tipCoordinate:0});
    const before=Object.fromEntries(['x','y','z','restLength','restRotation1','restRotation2','restRotation3',
        'kirchhoffBendCompliance1','kirchhoffBendCompliance2','kirchhoffTwistCompliance','velocityX','velocityY','velocityZ'].map(k=>[k,body[k].slice()]));
    const source=()=>createCompositeAppToolSource({body,toolId:'wire',nodeCoordinates:x,materialLabels:x,unwrappedAngles:[8*Math.PI+.37,8*Math.PI+.37],
        referenceWindingTurns:[0],massPerMaterialLength:body.mass/4,profile:{type:'glidewire',tipMaterialCoordinate:0}});
    assert.throws(()=>initializeCompositeAppState({tools:[source()]}),error=>error.details?.reason==='explicit-reference-does-not-belong-to-source-edge');
    const epoch={},evidence=alignCompositeAppInitialMaterialFrames({body,epoch});
    assert.ok(evidence.maximumCorrection>1e-8&&evidence.maximumCorrection<1e-3);
    assert.ok(evidence.maximumAlignedTangentError<1e-14);assert.equal(evidence.restConfigurationChanged,false);
    const imported=initializeCompositeAppState({tools:[source()]});
    assert.deepEqual(Array.from(imported.state.angles.get('wire')),[8*Math.PI+.37,8*Math.PI+.37]);
    for(const [key,values] of Object.entries(before))assert.deepEqual(body[key],values);
    const original=materialFrameDirectors(q,{}),a=['x','y','z'].map(k=>original.d3[k]),d=['x','y','z'].map(k=>original.d1[k]),
        cross=(u,v)=>[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]],dot=(u,v)=>u.reduce((s,x,k)=>s+x*v[k],0);
    for(let e=0;e<body.segmentCount;e++) {
        const now=materialFrameDirectors(Object.fromEntries(['x','y','z','w'].map(k=>[k,body[`orientation${k.toUpperCase()}`][e]])),{}),b=['x','y','z'].map(k=>now.d3[k]),
            axis=cross(a,b),first=cross(axis,d),second=cross(axis,first),carried=d.map((v,k)=>v+first[k]+second[k]/(1+dot(a,b))),d1=['x','y','z'].map(k=>now.d1[k]);
        assert.ok(Math.hypot(...d1.map((v,k)=>v-carried[k]))<1e-14,'Only tangent swing, no new material roll');
        for(const k of ['X','Y','Z','W'])assert.equal(body[`orientation${k}`][e],body[`previousOrientation${k}`][e]);
    }
    assert.equal(alignCompositeAppInitialMaterialFrames({body,epoch}).reused,true);
    body.x[0]+=.01;assert.throws(()=>alignCompositeAppInitialMaterialFrames({body,epoch}),/changed after/);body.x.set(before.x);
    body.angularVelocityX[0]=1;assert.throws(()=>alignCompositeAppInitialMaterialFrames({body,epoch:{}}),/zero angular history/);body.angularVelocityX[0]=0;
    Object.defineProperty(body,'jointStateView',{value:{},configurable:true});
    assert.throws(()=>alignCompositeAppInitialMaterialFrames({body,epoch:{}}),/cannot be reinitialized/);
});

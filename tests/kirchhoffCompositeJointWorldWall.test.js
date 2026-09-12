import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createBishopFrame} from '../src/physics/discreteKirchhoffRod.js';
import {importCompositeJointWorld} from '../src/physics/kirchhoffCompositeJointWorldImport.js';
import {createCompositeJointWorldAdapter} from '../src/physics/kirchhoffCompositeJointWorldAdapter.js';
import {prepareCompositeJointWorldWall,assertCompositeJointWorldWall} from '../src/physics/kirchhoffCompositeJointWorldWall.js';
import {advanceCompositeJointTimeStep} from '../src/physics/kirchhoffCompositeJointTimeStep.js';
import {createContactResult,VesselContactField} from '../src/physics/collision/vesselContactField.js';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';

const dt=1/120,obj=v=>({x:v[0],y:v[1],z:v[2]});
test('World preparation proves requested seams against current anatomy and binds them into its source proof',async()=>{
    const {createCompositeAnatomyField}=await import('./helpers/compositeAnatomyField.js'),anatomy=createCompositeAnatomyField();
    try {
        const f=fixture(),report=JSON.parse(fs.readFileSync(new URL('../reports/composite-material-point-rejected-step361.json',import.meta.url),'utf8')),
            origin=JSON.parse(fs.readFileSync(new URL('./fixtures/compositeNativeAppInitial.json',import.meta.url),'utf8')).context.sheaths[0].start,
            p=new Map(report.result.diagnostics.rejectedConfiguration.positions).get('wire')[39].map((v,k)=>v+origin[k]),
            request={owner:'wire',edge:0,fraction:0,sdfSeam:{face:{axis:2,gridIndex:215},domainBox:{lower:p.map(v=>v-.02),upper:p.map(v=>v+.02)}}};
        f.world.contactField=anatomy.field;
        f.imported.state.toolPositions.get('wire')[0]=p;
        const input={world:f.world,state:f.imported.state,bindings:f.imported.bindings,contactMode:'material-points',sdfSeams:[request]},
            prepared=prepareCompositeJointWorldWall(input),site=prepared.wall.pressureSites.find(s=>s.owner==='wire'&&s.fraction===0);
        assert.deepEqual(site.sdfSeam,request.sdfSeam);assert.notEqual(site.sdfSeam,request.sdfSeam);
        assert.equal(prepared.proof.preparationContactQueries,1);assert.equal(prepared.proof.declaredSdfSeams,1);
        assertCompositeJointWorldWall({...input,...prepared});
        assert.throws(()=>prepareCompositeJointWorldWall({...input,sdfSeams:[request,request]}),/distinct existing material site/);
        const automatic={...input,sdfSeams:[],seamUpdates:'automatic',rateMode:'backward-euler-grid'},state=input.state;
        state.wallFrictionState={contactMode:'material-points',pressureDiscretization:'fixed-material-points',pressureSites:[0,1].map(branchIndex=>
            ['wire',0,0,'right',0,{...structuredClone(request.sdfSeam),branchIndex,fraction:0}])};
        const oldHistory=structuredClone(state.wallFrictionState),recovered=prepareCompositeJointWorldWall(automatic);
        assert.deepEqual(recovered.wall.pressureSites.find(s=>s.owner==='wire'&&s.fraction===0).sdfSeam,request.sdfSeam);
        assert.equal(recovered.proof.preparationContactQueries,1);assert.equal(recovered.proof.declaredSdfSeams,1);
        assert.deepEqual(state.wallFrictionState,oldHistory);assertCompositeJointWorldWall({...automatic,...recovered});
        const matching=prepareCompositeJointWorldWall({...automatic,sdfSeams:[request]});assert.equal(matching.proof.preparationContactQueries,1);
        const conflict=structuredClone(request);conflict.sdfSeam.domainBox.lower[0]-=.001;
        assert.throws(()=>prepareCompositeJointWorldWall({...automatic,sdfSeams:[conflict]}),/conflicts with accepted history/);
        state.wallFrictionState.pressureSites[1][5].domainBox.lower[0]-=.001;
        assert.throws(()=>prepareCompositeJointWorldWall(automatic),/Conflicting accepted/);
        state.wallFrictionState=structuredClone(oldHistory);state.wallFrictionState.pressureSites.pop();
        assert.throws(()=>prepareCompositeJointWorldWall(automatic),/both branch descriptors/);
        state.wallFrictionState=structuredClone(oldHistory);
        const oldPoint=state.toolPositions.get('wire')[0];state.toolPositions.get('wire')[0]=oldPoint.map(v=>v+10);
        assert.throws(()=>prepareCompositeJointWorldWall(automatic),/not proved/);state.toolPositions.get('wire')[0]=oldPoint;
        delete state.wallFrictionState;
        const cold=prepareCompositeJointWorldWall(automatic);
        assert.ok(cold.wall.pressureSites.every(s=>s.sdfSeam===undefined));assert.equal(cold.proof.declaredSdfSeams,undefined);

        request.sdfSeam.domainBox.lower[0]-=1;
        assertCompositeJointWorldWall({...input,...prepared});
        site.sdfSeam.domainBox.lower[0]-=1;
        assert.throws(()=>assertCompositeJointWorldWall({...input,...prepared}),/prepared-options/);
        assert.throws(()=>prepareCompositeJointWorldWall(input),/not proved/);
        assert.throws(()=>prepareCompositeJointWorldWall({...input,contactMode:'capsule'}),/material-point/);
        assert.throws(()=>prepareCompositeJointWorldWall({...input,sdfSeams:[{...request,owner:'missing'}]}),/existing material site/);
    } finally {anatomy.dispose();}
});
const accepted=r=>assert.equal(r.accepted,true,JSON.stringify(r));
const close=(a,b,t=1e-8)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}`);
function fixture({differentGrid=false,initialMotion=false}={}) {
    let imported,preparations=0;
    const configuration={plane:{normal:[0,1,0],offset:0}};
    const field={calls:0,queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out=createContactResult()){
        this.calls++;const t=ay<by?0:1,p=t===0?[ax,ay,az]:[bx,by,bz];
        out.signedDistance=p[1];out.signedGap=p[1]-radius;out.segmentT=t;out.inward.values.set([0,1,0]);out.closestPoint.values.set([p[0],0,p[2]]);
        out.source='analytic-plane';out.faceIndex=0;out.capsuleSampleCount=2;return out;
    }};
    const adapter=createCompositeJointWorldAdapter({worldWall:configuration,initialize:()=>imported,prepareStep({state}){preparations++;return options(state);}});
    const world=new EndovascularPhysicsWorld({fixedDt:dt,contactField:field,wholeStepSystem:adapter});
    const tools=['wire','catheter'].map(toolId=>{
        const coordinates=differentGrid?(toolId==='wire'?[0,5,10,15,20]:[0,4,8,12]):[0,2,4],radius=toolId==='wire'?.16:.8;
        const body=world.createRod(toolId==='wire'?'guidewire':'catheter',coordinates.length,coordinates[1]);
        coordinates.forEach((x,i)=>body.setNodePosition(i,x,radius+.00001+(toolId==='wire'?.000002:.002)*x,toolId==='wire'?2:0));
        body.nodeRadius.fill(radius);body.setCollisionRange(0,differentGrid?body.segmentCount-1:0);
        body.wallStaticFriction=body.wallKineticFriction=toolId==='wire'?.006:.002;
        const positions=coordinates.map((_,i)=>[body.x[i],body.y[i],body.z[i]]),reference=captureCompositeReferenceFrames(positions);
        reference.forEach((f,edge)=>{const q=createBishopFrame(obj(f.tangent),obj(f.director));for(const key of ['x','y','z','w'])body[`orientation${key.toUpperCase()}`][edge]=q[key];});
        body.copyCurrentToPrevious();
        return {body,toolId,nodeCoordinates:coordinates,reference,angles:reference.map(()=>0),referenceTwists:reference.slice(1).map(()=>0),winding:'explicit-unwrapped',
            velocityInterpretation:'physical-material-velocity',materialLabels:coordinates.map(x=>20+x),
            material:{dsDx:1,massPerMaterialLength:.13,materialAt:()=>compileCompositeMaterial({EI1:2,EI2:3,GJ:1})}};
    });
    if(initialMotion)for(const t of tools) {
        t.angularVelocityInterpretation='physical-material-angular-velocity';
        t.body.velocityX.fill(t.toolId==='wire'?1/1024:-1/2048);
        t.body.angularVelocityX.fill(t.toolId==='wire'?.125:-.25);
        t.body.wallKineticFriction=t.body.wallStaticFriction/2;
    }
    imported=importCompositeJointWorld({tools});
    function options(state) {
        return {dt,torsionMode:'quasi-static',contacts:'none',wall:'none',
            inertia:{previousPositions:structuredClone(state.toolPositions),inertiaEdges:state.layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,massPerMaterialLength:.13,
                materialMap:{sStart:20+state.coordinates[e],dsDx:1,dsDt:0},oldMaterialVelocities:structuredClone(state.materialVelocities[e].tools.find(t=>t.id===id).velocities)}))}))},
            boundaries:{positions:tools.map(t=>{const node=imported.mappings.get(t.toolId).nodes.at(-1).jointNode;return {toolId:t.toolId,node,value:imported.state.toolPositions.get(t.toolId)[node].slice()};}),
                spins:tools.map(t=>({toolId:t.toolId,edge:0,value:(t.toolId==='wire'?.03:-.02)*(state.time+dt)}))},
            loads:{forces:tools.map(t=>({toolId:t.toolId,node:0,value:[0,-.4,0]})),torques:tools.map(t=>({toolId:t.toolId,edge:1,value:t.toolId==='wire'?.001:-.001}))}};
    }
    const source=(state=imported.state)=>prepareCompositeJointWorldWall({world,state,bindings:imported.bindings,inertia:options(state).inertia,...configuration});
    return {world,adapter,field,tools,imported,configuration,source,options,get preparations(){return preparations;}};
}

test('actual 5/4 body grids expose every original collision interval with its parent radius, including both tools on one chart edge',()=>{
    const f=fixture({differentGrid:true}),wire=f.tools[0].body,cat=f.tools[1].body;
    wire.setCollisionRange(1,3);cat.setCollisionRange(0,1);
    // A child must keep the ORIGINAL source capsule's maximum radius.
    wire.nodeRadius[1]=.2;wire.nodeRadius[2]=.4;
    const prepared=prepareCompositeJointWorldWall({world:f.world,state:f.imported.state,bindings:f.imported.bindings,...f.configuration,contactMode:'capsule'});
    assert.equal(f.field.calls,0);assert.equal(prepared.wall.field,f.world.contactField);assert.equal(prepared.proof.originalCapsules,5);
    const actual=prepared.wall.contactOwners.edges.flatMap(e=>e.walls.map(w=>({edge:e.edge,...w})));
    const expected=[];
    for(const t of f.tools)for(const e of f.imported.mappings.get(t.toolId).jointEdges)if(e.bodyEdge>=t.body.collisionStartSegment&&e.bodyEdge<=t.body.collisionEndSegment)
        expected.push([t.toolId,e.jointEdge,Math.max(t.body.nodeRadius[e.bodyEdge],t.body.nodeRadius[e.bodyEdge+1])]);
    const order=(a,b)=>JSON.stringify(a).localeCompare(JSON.stringify(b));
    assert.deepEqual(actual.map(w=>[w.owner,w.edge,w.radius]).sort(order),expected.sort(order));
    assert.ok(prepared.proof.overlappingExposedIntervals>0);assert.equal(prepared.proof.jointCapsules,expected.length);
    assertCompositeJointWorldWall({...prepared,world:f.world,state:f.imported.state,bindings:f.imported.bindings});
    prepared.wall.contactOwners.edges.forEach(e=>e.walls=[]);
    assert.throws(()=>assertCompositeJointWorldWall({...prepared,world:f.world,state:f.imported.state,bindings:f.imported.bindings}),/coverage/);
});

test('World source proof cannot be forged, reused after range/radius/law changes, or derived from incomplete own bindings',()=>{
    const f=fixture(),args={world:f.world,state:f.imported.state,bindings:f.imported.bindings},p=f.source();
    assert.throws(()=>assertCompositeJointWorldWall({...args,wall:p.wall,proof:{...p.proof}}),/coverage/);
    for(const change of [b=>b.collisionEndSegment=1,b=>b.nodeRadius[0]=.2,b=>b.wallStaticFriction=.007]) {
        const b=f.tools[0].body,old={end:b.collisionEndSegment,radius:b.nodeRadius[0],mu:b.wallStaticFriction};change(b);
        assert.throws(()=>assertCompositeJointWorldWall({...args,...p}),/coverage/);
        b.collisionEndSegment=old.end;b.nodeRadius[0]=old.radius;b.wallStaticFriction=old.mu;
        assert.doesNotThrow(()=>assertCompositeJointWorldWall({...args,...p}));
    }
    assert.throws(()=>prepareCompositeJointWorldWall({...args,bindings:f.imported.bindings.slice(1)}),/every actual/);
    const bad=f.imported.bindings.map(b=>({...b,nodes:b.nodes.slice(1)}));
    assert.throws(()=>prepareCompositeJointWorldWall({...args,bindings:bad}),/complete/);
    f.field.version=1;assert.throws(()=>assertCompositeJointWorldWall({...args,...p}),/coverage/);
    delete f.field.version;assert.doesNotThrow(()=>assertCompositeJointWorldWall({...args,...p}));
});

test('actual World field runs two complete source-derived wall steps for both tools and retries without consuming dt twice',()=>{
    const f=fixture(),initial=f.imported.state,p=f.source(),direct=advanceCompositeJointTimeStep(initial,{...f.options(initial),wall:p.wall});accepted(direct);
    assert.equal(f.world.advance(dt),1);assert.deepEqual(f.adapter.snapshot(),direct.state);
    for(const toolId of ['wire','catheter']) {
        const samples=f.world.lastStepResult.diagnostics.certificate.wallFriction.samples.filter(s=>s.owner===toolId);
        assert.equal(samples.length,2);assert.ok(samples.some(s=>s.Fn>0));
        assert.ok(samples.every(s=>s.mu.every(mu=>mu===(toolId==='wire'?.006:.002))));
        const balance=f.world.lastStepResult.balances.get(toolId);
        balance.momentumRate.forEach((v,k)=>close(v,balance.appliedForce[k]+balance.boundaryForce[k]+balance.contactForce[k],1e-7));
    }
    const state=f.adapter.snapshot(),cold=advanceCompositeJointTimeStep(state,{...f.options(state),wall:f.source(state).wall});accepted(cold);
    f.adapter.setBudget({contactQueries:cold.diagnostics.contactQueries-1});assert.equal(f.world.advance(dt),0);assert.equal(f.world.accumulator,dt);
    assert.deepEqual(f.adapter.snapshot(),state);const body=f.tools[0].body,radius=body.nodeRadius[0];body.nodeRadius[0]=.2;
    f.adapter.setBudget(null);assert.equal(f.world.advance(0),0);assert.equal(f.world.lastStepResult.status,'joint-world-wall-adapter-required');
    assert.deepEqual(f.adapter.snapshot(),state);body.nodeRadius[0]=radius;
    assert.equal(f.world.advance(0),1);assert.deepEqual(f.adapter.snapshot(),cold.state);assert.equal(f.preparations,2);
    assert.equal(f.world.stepCount,2);assert.equal(f.world.accumulator,0);assert.equal(f.adapter.diagnostics.publications,2);
});

test('actual unequal source coefficients use static and kinetic branches and an entirely shielded source proves zero exposure',()=>{
    const f=fixture();f.tools[0].body.wallStaticFriction=.006;f.tools[0].body.wallKineticFriction=.002;f.tools[0].body.wallFriction=.002;
    const p0=f.source();assert.equal(f.field.calls,0);assert.equal(p0.wall.friction.law,'coulomb-static-kinetic');
    assert.deepEqual(p0.wall.friction.muByOwner.find(c=>c.owner==='wire'),{owner:'wire',muStatic:[.006,.006],muKinetic:[.002,.002]});
    for(let step=1;step<=2;step++) {
        accepted(f.world.stepFixed());assert.equal(f.world.lastStepResult.diagnostics.wallFrictionModes.accepted,true);
        assert.equal(f.adapter.snapshot().wallFrictionState.law,'coulomb-static-kinetic');assert.equal(f.adapter.snapshot().step,step);
    }
    for(const t of f.tools)t.body.setCollisionRange(0,-1);
    const p=f.source();assert.equal(p.proof.originalCapsules,0);assert.equal(p.proof.jointCapsules,0);assert.equal(p.wall,'none');
});

test('initial source rest cannot be invented from moving material or angular input, and angular changes during retry invalidate its proof',()=>{
    for(const motion of ['prepared','angular','source']) {
        const f=fixture(),body=f.tools[0].body;body.wallKineticFriction=.002;
        if(motion==='angular')body.angularVelocityX[0]=.1;
        if(motion==='source')body.velocityZ[0]=.1;
        const inertia=f.options(f.imported.state).inertia;
        if(motion==='prepared')inertia.inertiaEdges[0].tools.find(t=>t.id==='wire').oldMaterialVelocities[0][0]=.1;
        assert.throws(()=>prepareCompositeJointWorldWall({world:f.world,state:f.imported.state,bindings:f.imported.bindings,inertia,...f.configuration}),/Moving initial/);
    }
    const f=fixture();f.tools[0].body.wallKineticFriction=.002;f.adapter.setBudget({directions:0});assert.equal(f.world.advance(dt),0);
    f.tools[0].body.angularVelocityX[0]=.1;f.adapter.setBudget(null);assert.equal(f.world.advance(0),0);assert.equal(f.world.lastStepResult.status,'joint-world-wall-adapter-required');
    f.tools[0].body.angularVelocityX[0]=0;assert.equal(f.world.advance(0),1);assert.equal(f.preparations,1);
});

test('the original Aorta field and actual Float32 body radii enter the World bridge and certify their original capsule gap',context=>{
    const bytes=fs.readFileSync(new URL('../res/Aorta_plain.collision.bin',import.meta.url));
    const field=new VesselContactField(decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength)));
    let imported;
    const adapter=createCompositeJointWorldAdapter({worldWall:{contactMode:'capsule'},initialize:()=>imported,prepareStep:({state})=>options(state)});
    const world=new EndovascularPhysicsWorld({fixedDt:dt,contactField:field,wholeStepSystem:adapter});
    const a=[65.00287246704102,-462.00980948623305,-79.56869888305664],b=[64.95548751831055,-461.7916869276393,-79.65612350463867],points=[a,b,b.map((v,k)=>2*v-a[k])];
    const body=world.createRod('catheter',3,1);points.forEach((p,i)=>body.setNodePosition(i,...p));
    const actual=points.map((_,i)=>[body.x[i],body.y[i],body.z[i]]),length=Math.hypot(...actual[1].map((v,k)=>v-actual[0][k])),coordinates=[0,length,2*length];
    body.restLength.fill(length);body.nodeRadius.fill(.4445);body.setCollisionRange(0,0);body.wallStaticFriction=body.wallKineticFriction=0;
    const reference=captureCompositeReferenceFrames(actual);
    reference.forEach((f,edge)=>{const q=createBishopFrame(obj(f.tangent),obj(f.director));for(const key of ['x','y','z','w'])body[`orientation${key.toUpperCase()}`][edge]=q[key];});
    body.copyCurrentToPrevious();
    imported=importCompositeJointWorld({tools:[{body,toolId:'catheter',nodeCoordinates:coordinates,reference,angles:[0,0],referenceTwists:[0],winding:'explicit-unwrapped',
        velocityInterpretation:'physical-material-velocity',materialLabels:coordinates,
        material:{dsDx:1,massPerMaterialLength:.01,materialAt:()=>compileCompositeMaterial({EI1:.001,GJ:.001})}}]});
    function options(state){return {dt,torsionMode:'quasi-static',contacts:'none',inertia:{previousPositions:structuredClone(state.toolPositions),
        inertiaEdges:state.layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,massPerMaterialLength:.01,materialMap:{sStart:coordinates[e],dsDx:1,dsDt:0},
            oldMaterialVelocities:structuredClone(state.materialVelocities[e].tools.find(t=>t.id===id).velocities)}))}))},
        boundaries:{positions:[],spins:[{toolId:'catheter',edge:0,value:0}]}};}
    const originalRadius=body.nodeRadius[0],initial=field.queryCapsuleCoordinates(...actual[0],...actual[1],originalRadius,createContactResult()).signedGap;
    assert.ok(initial<0);
    for(let step=1;step<=2;step++) {
        accepted(world.stepFixed());const state=adapter.snapshot(),p=state.toolPositions.get('catheter');
        const raw=field.queryCapsuleCoordinates(...p[0],...p[1],originalRadius,createContactResult());
        assert.ok(raw.signedGap>=-1e-8);assert.ok(world.lastStepResult.diagnostics.certificate.wall.converged);
        assert.equal(state.step,step);assert.equal(body.jointStateView.step,step);
        assert.ok(state.wallContactState.normalForces.some(v=>v>0));
        const balance=world.lastStepResult.balances.get('catheter');balance.momentumRate.forEach((v,k)=>close(v,balance.appliedForce[k]+balance.boundaryForce[k]+balance.contactForce[k],1e-7));
        context.diagnostic(JSON.stringify({step,initialGap:initial,finalGap:raw.signedGap,source:raw.source,directions:world.lastStepResult.diagnostics.directions,evaluations:world.lastStepResult.diagnostics.evaluations}));
    }
});

test('explicitly moving source tools enter kinetic wall friction through World, with identical solve and atomic retry',()=>{
    const f=fixture({initialMotion:true}),initial=f.imported.state,p=f.source(),options=f.options(initial);
    assert.equal(f.field.calls,0);assert.ok(p.wall.friction.incomingSurfaceMotion.every(m=>m.interpretation==='physical-material-surface-velocity'));
    // With tangent along x and wall normal y, both declared axial velocities
    // are nonzero. Any nonzero explicit tangent input selects the same branch;
    // its magnitude is not an artificial displacement or load in the solve.
    const oracleWall={...p.wall,friction:{...p.wall.friction,incomingSurfaceMotion:p.wall.friction.incomingSurfaceMotion.map(m=>({
        owner:m.owner,edge:m.edge,node:m.node,interpretation:'physical-tangential-surface-velocity',velocity:[1,0]}))}};
    const oracle=advanceCompositeJointTimeStep(initial,{...options,wall:oracleWall});accepted(oracle);
    const direct=advanceCompositeJointTimeStep(initial,{...options,wall:p.wall});accepted(direct);
    assert.deepEqual(direct.state,oracle.state);assert.deepEqual(direct.balances,oracle.balances);
    assert.ok(direct.diagnostics.wallFrictionModes.samples.every(s=>s.initialMode==='kinetic'));
    f.adapter.setBudget({directions:0});assert.equal(f.world.advance(dt),0);assert.equal(f.world.accumulator,dt);
    const body=f.tools[0].body,omega=body.angularVelocityX[0],v=body.velocityX[0];
    for(const [key,value] of [['angularVelocityX',omega+1],['velocityX',v+1]]) {
        const old=body[key][0];body[key][0]=value;f.adapter.setBudget(null);assert.equal(f.world.advance(0),0);
        assert.equal(f.world.lastStepResult.status,'joint-world-wall-adapter-required');assert.equal(f.adapter.snapshot().step,0);body[key][0]=old;
    }
    assert.equal(f.world.advance(0),1);assert.deepEqual(f.adapter.snapshot(),direct.state);
    assert.equal(f.preparations,1);assert.equal(f.world.accumulator,0);
    // Accepted same-label mode history is authoritative in the second dt.
    // Quasi-static torsion still supplies no inferred instantaneous omega.
    assert.ok(f.adapter.snapshot().materialVelocities.flatMap(e=>e.tools).every(t=>t.angularVelocity===null));
    assert.equal(f.world.advance(dt),1);assert.equal(f.adapter.snapshot().step,2);assert.equal(f.preparations,2);
});

test('moving initial union surfaces preserve own velocities and rates and reject incompatible inertia or feed maps',()=>{
    const f=fixture({differentGrid:true,initialMotion:true}),p=f.source(),args={world:f.world,state:f.imported.state,bindings:f.imported.bindings};
    assert.equal(f.field.calls,0);
    for(const m of p.wall.friction.incomingSurfaceMotion) {
        assert.deepEqual(m.centerVelocities,Array.from({length:2},()=>[m.owner==='wire'?1/1024:-1/2048,0,0]));
        assert.deepEqual(m.angularVelocity,[m.owner==='wire'?.125:-.25,0,0]);
        assert.deepEqual(m.materialLabels,[20+f.imported.state.coordinates[m.edge],20+f.imported.state.coordinates[m.edge+1]]);
    }
    for(const change of [t=>t.oldMaterialVelocities[0][0]+=1,t=>t.materialMap.dsDt=.1,t=>t.materialMap.sStart+=1]) {
        const inertia=f.options(f.imported.state).inertia;change(inertia.inertiaEdges[0].tools[0]);
        assert.throws(()=>prepareCompositeJointWorldWall({...args,inertia,...f.configuration}),/inertia|fixed material chart/);
    }
    const inertia=f.options(f.imported.state).inertia;
    f.imported.state.materialVelocities[0].tools[0].angularVelocity[0]+=1;
    assert.throws(()=>assertCompositeJointWorldWall({...args,...p,inertia}),/angular velocity disagrees/);
});

test('actual zero exposed intervals return wall:none with private coverage proof that invalidates on first exposed source edge',()=>{
    const f=fixture(),state=f.imported.state,bindings=f.imported.bindings,args={world:f.world,state,bindings};
    for(const t of f.tools)t.body.setCollisionRange(t.body.segmentCount,t.body.segmentCount-1);
    const p=f.source();assert.equal(p.wall,'none');assert.equal(p.proof.originalCapsules,0);assert.equal(f.field.calls,0);
    assert.equal(assertCompositeJointWorldWall({...args,...p}),p.proof);
    assert.throws(()=>assertCompositeJointWorldWall({...args,...p,proof:{...p.proof}}),/coverage/);
    assert.throws(()=>assertCompositeJointWorldWall({...args,...p,wall:{field:f.field}}),/coverage/);
    const old=f.tools[0].body.nodeRadius[0];f.tools[0].body.nodeRadius[0]=old+.1;
    assert.throws(()=>assertCompositeJointWorldWall({...args,...p}),/coverage/);f.tools[0].body.nodeRadius[0]=old;
    f.tools[0].body.setCollisionRange(0,0);assert.throws(()=>assertCompositeJointWorldWall({...args,...p}),/coverage/);
    assert.notEqual(f.source().wall,'none');
});

test('initial native source motion accepts equivalent complete piecewise affine inertia and rejects an altered interior velocity',()=>{
    const f=fixture({differentGrid:true,initialMotion:true}),inertia=f.options(f.imported.state).inertia;
    for(const e of inertia.inertiaEdges)for(const t of e.tools){const v=t.oldMaterialVelocities;delete t.oldMaterialVelocities;
        t.oldVelocityPieces=[[0,.4],[.4,1]].map(fractions=>({fractions,oldMaterialVelocities:fractions.map(x=>v[0].map((a,k)=>a+x*(v[1][k]-a))),interpretation:'physical-material-velocity'}));}
    const args={world:f.world,state:f.imported.state,bindings:f.imported.bindings,inertia,...f.configuration},p=prepareCompositeJointWorldWall(args);
    assert.ok(p.wall.friction.incomingSurfaceMotion.every(m=>m.interpretation==='physical-material-surface-velocity'));
    assert.equal(assertCompositeJointWorldWall({...args,...p}),p.proof);
    inertia.inertiaEdges[0].tools[0].oldVelocityPieces[0].oldMaterialVelocities[1][0]+=.01;
    assert.throws(()=>assertCompositeJointWorldWall({...args,...p}),/inertia disagrees/);
});

test('native implicit capsule friction starts after a free accepted step and advances fed labels across an internal edge',()=>{
    const snapshot=s=>JSON.stringify(s,(_,v)=>v instanceof Map?Array.from(v):typeof v==='function'?v.toString():v);
    const f=fixture({initialMotion:true}),state=f.imported.state;for(const t of f.tools)t.body.setCollisionRange(0,-1);
    const free=f.options(state);free.boundaries.positions=[];free.boundaries.spins.forEach(b=>b.value=0);free.loads={forces:[],torques:[]};
    const first=advanceCompositeJointTimeStep(state,free);accepted(first);assert.equal(first.state.wallFrictionState,undefined);
    assert.equal(first.state.nativeRateHistory.kind,'owned-native-endpoint-material-rate');
    for(const t of f.tools)t.body.setCollisionRange(1,1);
    let current=first.state;
    for(let step=0;step<2;step++) {
        const input=f.options(current);input.boundaries.positions=[];input.boundaries.spins.forEach(b=>b.value=0);
        input.loads={forces:f.tools.map(t=>({toolId:t.toolId,node:1,value:[0,-.4,0]})),torques:[]};
        input.inertia.inertiaEdges.forEach((e,edge)=>e.tools.forEach(t=>{const old=current.materialVelocities[edge].tools.find(o=>o.id===t.id),feed=t.id==='wire'?-.04:-.02;
            t.materialMap={sStart:old.sStart+dt*feed,dsDx:1,dsDt:feed};}));
        const source=prepareCompositeJointWorldWall({world:f.world,state:current,bindings:f.imported.bindings,inertia:input.inertia,
            ...f.configuration,contactMode:'capsule',rateMode:'backward-euler-grid'});input.wall=source.wall;
        assert.equal(source.wall.friction.incomingRateHistory.kind,'owned-native-endpoint-material-rate');
        assert.equal(source.wall.friction.finiteStepSlipKnown,false);
        const before=snapshot(current),limited=advanceCompositeJointTimeStep(current,{...input,budget:{directions:0}});assert.equal(limited.accepted,false);assert.equal(snapshot(current),before);
        const r=advanceCompositeJointTimeStep(current,input);accepted(r);
        assert.ok(r.diagnostics.certificate.wallFriction.samples.every(s=>s.finiteStepSlipKnown===false&&s.converged));
        assert.ok(r.state.wallContactState.normalForces.some(x=>x>0));for(const b of r.balances.values())b.residual.forEach(v=>close(v,0,1e-7));
        assert.equal(snapshot(current),before);current=r.state;
    }
});

test('native implicit wall exactly eliminates a source-proved open zero cone without inventing a surface witness',()=>{
    const f=fixture({initialMotion:true}),query=f.field.queryCapsuleCoordinates;
    f.field.queryCapsuleCoordinates=function(...args){const out=query.apply(this,args);out.signedDistance+=10;out.signedGap+=10;out.source='safe-core';return out;};
    const input=f.options(f.imported.state);input.loads={forces:[],torques:[]};input.boundaries.spins.forEach(b=>b.value=0);
    input.wall=prepareCompositeJointWorldWall({world:f.world,state:f.imported.state,bindings:f.imported.bindings,inertia:input.inertia,
        contactMode:'capsule',rateMode:'backward-euler-grid'}).wall;
    const result=advanceCompositeJointTimeStep(f.imported.state,input);accepted(result);
    assert.ok(result.diagnostics.certificate.wallFriction.samples.every(s=>s.unloaded&&s.slip===null&&s.Fn===0));
    assert.ok(result.state.wallFrictionState.tractions.every(v=>v===0));
});

for(const initialMotion of [false,true])test(`fixed material wall sites preserve original exposure and run a joint step with independent tool friction (${initialMotion})`,()=>{
    const f=fixture({differentGrid:true,initialMotion});f.field.voxelSize=.5;
    Object.assign(f.configuration,{contactMode:'material-points',rateMode:'backward-euler-grid'});
    const prepared=f.source();
    assert.equal(prepared.wall.pressureDiscretization,'fixed-material-points');
    assert.ok(prepared.wall.pressureSites.some(s=>s.fraction>0&&s.fraction<1));
    assert.deepEqual(new Set(prepared.wall.pressureSites.map(s=>s.owner)),new Set(['wire','catheter']));
    assertCompositeJointWorldWall({world:f.world,state:f.imported.state,bindings:f.imported.bindings,inertia:f.options(f.imported.state).inertia,...prepared});
    const result=advanceCompositeJointTimeStep(f.imported.state,{...f.options(f.imported.state),wall:prepared.wall});
    accepted(result);
});

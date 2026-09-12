import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeJointTimeStepState,advanceCompositeJointTimeStep,createCompositeJointTimeStepWorkspace} from '../src/physics/kirchhoffCompositeJointTimeStep.js';
import {prepareCompositeJointWorldSheath,assertCompositeJointWorldSheath,evaluateCompositeJointSheathGeometry,createCompositeJointSheathRows} from '../src/physics/kirchhoffCompositeJointWorldSheath.js';
import {prepareCompositeAppMaterialProfile,prepareCompositeAppInputs} from '../src/physics/kirchhoffCompositeAppInputs.js';

const ids=['wire','catheter'],dt=1/120,zero=[0,0,0],close=(a,b,t=1e-8)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}`),
    tol={force:1e-7,linearConstraint:1e-10,sheathGap:1e-8,sheathNcp:1e-8,sheathWork:1e-9};
function fixture({clearance=.125}={}) {
    const coordinates=[0,1,2],layout=createCompositeChainLayout([ids,ids]),positions=coordinates.map(x=>[x,-clearance,0]),wire=coordinates.map(x=>[x,clearance,0]),
        modes=coordinates.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]})),tools=ids.map(id=>({id,dsDx:1,
            reference:captureCompositeReferenceFrames(id==='wire'?wire:positions),referenceTwists:new Float64Array(1),
            material:compileCompositeMaterial({EI1:id==='wire'?2:9,EI2:id==='wire'?3:12,GJ:id==='wire'?1:4})})),
        state=createCompositeJointTimeStepState({layout,coordinates,positions,tools,modes,relative:modes.flatMap(()=>[0,2*clearance,0]),
            angles:new Map(ids.map(id=>[id,new Float64Array(2)])),restLengths:new Map(ids.map(id=>[id,[1,1]])),
            inertiaGeometryByTool:new Map(ids.map(id=>[id,{}])),elasticityGeometry:'continuous-material-frame',lengthGeometry:'continuous-arclength',materialCoordinate:'reference-arclength'}),
        world=new EndovascularPhysicsWorld({fixedDt:dt}),bindings=ids.map(toolId=>({toolId,body:world.createRod(toolId==='wire'?'guidewire':'catheter',3,1),
            nodes:coordinates.map((_,node)=>({node,jointNode:node}))}));
    bindings.forEach(b=>{b.body.nodeRadius.fill(.25);b.body.sheathMaterialEndNode=2;});
    world.addSheath({start:{x:0,y:0,z:0},end:{x:4,y:0,z:0},innerRadius:.25+clearance,proximalExtension:1,bodies:bindings.map(b=>b.body)});
    return {state,world,bindings,source:(current=state)=>prepareCompositeJointWorldSheath({world,state:current,bindings})};
}
function input(state,sheath) {
    return {dt,torsionMode:'quasi-static',contacts:'none',sheath,
        inertia:{previousPositions:structuredClone(state.toolPositions),inertiaEdges:state.layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,massPerMaterialLength:.13,
            materialMap:{sStart:20+state.coordinates[e],dsDx:1,dsDt:0},oldMaterialVelocities:Array.from({length:2},()=>[id==='wire'?.2:-.1,0,0])}))}))},
        boundaries:{positions:[],spins:ids.map(toolId=>({toolId,edge:0,value:toolId==='wire'?.1:-.2}))},
        loads:{forces:ids.flatMap(toolId=>[0,1,2].map(node=>({toolId,node,value:[0,toolId==='wire'?1:-.4,0]}))),torques:[]}};
}

test('actual sheath adapter preserves source material ownership, radii, both tools and open tube geometry; proof rejects omission or mutation',()=>{
    const f=fixture(),[wire,cat]=f.bindings.map(b=>b.body);wire.sheathMaterialEndNode=1;cat.nodeRadius[1]=.3125;
    const p=f.source();assert.equal(p.sheath.sites.length,5);
    assert.equal(p.sheath.sites.find(s=>s.toolId==='catheter'&&s.node===1).clearance,.0625);
    assert.ok(!p.sheath.sites.some(s=>s.toolId==='wire'&&s.node===2));
    assert.equal(assertCompositeJointWorldSheath({...p,...f}),true);
    const copied=structuredClone(p.sheath);copied.sites.pop();assert.throws(()=>assertCompositeJointWorldSheath({...f,proof:p.proof,sheath:copied}),/changed/);
    assert.throws(()=>assertCompositeJointWorldSheath({...f,sheath:p.sheath,proof:{...p.proof}}),/changed/);
    f.world.sheaths[0].innerRadius+=.01;assert.throws(()=>assertCompositeJointWorldSheath({...p,...f}),/changed/);
    f.world.sheaths[0].innerRadius-=.01;wire.sheathMaterialEndNode=2;assert.throws(()=>assertCompositeJointWorldSheath({...p,...f}),/changed/);
});

test('physical radial source G and signed reaction derivative match independent finite differences and leave axial feed free',()=>{
    const axis=[1,2,3].map(v=>v/Math.sqrt(14)),site={axis,start:[.4,-.3,.2],clearance:.4,length:4,proximalExtension:2},p=[.7,-.1,.7],
        r=evaluateCompositeJointSheathGeometry(site,p),h=1e-6;
    assert.equal(r.supported,true);
    for(let j=0;j<3;j++){
        const plus=p.slice(),minus=p.slice();plus[j]+=h;minus[j]-=h;
        const a=evaluateCompositeJointSheathGeometry(site,plus),b=evaluateCompositeJointSheathGeometry(site,minus);
        close((a.gap-b.gap)/(2*h),r.gapJacobian[j],1e-10);
        for(let i=0;i<3;i++)close((a.normalForceColumn[i]-b.normalForceColumn[i])/(2*h),r.normalDerivative[3*i+j],1e-9);
    }
    close(r.normalForceColumn.reduce((sum,v,i)=>sum+axis[i]*v,0),0,1e-14);
    const shifted=evaluateCompositeJointSheathGeometry(site,p.map((v,k)=>v+.8*axis[k]));close(shifted.gap,r.gap,1e-14);
    assert.equal(evaluateCompositeJointSheathGeometry(site,p.map((v,k)=>v+9*axis[k])).applicable,false);
});

test('nodal pressure rows pull full radial reactions to common and rotated relative coordinates with their complete geometric tangent',()=>{
    const f=fixture(),sheath=f.source().sheath,modes=f.state.modes.map(m=>({...m,basis:[[0,1,0],[0,0,1],[1,0,0]]})),
        manager=createCompositeJointSheathRows({layout:f.state.layout,modes,relativeToolId:'wire',sheath,tolerances:tol}),positions=structuredClone(f.state.toolPositions),
        refresh=()=>{const cg=new Float64Array(f.state.layout.dofCount),rg=new Float64Array(9),certificate=manager.refresh({toolPositions:positions,commonResidual:cg,relativeResidual:rg,order:'full'});return {cg,rg,certificate};};
    positions.get('wire')[1][2]=.04;manager.forces[1]=1.2;const initial=refresh(),row=manager.rows[1],H=row.geometricTangent.slice(),J=row.jacobian.slice(),F=row.forceColumn.slice(),h=1e-6;
    close(initial.cg[f.state.layout.positions[1]+1],initial.rg[3]);
    assert.ok(row.relativeDofs.length===3&&row.commonDofs.length===3);
    for(let j=0;j<6;j++) {
        const delta=j<3?[[1,0,0],[0,1,0],[0,0,1]][j]:modes[1].basis[j-3],p=positions.get('wire')[1],base=p.slice();
        p.forEach((_,k)=>p[k]=base[k]+h*delta[k]);refresh();const a=manager.rows[1].forceColumn.slice(),ga=manager.rows[1].residual;
        p.forEach((_,k)=>p[k]=base[k]-h*delta[k]);refresh();const b=manager.rows[1].forceColumn.slice(),gb=manager.rows[1].residual;
        close((ga-gb)/(2*h),J[j],1e-9);for(let i=0;i<6;i++)close(1.2*(a[i]-b[i])/(2*h),H[6*i+j],2e-8);
        p.forEach((_,k)=>p[k]=base[k]);
    }
    refresh();F.forEach((v,i)=>close(v,row.forceColumn[i],1e-14));
    manager.forces[1]=-1;assert.equal(refresh().certificate.converged,false);assert.throws(()=>manager.commit(),/freshly certified/);
});

test('full continuous joint step balances both tools against actual sheath while sliding and spinning independently; retry owns reactions',()=>{
    const f=fixture(),sheath=f.source().sheath,options=input(f.state,sheath),workspace=createCompositeJointTimeStepWorkspace(f.state),before=structuredClone(f.state),
        limited=advanceCompositeJointTimeStep(f.state,{...options,workspace,budget:{directions:0}});
    assert.equal(limited.accepted,false);assert.deepEqual(f.state,before);
    const r=advanceCompositeJointTimeStep(f.state,{...options,workspace});assert.equal(r.accepted,true,JSON.stringify({status:r.status,error:r.error,certificate:r.diagnostics?.certificate}));
    assert.equal(r.diagnostics.certificate.sheath.pressureDiscretization,'actual-world-nodes');assert.equal(r.diagnostics.sheathRows,6);
    for(const id of ids)for(let node=0;node<3;node++) {
        close(r.state.toolPositions.get(id)[node][0],node+dt*(id==='wire'?.2:-.1),1e-9);
        close(r.state.toolPositions.get(id)[node][1],id==='wire'?.125:-.125,1e-9);
        close(r.contactForces.get(id)[node][1],id==='wire'?-1:.4,1e-7);
    }
    for(const b of r.balances.values())b.residual.forEach(v=>close(v,0,1e-7));
    const second=advanceCompositeJointTimeStep(r.state,{...input(r.state,sheath),workspace});assert.equal(second.accepted,true,JSON.stringify({status:second.status,error:second.error}));
    assert.ok(second.state.sheathContactState.forces.some(v=>v>0));assert.deepEqual(f.state,before);
    assert.throws(()=>advanceCompositeJointTimeStep(second.state,{...input(second.state,sheath),sheath:'none'}),/cannot discard/);
});

test('zero-clearance source uses two signed radial reactions without pinning axial feed; stale commit and absent slab unload are explicit',()=>{
    const f=fixture({clearance:0}),sheath=f.source().sheath,options=input(f.state,sheath),r=advanceCompositeJointTimeStep(f.state,options);
    assert.equal(r.accepted,true,JSON.stringify({status:r.status,error:r.error,certificate:r.diagnostics?.certificate}));
    assert.equal(r.diagnostics.sheathRows,12);assert.ok(r.state.sheathContactState.forces.some(v=>v<0));
    for(const id of ids)r.state.toolPositions.get(id).forEach((p,node)=>{close(p[0],node+dt*(id==='wire'?.2:-.1));close(p[1],0);close(p[2],0);});
    const manager=createCompositeJointSheathRows({layout:r.state.layout,modes:r.state.modes,sheath,tolerances:tol}),points=structuredClone(r.state.toolPositions),
        refresh=()=>manager.refresh({toolPositions:points,commonResidual:new Float64Array(r.state.layout.dofCount),relativeResidual:new Float64Array(9),order:'full'});
    assert.equal(refresh().converged,true);manager.commit();points.get('wire')[0][0]+=.01;assert.throws(()=>manager.commit(),/unchanged/);
    points.get('wire')[0][0]=20;manager.forces[0]=.7;assert.equal(refresh().converged,false);assert.equal(manager.rows[0].active,false);assert.equal(manager.rows[0].multiplierDerivative,1);
});

test('actual native hold step162 pivots an entering contact and tensile neighbour within the same coupled direction',()=>{
    const capture=JSON.parse(readFileSync(new URL('./fixtures/compositeNativeSheathHold162.json',import.meta.url),'utf8')),raw=capture.state,
        state=createCompositeJointTimeStepState({...raw,layout:createCompositeChainLayout(raw.edgeToolIds),angles:new Map(raw.angles),restLengths:new Map(raw.restLengths),
            tools:raw.tools.map(t=>({...t,...prepareCompositeAppMaterialProfile(t.appMaterialProfile,t.appMaterialProfile.coordinateOrigin,t.appMaterialProfile.materialOrigin)}))});
    state.materialVelocities=structuredClone(raw.materialVelocities);state.boundaryMultipliers=new Map(raw.boundaryMultipliers);
    const prepared=prepareCompositeAppInputs({state,dt,commands:state.tools.map(t=>({toolId:t.id,labelShift:0,feedVelocity:0,spinIncrement:0})),positionBoundaries:capture.positionBoundaries}),
        options={...prepared.options,sheath:JSON.parse(raw.sheathContactState.signature).sheath},before=structuredClone({...state,tools:state.tools.map(({materialAt,...t})=>t)}),
        old=advanceCompositeJointTimeStep(prepared.state,{...options,globalization:'newton',budget:{evaluations:32}});
    assert.equal(old.accepted,false);assert.equal(old.status,'evaluation-budget');
    const result=advanceCompositeJointTimeStep(prepared.state,{...options,budget:{directions:6,evaluations:12}});
    assert.equal(result.accepted,true,JSON.stringify({status:result.status,certificate:result.diagnostics?.certificate}));
    assert.equal(result.state.step,163);assert.equal(result.state.time,state.time+dt);
    const certificate=result.diagnostics.certificate,changes=result.diagnostics.sheathActiveSetPivots.flatMap(p=>p.changes);
    assert.ok(changes.some(p=>JSON.parse(p.site).at(-1)===9&&p.active));
    assert.ok(changes.some(p=>JSON.parse(p.site).at(-1)===10&&!p.active&&p.predictedForce<0));
    const release=certificate.sheath.samples.find(p=>JSON.parse(p.id).at(-1)===10);
    assert.equal(release.force,0);assert.equal(release.active,false);assert.ok(release.gap>0);
    assert.ok(certificate.force<=1e-7);assert.ok(certificate.length<=1e-8);assert.ok(certificate.sheath.complementarity<=1e-9);
    assert.deepEqual(result.diagnostics.acceptedAlphas,[1,1]);assert.equal(result.diagnostics.regularizationActivations.length,0);
    assert.deepEqual({...state,tools:state.tools.map(({materialAt,...t})=>t)},before);
    const next=prepareCompositeAppInputs({state:result.state,dt,commands:state.tools.map(t=>({toolId:t.id,labelShift:0,feedVelocity:0,spinIncrement:0})),positionBoundaries:capture.positionBoundaries});
    assert.equal(advanceCompositeJointTimeStep(next.state,{...next.options,sheath:options.sheath}).accepted,true);
});

test('actual late hold releases sub-tolerance tensile targets through an exact inactive equation without relaxing the force-sign gate',()=>{
    const capture=JSON.parse(readFileSync(new URL('./fixtures/compositeNativeSheathHold3550.json',import.meta.url),'utf8')),raw=capture.state,
        state=createCompositeJointTimeStepState({...raw,layout:createCompositeChainLayout(raw.edgeToolIds),angles:new Map(raw.angles),restLengths:new Map(raw.restLengths),
            tools:raw.tools.map(t=>({...t,...prepareCompositeAppMaterialProfile(t.appMaterialProfile,t.appMaterialProfile.coordinateOrigin,t.appMaterialProfile.materialOrigin)}))});
    state.materialVelocities=structuredClone(raw.materialVelocities);state.boundaryMultipliers=new Map(raw.boundaryMultipliers);
    const prepare=current=>prepareCompositeAppInputs({state:current,dt,commands:current.tools.map(t=>({toolId:t.id,labelShift:0,feedVelocity:0,spinIncrement:0})),positionBoundaries:capture.positionBoundaries}),
        prepared=prepare(state),sheath=JSON.parse(raw.sheathContactState.signature).sheath,
        result=advanceCompositeJointTimeStep(prepared.state,{...prepared.options,sheath,budget:{directions:3,evaluations:8}});
    assert.equal(result.accepted,true,JSON.stringify({status:result.status,certificate:result.diagnostics?.certificate}));
    const changed=result.diagnostics.sheathActiveSetPivots.flatMap(p=>p.changes).find(p=>!p.active&&p.predictedForce<0&&p.predictedForce>-tol.force);
    assert.ok(changed,'The original late-step tensile target lies inside the old numerical deadband');
    assert.equal(result.diagnostics.certificate.sheath.samples.find(s=>s.id===changed.site).force,0);
    assert.ok(result.diagnostics.certificate.sheath.samples.every(s=>s.force>=0));
    assert.equal(result.diagnostics.certificate.sheath.minForce,0);assert.deepEqual(result.diagnostics.acceptedAlphas,[1]);
    assert.equal(result.diagnostics.regularizationActivations.length,0);assert.equal(result.state.step,3551);
    for(const b of result.balances.values())b.residual.forEach(v=>close(v,0,1e-7));
    let next=result.state;
    for(let i=0;i<20;i++) {const p=prepare(next),r=advanceCompositeJointTimeStep(p.state,{...p.options,sheath});
        assert.equal(r.accepted,true,`${i}: ${r.status}`);assert.ok(r.diagnostics.certificate.sheath.minForce>=0);next=r.state;}
    assert.equal(next.step,3571);
});

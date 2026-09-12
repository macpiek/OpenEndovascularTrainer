import assert from 'node:assert/strict';
import test from 'node:test';
import { createCompositeChainLayout } from '../src/physics/kirchhoffCompositeChain.js';
import { captureCompositeReferenceFrames, compileCompositeMaterial } from '../src/physics/kirchhoffCompositeElement.js';
import { createCompositeJointTimeStepState, createCompositeJointTimeStepWorkspace, advanceCompositeJointTimeStep } from '../src/physics/kirchhoffCompositeJointTimeStep.js';

const close=(a,b,t=1e-9)=>assert.ok(Number.isFinite(a)&&Math.abs(a-b)<=t,`${a} != ${b} (tol ${t})`);
const vectorClose=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const options={dt:.06,torsionMode:'quasi-static',contacts:'none',
    tolerances:{force:1e-7,torque:1e-8,length:1e-8,boundary:1e-9,linearForce:5e-10,linearTorque:5e-10,linearConstraint:5e-11}};
const zero=[0,0,0];
function fixture({n=5,wireSlope=1.02,catheterSlope=1,angle=.07,wireVelocity=zero,catheterVelocity=zero,wireFeed=0}={}) {
    const layout=createCompositeChainLayout(Array.from({length:n-1},()=>['wire','catheter'])), coordinates=Array.from({length:n},(_,i)=>2*i);
    const positions=coordinates.map(x=>[x*catheterSlope,0,0]),wire=coordinates.map(x=>[x*wireSlope*Math.cos(angle),x*wireSlope*Math.sin(angle),.03]);
    const modes=coordinates.map((_,node)=>({node,basis:[[1,0,0],[0,1,0],[0,0,1]]}));
    const relative=Float64Array.from(wire.flatMap((p,i)=>p.map((v,k)=>v-positions[i][k]))),angles=new Map(),restLengths=new Map();
    const tools=['wire','catheter'].map(id=>{
        const dsDx=id==='wire'?wireSlope:catheterSlope;angles.set(id,new Float64Array(n-1));restLengths.set(id,new Float64Array(n-1).fill(2*dsDx));
        return {id,dsDx,reference:captureCompositeReferenceFrames(id==='wire'?wire:positions),referenceTwists:new Float64Array(n-2),
            // Declared test material parameters in consistent N/mm/s units,
            // not measured guidewire/catheter values or runtime tuning.
            material:compileCompositeMaterial({stiffness:(id==='wire'?[2,3,1]:[8,11,4]).map((v,i)=>Array.from({length:3},(_,j)=>i===j?v:0)),intrinsic:[0,0,0]})};
    });
    const state=createCompositeJointTimeStepState({layout,coordinates,positions,relative,modes,angles,tools,restLengths,materialCoordinate:'reference-arclength'});
    const inputs={...options,inertia:inertiaFor(state,{wireVelocity,catheterVelocity,wireFeed}),boundaries:{positions:[],spins:[{toolId:'wire',edge:0,value:0},{toolId:'catheter',edge:0,value:0}]}};
    return {state,inputs};
}
function inertiaFor(state,{wireVelocity,catheterVelocity,wireFeed=0}={}) {
    const byId=new Map(state.tools.map(t=>[t.id,t]));
    return {previousPositions:structuredClone(state.toolPositions),inertiaEdges:state.layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,
        massPerMaterialLength:id==='wire'?.13:.24,
        materialMap:{sStart:20+state.coordinates[e]*byId.get(id).dsDx,dsDx:byId.get(id).dsDx,dsDt:id==='wire'?wireFeed:0},
        oldMaterialVelocities:(id==='wire'?wireVelocity:catheterVelocity)!==undefined
            ?[Array.from(id==='wire'?wireVelocity:catheterVelocity),Array.from(id==='wire'?wireVelocity:catheterVelocity)]
            :(state.materialVelocities?.[e].tools.find(t=>t.id===id)?.velocities??[zero,zero]).map(v=>Array.from(v))}))}))};
}
function run(f,extra={}) {return advanceCompositeJointTimeStep(f.state,{...f.inputs,...extra});}
function accepted(result) {assert.equal(result.accepted,true,JSON.stringify({status:result.status,error:result.error,diagnostics:result.diagnostics}));}
function originalLengths(result) {
    for(const [id,p] of result.state.toolPositions)for(let e=0;e<p.length-1;e++)
        close(Math.hypot(...p[e+1].map((v,k)=>v-p[e][k])),result.state.restLengths.get(id)[e],1e-8);
}
function balance(result,t=5e-7) {for(const b of result.balances.values())vectorClose(b.momentumRate,b.appliedForce.map((v,k)=>v+b.boundaryForce[k]),t);}
function samePhysicalResult(a,b) {
    assert.equal(a.accepted,b.accepted);assert.equal(a.status,b.status);assert.equal(a.error,b.error);
    for(const k of ['state','perTool','boundaryForces','spinReactions','balances'])assert.deepEqual(a[k],b[k],k);
    for(const k of ['certificate','directions','evaluations','fullAssemblies','gradientAssemblies','linearSolves','factorizations','acceptedAlphas','retainedLengthRows'])
        assert.deepEqual(a.diagnostics[k],b.diagnostics[k],k);
}

test('unloaded full overlap preserves different physical axes, rest metrics and owned histories',()=>{
    const f=fixture(),before=structuredClone(f.state),r=run(f);accepted(r);
    assert.deepEqual(f.state,before);assert.notEqual(r.state,f.state);
    assert.equal(r.state.time,options.dt);assert.equal(r.state.step,1);assert.equal(r.diagnostics.directions,0);
    assert.equal(r.diagnostics.evaluations,2);assert.equal(r.diagnostics.relativeDofs,15);
    for(const [id,p] of f.state.toolPositions)p.forEach((v,i)=>vectorClose(r.state.toolPositions.get(id)[i],v,1e-14));
    originalLengths(r);balance(r);assert.equal(r.contacts,'none');assert.equal(r.includesAngularInertia,false);
    assert.notDeepEqual(r.state.tools[0].reference,r.state.tools[1].reference);
    close(r.perTool.get('wire').mass,.13*8*1.02,1e-14);close(r.perTool.get('catheter').mass,.24*8,1e-14);
    assert.ok(r.diagnostics.preparationMs>=0&&r.diagnostics.iterationMs>=0&&r.diagnostics.commitMs>=0);
});

test('a numerical geometry guess does not replace the incoming inertia or advance an extra dt',()=>{
    const f=fixture(),before=structuredClone(f.state),inertiaBefore=structuredClone(f.inputs.inertia),
        initialGuess={positions:f.state.positions.map(p=>p.map((v,k)=>v+(k===1?.15:0))),relative:f.state.relative.slice(),angles:structuredClone(f.state.angles)},
        saved=structuredClone(initialGuess),r=run(f,{initialGuess});
    accepted(r);assert.ok(r.diagnostics.directions>0);assert.equal(r.state.step,1);assert.equal(r.state.time,options.dt);
    for(const [id,p] of before.toolPositions)p.forEach((v,i)=>vectorClose(r.state.toolPositions.get(id)[i],v,1e-7));
    assert.deepEqual(f.state,before);assert.deepEqual(f.inputs.inertia,inertiaBefore);assert.deepEqual(initialGuess,saved);balance(r);
});

test('an unconverged numerical guess is rejected without overwriting the accepted geometry',()=>{
    const f=fixture(),before=structuredClone(f.state),initialGuess={positions:f.state.positions.map(p=>[p[0],p[1]+.2,p[2]]),relative:f.state.relative.slice(),angles:structuredClone(f.state.angles)},
        r=run(f,{initialGuess,budget:{directions:0}});
    assert.equal(r.accepted,false);assert.equal(r.status,'direction-budget');assert.deepEqual(f.state,before);
    assert.notDeepEqual(new Map(r.diagnostics.rejectedConfiguration.positions).get('catheter'),before.toolPositions.get('catheter'));
    const seed=r.diagnostics.numericalRestart,retry=run(f,{initialGuess:{...seed.initialGuess,angles:new Map(seed.initialGuess.angles)},initialWallReactions:seed.initialWallReactions});
    accepted(retry);assert.equal(retry.state.step,1);assert.equal(retry.state.time,options.dt);assert.deepEqual(f.state,before);balance(retry);
    assert.throws(()=>run(f,{initialGuess:{...initialGuess,time:7}}),/numerical initial guess/);
    assert.throws(()=>run(f,{initialGuess:{...initialGuess,relative:[]}}),/Initial guess relative/);
});

test('material measure, previous geometry, full overlap modes and explicit physical inputs reject inconsistencies',()=>{
    const f=fixture();
    assert.throws(()=>createCompositeJointTimeStepState({...f.state,modes:f.state.modes.slice(1)}),/EVERY overlap/);
    assert.throws(()=>createCompositeJointTimeStepState({...f.state,materialCoordinate:'arbitrary-label'}),/reference-arclength/);
    assert.throws(()=>createCompositeJointTimeStepState({...f.state,tools:f.state.tools.map(t=>({...t,dsDx:()=>t.dsDx}))}),/finite/);
    const inconsistent=structuredClone(f.inputs.inertia);inconsistent.inertiaEdges.forEach(e=>e.tools.find(t=>t.id==='wire').materialMap.dsDx*=2);
    assert.throws(()=>run(f,{inertia:inconsistent}),/one reference-arclength measure/);
    const differentRest=createCompositeJointTimeStepState(f.state);differentRest.restLengths.get('wire')[1]+=.001;
    assert.throws(()=>advanceCompositeJointTimeStep(differentRest,f.inputs),/one reference-arclength measure/);
    const previous=structuredClone(f.inputs.inertia);previous.previousPositions.get('wire')[1][1]+=.001;
    assert.throws(()=>run(f,{inertia:previous}),/SAME chart/);
    assert.throws(()=>run(f,{contacts:undefined}),/Explicit/);
    assert.throws(()=>run(f,{boundaries:{positions:[],spins:[]}}),/spin boundary/);
});

test('independent free rigid material velocities retain momentum with no artificial catheter drag',()=>{
    const w=[.09,-.03,.02],c=[-.04,.01,.025],f=fixture({wireVelocity:w,catheterVelocity:c}),r=run(f);accepted(r);
    for(const [id,p] of f.state.toolPositions) {
        const v=id==='wire'?w:c;
        p.forEach((old,i)=>vectorClose(r.state.toolPositions.get(id)[i],old.map((x,k)=>x+options.dt*v[k]),2e-10));
        vectorClose(r.perTool.get(id).momentumIncrement,zero,1e-9);
    }
    for(const e of r.state.materialVelocities)for(const t of e.tools)t.velocities.forEach(v=>vectorClose(v,t.id==='wire'?w:c,4e-9));
    originalLengths(r);balance(r);
});

test('wire endpoint feed and rotation with every catheter position fixed remain independent',()=>{
    const f=fixture({angle:0}),cat=f.state.toolPositions.get('catheter'),wire=f.state.toolPositions.get('wire'),feed=.002;
    const boundaries={positions:cat.map((value,node)=>({toolId:'catheter',node,value})).concat([{toolId:'wire',node:0,value:wire[0].map((v,k)=>v+(k===0?feed:0))}]),
        spins:[{toolId:'catheter',edge:0,value:0},{toolId:'wire',edge:0,value:.4}]};
    const r=run(f,{boundaries});accepted(r);
    cat.forEach((p,i)=>assert.deepEqual(r.state.toolPositions.get('catheter')[i],p));
    wire.forEach((p,i)=>vectorClose(r.state.toolPositions.get('wire')[i],p.map((v,k)=>v+(k===0?feed:0)),3e-10));
    r.state.angles.get('wire').forEach(a=>close(a,.4,2e-9));r.state.angles.get('catheter').forEach(a=>close(a,0,2e-12));
    const expected=r.perTool.get('wire').mass*feed/options.dt**2;
    close(r.boundaryForces.get('wire')[0][0],expected,3e-8);
    r.boundaryForces.get('catheter').forEach(v=>vectorClose(v,zero,3e-8));
    assert.equal(r.diagnostics.suppressedPrescribedLengthRows.length,4);
    for(const e of r.state.materialVelocities)for(const t of e.tools)t.velocities.forEach(v=>vectorClose(v,t.id==='wire'?[feed/options.dt,0,0]:zero,6e-9));
    originalLengths(r);balance(r);
});

test('through-chart feed is counted once in accepted material velocity on each own physical tangent',()=>{
    const feed=-.2,angle=.07,wireVelocity=[.2*Math.cos(angle),.2*Math.sin(angle),0],f=fixture({wireFeed:feed,wireVelocity,angle}),r=run(f);accepted(r);
    assert.equal(r.diagnostics.directions,0);
    for(const [id,p] of f.state.toolPositions)p.forEach((v,i)=>vectorClose(r.state.toolPositions.get(id)[i],v,1e-14));
    for(const e of r.state.materialVelocities)for(const t of e.tools)t.velocities.forEach(v=>vectorClose(v,t.id==='wire'?wireVelocity:zero,1e-13));
    balance(r);
});

test('all physical positions prescribed remove only dependent edge duals and retain both original force balances',()=>{
    const f=fixture({angle:0}),boundaries={positions:[],spins:f.inputs.boundaries.spins};
    for(const [id,p] of f.state.toolPositions)p.forEach((v,node)=>boundaries.positions.push({toolId:id,node,value:v.map((x,k)=>x+(k===0?(id==='wire'?.001:-.002):0))}));
    const r=run(f,{boundaries});accepted(r);assert.equal(r.diagnostics.retainedLengthRows,0);assert.equal(r.diagnostics.originalLengthRows,8);
    assert.equal(r.state.lengthMultipliers.every(v=>v===0),true);originalLengths(r);balance(r);
    close(r.balances.get('wire').boundaryForce[0],r.perTool.get('wire').mass*.001/options.dt**2,2e-8);
    close(r.balances.get('catheter').boundaryForce[0],-r.perTool.get('catheter').mass*.002/options.dt**2,2e-8);
    const bad=structuredClone(boundaries);bad.positions[1].value[0]+=.01;
    assert.throws(()=>run(f,{boundaries:bad}),/conflict/);
});

test('nonlinear bending solves both metrics, updates each time frame and conserves per-material momentum with separate supports',()=>{
    const f=fixture(),boundaries={positions:[...f.state.toolPositions].map(([toolId,p])=>({toolId,node:0,value:p[0]})),spins:f.inputs.boundaries.spins};
    const loads={forces:[{toolId:'wire',node:4,value:[.01,.025,-.012]},{toolId:'catheter',node:3,value:[-.008,.016,.007]}],torques:[{toolId:'wire',edge:3,value:.001}]};
    const r=run(f,{boundaries,loads});accepted(r);assert.ok(r.diagnostics.directions>=2);
    assert.ok(Math.abs(r.state.toolPositions.get('wire')[4][1]-f.state.toolPositions.get('wire')[4][1])>1e-5);
    originalLengths(r);balance(r);
    for(const t of r.state.tools) {
        const p=r.state.toolPositions.get(t.id);
        t.reference.forEach((frame,e)=>{const delta=p[e+1].map((v,k)=>v-p[e][k]),len=Math.hypot(...delta);vectorClose(frame.tangent,delta.map(v=>v/len),2e-13);close(frame.director.reduce((s,v,k)=>s+v*frame.tangent[k],0),0,2e-13);});
        assert.ok(t.referenceTwists.every(Number.isFinite));
    }
    const next=advanceCompositeJointTimeStep(r.state,{...f.inputs,boundaries,loads,inertia:inertiaFor(r.state)});accepted(next);assert.equal(next.state.step,2);
});

test('rejected nonlinear attempt owns no commits and identical retry is deterministic; accepted-only time advances once',()=>{
    const f=fixture({wireVelocity:[.1,0,0]}),before=structuredClone(f.state),inputBefore=structuredClone(f.inputs);
    const limited={budget:{directions:0,evaluations:20}},a=run(f,limited),b=run(f,limited);
    assert.equal(a.accepted,false);assert.equal(a.status,'direction-budget');assert.equal(a.state,f.state);assert.equal(b.state,f.state);
    assert.deepEqual(a.diagnostics.certificate,b.diagnostics.certificate);assert.deepEqual(f.state,before);assert.deepEqual(f.inputs,inputBefore);
    const acceptedA=run(f),acceptedB=run(f);accepted(acceptedA);accepted(acceptedB);assert.deepEqual(acceptedA.state,acceptedB.state);
    assert.equal(acceptedA.state.time,options.dt);assert.equal(acceptedA.state.step,1);
    const noCommitBudget=run(f,{budget:{evaluations:1,directions:0}});assert.equal(noCommitBudget.accepted,false);assert.equal(noCommitBudget.state,f.state);
});

test('lazy gradient trials and full fresh tangent policy agree on nonlinear physical response',()=>{
    const f=fixture(),loads={forces:[{toolId:'wire',node:4,value:[0,.008,.002]}]},a=run(f,{loads}),b=run(f,{loads,assemblyPolicy:'full'});accepted(a);accepted(b);
    for(const [id,p] of a.state.toolPositions)p.forEach((v,i)=>vectorClose(v,b.state.toolPositions.get(id)[i],1e-13));
    assert.deepEqual(a.diagnostics.certificate,b.diagnostics.certificate);assert.ok(a.diagnostics.gradientAssemblies>0);assert.equal(b.diagnostics.gradientAssemblies,0);
    assert.ok(a.diagnostics.fullAssemblies<b.diagnostics.fullAssemblies);
});

test('independent material frame winding stays continuous across two commits and a late rejected commit owns nothing',()=>{
    const f=fixture();
    for(const t of f.state.tools) {
        const winding=t.id==='wire'?4*Math.PI:-2*Math.PI;
        t.referenceTwists.fill(winding);f.state.angles.get(t.id).forEach((_,e)=>f.state.angles.get(t.id)[e]=-e*winding);
    }
    const before=structuredClone(f.state),denied=run(f,{budget:{evaluations:1}});
    assert.equal(denied.accepted,false);assert.equal(denied.status,'evaluation-budget');assert.equal(denied.state,f.state);assert.deepEqual(f.state,before);
    const a=run(f);accepted(a);
    const b=advanceCompositeJointTimeStep(a.state,{...f.inputs,inertia:inertiaFor(a.state)});accepted(b);
    for(const state of [a.state,b.state])for(const t of state.tools)t.referenceTwists.forEach(v=>close(v,t.id==='wire'?4*Math.PI:-2*Math.PI,2e-13));
    assert.equal(b.state.time,2*options.dt);assert.equal(b.state.step,2);
});

test('complete rotated relative coordinates produce the same nonlinear physical motion and individual reactions',()=>{
    const f=fixture(),base=f.state,basis=[[.8,.6,0],[-.6,.8,0],[0,0,1]],relative=[];
    for(const m of base.modes) {
        const displacement=base.toolPositions.get('wire')[m.node].map((v,k)=>v-base.positions[m.node][k]);
        for(const b of basis)relative.push(b.reduce((s,v,k)=>s+v*displacement[k],0));
    }
    const rotated=createCompositeJointTimeStepState({...base,modes:base.modes.map(m=>({...m,basis})),relative});
    const loads={forces:[{toolId:'wire',node:4,value:[.008,.016,-.007]}]},a=run(f,{loads});
    const b=advanceCompositeJointTimeStep(rotated,{...f.inputs,loads,inertia:inertiaFor(rotated)});accepted(a);accepted(b);
    for(const [id,p] of a.state.toolPositions)p.forEach((v,i)=>vectorClose(v,b.state.toolPositions.get(id)[i],3e-11));
    balance(a);balance(b);
});

test('catheter tip inside a longer wire retains all overlap endpoint modes and separate exposed material history',()=>{
    const f=fixture({wireSlope:1,angle:0,n:6}),layout=createCompositeChainLayout([['wire','catheter'],['wire','catheter'],['wire','catheter'],['wire'],['wire']]);
    const positions=f.state.positions.map((p,i)=>i>3?f.state.toolPositions.get('wire')[i]:p);
    const s=createCompositeJointTimeStepState({...f.state,layout,positions,modes:f.state.modes.slice(0,4),relative:f.state.relative.slice(0,12),lengthMultipliers:null});
    const inputs={...f.inputs,inertia:inertiaFor(s),loads:{forces:[{toolId:'wire',node:5,value:[.012,.009,.001]}]},boundaries:{positions:[],spins:f.inputs.boundaries.spins}};
    const r=advanceCompositeJointTimeStep(s,inputs);accepted(r);balance(r);
    assert.equal(r.state.relative.length,12);assert.equal(r.state.modes.at(-1).node,3);
    assert.equal(r.state.tools.find(t=>t.id==='catheter').reference[3],null);
    for(const [id,p] of r.state.toolPositions)for(let e=0;e<layout.nodeCount-1;e++)if(layout.spins.get(id)[e]>=0)
        close(Math.hypot(...p[e+1].map((v,k)=>v-p[e][k])),r.state.restLengths.get(id)[e],1e-8);
});

test('GN and exact backends use the same original force gates; a rejected deformed trial cannot alter retry history',()=>{
    const f=fixture(),loads={forces:[{toolId:'wire',node:4,value:[.01,.025,-.012]}]},before=structuredClone(f.state);
    const rejected=run(f,{loads,budget:{directions:1}});
    assert.equal(rejected.accepted,false);assert.equal(rejected.status,'direction-budget');assert.equal(rejected.diagnostics.directions,1);
    assert.equal(rejected.state,f.state);assert.deepEqual(f.state,before);
    // The rejection diagnostic must describe the deformed private trial,
    // not the cached physical geometry created at the beginning of the step.
    const trial=new Map(rejected.diagnostics.rejectedConfiguration.positions);
    assert.ok([...trial].some(([id,p])=>p.some((v,i)=>v.some((x,k)=>x!==before.toolPositions.get(id)[i][k]))));
    const exact=run(f,{loads}),gn=run(f,{loads,elementBackend:'wasm'}),js=run(f,{loads,elementBackend:'javascript'});
    accepted(exact);accepted(gn);accepted(js);originalLengths(exact);originalLengths(gn);originalLengths(js);
    for(const r of [exact,gn,js]) {assert.ok(r.diagnostics.certificate.force<=options.tolerances.force);balance(r);}
    for(const [id,p] of exact.state.toolPositions)p.forEach((v,i)=>vectorClose(v,gn.state.toolPositions.get(id)[i],1e-8));
});

test('reused timestep structures preserve exact cold states, nonzero reaction history and changed profiles/loads/maps',()=>{
    const f=fixture(),workspace=createCompositeJointTimeStepWorkspace(f.state),initial=structuredClone(f.state);
    let state=f.state;
    for(let step=0;step<4;step++) {
        if(step===2) {
            state=createCompositeJointTimeStepState(state);
            state.tools[0].material.stiffness[0]*=1.19;state.tools[0].material.intrinsic[1]+=.0002;state.tools[1].material.energyOffset=.001;
            // This is current input, not workspace history. Nonzero bilateral
            // forces must be retained as seeds even when profiles change.
            state.lengthMultipliers[2]=.003;
        }
        const input={...f.inputs,inertia:inertiaFor(state),
            boundaries:{positions:[...state.toolPositions].map(([toolId,p])=>({toolId,node:0,value:p[0]})),spins:f.inputs.boundaries.spins},
            loads:{forces:[{toolId:'wire',node:4,value:[.003,.009+step*.001,-.002]}]}};
        if(step===3)for(const e of input.inertia.inertiaEdges)for(const t of e.tools) {
            t.massPerMaterialLength*=1.07;t.materialMap.dsDt=t.id==='wire'?-.001:0;t.oldMaterialVelocities[0][1]+=.0003;
        }
        const cold=advanceCompositeJointTimeStep(state,input),warm=advanceCompositeJointTimeStep(state,{...input,workspace});accepted(cold);accepted(warm);samePhysicalResult(warm,cold);
        state=warm.state;
    }
    assert.deepEqual(f.state,initial);assert.equal(workspace.diagnostics.assembly.structureBuilds,1);assert.equal(workspace.diagnostics.lengthBuilds,1);
    assert.equal(workspace.diagnostics.directionBuilds,1);assert.equal(workspace.diagnostics.directionHits,3);
    assert.ok(state.lengthMultipliers.some(v=>Math.abs(v)>1e-6));
});

test('bounded row-pattern LRU refreshes targets/fixed masks and preserves independent physical BC reactions',()=>{
    const f=fixture({angle:0}),workspace=createCompositeJointTimeStepWorkspace({...f.state,rowCacheCapacity:2});
    const allCat=f.state.toolPositions.get('catheter').map((value,node)=>({toolId:'catheter',node,value}));
    const patterns=[[],allCat.slice(0,2),allCat,[],allCat];
    for(let i=0;i<patterns.length;i++) {
        const boundary={positions:[...patterns[i],{toolId:'wire',node:0,value:f.state.toolPositions.get('wire')[0].map((v,k)=>v+(k===0?.0001*(i+1):0))}],
            spins:[{toolId:'wire',edge:0,value:.04*i},{toolId:'catheter',edge:0,value:0}]};
        const input={...f.inputs,boundaries:boundary},cold=run(f,{boundaries:boundary}),warm=run(f,{boundaries:boundary,workspace});
        accepted(cold);accepted(warm);samePhysicalResult(warm,cold);assert.ok(workspace.diagnostics.retainedRowPatterns<=2);
    }
    assert.equal(workspace.diagnostics.directionBuilds,4);assert.equal(workspace.diagnostics.directionHits,1);
    assert.throws(()=>createCompositeJointTimeStepWorkspace({...f.state,rowCacheCapacity:5}),/1..4/);
});

test('workspace rejects structural/reentrant inputs and resets after early errors or rejected deformed/late-commit trials',()=>{
    const f=fixture(),workspace=createCompositeJointTimeStepWorkspace(f.state),before=structuredClone(f.state);
    const loads={forces:[{toolId:'wire',node:4,value:[.01,.025,-.012]}]};
    for(const budget of [{directions:0},{directions:1},{evaluations:1}]) {
        const cold=run(f,{loads,budget}),warm=run(f,{loads,budget,workspace});assert.equal(warm.accepted,false);samePhysicalResult(warm,cold);
        assert.equal(warm.state,f.state);assert.deepEqual(f.state,before);
    }
    const malformed=structuredClone(f.inputs.inertia);malformed.inertiaEdges[0].tools[0].materialMap.dsDx*=2;
    assert.throws(()=>run(f,{workspace,inertia:malformed}),/reference-arclength/);
    const changed=fixture();changed.state.coordinates[2]+=.01;
    assert.throws(()=>advanceCompositeJointTimeStep(changed.state,{...changed.inputs,workspace}),/reference-arclength|Frozen/);
    const callback=f.state.tools[0].materialAt;
    f.state.tools[0].materialAt=()=>run(f,{workspace});assert.throws(()=>run(f,{workspace}),/busy/);f.state.tools[0].materialAt=callback;
    const recovered=run(f,{loads,workspace}),cold=run(f,{loads});accepted(recovered);samePhysicalResult(recovered,cold);
    const saved=structuredClone(recovered.state);run(f,{workspace});assert.deepEqual(recovered.state,saved,'accepted output cannot alias reusable scratch');
});

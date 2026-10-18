import test from 'node:test';
import assert from 'node:assert/strict';
import {PlaneGeometry} from 'three';
import {createSharedAxisNative,feedSharedAxisNative,extendSharedAxisNativeRows} from '../src/physics/kirchhoffSharedAxisNative.js';
import {createSharedAxisContacts} from '../src/physics/kirchhoffSharedAxisContacts.js';
import {createSharedAxisVesselWitness} from '../src/physics/kirchhoffSharedAxisVesselWitnesses.js';
import {prepareSharedAxisDynamicStep} from '../src/physics/kirchhoffSharedAxisDynamics.js';
import {stepSharedAxis} from '../src/physics/kirchhoffSharedAxisTimeStep.js';
import {adaptiveMeshOptions,coarsenSharedAxisMesh,retainAdaptiveShapeSamples,ADAPTIVE_SOLVE_OPTIONS,CONTACT_PROTECTED_MESH} from '../src/physics/kirchhoffSharedAxisAdaptiveMesh.js';
import {createSharedAxisAppSystem} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {captureSharedAxisReplay,restoreSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {createCoupledSolverSelection,resolveAppCoupledSolver} from '../src/physics/coupledSolverSelection.js';

const tools=[{id:'wire',insertion:300,type:'glidewire'},{id:'catheter',insertion:240,type:'straight'}];
const make=(adaptiveMesh=true,extra={})=>createSharedAxisNative({tools,startCoordinate:-40,boundaryCoordinates:[0,20],adaptiveMesh,...extra});

test('adaptive grid reduces straight shaft DOFs, preserves boundaries, tips and total mass',()=>{
    const fine=make(false),adaptive=make();
    assert.ok(adaptive.layout.dofCount<fine.layout.dofCount*.7,`${adaptive.layout.dofCount}/${fine.layout.dofCount}`);
    for(const x of [-40,-35,0,20,240,300])assert.ok(adaptive.coordinates.includes(x),`Missing ${x}`);
    const lengths=adaptive.coordinates.slice(1).map((x,i)=>x-adaptive.coordinates[i]);
    assert.ok(lengths.includes(20));assert.ok(Math.max(...lengths)<=20);
    assert.deepEqual(adaptive.coordinates.filter(x=>x>=270),fine.coordinates.filter(x=>x>=270));
    for(const s of [fine,adaptive])prepareSharedAxisDynamicStep(s,1/60);
    const mass=s=>s.dynamicStep.masses.reduce((a,b)=>a+b,0);
    assert.ok(Math.abs(mass(fine)-mass(adaptive))<1e-10);
});

test('curvature refines the mesh and straightening permits coarsening again',()=>{
    const xs=Array.from({length:61},(_,i)=>i*5),options=adaptiveMeshOptions(true);
    const run=positions=>{const x=xs.slice();coarsenSharedAxisMesh(x,positions,{tools:[],boundaries:[0,300],spacing:5,options});return x;};
    const straight=run(xs.map(x=>[x,0,0]));
    const curved=run(xs.map(x=>[20*Math.sin(x/20),20*(1-Math.cos(x/20)),0]));
    assert.ok(curved.length>straight.length*2);
    assert.ok(curved.every((x,i)=>i===0||x-curved[i-1]===5));
});

test('optional fine prefix keeps the original proximal grid while the long shaft coarsens',()=>{
    const extra={tools:tools.map(t=>({...t,insertion:t.id==='wire'?900:850}))};
    const fine=make(false,extra),adaptive=make({finePrefixLength:250,tipLength:80,tipRefinementAhead:80},extra);
    assert.deepEqual(adaptive.coordinates.filter(x=>x<=255),fine.coordinates.filter(x=>x<=255));
    assert.ok(adaptive.coordinates.some((x,i)=>i>0&&x>255&&x-adaptive.coordinates[i-1]===20));
    assert.deepEqual(adaptive.coordinates.filter(x=>x>=770),fine.coordinates.filter(x=>x>=770));
    assert.equal(adaptiveMeshOptions({finePrefixLength:0}).finePrefixLength,0);
    assert.throws(()=>adaptiveMeshOptions({finePrefixLength:-1}),/Invalid/);
});

test('fine prefix discards obsolete shape knots but retains required cloth contact bends',()=>{
    const options=adaptiveMeshOptions({finePrefixLength:250,shapeTolerance:.01});
    const previous={spacing:5,coordinates:[0,5,7.5,10,15],positions:[[0,0,0],[5,0,0],[7.5,.1,0],[10,0,0],[15,0,0]],wallSamples:[]};
    const run=()=>{const xs=[0,5,10,15],ps=xs.map(x=>[x,0,0]);retainAdaptiveShapeSamples(xs,ps,previous,options);return xs;};
    assert.deepEqual(run(),[0,5,10,15]);
    previous.wallSamples=[{adaptiveContactKnots:()=>new Set([7.5])}];
    assert.deepEqual(run(),[0,5,7.5,10,15]);
});

test('zero contact margin frees near-wall nodes but preserves touching and loaded contacts',()=>{
    const xs=Array.from({length:61},(_,i)=>i*5),ps=xs.map(x=>[x,0,0]);
    const run=(contactMargin,gap=.5,load=0)=>{
        const coordinates=xs.slice(),positions=ps.map(p=>p.slice());
        const previous={coordinates:xs,positions:ps,definitions:[{id:'contact',kind:'wall',edge:20}],
            multipliers:[load],acceptedWallGaps:new Map([['contact',gap]])};
        coarsenSharedAxisMesh(coordinates,positions,{tools:[],boundaries:[0,300],previous,spacing:5,
            options:adaptiveMeshOptions({contactMargin})});
        return coordinates;
    };
    const near=run(1),released=run(0);
    assert.ok(released.length<near.length,'reducing the margin actually removes near-wall nodes');
    for(const coordinate of [95,100,105,110]) {
        assert.ok(near.includes(coordinate));
        assert.ok(run(0,0).includes(coordinate),'touching wall remains protected');
        assert.ok(run(0,-.01).includes(coordinate),'penetration remains protected');
        assert.ok(run(0,.5,2).includes(coordinate),'loaded contact remains protected regardless of gap');
    }
    assert.equal(adaptiveMeshOptions({contactMargin:0}).contactMargin,0);
    assert.throws(()=>adaptiveMeshOptions({contactMargin:-.01}),/Invalid/);
});

test('arc-loss and maximum-spacing budgets independently control coarsening',()=>{
    const xs=Array.from({length:61},(_,i)=>i*5);
    const run=(curve,extra)=>{
        const coordinates=xs.slice(),positions=xs.map(curve);
        coarsenSharedAxisMesh(coordinates,positions,{tools:[],boundaries:[0,300],spacing:5,
            options:adaptiveMeshOptions({shapeTolerance:50,...extra})});
        return coordinates;
    };
    const curve=x=>[100*Math.sin(x/100),100*(1-Math.cos(x/100)),0];
    assert.ok(run(curve,{maxArcLoss:.01}).length<run(curve,{maxArcLoss:0}).length,
        'a larger arc-loss budget removes more curved nodes');
    const straight=x=>[x,0,0],long=run(straight,{maxSpacing:100});
    assert.ok(long.length<run(straight,{maxSpacing:20}).length,
        'a larger spacing budget permits a coarser straight shaft');
    assert.ok(long.slice(1).every((x,i)=>x-long[i]<=100));
    assert.equal(adaptiveMeshOptions({maxArcLoss:0}).maxArcLoss,0);
    assert.throws(()=>adaptiveMeshOptions({maxArcLoss:-.01}),/Invalid/);
    assert.throws(()=>adaptiveMeshOptions({maxArcLoss:1}),/Invalid/);
});

test('feed and withdrawal do not accumulate old moving-tip knots; histories and accepted state remain owned',()=>{
    let s=make();const initial=s.coordinates.length;
    s.wallFrictionHistory=[{id:'saved',elastic:[.01,0,0]}];
    s.velocities=s.coordinates.map(()=>[1,2,3]);
    s.angularVelocities=Object.fromEntries(s.materials.map(m=>[m.spec.id,Array.from({length:m.last},()=>[0,0,.1])]));
    const original=JSON.stringify(s.positions);
    const before=s;
    for(let i=1;i<=100;i++) {
        const delta=i<=50?i*.7:(100-i)*.7;
        s=feedSharedAxisNative(s,{wire:300+delta,catheter:240+delta/2});
        assert.ok(s.coordinates.length<initial+15,`${i}: ${s.coordinates.length} vs ${initial}`);
        assert.ok(s.velocities.every(v=>v.every((x,k)=>Math.abs(x-(k+1))<1e-12)));
        for(const m of s.materials)assert.equal(s.angularVelocities[m.spec.id].length,m.last);
        assert.equal(s.wallFrictionHistory,before.wallFrictionHistory);
    }
    assert.equal(JSON.stringify(before.positions),original);
    assert.deepEqual(s.materials.map(m=>m.coordinates.at(-1)),[300,240]);
});

test('refining a loaded coarse contact preserves its physical site, reaction and friction history',()=>{
    const s=make(),edge=s.coordinates.findIndex((x,i)=>x>20&&s.coordinates[i+1]-x===20);
    assert.ok(edge>=0);
    const geometry=new PlaneGeometry(2000,2000),field={fallbackGeometry:geometry};
    const a=s.coordinates[edge],b=s.coordinates[edge+1],t=.37;
    const row=createSharedAxisVesselWitness(field,{id:'loaded',kind:'wall',edge,witness:{face:0,t},
        dofs:[s.layout.positions[edge],s.layout.positions[edge+1]].flatMap(p=>[p,p+1,p+2])});
    extendSharedAxisNativeRows(s,[row]);s.multipliers[s.multipliers.length-1]=2;
    const site=a+(b-a)*t;
    s.wallFrictionHistory=[{id:'loaded',siteCoordinate:site,owner:'catheter',face:0,elastic:[.01,0,0]}];
    const next=feedSharedAxisNative(s,{wire:300.7,catheter:240.4});
    const index=next.definitions.findIndex(r=>r.witness?.face===0);
    const mapped=next.definitions[index];assert.ok(mapped);
    const ca=next.coordinates[mapped.edge],cb=next.coordinates[mapped.edge+1];
    assert.ok(cb-ca<=5);assert.ok(Math.abs(ca+(cb-ca)*mapped.witness.t-site)<1e-12);
    assert.equal(next.multipliers[index],2);assert.equal(next.wallFrictionHistory,s.wallFrictionHistory);
    const input={a:[0,0,2],b:[5,0,2],radius:1,state:next};
    const evaluation=mapped.evaluate(input);
    assert.ok(Math.abs(evaluation.jacobian[2]/(evaluation.jacobian[2]+evaluation.jacobian[5])-(1-mapped.witness.t))<1e-12);
    geometry.dispose();
});

test('adaptive replay restores the exact topology, pose and future adaptation policy',()=>{
    const sheath={start:[10,20,30],end:[30,20,30],innerRadius:2,proximalExtension:40};
    const s=createSharedAxisNative({...createSharedAxisContacts({sheath,localCoordinates:true}),tools,adaptiveMesh:true});
    const saved=JSON.parse(JSON.stringify(captureSharedAxisReplay(s,sheath)));
    const restored=restoreSharedAxisReplay(saved,null);
    assert.deepEqual(captureSharedAxisReplay(restored,sheath),saved);
    assert.deepEqual(feedSharedAxisNative(restored,{wire:301}).coordinates,feedSharedAxisNative(s,{wire:301}).coordinates);
});

test('near-wall tip sites do not leave a permanent submillimetre mesh trail',()=>{
    let s=make();const geometry=new PlaneGeometry(2000,2000),field={fallbackGeometry:geometry};
    for(let i=1;i<=80;i++) {
        const edge=s.materials[1].last-1;
        const row=createSharedAxisVesselWitness(field,{id:`tip/${i}`,kind:'wall',edge,witness:{face:0,t:1},
            dofs:[s.layout.positions[edge],s.layout.positions[edge+1]].flatMap(p=>[p,p+1,p+2])});
        extendSharedAxisNativeRows(s,[row]);s.acceptedWallGaps=new Map([[row.id,.2]]);
        s=feedSharedAxisNative(s,{wire:300+i*.7,catheter:240+i*.7});
        assert.ok(s.coordinates.length<80,`Moving contact trail: ${s.coordinates.length}`);
        assert.ok(s.definitions.filter(r=>r.witness).length<4,'Unloaded old sites must be rediscovered');
    }
    geometry.dispose();
});

test('adaptive dynamics conserves a free translation and accepts finite state within its residual budget',()=>{
    const s=make(true,{tools:[tools[0]]});s.fixed.fill(0);s.velocities=s.coordinates.map(()=>[1,2,3]);
    const before=s.positions.map(p=>p.slice()),dt=1/60;
    const r=stepSharedAxis(s,dt,ADAPTIVE_SOLVE_OPTIONS);
    assert.ok(r.converged,JSON.stringify(r));assert.equal(r.quality.finite,true);
    // Straight material has no shape force; all grid nodes follow the same motion.
    for(let i=0;i<s.positions.length;i++)for(let k=0;k<3;k++)
        assert.ok(Math.abs(s.positions[i][k]-before[i][k]-(k+1)*dt*.98**2)<1e-6);
});

test('adaptive solver is opt-in; reference remains the default whole-step provider',()=>{
    assert.equal(resolveAppCoupledSolver(''),'shared-axis');
    const system=createSharedAxisAppSystem({readTools:()=>[],readSheath:()=>null,adaptiveMesh:true});
    assert.equal(system.id,'shared-axis-adaptive');
    const selected=createCoupledSolverSelection(resolveAppCoupledSolver('?coupledSolver=shared-axis-adaptive'),{wholeStepSystem:system});
    assert.equal(selected.wholeStepSystem,system);assert.equal(selected.coupledSystem,null);
    assert.throws(()=>adaptiveMeshOptions({shapeTolerance:-1}),/budget/);
});

test('optional coarsening near contacts keeps the loaded physical site and reaction on the coarser element',()=>{
    const s=make(false),edge=s.coordinates.indexOf(100),site=102;
    s.adaptiveMesh=adaptiveMeshOptions({contactMaxSpacing:10});
    const geometry=new PlaneGeometry(2000,2000),field={fallbackGeometry:geometry};
    const row=createSharedAxisVesselWitness(field,{id:'loaded-coarsening',kind:'wall',edge,witness:{face:0,t:.4},
        dofs:[s.layout.positions[edge],s.layout.positions[edge+1]].flatMap(p=>[p,p+1,p+2])});
    extendSharedAxisNativeRows(s,[row]);s.multipliers[s.multipliers.length-1]=2;
    s.acceptedWallGaps=new Map([[row.id,0]]);
    s.wallFrictionHistory=[{id:row.id,siteCoordinate:site,owner:'catheter',face:0,elastic:[.01,0,0]}];
    const next=feedSharedAxisNative(s,{wire:300.7,catheter:240.4},{pruneInactiveWitnesses:true});
    const i=next.definitions.findIndex(r=>r.witness?.face===0),mapped=next.definitions[i];assert.ok(mapped);
    const a=next.coordinates[mapped.edge],b=next.coordinates[mapped.edge+1];
    assert.ok(b-a>5&&b-a<=10);assert.ok(Math.abs(a+(b-a)*mapped.witness.t-site)<1e-12);
    assert.equal(next.multipliers[i],2);assert.equal(next.wallFrictionHistory,s.wallFrictionHistory);
    assert.ok(next.coordinates.length<s.coordinates.length*.8);
    assert.throws(()=>adaptiveMeshOptions({contactMaxSpacing:0}),/Invalid/);
    geometry.dispose();
});

test('ahead refinement retains fine mechanics before an approaching catheter without densifying the whole shaft',()=>{
    const tools=[{id:'wire',insertion:500,type:'glidewire'},{id:'catheter',insertion:200,type:'straight'}];
    const create=tipRefinementAhead=>createSharedAxisNative({tools,startCoordinate:-40,
        adaptiveMesh:{contactMaxSpacing:10,tipRefinementAhead}});
    const ordinary=create(0),ahead=create(40);
    for(let x=200;x<=240;x+=5)assert.ok(ahead.coordinates.includes(x),`${x} mm ahead of catheter`);
    assert.ok(ahead.coordinates.length>ordinary.coordinates.length);
    assert.ok(ahead.coordinates.slice(1).some((x,i)=>x-ahead.coordinates[i]===20),'The distant shaft remains coarse');
    assert.throws(()=>create(-1),/budget/);
});

test('contact-protected shaft approximation retains the original fine spacing at a loaded wall site and ahead of the tip',()=>{
    for(const spacing of [2,5]) {
        const fine=make(false,{spacing}),edge=fine.coordinates.indexOf(100);
        const geometry=new PlaneGeometry(2000,2000),field={fallbackGeometry:geometry};
        try {
            const row=createSharedAxisVesselWitness(field,{id:'loaded-protected',kind:'wall',edge,witness:{face:0,t:.4},
                dofs:[fine.layout.positions[edge],fine.layout.positions[edge+1]].flatMap(p=>[p,p+1,p+2])});
            extendSharedAxisNativeRows(fine,[row]);fine.multipliers[fine.multipliers.length-1]=2;
            fine.acceptedWallGaps=new Map([[row.id,0]]);
            const next=feedSharedAxisNative({...fine,adaptiveMesh:CONTACT_PROTECTED_MESH},{wire:300.7,catheter:240.4});
            for(let x=100-spacing;x<=100+2*spacing;x+=spacing)assert.ok(next.coordinates.includes(x),`Loaded region ${x}/${spacing}`);
            for(let x=Math.ceil(240.4/spacing)*spacing;x<=280.4;x+=spacing)assert.ok(next.coordinates.includes(x),`Ahead of tip ${x}/${spacing}`);
            assert.ok(next.coordinates.length<fine.coordinates.length,'Only the distant shaft may be coarsened');
            const mapped=next.definitions.findIndex(r=>r.id===row.id);
            assert.ok(mapped>=0);assert.equal(next.multipliers[mapped],2,'The same wall equation keeps its reaction');
        } finally {geometry.dispose();}
    }
});

test('adaptive shaft keeps the reference grid around curved moving tips instead of accumulating short cells',()=>{
    const tools=[{id:'wire',insertion:300.2,type:'glidewire'},{id:'catheter',insertion:240.6666666667,type:'pigtail'}];
    const options=adaptiveMeshOptions({...CONTACT_PROTECTED_MESH,shapeTolerance:.005,tipLength:80,tipRefinementAhead:80});
    const fine=createSharedAxisNative({tools,startCoordinate:-40,samplePosition:x=>[x,10*Math.sin(x/20),0]});
    let reference=fine,reduced={...fine,adaptiveMesh:options};
    for(let step=1;step<=20;step++) {
        const offset=step<=10?-step*.5:-(20-step)*.5;
        const targets={wire:300.2+offset,catheter:240.6666666667+offset};
        reference=feedSharedAxisNative(reference,targets);reduced=feedSharedAxisNative(reduced,targets);
        const first=targets.catheter-options.tipLength;
        assert.deepEqual(reduced.coordinates.filter(x=>x>=first),reference.coordinates.filter(x=>x>=first),`Tip topology at feed ${step}`);
        for(const m of reduced.materials) {
            const r=reference.materials.find(t=>t.spec.id===m.spec.id);
            for(let i=0;i<m.coordinates.length-1;i++)if(m.coordinates[i]>=first) {
                const j=r.coordinates.indexOf(m.coordinates[i]);assert.ok(j>=0);
                assert.equal(m.body.restLength[i],r.body.restLength[j]);
            }
        }
    }
});


test('fine tip grid still retains physical cloth bends and distant adaptive shaft samples',()=>{
    const tools=[{id:'catheter',insertion:240,type:'pigtail'}];
    const options=adaptiveMeshOptions({...CONTACT_PROTECTED_MESH,tipLength:80});
    const previous={coordinates:[0,2,5,235,237,240],positions:[[0,0,0],[2,.1,0],[5,0,0],[235,0,0],[237,.1,0],[240,0,0]],
        wallSamples:[{adaptiveContactKnots:()=>[237]}]};
    const saved=structuredClone(previous.positions),coordinates=[0,5,235,240],positions=coordinates.map(x=>[x,0,0]);
    retainAdaptiveShapeSamples(coordinates,positions,previous,options,{tools});
    assert.ok(coordinates.includes(2),'Distant shaft bend keeps its shape sample');
    assert.ok(coordinates.includes(237),'A physical cloth bend is not dropped by the tip-grid rule');
    assert.deepEqual(previous.positions,saved);
});

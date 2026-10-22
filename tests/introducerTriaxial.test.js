import {introducerDimensions,INTRODUCER_SIZES_F,prepareIntroducerExchange} from '../src/devices/introducerSheath.js';
import {INTRODUCER_SHEATH_RADIUS_MM,INTRODUCER_SHEATH_INNER_RADIUS_MM,PIGTAIL_CATHETER_DIAMETER_MM,PIGTAIL_CATHETER_RENDER_RADIUS_MM} from '../src/toolDimensions.js';
import {DEFAULT_DEVICE_HYDRAULIC_PROFILES} from '../src/contrast/injectionHydraulics.js';
import test from 'node:test';
import assert from 'node:assert/strict';
import {defineKirchhoffMaterialProfile} from '../src/physics/kirchhoffMaterialProfile.js';
import {createSharedAxisNative,feedSharedAxisNative,relaxSharedAxisNative,sharedAxisOuterMaterialAt,sharedAxisOuterIntervals} from '../src/physics/kirchhoffSharedAxisNative.js';
import {createSharedAxisAppSystem} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {createSharedAxisContacts} from '../src/physics/kirchhoffSharedAxisContacts.js';
import {initializeElasticIntroducer,introducerAccessBoundary,elasticIntroducerTool,prepareElasticIntroducer,publishElasticIntroducer} from '../src/devices/introducerMechanics.js';
import {sampleSheathPath} from '../src/physics/sheathPath.js';
import {sharedAxisToolMessage,updateSharedAxisWorkerTools,captureSharedAxisWorkerBodies,publishSharedAxisWorkerBodies} from '../src/physics/kirchhoffSharedAxisWorkerData.js';
import {captureSharedAxisReplay,restoreSharedAxisReplay} from './helpers/sharedAxisReplay.js';

const beam=(EI,curvature=0)=>defineKirchhoffMaterialProfile({id:`beam-${EI}-${curvature}`,sampleEI1:()=>EI,sampleGJ:()=>EI/1.4,sampleKappa01:()=>curvature});
const accepted=s=>{const result=relaxSharedAxisNative(s);assert.ok(result.converged,JSON.stringify(result));};
const fresh=()=>{const sheath={start:{x:0,y:0,z:0},end:{x:70,y:0,z:0}};initializeElasticIntroducer(sheath);return sheath;};

test('sheath French is its lumen size: same-size catheters fit and the outer wall is larger',()=>{
    for(const french of INTRODUCER_SIZES_F) {
        const d=introducerDimensions(french);
        assert.equal(2*d.innerRadius,french/3);
        assert.ok(d.radius>d.innerRadius);
        const sheath=fresh();sheath.insertion=0;
        assert.doesNotThrow(()=>prepareElasticIntroducer(sheath,{selection:{length:450,french},catheterProgress:100,catheterRadius:french/6}));
        assert.throws(()=>prepareElasticIntroducer(sheath,{delta:1,catheterProgress:100,catheterRadius:(french+1)/6}),/nie mieści/);
        assert.doesNotThrow(()=>prepareIntroducerExchange({sheath,length:450,insertion:0,french,catheterProgress:100,catheterRadius:french/6}));
    }
    const d=introducerDimensions(6);
    assert.equal(d.radius,INTRODUCER_SHEATH_RADIUS_MM);
    assert.equal(d.innerRadius,INTRODUCER_SHEATH_INNER_RADIUS_MM);
    assert.equal(DEFAULT_DEVICE_HYDRAULIC_PROFILES.sheath.innerDiameterMm,2*d.innerRadius);
    assert.equal(d.radius*2,2.2);
    assert.ok(Math.abs(PIGTAIL_CATHETER_DIAMETER_MM-5/3)<1e-12);
    assert.equal(2*PIGTAIL_CATHETER_RENDER_RADIUS_MM,PIGTAIL_CATHETER_DIAMETER_MM);
    assert.ok(d.innerRadius*2>PIGTAIL_CATHETER_DIAMETER_MM);
});

test('oblique access roundoff cannot request more than the nominal sheath length',()=>{
    const sheath={start:{x:0,y:0,z:0},end:{x:70.00000000000003,y:0,z:0}};
    initializeElasticIntroducer(sheath);assert.equal(sheath.insertion,70);
});

test('three elastic materials: inserting and withdrawing wire changes the proximal sheath shaft too',()=>{
    const make=wire=>createSharedAxisNative({startCoordinate:-15,tools:[
        {id:'wire',insertion:wire,type:beam(9e6),radius:.4445},
        {id:'catheter',insertion:80,type:beam(2e6,.008),radius:5/6},
        {id:'sheath',insertion:70,type:beam(2e6),radius:1}
    ]});
    let s=make(0);accepted(s);
    const before=s.positions[s.coordinates.indexOf(30)].slice();
    s=feedSharedAxisNative(s,{wire:70,catheter:80,sheath:70});accepted(s);
    const supported=s.positions[s.coordinates.indexOf(30)].slice();
    assert.ok(Math.hypot(...supported.map((x,i)=>x-before[i]))>.5,'shaft 40 mm proximal to sheath tip moves with wire');
    assert.ok(Math.abs(supported[1])<Math.abs(before[1])*.5,'wire straightens the complete overlap');
    const sheath=s.materials.find(m=>m.spec.id==='sheath');
    assert.ok(sheath.body.kirchhoffBendCompliance1[3]>0,'shaft has finite bending compliance');
    assert.equal(s.layout.spins.size,3);
    s=feedSharedAxisNative(s,{wire:0,catheter:80,sheath:70});accepted(s);
    const withdrawn=s.positions[s.coordinates.indexOf(30)];
    assert.ok(Math.hypot(...withdrawn.map((x,i)=>x-before[i]))<1e-4,'withdrawing wire restores the softer equilibrium');
});

test('outer contact follows sheath, catheter and wire across near-coincident outlets',()=>{
    const s=createSharedAxisNative({startCoordinate:-15,tools:[
        {id:'wire',insertion:30.0002,type:beam(1e6),radius:.44},
        {id:'catheter',insertion:30,type:beam(1e6),radius:.8},
        {id:'sheath',insertion:29.9995,type:'introducer-sheath',radius:1}
    ]});
    const e=s.coordinates.length-2,intervals=sharedAxisOuterIntervals(s,e);
    assert.deepEqual(intervals.map(i=>i.material.spec.id),['sheath','catheter','wire']);
    assert.equal(sharedAxisOuterMaterialAt(s,e,0).body.radius,1);
    assert.equal(sharedAxisOuterMaterialAt(s,e,1).spec.id,'wire');
});

test('nearly coincident inner tips do not create a tiny cell inside the longer sheath',()=>{
    const s=createSharedAxisNative({startCoordinate:-15,tools:[
        {id:'wire',insertion:29.9999,type:beam(1e6),radius:.44},
        {id:'catheter',insertion:30,type:beam(1e6),radius:.8},
        {id:'sheath',insertion:80,type:'introducer-sheath',radius:1}
    ]});
    assert.ok(s.coordinates.slice(1).every((x,i)=>x-s.coordinates[i]>=.001));
    assert.equal(s.materials[0].coordinates.at(-1),29.9999);
    assert.equal(s.materials[1].coordinates.at(-1),30);
    accepted(s);
});

test('fixed access only covers the entry tract; elastic intravascular shaft has no frozen tunnel',()=>{
    const sheath=fresh();sheath.length=450;sheath.insertion=300;
    sheath.path=[{x:0,y:0,z:0,s:0},{x:180,y:70,z:0,s:300}];
    const contacts=createSharedAxisContacts({sheath:introducerAccessBoundary(sheath)});
    assert.equal(contacts.length,70);
    const row=contacts.wallSamples[0]({a:[90,50,0],b:[95,50,0],coordinateA:90,coordinateB:95,radius:1});
    assert.equal(row.gap,1);assert.ok(row.jacobian.every(x=>x===0));
});

function fixture() {
    const sheath=fresh(),world=new EndovascularPhysicsWorld(),dt=1/60;
    const wire=world.createRod('wire',30,5,{radius:.4445,mass:1}),catheter=world.createRod('catheter',30,5,{radius:5/6,mass:1.4}),body=world.createRod('sheath',191,5,{radius:1,mass:1.6});
    const controls={wire:0,catheter:0};
    const readTools=()=>[
        {id:'wire',type:'glidewire',body:wire,insertion:controls.wire,rotation:0,shaftStiffness:1,tipStiffness:1,nodeCoordinates:Array.from({length:30},(_,i)=>-40+i*5)},
        {id:'catheter',type:'berenstein',body:catheter,insertion:controls.catheter,rotation:0,shaftStiffness:1,tipStiffness:1,nodeCoordinates:Array.from({length:30},(_,i)=>-40+i*5)},
        elasticIntroducerTool(sheath,body)
    ];
    world.contactField={voxelSize:1,queryCapsuleSoA(x,y,z,r,e,out){return Object.assign(out,{signedDistance:100,signedGap:100-r[e],segmentT:.5,faceIndex:0});}};
    const system=createSharedAxisAppSystem({readTools,readSheath:()=>introducerAccessBoundary(sheath),workSliceMs:Infinity});
    const step=()=>{const r=system.step(world,dt);assert.ok(r.accepted,JSON.stringify(r));publishElasticIntroducer(sheath,body);};
    return {sheath,world,body,controls,readTools,system,step};
}

test('application publishes all three rods; manual 45/90 cm feed, selection and worker buffers retain full sheath',()=>{
    const f=fixture();f.step();assert.equal(f.sheath.path.at(-1).s,70);
    assert.throws(()=>prepareElasticIntroducer(f.sheath,{selection:{length:450,french:6}}),/wycofaj/);
    prepareElasticIntroducer(f.sheath,{delta:-70});f.step();
    for(const length of [450,900]) {
        prepareElasticIntroducer(f.sheath,{selection:{length,french:length===450?6:8}});f.step();
        // All three are inserted together, then independently withdrawn.
        f.controls.wire=110;f.controls.catheter=90;
        prepareElasticIntroducer(f.sheath,{delta:100});f.step();
        assert.equal(f.body.jointStateView.coordinates.at(-1),100);
        assert.ok(f.sheath.path.length>10);
        assert.ok(Number.isFinite(sampleSheathPath(f.sheath,95).x));
        const arc=f.sheath.path.reduce((sum,p,i,path)=>i?sum+Math.hypot(p.x-path[i-1].x,p.y-path[i-1].y,p.z-path[i-1].z):0,0);
        assert.ok(Math.abs(arc-140)<.01,'published shaft preserves material length');
        const tools=f.readTools(),workers=updateSharedAxisWorkerTools(tools.map(sharedAxisToolMessage));
        publishSharedAxisWorkerBodies(workers,captureSharedAxisWorkerBodies(tools));
        assert.deepEqual(workers[2].body.jointStateView,f.body.jointStateView);
        prepareElasticIntroducer(f.sheath,{delta:-100});f.step();
        assert.equal(f.sheath.path.at(-1).s,0);
        assert.equal(f.readTools()[0].insertion,110);assert.equal(f.readTools()[1].insertion,90);
    }
});

test('three-tool native replay preserves sheath material and its contacts',()=>{
    const sheath=fresh();
    const s=createSharedAxisNative({...createSharedAxisContacts({sheath:introducerAccessBoundary(sheath)}),tools:[
        {id:'wire',type:'glidewire',insertion:90,radius:.44},
        {id:'catheter',type:'berenstein',insertion:80,radius:5/6},
        {id:'sheath',type:'introducer-sheath',insertion:70,radius:1,length:450}
    ]});accepted(s);
    const restored=restoreSharedAxisReplay(captureSharedAxisReplay(s,introducerAccessBoundary(sheath)));
    assert.equal(restored.materials.length,3);
    assert.deepEqual(restored.positions,s.positions);
    assert.equal(restored.materials[2].spec.type,'introducer-sheath');
});

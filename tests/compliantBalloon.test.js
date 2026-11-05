import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {RELIANT,RELIANT_INFLATION_TABLE,balloonFreeDiameter,balloonInflationBlock,balloonMotionLocked,advanceBalloonVolume,sampleCompliantBalloon,sampleBalloonOnCatheter,updateBalloonGeometry} from '../src/devices/compliantBalloon.js';
import {kirchhoffMaterialProfile} from '../src/physics/kirchhoffMaterialProfile.js';
import {catheterMaterialProfile} from '../src/physics/catheterMaterialProfile.js';
import {PigtailCatheter} from '../src/pigtailCatheter.js';
import {createSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {captureSharedAxisReplay} from '../src/physics/kirchhoffSharedAxisReplay.js';
import {restoreSharedAxisReplay} from '../src/physics/kirchhoffSharedAxisRestore.js';
import {createSharedAxisContacts} from '../src/physics/kirchhoffSharedAxisContacts.js';

const path={getPointAt:(u,out)=>out.set(0,0,u*200),getTangentAt:(_u,out)=>out.set(0,0,1)};
const sample=(v,wallDistance=()=>100)=>sampleCompliantBalloon({path,coordinates:[0,200],volumeMl:v,wallDistance});

test('Reliant uses the manufacturer diameter/volume guide and independent straight shaft profile',()=>{
    for(const [v,d] of RELIANT_INFLATION_TABLE)assert.equal(balloonFreeDiameter(v),d);
    assert.equal(balloonFreeDiameter(100),46);assert.throws(()=>balloonFreeDiameter(NaN));
    const p=kirchhoffMaterialProfile(RELIANT.type);
    assert.equal(p.id,RELIANT.type);assert.equal(p.sample(20).kappa01,0);
    assert.ok(p.sample(100).EI1>p.sample(0).EI1);
    assert.equal(catheterMaterialProfile(RELIANT.type).naturalArcLength,0);
    const c=new PigtailCatheter({wire:{nodes:[]},segmentLength:5,guidewireLength:1800,tailProgressRef:()=>0});c.setType(RELIANT.type);
    assert.equal(c.type,RELIANT.type);assert.equal(c.maxLength,1000);assert.deepEqual(c.getInjectionPorts(),[]);c.dispose();
});
test('free balloon volume is conserved across the published range, with bounded axial extent',()=>{
    for(const v of [.001,.1,1,3,9,19,41,60]) {
        const s=sample(v);assert.equal(s.invalid,false);assert.equal(s.limited,false);
        assert.ok(Math.abs(s.volumeMl-v)<v*.001,`${v}: ${s.volumeMl}`);
        assert.ok(s.length<=RELIANT.sectionLength);
        assert.ok(Math.abs(s.maxDiameter-balloonFreeDiameter(v))<1e-10);
        assert.ok(s.points.every(Number.isFinite));
    }
});
test('membrane conforms asymmetrically, refuses excess fluid, and does not widen a narrow lumen',()=>{
    const s=sample(41,(_p,d)=>d.x>0?5:12);
    assert.equal(s.invalid,false);assert.equal(s.limited,true);assert.ok(s.volumeMl<41);
    for(let i=0;i<s.points.length;i+=3) {
        const x=s.points[i],y=s.points[i+1],r=Math.hypot(x,y);
        assert.ok(r<=(x>1e-5?5:12)+1e-5);
    }
    assert.equal(sample(10,()=>NaN).invalid,true);
    assert.equal(sample(10,()=>1).invalid,true);
    const g=new THREE.BufferGeometry();updateBalloonGeometry(g,s);const buffer=g.attributes.position.array;
    updateBalloonGeometry(g,sample(9));assert.equal(g.attributes.position.array,buffer);g.dispose();
});
test('mesh rays clip against the actual wall, and missing anatomy fails closed',()=>{
    const geometry=new THREE.CylinderGeometry(8,8,400,64,1,true).rotateX(Math.PI/2).translate(0,0,150);
    const tree=new MeshBVH(geometry);
    const catheter={physicsBody:{jointStateView:{coordinates:new Float64Array([0,200]),positions:new Float64Array([0,0,0,0,0,200])}}};
    const s=sampleBalloonOnCatheter(catheter,tree,41);
    assert.equal(s.invalid,false);assert.equal(s.limited,true);assert.ok(s.maxDiameter<16);
    assert.equal(sampleBalloonOnCatheter(catheter,null,41),null);geometry.dispose();
});
test('inflation requires 12F, removed dilator and full exposure; movement unlocks only after deflation',()=>{
    const c={type:RELIANT.type,progress:160};
    const options={catheter:c,sheathInsertion:70,sheathFrench:12,hasWall:true};
    assert.equal(balloonInflationBlock(options),'');
    assert.match(balloonInflationBlock({...options,sheathFrench:6}),/12F/);
    assert.match(balloonInflationBlock({...options,dilatorInsertion:1}),/rozszerzacz/);
    assert.match(balloonInflationBlock({...options,sheathInsertion:130}),/poza koszulkę/);
    assert.ok(balloonMotionLocked(c,10));assert.equal(balloonMotionLocked(c,0),false);
    c.balloonDriveMl=1;assert.ok(balloonMotionLocked(c,0));
    let volume=0;for(let i=0;i<100;i++)volume=advanceBalloonVolume(volume,60,1/60);
    assert.ok(Math.abs(volume-10)<1e-10);assert.equal(advanceBalloonVolume(volume,60,0),volume);
    for(let i=0;i<51;i++)volume=advanceBalloonVolume(volume,0,1/60);
    assert.equal(volume,0);assert.throws(()=>advanceBalloonVolume(0,NaN,1));
});
test('native coaxial solver and replay retain the 8F balloon shaft material',()=>{
    const sheath={start:{x:0,y:0,z:0},end:{x:0,y:0,z:70},innerRadius:2};
    const contacts=createSharedAxisContacts({sheath});
    const s=createSharedAxisNative({...contacts,tools:[
        {id:'wire',type:'glidewire',insertion:200,radius:.4445,length:1800},
        {id:'catheter',type:RELIANT.type,insertion:160,radius:RELIANT.shaftRadius,length:1000}
    ]});
    const r=restoreSharedAxisReplay(captureSharedAxisReplay(s,sheath));
    const m=r.materials.find(m=>m.spec.id==='catheter');
    assert.equal(m.spec.type,RELIANT.type);assert.equal(m.body.radius,8/6);
    assert.deepEqual(m.body.kirchhoffBendCompliance1,s.materials[1].body.kirchhoffBendCompliance1);
});

test('constrained inflation never loses fluid and deflation shrinks the accepted membrane monotonically',async()=>{
    const {stepCompliantBalloon}=await import('../src/devices/compliantBalloon.js');
    const s={};let volume=0;
    for(let i=0;i<650;i++) {
        stepCompliantBalloon(s,60,1/60,v=>sample(v,()=>4));
        assert.ok((s.sample?.volumeMl??0)>=volume-1e-7);volume=s.sample?.volumeMl??0;
    }
    assert.ok(volume>0&&volume<10);assert.equal(s.sample.limited,true);
    const held=s.sample;stepCompliantBalloon(s,0,0,()=>{throw new Error('paused');});assert.equal(s.sample,held);
    let width=s.sample.maxDiameter;
    for(let i=0;i<100;i++) {
        stepCompliantBalloon(s,0,1/60,()=>{throw new Error('deflation must use the accepted membrane');});
        assert.ok((s.sample?.volumeMl??0)<=volume);assert.ok((s.sample?.maxDiameter??0)<=width+1e-5);
        volume=s.sample?.volumeMl??0;width=s.sample?.maxDiameter??0;
    }
    assert.equal(s.driveMl,0);assert.equal(s.sample,null);
});

test('selecting the withdrawn balloon beside a 6F sheath does not stall the app solver',async()=>{
    const {EndovascularPhysicsWorld}=await import('../src/physics/endovascularPhysicsWorld.js');
    const {createSharedAxisAppSystem}=await import('../src/physics/kirchhoffSharedAxisAppSystem.js');
    const {initializeElasticIntroducer,elasticIntroducerTool,introducerAccessBoundary}=await import('../src/devices/introducerMechanics.js');
    const sheath={start:{x:0,y:0,z:0},end:{x:70,y:0,z:0}};initializeElasticIntroducer(sheath);
    const world=new EndovascularPhysicsWorld();
    const wire=world.createRod('wire',41,5,{radius:.4445}),catheter=world.createRod('catheter',41,5,{radius:8/6}),sleeve=world.createRod('sheath',41,5,{radius:1.1});
    world.contactField={voxelSize:1,queryCapsuleSoA(x,y,z,r,e,out){return Object.assign(out,{signedDistance:100,signedGap:100-r[e],segmentT:.5,faceIndex:0});}};
    const app=createSharedAxisAppSystem({workSliceMs:Infinity,readSheath:()=>introducerAccessBoundary(sheath,RELIANT.shaftRadius),readTools:()=>[
        {id:'wire',type:'glidewire',body:wire,insertion:0,rotation:0,nodeCoordinates:Array.from({length:41},(_,i)=>-40+i)},
        {id:'catheter',type:RELIANT.type,body:catheter,insertion:0,rotation:0,shaftStiffness:40.65,tipStiffness:100,nodeCoordinates:Array.from({length:41},(_,i)=>-40+i)},
        elasticIntroducerTool(sheath,sleeve)
    ]});
    const r=app.step(world,1/60);assert.ok(r.accepted,JSON.stringify(r));assert.equal(catheter.jointStateView.coordinates.at(-1),0);
});


test('withdrawn sheath unlock tolerates only publication round-off at zero',async()=>{
    const {canSelectIntroducer}=await import('../src/devices/introducerSheath.js');
    const s={start:{x:0,y:0,z:0},end:{x:0,y:0,z:0},path:[{s:1e-14}],dilatorInsertion:0};
    assert.equal(canSelectIntroducer(s),true);s.path[0].s=.001;assert.equal(canSelectIntroducer(s),false);
    s.path[0].s=0;s.dilatorInsertion=1;assert.equal(canSelectIntroducer(s),false);
});

test('balloon wall queries share the accepted local vessel displacement',async()=>{
    const {VesselDeformation}=await import('../src/vesselDeformation.js');
    const d=new VesselDeformation();d.update([{center:[1,0,0],normal:[1,0,0],width:4,displacement:.2}]);
    assert.ok(Math.abs(d.displacementAt(new THREE.Vector3(1,0,0)).x-.2)<1e-7);
    assert.equal(d.displacementAt(new THREE.Vector3(-4,0,0)).length(),0);
    d.update([]);assert.equal(d.displacementAt(new THREE.Vector3(1,0,0)).length(),0);d.dispose();
});

test('a rejected inflation trial retains contained fluid instead of emptying the balloon',async()=>{
    const {stepCompliantBalloon}=await import('../src/devices/compliantBalloon.js');
    const state={};stepCompliantBalloon(state,10,.5,v=>sample(v));const volume=state.sample.volumeMl;
    stepCompliantBalloon(state,10,.5,()=>({invalid:true}));
    assert.equal(state.sample.volumeMl,volume);assert.equal(state.sample.limited,true);assert.equal(state.invalid,false);
    stepCompliantBalloon(state,0,1,()=>{throw Error('no sampling during deflation');});assert.equal(state.sample,null);
});

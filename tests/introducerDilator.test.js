import test from 'node:test';
import assert from 'node:assert/strict';
import {initializeElasticIntroducer,prepareElasticIntroducer,elasticIntroducerTool,introducerAccessBoundary,publishElasticIntroducer} from '../src/devices/introducerMechanics.js';
import {INTRODUCER_LENGTHS_MM,INTRODUCER_SIZES_F,canSelectIntroducer} from '../src/devices/introducerSheath.js';
import {initializeIntroducerDilator,dilatorDimensions,dilatorRadiusAtDistance,dilatorMaterialProfile,elasticDilatorTool,prepareDilatorMotion,
    prepareIntroducerAssembly,dilatorLimitedSheathDelta,followSeatedDilator,publishElasticDilator,DILATOR_TIP_LENGTH_MM,DILATOR_TIP_RADIUS_MM} from '../src/devices/introducerDilator.js';
import {createIntroducerDilatorGeometry} from '../src/devices/introducerDilatorGeometry.js';
import {createSharedAxisNative,feedSharedAxisNative,relaxSharedAxisNative,sharedAxisOuterMaterialAt} from '../src/physics/kirchhoffSharedAxisNative.js';
import {createSharedAxisContacts} from '../src/physics/kirchhoffSharedAxisContacts.js';
import {createSharedAxisAppSystem} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {defineKirchhoffMaterialProfile} from '../src/physics/kirchhoffMaterialProfile.js';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {createPreparedInputCheckpoint} from '../src/physics/preparedInputCheckpoint.js';
import {captureSharedAxisReplay,restoreSharedAxisReplay} from '../src/physics/kirchhoffSharedAxisRestore.js';

const fresh=()=>{
    const s={start:{x:0,y:0,z:0},end:{x:70,y:0,z:0}};
    initializeElasticIntroducer(s);initializeIntroducerDilator(s);return s;
};
const moveSheath=(s,delta,wire=2000)=>{const old=s.insertion;
    prepareElasticIntroducer(s,{delta:dilatorLimitedSheathDelta(s,delta,wire)});followSeatedDilator(s,old);};
const relax=s=>{const result=relaxSharedAxisNative(s);assert.ok(result.converged,JSON.stringify(result));return result;};

test('every 7/45/90 cm sheath has a matching hollow dilator with a smooth tapered tip',()=>{
    for(const length of INTRODUCER_LENGTHS_MM)for(const french of INTRODUCER_SIZES_F) {
        const s=fresh();s.insertion=0;prepareElasticIntroducer(s,{selection:{length,french}});
        const d=dilatorDimensions(s);assert.equal(d.length,length+DILATOR_TIP_LENGTH_MM);
        assert.ok(d.radius<s.innerRadius&&d.innerRadius>.4445&&DILATOR_TIP_RADIUS_MM>d.innerRadius);
        assert.equal(dilatorRadiusAtDistance(0,d.radius),DILATOR_TIP_RADIUS_MM);
        assert.equal(dilatorRadiusAtDistance(25,d.radius),d.radius);
        assert.ok(dilatorRadiusAtDistance(.01,d.radius)-DILATOR_TIP_RADIUS_MM<1e-5);
        const p=dilatorMaterialProfile(d.radius);
        assert.ok(p.sample(25).EI1>p.sample(0).EI1&&p.sample(0).EI1>0);
        assert.equal(p.sample(0).kappa01,0);
    }
});

test('manual seating, combined feed, independent withdrawal, and catheter exclusion',()=>{
    const s=fresh();
    assert.throws(()=>prepareDilatorMotion(s,10,{wireProgress:200,catheterProgress:1}),/Wycofaj cewnik/);
    assert.throws(()=>prepareDilatorMotion(s,10),/prowadnik/);
    prepareDilatorMotion(s,1000,{wireProgress:200});assert.equal(s.dilatorInsertion,95);assert.equal(s.dilatorSeated,true);
    s.length=450;moveSheath(s,100);assert.equal(s.insertion,170);assert.equal(s.dilatorInsertion,195);
    moveSheath(s,100,215);assert.equal(s.dilatorInsertion,205,'wire must remain ahead of the tip');
    const sheathPosition=s.insertion;
    prepareDilatorMotion(s,-50);assert.equal(s.dilatorInsertion,155);assert.equal(s.insertion,sheathPosition);assert.equal(s.dilatorSeated,false);
    moveSheath(s,-1000);assert.equal(s.insertion,130,'partly withdrawn core must not become a long exposed shaft');
    prepareDilatorMotion(s,-1000);assert.equal(s.dilatorInsertion,0);moveSheath(s,-1000);assert.equal(s.insertion,0);
});

test('selection and rollback preserve both independent device positions',()=>{
    const s=fresh();s.insertion=0;s.path=[{...s.start,s:0},{...s.start,s:0}];
    prepareDilatorMotion(s,25,{wireProgress:100});
    assert.equal(canSelectIntroducer(s),false);
    assert.throws(()=>prepareElasticIntroducer(s,{selection:{length:900,french:8}}),/rozszerzacz/);
    const checkpoint=createPreparedInputCheckpoint();checkpoint.capture([s]);
    moveSheath(s,30);assert.equal(s.dilatorInsertion,55);checkpoint.restore();
    assert.equal(s.insertion,0);assert.equal(s.dilatorInsertion,25);assert.equal(s.dilatorSeated,true);
    prepareDilatorMotion(s,-25);assert.equal(canSelectIntroducer(s),true);
});

test('native contact uses the taper, and removing the core removes its bending support',()=>{
    const curved=defineKirchhoffMaterialProfile({id:'curved-dilator-test-sheath',sampleEI1:()=>2e6,sampleGJ:()=>2e6/1.4,sampleKappa01:()=>.008});
    let s=createSharedAxisNative({startCoordinate:-15,tools:[
        {id:'wire',type:'glidewire',insertion:0,radius:.4445},
        {id:'catheter',type:'berenstein',insertion:0,radius:5/6},
        {id:'sheath',type:curved,insertion:80,radius:1.1},
        {id:'dilator',type:'introducer-dilator',insertion:105,radius:.96,length:475}
    ]});relax(s);
    const supported=s.positions[s.coordinates.indexOf(40)].slice();
    for(const x of [82,90,100,105]) {
        const edge=s.coordinates.findIndex((v,i)=>i<s.coordinates.length-1&&v<=x&&s.coordinates[i+1]>=x);
        const t=(x-s.coordinates[edge])/(s.coordinates[edge+1]-s.coordinates[edge]);
        const owner=sharedAxisOuterMaterialAt(s,edge,t);
        assert.equal(owner.spec.id,'dilator');
        assert.ok(Math.abs(owner.body.radius-dilatorRadiusAtDistance(105-x,.96))<1e-12);
    }
    s=feedSharedAxisNative(s,{dilator:0});relax(s);
    assert.ok(Math.hypot(...s.positions[s.coordinates.indexOf(40)].map((v,k)=>v-supported[k]))>.1,
        'removing the dilator must soften the mechanical overlap, not just hide a mesh');
});

test('rendered dilator has the same shaft and tip diameters as mechanical contact',()=>{
    const s=fresh();s.dilatorInsertion=95;s.dilatorPath=[{x:-40,y:0,z:0,s:-40},{x:95,y:0,z:0,s:95}];
    const g=createIntroducerDilatorGeometry(s),p=g.attributes.position,a=Array.from(p.array);
    assert.ok(a.every(Number.isFinite));
    for(const x of [0,95]) {
        const radii=[];for(let i=0;i<p.count;i++)if(Math.abs(p.getX(i)-x)<1e-5)radii.push(Math.hypot(p.getY(i),p.getZ(i)));
        assert.ok(radii.length>0);const expected=x===95?DILATOR_TIP_RADIUS_MM:dilatorDimensions(s).radius;
        for(const r of radii)assert.ok(Math.abs(r-expected)<1e-6);
    }
    g.dispose();
});

test('application solver publishes and replays a removable dilator as a fourth material',()=>{
    const sheath=fresh(),world=new EndovascularPhysicsWorld();
    const wire=world.createRod('wire',30,5,{radius:.4445,mass:1}),catheter=world.createRod('catheter',30,5,{radius:5/6,mass:1.4}),
        sleeve=world.createRod('sheath',191,5,{radius:1.1,mass:1.6}),core=world.createRod('dilator',201,5,{radius:.96,mass:1.4});
    const tools=()=>[
        {id:'wire',type:'glidewire',body:wire,insertion:150,rotation:0,nodeCoordinates:Array.from({length:30},(_,i)=>-40+i*5)},
        {id:'catheter',type:'berenstein',body:catheter,insertion:0,rotation:0,nodeCoordinates:Array.from({length:30},(_,i)=>-40+i*5)},
        elasticIntroducerTool(sheath,sleeve),elasticDilatorTool(sheath,core)
    ];
    world.contactField={voxelSize:1,queryCapsuleSoA(x,y,z,r,e,out){return Object.assign(out,{signedDistance:100,signedGap:100-r[e],segmentT:.5,faceIndex:0});}};
    const app=createSharedAxisAppSystem({readTools:tools,readSheath:()=>introducerAccessBoundary(sheath),workSliceMs:Infinity});
    for(const delta of [95,-35,-60]) {
        prepareDilatorMotion(sheath,delta,{wireProgress:150});
        const result=app.step(world,1/60);assert.ok(result.accepted,JSON.stringify(result));
        publishElasticIntroducer(sheath,sleeve);publishElasticDilator(sheath,core);
        assert.equal(core.jointStateView.coordinates.at(-1),sheath.dilatorInsertion);
        assert.equal(sheath.dilatorPath.at(-1).s,sheath.dilatorInsertion);
        assert.equal(app.diagnostics.last.quality.bodies.length,4);
    }
    const s=createSharedAxisNative({...createSharedAxisContacts({sheath:introducerAccessBoundary(sheath)}),tools:tools().map(({body,nodeCoordinates,...tool})=>({...tool,radius:body.radius}))});
    const restored=restoreSharedAxisReplay(captureSharedAxisReplay(s,introducerAccessBoundary(sheath)));
    assert.equal(restored.materials.at(-1).spec.type,'introducer-dilator');
    assert.deepEqual(restored.materials.at(-1).body.kirchhoffBendCompliance1,s.materials.at(-1).body.kirchhoffBendCompliance1);
});

test('sheath insertion from zero automatically advances the matching core, then the pair',()=>{
    for(const length of INTRODUCER_LENGTHS_MM)for(const french of INTRODUCER_SIZES_F) {
        const s=fresh();s.insertion=0;
        prepareIntroducerAssembly(s,{selection:{length,french}});
        assert.equal(s.dilatorInsertion,0,'selection alone does not insert the core');
        prepareIntroducerAssembly(s,{delta:10,wireProgress:2000});
        assert.equal(s.insertion,0);assert.equal(s.dilatorInsertion,10);assert.equal(s.dilatorSeated,false);
        prepareIntroducerAssembly(s,{delta:20,wireProgress:2000});
        assert.equal(s.insertion,5);assert.equal(s.dilatorInsertion,30);assert.equal(s.dilatorSeated,true);
        assert.equal(dilatorDimensions(s).radius,s.innerRadius-.04);
        prepareIntroducerAssembly(s,{delta:1000,wireProgress:2000});
        assert.equal(s.insertion,length);assert.equal(s.dilatorInsertion,length+25);
    }
});

test('automatic insertion respects wire support and preserves exchange over a catheter',()=>{
    const s=fresh();s.insertion=0;const before={...s};
    assert.throws(()=>prepareIntroducerAssembly(s,{delta:1}),/prowadnik/);
    assert.deepEqual(s,before);
    prepareIntroducerAssembly(s,{delta:30,wireProgress:20});
    assert.equal(s.insertion,0);assert.equal(s.dilatorInsertion,10);
    prepareIntroducerAssembly(s,{delta:30,wireProgress:40});
    assert.equal(s.insertion,5);assert.equal(s.dilatorInsertion,30);
    assert.equal(prepareIntroducerAssembly(s,{delta:1,wireProgress:40}),false);
    const exchange=fresh();exchange.insertion=0;
    prepareIntroducerAssembly(exchange,{delta:5,catheterProgress:150,wireProgress:200});
    assert.equal(exchange.insertion,5);assert.equal(exchange.dilatorInsertion,0);
    const legacy=fresh();legacy.insertion=0;
    prepareIntroducerAssembly(legacy,{delta:5,hasDilator:false});
    assert.equal(legacy.insertion,5);assert.equal(legacy.dilatorInsertion,0);
});

test('withdrawing the complete assembly clears the remaining tapered tip and allows reselection',()=>{
    const s=fresh();s.insertion=0;s.path=[{...s.start,s:0}];
    prepareIntroducerAssembly(s,{delta:55,wireProgress:200});
    prepareIntroducerAssembly(s,{delta:-35,wireProgress:200});
    assert.equal(s.insertion,0);assert.equal(s.dilatorInsertion,20);assert.equal(s.dilatorSeated,false);
    assert.equal(canSelectIntroducer(s),false);
    prepareIntroducerAssembly(s,{delta:-20,wireProgress:200});
    assert.equal(s.dilatorInsertion,0);assert.equal(canSelectIntroducer(s),true);
    prepareIntroducerAssembly(s,{selection:{length:900,french:12}});
    prepareIntroducerAssembly(s,{delta:30,wireProgress:200});
    assert.equal(s.insertion,5);assert.equal(s.dilatorInsertion,30);assert.equal(s.dilatorSeated,true);
});

test('automatic core preparation rolls back with the sheath and does not repeat on a paused step',()=>{
    const s=fresh();s.insertion=0;const before={...s},checkpoint=createPreparedInputCheckpoint();
    checkpoint.capture([s]);prepareIntroducerAssembly(s,{delta:30,wireProgress:200});
    assert.equal(s.dilatorInsertion,30);checkpoint.restore();assert.deepEqual(s,before);
    assert.equal(prepareIntroducerAssembly(s,{delta:0,wireProgress:200}),false);assert.deepEqual(s,before);
});

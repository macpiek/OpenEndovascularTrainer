import test from 'node:test';
import assert from 'node:assert/strict';
import {BufferGeometry,Float32BufferAttribute} from 'three';
import {RELIANT,balloonFreeRadiusAt} from '../src/devices/balloonProfile.js';
import {prepareBalloonInflation,balloonMaterialProfile} from '../src/devices/balloonMechanics.js';
import {createSharedAxisNative,assembleSharedAxisNative,sharedAxisOuterMaterialAt} from '../src/physics/kirchhoffSharedAxisNative.js';
import {createSharedAxisVesselDiscovery,relaxSharedAxisWithContacts} from '../src/physics/kirchhoffSharedAxisVesselWitnesses.js';
import {DEFAULT_VESSEL_ELASTICITY,captureVesselElasticPatches} from '../src/physics/vesselElasticity.js';

function planeFixture(volume) {
    const geometry=new BufferGeometry().setAttribute('position',new Float32BufferAttribute([-1000,4,-1000,1000,4,-1000,0,4,1000],3));
    const field={fallbackGeometry:geometry,voxelSize:1,queryCapsuleSoA(x,y,z,r,i,out){
        out.segmentT=y[1]>y[0]?1:0;out.faceIndex=0;
        out.signedDistance=4-Math.max(y[0],y[1]);out.signedGap=out.signedDistance-r[0];
        out.inward.x=0;out.inward.y=-1;out.inward.z=0;return out;
    }};
    const s=createSharedAxisNative({tools:[{id:'wire',type:'glidewire',insertion:120,shaftStiffness:13.15,tipStiffness:19.4},
        {id:'catheter',type:RELIANT.type,insertion:100,radius:RELIANT.shaftRadius,shaftStiffness:40.65,tipStiffness:100,balloonInflationMl:volume}],
        wallSamples:[createSharedAxisVesselDiscovery(field,0)]});
    s.vesselElasticity=DEFAULT_VESSEL_ELASTICITY;
    return {s,geometry};
}

test('inflation changes the shared catheter/wire equilibrium and loads the actual wall',()=>{
    const empty=planeFixture(0),full=planeFixture(3);
    try {
        const baseline=relaxSharedAxisWithContacts(empty.s);
        assert.ok(baseline.converged,JSON.stringify(baseline));
        const result=relaxSharedAxisWithContacts(full.s);
        assert.ok(result.converged,JSON.stringify(result));
        assert.ok(result.quality.maxPenetration<1e-5);
        const at=full.s.coordinates.indexOf(55);
        assert.ok(full.s.positions[at][1]<empty.s.positions[at][1]-.1);
        assert.ok(result.quality.maxBalloonCompression>0);
        const patches=captureVesselElasticPatches(full.s,assembleSharedAxisNative(full.s).rows,sharedAxisOuterMaterialAt);
        assert.ok(patches.length>0);
        assert.ok(patches.some(p=>p.displacement>0&&p.normal[1]>.99));
        // Both materials own frames but are constrained to the same displaced axis.
        for(const material of full.s.materials)assert.ok(Math.abs(material.body.y[at]-full.s.positions[at][1])<1e-6);
    }finally{empty.geometry.dispose();full.geometry.dispose();}
});

test('free membrane grows monotonically at every location and tension increases rigidity',()=>{
    for(let s=0;s<=90;s+=.25){let previous=RELIANT.shaftRadius;
        for(let v=0;v<=60;v+=.5){const r=balloonFreeRadiusAt(s,v);assert.ok(r>=previous-1e-12);previous=r;}}
    assert.ok(balloonMaterialProfile(20).sample(44).EI1>balloonMaterialProfile(3).sample(44).EI1);
});

test('prepared fill follows contained-volume feedback, respects pause and deflates',()=>{
    const catheter={type:RELIANT.type,balloonDriveMl:3,balloonVolumeMl:3};
    prepareBalloonInflation(catheter,5,1/60);assert.equal(catheter.balloonDriveMl,3.1);
    prepareBalloonInflation(catheter,5,0);assert.equal(catheter.balloonDriveMl,3.1);
    prepareBalloonInflation(catheter,0,1/60);assert.ok(Math.abs(catheter.balloonDriveMl-2.9)<1e-12);
    catheter.balloonVolumeMl=5;prepareBalloonInflation(catheter,5,1/60);assert.ok(Math.abs(catheter.balloonDriveMl-2.9)<1e-12);
});

test('inflation in a closed lumen loads opposing walls, then releases them on deflation',async()=>{
    const {BoxGeometry,Vector3}=await import('three');
    const {MeshBVH}=await import('three-mesh-bvh');
    const {feedSharedAxisNative}=await import('../src/physics/kirchhoffSharedAxisNative.js');
    const {stepSharedAxis}=await import('../src/physics/kirchhoffSharedAxisTimeStep.js');
    const geometry=new BoxGeometry(400,8,8);geometry.boundsTree=new MeshBVH(geometry);
    const point=new Vector3(),nearest={point:new Vector3()};
    const field={fallbackGeometry:geometry,voxelSize:1,queryCapsuleSoA(x,y,z,r,i,out){
        point.set((x[0]+x[1])/2,(y[0]+y[1])/2,(z[0]+z[1])/2);
        geometry.boundsTree.closestPointToPoint(point,nearest);
        const inward=point.clone().sub(nearest.point).normalize();
        out.segmentT=.5;out.faceIndex=nearest.faceIndex;out.signedDistance=nearest.distance;out.signedGap=nearest.distance-r[0];
        Object.assign(out.inward,inward);return out;
    }};
    let s=createSharedAxisNative({tools:[{id:'wire',type:'glidewire',insertion:120,shaftStiffness:13.15,tipStiffness:19.4},
        {id:'catheter',type:RELIANT.type,insertion:100,radius:RELIANT.shaftRadius,shaftStiffness:40.65,tipStiffness:100,balloonInflationMl:0}],
        wallSamples:[createSharedAxisVesselDiscovery(field,0)]});
    const options={vesselElasticity:DEFAULT_VESSEL_ELASTICITY,projectionMode:'reduced',reuseConstraintWork:true,reuseRowBuffers:true,reuseMatrixAssembly:true};
    try {
        for(let v=0;v<=4;v+=.25){
            const source={...s,materials:s.materials.map(m=>({...m,spec:{...m.spec,...(m.spec.id==='catheter'?{balloonInflationMl:v}:{})}}))};
            s=feedSharedAxisNative(source,{});
            const result=stepSharedAxis(s,1/60,options);
            assert.ok(result.converged,`${v} ml: ${JSON.stringify(result)}`);
            assert.ok(result.quality.maxPenetration<1e-5);
        }
        assert.ok(s.vesselElasticPatches.length>0);
        assert.ok(s.vesselElasticPatches.some(p=>p.normal[1]>.9));
        assert.ok(s.vesselElasticPatches.some(p=>p.normal[1]<-.9));
        const source={...s,materials:s.materials.map(m=>({...m,spec:{...m.spec,balloonInflationMl:0}}))};
        s=feedSharedAxisNative(source,{});
        const unloaded=stepSharedAxis(s,1/60,options);
        assert.ok(unloaded.converged,JSON.stringify(unloaded));
        assert.deepEqual(s.vesselElasticPatches,[]);
    }finally{geometry.dispose();}
});

test('prepared inflation rolls back atomically, and replay retains inflation-dependent stiffness',async()=>{
    const {createPreparedInputCheckpoint}=await import('../src/physics/preparedInputCheckpoint.js');
    const {captureSharedAxisReplay}=await import('../src/physics/kirchhoffSharedAxisReplay.js');
    const {restoreSharedAxisReplay}=await import('../src/physics/kirchhoffSharedAxisRestore.js');
    const {createSharedAxisContacts}=await import('../src/physics/kirchhoffSharedAxisContacts.js');
    const c={type:RELIANT.type,balloonDriveMl:3,balloonVolumeMl:2},checkpoint=createPreparedInputCheckpoint();
    checkpoint.capture([c]);prepareBalloonInflation(c,10,1/60);assert.notEqual(c.balloonDriveMl,3);
    checkpoint.restore();assert.equal(c.balloonDriveMl,3);assert.equal(c.balloonVolumeMl,2);
    const sheath={start:{x:0,y:0,z:0},end:{x:10,y:0,z:0},innerRadius:2};
    const s=createSharedAxisNative({...createSharedAxisContacts({sheath}),tools:[
        {id:'wire',type:'glidewire',insertion:120},
        {id:'catheter',type:RELIANT.type,insertion:100,radius:RELIANT.shaftRadius,balloonInflationMl:3,wallStaticFriction:.2,wallKineticFriction:.15}]});
    const r=restoreSharedAxisReplay(captureSharedAxisReplay(s,sheath));
    assert.equal(r.materials[1].spec.balloonInflationMl,3);
    assert.deepEqual(r.materials[1].body.kirchhoffBendCompliance1,s.materials[1].body.kirchhoffBendCompliance1);
    const balloon=sharedAxisOuterMaterialAt(r,r.coordinates.indexOf(55),0),shaft=sharedAxisOuterMaterialAt(r,r.coordinates.indexOf(10),0);
    assert.ok(balloon.body.radius>RELIANT.shaftRadius);
    assert.equal(balloon.spec.wallStaticFriction,0);
    assert.equal(shaft.spec.wallStaticFriction,.2);
});

test('a confined balloon limits further pumping and relieves excess overdrive',()=>{
    const c={type:RELIANT.type,balloonDriveMl:3,balloonVolumeMl:2.4};
    for(let i=0;i<100;i++)prepareBalloonInflation(c,60,1/60);
    assert.ok(Math.abs(c.balloonDriveMl-3.24)<1e-10);
    c.balloonVolumeMl=2;prepareBalloonInflation(c,60,1/60);
    assert.ok(Math.abs(c.balloonDriveMl-3.04)<1e-10);
    // As the wall yields and contained volume grows, filling can resume.
    c.balloonVolumeMl=3;prepareBalloonInflation(c,60,1/60);
    assert.ok(Math.abs(c.balloonDriveMl-3.14)<1e-10);
});

test('catheter stiffness sliders do not amplify balloon membrane tension',()=>{
    for(const distance of [16,20,44,70]) {
        const t=Math.max(0,Math.min(1,(distance-15)/8)),scale=100*Math.pow(40.65/100,t*t*(3-2*t));
        const base=balloonMaterialProfile(0).sample(distance).EI1;
        const nominal=balloonMaterialProfile(3).sample(distance).EI1-base;
        const scaled=(balloonMaterialProfile(3,40.65,100).sample(distance).EI1-base)*scale;
        assert.ok(Math.abs(nominal-scaled)<1e-7);
    }
});

test('cancelling a private balloon solve drops coverage scratch and restores the incoming rods',async()=>{
    const {iterateSharedAxisTimeStep}=await import('../src/physics/kirchhoffSharedAxisTimeStep.js');
    const {captureSharedAxisNative}=await import('../src/physics/kirchhoffSharedAxisNative.js');
    const {s,geometry}=planeFixture(3),before=captureSharedAxisNative(s);
    try {
        const iterator=iterateSharedAxisTimeStep(s,1/60,{vesselElasticity:DEFAULT_VESSEL_ELASTICITY});
        const next=iterator.next();assert.equal(next.done,false);assert.ok(s.contactCoverageCache);
        iterator.return();assert.equal(s.contactCoverageCache,undefined);
        const after=captureSharedAxisNative(s);
        assert.deepEqual(after.positions,before.positions);assert.deepEqual(after.frames,before.frames);
        assert.deepEqual(Array.from(after.multipliers.slice(0,before.multipliers.length)),Array.from(before.multipliers));
    }finally{geometry.dispose();}
});

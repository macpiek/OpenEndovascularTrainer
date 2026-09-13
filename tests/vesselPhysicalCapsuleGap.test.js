import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {Vector3} from 'three';
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader.js';
import {MeshBVH} from 'three-mesh-bvh';
import {generateVessel} from '../src/vesselGeometry.js';
import {transformAortaGeometry} from '../src/aortaTransform.js';
import {VesselContactField,createContactResult} from '../src/physics/collision/vesselContactField.js';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';
const buffer=path=>{const b=fs.readFileSync(path);return b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength);};
const asset=decodeCollisionAsset(buffer('res/Aorta_plain.collision.bin'));
const geometry=new STLLoader().parse(buffer('res/Aorta_plain.stl'));
transformAortaGeometry(geometry,generateVessel(140,0).vessel);geometry.boundsTree=new MeshBVH(geometry);
const fixture=()=>new VesselContactField(asset,{fallbackGeometry:geometry,bvhValidationDistance:.02,capsuleBvhValidation:-.1});
const base=[[-2.639277458190918,-233.7256317138672,.8334330916404724],[-4.30875301361084,-232.91119384765625,3.4974892139434814]];
const rejected=[[-2.6413328647613525,-233.72775268554688,.832927942276001],[-4.30723762512207,-232.90821838378906,3.4975295066833496]];
const radius=.8333333134651184;
function query(field,fraction,physical,points=null,samples=0) {
    points??=base.map((p,j)=>p.map((v,i)=>v+fraction*(rejected[j][i]-v)));
    const arrays=[0,1,2].map(i=>Float64Array.from(points,p=>p[i]));
    const c=field.queryCapsuleSoA(...arrays,new Float64Array([radius,radius]),0,createContactResult(),-1,false,false,354,false,samples?Math.hypot(...points[1].map((v,i)=>v-points[0][i])):-1,samples,physical);
    return {gap:c.signedGap,t:c.segmentT,branch:c.branchId,face:c.faceIndex,source:c.source,samples:c.capsuleSampleCount};
}

test('physical capsule removes the captured safe-core threshold jump without changing default queries',()=>{
    const field=fixture(),before=query(field,0),after=query(field,1);
    assert.equal(before.source,'centerline-safe-core');assert.equal(before.t,.5);assert.equal(before.branch,354);
    assert.ok(Math.abs(before.gap-.12512111240452573)<1e-10);
    assert.equal(after.source,'sparse-sdf');assert.equal(after.t,1);assert.ok(Math.abs(after.gap-1.0683527270739432)<1e-10);
    const split=.4301057104317,rows=[];
    for(const epsilon of [1e-3,1e-4,1e-5,1e-6,1e-7]) {
        const lo=query(field,split-epsilon,true),hi=query(field,split+epsilon,true);
        assert.notEqual(lo.source,'centerline-safe-core');assert.notEqual(hi.source,'centerline-safe-core');
        assert.equal(lo.samples,2);assert.equal(hi.samples,2);
        rows.push(Math.abs(hi.gap-lo.gap));
    }
    assert.ok(rows.at(-1)<1e-8,JSON.stringify(rows));
    assert.ok(rows.at(-1)<rows[0]*.001);
    assert.deepEqual(query(field,0,false),before,'explicit false and implicit default remain identical after physical queries');
});

test('endpoint cache cannot cross physical/default modes, including coincident capsules',()=>{
    const point=base[0].map((v,i)=>(v+base[1][i])/2),points=[point,point];
    const expectedDefault=query(fixture(),0,false,points),expectedPhysical=query(fixture(),0,true,points);
    assert.equal(expectedDefault.source,'centerline-safe-core');assert.notEqual(expectedPhysical.source,'centerline-safe-core');
    const field=fixture();
    for(const physical of [false,true,true,false,false,true])
        assert.deepEqual(query(field,0,physical,points),physical?expectedPhysical:expectedDefault);
    const ordinary=field.queryCapsuleCoordinates(...point,...point,radius,createContactResult());
    assert.equal(ordinary.source,expectedDefault.source);assert.equal(ordinary.signedGap,expectedDefault.gap);
    query(field,0,true,points);
    field.resetStats();query(field,0,true,points);
    assert.equal(field.getStats().capsuleSamples,0,'same-mode endpoint reuse remains available');
});

test('physical capsule evaluates every requested existing sample',()=>{
    const field=fixture();field.resetStats();
    const result=query(field,0,true,null,8);
    assert.equal(result.samples,8);assert.equal(field.getStats().capsuleSamples,9);
});

test('exception restores transient physical/refinement modes for subsequent ordinary query',()=>{
    const field=fixture(),expected=query(fixture(),0,false);
    const broken={get boundsTree(){throw new Error('intentional geometry failure');}};
    field.setFallbackGeometry(broken);
    // Large radius forces BVH refinement through public input/geometry only.
    const arrays=[0,1,2].map(i=>Float64Array.from(base,p=>p[i]));
    assert.throws(()=>field.queryCapsuleSoA(...arrays,new Float64Array([3,3]),0,createContactResult(),-1,false,false,354,false,-1,0,true),/intentional/);
    field.setFallbackGeometry(geometry);
    assert.deepEqual(query(field,0,false),expected);
});

test('mesh reference for step663 exposes SDF penetration artifact and has a consistent derivative',()=>{
    const field=fixture();
    // Reference configuration, not the application default: enabling this
    // band alone failed the full nonlinear browser cycle.
    const meshField=new VesselContactField(asset,{fallbackGeometry:geometry,bvhValidationDistance:.2,capsuleBvhValidation:.2});
    const points=[[-3.9944071769714355,-234.25193786621094,9.069884300231934],[-4.901269912719727,-236.12643432617188,11.50005054473877]];
    const end=[[-3.9944465160369873,-234.25192260742188,9.069734573364258],[-4.901449680328369,-236.12643432617188,11.499829292297363]];
    const sample=(p,physical,finite=false)=>{
        const c=(physical&&!finite?meshField:field).queryCapsuleSoA(...[0,1,2].map(i=>Float64Array.from(p,v=>v[i])),new Float64Array([radius,radius]),0,createContactResult(),-1,false,false,351,false,-1,0,physical,finite);
        return {gap:c.signedGap,t:c.segmentT,face:c.faceIndex,source:c.source,normal:Array.from(c.inward.values)};
    };
    const ordinary=sample(points,false);
    assert.equal(ordinary.source,'sparse-sdf');assert.equal(ordinary.face,-1);
    assert.ok(Math.abs(ordinary.gap-(-.004506160943153037))<1e-10);
    for(const [p,expectedGap] of [[points,.016040404360703198],[end,.015980032494756324]]) {
        const actual=sample(p,true);
        assert.deepEqual(sample(p,true,true),actual,'explicit all-sample finite mode matches the independent reference at this captured contact');
        assert.equal(actual.source,'sparse-sdf-bvh');assert.equal(actual.face,325170);assert.equal(actual.t,1);
        assert.ok(Math.abs(actual.gap-expectedGap)<1e-10);
        const h=1e-6,gradient=[0,1,2].map(axis=>{
            const plus=p.map(v=>v.slice()),minus=p.map(v=>v.slice());
            plus[1][axis]+=h;minus[1][axis]-=h;
            const a=sample(plus,true),b=sample(minus,true);
            assert.equal(a.face,actual.face);assert.equal(b.face,actual.face);
            return(a.gap-b.gap)/(2*h);
        });
        assert.ok(Math.abs(Math.hypot(...gradient)-1)<1e-7);
        gradient.forEach((v,i)=>assert.ok(Math.abs(v-actual.normal[i])<1e-7,`${i}: ${v} vs ${actual.normal[i]}`));
    }
    assert.deepEqual(sample(points,false),ordinary,'separate mesh reference must not change default query');
});

test('finite mesh capsule compares exact triangle distances at every sample, with separate endpoint cache',()=>{
    const field=fixture(),samples=8;
    const arrays=[0,1,2].map(i=>Float64Array.from(base,p=>p[i]));
    const run=(finite)=>field.queryCapsuleSoA(...arrays,new Float64Array([radius,radius]),0,
        createContactResult(),-1,false,false,354,false,4,samples,true,finite);
    const ordinary=run(false),coarse={gap:ordinary.signedGap,face:ordinary.faceIndex};
    const exact=run(true);
    let minimum=Infinity,winner=-1;
    const point=new Vector3(),hit={point:point.clone(),distance:Infinity,faceIndex:-1};
    for(let sample=0;sample<=samples;sample++){
        const t=sample/samples;
        point.set(...base[0].map((v,i)=>v+t*(base[1][i]-v)));
        geometry.boundsTree.closestPointToPoint(point,hit);
        if(hit.distance<minimum){minimum=hit.distance;winner=sample;}
    }
    assert.ok(exact.faceIndex>=0);
    assert.ok(Math.abs(exact.signedGap-(minimum-radius))<1e-10);
    assert.equal(exact.segmentT,winner/samples);
    const again=run(false);
    assert.deepEqual({gap:again.signedGap,face:again.faceIndex},coarse);
    assert.throws(()=>new VesselContactField(asset).queryCapsuleSoA(...arrays,
        new Float64Array([radius,radius]),0,createContactResult(),-1,false,false,-1,false,-1,0,true,true),/mesh BVH/);
});

test('optional capsule visitor exposes every already evaluated finite sample without extra geometry queries',()=>{
    const ordinary=fixture(),visited=fixture(),samples=8;
    const snapshot=c=>({values:Array.from(c.values),face:c.faceIndex,source:c.source,normal:Array.from(c.inward.values)});
    const call=(field,points,visitor=null)=>field.queryCapsuleSoA(...[0,1,2].map(k=>Float64Array.from(points,p=>p[k])),new Float64Array([radius,radius]),0,
        createContactResult(),-1,false,false,354,false,4,samples,true,true,visitor);
    for(const points of [base,[base[1],base[1].map((v,k)=>v+base[1][k]-base[0][k])]]) {
        const collected=[],a=snapshot(call(ordinary,points)),b=snapshot(call(visited,points,(c,t)=>collected.push({t,...snapshot(c)})));
        assert.deepEqual(a,b);assert.equal(collected.length,samples+1);
        assert.deepEqual(collected.map(c=>c.t).sort((a,b)=>a-b),Array.from({length:samples+1},(_,i)=>i/samples));
        assert.ok(collected.every(c=>c.face>=0));
        const statsA=ordinary.getStats(),statsB=visited.getStats();
        for(const key of ['capsuleSamples','bvhRefinements','bvhContactRefinements'])assert.equal(statsA[key],statsB[key],key);
    }
});

test('a capsule visitor exception restores query flags and invalidates its partial endpoint cache',()=>{
    const field=fixture(),arrays=[0,1,2].map(k=>Float64Array.from(base,p=>p[k]));
    assert.throws(()=>field.queryCapsuleSoA(...arrays,new Float64Array([radius,radius]),0,createContactResult(),-1,false,false,354,false,4,8,true,true,()=>{throw new Error('visitor interrupted');}),/visitor interrupted/);
    assert.deepEqual(query(field,0,false),query(fixture(),0,false));
});

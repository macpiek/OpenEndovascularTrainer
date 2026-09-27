import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {rowExposure,fullyOpenDistance,graftFaceExposed} from '../src/devices/stentGraftDeployment.js';
import {packedFrames,releaseSection} from '../src/devices/stentGraftReleaseShape.js';
import {fabricPoint} from '../src/devices/stentGraftScaffold.js';

for(const side of ['right','left'])test(`${side}: release follows the sheath edge, pauses, and requires a separate tip release`,()=>{
    const {system,device:d,sources}=previewFixture(side);
    const step=(dt,release,id=d.id)=>system.updateAccess(side,dt,null,{deviceId:id,release});
    try {
        const folded=d.parts.map(p=>Array.from(p.mesh.geometry.attributes.position.array));
        step(5,null);step(0,'sheath');step(5,'sheath',d.id+1);
        assert.equal(d.sheathWithdrawal,0);assert.deepEqual(d.parts.map(p=>Array.from(p.mesh.geometry.attributes.position.array)),folded);
        step(3,'sheath');assert.equal(d.sheathWithdrawal,36);
        assert.equal(rowExposure(d,0),1);assert.equal(rowExposure(d,26),0);
        const distal=d.parts[1],p=distal.mesh.geometry.attributes.position;
        const mean=new THREE.Vector3();for(let j=0;j<distal.sides;j++)mean.add(new THREE.Vector3().fromBufferAttribute(p,j));mean.divideScalar(distal.sides);
        assert.ok(mean.distanceTo(distal.folded[0])<1e-5,'covered ipsilateral limb encloses the delivery axis');
        assert.deepEqual(Array.from(distal.mesh.geometry.attributes.position.array),folded[1],'distal limb cannot expand with the proximal trunk');
        step(10,null); // Uncovered rings finish opening independently of the stopped cover.
        const frozen=d.parts.map(p=>Array.from(p.mesh.geometry.attributes.position.array));
        step(20,null);assert.equal(d.sheathWithdrawal,36);
        assert.deepEqual(d.parts.map(p=>Array.from(p.mesh.geometry.attributes.position.array)),frozen);
        sources[side].nodes.at(-1).x+=15;
        system.refreshDelivery(side);
        assert.ok(d.sheathMarker.position.distanceTo(d.deliveryPath.sample(d.position+d.coverLead-36))<1e-6);
        step(100,'sheath');assert.equal(d.releaseStage,'tip');assert.equal(d.tipRelease,0);
        assert.equal(d.sheathWithdrawal,d.sheathTravel);assert.ok(d.sheathWithdrawal>d.gateTravel);
        assert.equal(d.gate.marker.visible,true);assert.equal(system.surface,null,'no premature flow seal');
        const stopped=d.sheathWithdrawal;step(100,'sheath');assert.equal(d.sheathWithdrawal,stopped);
        step(.4,'tip');assert.equal(d.tipRelease,.4);step(20,null);assert.equal(d.tipRelease,.4);
        step(1,'tip');assert.equal(d.tipRelease,1);assert.equal(d.phase,'deployed');assert.equal(d.sheathWithdrawal,stopped);
        const implanted=d.parts.map(p=>Array.from(p.mesh.geometry.attributes.position.array));
        const surface=system.surface;step(100,'resheath');assert.equal(d.sheathWithdrawal,0);
        assert.equal(d.deployment,1);assert.equal(system.surface,surface);
        assert.deepEqual(d.parts.map(p=>Array.from(p.mesh.geometry.attributes.position.array)),implanted,'detached implant cannot be pulled back into the cover');
        step(.1,'sheath');assert.ok(d.sheathWithdrawal>0);
        for(const part of d.parts) {
            assert.ok(part.rings.isLineSegments2);assert.ok(part.rings.count>0);
            assert.ok(Array.from(part.rings.geometry.attributes.instanceStart.data.array).every(Number.isFinite));
            assert.ok(part.scaffoldSegments.some(([a,b])=>a.s!==b.s),'stents have a zigzag profile');
        }
        assert.equal(d.crown.name,'suprarenal-capture-crown');assert.ok(d.parts[0].orientationMarker);
    } finally {system.dispose();}
});

test('opposed folded and expanded frames retain an open, round intermediate ring',()=>{
    const points=[new THREE.Vector3(0,0,0),new THREE.Vector3(0,0,2)];
    const frames=packedFrames(points),part={points,folded:points,sides:24,ringFrames:frames,target:new Float32Array(144)};
    for(let i=0;i<2;i++)for(let j=0;j<24;j++)points[i].clone()
        .addScaledVector(frames[i].u,10*Math.cos(j/24*2*Math.PI))
        .addScaledVector(frames[i].v,10*Math.sin(j/24*2*Math.PI)).toArray(part.target,(i*24+j)*3);
    const reversed={u:frames[0].u.clone().negate(),v:frames[0].v.clone().negate(),tangent:frames[0].tangent};
    const section=releaseSection(part,0,.5,reversed,{radius:1.7,lateral:0});
    for(let j=0;j<24;j++)assert.ok(Math.abs(section.point(j).distanceTo(section.center)-5.85)<1e-5);
});

for(const side of ['left','right'])test(`${side}: freed gate springs open with a stationary sheath and enables only matching contacts`,()=>{
    const {system,device:d}=previewFixture(side);
    const step=(dt,release)=>system.updateAccess(side,dt,null,{deviceId:d.id,release});
    try {
        const gate=d.parts[2],rim=()=>fabricPoint(gate,gate.path.length,0);
        const travel=d.coverLead+gate.releaseOffset+gate.path.length;
        step((travel-.5)/12,'sheath');
        assert.equal(gate.exposure.at(-1),0,'free end remains captured before clearance');
        step(1,null);assert.equal(gate.exposure.at(-1),0,'waiting cannot release a covered gate');
        assert.ok(d.contactFaces.some(f=>f.gate));
        assert.ok(d.contactFaces.filter(f=>f.gate).some(f=>graftFaceExposed(d,f)),'already uncovered gate rings have contact before its end is freed');
        step(.5/12,'sheath');
        const withdrawal=d.sheathWithdrawal;
        step(0,null);assert.equal(gate.exposure.at(-1),0,'paused simulation cannot advance the spring');
        step(1/60,null);assert.ok(gate.exposure.at(-1)>0&&gate.exposure.at(-1)<1);
        assert.ok(d.contactFaces.filter(f=>f.gate).some(f=>graftFaceExposed(d,f)),'contact follows the opening cloth');
        step(10,null);
        assert.equal(d.sheathWithdrawal,withdrawal,'opening no longer needs further cover motion');
        assert.equal(gate.exposure.at(-1),1);
        assert.ok(d.contactFaces.filter(f=>f.gate).every(f=>graftFaceExposed(d,f)));
        const marker=new THREE.Vector3().fromBufferAttribute(d.gate.marker.geometry.attributes.instanceStart,0);
        assert.ok(marker.distanceTo(rim())<1e-5,'rim marker follows the opening');
        assert.ok(gate.exposure.every(value=>value===1),'the whole gate is free even while its neighbour is still covered');
        step(100,'resheath');assert.equal(d.gateOpening,0);
        assert.ok(gate.exposure.every(x=>x===0),'cover can recapture before implant detachment');
        step(100,'sheath');
        for(const part of d.parts.slice(1))assert.deepEqual(part.mesh.geometry.attributes.position.array,part.target);
        step(1,'tip'); // The captured proximal fabric is still mechanically restrained until this release.
        for(const part of d.parts)assert.deepEqual(part.mesh.geometry.attributes.position.array,part.target);
    }finally{system.dispose();}
});

test('standalone limb uses the same pausable sheath release without a suprarenal stage',()=>{
    const {system,device:d}=previewFixture('left','limb');
    try {
        system.updateAccess('left',2,null,{deviceId:d.id,release:'sheath'});
        assert.equal(d.sheathWithdrawal,24);assert.equal(d.tipRelease,1);assert.equal(d.crown,undefined);
        system.updateAccess('left',30);assert.equal(d.sheathWithdrawal,24);assert.equal(d.phase,'deploying');
    } finally {system.dispose();}
});

for(const side of ['right','left'])test(`${side}: cover reverses, updates contacts, and operates independently of capture`,()=>{
    const {system,device:d}=previewFixture(side);
    const step=(dt,release)=>system.updateAccess(side,dt,null,{deviceId:d.id,release});
    try {
        const initial=d.parts.map(p=>Array.from(p.mesh.geometry.attributes.position.array));
        step(3,'sheath');const opened=system.mechanicalSurface;
        assert.ok(opened);const old=Array.from(opened.geometry.attributes.position.array);
        step(1,'resheath');assert.equal(d.sheathWithdrawal,24);assert.equal(d.tipRelease,0);
        assert.ok((system.mechanicalSurface?.geometry.attributes.position.count??0)<old.length/3);
        assert.deepEqual(Array.from(opened.geometry.attributes.position.array),old,'suspended step keeps immutable contacts');
        step(100,'resheath');assert.equal(d.sheathWithdrawal,0);assert.equal(d.deployment,0);
        assert.equal(system.mechanicalSurfaceForAccess(side),null);
        assert.deepEqual(d.parts.map(p=>Array.from(p.mesh.geometry.attributes.position.array)),initial);
        step(.4,'tip');assert.equal(d.tipRelease,.4);assert.equal(d.sheathWithdrawal,0);
        step(.25,{sheath:1,tip:true});assert.equal(d.sheathWithdrawal,3);assert.equal(d.tipRelease,.65);
        step(1,'tip');assert.equal(d.tipRelease,1);assert.equal(d.phase,'deploying','capture release does not move the cover');
        step(100,'sheath');assert.equal(d.phase,'deployed');assert.ok(system.surface);
    } finally {system.dispose();}
});

for(const side of ['right','left'])test(`${side}: suprarenal peaks stay captured until the latch releases, then snap open`,()=>{
    const {system,device:d}=previewFixture(side);
    const step=(dt,release)=>system.updateAccess(side,dt,null,{deviceId:d.id,release});
    const crown=()=>d.crownPolylines.flatMap(line=>line.flatMap(p=>p.toArray()));
    try {
        step(100,'sheath');
        const held=crown();
        step(.5,'tip');assert.equal(d.tipRelease,.5);assert.deepEqual(crown(),held,'half capture travel must not partially expand struts');
        step(5,null);assert.deepEqual(crown(),held,'pausing the latch keeps peaks captured');
        step(.49,'tip');assert.equal(d.tipRelease,.99);assert.deepEqual(crown(),held);
        step(0,'tip');assert.deepEqual(crown(),held,'uncommitted time cannot trigger release');
        step(.01,'tip');assert.equal(d.tipRelease,1);assert.notDeepEqual(crown(),held);
        const peaks=d.crownPolylines.filter((_,i)=>i%3!==2).map(line=>line.at(-1));
        const center=peaks.reduce((sum,p)=>sum.add(p),new THREE.Vector3()).divideScalar(peaks.length);
        assert.ok(peaks.every(p=>p.distanceTo(center)>8),'peaks spring out around the lumen after detachment');
        const open=crown();step(10,null);assert.deepEqual(crown(),open,'expanded crown remains open without holding L');
    }finally{system.dispose();}
});

for(const side of ['right','left'])test(`${side}: ipsilateral rim springs open at sleeve clearance without extra withdrawal`,()=>{
    const {system,device:d}=previewFixture(side);
    const step=(dt,release)=>system.updateAccess(side,dt,null,{deviceId:d.id,release});
    try {
        const ipsi=d.parts[1],travel=d.coverLead+ipsi.releaseOffset+ipsi.path.length;
        step((travel-.5)/12,'sheath');
        assert.equal(ipsi.exposure.at(-1),0);
        assert.ok(ipsi.exposure.some(x=>x>0),'upstream rings expand progressively');
        step(.5/12,'sheath');const withdrawal=d.sheathWithdrawal;
        step(0,null);assert.equal(ipsi.exposure.at(-1),0);
        const tail=d.contactFaces.filter(f=>f.ipsilateral&&!f.gate&&f.distance>fullyOpenDistance(d));
        assert.ok(tail.length);
        const covered=tail.filter(f=>f.bindings.some(b=>b.indices.some(index=>b.part.exposure[Math.floor(index/b.part.sides)]===0)));
        assert.ok(covered.length);assert.ok(covered.every(f=>!graftFaceExposed(d,f)),'covered tail remains inactive; already exposed cloth can collide');
        step(1/60,null);assert.ok(ipsi.exposure.at(-1)>0&&ipsi.exposure.at(-1)<1);
        step(10,null);assert.equal(d.ipsiOpening,1);assert.equal(ipsi.exposure.at(-1),1);
        assert.equal(d.sheathWithdrawal,withdrawal,'spring runs with the cover stopped');
        assert.equal(d.phase,'deploying');assert.equal(d.tipRelease,0,'proximal latch remains independently attached');
        assert.deepEqual(ipsi.mesh.geometry.attributes.position.array,ipsi.target);
        assert.ok(tail.every(f=>graftFaceExposed(d,f)),'contacts include the freed distal end');
        step(1,'resheath');assert.equal(d.ipsiOpening,0);assert.equal(ipsi.exposure.at(-1),0);
        step(1,'sheath');step(10,null);step(1,'tip');
        assert.equal(d.phase,'deployed','no extra numerical contact margin is required beyond physical sleeve clearance');
        assert.equal(d.sheathWithdrawal,withdrawal);
    }finally{system.dispose();}
});

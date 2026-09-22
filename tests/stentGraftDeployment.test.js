import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {rowExposure} from '../src/devices/stentGraftDeployment.js';

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
        assert.ok(mean.distanceTo(distal.folded[0])<1e-5,'covered limb stays on the delivery axis');
        assert.deepEqual(Array.from(distal.mesh.geometry.attributes.position.array),folded[1],'distal limb cannot expand with the proximal trunk');
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
        assert.ok(system.mechanicalSurface.geometry.attributes.position.count<old.length/3);
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

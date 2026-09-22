import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {DevicePath} from '../src/devices/stentGraftPaths.js';
import {createFlexibleNoseGeometry,NOSECONE_LENGTH_MM} from '../src/devices/stentGraftNose.js';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';

test('long tapered nose follows a curved wire and updates when the wire bends',()=>{
    const path=new DevicePath(Array.from({length:81},(_,i)=>new THREE.Vector3(40*Math.sin(i/40),40*(1-Math.cos(i/40)),0)));
    const geometry=createFlexibleNoseGeometry(path,5);
    try {
        assert.equal(NOSECONE_LENGTH_MM,60);
        const p=geometry.attributes.position,sides=16,rows=60;
        const centers=[];
        for(let i=0;i<=rows;i++) {
            const center=new THREE.Vector3();for(let j=0;j<sides;j++)center.add(new THREE.Vector3().fromBufferAttribute(p,i*sides+j));center.divideScalar(sides);centers.push(center);
            assert.ok(center.distanceTo(path.sample(5+i))<1e-5);
            const radius=center.distanceTo(new THREE.Vector3().fromBufferAttribute(p,i*sides));
            assert.ok(Math.abs(radius-(.12+2.38*Math.pow(1-i/rows,1.15)))<1e-5);
        }
        assert.ok(centers[12].distanceTo(centers[0].clone().lerp(centers.at(-1),12/60))>1,'nose bends instead of remaining a straight cone');
        assert.ok(Array.from(geometry.attributes.normal.array).every(Number.isFinite));
        for(const point of path.points)point.z=point.x*.2;
        const changed=createFlexibleNoseGeometry(path,5);assert.notDeepEqual(changed.attributes.position.array,p.array);changed.dispose();
    } finally {geometry.dispose();}
});

test('scaffold struts are thinner and delivery uses the flexible 60 mm nose mesh',()=>{
    const {system,device:d}=previewFixture();
    try {
        assert.equal(d.parts[0].rings.userData.wireRadius,.045);
        assert.equal(d.crown.userData.wireRadius,.055);
        system.refreshDelivery('right');assert.equal(d.noseMarker.geometry.type,'BufferGeometry');
        assert.ok(d.noseMarker.geometry.attributes.position.count>400);
        const positions=Array.from(d.noseMarker.geometry.attributes.position.array);
        assert.ok(positions.every(Number.isFinite));
    } finally {system.dispose();}
});

for(const [side,type]of [['right','body'],['left','limb']])test(`${side} ${type}: packed metal and fabric follow insertion without creating an implant or duplicate release meshes`,()=>{
    const {system,device:d}=previewFixture(side,type,false);
    try {
        system.refreshDelivery(side);
        assert.equal(d.phase,'loaded');assert.equal(system.implants.length,0);assert.equal(system.surface,null);assert.equal(system.mechanicalSurface,null);
        assert.ok(d.foldedPreview.length===(type==='body'?4:1));
        for(const part of d.foldedPreview){
            assert.equal(part.rings.visible,true);assert.ok(part.rings.count>0);
            assert.ok(Array.from(part.mesh.geometry.attributes.position.array).every(Number.isFinite));
            assert.ok(part.mesh.visible||part.crown);
        }
        const part=d.foldedPreview[0],before=Array.from(part.mesh.geometry.attributes.position.array);
        d.position-=10;system.refreshDelivery(side);
        assert.notDeepEqual(Array.from(part.mesh.geometry.attributes.position.array),before);
        system.setFluoroscopy(true);assert.equal(system.fabric.visible,false);assert.ok(system.metal.visible&&part.rings.visible,'metal remains visible in X-ray');
        const objects=d.foldedPreview.flatMap(p=>[p.mesh,p.rings,p.markers]);
        assert.equal(system.deploy(side).ok,true);assert.equal(d.foldedPreview,null);
        assert.ok(objects.every(o=>o.parent===null),'preview removed before real deployment parts are drawn');
        assert.equal(system.implants.length,1);
    } finally {system.dispose();}
});

test('withdrawn preview is hidden, reappears on insertion, and removal disposes only its geometry',()=>{
    const {system,device:d}=previewFixture('right','body',false);
    try {
        system.refreshDelivery('right');const parts=d.foldedPreview,metal=system.metalMaterial;
        let disposed=0;parts[0].rings.geometry.addEventListener('dispose',()=>disposed++);
        d.position=0;system.refreshDelivery('right');assert.ok(parts.every(p=>!p.mesh.visible&&!p.rings.visible&&!p.markers.visible));
        d.position=25;system.refreshDelivery('right');assert.ok(parts.every(p=>p.rings.visible));
        d.position=0;assert.ok(system.removeDelivery('right').ok);assert.equal(disposed,1);
        assert.ok(parts.every(p=>p.rings.parent===null));assert.equal(system.metalMaterial,metal);
    } finally {system.dispose();}
});

test('nose retracts on committed time to the moving sheath edge without displacing the implant',async()=>{
    const {deliveryNoseState}=await import('../src/devices/stentGraftDeployment.js');
    const {system,device:d}=previewFixture();
    const step=(dt,release,id=d.id)=>system.updateAccess('right',dt,null,{deviceId:id,release});
    try {
        step(3,'sheath');step(1,'nose');assert.equal(d.noseRetraction,0,'captured fixation cannot be pulled through its own implant');
        step(1,'tip');
        const position=d.position,cover=d.sheathWithdrawal;
        const fabric=d.parts.map(p=>Array.from(p.mesh.geometry.attributes.position.array));
        const before=deliveryNoseState(d);
        step(0,'nose');step(1,'nose',d.id+1);assert.equal(d.noseRetraction,0);
        step(.5,'nose');assert.equal(d.noseRetraction,6);
        assert.equal(deliveryNoseState(d).position,before.position-6);
        step(5,null);assert.equal(d.noseRetraction,6,'releasing the button pauses movement');
        step(100,'nose');assert.equal(deliveryNoseState(d).remaining,0);
        assert.equal(deliveryNoseState(d).position,deliveryNoseState(d).sheathEdge);
        assert.equal(d.position,position);assert.equal(d.sheathWithdrawal,cover);
        assert.deepEqual(d.parts.map(p=>Array.from(p.mesh.geometry.attributes.position.array)),fabric);
        system.refreshDelivery('right');
        const p=d.noseMarker.geometry.attributes.position,center=new THREE.Vector3();
        for(let j=0;j<16;j++)center.add(new THREE.Vector3().fromBufferAttribute(p,j));center.divideScalar(16);
        assert.ok(center.distanceTo(d.sheathMarker.position)<1e-5,'cone base actually meets the rendered cover marker');
        step(1,'sheath');assert.equal(deliveryNoseState(d).remaining,12,'pulling the cover farther creates another gap');
        step(1,'nose');assert.equal(deliveryNoseState(d).remaining,0);
        step(1,'resheath');assert.equal(deliveryNoseState(d).remaining,0,'advancing cover cannot pass through the nose base');
    } finally {system.dispose();}
});

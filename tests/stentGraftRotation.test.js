import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {MAIN_BODY_MODELS,distalDiameters,proximalDiameters} from '../src/devices/stentGraftModels.js';
import {renderMetalProjection} from '../src/imaging/renderMetalProjection.js';

for(const side of ['right','left'])test(`${side}: committed roll moves the attached gate and contacts; released graft stays fixed`,()=>{
    const {system,device:d}=previewFixture(side,'body',false);
    const step=(angle,release=null,dt=.1,id=d.id)=>system.updateAccess(side,dt,null,{deviceId:id,mechanicalRotation:angle,release});
    const positions=()=>d.parts.map(p=>Array.from(p.mesh.geometry.attributes.position.array));
    try {
        step(.3,null,0);assert.equal(d.deliveryRotation,0);
        step(.3,null,.1,d.id+1);assert.equal(d.deliveryRotation,0);
        step(.3);system.refreshDelivery(side);assert.equal(d.graftRotation,.3);
        const folded=Array.from(d.foldedPreview[1].mesh.geometry.attributes.position.array);
        step(.8);system.refreshDelivery(side);assert.notDeepEqual(Array.from(d.foldedPreview[1].mesh.geometry.attributes.position.array),folded);
        system.deploy(side);step(.8,'sheath',7);
        const before=positions(),gate=d.gate.entry.clone(),snapshot=system.mechanicalSurface,old=Array.from(snapshot.geometry.attributes.position.array);
        step(1.2);assert.equal(d.graftRotation,1.2);assert.ok(d.gate.entry.distanceTo(gate)>1);
        assert.notDeepEqual(positions(),before);assert.notEqual(system.mechanicalSurface,snapshot);
        assert.deepEqual(Array.from(snapshot.geometry.attributes.position.array),old,'a preceding Newton snapshot stays immutable');
        assert.notDeepEqual(Array.from(system.mechanicalSurface.geometry.attributes.position.array),old);
        step(1.2,'tip',1);assert.equal(d.tipRelease,1);assert.equal(d.phase,'deploying');
        const frozenTargets=d.parts.map(p=>Array.from(p.target)),frozenGate=d.gate.entry.clone(),revision=d.poseRevision;
        step(2);assert.equal(d.deliveryRotation,2);assert.equal(d.graftRotation,1.2);assert.equal(d.poseRevision,revision);
        assert.deepEqual(d.parts.map(p=>Array.from(p.target)),frozenTargets);assert.deepEqual(d.gate.entry,frozenGate);
        step(2,'sheath',100);const implanted=positions(),surface=system.surface;
        step(-1);assert.deepEqual(positions(),implanted);assert.equal(system.surface,surface);
        system.refreshDelivery(side);const marker=d.rotationMarker.position.clone();
        step(-.5);system.refreshDelivery(side);assert.ok(d.rotationMarker.position.distanceTo(marker)>.1);
    } finally {system.dispose();}
});

test('catalogue lengths drive packed and expanded body material coordinates, not the anatomical bifurcation',()=>{
    for(const model of MAIN_BODY_MODELS) {
        const {system,device:d}=previewFixture('right','body',false,model.id);
        try {
            system.refreshDelivery('right');assert.equal(d.foldedPreview[1].offset+d.foldedPreview[1].length,model.length);
            system.deploy('right');assert.equal(d.releaseLength,model.length);
            assert.equal(d.parts[2].releaseOffset+d.parts[2].path.length,model.gateLength);
            assert.ok(d.parts[2].path.length>=30);assert.ok(d.parts[1].path.length>d.parts[2].path.length);
            assert.equal(d.gateTravel,12+model.gateLength+2);
        } finally {system.dispose();}
    }
    assert.ok(!proximalDiameters('ii-124').includes(36));assert.deepEqual(distalDiameters('iis-103',28),[14]);
});

test('continuous wire shader survives the X-ray material pass and has finite joined endpoints',()=>{
    const {system,device:d}=previewFixture();
    try {
        const part=d.parts[0],wire=part.rings,scene=new THREE.Scene(),camera=new THREE.OrthographicCamera(-50,50,50,-50,.1,1000);
        system.updateAccess('right',100,null,{deviceId:d.id,release:'sheath'});scene.add(system.group);
        const material=wire.material,projection=wire.userData.projectionMaterial;
        let drawn=false;
        const renderer={getCurrentViewport:v=>v.set(0,0,512,512),render(){
            drawn=true;assert.equal(wire.material,projection);
            wire.onBeforeRender(this,scene,camera,wire.geometry,wire.material);
            assert.ok(wire.material.linewidth>=1);assert.equal(wire.material.resolution.y,512);
        }};
        renderMetalProjection(renderer,scene,camera,{wireGroups:[],graftGroup:system.group,wireMaterial:system.metalMaterial,graftMaterial:system.metalMaterial});
        assert.ok(drawn);assert.equal(wire.material,material);
        const a=wire.geometry.attributes.instanceStart,b=wire.geometry.attributes.instanceEnd;
        for(let i=1;i<96;i++)assert.ok(new THREE.Vector3().fromBufferAttribute(a,i).distanceTo(new THREE.Vector3().fromBufferAttribute(b,i-1))<1e-5,'wire segments meet without gaps');
        assert.ok(Array.from(a.data.array).every(Number.isFinite));
    } finally {system.dispose();}
});

test('rotation of the partially released body stays inside the real aneurysm wall',async()=>{
    const {fixture,place}=await import('./helpers/stentGraftFixture.js');
    const {createContactResult}=await import('../src/physics/collision/vesselContactField.js');
    const f=fixture('Aorta_infrarenal_aneurysm');
    try {
        const {system}=f;place(system,'right','body');system.deploy('right');
        const d=system.accesses.right.device;
        system.updateAccess('right',7,null,{deviceId:d.id,release:'sheath'});
        system.updateAccess('right',.1,null,{deviceId:d.id,mechanicalRotation:Math.PI/2});
        const result=createContactResult();
        for(const part of d.parts)for(let i=0;i<part.target.length;i+=3) {
            const p=new THREE.Vector3().fromArray(part.target,i);
            assert.equal(f.contactField.querySphere(p,0,result).violation,false,'rotating the gate must not move fabric outside the vessel');
        }
        assert.ok(d.contactFaces.every(face=>face.positions.every(Number.isFinite)));
    } finally {f.dispose();}
});

for(const side of ['right','left'])test(`${side}: partial release permits translation and roll together; detachment freezes the implant`,()=>{
    const {system,device:d}=previewFixture(side);
    const initial=d.position;
    const step=(position,rotation,release=null,dt=.1,id=d.id)=>system.updateAccess(side,dt,null,{deviceId:id,mechanicalPosition:position,mechanicalRotation:rotation,release});
    try {
        step(initial,0,'sheath',3);
        const original=d.parts[0].points[0].clone(),gate=d.gate.entry.clone(),cover=d.sheathWithdrawal;
        const old=system.mechanicalSurface,oldPositions=Array.from(old.geometry.attributes.position.array);
        step(initial-8,.4,'sheath',0);assert.equal(d.position,initial);
        step(initial-8,.4,null,.1,d.id+1);assert.equal(d.position,initial);
        step(initial-8,.4);
        assert.equal(d.position,initial-8);assert.equal(d.implantPosition,initial-8);assert.equal(d.graftRotation,.4);
        assert.ok(d.parts[0].points[0].distanceTo(original)>5);assert.ok(d.gate.entry.distanceTo(gate)>2);
        assert.equal(d.sheathWithdrawal,cover,'moving the whole assembly keeps relative cover opening');
        assert.notEqual(system.mechanicalSurface,old);assert.deepEqual(Array.from(old.geometry.attributes.position.array),oldPositions);
        step(initial,0);assert.ok(d.parts[0].points[0].distanceTo(original)<1e-6,'return uses reference pose without cumulative distortion');
        step(initial,0,'tip',1);
        const targets=d.parts.map(p=>Array.from(p.target)),folded=d.parts.map(p=>p.folded.map(v=>v.toArray())),origin=d.implantPosition;
        const exposedBefore=system.mechanicalSurfaceForAccess(side).lumenSections.length;
        step(initial-10,.8);
        assert.equal(d.implantPosition,origin);assert.equal(d.graftRotation,0);assert.equal(d.deliveryRotation,.8);
        assert.deepEqual(d.parts.map(p=>Array.from(p.target)),targets);
        assert.deepEqual(d.parts.map(p=>p.folded.map(v=>v.toArray())),folded,'detached covered rows stay with the implant');
        assert.ok(system.mechanicalSurfaceForAccess(side).lumenSections.length>exposedBefore,'cover travels relative to a detached implant');
    } finally {system.dispose();}
});

test('subpixel wires retain physical coverage while folded radiopaque markers remain distinct',()=>{
    const {system,device:d}=previewFixture('right','body',false);
    try {
        system.refreshDelivery('right');
        const part=d.foldedPreview[0],camera=new THREE.OrthographicCamera(-110,110,110,-110,.1,1000);
        const renderer={getCurrentViewport:v=>v.set(0,0,300,600)};
        for(const mesh of [part.rings,part.markers])mesh.onBeforeRender(renderer,null,camera,mesh.geometry,mesh.userData.projectionMaterial);
        const thin=part.rings.userData.projectionMaterial,marker=part.markers.userData.projectionMaterial;
        assert.equal(thin.linewidth,1);assert.ok(thin.opacity<.3,'sampling footprint must not become an opaque pixel');
        assert.ok(Math.abs(thin.linewidth*thin.opacity-.09*600/220)<1e-6);
        assert.ok(marker.linewidth*marker.opacity>6*thin.linewidth*thin.opacity);
        assert.equal(part.markers.visible,true);assert.equal(part.markers.material.depthTest,false);
        assert.ok(part.markers.renderOrder>part.rings.renderOrder);
        const a=part.markers.geometry.attributes.instanceStart,b=part.markers.geometry.attributes.instanceEnd;
        assert.ok(new THREE.Vector3().fromBufferAttribute(a,0).distanceTo(new THREE.Vector3().fromBufferAttribute(b,0))>2,'packed markers are short longitudinal strips');
    } finally {system.dispose();}
});

test('contralateral outlet springs to the side even when both iliac routes share the aortic axis',()=>{
    const {system,device:d}=previewFixture('right','body',false);
    try {
        d.position+=25;d.target=d.position;system.deploy('right');
        const gate=d.parts[2],center=()=>{
            const p=gate.mesh.geometry.attributes.position,result=new THREE.Vector3();
            for(let j=0;j<gate.sides;j++)result.add(new THREE.Vector3().fromBufferAttribute(p,(gate.rows-1)*gate.sides+j));
            return result.divideScalar(gate.sides);
        };
        system.updateAccess('right',(d.gateTravel-2.1)/12,null,{deviceId:d.id,release:'sheath'});
        const folded=center();system.updateAccess('right',.3,null,{deviceId:d.id,release:'sheath'});
        assert.ok(center().distanceTo(folded)>5,'distal end opens laterally when it clears the cover');
        assert.ok(gate.path.sample(gate.path.length).distanceTo(d.parts[1].path.sample(gate.path.length))>14,'short and long outlets do not converge onto one axis');
        assert.equal(d.gate.marker.visible,true);
    } finally {system.dispose();}
});

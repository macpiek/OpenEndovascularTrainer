import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {createContactResult} from '../src/physics/collision/vesselContactField.js';
import {generateVessel} from '../src/vesselGeometry.js';
import {StentGraftSystem} from '../src/devices/stentGraftSystem.js';
import {wireDevicePath} from '../src/devices/stentGraftPaths.js';
import {HybridContrastSystem} from '../src/contrast/hybridContrastSystem.js';
import {ContrastVolumeRenderer} from '../src/contrast/contrastVolumeRenderer.js';

import {fixture,place,finish} from './helpers/stentGraftFixture.js';
const {vessel}=generateVessel();

test('wire follower interpolates material position and cannot extrapolate beyond the wire tip',()=>{
    const path=wireDevicePath({nodes:[{x:-10,y:0,z:0},{x:10,y:0,z:0},{x:20,y:10,z:0}],coordinate:i=>i*20-10});
    assert.equal(path.length,30);assert.deepEqual(path.sample(0).toArray(),[0,0,0]);
    assert.deepEqual(path.sample(20).toArray(),[15,5,0]);assert.deepEqual(path.sample(100).toArray(),[20,10,0]);
});

test('cooperative solver trials cannot move the delivery path until the rod step commits',()=>{
    const nodes=[new THREE.Vector3(0,0,0),new THREE.Vector3(0,20,0)];let feed=20;
    const source={nodes,coordinate:i=>i*feed,catheterMm:0};
    const system=new StentGraftSystem({readAccess:()=>source,readAnatomy:()=>null,sheaths:{}});
    try {
        system.updateAccess('right',0,source);
        nodes[1].x=8;feed=25;source.catheterMm=10;
        assert.deepEqual(system.getPath('right').sample(20).toArray(),[0,20,0]);
        assert.equal(system.getPath('right').length,20);assert.equal(system.catheterPosition('right'),0);
        system.updateAccess('right',1/60,source);
        assert.deepEqual(system.getPath('right').sample(25).toArray(),[8,20,0]);
        assert.equal(system.catheterPosition('right'),10);
    } finally {system.dispose();}
});

for(const variant of ['Aorta_plain','Aorta_infrarenal_aneurysm'])test(`${variant}: body and contralateral limb deploy through separate accesses and remain implanted`,()=>{
    const f=fixture(variant),{system,sources,contactField}=f;
    try {
        assert.equal(system.load('left','limb').ok,true);assert.equal(system.removeDelivery('left').ok,true);
        sources.right.catheterMm=50;assert.equal(system.load('right','body').ok,false);sources.right.catheterMm=0;
        place(system,'right','body');assert.equal(system.deploy('right').ok,true);
        assert.equal(system.load('left','limb').ok,true,'a limb can be selected before the body finishes');
        assert.equal(system.availableGate('left'),null,'an unfinished body must not advertise a connected gate');
        assert.equal(system.removeDelivery('left').ok,true);
        finish(system,'right');const body=system.implants[0];
        assert.equal(body.phase,'deployed');assert.equal(body.parts.length,3);
        assert.equal(system.load('right','limb').ok,false);
        const frozen=body.parts.map(p=>Array.from(p.mesh.geometry.attributes.position.array));
        place(system,'left','limb');assert.equal(system.deploy('left').ok,true);finish(system,'left');
        assert.equal(system.implants[1].parentId,body.id);assert.equal(body.connectedLimbId,system.implants[1].id);
        assert.equal(system.surface.sealed,true);
        const trunk=body.parts[0],center=trunk.points[Math.floor(trunk.rows/2)];
        assert.equal(system.surface.contains(center),true,'union has a real enclosed lumen');
        const contrast=new HybridContrastSystem({centerlineSegments:f.segments,contactField,sheath:vessel.sheaths.right});
        const network=contrast.flowNetwork;
        for(const edge of network.edges)network.depositIodine(edge.index,0,.01);
        contrast.totalInjectedIodineMassMg=network.getIodineMassMg();
        const renderer=new ContrastVolumeRenderer(contrast);
        try {
            assert.equal(contrast.setStentGraftSurface(system.surface),true);
            renderer.update();
            assert.ok(network.stentGraftRemodeling.coveredEdges>5);
            for(const side of ['right','left']) {
                const end=vessel.sheaths[side].end,location=network.findNearestLocation(end,{});
                assert.ok(network.edges[location.edgeIndex].meanFlowMm3PerS>0,`${side} distal access remains perfused`);
            }
            assert.ok(network.stentGraftRemodeling.trappedIodineMassMg>0);
            assert.ok(Math.abs(contrast.getMetrics().balanceErrorMg)<1e-7,'deployment conserves pre-existing iodine');
            for(let i=0;i<60;i++)contrast.update(1/30);
            assert.ok(Math.abs(contrast.getMetrics().balanceErrorMg)<1e-5);
            assert.equal(contrast.setStentGraftSurface(system.surface),false,'do not remodel each frame');
        } finally {renderer.dispose();}
        const result=createContactResult();let outside=0,total=0;
        for(const implant of system.implants)for(const part of implant.parts) {
            const p=part.mesh.geometry.attributes.position;
            for(let i=0;i<p.count;i++) {
                total++;if(contactField.querySphere(new THREE.Vector3(p.getX(i),p.getY(i),p.getZ(i)),0,result).violation)outside++;
            }
            for(let i=0;i<part.target.length;i++)assert.ok(Math.abs(p.array[i]-part.target[i])<1e-4,'every row must fully expand');
        }
        assert.equal(outside,0,`${outside}/${total} graft vertices lie outside the actual lumen`);
        system.setPosition('right',0);system.setPosition('left',0);
        for(let i=0;i<1200;i++){system.updateAccess('right',1/60);system.updateAccess('left',1/60);}
        assert.equal(system.removeDelivery('right').ok,true);assert.equal(system.removeDelivery('left').ok,true);
        sources.right={nodes:[],coordinate:()=>0,catheterMm:0};sources.left={nodes:[],coordinate:()=>0,catheterMm:0};
        assert.deepEqual(body.parts.map(p=>Array.from(p.mesh.geometry.attributes.position.array)),frozen);
        assert.equal(system.implants.length,2);assert.equal(system.availableGate('left'),null);
    } finally {f.dispose();}
});

test('device motion and deployment advance only on committed steps; empty and unsupported release is rejected',()=>{
    const f=fixture();try {
        const {system,sources}=f;assert.equal(system.load('left','body').ok,true);
        const d=system.accesses.left.device;system.setPosition('left',100);
        system.updateAccess('left',0);assert.equal(d.position,0);
        assert.equal(system.deploy('left').ok,false);
        assert.equal(system.positionAtTarget('left').ok,true);
        for(let i=0;i<1200;i++)system.updateAccess('left',1/60);
        const source=sources.left;sources.left={nodes:[],coordinate:()=>0,catheterMm:0};
        assert.equal(system.deploy('left').ok,false);sources.left=source;
        assert.equal(system.deploy('left').ok,true);system.updateAccess('left',0);assert.equal(d.deployment,0);
        assert.equal(system.removeDelivery('left').ok,false);
        finish(system,'left');place(system,'right','limb');assert.equal(system.deploy('right').ok,true);
    } finally {f.dispose();}
});

test('manual release accepts an off-target body and a standalone limb at their actual delivery noses',()=>{
    const f=fixture();
    try {
        const {system}=f;
        assert.equal(system.load('right','body').ok,true);
        const body=system.accesses.right.device;
        system.setPosition('right',100);for(let i=0;i<250;i++)system.updateAccess('right',1/60);
        const nose=system.getPath('right').sample(body.position);
        assert.ok(nose.y<-218,'test is deliberately outside the former landing zone');
        system.setPosition('right',120); // release the current pose, not the pending target
        assert.equal(system.validation('right').ok,true);
        assert.equal(system.deploy('right').ok,true);
        assert.ok(body.parts[0].points[0].distanceTo(nose)<1e-8);
        assert.equal(body.target,body.position);finish(system,'right');
        assert.equal(system.surface.sealed,false);
        assert.equal(system.load('left','limb').ok,true);
        const limb=system.accesses.left.device;
        system.setPosition('left',50);for(let i=0;i<130;i++)system.updateAccess('left',1/60);
        const limbNose=system.getPath('left').sample(limb.position);
        assert.equal(system.validation('left').ok,true);
        assert.equal(system.deploy('left').ok,true);
        assert.ok(limb.parts[0].points[0].distanceTo(limbNose)<1e-8);
        assert.equal(limb.parentId,null,'a misplaced limb must not claim a sealed connection');
        finish(system,'left');assert.equal(system.surface.sealed,false);
        assert.equal(system.surface.portals.length,5,'standalone limb retains both open ends');
    } finally {f.dispose();}
});

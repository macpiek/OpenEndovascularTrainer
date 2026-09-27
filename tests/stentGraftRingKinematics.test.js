import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {MAIN_BODY_MODELS,proximalDiameters} from '../src/devices/stentGraftModels.js';
import {advanceRelease} from '../src/devices/stentGraftDeployment.js';
import {RING_GAP_MM} from '../src/devices/stentGraftRingKinematics.js';

function verifyWires(parts) {
    for(const part of parts) {
        const starts=part.rings.geometry.attributes.instanceStart,ends=part.rings.geometry.attributes.instanceEnd;
        let first=0;
        for(const [i,ring] of part.scaffoldRings.entries()) {
            let length=0;
            for(let j=first;j<first+ring.samples;j++)length+=new THREE.Vector3().fromBufferAttribute(starts,j).distanceTo(new THREE.Vector3().fromBufferAttribute(ends,j));
            assert.ok(Math.abs(length/ring.restLength-1)<.0001,`wire length error ${length/ring.restLength-1}`);
            assert.ok(ring.center-ring.currentHeight/2>=0);
            assert.ok(ring.center+ring.currentHeight/2<=part.path.length);
            if(i){const previous=part.scaffoldRings[i-1];assert.ok(ring.center-previous.center-(ring.currentHeight+previous.currentHeight)/2>=RING_GAP_MM-1e-6,'adjacent waves retain a material gap');}
            first+=ring.samples;
        }
    }
}
for(const model of MAIN_BODY_MODELS)for(const diameter of [23,proximalDiameters(model.id).at(-1)]) {
    test(`${model.id}/${diameter}: metal arc length survives crimping, release, rotation and recapture`,()=>{
        const {system,device:d}=previewFixture('right','body',false,model.id);
        try {
            system.setDiameter('right',diameter);system.refreshDelivery('right');verifyWires(d.foldedPreview);
            const previewLengths=d.foldedPreview.slice(0,3).map(p=>p.scaffoldRings.map(r=>r.restLength));
            system.deploy('right');
            assert.deepEqual(d.parts.map(p=>p.scaffoldRings.map(r=>r.restLength)),previewLengths,'same metal before and after initiating deployment');
            verifyWires(d.parts);
            for(let k=0;k<40;k++){
                system.updateAccess('right',d.sheathTravel/12/40,null,{deviceId:d.id,release:'sheath',mechanicalRotation:k/40*.8});verifyWires(d.parts);
            }
            system.updateAccess('right',.2);verifyWires(d.parts);
            system.updateAccess('right',100,null,{deviceId:d.id,release:'resheath'});verifyWires(d.parts);
        }finally{system.dispose();}
    });
}
test('gate opening uses committed time and is independent of frame subdivision',()=>{
    const make=()=>({type:'body',phase:'deploying',position:100,sheathWithdrawal:0,sheathTravel:136,coverLead:12,tipRelease:0,gateOpening:0,gateFreeTime:0,parts:[{},{},{releaseOffset:50,path:{length:34}}]});
    const a=make(),b=make();
    advanceRelease(a,8.05,'sheath');
    for(let i=0;i<483;i++)advanceRelease(b,1/60,'sheath');
    assert.ok(Math.abs(a.gateOpening-b.gateOpening)<1e-10);
    advanceRelease(a,.2,null);advanceRelease(b,.2,null);
    assert.equal(a.gateOpening,1);assert.equal(b.gateOpening,1);
});

test('anatomy-fitted rings stay disjoint and retain wire length throughout release',async()=>{
    const {fixture}=await import('./helpers/stentGraftFixture.js');
    const {wireIntersections}=await import('./helpers/stentGraftRingSeparation.js');
    const f=fixture(),{system}=f;
    try {
        system.load('right','body');system.positionAtTarget('right');
        const d=system.accesses.right.device;d.position=d.target;system.deploy('right');
        for(let i=0;i<=24;i++) {
            if(i)system.updateAccess('right',d.sheathTravel/12/24,null,{deviceId:d.id,release:'sheath'});
            verifyWires(d.parts);
            assert.deepEqual(wireIntersections(d.parts),[],`intersecting metal at cover ${d.sheathWithdrawal} mm`);
        }
        system.updateAccess('right',100,null,{deviceId:d.id,release:'resheath'});
        const end=d.coverLead+d.parts[2].releaseOffset+d.parts[2].path.length;
        system.updateAccess('right',end/12,null,{deviceId:d.id,release:'sheath'});
        for(let i=0;i<25;i++) {
            system.updateAccess('right',1/120);verifyWires(d.parts);
            assert.deepEqual(wireIntersections(d.parts),[],`gate opening ${d.gateOpening}`);
        }
    }finally{f.dispose();}
});
test('standalone limb metal remains inextensible during release',()=>{
    const {system,device:d}=previewFixture('left','limb');
    try {
        for(let i=0;i<30;i++) {
            system.updateAccess('left',d.sheathTravel/12/30,null,{deviceId:d.id,release:'sheath'});
            verifyWires(d.parts);
        }
    }finally{system.dispose();}
});

test('both branch springs use committed time independently of frame subdivision',()=>{
    const make=()=>({type:'body',phase:'deploying',position:100,implantPosition:100,sheathWithdrawal:0,sheathTravel:136,releaseLength:103,
        coverLead:12,tipRelease:0,gateOpening:0,gateFreeTime:0,ipsiOpening:0,ipsiFreeTime:0,
        parts:[{releaseOffset:0,path:{length:50}},{releaseOffset:50,path:{length:53}},{releaseOffset:50,path:{length:34}}]});
    const a=make(),b=make();
    advanceRelease(a,9.65,'sheath');
    for(let i=0;i<579;i++)advanceRelease(b,1/60,'sheath');
    for(const key of ['gateOpening','ipsiOpening'])assert.ok(Math.abs(a[key]-b[key])<1e-9,key);
    advanceRelease(a,.2,null);advanceRelease(b,.2,null);
    assert.equal(a.ipsiOpening,1);assert.equal(b.ipsiOpening,1);
});

import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {graftLumenAt,graftLumenSections} from '../src/devices/stentGraftLumenContact.js';

test('recovery follows translated, flattened accepted cloth without pushing valid interior points',()=>{
    const sides=24,positions=new Float32Array(sides*2*3),target=new Float32Array(positions.length);
    for(let row=0;row<2;row++)for(let j=0;j<sides;j++) {
        const angle=j/sides*Math.PI*2,offset=(row*sides+j)*3;
        positions.set([50+10*Math.cos(angle),2*Math.sin(angle),row*10],offset);
        target.set([Math.cos(angle),Math.sin(angle),row*10],offset);
    }
    const geometry=new THREE.BufferGeometry();
    geometry.setAttribute('position',new THREE.BufferAttribute(positions,3));
    const part={sides,rows:2,points:[{},{}],path:{coordinates:[0,10]},releaseOffset:0,
        wasCaptured:false,target,mesh:{geometry}};
    const device={type:'body',phase:'deploying',parts:[part],position:100,implantPosition:100,
        sheathWithdrawal:100,coverLead:0};
    try {
        const sections=graftLumenSections(device);
        assert.equal(sections.length,1);
        assert.ok(graftLumenAt(sections,95,[58,0,5],.8).penetration<0,
            'a tool with clearance inside the long axis must not feel a circular recovery force');
        assert.ok(graftLumenAt(sections,95,[63,0,5],.8).penetration>0,
            'recovery outside the accepted cloth remains active');
        // Indentation is applied by the contact law itself. Do not count it
        // twice or change old snapshots when presentation buffers are edited.
        part.contactBasePositions=positions.slice();
        positions[0]+=20;
        assert.deepEqual(graftLumenSections(device),sections);
        assert.equal(graftLumenAt(sections,95,[58,0,20],.8),null,'open portal stays open');
    } finally {geometry.dispose();}
});

for(const side of ['right','left'])for(const type of ['body','limb'])
test(`${side} ${type}: completion waits for the entire sewn assembly to settle`,()=>{
    const {system,device:d}=previewFixture(side,type);
    try {
        let step=0;
        for(;step<1500&&d.phase!=='deployed';step++) {
            system.updateAccess(side,1/60,null,{deviceId:d.id,release:{sheath:1,tip:true}});
            if(d.parts.some(part=>part.releaseRelaxing)) {
                assert.equal(d.phase,'deploying','do not freeze geometry which still needs relaxation');
                assert.equal(system.surface,null,'do not publish an unfinished implant');
            }
        }
        assert.equal(d.phase,'deployed','settling must finish, not leave the device permanently expanding');
        assert.ok(d.parts.every(part=>!part.releaseRelaxing));
        const accepted=d.parts.map(part=>part.mesh.geometry.attributes.position.array.slice());
        for(let i=0;i<30;i++)system.updateAccess(side,1/60);
        assert.deepEqual(d.parts.map(part=>part.mesh.geometry.attributes.position.array),accepted);
    } finally {system.dispose();}
});

for(const side of ['right','left'])test(`${side}: delivery follows committed rod withdrawal with no guidewire`,()=>{
    const {system,device:d}=previewFixture(side,'body',false);
    const body={jointStateView:{coordinates:new Float64Array([0,200]),
        positions:new Float64Array([0,0,0,0,200,0])}};
    const source={nodes:[],coordinate:()=>0,catheterMm:0,deliveryBody:body};
    try {
        system.updateAccess(side,1/60,source,{deviceId:d.id,mechanicalPosition:200});
        system.refreshDelivery(side);
        const before=d.noseBand.position.clone();
        assert.deepEqual(before.toArray(),[0,212,0]);
        body.jointStateView={coordinates:new Float64Array([0,190]),
            positions:new Float64Array([10,0,0,10,190,0])};
        system.refreshDelivery(side);
        assert.deepEqual(d.noseBand.position,before,'a trial cannot publish uncommitted rod geometry');
        system.updateAccess(side,1/60,source,{deviceId:d.id,mechanicalPosition:190});
        system.refreshDelivery(side);
        assert.deepEqual(d.noseBand.position.toArray(),[10,202,0]);
        assert.ok(d.deliveryMesh.visible&&d.noseMarker.visible);
        assert.ok(d.noseMarker.geometry.attributes.position.array.every(Number.isFinite));
        body.jointStateView.positions.fill(999);
        system.refreshDelivery(side);
        assert.deepEqual(d.noseBand.position.toArray(),[10,202,0],'committed snapshot owns its positions');
    } finally {system.dispose();}
});

test('preview-only delivery can withdraw along its last supported route after wire removal',()=>{
    const {system,device:d,sources}=previewFixture('right','body',false);
    try {
        system.refreshDelivery('right');const before=d.noseBand.position.clone();
        sources.right.nodes=[];
        system.updateAccess('right',.4,null,{deviceId:d.id,mechanicalPosition:d.position-10});
        system.refreshDelivery('right');
        assert.ok(d.noseBand.position.distanceTo(before)>9,'withdrawal must remain visible');
    } finally {system.dispose();}
});

test('delivery follows legacy rod coordinates across the sheath inlet without a guidewire',()=>{
    const {system,device:d}=previewFixture('right','body',false);
    const body={activeStart:1,activeEnd:2,
        materialCoordinate:new Float64Array([-10,10,190]),
        x:new Float64Array([4,4,4]),y:new Float64Array([-10,10,190]),z:new Float64Array(3)};
    const source={nodes:[],coordinate:()=>0,catheterMm:0,deliveryBody:body};
    try {
        system.updateAccess('right',1/60,source,{deviceId:d.id,mechanicalPosition:190});
        system.refreshDelivery('right');
        assert.deepEqual(d.noseBand.position.toArray(),[4,202,0]);
        assert.deepEqual(system.getDeliveryPath('right').sample(0).toArray(),[4,0,0]);
        body.x.fill(999);
        system.refreshDelivery('right');
        assert.deepEqual(d.noseBand.position.toArray(),[4,202,0],'legacy publication also owns its geometry');
    } finally {system.dispose();}
});

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

for(const side of ['right','left'])for(const type of ['body','limb'])
test(`${side} ${type}: nose follows the committed distal wire beyond the solved delivery shaft`,()=>{
    const {system,device:d}=previewFixture(side,type,false);
    const nodes=Array.from({length:301},(_,s)=>s<=200?new THREE.Vector3(0,s,0):
        new THREE.Vector3(50*(1-Math.cos((s-200)/50)),200+50*Math.sin((s-200)/50),0));
    const body={jointStateView:{coordinates:new Float64Array([0,200]),positions:new Float64Array([0,0,0,0,200,0])}};
    const source={nodes,coordinate:i=>i,catheterMm:0,deliveryBody:body};
    const noseCenters=()=>{
        const p=d.noseMarker.geometry.attributes.position;
        return Array.from({length:61},(_,row)=>{
            const center=new THREE.Vector3();
            for(let j=0;j<16;j++)center.add(new THREE.Vector3().fromBufferAttribute(p,row*16+j));
            return center.divideScalar(16);
        });
    };
    try {
        system.updateAccess(side,1/60,source,{deviceId:d.id,mechanicalPosition:200});
        system.refreshDelivery(side);
        const start=type==='body'?212:200,centers=noseCenters();
        for(let row=0;row<=60;row++)assert.ok(centers[row].distanceTo(nodes[start+row])<2e-5,
            `nose row ${row} must follow the wire, not the shaft tangent`);
        assert.ok(centers[30].distanceTo(centers[0].clone().lerp(centers[60],.5))>5);
        assert.deepEqual(system.getDeliveryPath(side).sample(150).toArray(),[0,150,0],
            'the accepted stiff shaft is not replaced by a decorative wire path');
        const accepted=d.noseMarker.geometry;
        for(let i=201;i<nodes.length;i++)nodes[i].z=(i-200)*.1;
        system.refreshDelivery(side);assert.equal(d.noseMarker.geometry,accepted,'uncommitted trials stay invisible');
        system.updateAccess(side,1/60,source,{deviceId:d.id,mechanicalPosition:200});
        system.refreshDelivery(side);assert.notEqual(d.noseMarker.geometry,accepted);
        assert.ok(noseCenters()[60].distanceTo(nodes[start+60])<2e-5,'distal wire motion invalidates the nose cache');
    } finally {system.dispose();}
});

for(const side of ['right','left'])test(`${side}: stationary tool indentation cannot keep release and collision rebuilding alive`,()=>{
    const {system,device:d}=previewFixture(side);
    try {
        let patch;
        for(let i=0;i<900&&d.phase!=='deployed';i++) {
            system.updateAccess(side,1/60,null,{deviceId:d.id,release:{sheath:1,tip:true}});
            if(i===580) {
                const p=d.parts[0],a=p.mesh.geometry.attributes.position,j=10*p.sides;
                patch={point:[a.getX(j),a.getY(j),a.getZ(j)],normal:[1,0,0],depth:.3,width:8};
                system.mechanicalSurfaceForAccess(side).commitContactPatches([patch]);
            }
        }
        assert.ok(patch,'contact was applied before the end of release');
        assert.equal(d.phase,'deployed','a stationary contact must not feed displacement back into expansion forever');
        assert.ok(d.parts.every(p=>!p.releaseRelaxing));
        const revision=system.geometryRevision,surface=system.mechanicalSurface;
        const base=d.parts.map(p=>p.contactBasePositions.slice());
        for(let i=0;i<60;i++)system.updateAccess(side,1/60);
        assert.equal(system.geometryRevision,revision,'idle physics must reuse collision geometry');
        assert.equal(system.mechanicalSurface,surface);
        assert.deepEqual(d.parts.map(p=>p.contactBasePositions),base);
        assert.ok(d.parts.some(p=>p.mesh.geometry.attributes.position.array.some((v,i)=>v!==p.contactBasePositions[i])),
            'settling preserves the actual tool indentation');
        system.mechanicalSurfaceForAccess(side).commitContactPatches([]);
        assert.deepEqual(d.parts.map(p=>p.mesh.geometry.attributes.position.array),base,'withdrawing the tool clears indentation');
    } finally {system.dispose();}
});

for(const replacement of [false,true])test(`withdrawn detached delivery can be removed while fabric settles (replacement: ${replacement})`,()=>{
    const {system,device:d}=previewFixture();
    try {
        system.updateAccess('right',.1,null,{deviceId:d.id,release:'tip'});
        d.position=d.target=0;
        assert.equal(system.removeDelivery('right').ok,false,'captured implant still belongs to delivery');
        d.position=d.target=d.implantPosition;
        system.updateAccess('right',1,null,{deviceId:d.id,release:'tip'});
        // All hardware clears in this committed step; cloth still needs time.
        system.updateAccess('right',1/60,null,{deviceId:d.id,mechanicalPosition:0});
        assert.equal(d.phase,'deploying');
        const anchor=d.implantPosition;
        assert.equal(system.removeDelivery('right').ok,true);
        assert.equal(system.accesses.right.device,null);
        if(replacement)assert.equal(system.load('right','limb').ok,true);
        const next=system.accesses.right.device;
        const snapshot=d.parts[0].mesh.geometry.attributes.position.array.slice();
        system.updateAccess('right',0);
        assert.deepEqual(d.parts[0].mesh.geometry.attributes.position.array,snapshot,'zero dt cannot advance a removed delivery’s implant');
        for(let i=0;i<900&&d.phase!=='deployed';i++)system.updateAccess('right',1/60);
        assert.equal(d.phase,'deployed','implant keeps advancing after hardware removal');
        assert.equal(d.implantPosition,anchor);
        assert.equal(system.accesses.right.device,next,'old implant must not replace the new delivery');
        assert.ok(system.implants.includes(d));
        assert.ok(system.surface.parts.includes(d.parts[0]));
    } finally {system.dispose();}
});

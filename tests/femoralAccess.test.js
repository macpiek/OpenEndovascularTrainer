import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {generateVessel} from '../src/vesselGeometry.js';
import {createFemoralAccessController} from '../src/femoralAccessController.js';
import {RodState} from '../src/physics/rodState.js';
import {GuidewireTransport} from '../src/physics/guidewireTransport.js';
import {PigtailCatheter} from '../src/pigtailCatheter.js';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';
import {PackedLumenField} from '../src/physics/collision/packedLumenField.js';
import {INTRODUCER_SHEATH_RADIUS_MM} from '../src/toolDimensions.js';

const {vessel} = generateVessel();
function createAccess(id) {
    const wire = new RodState(201, 5);
    const activeSheath = vessel.sheaths[id];
    const transport = new GuidewireTransport({rod:wire,segmentLength:5,guidewireLength:1000,
        sheath:activeSheath,advanceRate:44,minInsert:0,maxInsert:1000});
    transport.initialize();
    const catheter = new PigtailCatheter({wire,segmentLength:5,guidewireLength:1000,
        tailProgressRef:()=>transport.progress,vessel:{...vessel,sheath:activeSheath}});
    return {wire,activeSheath,guidewireTransport:transport,pigtailCatheter:catheter,rotation:0};
}

test('left sheath is in the actual left lumen and has a clear distal exit', () => {
    const bytes = fs.readFileSync(new URL('../res/Aorta_plain.collision.bin', import.meta.url));
    const asset = decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
    const field = new PackedLumenField(asset.metadata,asset.arrays);
    assert.equal(vessel.sheath, vessel.sheaths.right, 'legacy right access remains the default');
    assert.ok(vessel.sheaths.left.end.x > 0);
    assert.ok(vessel.sheaths.right.end.x < 0);
    for (const [id,sheath] of Object.entries(vessel.sheaths)) {
        const axis = Object.fromEntries(['x','y','z'].map(k=>[k,(sheath.end[k]-sheath.start[k])/sheath.length]));
        for (let distance=-5;distance<=8;distance+=.5) {
            const p = Object.fromEntries(['x','y','z'].map(k=>[k,sheath.end[k]+distance*axis[k]]));
            const hit = field.query(p);
            assert.ok(hit.inside && hit.signedDistance > INTRODUCER_SHEATH_RADIUS_MM,
                `${id} sheath blocked at ${distance} mm: ${hit.signedDistance}`);
        }
    }
});

test('switch preserves independent wire, catheter, rotation and material storage', () => {
    const right = createAccess('right'), left = createAccess('left');
    let active = right, pending = true;
    const manager = createFemoralAccessController({initial:'right',entries:{right,left},
        capture:()=>({...active}),restore:state=>{active=state;},isPending:()=>pending});
    right.guidewireTransport.advance(1,2);
    right.pigtailCatheter.advance(1,1,88);
    right.rotation=.7;
    const rightPositions = right.wire.nodes.map(p=>[p.x,p.y,p.z]);
    const rightCatheterProgress = right.pigtailCatheter.progress;
    manager.request('left');
    assert.equal(manager.applyRequested(),false,'must finish the prepared timestep before switching');
    assert.equal(active,right);
    pending=false;
    assert.equal(manager.applyRequested(),true);
    active.guidewireTransport.advance(1,1);
    active.pigtailCatheter.advance(1,.5,44);
    active.rotation=-.3;
    assert.equal(right.guidewireTransport.progress,88);
    assert.deepEqual(right.wire.nodes.map(p=>[p.x,p.y,p.z]),rightPositions);
    assert.notEqual(right.wire.nodeStorage,left.wire.nodeStorage);
    manager.request('right');manager.applyRequested();
    assert.equal(active.guidewireTransport.progress,88);
    assert.equal(active.pigtailCatheter.progress,rightCatheterProgress);
    assert.equal(active.rotation,.7);
    assert.equal(left.guidewireTransport.progress,44);
    assert.equal(left.rotation,-.3);
    assert.throws(()=>manager.request('unknown'));
});

test('injection source follows the chosen sheath without relocating existing contrast', async () => {
    const {HybridContrastSystem} = await import('../src/contrast/hybridContrastSystem.js');
    const {Vector3} = await import('three');
    const system = new HybridContrastSystem({centerlineSegments:[{id:0,nodeStartId:0,nodeEndId:1,
        start:new Vector3(0,0,0),end:new Vector3(0,140,0),radiusStart:7,radiusEnd:7,safeRadius:7}],
        sheath:{start:{x:0,y:-10,z:0},end:{x:0,y:10,z:0}}});
    assert.equal(system.startInjection({source:'sheath',rateMlPerSec:2,volumeMl:5}).ok,true);
    system.update(1/60);
    const network=system.flowNetwork, delivered=system.totalDeliveredVolumeMl;
    const catheter={type:'berenstein'}, sheath={start:{x:0,y:50,z:0},end:{x:0,y:70,z:0}};
    system.setAccess(sheath,catheter);
    assert.equal(system.isInjecting,false);
    assert.equal(system.flowNetwork,network);
    assert.equal(system.totalDeliveredVolumeMl,delivered);
    assert.equal(system.catheter,catheter);
    assert.deepEqual(system._sheathPort.position.toArray(),[0,70,0]);
    assert.equal(system.getSourceStatus('sheath').valid,true);
    assert.equal(system.startInjection({source:'sheath',rateMlPerSec:2,volumeMl:5}).ok,true);
});

test('interleaved access work isolates rollback snapshots and restores context after exceptions', async () => {
    const {createPreparedInputCheckpoint}=await import('../src/physics/preparedInputCheckpoint.js');
    const entries=Object.fromEntries(['right','left'].map((id,index)=>[id,{
        id,body:{x:new Float64Array([index+1])},checkpoint:createPreparedInputCheckpoint(),simulationExecutedSteps:0
    }]));
    let state=entries.right;
    const manager=createFemoralAccessController({initial:'right',entries,capture:()=>({...state}),
        restore:value=>{state=value;},isPending:()=>false});
    for(const id of ['right','left'])manager.run(id,()=>{
        state.checkpoint.capture([state.body]);state.body.x[0]+=10;
    });
    manager.run('left',()=>state.checkpoint.restore());
    assert.equal(entries.left.body.x[0],2);
    assert.equal(entries.right.body.x[0],11,'left rollback must not restore right input');
    assert.throws(()=>manager.run('left',()=>{throw new Error('failed slice');}));
    assert.equal(state.id,'right');
    manager.run('right',()=>state.checkpoint.restore());
    assert.equal(entries.right.body.x[0],1);
});

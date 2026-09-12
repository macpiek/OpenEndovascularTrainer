import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader.js';
import {MeshBVH} from 'three-mesh-bvh';
import {createCompositeAppLocalFrame} from '../src/physics/kirchhoffCompositeAppLocalFrame.js';
import {compositeJointWallSourceSignature} from '../src/physics/kirchhoffCompositeJointWallRows.js';
import {queryCompositeWallBvhCapsuleGeometry} from '../src/physics/kirchhoffCompositeWallBvhGeometry.js';
import {VesselContactField,createContactResult} from '../src/physics/collision/vesselContactField.js';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';
import {transformAortaGeometry} from '../src/aortaTransform.js';
import {generateVessel} from '../src/vesselGeometry.js';
import {createCapturedCompositeAppFixture} from './helpers/capturedCompositeAppFixture.js';

test('constant local World view preserves actual anatomy witnesses, BVH differential and source identity guards',()=>{
    const binary=name=>{const b=readFileSync(new URL(`../res/${name}`,import.meta.url));return b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength);};
    const asset=decodeCollisionAsset(binary('Aorta_plain.collision.bin')),geometry=new STLLoader().parse(binary('Aorta_plain.stl'));
    transformAortaGeometry(geometry,generateVessel(140,0).vessel);geometry.boundsTree=new MeshBVH(geometry);
    const field=new VesselContactField(asset,{fallbackGeometry:geometry,bvhValidationDistance:.02,capsuleBvhValidation:-.1});
    const origin=[-89.60763512409662,-449.43054049638647,28.531680733584537],body={},sheath={startX:origin[0],startY:origin[1],startZ:origin[2],axisX:0,axisY:1,axisZ:0,bodies:[body]},
        world={bodies:[body],sheaths:[sheath],contactField:field},frame=createCompositeAppLocalFrame(world,'sheath-start'),local=frame.world.contactField;
    assert.equal(frame.world.bodies,world.bodies);assert.equal(frame.world.sheaths[0].bodies,sheath.bodies);
    assert.deepEqual(['startX','startY','startZ'].map(k=>frame.world.sheaths[0][k]),[0,0,0]);
    assert.equal(local,frame.world.contactField);assert.equal(local.queryCapsuleCoordinates,frame.world.contactField.queryCapsuleCoordinates);
    assert.equal(local.fallbackGeometry.boundsTree.geometry,local.fallbackGeometry);
    assert.equal(local.fallbackGeometry.index,geometry.index);assert.equal(local.fallbackGeometry.attributes.position.array,geometry.attributes.position.array);
    assert.deepEqual(Array.from(local.sdfOrigin),field.sdfOrigin.map((v,k)=>v-origin[k]));
    const seamPoint=[0,0,local.sdfFaceCoordinate(2,215)];
    assert.equal(frame.toWorld(seamPoint)[2],3.5);assert.equal(local.sdfGridCoordinates(seamPoint)[2],215);
    assert.notEqual(local.sdfOrigin[2]+215*local.voxelSize,seamPoint[2],'Translated-origin arithmetic does not represent the same exact plane');
    for(const shift of [1,-4]) {
        const positions=[[65.00287246704102,-462.2578430175781+shift,-79.56869888305664],[64.95548751831055,-462.0397204589844+shift,-79.65612350463867]],
            localPositions=positions.map(frame.toLocal),globalPoints=localPositions.map(frame.toWorld),radius=.4445,
            raw=field.queryCapsuleCoordinates(...globalPoints[0],...globalPoints[1],radius,createContactResult()),out=createContactResult(),
            translated=local.queryCapsuleCoordinates(...localPositions[0],...localPositions[1],radius,out);
        assert.equal(translated,out);
        for(const key of ['signedDistance','signedGap','segmentT','capsuleSampleCount','faceIndex','source'])assert.equal(translated[key],raw[key]);
        for(const key of ['point','target','closestPoint'])assert.deepEqual(Array.from(translated[key].values),Array.from(raw[key].values,(v,k)=>v-origin[k]));
        assert.deepEqual(translated.inward.values,raw.inward.values);
        const differential=queryCompositeWallBvhCapsuleGeometry({field:local,positions:localPositions,radius});
        assert.equal(differential.supported,true,differential.reason);
        const repeated=local.queryCapsuleCoordinates(...localPositions[0],...localPositions[1],radius,out);
        assert.deepEqual(repeated.closestPoint.values,translated.closestPoint.values,'Reused provider output is translated once per query');
        const sphere=local.querySphere({x:localPositions[0][0],y:localPositions[0][1],z:localPositions[0][2]},radius);
        const rawSphere=field.querySphere(globalPoints[0],radius);
        assert.deepEqual(Array.from(sphere.closestPoint.values),Array.from(rawSphere.closestPoint.values,(v,k)=>v-origin[k]));
    }
    const before=compositeJointWallSourceSignature(local);field.version=1;assert.notEqual(compositeJointWallSourceSignature(local),before);
    const prior=local.queryCapsuleCoordinates;field.queryCapsuleCoordinates=function(...args){return VesselContactField.prototype.queryCapsuleCoordinates.apply(this,args);};
    assert.notEqual(local.queryCapsuleCoordinates,prior,'Replacing the real source query must invalidate a pending proof');
    const stable=local.queryCapsuleCoordinates;assert.equal(stable,local.queryCapsuleCoordinates);
    geometry.dispose();
});

test('persistent local native state accepts actual slow catheter tiny-cell feed and budget retry without global history roundtrip',()=>{
    const f=createCapturedCompositeAppFixture({coordinateOrigin:'sheath-start'});
    assert.equal(f.system.diagnostics.initializations,0,'UI must see a fresh epoch before deciding to align source frames');
    assert.equal(f.system.diagnostics.pending,false);
    for(let step=0;step<54;step++) {
        f.prepare(step>0&&step<=45?1:0,step>45&&step<54?16/52:0);
        const retry=step===49,oldCounters={...f.counters};if(retry)f.system.setBudget({directions:0});
        let result=f.system.step(f.world,f.dt);
        if(retry) {
            assert.equal(result.accepted,false);assert.equal(result.status,'direction-budget');
            f.system.setBudget(null);result=f.system.step(f.world,f.dt);
            assert.equal(f.counters.controls,oldCounters.controls+1);assert.equal(f.counters.layout,oldCounters.layout+1);
        }
        assert.equal(result.accepted,true,JSON.stringify({step,status:result.status,message:result.message,certificate:result.diagnostics?.certificate}));
        assert.ok(result.diagnostics.certificate.force<=1e-7);
        f.bodies.get('wire').syncToRodState(f.rod);
    }
    const state=f.system.snapshot(),origin=f.system.diagnostics.coordinateOrigin;
    assert.equal(f.counters.initial,1);assert.equal(state.step,54);assert.equal(f.system.diagnostics.coordinateFrame,'constant-local');
    assert.ok(Math.abs(f.catheter.progress-16*8/120)<1e-12);assert.equal(f.containment.enabled,true);
    assert.ok(state.positions.every(p=>Math.abs(p[1])<100));
    assert.ok(state.nativeRateHistory.tools.every(t=>t.edges.every(e=>e.positions.every(p=>Math.abs(p[1])<100))));
    assert.ok(f.bodies.get('wire').jointStateView.positions.every((v,i)=>i%3!==1||v<-400),'Published view stays in anatomy coordinates');
    assert.ok(origin[1]<-400);
    assert.throws(()=>f.system.reset({}),/exactly one World/);
    assert.equal(f.system.snapshot().step,54);
    f.system.reset(f.world);assert.equal(f.system.snapshot(),null);assert.equal(f.system.diagnostics.coordinateOrigin,null);
    assert.equal(f.system.diagnostics.initializations,0,'Reset must expose the same fresh alignment epoch as startup');
});

test('local sign-domain proof is queried in the original anatomy coordinates',()=>{
    const calls=[],proof={supported:true},source={certifyInsideBallCoordinates(...args){calls.push(args);return proof;}},
        frame=createCompositeAppLocalFrame({contactField:source,sheaths:[],bodies:[]},[10,-20,30]);
    assert.equal(frame.world.contactField.certifyInsideBallCoordinates(1,2,3,.04),proof);
    assert.deepEqual(calls,[[11,-18,33,.04]]);
});

test('actual anatomy branch chart uses the exact provider grid plane, not a near-seam local coordinate',async()=>{
    const {createCompositeAnatomyField}=await import('./helpers/compositeAnatomyField.js');
    const {createCompositeWallSdfBranchesWorkspace,evaluateCompositeWallSdfBranches,measureCompositeWallSdfBranches}=await import('../src/physics/kirchhoffCompositeWallSdfBranches.js');
    const anatomy=createCompositeAnatomyField();
    try {
        const report=JSON.parse(readFileSync(new URL('../reports/composite-material-point-rejected-step361.json',import.meta.url),'utf8')),
            origin=JSON.parse(readFileSync(new URL('./fixtures/compositeNativeAppInitial.json',import.meta.url),'utf8')).context.sheaths[0].start,
            frame=createCompositeAppLocalFrame({contactField:anatomy.field,sheaths:[],bodies:[]},origin),field=frame.world.contactField,
            point=new Map(report.result.diagnostics.rejectedConfiguration.positions).get('wire')[39].slice(),radius=.44449999928474426,
            face={axis:2,gridIndex:215},exact=field.sdfFaceCoordinate(2,215);
        point[2]=exact;
        const domainBox={lower:point.map(v=>v-.02),upper:point.map(v=>v+.02)},evaluate=p=>{
            const contact=field.queryCapsuleCoordinates(...p,...p,radius,createContactResult());
            return evaluateCompositeWallSdfBranches({field,positions:[p],radius,face,domainBox,contact},createCompositeWallSdfBranchesWorkspace(1));
        },chart=evaluate(point);
        assert.equal(frame.toWorld(point)[2],3.5);assert.equal(field.sdfGridCoordinates(point)[2],215);
        assert.equal(chart.supported,true,chart.reason);assert.equal(chart.onSeam,true);assert.ok(chart.rows.every(r=>r.inOriginalCell));
        const gates={forces:[136.6,348.4],penalty:1,gapTolerance:1e-8,forceTolerance:1e-7,workTolerance:1e-8};
        assert.equal(measureCompositeWallSdfBranches(chart,gates).domainAdmissible,true);
        const old=point.slice();old[2]=-25.031680733584555;
        assert.notEqual(old[2],exact);assert.notEqual(field.sdfGridCoordinates(old)[2],215);
        const near=evaluate(old);assert.equal(near.supported,true,near.reason);assert.equal(near.onSeam,false);
        assert.equal(measureCompositeWallSdfBranches(near,gates).domainAdmissible,false);
    }finally{anatomy.dispose();}
});

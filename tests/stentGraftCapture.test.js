import test from 'node:test';
import * as THREE from 'three';
import assert from 'node:assert/strict';
import {captureReaction,createCapturePotential} from '../src/devices/stentGraftCapture.js';
import {createSharedAxisNative,relaxSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {defineKirchhoffMaterialProfile} from '../src/physics/kirchhoffMaterialProfile.js';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';

const links={offset:[12,0,0],armLength:18,stiffness:180,roots:[[110,20,0],[109,19,1]]};
test('capture forces are equal and opposite; energy gradient and tangent match finite differences',()=>{
    const p=[112,0,0],r=captureReaction(links,p),h=1e-5;
    for(let i=0;i<3;i++) {
        assert.ok(Math.abs(r.deliveryForce[i]+r.graftForces.reduce((sum,f)=>sum+f[i],0))<1e-10);
        const a=p.slice(),b=p.slice();a[i]+=h;b[i]-=h;
        const ra=captureReaction(links,a),rb=captureReaction(links,b);
        assert.ok(Math.abs((ra.energy-rb.energy)/(2*h)-r.gradient[i])<1e-5);
        for(let j=0;j<3;j++)assert.ok(Math.abs((ra.gradient[j]-rb.gradient[j])/(2*h)-r.hessian[j][i])<1e-5);
    }
});
test('attached graft reaction deflects the delivery shaft and its shared wire; detachment removes that force',()=>{
    const type=defineKirchhoffMaterialProfile({id:'capture-test-shaft',sampleEI1:()=>1e6,sampleGJ:()=>1e6});
    const state=createSharedAxisNative({tools:[{id:'wire',type,insertion:110},{id:'catheter',type,insertion:100}],spacing:5,
        wallSamples:[createCapturePotential(links)]});
    let result=relaxSharedAxisNative(state,{maxIterations:100,forceTolerance:1e-5});
    assert.ok(result.converged,JSON.stringify(result));
    assert.ok(state.positions.at(-1)[1]>.1,'reaction must affect mechanical coordinates, not just render geometry');
    assert.ok(Math.hypot(...state.graftCaptureStats.deliveryForce)>1e-3);
    // Leave the inactive base row to preserve this test's topology; remove only
    // the potential, exactly the load which detachment eliminates.
    delete state.wallSamples[0].addPotential;
    result=relaxSharedAxisNative(state,{maxIterations:100,forceTolerance:1e-5});
    assert.ok(result.converged,JSON.stringify(result));
    assert.ok(Math.abs(state.positions.at(-1)[1])<1e-3,'released shaft returns to its unloaded shape');
});
test('capture is owned by the delivery access, survives cover motion and disappears only at latch release',()=>{
    const {system,device:d}=previewFixture();try {
        assert.ok(system.captureForAccess('right'));assert.equal(system.captureForAccess('left'),null);
        const old=system.captureForAccess('right');system.updateAccess('right',100,null,{deviceId:d.id,release:'sheath'});
        assert.ok(system.captureForAccess('right'));assert.notDeepEqual(system.captureForAccess('right').roots,old.roots);
        assert.ok(old.roots.every(p=>p.every(Number.isFinite)),'snapshot remains owned');
        system.updateAccess('right',.5,null,{deviceId:d.id,release:'tip'});assert.ok(system.captureForAccess('right'));
        system.updateAccess('right',.5,null,{deviceId:d.id,release:'tip'});assert.equal(system.captureForAccess('right'),null);
    }finally{system.dispose();}
});

test('contact indentation refresh preserves the attached crown length and straight roots',()=>{
    const {system,device:d}=previewFixture();try {
        system.updateAccess('right',100,null,{deviceId:d.id,release:'sheath'});
        const part=d.parts[0],positions=part.mesh.geometry.attributes.position;
        const before=Array.from(positions.array.slice(0,part.sides*3));
        system.refreshContactIndentation();
        assert.deepEqual(Array.from(positions.array.slice(0,part.sides*3)),before);
        for(let j=0;j<part.sides;j++){
            const distance=Math.hypot(positions.getX(j)-d.crownCaptured.x,positions.getY(j)-d.crownCaptured.y,positions.getZ(j)-d.crownCaptured.z);
            assert.ok(Math.abs(distance-d.crownMaterial.armLength)<1e-4);
        }
    }finally{system.dispose();}
});

for(const side of ['right','left'])test(`${side}: contact commits during captured release update cloth and can clear its indentation`,()=>{
    const {system,device:d}=previewFixture(side);
    try {
        system.updateAccess(side,64.8/12,null,{deviceId:d.id,release:'sheath'});
        assert.equal(d.tipRelease,0);
        const surface=system.mechanicalSurfaceForAccess(side),part=d.parts[0],attribute=part.mesh.geometry.attributes.position;
        surface.commitContactPatches([]);
        const before=Array.from(attribute.array),row=part.exposure.findIndex((exposure,i)=>i>2&&exposure===1);
        assert.ok(row>2);
        const index=row*part.sides,point=new THREE.Vector3().fromBufferAttribute(attribute,index);
        const normal=point.clone().sub(part.points[row]).normalize();
        const patch={point:point.toArray(),normal:normal.toArray(),width:3,depth:.2};
        surface.commitContactPatches([patch]);
        assert.ok(new THREE.Vector3().fromBufferAttribute(attribute,index).distanceTo(point)>.1);
        assert.ok(Array.from(attribute.array).every(Number.isFinite));
        const version=attribute.version,revision=system.contactRevisions[side];
        surface.commitContactPatches([patch]);
        assert.equal(attribute.version,version,'identical contact must not rebuild geometry');
        assert.equal(system.contactRevisions[side],revision);
        surface.commitContactPatches([]);
        assert.deepEqual(Array.from(attribute.array),before,'clearing the contact restores cloth without changing capture');
        assert.ok(system.captureForAccess(side));
    } finally {system.dispose();}
});

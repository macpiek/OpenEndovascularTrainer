import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {CatheterGraduationMarkers} from '../src/catheterGraduationMarkers.js';
import {getCompositeJointRenderPath} from '../src/compositeJointRenderPath.js';
import {PigtailCatheter} from '../src/pigtailCatheter.js';
import {RodState} from '../src/physics/rodState.js';

const points = [new THREE.Vector3(), new THREE.Vector3(0, 100, 0)];
function centers(bands) {
    return Array.from({length:bands.count}, (_, i) => {
        const m = new THREE.Matrix4(); bands.getMatrixAt(i, m);
        return new THREE.Vector3().setFromMatrixPosition(m);
    });
}

test('graduations are 10 mm apart from the loop base and follow insertion', () => {
    const material = new THREE.MeshBasicMaterial();
    const bands = new CatheterGraduationMarkers(0.83, 1000, material);
    bands.update({enabled:true, points, loopLengthMm:20});
    assert.deepEqual(centers(bands).map(p=>Math.round(p.y)), [70,60,50,40,30,20,10]);
    bands.update({enabled:true, points:[points[0], new THREE.Vector3(0, 110, 0)], loopLengthMm:20});
    assert.deepEqual(centers(bands).map(p=>Math.round(p.y)), [80,70,60,50,40,30,20,10]);
    bands.update({enabled:false, points, loopLengthMm:20});
    assert.equal(bands.visible, false); assert.equal(bands.count, 0);
    bands.dispose(); material.dispose();
});

test('adaptive mesh spacing does not shift the material graduations around a bend', () => {
    const material = new THREE.MeshBasicMaterial();
    const bands = new CatheterGraduationMarkers(0.83, 1000, material);
    const sparse = {coordinates:[-100,-50,0], positions:[0,0,0, 0,50,0, 50,50,0]};
    const refined = {coordinates:[-100,-80,-50,-35,0], positions:[0,0,0, 0,20,0, 0,50,0, 15,50,0, 50,50,0]};
    const states = [sparse,refined].map(view => {
        bands.update({enabled:true, points, path:getCompositeJointRenderPath(view), spanMm:100, loopLengthMm:20});
        return centers(bands).map(p=>p.toArray());
    });
    assert.deepEqual(states[0], states[1]);
    assert.deepEqual(states[0][0], [20,50,0]);
    bands.dispose(); material.dispose();
});

test('scaled variant retains pigtail mechanics and side injection ports independently per access', () => {
    const make = () => new PigtailCatheter({wire:new RodState(41,5), segmentLength:5,
        guidewireLength:200, tailProgressRef:()=>0});
    const right=make(), left=make();
    right.setType('pigtail-calibrated');
    assert.equal(right.type, 'pigtail'); assert.equal(right.calibrated, true);
    assert.equal(left.calibrated, false);
    // Synthetic accepted rod pose: render and injection use the same catheter.
    right.physicsBody={activeStart:0,activeEnd:2,count:3,segmentLength:50,
        x:[0,0,0],y:[0,50,100],z:[0,0,0],
        jointStateView:{coordinates:[0,50,100],positions:[0,0,0,0,50,0,0,100,0]}};
    right.physicsActiveCount=3;
    right.updateMesh();
    assert.ok(right.graduationMarkers.count>0);
    assert.equal(right.getInjectionPorts().length,8);
    assert.ok(right.getInjectionPorts().every(p=>p.kind==='pigtail-side'));
    right.setType('pigtail');
    assert.equal(right.graduationMarkers.visible,false);
    right.dispose(); left.dispose();
});

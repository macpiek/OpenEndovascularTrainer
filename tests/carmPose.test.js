import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {applyAnteriorCArmPose} from '../src/carmPose.js';

const pivot = new THREE.Vector3(20,-220,60), radius = 350;
const deg = THREE.MathUtils.degToRad;

test('source and optical frame stay on the anterior hemisphere for every permitted tilt and roll',()=>{
    const camera = new THREE.PerspectiveCamera();
    for(const pitch of [-45,0,45])for(const roll of [-90,0,90])for(let yaw=-270;yaw<=270;yaw++){
        const angles=applyAnteriorCArmPose(camera,pivot,radius,deg(yaw),deg(pitch),deg(roll));
        const source=camera.position.clone().sub(pivot);
        const forward=camera.getWorldDirection(new THREE.Vector3());
        assert.ok(source.z>=0,`posterior source at requested yaw ${yaw}`);
        assert.ok(forward.z<=1e-12,`posterior view at requested yaw ${yaw}`);
        assert.ok(source.clone().normalize().add(forward).length()<1e-12,'optical frame must face the isocentre');
        assert.ok(Math.abs(source.length()-radius)<1e-10);
        assert.ok(Math.abs(angles.yaw)<=Math.PI/2);
    }
});

test('reversing at both lateral stops retraces exactly the same poses, including image projection',()=>{
    const camera=new THREE.PerspectiveCamera(45,1,.1,2000);
    const landmark=pivot.clone().add(new THREE.Vector3(24,18,35));
    for(const sign of [-1,1])for(const pitch of [-45,0,45]) {
        const outward=[];
        for(let angle=0;angle<=90;angle++){
            applyAnteriorCArmPose(camera,pivot,radius,sign*deg(angle),deg(pitch),.2);
            outward.push({p:camera.position.clone(),q:camera.quaternion.clone(),image:landmark.clone().project(camera)});
        }
        // A sustained outward command must remain at the lateral stop.
        applyAnteriorCArmPose(camera,pivot,radius,sign*deg(120),deg(pitch),.2);
        assert.ok(camera.position.distanceTo(outward[90].p)<1e-10);
        for(let angle=90;angle>=0;angle--){
            applyAnteriorCArmPose(camera,pivot,radius,sign*deg(angle),deg(pitch),.2);
            assert.ok(camera.position.distanceTo(outward[angle].p)<1e-10);
            assert.ok(camera.quaternion.angleTo(outward[angle].q)<1e-7);
            assert.ok(landmark.clone().project(camera).distanceTo(outward[angle].image)<1e-10);
        }
        assert.ok(camera.position.z>pivot.z);
    }
});

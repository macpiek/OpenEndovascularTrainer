import test from 'node:test';
import assert from 'node:assert/strict';
import * as T from 'three';
import { createAnatomyProjectionCamera } from '../src/imaging/anatomyProjectionCamera.js';
import { applyAnteriorCArmPose } from '../src/carmPose.js';
import { applyCArmProjection } from '../src/imaging/carmProjection.js';

test('table translation never clips anatomy at the detector, without changing its field or the source camera', () => {
    const source = new T.PerspectiveCamera(45, 1, .1, 700);
    const geometry = new T.BoxGeometry(700, 1800, 450);
    const anatomy = new T.Mesh(geometry);
    anatomy.position.set(0, -200, -120);
    const projection = createAnatomyProjectionCamera(source, anatomy);
    const bounds = new T.Box3().setFromObject(anatomy);
    const point = new T.Vector3();
    try {
        for (const yaw of [-90,-50,0,50,90]) for (const pitch of [-45,0,45])
            for (const x of [-200,0,200]) for (const y of [-1300,-390,-200,300])
                for (const height of [-40,60,180]) for (const detector of [200,350,600]) {
                    applyAnteriorCArmPose(source, new T.Vector3(x,y,120-height), 350,
                        T.MathUtils.degToRad(yaw), T.MathUtils.degToRad(pitch), .4);
                    applyCArmProjection(source,350,detector,1.4,300);
                    const sourceProjection = source.projectionMatrix.toArray();
                    const actual = projection.update();
                    assert.equal(source.far,350+detector);
                    assert.deepEqual(source.projectionMatrix.toArray(),sourceProjection);
                    assert.equal(actual.near,source.near);
                    assert.deepEqual(actual.matrixWorld.toArray(),source.matrixWorld.toArray());
                    assert.ok(actual.position.distanceTo(source.position)<1e-9);
                    assert.ok(actual.quaternion.angleTo(source.quaternion)<1e-7);
                    for (let index=0;index<8;index++) {
                        point.set(index&1?bounds.max.x:bounds.min.x,
                            index&2?bounds.max.y:bounds.min.y,index&4?bounds.max.z:bounds.min.z);
                        const original=point.clone().project(source), fitted=point.clone().project(actual);
                        const depth=-point.clone().applyMatrix4(actual.matrixWorldInverse).z;
                        if(depth>actual.near) {
                            assert.ok(Math.abs(original.x-fitted.x)<1e-7 && Math.abs(original.y-fitted.y)<1e-7,
                                'detector position, scale and 2D coverage must remain identical');
                            assert.ok(fitted.z<1,'anatomy must fit in the depth range');
                        }
                    }
                }
    } finally { geometry.dispose(); anatomy.material.dispose(); }
});

test('anatomy projection handles loading, transformed anatomy and parented cameras', () => {
    const source=new T.PerspectiveCamera(45,1,.1,100);
    const parent=new T.Group();parent.position.set(30,70,120);parent.rotation.y=.6;parent.add(source);
    const anatomy=new T.Group(),projection=createAnatomyProjectionCamera(source,anatomy);
    parent.updateMatrixWorld(true);
    assert.equal(projection.update().far,100,'empty model should retain the source range');
    const mesh=new T.Mesh(new T.BoxGeometry(100,100,100));anatomy.add(mesh);anatomy.position.set(-300,0,-1000);
    try {
        const actual=projection.update(),position=new T.Vector3();
        source.getWorldPosition(position);assert.ok(position.distanceTo(actual.position)<1e-9);
        assert.ok(actual.far>100);
        assert.ok(anatomy.position.clone().project(actual).z<1);
        const previousFar=actual.far;
        anatomy.position.z=-3000;
        assert.ok(projection.update().far>previousFar+500,'moved anatomy must expand the projection range');
    } finally {mesh.geometry.dispose();mesh.material.dispose();}
});

import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {fixture,place,finish} from './helpers/stentGraftFixture.js';
import {StentGraftWallFit} from '../src/devices/stentGraftWallFit.js';

test('sparse centreline fallback cannot create a phantom wall in the aneurysm or collapse a released body',()=>{
    const f=fixture();
    try {
        const wall=new StentGraftWallFit({geometry:f.geometry,contactField:f.contactField});
        const point=new THREE.Vector3(4,-217,0),anchor=new THREE.Vector3(4,-217,-5);
        const approximate=f.contactField.querySphere(point,.7);
        assert.equal(approximate.source,'centerline-estimate');
        assert.equal(approximate.violation,true,'reproduce the false collision');
        const exact=f.geometry.boundsTree.closestPointToPoint(point);
        assert.ok(exact.distance>10,'the real vessel wall is far away');
        assert.equal(f.contactField.packedLumenField.isInsideCoordinates(point.x,point.y,point.z),true);
        assert.ok(wall.fit(point.clone(),anchor,.7).distanceTo(point)<1e-6,'free lumen does not deform a section');
        const outside=new THREE.Vector3(4,-217,70),fitted=wall.fit(outside.clone(),anchor,.7);
        assert.ok(fitted.distanceTo(outside)>1,'actual wall still stops expansion');
        assert.ok(f.contactField.packedLumenField.isInsideCoordinates(fitted.x,fitted.y,fitted.z));
        assert.equal(f.geometry.boundsTree.raycastFirst(new THREE.Ray(anchor, fitted.clone().sub(anchor).normalize()),THREE.DoubleSide,0,anchor.distanceTo(fitted)),null,'no wall crossing');
        place(f.system,'right','body');assert.equal(f.system.deploy('right').ok,true);finish(f.system,'right');
        const part=f.system.accesses.right.device.parts[0];let minimum=Infinity;
        for(let i=0;i<part.rows;i++)for(let j=0;j<part.sides;j++) {
            const vertex=new THREE.Vector3().fromArray(part.target,(i*part.sides+j)*3);
            const rest=part.restRadii[i*part.sides+j];
            minimum=Math.min(minimum,vertex.distanceTo(part.points[i])/rest);
            assert.equal(wall.query(vertex,.2).violation,false,'released fabric stays in the vessel');
        }
        assert.ok(minimum>.5,`no unsupported pinch: minimum rest-radius fraction ${minimum}`);
    } finally {f.dispose();}
});

import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {fixture} from './helpers/stentGraftFixture.js';
import {rotateGraftPose} from '../src/devices/stentGraftRotation.js';

for(const side of ['right','left'])test(`${side}: moving an attached body transports the sewn assembly without folding it along the ipsilateral elbow`,()=>{
    const {system,device:d}=previewFixture(side);
    try {
        system.updateAccess(side,101.6/12,null,{deviceId:d.id,release:'sheath'});
        const reference=d.parts.flatMap(p=>p.points.map(q=>q.clone()));
        const position=d.implantPosition;
        for(const shift of [-15,15,-30,0]) {
            rotateGraftPose(d,.6,position+shift);
            const points=d.parts.flatMap(p=>p.points);
            // Pairwise distances across the trunk AND both branches detect
            // the artificial hinge, which intra-ring planarity tests missed.
            for(let i=0;i<points.length;i++)for(let j=i+1;j<points.length;j++)
                assert.ok(Math.abs(points[i].distanceTo(points[j])-reference[i].distanceTo(reference[j]))<1e-6,'unloaded assembly retains its shape under translation and roll');
        }
    } finally {system.dispose();}
});

test('partly released 23 mm body translated from the neck to 29.3 cm keeps an open trunk and respects the actual aneurysm wall',()=>{
    const f=fixture(),{system:s}=f;
    try {
        s.load('right','body');s.positionAtTarget('right');const d=s.accesses.right.device;
        d.position=d.target;s.deploy('right');
        s.updateAccess('right',101.6/12,null,{deviceId:d.id,release:'sheath'});
        s.updateAccess('right',.37,null,{deviceId:d.id,release:'tip'});
        s.updateAccess('right',1/60,null,{deviceId:d.id,mechanicalPosition:293.333333333});
        assert.equal(d.gateOpening,1);assert.equal(d.ipsiOpening,0);assert.ok(d.tipRelease<1);
        const trunk=d.parts[0],direction=trunk.points.at(-1).clone().sub(trunk.points[0]).normalize();
        for(let i=1;i<trunk.rows;i++) {
            const segment=trunk.points[i].clone().sub(trunk.points[i-1]).normalize();
            assert.ok(segment.dot(direction)>.98,'body cannot fold back at the transport-axis bifurcation');
        }
        for(const p of d.parts)for(let i=0;i<p.target.length;i+=3) {
            const point=new THREE.Vector3().fromArray(p.target,i);
            assert.equal(d.wallFit.query(point,.2).violation,false,'nominal expanded shape stays inside the vessel');
        }
        assert.ok(s.captureForAccess('right'),'attachment still transmits force until released');
    } finally {f.dispose();}
});

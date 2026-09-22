import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {fixture,place} from './helpers/stentGraftFixture.js';
import {AORTIC_NECK} from '../src/devices/stentGraftPaths.js';
import {StentGraftWallFit} from '../src/devices/stentGraftWallFit.js';
import {createContactResult} from '../src/physics/collision/vesselContactField.js';

for(const [variant,side] of [['Aorta_plain','left'],['Aorta_infrarenal_aneurysm','right']])
test(`${variant}: eccentric delivery does not tilt the graft rim or push the releasing crown through the wall`,()=>{
    const f=fixture(variant),{system,sources,contactField}=f;
    try {
        place(system,side,'body');const d=system.accesses[side].device;d.diameter=20;
        const fit=new StentGraftWallFit({geometry:f.geometry,contactField});
        // Reproduce a guidewire near the lateral wall, rather than the ideal
        // centreline used by the original deployment tests.
        for(const p of sources[side].nodes) {
            const shift=6*Math.exp(-p.distanceToSquared(AORTIC_NECK)/900);
            p.copy(fit.fit(p.clone().add(new THREE.Vector3(shift,0,0)),p,.9));
        }
        const nose=system.getPath(side).sample(d.position),center=system.routes[side].nearest(nose);
        assert.ok(nose.distanceTo(center.point)>2,'fixture must be eccentric');
        assert.equal(system.deploy(side).ok,true);
        assert.ok(d.parts[0].points[0].distanceTo(center.point)<1e-6,'released rim follows the vessel, not an isolated offset wire node');
        const outward=d.parts[0].points[0].clone().sub(d.parts[0].points[1]).normalize();
        const crownDirection=d.crownPath.points[1].clone().sub(d.crownPath.points[0]).normalize();
        assert.ok(outward.dot(crownDirection)>.9,'crown continues the graft axis smoothly');
        const result=createContactResult();
        const checkCrown=()=>{
            let outside=0,worst=0,total=0;
            for(const line of d.crownPolylines)for(let i=1;i<line.length;i++)for(const t of [0,.25,.5,.75,1]) {
                const point=line[i-1].clone().lerp(line[i],t),hit=contactField.querySphere(point,.22,result);
                total++;if(hit.violation){outside++;worst=Math.max(worst,hit.penetration);}
            }
            assert.equal(outside,0,`${outside}/${total} crown samples outside, worst ${worst} mm at tip release ${d.tipRelease}`);
        };
        const step=(dt,release)=>system.updateAccess(side,dt,null,{deviceId:d.id,release});
        step(100,'sheath');assert.equal(d.releaseStage,'tip');checkCrown();
        for(let i=0;i<4;i++){step(.25,'tip');checkCrown();}
        step(100,'sheath');assert.equal(d.phase,'deployed');
        for(const part of d.parts) {
            const positions=part.mesh.geometry.attributes.position;
            for(let i=0;i<positions.count;i++)assert.equal(contactField.querySphere(new THREE.Vector3().fromBufferAttribute(positions,i),.2,result).violation,false,'fabric has wall clearance');
        }
        assert.ok(system.surface,'contact surface is still published after release');
    } finally {f.dispose();}
});

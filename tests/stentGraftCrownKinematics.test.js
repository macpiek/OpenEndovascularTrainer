import test from 'node:test';
import assert from 'node:assert/strict';
import {Vector3} from 'three';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {fixture,place} from './helpers/stentGraftFixture.js';
import {fabricPoint} from '../src/devices/stentGraftScaffold.js';

function verify(crown,mesh,material,rootPart) {
    assert.equal(crown.length,36);let offset=0;
    for(let i=0;i<crown.length;i++) {
        const expected=i%3===2?material.hookLength:material.armLength;
        const points=crown[i];let length=0,rendered=0;
        for(let j=1;j<points.length;j++,offset++) {
            length+=points[j-1].distanceTo(points[j]);
            assert.ok(points[j].distanceTo(points[0].clone().lerp(points.at(-1),j/(points.length-1)))<1e-8,'metal arms stay straight');
            rendered+=new Vector3().fromBufferAttribute(mesh.geometry.attributes.instanceStart,offset).distanceTo(
                new Vector3().fromBufferAttribute(mesh.geometry.attributes.instanceEnd,offset));
        }
        assert.ok(Math.abs(length/expected-1)<1e-4,`arm ${i}: length ${length}, expected ${expected}`);
        assert.ok(Math.abs(rendered/expected-1)<1e-4,`rendered arm ${i}: length ${rendered}, expected ${expected}`);
        if(i%3!==2) {
            const wave=Math.floor(i/3),angle=(wave+(i%3===1?1:0))/12*2*Math.PI;
            assert.ok(points[0].distanceTo(fabricPoint(rootPart,0,angle))<1e-6,'wire root stays sewn to fabric');
        }
        if(i%3===1)assert.ok(points.at(-1).distanceTo(crown[i-1].at(-1))<1e-10,'arms share one apex');
    }
}
for(const scale of [1,.7789879467])for(const diameter of [23,36])test(`crown ${diameter} mm, scale ${scale}: same metal in cover, release, rotation and recapture`,()=>{
    const {system,device:d}=previewFixture('right','body',false,'iis-103');
    try {
        d.dimensionScale=scale;system.setDiameter('right',diameter);system.refreshDelivery('right');
        const folded=d.foldedPreview.at(-1),material={...folded.crownMaterial};
        verify(folded.crownPolylines,folded.rings,material,d.foldedPreview[0]);
        system.deploy('right');assert.deepEqual(d.crownMaterial,material);
        const check=()=>{
            verify(d.crownPolylines,d.crown,material,d.parts[0]);
            if(d.tipRelease<1)for(const [i,line] of d.crownPolylines.entries())if(i%3!==2)
                assert.ok(line.at(-1).distanceTo(d.crownCaptured)<1e-4,'captured crown cannot detach from its latch');
        };check();
        for(let i=0;i<8;i++){system.updateAccess('right',d.sheathTravel/12/8,null,{deviceId:d.id,release:'sheath',mechanicalRotation:i*.1});check();}
        system.updateAccess('right',100,null,{deviceId:d.id,release:'resheath'});check();
        system.updateAccess('right',100,null,{deviceId:d.id,release:'sheath'});check();
        system.updateAccess('right',100,null,{deviceId:d.id,release:'tip'});check();
    }finally{system.dispose();}
});
test('anatomy-fitted suprarenal metal keeps its length while captured and released',()=>{
    const f=fixture();try {
        place(f.system,'right','body');f.system.deploy('right');const d=f.system.accesses.right.device;
        for(const release of ['sheath','tip']) {
            f.system.updateAccess('right',100,null,{deviceId:d.id,release});
            verify(d.crownPolylines,d.crown,d.crownMaterial,d.parts[0]);
        }
    }finally{f.dispose();}
});

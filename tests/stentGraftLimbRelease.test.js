import test from 'node:test';
import assert from 'node:assert/strict';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';
import {graftAttached} from '../src/devices/stentGraftDeployment.js';
import {LIMB_MODELS,limbModel,deliveryRadiusMm} from '../src/devices/stentGraftModels.js';
import {limbCards} from '../src/ui/stentGraftModelCards.js';

for(const side of ['left','right'])test(`${side}: uncovered limb detaches before settling and cannot follow withdrawal or roll`,()=>{
    const {system,device:d}=previewFixture(side,'limb');
    const step=(dt,release=null,position=d.position,angle=0)=>system.updateAccess(side,dt,null,{deviceId:d.id,release,mechanicalPosition:position,mechanicalRotation:angle});
    const target=()=>d.parts.map(p=>Array.from(p.target));
    const displayed=()=>d.parts.map(p=>Array.from(p.mesh.geometry.attributes.position.array));
    try {
        assert.equal(d.sheathTravel,d.length,'numerical contact margin cannot retain an uncovered implant');
        step((d.length-1)/12,'sheath');assert.equal(graftAttached(d),true);
        step(1/12,'sheath');assert.equal(d.limbReleased,true);assert.equal(graftAttached(d),false);
        const origin=d.implantPosition,rotation=d.graftRotation,shape=target();
        step(.1,null,origin-5,.6);
        assert.equal(d.implantPosition,origin);assert.equal(d.graftRotation,rotation);assert.deepEqual(target(),shape);
        step(.2,'resheath',origin-10,1.2);
        for(let i=0;i<180&&d.phase!=='deployed';i++)step(1/60);
        assert.equal(d.phase,'deployed');
        const deployed=displayed(),surface=system.surface;
        for(let i=1;i<=8;i++)step(.1,'resheath',origin-10-i*5,i*.3);
        assert.equal(d.phase,'deployed');assert.equal(graftAttached(d),false);
        assert.deepEqual(displayed(),deployed,'implant geometry is fixed while the delivery system moves');
        assert.equal(system.surface,surface,'contact geometry stays at the implanted pose');
        assert.equal(d.implantPosition,origin);
    }finally{system.dispose();}
});

test('all 30 shared II/IIs limbs have catalogue dimensions, delivery OD and an illustrated choice',()=>{
    assert.equal(LIMB_MODELS.length,30);assert.equal(new Set(LIMB_MODELS.map(m=>m.id)).size,30);
    const cards=limbCards(LIMB_MODELS);assert.equal((cards.match(/<svg /g)||[]).length,30);
    for(const m of LIMB_MODELS) {
        assert.equal(m.diameter,16);
        assert.equal(m.id,`ETLW16${m.distalDiameter}C${m.length}EE`);
        assert.equal(deliveryRadiusMm({...m,type:'limb',modelId:m.id})*6,m.distalDiameter>16||m.length>124?16:14);
        assert.ok(cards.includes(`id="graftLimb-${m.id}"`));
        assert.equal(limbModel(m.id),m);
        const {system,device}=previewFixture('left','limb',false,m.id);
        try {
            assert.equal(device.length,m.length);assert.equal(device.diameter,16);assert.equal(device.distalDiameter,m.distalDiameter);
            system.deploy('left');system.updateAccess('left',100,null,{deviceId:device.id,release:'sheath'});
            assert.equal(device.phase,'deployed');
            const part=device.parts[0];assert.equal(part.path.length,m.length);
            assert.equal(part.rowRadii[0],8);assert.equal(part.rowRadii.at(-1),m.distalDiameter/2);
            assert.ok(part.scaffoldRings.every(r=>Math.abs(r.lengthError)<.0001),'nominal tapered geometry preserves metal length');
        }finally{system.dispose();}
    }
});

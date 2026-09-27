import test from 'node:test';
import assert from 'node:assert/strict';
import {MAIN_BODY_MODELS,bodyDimensions,proximalDiameters,distalDiameters} from '../src/devices/stentGraftModels.js';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';

test('main body nominal dimensions match Endurant IFU tables 2/3 and catalogue A–D',()=>{
    for(const model of MAIN_BODY_MODELS)for(const diameter of proximalDiameters(model.id))for(const distalDiameter of distalDiameters(model.id,diameter)) {
        const d=bodyDimensions({modelId:model.id,diameter,distalDiameter});
        assert.equal(d.length,model.id==='iis-103'?103:Number(model.id.slice(3)));
        assert.equal(d.trunkLength,model.id==='ii-124'?40:50);
        assert.equal(d.gateLength,model.id==='ii-124'?74:84);
        assert.equal(d.contraLength,34);
        assert.equal(d.gateDiameter,model.id!=='iis-103'&&diameter===23?12:14);
        assert.equal(d.trunkLength+d.ipsiLength,d.length);
        assert.equal(d.ipsiRootDiameter+d.gateDiameter,d.crotchDiameter);
    }
});

test('large Endurant II distal diameter is reached by a taper, not a step at the crotch',()=>{
    const {system,device:d}=previewFixture('right','body',false,'ii-166');
    try {
        system.setDiameter('right',32);system.setDistalDiameter('right',20);system.deploy('right');
        const [trunk,ipsi,contra]=d.parts;
        assert.equal(trunk.rowRadii[0],16);assert.equal(trunk.rowRadii.at(-1),15);
        assert.equal(ipsi.rowRadii[0],8);assert.equal(ipsi.rowRadii.at(-1),10);
        assert.equal(contra.radius,7);
        assert.ok(ipsi.rowRadii.some(r=>r>8&&r<10));
        for(const part of d.parts)assert.ok(Array.from(part.target).every(Number.isFinite));
        assert.equal(contra.path.length,34);
    }finally{system.dispose();}
});

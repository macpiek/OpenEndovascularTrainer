import test from 'node:test';
import assert from 'node:assert/strict';
import {LIMB_MODELS,nominalPartRadius,limbProfileDimensions} from '../src/devices/stentGraftModels.js';
import {previewFixture} from './helpers/stentGraftPreviewFixture.js';

// Independent catalogue examples: proximal straight / transition / distal straight.
for(const [distal,length,proximal,transition,terminal] of [
    [20,82,42,10,30],[24,93,33,20,40],[28,199,139,20,40],
    [10,82,32,20,30],[13,156,74,10,72],[10,199,64,20,115],
])test(`ETLW 16/${distal}/${length}: diameter changes only in the catalogue transition`,()=>{
    const device={type:'limb',diameter:16,distalDiameter:distal,length};
    assert.deepEqual(limbProfileDimensions(device),{transitionStart:proximal,transitionLength:transition,distalStraight:terminal});
    for(let s=0;s<=proximal;s+=.5)assert.equal(nominalPartRadius(device,0,s),8);
    assert.equal(nominalPartRadius(device,0,proximal+transition/2),(16+distal)/4);
    for(let s=proximal+transition;s<=length;s+=.5)assert.equal(nominalPartRadius(device,0,s),distal/2);
});

test('all limb fabric/contact rows and scaffold rest radii share the localised diameter profile',()=>{
    for(const model of LIMB_MODELS) {
        const {system,device}=previewFixture('left','limb',true,model.id);
        try {
            const part=device.parts[0],profile=limbProfileDimensions(device);
            for(let i=0;i<part.rows;i++) {
                const s=part.path.coordinates[i];
                assert.ok(Math.abs(part.rowRadii[i]-part.nominalRadius(s))<1e-10);
                if(s<=profile.transitionStart)assert.equal(part.rowRadii[i],8);
                if(s>=profile.transitionStart+profile.transitionLength)assert.equal(part.rowRadii[i],model.distalDiameter/2);
            }
            assert.ok(Array.from(part.target).every(Number.isFinite));
        }finally{system.dispose();}
    }
});

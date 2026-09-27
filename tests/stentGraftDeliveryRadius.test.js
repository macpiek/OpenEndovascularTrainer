import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {deliveryRadiusMm} from '../src/devices/stentGraftModels.js';
import {fixture} from './helpers/stentGraftFixture.js';
import {restoreSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {createStentGraftContacts} from '../src/devices/stentGraftContacts.js';
import {VesselContactField} from '../src/physics/collision/vesselContactField.js';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';

test('delivery OD follows device size rather than assigning every limb an 18 Fr body sheath',()=>{
    for(const diameter of [23,25,28,32,36])for(const length of [103,124,145,166])
        assert.equal(deliveryRadiusMm({type:'body',diameter,length})*6,diameter>=32?20:18);
    for(const diameter of [10,13,16,20,24,28])for(const length of [82,93,124,146,156,199])
        assert.equal(deliveryRadiusMm({type:'limb',diameter,length})*6,diameter>16||length>124?16:14);
});

test('captured 92.5 mm contralateral insertion continues with catalogue delivery radius and full collisions',()=>{
    const input=JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/shared-axis/stent-graft-limb-feed-92.50.json.gz',import.meta.url))));
    // Keep the exact captured pose/contact history, correcting the old material radius.
    const radius=deliveryRadiusMm({type:'limb',diameter:16,length:80});
    input.tools.find(t=>t.id==='catheter').radius=radius;
    input.stepRequest.tools.find(t=>t.id==='catheter').radius=radius;
    input.sheath.innerRadius=radius+.2;
    const f=fixture(),bytes=readFileSync(new URL('../res/Aorta_infrarenal_aneurysm.collision.bin',import.meta.url));
    const field=new VesselContactField(decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength)),{fallbackGeometry:f.geometry,bvhValidationDistance:.02,capsuleBvhValidation:-.1});
    let geometry;
    try {
        let state=restoreSharedAxisReplay(input,field),rotations=input.stepRequest.rotations;
        geometry=state.graftReplayGeometry;
        const surface=state.wallSamples.find(s=>s.graftSurface).surface;
        const req=input.stepRequest;
        for(let step=0;step<24;step++) {
            if(step) {
                const graft=createStentGraftContacts(surface,state);
                state={...state,wallSamples:[...state.wallSamples.filter(s=>!s.graftSurface),graft],graftRevision:surface.revision,graftRecovery:graft.recovery};
            }
            const tools=req.tools.map(t=>({...t,insertion:t.insertion+(t.id==='catheter'?step*25*req.dt:0)}));
            const iterator=advanceSharedAxis(state,rotations,req.dt,tools,req.options);let next;
            do{next=iterator.next();}while(!next.done);
            const result=next.value.result;
            assert.ok(next.value.state&&result.converged,JSON.stringify({step,result}));
            assert.ok(result.factorizations<256,`step ${step}: runaway factorization loop (${result.factorizations})`);
            assert.ok(result.quality.maxPenetration<1e-4,'finite-radius vessel contacts remain enforced');
            state=next.value.state;rotations=next.value.rotations;
        }
        assert.ok(state.materials.find(m=>m.spec.id==='catheter').spec.insertion>102.49);
    }finally{geometry?.dispose();f.dispose();}
});

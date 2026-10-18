import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {fixture} from './helpers/stentGraftFixture.js';
import {restoreSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {createStentGraftContacts} from '../src/devices/stentGraftContacts.js';
import {VesselContactField} from '../src/physics/collision/vesselContactField.js';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';

test('captured crown at 75 mm cover withdrawal settles without alternating bifurcation impulses',t=>{
    const input=JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/shared-axis/stent-graft-captured-bifurcation.json.gz',import.meta.url))));
    const f=fixture(),bytes=readFileSync(new URL('../res/Aorta_infrarenal_aneurysm.collision.bin',import.meta.url));
    const field=new VesselContactField(decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength)),
        {fallbackGeometry:f.geometry,bvhValidationDistance:.02,capsuleBvhValidation:-.1});
    let state=restoreSharedAxisReplay(input,field),rotations=input.stepRequest.rotations;
    const geometry=state.graftReplayGeometry,surface=state.wallSamples.find(s=>s.graftSurface).surface;
    let firstFactorizations=0,maxLateSpeed=0;
    try {
        for(let step=0;step<250;step++) {
            const contact=createStentGraftContacts(surface,state);
            const source={...state,wallSamples:state.wallSamples.map(s=>s.graftSurface?contact:s),graftRevision:surface.revision,graftRecovery:contact.recovery};
            const iterator=advanceSharedAxis(source,rotations,input.stepRequest.dt,input.stepRequest.tools,input.stepRequest.options);
            let next;do{next=iterator.next();}while(!next.done);
            const result=next.value.result;
            assert.ok(next.value.state&&result.converged,JSON.stringify({step,result}));
            assert.ok(result.quality.finite&&result.quality.maxPenetration<1e-4,'vessel containment remains enforced');
            assert.ok(result.quality.bodies.every(b=>b.maxLengthError<1e-4),'rods retain their material lengths');
            if(step===0) {
                firstFactorizations=result.factorizations;
                assert.equal(result.subdivisions,1);
                assert.ok(firstFactorizations<150,`formerly 2413 factorizations: ${firstFactorizations}`);
            }
            if(step>=220)maxLateSpeed=Math.max(maxLateSpeed,...result.quality.bodies.map(b=>b.maxSpeed));
            state=next.value.state;rotations=next.value.rotations;
        }
        // The old nearest-face recovery sustains a 18.9 mm/s two-step cycle
        // even with stationary cloth and no operator input.
        assert.ok(maxLateSpeed<1,`persistent bifurcation oscillation: ${maxLateSpeed} mm/s`);
        assert.deepEqual(state.materials.map(m=>m.spec.insertion),input.tools.map(tool=>tool.insertion));
        t.diagnostic(JSON.stringify({firstFactorizations,maxLateSpeed}));
    } finally {geometry.dispose();f.dispose();}
});

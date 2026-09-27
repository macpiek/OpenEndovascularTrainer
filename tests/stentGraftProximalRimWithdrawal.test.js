import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {fixture} from './helpers/stentGraftFixture.js';
import {restoreSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {createStentGraftContacts} from '../src/devices/stentGraftContacts.js';
import {VesselContactField} from '../src/physics/collision/vesselContactField.js';
import {createSharedAxisSegmentContact} from '../src/physics/kirchhoffSharedAxisSegmentContact.js';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';

// Exact accepted pose and indentation history immediately before the user's
// delivery-withdrawal failure, including the unchanged graft revision.
test('proximal rim: captured rejection recovers and the delivery system withdraws completely',t=>{
    const input=JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/shared-axis/stent-graft-proximal-rim-withdrawal.json.gz',import.meta.url))));
    const f=fixture(),bytes=readFileSync(new URL('../res/Aorta_infrarenal_aneurysm.collision.bin',import.meta.url));
    const field=new VesselContactField(decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength)),{fallbackGeometry:f.geometry,bvhValidationDistance:.02,capsuleBvhValidation:-.1});
    const geometries=new Set();
    try {
        let state=restoreSharedAxisReplay(input,field);geometries.add(state.graftReplayGeometry);
        const raw=state.wallSamples.find(s=>s.graftSurface).surface;
        geometries.add(raw.geometry);
        const surface=raw.restSurface??raw;
        const contact=input.failure.result.lastOutsideContact;
        const initial=createSharedAxisSegmentContact(raw.geometry).query(contact.referenceA,contact.referenceB,contact.radius);
        assert.ok(initial.distance>=contact.radius-1e-5,'relaxation leaves room for the full guidewire radius at the saved blocking rim');
        let rotations=input.stepRequest.rotations,maxIterations=0,maxFactorizations=0,totalIterations=0;
        for(let step=0;step<125;step++) {
            const sampler=createStentGraftContacts(surface,state);
            geometries.add(sampler.surface.geometry);
            const source={...state,wallSamples:[...state.wallSamples.filter(s=>!s.graftSurface),sampler],graftRevision:surface.revision,graftRecovery:sampler.recovery};
            const tools=input.stepRequest.tools.map(tool=>({...tool,insertion:Math.max(0,tool.insertion-(tool.id==='catheter'?Math.max(0,step-3)*.5:0))}));
            const iterator=advanceSharedAxis(source,rotations,input.stepRequest.dt,tools,input.stepRequest.options);let next;
            do{next=iterator.next();}while(!next.done);
            const result=next.value.result;
            assert.ok(next.value.state&&result.converged,JSON.stringify({step,result}));
            state=next.value.state;rotations=next.value.rotations;
            maxIterations=Math.max(maxIterations,result.iterations);maxFactorizations=Math.max(maxFactorizations,result.factorizations);totalIterations+=result.iterations;
            assert.ok(result.quality.maxPenetration<1e-4,'vessel containment remains enforced');
            const query=createSharedAxisSegmentContact(sampler.surface.geometry);
            for(let e=0;e<state.positions.length-1;e++) {
                const a=state.positions[e].map((v,k)=>v+state.origin[k]),b=state.positions[e+1].map((v,k)=>v+state.origin[k]);
                assert.equal(query.query(a,b,.01,{axisOnly:true}).crossing,false,`step ${step}, segment ${e}: no cloth piercing`);
            }
        }
        assert.equal(state.materials.find(m=>m.spec.id==='catheter').spec.insertion,0);
        assert.equal(state.materials.find(m=>m.spec.id==='wire').spec.insertion,input.tools.find(tool=>tool.id==='wire').insertion);
        t.diagnostic(JSON.stringify({steps:125,maxIterations,maxFactorizations,totalIterations}));
    }finally{for(const geometry of geometries)geometry?.dispose();f.dispose();}
});

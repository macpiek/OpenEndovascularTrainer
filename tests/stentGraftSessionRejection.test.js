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
for(const name of ['stent-graft-bifurcation-contact','stent-graft-lost-indentation'])test(`${name}: captured rejection recovers and permits continued withdrawal`,()=>{
    const input=JSON.parse(gunzipSync(readFileSync(new URL(`./fixtures/shared-axis/${name}.json.gz`,import.meta.url))));
    const f=fixture(),bytes=readFileSync(new URL('../res/Aorta_infrarenal_aneurysm.collision.bin',import.meta.url));
    const field=new VesselContactField(decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength)),{fallbackGeometry:f.geometry,bvhValidationDistance:.02,capsuleBvhValidation:-.1});
    let geometry;
    try {
        let state=restoreSharedAxisReplay(input,field);geometry=state.graftReplayGeometry;
        const raw=state.wallSamples.find(s=>s.graftSurface).surface;
        const surface=raw.restSurface??raw;
        const initialGeometry=state.wallSamples.find(s=>s.graftSurface).surface.geometry;
        if(name==='stent-graft-lost-indentation'){
            const query=createSharedAxisSegmentContact(initialGeometry);
            assert.ok(state.positions.slice(1).every((b,i)=>!query.query(state.positions[i].map((v,k)=>v+state.origin[k]),b.map((v,k)=>v+state.origin[k]),.01,{axisOnly:true}).crossing),'accepted indentation must remain open around the wire');
        }
        let rotations=input.stepRequest.rotations;
        for(let step=0;step<(name==='stent-graft-lost-indentation'?80:24);step++) {
            const sampler=createStentGraftContacts(surface,state),source={...state,wallSamples:[...state.wallSamples.filter(s=>!s.graftSurface),sampler],graftRevision:surface.revision,graftRecovery:sampler.recovery};
            const withdrawal=name==='stent-graft-lost-indentation'?Math.max(0,Math.min(step-3,77-step))*.1:Math.max(0,step-3)*.1;
            const tools=input.stepRequest.tools.map(t=>({...t,insertion:t.insertion-(t.id==='wire'?withdrawal:0)}));
            const iterator=advanceSharedAxis(source,rotations,input.stepRequest.dt,tools,input.stepRequest.options);let next;
            do{next=iterator.next();}while(!next.done);
            assert.ok(next.value.state&&next.value.result.converged,JSON.stringify({name,step,result:next.value.result}));
            state=next.value.state;rotations=next.value.rotations;
            assert.ok(next.value.result.quality.maxPenetration<1e-4,'vessel containment is still enforced');
            if(name==='stent-graft-lost-indentation') {
                const query=createSharedAxisSegmentContact(sampler.surface.geometry);
                assert.ok(state.positions.slice(1).every((b,i)=>!query.query(state.positions[i].map((v,k)=>v+state.origin[k]),b.map((v,k)=>v+state.origin[k]),.01,{axisOnly:true}).crossing),'continued movement must not pierce the displaced cloth');
            }
        }
        if(name==='stent-graft-lost-indentation')assert.equal(state.materials.find(m=>m.spec.id==='wire').spec.insertion,input.tools.find(t=>t.id==='wire').insertion);
        else assert.ok(state.materials.find(m=>m.spec.id==='wire').spec.insertion<input.tools.find(t=>t.id==='wire').insertion-1.9);
    }finally{geometry?.dispose();f.dispose();}
});

test('legacy exterior snapshot already crossing the vessel is rejected atomically by finite-radius discovery',()=>{
    const input=JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/shared-axis/stent-graft-exterior-withdrawal.json.gz',import.meta.url))));
    const f=fixture();let geometry;
    try {
        const state=restoreSharedAxisReplay(input,f.contactField);geometry=state.graftReplayGeometry;
        const query=createSharedAxisSegmentContact(f.geometry);
        assert.ok(state.positions.slice(1).some((b,i)=>query.query(state.positions[i].map((v,k)=>v+state.origin[k]),b.map((v,k)=>v+state.origin[k]),.45).crossing),'fixture starts with a real vessel crossing');
        const before=structuredClone({positions:state.positions,velocities:state.velocities});
        const {tools,rotations,dt,options}=input.stepRequest;
        const iterator=advanceSharedAxis(state,rotations,dt,tools,options);let next;
        do{next=iterator.next();}while(!next.done);
        assert.equal(next.value.state,undefined,'an already crossed wall cannot silently become an accepted state');
        assert.equal(next.value.result.converged,false,'batch discovery may reject another constraint first, but must never accept this state');
        assert.deepEqual({positions:state.positions,velocities:state.velocities},before,'rejection preserves the scene');
    }finally{geometry?.dispose();f.dispose();}
});

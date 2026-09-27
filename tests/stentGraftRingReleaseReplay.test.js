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

// Captured failure while opening the bifurcation with the crown released.
for(const name of ['stent-graft-ring-release-block','stent-graft-late-ring-release-block'])test(`${name}: captured rejection settles and permits withdrawal`,t=>{
    const input=JSON.parse(gunzipSync(readFileSync(new URL(`./fixtures/shared-axis/${name}.json.gz`,import.meta.url))));
    const f=fixture(),bytes=readFileSync(new URL('../res/Aorta_infrarenal_aneurysm.collision.bin',import.meta.url));
    const field=new VesselContactField(decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength)),{fallbackGeometry:f.geometry,bvhValidationDistance:.02,capsuleBvhValidation:-.1});
    const geometries=new Set();
    try {
        let state=restoreSharedAxisReplay(input,field);geometries.add(state.graftReplayGeometry);
        const raw=state.wallSamples.find(s=>s.graftSurface).surface;
        geometries.add(raw.geometry);
        const surface=raw.restSurface??raw;

        let rotations=input.stepRequest.rotations,maxIterations=0,maxFactorizations=0,totalIterations=0,firstStep;
        for(let step=0;step<24;step++) {
            const sampler=createStentGraftContacts(surface,state);
            geometries.add(sampler.surface.geometry);
            const source={...state,wallSamples:[...state.wallSamples.filter(s=>!s.graftSurface),sampler],graftRevision:surface.revision,graftRecovery:sampler.recovery};
            const tools=input.stepRequest.tools.map(tool=>({...tool,insertion:Math.max(0,tool.insertion-(tool.id==='catheter'?Math.max(0,step-3)*.5:0))}));
            const iterator=advanceSharedAxis(source,rotations,input.stepRequest.dt,tools,input.stepRequest.options);let next;
            do{next=iterator.next();}while(!next.done);
            const result=next.value.result;
            assert.ok(next.value.state&&result.converged,JSON.stringify({step,result}));
            if(step===0)firstStep={iterations:result.iterations,factorizations:result.factorizations,fullAssemblies:result.fullAssemblies,residualAssemblies:result.residualAssemblies,ms:result.ms};
            // The older archive contains folded faces with reversed normals. Its
            // physically correct recovery costs more than the late rim case,
            // but must remain below half its original 4321-factorization replay.
            const budget=name==='stent-graft-ring-release-block'?2000:200;
            if(step===0)assert.ok(result.factorizations<budget,`captured rejection: ${result.factorizations} factorizations`);
            state=next.value.state;rotations=next.value.rotations;
            maxIterations=Math.max(maxIterations,result.iterations);maxFactorizations=Math.max(maxFactorizations,result.factorizations);totalIterations+=result.iterations;
            assert.ok(result.quality.maxPenetration<1e-4,'vessel containment remains enforced');
        }
        assert.equal(state.materials.find(m=>m.spec.id==='wire').spec.insertion,input.tools.find(tool=>tool.id==='wire').insertion);
        t.diagnostic(JSON.stringify({steps:24,firstStep,maxIterations,maxFactorizations,totalIterations}));
    }finally{for(const geometry of geometries)geometry?.dispose();f.dispose();}
});

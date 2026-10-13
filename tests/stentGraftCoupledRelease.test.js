import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {Vector3} from 'three';
import {fixture} from './helpers/stentGraftFixture.js';
import {restoreSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis,sampleSharedAxisPosition} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {createStentGraftContacts} from '../src/devices/stentGraftContacts.js';
import {createCapturePotential} from '../src/devices/stentGraftCapture.js';
import {advanceRelease} from '../src/devices/stentGraftDeployment.js';
import {deliveryMechanicalExtent} from '../src/devices/stentGraftDeliveryMechanics.js';

test('coupled rod and moving graft release passes the bifurcation without impulses, then settles',t=>{
    const input=JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/shared-axis/stent-graft-captured-bifurcation.json.gz',import.meta.url))));
    const f=fixture(),side=input.accessId??'right',dt=1/60;
    let state=restoreSharedAxisReplay(input,f.contactField),rotations=input.stepRequest.rotations;
    const geometry=state.graftReplayGeometry;
    // Reuse the anatomical rod pose, but start a fresh, crimped device. The
    // first second equilibrates its cover stiffness before sheath withdrawal.
    state={...state,wallSamples:state.wallSamples.filter(s=>!s.graftSurface&&!s.graftCapture),graftRevision:0,graftRecovery:undefined};
    const source=()=>{
        const nodes=state.coordinates.map(s=>new Vector3(...sampleSharedAxisPosition(state,s)));
        const catheter=state.materials.find(m=>m.spec.id==='catheter');
        const coordinates=Float64Array.from(catheter.coordinates);
        const positions=Float64Array.from(Array.from(coordinates).flatMap(s=>sampleSharedAxisPosition(state,s)));
        return {nodes,coordinate:i=>state.coordinates[i],catheterMm:0,deliveryBody:{jointStateView:{coordinates,positions}}};
    };
    let maxReleaseSpeed=0,maxLateSpeed=0,maxReleaseFactorizations=0;
    try {
        f.sources[side]=source();f.system.load(side,'body');
        const device=f.system.accesses[side].device;
        device.position=device.target=state.materials.find(m=>m.spec.id==='catheter').spec.insertion;
        f.system.updateAccess(side,0,source());f.system.deploy(side);
        for(let step=0;step<660;step++) {
            const release=step>=60&&step<510?'sheath':null;
            // Prepare the next material coverage privately, just like the app.
            const trial={...device,parts:device.parts.map(p=>({...p,scaffoldRings:p.scaffoldRings.map(r=>({...r}))}))};
            advanceRelease(trial,dt,release);
            const extent=deliveryMechanicalExtent(trial);
            const tools=input.stepRequest.tools.map(tool=>tool.id==='catheter'?{...tool,insertion:extent.insertion,deliveryExposureMm:extent.exposedLength}:tool);
            const surface=f.system.mechanicalSurfaceForAccess(side);
            const contact=surface?createStentGraftContacts(surface,state):null,capture=f.system.captureForAccess(side);
            const wallSamples=state.wallSamples.filter(s=>!s.graftSurface&&!s.graftCapture);
            if(contact)wallSamples.push(contact);
            if(capture)wallSamples.push(createCapturePotential(capture));
            const iterator=advanceSharedAxis({...state,wallSamples,graftRevision:surface?.revision??0,graftRecovery:contact?.recovery},
                rotations,dt,tools,{...input.stepRequest.options,wasmLinearAssembly:true});
            let next;do{next=iterator.next();}while(!next.done);
            const {result}=next.value;
            assert.ok(next.value.state&&result.converged,JSON.stringify({step,result}));
            assert.ok(result.quality.finite&&result.quality.maxPenetration<1e-4,'vessel containment');
            assert.ok(result.quality.bodies.every(b=>b.maxLengthError<1e-4),'rod inextensibility');
            const speed=Math.max(...result.quality.bodies.map(b=>b.maxSpeed));
            if(step>=390) {
                maxReleaseSpeed=Math.max(maxReleaseSpeed,speed);
                maxReleaseFactorizations=Math.max(maxReleaseFactorizations,result.factorizations);
            }
            if(step>=630)maxLateSpeed=Math.max(maxLateSpeed,speed);
            state=next.value.state;rotations=next.value.rotations;
            surface?.commitContactPatches?.(state.wallSamples.find(s=>s.graftSurface)?.contactPatches??[]);
            // Both directions of coupling are essential: the accepted rods
            // move the still-supported cloth, which contacts the next rod step.
            f.system.updateAccess(side,dt,source(),{deviceId:device.id,mechanicalPosition:device.position,mechanicalRotation:0,release});
        }
        assert.ok(device.sheathWithdrawal>device.coverLead+device.parts[1].releaseOffset);
        assert.equal(device.tipRelease,0,'crown remains captured throughout');
        assert.ok(maxReleaseSpeed<20,`formerly 153.9 mm/s: ${maxReleaseSpeed}`);
        assert.ok(maxLateSpeed<2,`stationary cover must settle: ${maxLateSpeed}`);
        assert.ok(maxReleaseFactorizations<30,`formerly 113 factorizations: ${maxReleaseFactorizations}`);
        t.diagnostic(JSON.stringify({maxReleaseSpeed,maxLateSpeed,maxReleaseFactorizations}));
    } finally {geometry.dispose();f.dispose();}
});

import test from 'node:test';
import assert from 'node:assert/strict';
import {createCapturedCompositeAppFixture} from './helpers/capturedCompositeAppFixture.js';

test('actual44/52 short catheter feed crosses the native fillet-to-side boundary within the common step',()=>{
    const f=createCapturedCompositeAppFixture({wireRate:44}),changes=[];
    for(let step=0;step<=53;step++) {
        f.prepare(step>0&&step<=45?1:0,step>45?1:0);
        const result=f.system.step(f.world,f.dt),certificate=result.diagnostics?.certificate;
        assert.equal(result.accepted,true,JSON.stringify({step,status:result.status,message:result.message,certificate}));
        assert.ok(certificate.converged);assert.ok(certificate.force<=1e-7);
        if(certificate.contact)for(const sample of certificate.contact.samples)if(sample.feature&&sample.sourceFeature!==sample.feature)
            changes.push({step,...sample});
        if(certificate.friction) {
            assert.equal(certificate.friction.converged,true);
            for(const sample of certificate.friction.samples)assert.ok(sample.Fn>=0);
        }
        f.bodies.get('wire').syncToRodState(f.rod);
    }
    assert.ok(changes.some(s=>s.sourceFeature==='distal-fillet'&&s.feature==='side'&&s.step===51));
    assert.ok(Math.abs(f.catheter.progress-52*8/120)<1e-12);assert.ok(Math.abs(f.transport.progress-16.5)<1e-12);
    assert.equal(f.system.snapshot().step,54);assert.equal(f.counters.initial,1);
});

import test from 'node:test';
import assert from 'node:assert/strict';
import {createCapturedCompositeAppFixture} from './helpers/capturedCompositeAppFixture.js';

test('actual local slow catheter feed admits new distal rim on first hold and continues with original gates',()=>{
    const f=createCapturedCompositeAppFixture({coordinateOrigin:'sheath-start'}),sampleCounts=[];
    for(let step=0;step<=74;step++) {
        f.prepare(step>0&&step<=45?1:0,step>45&&step<=53?16/52:0);
        const result=f.system.step(f.world,f.dt),certificate=result.diagnostics?.certificate;
        assert.equal(result.accepted,true,JSON.stringify({step,status:result.status,message:result.message,certificate}));
        assert.equal(certificate.converged,true);assert.ok(certificate.force<=1e-7);
        if(step===53||step===54)sampleCounts.push(certificate.contact.samples.length);
        if(certificate.friction) {
            assert.equal(certificate.friction.converged,true);
            for(const sample of certificate.friction.samples)assert.ok(sample.Fn>=0);
        }
        f.bodies.get('wire').syncToRodState(f.rod);
    }
    assert.deepEqual(sampleCounts,[1,2]);
    assert.ok(Math.abs(f.transport.progress-6)<1e-12);assert.ok(Math.abs(f.catheter.progress-16*8/120)<1e-12);
    assert.equal(f.system.snapshot().step,75);assert.equal(f.counters.initial,1);
});

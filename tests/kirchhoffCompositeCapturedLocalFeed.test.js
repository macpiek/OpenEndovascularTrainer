import test from 'node:test';
import assert from 'node:assert/strict';
import {createCapturedCompositeAppFixture} from './helpers/capturedCompositeAppFixture.js';

test('persistent local app state passes captured precision stalls and the 55mm valve-grid coincidence during wire feed',()=>{
    const f=createCapturedCompositeAppFixture({wireRate:44,coordinateOrigin:'sheath-start'});
    try {
        for(let step=0;step<160;step++) {
            f.prepare(1,0);
            const result=f.system.step(f.world,f.dt);
            assert.equal(result.accepted,true,JSON.stringify({step,status:result.status,certificate:result.diagnostics?.certificate}));
            assert.ok(result.diagnostics.certificate.force<=1e-7);
            f.bodies.get('wire').syncToRodState(f.rod);
        }
        assert.equal(f.system.snapshot().step,160);
        assert.ok(f.transport.progress>58);
    } finally {f.catheter.dispose();}
});

import test from 'node:test';
import assert from 'node:assert/strict';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {readLunderquistTriaxialReplay,runLunderquistTriaxialReplay,compareTriaxialPoses} from './helpers/lunderquistTriaxialReplay.js';

test('recorded Lunderquist/sheath/delivery failure and feed/withdrawal retain strict mechanics',async()=>{
    const anatomy=await loadCoupledRuntimeAnatomy(undefined,'baseline');
    try {
        const fixture=readLunderquistTriaxialReplay();
        const reference=runLunderquistTriaxialReplay(fixture,anatomy.field,{patch:{newtonActiveSetLimit:16}});
        const fast=runLunderquistTriaxialReplay(fixture,anatomy.field);
        for(const run of [reference,fast]) {
            assert.equal(run.summary.allAccepted,true,JSON.stringify(run.steps.filter(s=>!s.accepted)));
            for(const s of run.steps) {
                assert.equal(s.subdivisions,1,'No artificial slowing/subdivision of this 1/60 s step');
                assert.ok(s.iterations<20,'The recorded stall must not return as a long iteration loop');
                assert.ok(s.certificateBound<=1e-6,'Unchanged force/friction certificate');
                assert.ok(s.residual.force<=1e-6&&s.residual.torque<=1e-6&&s.residual.length<=1e-5);
                assert.equal(s.quality.finite,true);assert.ok(s.quality.maxPenetration<=1e-5);
                for(const b of s.quality.bodies)assert.ok(b.maxLengthError<=1e-5);
            }
        }
        const diff=compareTriaxialPoses(reference.poses,fast.poses);
        assert.equal(diff.sameTopology,true);
        assert.ok(diff.maxPositionMm<1e-9,JSON.stringify(diff));
        assert.ok(diff.maxFrameComponent<1e-9,JSON.stringify(diff));
        assert.ok(diff.maxVelocity<1e-8,JSON.stringify(diff));
        assert.ok(fast.steps[0].factorizations<reference.steps[0].factorizations*.8,
            'The faster search must actually remove redundant factorizations');
    } finally {anatomy.dispose();}
});

import test from 'node:test';
import assert from 'node:assert/strict';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {readLunderquistTriaxialReplay,runLunderquistTriaxialReplay} from './helpers/lunderquistTriaxialReplay.js';
import {DEFAULT_VESSEL_ELASTICITY} from '../src/physics/vesselElasticity.js';

test('elastic anatomical wall accepts triaxial insertion, withdrawal and settling with unchanged certificates',async()=>{
    const anatomy=await loadCoupledRuntimeAnatomy(undefined,'baseline');
    try {
        const run=runLunderquistTriaxialReplay(readLunderquistTriaxialReplay(),anatomy.field,
            {patch:{vesselElasticity:DEFAULT_VESSEL_ELASTICITY}});
        assert.ok(run.summary.allAccepted,JSON.stringify(run.steps.filter(s=>!s.accepted)));
        assert.ok(run.steps.some(s=>s.quality.maxWallDisplacement>.001),'Real tool loads must deform the vessel');
        for(const s of run.steps) {
            assert.ok(s.certificateBound<=1e-6);
            assert.ok(s.residual.force<=1e-6&&s.residual.torque<=1e-6&&s.residual.length<=1e-5);
            assert.ok(s.quality.finite&&s.quality.maxPenetration<=1e-5);
            assert.ok(s.quality.maxWallDisplacement<=DEFAULT_VESSEL_ELASTICITY.maxDisplacement);
            for(const b of s.quality.bodies)assert.ok(b.maxLengthError<=1e-5);
        }
    } finally {anatomy.dispose();}
});

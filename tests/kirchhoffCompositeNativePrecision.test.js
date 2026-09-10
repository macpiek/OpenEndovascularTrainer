import test from 'node:test';
import assert from 'node:assert/strict';
import {advanceCompositeJointTimeStep} from '../src/physics/kirchhoffCompositeJointTimeStep.js';
import {createCompositeCoordinatePrecisionCase} from './helpers/compositeCoordinatePrecisionCase.js';

test('actual fractional catheter step resolves in an introducer-relative frame with identical physical gates',()=>{
    const global=createCompositeCoordinatePrecisionCase(),local=createCompositeCoordinatePrecisionCase({translated:true});
    assert.deepEqual(local.state.relative,global.state.relative);
    assert.deepEqual(local.state.materialVelocities,global.state.materialVelocities);
    assert.deepEqual(local.state.angles,global.state.angles);assert.deepEqual(local.state.restLengths,global.state.restLengths);
    assert.deepEqual(local.options.tolerances,global.options.tolerances);
    local.state.positions.forEach((p,i)=>p.forEach((v,k)=>assert.equal(v,global.state.positions[i][k]-local.origin[k])));
    const worldResult=advanceCompositeJointTimeStep(global.state,global.options),localResult=advanceCompositeJointTimeStep(local.state,local.options);
    assert.equal(worldResult.accepted,false);assert.ok(worldResult.diagnostics.certificate.force>1e-7);
    assert.equal(localResult.accepted,true,JSON.stringify({status:localResult.status,certificate:localResult.diagnostics.certificate}));
    const certificate=localResult.diagnostics.certificate;
    assert.ok(certificate.force<1e-7);assert.ok(certificate.torque<1e-8);assert.ok(certificate.length<1e-8);assert.ok(certificate.boundary<1e-9);
    assert.equal(certificate.sheath.converged,true);assert.ok(localResult.diagnostics.evaluations<20);
    assert.equal(localResult.state.time,global.state.time+global.options.dt);assert.equal(localResult.state.step,global.state.step+1);
});

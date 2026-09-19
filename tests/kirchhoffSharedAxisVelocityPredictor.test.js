import test from 'node:test';
import assert from 'node:assert/strict';
import {createSharedAxisNative,captureSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {stepSharedAxis,iterateSharedAxisTimeStep} from '../src/physics/kirchhoffSharedAxisTimeStep.js';

function rod(){
    const s=createSharedAxisNative({tools:[{id:'wire',insertion:20}]});
    s.velocities=s.positions.map(()=>[0,2,0]);return s;
}

test('velocity prediction changes only the private initial guess, preserves inertia reference and rolls back on cancellation',()=>{
    const s=rod(),before=captureSharedAxisNative(s),velocities=s.velocities,dt=1/120;let inspected=false;
    const it=iterateSharedAxisTimeStep(s,dt,{velocityPredictor:1,observeIteration:()=>{
        if(inspected)return;inspected=true;
        assert.deepEqual(s.dynamicStep.positions,before.positions);
        for(let n=0;n<s.positions.length;n++)for(let a=0;a<3;a++){
            const dof=s.layout.positions[n]+a,expected=before.positions[n][a]+(s.fixed[dof]?0:dt*.98*velocities[n][a]);
            assert.ok(Math.abs(s.positions[n][a]-expected)<1e-12);
        }
    }});
    for(let n=0;n<100&&!inspected;n++)assert.equal(it.next().done,false);
    assert.ok(inspected);it.return();
    assert.deepEqual(captureSharedAxisNative(s),before);assert.equal(s.velocities,velocities);
    assert.equal(s.dynamicStep,null);assert.equal(s.wallFrictionStep,null);
});

test('a failed predictor retries from the original pose and includes failed work',()=>{
    const a=rod(),b=rod(),common={forceTolerance:1e-6,lengthTolerance:1e-5};
    const reference=stepSharedAxis(a,1/120,common);let injected=false;
    const result=stepSharedAxis(b,1/120,{...common,velocityPredictor:1,observeTrial:()=>{
        if(!injected){injected=true;throw new Error('injected predictor failure');}
    }});
    assert.ok(injected);assert.ok(reference.converged&&result.converged);
    assert.equal(result.predictorFallbacks,1);assert.ok(result.factorizations>reference.factorizations);
    assert.deepEqual(b.positions,a.positions);assert.deepEqual(b.velocities,a.velocities);
    assert.ok(result.certificateBound<=common.forceTolerance);
});

test('velocity prediction retains the same dynamic equilibrium and force certificate for a free rod',()=>{
    const a=rod(),b=rod();a.fixed.fill(0);b.fixed.fill(0);
    const ref=stepSharedAxis(a,1/120),actual=stepSharedAxis(b,1/120,{velocityPredictor:1});
    assert.ok(ref.converged&&actual.converged);assert.ok(actual.certificateBound<=1e-6);
    assert.ok(actual.residual.length<=1e-5);
    for(let i=0;i<a.positions.length;i++)for(let k=0;k<3;k++)assert.ok(Math.abs(a.positions[i][k]-b.positions[i][k])<1e-7);
});

test('invalid prediction strengths are rejected before physical work',()=>{
    for(const velocityPredictor of [NaN,Infinity,-1,1.01]) {
        const s=rod(),before=captureSharedAxisNative(s);
        assert.throws(()=>stepSharedAxis(s,1/120,{velocityPredictor}),RangeError);
        assert.deepEqual(captureSharedAxisNative(s),before);
    }
});

import test from 'node:test';
import assert from 'node:assert/strict';
import {createCapturedCompositeAppFixture as fixture} from './helpers/capturedCompositeAppFixture.js';

test('actual captured native application keeps independent material history over45 fractional wire feeds and active-range growth',()=>{
 const f=fixture();
 for(let step=0;step<=45;step++) {
  f.prepare(step>0?1:0,0);const result=f.system.step(f.world,f.dt);
  assert.equal(result.accepted,true,JSON.stringify({step,status:result.status,message:result.message,certificate:result.diagnostics?.certificate}));
  assert.equal(f.system.snapshot().step,step+1);
  if(step===1)assert.equal(result.diagnostics.suppressedPrescribedLengthRows.length,2,'Both union children of the prescribed wire segment retain its affine BC');
  if(step===36)assert.ok(result.diagnostics.certificate.force<1e-7,'Near-zero catheter union labels stay continuous');
  f.bodies.get('wire').syncToRodState(f.rod);
 }
 assert.equal(f.counters.initial,1);assert.equal(f.counters.controls,46);assert.equal(f.bodies.get('wire').activeStart,198);
 assert.ok(Math.abs(f.transport.progress-6)<1e-12);
});

// A later side/portal support transition remains the contact adapter's own
// explicit guard. This bounded regression covers the corrected application
// feed/BC/history path through activation of the actual source window.
test('actual44/52 application rates preserve the newborn catheter tip and activate the original containment window',()=>{
 const f=fixture({wireRate:44});
 for(let step=0;step<=49;step++){
  f.prepare(step>0&&step<=45?1:0,step>45?1:0);const result=f.system.step(f.world,f.dt);
  assert.equal(result.accepted,true,JSON.stringify({step,status:result.status,message:result.message,certificate:result.diagnostics?.certificate}));
  assert.ok(result.diagnostics.certificate.force<=1e-7);
  f.bodies.get('wire').syncToRodState(f.rod);
 }
 assert.equal(f.containment.enabled,true);assert.ok(Math.abs(f.catheter.progress-52*4/120)<1e-12);
 assert.ok(Math.abs(f.transport.progress-16.5)<1e-12);assert.equal(f.counters.initial,1);
 assert.equal(f.bodies.get('catheter').activeEnd,16);
 const accepted=f.system.snapshot();assert.equal(accepted.step,50);assert.ok(accepted.nativeRateHistory);
});

import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {feedSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {evaluateSharedAxisBendLimit} from '../src/physics/kirchhoffSharedAxisBendLimit.js';

test('Pigtail withdrawal preserves necessary axis knots and solves both command directions from the accepted checkpoint',async()=>{
    const fixture=JSON.parse(readFileSync(new URL('./fixtures/shared-axis/anatomy-pigtail-wire-withdraw-200.27-incoming.json',import.meta.url)));
    const anatomy=await loadCoupledRuntimeAnatomy();
    try {
        const s=restoreSharedAxisReplay(fixture,anatomy.field),req=fixture.stepRequest;
        const before=captureSharedAxisReplay(s,fixture.sheath);
        const run=tools=>{const iterator=advanceSharedAxis(s,req.rotations,req.dt,tools,req.options);let next;
            do{next=iterator.next();}while(!next.done);return next.value;};
        const seeded=feedSharedAxisNative(s,Object.fromEntries(req.tools.map(t=>[t.id,t.insertion])));
        assert.ok(seeded.coordinates.includes(200)||seeded.coordinates.includes(200.8));
        assert.ok(seeded.coordinates.includes(req.tools.find(t=>t.id==='wire').insertion));
        for(let edge=0;edge+2<seeded.positions.length;edge++)
            assert.ok(evaluateSharedAxisBendLimit({state:seeded,edge,needHessian:false},Math.PI/4).gap>=-1e-5);
        const withdrawal=run(req.tools);
        assert.ok(withdrawal.state,JSON.stringify(withdrawal.result));assert.equal(withdrawal.result.converged,true);
        assert.equal(withdrawal.result.subdivisions,1);assert.ok(withdrawal.result.factorizations<500);
        assert.ok(withdrawal.result.certificateBound<=req.options.forceTolerance);
        assert.ok(withdrawal.result.quality.maxPenetration<=req.options.lengthTolerance);
        assert.ok(withdrawal.result.quality.bodies.every(b=>b.maxBendAngleDegrees<=45.00001));
        const saved=captureSharedAxisReplay(withdrawal.state,fixture.sheath);
        assert.deepEqual(captureSharedAxisReplay(restoreSharedAxisReplay(saved,anatomy.field),fixture.sheath),saved,
            'A retained geometric knot must round-trip without becoming a permanent boundary');
        assert.deepEqual(captureSharedAxisReplay(s,fixture.sheath),before);
        const previousWire=fixture.tools.find(t=>t.id==='wire').insertion;
        const reversed=run(req.tools.map(t=>t.id==='wire'?{...t,insertion:previousWire+44*req.dt}:t));
        assert.ok(reversed.state,JSON.stringify(reversed.result));assert.equal(reversed.result.converged,true);
        assert.ok(reversed.result.certificateBound<=req.options.forceTolerance);
        assert.ok(reversed.result.quality.maxPenetration<=req.options.lengthTolerance);
        assert.equal(reversed.state.materials.find(m=>m.spec.id==='wire').spec.insertion,previousWire+44*req.dt);
        assert.deepEqual(captureSharedAxisReplay(s,fixture.sheath),before);
    }finally{anatomy.dispose();}
});

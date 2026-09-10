import test from 'node:test';
import assert from 'node:assert/strict';
import {assessCompositeStaticKineticFriction as assess} from '../src/physics/kirchhoffCompositeStaticKineticFriction.js';
const base={normalForce:2,muStatic:[.6,.6],muKinetic:[.2,.2],mode:'static',wholeStepConverged:true,slipTolerance:1e-9,coneTolerance:1e-9,workTolerance:1e-9};
test('static equilibrium holds forces beyond the kinetic cone; only converged static slip requests breakaway',()=>{
    const held=assess({...base,traction:[-.7,0],slip:[0,0]});assert.equal(held.accepted,true);assert.equal(held.nextMode,'static');
    const slip={...base,traction:[-1.2,0],slip:[.01,0]};
    assert.equal(assess(slip).status,'breakaway');assert.equal(assess(slip).change,true);
    assert.equal(assess({...slip,wholeStepConverged:false}).status,'unconverged');
    assert.equal(assess({...slip,wholeStepConverged:false}).change,false);
    assert.equal(assess({...slip,traction:[-.7,0]}).status,'unconverged');
});
test('arbitrarily small resolved kinetic sliding retains the kinetic coefficient and has no numerical penalty threshold',()=>{
    for(const length of [1e-2,1e-5,1e-8]) {
        const r=assess({...base,mode:'kinetic',traction:[-.4,0],slip:[length,0]});
        assert.equal(r.accepted,true);assert.equal(r.status,'sliding');assert.equal(r.nextMode,'kinetic');
        assert.equal(r.physical.minimumWork,-.4*length);
    }
});
test('a loaded kinetic stop returns to static only with an exact or strict-interior physical stop certificate',()=>{
    for(const [traction,slip,certificate] of [[[-.4,0],[0,0],'exact-zero-slip'],[[-.3,0],[1e-11,0],'strict-cone-interior-and-physical-residual']]) {
        const r=assess({...base,mode:'kinetic',traction,slip});assert.equal(r.accepted,true);assert.equal(r.nextMode,'static');assert.equal(r.stopCertificate,certificate);
    }
    const boundary=assess({...base,mode:'kinetic',traction:[-.4,0],slip:[1e-11,0]});assert.equal(boundary.accepted,false);assert.equal(boundary.status,'ambiguous');
});
test('disabled axes remain free and zero kinetic friction does not invent a stop',()=>{
    const free=assess({...base,muStatic:[.6,0],muKinetic:[.2,0],traction:[-.7,0],slip:[0,1]});assert.equal(free.accepted,true);assert.equal(free.nextMode,'static');
    const moving=assess({...base,mode:'kinetic',muKinetic:[0,0],traction:[0,0],slip:[1e-12,0]});assert.equal(moving.accepted,true);assert.equal(moving.nextMode,'kinetic');
    assert.throws(()=>assess({...base,muKinetic:[.7,.2],traction:[0,0],slip:[0,0]}),/static cone/);
});

import assert from 'node:assert/strict';
import test from 'node:test';
import {mechanicalMeshFixture as fixture,mechanicalMeshOptions as options,mechanicalMeshErrors as errors,gradedMechanicalNodes} from './fixtures/compositeMechanicalMesh.js';
import {advanceCompositeJointTimeStep as advance,createCompositeJointTimeStepWorkspace as workspace} from '../src/physics/kirchhoffCompositeJointTimeStep.js';

test('mechanical mesh refinement checks reactions separately from shape while retaining every lumen surface sample',()=>{
    const specs=[undefined,[0,1,2,3,4,6,8,12,16,24,32,48,64],gradedMechanicalNodes],results=[];
    let sites;
    for(const nodes of specs) {
        const f=fixture(nodes),before=structuredClone(f.state),x=f.state.coordinates;
        const current=f.contacts.pairs.flatMap(p=>p.quadrature.map(u=>x[p.innerEdge]+u*(x[p.innerEdge+1]-x[p.innerEdge])));
        if(sites)assert.deepEqual(current,sites);else sites=current;
        const r=advance(f.state,options(f,f.state,{force:[.1,.4,0],forceNode:0,workspace:workspace(f.state)}));
        assert.equal(r.accepted,true,JSON.stringify({status:r.status,error:r.error,diagnostics:r.diagnostics}));
        assert.deepEqual(f.state,before);assert.equal(r.state.time,1/120);assert.equal(r.state.step,1);
        assert.equal(r.diagnostics.certificate.converged,true);assert.equal(r.diagnostics.lumenRows,128);assert.equal(r.diagnostics.frictionRows,256);
        assert.equal(r.diagnostics.certificate.contact.samples.length,128);assert.equal(r.diagnostics.certificate.friction.samples.length,128);
        assert.equal(r.diagnostics.contactQueries,1280);
        assert.equal(r.diagnostics.lumenGauge.redundantSamples,0);assert.deepEqual(r.diagnostics.lumenGauge.transfers,[]);
        assert.ok(r.diagnostics.certificate.force<=1e-7&&r.diagnostics.certificate.torque<=1e-8&&r.diagnostics.certificate.length<=1e-8);
        results.push(r);
    }
    const tooCoarse=errors(results[0],results[1]),refined=errors(results[0],results[2]);
    assert.ok(tooCoarse.position<.001,'shape alone would admit the 13-node grid');
    assert.ok(tooCoarse.reaction>1e-7,'the separate reaction comparison must reject it');
    assert.ok(refined.position<1e-8&&refined.reaction<1e-7,JSON.stringify(refined));
    assert.equal(results[2].state.layout.nodeCount,15);
    assert.ok(results[2].diagnostics.directionSystems.every(d=>d.solvedUnknowns<=151&&d.eliminatedZeroDuals>=381));
    assert.ok(results[0].diagnostics.directionSystems.every(d=>d.solvedUnknowns>=648));
});

import test,{after} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay,captureSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';
import {sharedAxisOuterMaterialAt} from '../src/physics/kirchhoffSharedAxisNative.js';

const anatomy=await loadCoupledRuntimeAnatomy();after(()=>anatomy.dispose());
const accelerated={contactCandidateMargin:.01,deferActiveBasis:true,lazyBasisCoefficients:true};
const finish=iterator=>{let next;do{next=iterator.next();}while(!next.done);return next.value;};
for(const name of ['anatomy-berenstein-feed-138.67-live-cycle','anatomy-pigtail-wire-withdraw-200.27-incoming',
    'anatomy-pigtail-contact-burst-incoming','anatomy-pigtail-friction-settling-incoming','anatomy-berenstein-feed-312.87-projection'])
test(`narrow vessel candidates preserve certified motion and removed wall inequalities: ${name}`,t=>{
    const fixture=JSON.parse(readFileSync(new URL(`./fixtures/shared-axis/${name}.json`,import.meta.url))),req=fixture.stepRequest;
    const outputs=[];
    for(const narrow of [false,true]) {
        const input=restoreSharedAxisReplay(fixture,anatomy.field),before=captureSharedAxisReplay(input,fixture.sheath);
        const options={...req.options,promoteTrialAssembly:true,projectionMode:'reduced',wasmMaterial:true,
            reuseConstraintWork:true,reuseMatrixAssembly:true,reuseRowBuffers:true,...(narrow?accelerated:{})};
        if(narrow) {
            const cancelled=advanceSharedAxis(input,req.rotations,req.dt,req.tools,options);
            for(let i=0;i<12;i++)if(cancelled.next().done)break;
            cancelled.return();assert.deepEqual(captureSharedAxisReplay(input,fixture.sheath),before);
        }
        const output=finish(advanceSharedAxis(input,req.rotations,req.dt,req.tools,options));
        assert.ok(output.state&&output.result.converged,JSON.stringify(output.result));
        assert.ok(output.result.certificateBound<=(req.options.forceTolerance??1e-6));
        assert.ok(output.result.quality.finite&&output.result.quality.maxPenetration<=(req.options.lengthTolerance??1e-5));
        assert.deepEqual(captureSharedAxisReplay(input,fixture.sheath),before);
        outputs.push(output);
    }
    const [reference,actual]=outputs;assert.deepEqual(actual.state.coordinates,reference.state.coordinates);
    let maxDifference=0;
    actual.state.positions.forEach((p,i)=>{
        maxDifference=Math.max(maxDifference,Math.hypot(...p.map((v,k)=>v+actual.state.origin[k]-reference.state.positions[i][k]-reference.state.origin[k])));
    });
    // Working-set changes may take a different Newton path. Keep a 1 µm
    // trajectory comparison budget; force/contact tolerances above do not change.
    assert.ok(maxDifference<1e-3,`world position difference ${maxDifference} mm`);
    // Audit the full reference set at the accelerated pose, including every
    // witness omitted from the candidate solve. This is outside the timer.
    for(const row of reference.state.definitions)if(row.witness) {
        const s=actual.state,e=row.edge,owner=sharedAxisOuterMaterialAt(s,e,row.witness.t,row.witness.owner);
        const contact=row.evaluate({state:s,a:s.positions[e],b:s.positions[e+1],radius:owner.body.radius,needHessian:false});
        assert.ok(contact.gap>=-(req.options.lengthTolerance??1e-5),`${row.id}: ${contact.gap}`);
    }
    t.diagnostic(JSON.stringify({maxDifference,referenceRows:reference.state.definitions.length,actualRows:actual.state.definitions.length}));
});

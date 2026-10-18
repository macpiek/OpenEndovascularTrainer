import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {captureRejectedSharedAxisLinearization as capture,matchesRejectedSharedAxisLinearization as matches} from '../src/physics/kirchhoffSharedAxisRejectedLinearization.js';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {restoreSharedAxisReplay} from './helpers/sharedAxisReplay.js';
import {captureSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {advanceSharedAxis} from '../src/physics/kirchhoffSharedAxisAppSystem.js';

function problem() {
    return {chain:{layout:{dofCount:3,band:2},tangent:new Float64Array([1,2,3,4,5,6,7,8,9])},
        options:{gradient:new Float64Array([3,2,1]),fixed:new Uint8Array(3),tolerance:1e-8,
            rows:[{kind:'wall',dofs:[0,1],jacobian:[1,.5],gap:-.2,multiplier:2,compliance:0,
                extraForceDofs:[2],extraForceJacobian:[-.7],geometricHessian:new Float64Array([1,2,3,4])}]}};
}
test('rejected Newton memo owns exact matrix, contact and load values, including signed zero',()=>{
    const {chain,options}=problem(),saved=capture(chain,options,{converged:false,residual:1e-6});
    assert.ok(matches(saved,chain,options));
    const changes=[
        (c,o)=>c.tangent[0]+=1e-12,(c,o)=>c.layout.dofCount++,(c,o)=>c.layout.band++,
        (c,o)=>o.gradient[0]+=1e-12,(c,o)=>o.fixed[0]=1,(c,o)=>o.tolerance*=2,
        (c,o)=>o.rows[0].kind='length',(c,o)=>o.rows[0].gap+=1e-12,
        (c,o)=>o.rows[0].multiplier+=1e-12,(c,o)=>o.rows[0].compliance=-0,
        (c,o)=>o.rows[0].dofs[0]=2,(c,o)=>o.rows[0].jacobian[0]+=1e-12,
        (c,o)=>o.rows[0].extraForceDofs[0]=1,(c,o)=>o.rows[0].extraForceJacobian[0]+=1e-12,
        (c,o)=>o.rows[0].geometricHessian[0]+=1e-12,(c,o)=>o.rows.push({...o.rows[0]}),
        (c,o)=>{c.hessian=c.tangent;delete c.tangent;}
    ];
    for(const change of changes){const p=problem();change(p.chain,p.options);assert.equal(matches(saved,p.chain,p.options),false);}
    chain.tangent.fill(0);options.rows[0].jacobian.fill(0);
    const p=problem();assert.ok(matches(saved,p.chain,p.options),'later scratch writes cannot alter the snapshot');
    delete p.options.rows[0].geometricHessian;
    const absent=capture(p.chain,p.options,{converged:false});
    assert.ok(matches(absent,p.chain,p.options));
    p.options.rows[0].geometricHessian=null;
    assert.equal(matches(absent,p.chain,p.options),false,'optional arrays keep their presence tags');
    p.options.rows[0].kind='custom';
    assert.equal(capture(p.chain,p.options,{converged:false}),null,'unknown equation kinds retain the full retry');
});

test('limits and retained-factor strategies keep the unrestricted retry',()=>{
    const {chain,options}=problem();
    for(const result of [{converged:true},{converged:false,failure:'active-set-limit'},
        {converged:false,batchFailure:'active-set-limit',failure:'band-lu-rejected'}])
        assert.equal(capture(chain,options,result),null);
    for(const extra of [{modifiedNewtonContext:{}},{incrementalContacts:true}])
        assert.equal(capture(chain,{...options,...extra},{converged:false}),null);
    assert.equal(matches(null,chain,options),false);
});

test('long crossover contact recovery preserves the accepted state while avoiding repeated failed equations',async()=>{
    const fixture=JSON.parse(gunzipSync(fs.readFileSync(new URL('../reports/performance-scenarios/001-contact-tail-replay.json.gz',import.meta.url))));
    const anatomy=await loadCoupledRuntimeAnatomy(undefined,fixture.anatomy),request=fixture.stepRequest;
    try {
        const outputs=[];
        for(const reuseRejectedNewton of [false,true]) {
            let reused=0;
            const state=restoreSharedAxisReplay(fixture,anatomy.field);
            const iterator=advanceSharedAxis(state,request.rotations,request.dt,request.tools,
                {...request.options,reuseRejectedNewton,observeTrial:event=>{if(event.kind==='reused-rejected-direction')reused++;}});
            let next;do{next=iterator.next();}while(!next.done);
            const r=next.value.result;
            assert.ok(next.value.state&&r.converged&&r.quality.finite);
            assert.ok(r.certificateBound<=1e-6&&r.residual.length<=1e-5&&r.quality.maxPenetration<=1e-5);
            outputs.push({...next.value,reused});
        }
        const [before,after]=outputs;
        assert.ok(after.reused>0);assert.equal(before.reused,0);
        assert.ok(after.result.factorizations<before.result.factorizations);
        assert.equal(after.result.subdivisions,before.result.subdivisions);
        assert.deepEqual(after.result.residual,before.result.residual);
        const a=captureSharedAxisNative(after.state),b=captureSharedAxisNative(before.state);
        for(const key of ['positions','frames','multipliers'])assert.deepEqual(a[key],b[key],key);
        for(const key of ['velocities','angularVelocities','acceptedWallGaps','wallFrictionHistory'])
            assert.deepEqual(after.state[key],before.state[key],key);
    } finally {anatomy.dispose();}
});

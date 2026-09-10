import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {measureCompositeWallConstraints} from '../src/physics/kirchhoffCompositeWallContacts.js';
import {createCompositeWallEnvelopeWorkspace,refreshCompositeWallEnvelope,canonicalizeCompositeWallEnvelope} from '../src/physics/kirchhoffCompositeWallEnvelope.js';
function fixture(){
    const layout=createCompositeChainLayout([['wire','catheter'],['wire','catheter']]),positions=[[0,.8,0],[2,.8,0],[5,.8,0]];
    const contactOwners={edges:[0,1].map(edge=>({edge,wall:{owner:'catheter',radius:.8}}))};
    const calls=[],field={queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out){
        calls.push([ax,bx,radius]);const t=ay===by?.5:ay<by?0:1;
        out.signedGap=(1-t)*ay+t*by-radius;out.segmentT=t;out.inward.values.set([0,1,0]);
        out.closestPoint.values.set([(1-t)*ax+t*bx,0,(1-t)*az+t*bz]);out.faceIndex=1;out.source='analytic-plane';out.capsuleSampleCount=2;return out;
    }};
    const w=refreshCompositeWallEnvelope({positions,contactOwners,field},createCompositeWallEnvelopeWorkspace(layout));
    return {w,positions,contactOwners,field,calls};
}
const tolerances={penalty:1e4,gapTolerance:1e-9,forceTolerance:1e-9,workTolerance:1e-9};
test('endpoints and original capsule minima are all retained, with endpoint queries shared by adjacent edges',()=>{
    const f=fixture();assert.equal(f.w.rows.length,6);assert.equal(f.calls.length,5);
    assert.deepEqual(f.w.rows.map(r=>r.t),[0,1,.5,0,1,.5]);assert.ok(f.calls.every(c=>c[2]===.8));
});
test('exact local dependence transfers the total force and torque to unique endpoints without losing original gap checks',()=>{
    const f=fixture(),lambda=Float64Array.from([1,2,6,3,4,10]);
    const original=measureCompositeWallConstraints(f.w,{...tolerances,lambdas:lambda}).physicalGradient.slice();
    const representatives=canonicalizeCompositeWallEnvelope(f.w,lambda,f.contactOwners);
    assert.deepEqual(representatives,[0,1,4]);assert.deepEqual(Array.from(lambda),[4,13,0,0,9,0]);
    const after=measureCompositeWallConstraints(f.w,{...tolerances,lambdas:lambda});
    assert.deepEqual(after.physicalGradient,original);assert.ok(after.converged);
    f.w.rows[2].gap=-.01;
    assert.equal(measureCompositeWallConstraints(f.w,{...tolerances,lambdas:lambda}).converged,false,'removed linear row never removes its original physical gap');
});
test('a distinct interior wall obstacle remains an independent physical constraint',()=>{
    const f=fixture();f.w.rows[2].gap=-.02;
    const lambda=new Float64Array(6);lambda[2]=7;
    const reps=canonicalizeCompositeWallEnvelope(f.w,lambda,f.contactOwners);
    assert.ok(reps.includes(2));assert.equal(lambda[2],7);
});
test('nearby normals and gaps are never treated as exact dependence',()=>{
    const f=fixture();f.w.rows[2].normal[0]=1e-12;
    const lambda=new Float64Array(6);lambda[2]=7;
    assert.ok(canonicalizeCompositeWallEnvelope(f.w,lambda,f.contactOwners).includes(2));assert.equal(lambda[2],7);
});

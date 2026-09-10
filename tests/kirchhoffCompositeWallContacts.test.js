import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import {createCompositeChainLayout,createCompositeChainWorkspace} from '../src/physics/kirchhoffCompositeChain.js';
import {createCompositeWallWorkspace,refreshCompositeWallContacts,assembleCompositeWallAugmented,
    measureCompositeWallConstraints} from '../src/physics/kirchhoffCompositeWallContacts.js';
import {VesselContactField,createContactResult} from '../src/physics/collision/vesselContactField.js';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';

const close=(a,b,t=1e-9)=>assert.ok(Math.abs(a-b)<=t*(1+Math.max(Math.abs(a),Math.abs(b))),`${a} != ${b}`);
function fixture() {
    const layout=createCompositeChainLayout([['wire','catheter'],['wire']]);
    const chain=createCompositeChainWorkspace(layout),workspace=createCompositeWallWorkspace(layout);
    const positions=[[0,.2,0],[2,.1,.1],[4,.9,-.2]];
    const contactOwners={edges:[{edge:0,wall:{owner:'catheter',radius:.3}},{edge:1,wall:{owner:'wire',radius:.05}}]};
    const calls=[];
    const field={queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out) {
        calls.push(radius);const first=ay<by,y=first?ay:by;
        out.signedGap=y-radius;out.segmentT=first?0:1;out.inward.values.set([0,1,0]);
        out.closestPoint.values.set([first?ax:bx,0,first?az:bz]);out.branchId=0;out.faceIndex=1;
        out.source='analytic-plane';out.capsuleSampleCount=2;return out;
    }};
    return {chain,workspace,positions,contactOwners,field,calls};
}
const zero=chain=>{chain.energy=0;chain.gradient.fill(0);chain.hessian.fill(0);};

test('one outer capsule per edge owns vessel contact; the hidden wire and sheath do not add duplicate rows',()=>{
    const f=fixture();refreshCompositeWallContacts(f,f.workspace);
    assert.deepEqual(f.calls,[.3,.05]);assert.equal(f.workspace.rows[0].owner,'catheter');
    f.contactOwners.edges[0].wall=null;refreshCompositeWallContacts(f,f.workspace);
    assert.equal(f.workspace.queries,1);assert.equal(f.workspace.rows[0].included,false);
    assert.throws(()=>assembleCompositeWallAugmented(f.workspace,f.chain,{lambdas:[1,0],penalty:10}),/traction transfer/);
});

test('unilateral wall energy has the correct exact gradient and GN tangent on a planar branch',()=>{
    const f=fixture(),lambdas=[2,0],penalty=10;
    const evaluate=()=>{zero(f.chain);refreshCompositeWallContacts(f,f.workspace);
        assembleCompositeWallAugmented(f.workspace,f.chain,{lambdas,penalty});
        return {energy:f.chain.energy,gradient:f.chain.gradient.slice()};};
    const baseline=evaluate();close(f.workspace.trialNormalForces[0],4);close(f.workspace.trialNormalForces[1],0);
    const forceIndex=f.chain.layout.positions[1]+1;
    close(baseline.gradient[forceIndex],-4);close(f.chain.hessian[forceIndex*f.chain.layout.band],10);
    for(let node=0;node<3;node++) for(let axis=0;axis<3;axis++) {
        const eps=1e-6;f.positions[node][axis]+=eps;const plus=evaluate();
        f.positions[node][axis]-=2*eps;const minus=evaluate();f.positions[node][axis]+=eps;
        close(baseline.gradient[f.chain.layout.positions[node]+axis],(plus.energy-minus.energy)/(2*eps),1e-8);
    }
});

test('original wall KKT and physical reactions are separate from augmented trial pressure',()=>{
    const f=fixture();f.positions[1][1]=.3;f.positions[0][1]=.4;refreshCompositeWallContacts(f,f.workspace);
    const options={lambdas:[3,0],penalty:10,gapTolerance:1e-9,forceTolerance:1e-9,workTolerance:1e-9};
    const r=measureCompositeWallConstraints(f.workspace,options);
    assert.ok(r.converged);close(r.physicalGradient[f.chain.layout.positions[1]+1],-3);
    assert.equal(r.scope,'discrete-wall-KKT-only');
    f.positions[1][1]=.31;refreshCompositeWallContacts(f,f.workspace);
    const separated=measureCompositeWallConstraints(f.workspace,options);
    assert.equal(separated.converged,false,'positive gap with retained physical load violates complementarity');
    close(separated.maximumProjectedResidual,.1);close(separated.maximumComplementarity,.03);
    zero(f.chain);assembleCompositeWallAugmented(f.workspace,f.chain,options);
    close(f.workspace.trialNormalForces[0],2.9);
    close(separated.physicalGradient[f.chain.layout.positions[1]+1],-3);
});

test('a penetrated unloaded wall cannot be accepted from zero complementarity alone',()=>{
    const f=fixture();refreshCompositeWallContacts(f,f.workspace);
    const r=measureCompositeWallConstraints(f.workspace,{lambdas:[0,0],penalty:10,gapTolerance:1e-9,forceTolerance:1e-9,workTolerance:1e-9});
    assert.equal(r.maximumComplementarity,0);assert.equal(r.converged,false);close(r.maximumPenetration,.2);
    assert.throws(()=>measureCompositeWallConstraints(f.workspace,{lambdas:[-1,0],penalty:10}),/nonnegative/);
});

test('the collector preserves the actual anatomy capsule provider gap, normal, owner radius and sample count',()=>{
    const bytes=fs.readFileSync(new URL('../res/Aorta_plain.collision.bin',import.meta.url));
    const asset=decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
    const field=new VesselContactField(asset),segment=asset.arrays.centerlineSegments;
    const a=Array.from(segment.slice(0,3)),b=Array.from(segment.slice(3,6));
    const positions=[a,a.map((v,i)=>(v+b[i])/2),b],layout=createCompositeChainLayout([['wire'],['wire']]);
    const contactOwners={edges:[0,1].map(edge=>({edge,wall:{owner:'wire',radius:.4445}}))};
    const expected=[0,1].map(edge=>{
        const c=field.queryCapsuleCoordinates(...positions[edge],...positions[edge+1],.4445,createContactResult());
        return {gap:c.signedGap,t:c.segmentT,normal:Array.from(c.inward.values),samples:c.capsuleSampleCount};
    });
    const w=refreshCompositeWallContacts({positions,contactOwners,field},createCompositeWallWorkspace(layout));
    for(let i=0;i<2;i++) {
        close(w.rows[i].gap,expected[i].gap);close(w.rows[i].t,expected[i].t);
        assert.deepEqual(Array.from(w.rows[i].normal),expected[i].normal);assert.equal(w.rows[i].sampleCount,expected[i].samples);
    }
    assert.equal(w.queries,2);
});

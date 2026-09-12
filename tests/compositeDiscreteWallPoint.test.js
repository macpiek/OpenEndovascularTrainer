import test from 'node:test';
import assert from 'node:assert/strict';
import {createCompositeDiscreteWallPoint as create} from '../src/physics/compositeDiscreteWallPoint.js';
const close=(a,b,t=2e-7)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t*Math.max(1,Math.abs(a),Math.abs(b)),`${a} != ${b}`);
function field(kind) {
    const f={calls:0,queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out){
        assert.equal(this,f);this.calls++;this.lastResult=out;assert.deepEqual([ax,ay,az],[bx,by,bz]);
        const p=[ax,ay,az],value=kind==='plane'?ay:2+.5*(ax-.125)*(ay-.125)+.25*az,
            g=kind==='plane'?[0,1,0]:[.5*(ay-.125),.5*(ax-.125),.25],m=Math.hypot(...g),normal=g.map(v=>v/m);
        out.source=kind==='plane'?'analytic-plane':'sparse-sdf';out.signedDistance=value;out.signedGap=value-radius;
        out.point.values.set(p);out.inward.values.set(normal);out.closestPoint.values.set(p.map((v,k)=>v-value*normal[k]));
        out.segmentT=.8;out.capsuleSampleCount=5;out.faceIndex=-1;return out;
    }};
    if(kind==='sdf') {
        Object.assign(f,{sdfOrigin:[-1,-1,-1],voxelSize:.5,brickSize:2,sdfDimensions:[3,3,3],sdfQuantization:1/1024,
            sdfBrickLookup:new Uint16Array(27),sdfDistances:new Uint32Array(216)});
        for(let bz=0;bz<3;bz++)for(let by=0;by<3;by++)for(let bx=0;bx<3;bx++) {
            const brick=bx+3*(by+3*bz);f.sdfBrickLookup[brick]=brick;
            for(let z=0;z<2;z++)for(let y=0;y<2;y++)for(let x=0;x<2;x++) {
                const px=-1+.5*(2*bx+x),py=-1+.5*(2*by+y),pz=-1+.5*(2*bz+z);
                f.sdfDistances[8*brick+x+2*(y+2*z)]=(2+.5*(px-.125)*(py-.125)+.25*pz)*1024;
            }
        }
    }
    return f;
}
for(const kind of ['plane','sdf'])for(const fraction of [0,.37,1])test(`${kind}: fixed fraction ${fraction} lifts all endpoint derivatives while preserving original query`,()=>{
    const source=field(kind),site=create({fraction,edge:7,dofs:[21,22,23,24,25,26]}),positions=[[.1,.2,.1],[.35,.4,.2]],radius=.3;
    let queries=0;const row=site.refresh({field:source,positions,radius,consumeQuery:()=>queries++});
    assert.equal(queries,1);assert.equal(source.calls,1);assert.equal(row.role,'material-point');assert.equal(row.t,fraction);
    assert.equal(row.rawContact.segmentT,.8);assert.equal(row.rawContact.capsuleSampleCount,5);assert.equal(row.derivativeUnavailable,false,row.derivativeReason);
    assert.deepEqual(Array.from(row.dofs),[21,22,23,24,25,26]);assert.equal(row.edge,7);
    row.point.forEach((v,k)=>close(v,positions[0][k]+fraction*(positions[1][k]-positions[0][k]),1e-15));
    const G=row.gapJacobian.slice(),B=row.forceColumn.slice(),DB=row.normalDerivative.slice(),h=1e-6;
    for(let j=0;j<6;j++) {
        const plus=structuredClone(positions),minus=structuredClone(positions);plus[Math.floor(j/3)][j%3]+=h;minus[Math.floor(j/3)][j%3]-=h;
        const p=structuredClone(site.refresh({field:source,positions:plus,radius})),m=site.refresh({field:source,positions:minus,radius});
        close((p.gap-m.gap)/(2*h),G[j]);
        for(let i=0;i<6;i++)close(-(p.forceColumn[i]-m.forceColumn[i])/(2*h),DB[i*6+j]);
    }
    site.refresh({field:source,positions,radius});
    for(let k=0;k<3;k++)close(-(B[k]+B[k+3]),row.normal[k],1e-14);
    const saved=Array.from(row.rawContact.queryPoint);source.lastResult.point.values[0]+=4;
    assert.deepEqual(Array.from(row.rawContact.queryPoint),saved);
});

test('unsupported and failed queries revoke derivatives without inventing an inward response',()=>{
    const source=field('plane'),site=create({fraction:.4}),positions=[[.1,.2,.1],[.3,.4,.2]];
    site.refresh({field:source,positions,radius:.3});
    const unsupported={queryCapsuleCoordinates(...args){const out=source.queryCapsuleCoordinates(...args);out.source='centerline-safe-core';return out;}};
    const r=site.refresh({field:unsupported,positions,radius:.3});assert.equal(r.derivativeUnavailable,true);assert.ok(r.gapJacobian.every(Number.isNaN));
    site.refresh({field:source,positions,radius:.3});
    assert.throws(()=>site.refresh({field:source,positions,radius:.3,consumeQuery(){throw Error('budget');}}),/budget/);
    assert.equal(site.row.derivativeUnavailable,true);assert.ok(site.row.forceColumn.every(Number.isNaN));
    for(const fraction of [-1,1.01,NaN])assert.throws(()=>create({fraction}),RangeError);
});

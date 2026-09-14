import test from 'node:test';
import assert from 'node:assert/strict';
import {BufferGeometry,Float32BufferAttribute} from 'three';
import {createSharedAxisContacts} from '../src/physics/kirchhoffSharedAxisContacts.js';
import {createSharedAxisNative,assembleSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {createSharedAxisVesselDiscovery} from '../src/physics/kirchhoffSharedAxisVesselWitnesses.js';

const close=(a,b,tol=1e-8)=>assert.ok(Math.abs(a-b)<=tol*Math.max(1,Math.abs(a),Math.abs(b)),`${a} vs ${b}`);
const sheath={start:[0,0,0],end:[70,0,0],innerRadius:2,proximalExtension:40};

test('a catheter tip near the 70 mm outlet is retained without a short artificial sheath edge',()=>{
    for(const insertion of [69.5,69.766,69.999999,70,70.2,70.499,70.5]) {
        const s=createSharedAxisNative({...createSharedAxisContacts({sheath}),minimumEdgeLength:.5,
            tools:[{id:'wire',insertion:100},{id:'catheter',insertion}]});
        assert.ok(s.coordinates.includes(insertion),'The physical material tip must not move');
        for(let i=1;i<s.coordinates.length;i++)assert.ok(s.coordinates[i]-s.coordinates[i-1]>=.5-1e-12,
            `${insertion}: artificial ${s.coordinates[i]-s.coordinates[i-1]} mm edge`);
        if(insertion!==70&&Math.abs(insertion-70)<.5)assert.equal(s.coordinates.includes(70),false);
        else assert.equal(s.coordinates.includes(70),true);
    }
});

test('interpolated sheath outlet has the exact six-coordinate radial gradient and Hessian',()=>{
    const start=[101,-37,20],axis=[2/7,3/7,6/7],end=start.map((v,k)=>v+70*axis[k]);
    const [sample]=createSharedAxisContacts({sheath:{...sheath,start,end},localCoordinates:true}).wallSamples;
    const a=axis.map((v,k)=>65*v+[.3,-.2,0][k]),b=axis.map((v,k)=>70.2*v+[.6,.2,-.3][k]);
    const input={a,b,radius:.8,coordinateA:65,coordinateB:70.2},t=5/5.2,base=sample(input);
    const point=a.map((v,k)=>(1-t)*v+t*b[k]),along=point.reduce((sum,v,k)=>sum+v*axis[k],0);
    const radial=point.map((v,k)=>v-along*axis[k]);close(base.gap,1.2-Math.hypot(...radial));
    const h=1e-5;
    for(let col=0;col<6;col++) {
        const shifted=sign=>{
            const next={...input,a:a.slice(),b:b.slice()};next[col<3?'a':'b'][col%3]+=sign*h;return sample(next);
        };
        const plus=shifted(1),minus=shifted(-1);
        close((plus.gap-minus.gap)/(2*h),base.jacobian[col],2e-7);
        for(let row=0;row<6;row++)close((plus.jacobian[row]-minus.jacobian[row])/(2*h),base.hessian[row*6+col],2e-7);
    }
    // Both endpoints move the clipped outlet point; retaining only B's normal
    // would apply the sheath reaction at the wrong place.
    assert.ok(base.jacobian.slice(0,3).some(v=>Math.abs(v)>1e-5));
});

test('vessel discovery queries only the exposed capsule and maps its witness back to full element coordinates',()=>{
    const geometry=new BufferGeometry().setAttribute('position',new Float32BufferAttribute([
        -1000,-1000,32,1000,-1000,32,0,1000,32
    ],3));
    const calls=[],field={fallbackGeometry:geometry,voxelSize:1,
        queryCapsuleSoA(x,y,z,r,index,out,...flags){
            calls.push({a:[x[0],y[0],z[0]],b:[x[1],y[1],z[1]],radius:r[0],length:flags[5]});
            return Object.assign(out,{signedGap:.1,signedDistance:.9,segmentT:.25,faceIndex:0});
        }};
    const sample=createSharedAxisVesselDiscovery(field,70),origin=[100,-20,30];
    const state={origin,definitions:[],layout:{positions:[0,4]}};
    const a=[65,.2,1.1],b=[70.2,.3,1.5],input={state,a,b,edge:0,radius:.8,coordinateA:65,coordinateB:70.2};
    sample(input);assert.equal(calls.length,1);
    const clippedT=5/5.2,fullT=clippedT+(1-clippedT)*.25;
    calls[0].a.forEach((v,k)=>close(v,origin[k]+(1-clippedT)*a[k]+clippedT*b[k]));
    assert.deepEqual(calls[0].b,b.map((v,k)=>v+origin[k]));close(calls[0].length,.2);
    const [row]=state.pendingVesselRows.values();close(row.witness.t,fullT);assert.deepEqual(row.dofs,[0,1,2,4,5,6]);
    const value=row.evaluate(input),worldZ=origin[2]+(1-fullT)*a[2]+fullT*b[2];
    close(value.gap,32-worldZ-.8);
    value.jacobian.forEach((v,k)=>close(v,k===2?-(1-fullT):k===5?-fullT:0));
    for(let col=0;col<6;col++) {
        const h=1e-5,shift=sign=>{
            const next={...input,a:a.slice(),b:b.slice()};next[col<3?'a':'b'][col%3]+=sign*h;return row.evaluate(next).gap;
        };
        close((shift(1)-shift(-1))/(2*h),value.jacobian[col],1e-7);
    }
    geometry.dispose();
});

test('sheath and vessel checks meet at the exact outlet even for arbitrarily short exposed portions',()=>{
    const calls=[],field={voxelSize:1,queryCapsuleSoA(x,y,z,r,index,out){
        calls.push({a:[x[0],y[0],z[0]],b:[x[1],y[1],z[1]]});
        return Object.assign(out,{signedDistance:10,signedGap:9,segmentT:.5,faceIndex:0});
    }};
    const [radial,vessel]=createSharedAxisContacts({sheath,contactField:field}).wallSamples;
    const state={origin:[0,0,0],definitions:[],layout:{positions:[0,4]}};
    for(const coordinateB of [69.766,70,70+1e-7,70.2,75]) {
        const input={state,a:[65,1.9,0],b:[coordinateB,1.9,0],edge:0,radius:.8,coordinateA:65,coordinateB};
        const before=calls.length;
        assert.ok(radial(input).gap<0,'The endpoint or interpolated outlet remains constrained inside the sheath');
        vessel(input);
        assert.equal(calls.length-before,Number(coordinateB>70));
        if(coordinateB>70){close(calls.at(-1).a[0],70);close(calls.at(-1).b[0],coordinateB);}
    }
    const outside={state,a:[70,3,0],b:[75,3,0],edge:0,radius:.8,coordinateA:70,coordinateB:75};
    assert.deepEqual(radial(outside).jacobian,[0,0,0,0,0,0]);
    vessel(outside);close(calls.at(-1).a[0],70);
});

test('outer material radius and complete exposed coverage survive omission of an outlet mesh node',()=>{
    for(const insertion of [69.766,70.2]) {
        const calls=[],field={voxelSize:1,queryCapsuleSoA(x,y,z,r,index,out){
            calls.push({a:x[0],b:x[1],radius:r[0]});
            return Object.assign(out,{signedDistance:10,signedGap:9,segmentT:.5,faceIndex:0});
        }};
        const s=createSharedAxisNative({...createSharedAxisContacts({sheath,contactField:field}),
            tools:[{id:'wire',insertion:100,radius:.4},{id:'catheter',insertion,radius:.8}]});
        assembleSharedAxisNative(s);
        close(calls[0].a,70);close(calls.at(-1).b,100);
        for(let i=1;i<calls.length;i++)close(calls[i].a,calls[i-1].b);
        calls.forEach(c=>close(c.radius,c.b<=insertion?.8:.4));
        assert.ok(calls.every(c=>c.a>=70-1e-12&&c.b>c.a));
    }
});

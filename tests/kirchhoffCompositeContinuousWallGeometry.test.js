import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import {BufferGeometry,Float32BufferAttribute,Vector3} from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {createContactResult,VesselContactField} from '../src/physics/collision/vesselContactField.js';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {createCompositeContinuousGeometry,evaluateCompositeContinuousGeometry} from '../src/physics/kirchhoffCompositeContinuousGeometry.js';
import {createCompositeContinuousWallPoint,queryCompositeContinuousWallPoint,measureCompositeContinuousWallMeshClearance} from '../src/physics/kirchhoffCompositeContinuousWallGeometry.js';

const close=(a,b,t=1e-7)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}`);
function field(vertices=[[0,0,0],[2,0,0],[0,2,0]]) {
    const mesh=new BufferGeometry();mesh.setAttribute('position',new Float32BufferAttribute(vertices.flat(),3));mesh.boundsTree=new MeshBVH(mesh);
    const point=new Vector3(),target={point:new Vector3(),distance:Infinity,faceIndex:-1},f={fallbackGeometry:mesh,calls:0};
    f.querySphere=(position,radius,out=createContactResult())=>{
        f.calls++;mesh.boundsTree.closestPointToPoint(point.fromArray(position),target);out.source='sparse-sdf-bvh';out.faceIndex=target.faceIndex;
        out.signedDistance=target.distance;out.signedGap=target.distance-radius;out.closestPoint.values.set(target.point.toArray());
        out.inward.values.set(position.map((v,k)=>(v-target.point.getComponent(k))/target.distance));return out;
    };
    return f;
}
function chart(positions,field) {
    const coordinates=[0,1,2,3],geometry=createCompositeContinuousGeometry({coordinates}).edges[1],
        layout=createCompositeChainLayout([['wire','catheter'],['wire','catheter'],['wire','catheter']],{positionSupports:[geometry.nodeIndices]}),
        modes=coordinates.map((_,node)=>({node,basis:[[0,1,0],[0,0,1],[1,0,0]],relativeDofs:[3*node,3*node+1,3*node+2]})),
        toolPositions=new Map([['wire',positions.map(p=>p.slice())],['catheter',positions.map(p=>p.map((v,k)=>v+(k===1?.2:0)))]]),
        sample=createCompositeContinuousWallPoint({layout,modes,geometry,owner:'wire',fraction:.37,radius:.2,field});
    return {geometry,layout,modes,toolPositions,sample};
}

test('actual selected triangle contact has an exact full four-node C2 common/relative pullback, force, moment and virtual work',()=>{
    const f=field(),c=chart([0,1,2,3].map(i=>[.5+.2*i,-.4+.025*Math.sin(i),.7+.04*Math.cos(i)]),f),
        first=queryCompositeContinuousWallPoint(c.sample,{toolPositions:c.toolPositions}),G=first.gapJacobian.slice(),B=first.normalForceColumn.slice(),H=first.normalDerivative.slice(),physical=first.physicalForceColumn.slice(),position=first.position.slice();
    assert.equal(first.supported,true,first.reason);assert.equal(first.wholeCurveCertified,false);assert.deepEqual(first.nodeIndices,[0,1,2,3]);assert.equal(f.calls,1);
    const size=G.length,h=1e-6;
    for(let j=0;j<size;j++) {
        const physicalCol=j<12?j:j-12,node=Math.floor(physicalCol/3),component=physicalCol%3,
            delta=j<12?[[1,0,0],[0,1,0],[0,0,1]][component]:c.modes[node].basis[component],p=c.toolPositions.get('wire')[node],original=p.slice();
        p.forEach((_,k)=>p[k]=original[k]+h*delta[k]);queryCompositeContinuousWallPoint(c.sample,{toolPositions:c.toolPositions});const a={gap:c.sample.gap,B:c.sample.normalForceColumn.slice()};
        p.forEach((_,k)=>p[k]=original[k]-h*delta[k]);queryCompositeContinuousWallPoint(c.sample,{toolPositions:c.toolPositions});const b={gap:c.sample.gap,B:c.sample.normalForceColumn.slice()};
        close((a.gap-b.gap)/(2*h),G[j],2e-9);for(let i=0;i<size;i++)close((a.B[i]-b.B[i])/(2*h),H[size*i+j],2e-9);
        p.forEach((_,k)=>p[k]=original[k]);
    }
    const force=[0,0,0],moment=[0,0,0];c.toolPositions.get('wire').forEach((p,node)=>{
        const v=Array.from(physical.slice(3*node,3*node+3));v.forEach((x,k)=>force[k]+=x);
        moment[0]+=p[1]*v[2]-p[2]*v[1];moment[1]+=p[2]*v[0]-p[0]*v[2];moment[2]+=p[0]*v[1]-p[1]*v[0];
    });
    close(moment[0],position[1]*force[2]-position[2]*force[1],1e-13);close(moment[1],position[2]*force[0]-position[0]*force[2],1e-13);close(moment[2],position[0]*force[1]-position[1]*force[0],1e-13);
    const velocity=Array.from({length:size},(_,i)=>.04*Math.cos(i)),materialVelocity=[0,0,0];
    for(let node=0;node<4;node++)for(let k=0;k<3;k++)materialVelocity[k]+=evaluateCompositeContinuousGeometry(c.geometry,{positions:c.toolPositions.get('wire'),fraction:.37}).basis.weights[node]*
        (velocity[3*node+k]+c.modes[node].basis.reduce((sum,b,a)=>sum+b[k]*velocity[12+3*node+a],0));
    close(B.reduce((sum,v,i)=>sum+v*velocity[i],0),force.reduce((sum,v,k)=>sum+v*materialVelocity[k],0),1e-14);
    queryCompositeContinuousWallPoint(c.sample,{toolPositions:c.toolPositions,order:'gradient'});assert.equal(c.sample.hessianValid,false);assert.ok(c.sample.normalDerivative.every(Number.isNaN));
    f.querySphere=()=>{throw Error('must not query changed provider');};assert.throws(()=>queryCompositeContinuousWallPoint(c.sample,{toolPositions:c.toolPositions}),/source changed/);
    f.fallbackGeometry.dispose();
});

test('whole actual triangle coverage catches penetration between clear nodes and never turns an exhausted bound into signed contact coverage',()=>{
    const f=field([[-10,-10,0],[10,-10,0],[0,10,0]]),coordinates=[0,1,2,3],geometry=createCompositeContinuousGeometry({coordinates}).edges[1],positions=coordinates.map(x=>[x,0,.05+.2*(x-1.5)**2]),radius=.075;
    assert.ok(positions.every(p=>f.querySphere(p,radius).signedGap>0));
    const hit=measureCompositeContinuousWallMeshClearance({field:f,geometry,positions,radius});assert.equal(hit.meshCertified,false);assert.equal(hit.status,'mesh-penetration');close(hit.witness.fraction,.5);assert.ok(hit.witness.gap<0);
    const clear=positions.map(p=>[p[0],p[1],p[2]+.4]),limited=measureCompositeContinuousWallMeshClearance({field:f,geometry,positions:clear,radius,maxQueries:1});
    assert.equal(limited.meshCertified,false);assert.equal(limited.status,'coverage-query-budget');
    const accepted=measureCompositeContinuousWallMeshClearance({field:f,geometry,positions:clear,radius});assert.equal(accepted.meshCertified,true);assert.equal(accepted.signedProviderCertified,false);
    assert.ok(accepted.lowerBound>=-1e-8);close(accepted.intervals.reduce((sum,v)=>sum+v.fractions[1]-v.fractions[0],0),1,0);
    f.fallbackGeometry.dispose();
});

test('real Aorta quantized source is queried at the actual C2 sample and retains distinct true G and unit-normal B',()=>{
    const bytes=fs.readFileSync(new URL('../res/Aorta_plain.collision.bin',import.meta.url)),field=new VesselContactField(decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength))),
        witness=[65.00287246704102,-462.00980948623305,-79.56869888305664],positions=[0,1,2,3].map(i=>witness.map((v,k)=>v+(i-1.37)*[.003,-.002,.001][k])),
        c=chart(positions,field),r=queryCompositeContinuousWallPoint(c.sample,{toolPositions:c.toolPositions});
    assert.equal(r.supported,true,r.reason);assert.equal(r.source,'sparse-sdf');assert.ok(r.gapJacobian.some((v,i)=>Math.abs(v-r.normalForceColumn[i])>1e-5));
    const G=r.gapJacobian.slice(),H=r.normalDerivative.slice(),p=c.toolPositions.get('wire')[0],original=p[0],h=1e-5;
    p[0]=original+h;queryCompositeContinuousWallPoint(c.sample,{toolPositions:c.toolPositions});const plus={g:c.sample.gap,B:c.sample.normalForceColumn.slice()};
    p[0]=original-h;queryCompositeContinuousWallPoint(c.sample,{toolPositions:c.toolPositions});const minus={g:c.sample.gap,B:c.sample.normalForceColumn.slice()};
    close((plus.g-minus.g)/(2*h),G[0],2e-8);for(let i=0;i<G.length;i++)close((plus.B[i]-minus.B[i])/(2*h),H[G.length*i],2e-8);
});

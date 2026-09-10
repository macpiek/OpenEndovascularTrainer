import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import {pathToFileURL} from 'node:url';
import {createCompositeWallBvhGeometryWorkspace,differentiateCompositeWallBvhContact,
    queryCompositeWallBvhPointGeometry,queryCompositeWallBvhCapsuleGeometry} from '../src/physics/kirchhoffCompositeWallBvhGeometry.js';
import {evaluateCompositeWallMixedRow,evaluateCompositeWallGapAugmented} from '../src/physics/kirchhoffCompositeWallGeometry.js';

const root=process.env.OET_WALL_BVH_PROVIDER_ROOT;
const url=path=>root?pathToFileURL(`${root}/${path}`):new URL(`../${path}`,import.meta.url);
const THREE=await import(url('node_modules/three/build/three.module.js'));
const {MeshBVH}=await import(url('node_modules/three-mesh-bvh/src/index.js'));
const {STLLoader}=await import(url('node_modules/three/examples/jsm/loaders/STLLoader.js'));
const {VesselContactField,createContactResult}=await import(url('src/physics/collision/vesselContactField.js'));
const {decodeCollisionAsset}=await import(url('src/physics/collision/collisionAssetFormat.js'));
const {transformAortaGeometry}=await import(url('src/aortaTransform.js'));
const {generateVessel}=await import(url('src/vesselGeometry.js'));
const close=(a,b,t=1e-7)=>assert.ok(Math.abs(a-b)<=t*Math.max(1,Math.abs(a),Math.abs(b)),`${a} != ${b}`);
const vectorClose=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const standard=[[0,0,0],[2,0,0],[0,2,0]];

function synthetic(triangles=[standard],sign=1) {
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(triangles.flat(2),3));
    geometry.boundsTree=new MeshBVH(geometry);
    const point=new THREE.Vector3(),target={point:new THREE.Vector3(),distance:Infinity,faceIndex:-1};
    const field={fallbackGeometry:geometry,pointCalls:0,capsuleCalls:0,bvhQueries:0};
    const sample=(p,radius,out)=>{
        field.bvhQueries++;geometry.boundsTree.closestPointToPoint(point.fromArray(p),target);
        out.signedDistance=sign*target.distance;out.signedGap=out.signedDistance-radius;
        out.inward.values.set(target.distance>1e-8?p.map((v,i)=>sign*(v-target.point.getComponent(i))/target.distance):[0,0,1]);
        out.closestPoint.values.set(target.point.toArray());out.faceIndex=target.faceIndex;out.source='sparse-sdf-bvh';return out;
    };
    field.querySphere=(p,radius,out=createContactResult())=>{field.pointCalls++;return sample(p,radius,out);};
    field.queryCapsuleCoordinates=(ax,ay,az,bx,by,bz,radius,out=createContactResult())=>{
        field.capsuleCalls++;const a=[ax,ay,az],b=[bx,by,bz],candidate=createContactResult();let gap=Infinity;
        for(const t of [0,.5,1]){
            sample(a.map((v,i)=>v+(b[i]-v)*t),radius,candidate);
            if(candidate.signedGap<gap){gap=candidate.signedGap;out.signedDistance=candidate.signedDistance;out.signedGap=gap;
                out.inward.values.set(candidate.inward.values);out.closestPoint.values.set(candidate.closestPoint.values);
                out.faceIndex=candidate.faceIndex;out.source=candidate.source;out.segmentT=t;}
        }
        out.capsuleSampleCount=2;return out;
    };
    return field;
}

for(const [feature,position] of [['face',[.4,.5,.7]],['edge',[.8,-.4,.6]],['vertex',[-.4,-.6,.8]]])
for(const sign of [1,-1])test(`selected triangle ${feature}, sign ${sign}: exact gap and normal derivatives`,()=>{
    const field=synthetic([standard],sign),radius=.3;
    const base=queryCompositeWallBvhPointGeometry({field,position,radius});
    assert.ok(base.supported,base.reason);assert.equal(base.featureType,feature);assert.equal(base.branchSign,sign);
    assert.equal(base.classicalFeature,true);assert.equal(base.globalWinnerCertified,false);
    assert.equal(field.pointCalls,1);assert.equal(field.bvhQueries,1);
    vectorClose(base.gapGradient,base.normalForceColumn,0);assert.equal(base.gradientNorm,1);
    if(feature==='face')assert.ok(base.gapHessian.every(v=>v===0));
    const eps=1e-5;
    for(let j=0;j<3;j++){
        const plus=queryCompositeWallBvhPointGeometry({field,position:position.map((v,i)=>v+(i===j?eps:0)),radius});
        const minus=queryCompositeWallBvhPointGeometry({field,position:position.map((v,i)=>v-(i===j?eps:0)),radius});
        assert.ok(plus.supported&&minus.supported);assert.equal(plus.branchSignature,base.branchSignature);assert.equal(minus.branchSignature,base.branchSignature);
        close(base.gapGradient[j],(plus.gap-minus.gap)/(2*eps),1e-8);
        for(let i=0;i<3;i++){
            close(base.gapHessian[3*i+j],(plus.gapGradient[i]-minus.gapGradient[i])/(2*eps),1e-8);
            close(base.normalForceJacobian[3*i+j],(plus.normal[i]-minus.normal[i])/(2*eps),1e-8);
            close(base.gapHessian[3*i+j],base.gapHessian[3*j+i],1e-12);
        }
    }
    field.fallbackGeometry.dispose();
});

test('a selected contact is differentiated without a second provider or BVH query',()=>{
    const field=synthetic(),position=[.8,-.4,.6],radius=.3,contact=field.querySphere(position,radius);
    field.querySphere=()=>{throw new Error('duplicate provider query');};
    field.fallbackGeometry.boundsTree.closestPointToPoint=()=>{throw new Error('duplicate BVH query');};
    const w=createCompositeWallBvhGeometryWorkspace(1),g=differentiateCompositeWallBvhContact({field,contact,positions:[position],radius},w);
    assert.ok(g.supported,g.reason);assert.equal(field.bvhQueries,1);assert.equal(g.providerFaceIndex,contact.faceIndex);
    vectorClose(g.closestPoint,contact.closestPoint.values,0);vectorClose(g.normal,contact.inward.values,0);assert.equal(g.gap,contact.signedGap);
    field.fallbackGeometry.dispose();
});

test('the capsule endpoint pullback preserves a stable interior sample and its physical wrench',()=>{
    const field=synthetic(),positions=[[.8,-.7,.3],[1,-.3,.7]],radius=.3;
    const base=queryCompositeWallBvhCapsuleGeometry({field,positions,radius});assert.ok(base.supported,base.reason);
    assert.equal(base.featureType,'edge');assert.equal(base.sampleFraction,.5);assert.equal(base.sampleCount,2);assert.equal(field.bvhQueries,3);
    const eps=1e-5;
    for(let j=0;j<6;j++){
        const move=d=>positions.map((p,n)=>p.map((v,a)=>v+(3*n+a===j?d:0)));
        const plus=queryCompositeWallBvhCapsuleGeometry({field,positions:move(eps),radius}),minus=queryCompositeWallBvhCapsuleGeometry({field,positions:move(-eps),radius});
        assert.ok(plus.supported&&minus.supported);assert.equal(plus.branchSignature,base.branchSignature);assert.equal(minus.branchSignature,base.branchSignature);
        close(base.gapGradient[j],(plus.gap-minus.gap)/(2*eps),1e-8);
        for(let i=0;i<6;i++)close(base.normalForceJacobian[6*i+j],(plus.normalForceColumn[i]-minus.normalForceColumn[i])/(2*eps),1e-8);
    }
    const Fn=3,r=evaluateCompositeWallMixedRow({geometry:base,normalForce:Fn,penalty:1});
    const force=[0,1,2].map(i=>-r.mechanicalGradient[i]-r.mechanicalGradient[3+i]);vectorClose(force,Array.from(base.normal,v=>Fn*v),1e-12);
    const torque=[0,0,0];for(let k=0;k<2;k++){
        const p=positions[k],f=[0,1,2].map(i=>-r.mechanicalGradient[3*k+i]);
        torque[0]+=p[1]*f[2]-p[2]*f[1];torque[1]+=p[2]*f[0]-p[0]*f[2];torque[2]+=p[0]*f[1]-p[1]*f[0];
    }
    const p=base.point;vectorClose(torque,[p[1]*force[2]-p[2]*force[1],p[2]*force[0]-p[0]*force[2],p[0]*force[1]-p[1]*force[0]],1e-12);
    field.fallbackGeometry.dispose();
});

test('strict shared-edge projection is distinct from an ambiguous crease transition',()=>{
    const field=synthetic([standard,[[0,0,0],[2,0,0],[0,0,2]]]),radius=.2;
    const g=queryCompositeWallBvhPointGeometry({field,position:[.8,-.4,-.6],radius,localFaceIndices:[0,1]});
    assert.ok(g.supported,g.reason);assert.equal(g.featureType,'edge');assert.equal(g.interfaceKind,null);
    const boundary=queryCompositeWallBvhPointGeometry({field,position:[.8,0,-.6],radius,localFaceIndices:[0,1]});
    assert.equal(boundary.supported,false);assert.equal(boundary.interfaceKind,'crease-feature-interface');
    field.fallbackGeometry.dispose();
    const vertexField=synthetic([standard,[[0,0,0],[-2,0,0],[0,0,2]]]);
    const vertexBoundary=queryCompositeWallBvhPointGeometry({field:vertexField,position:[0,-.4,-.6],radius,localFaceIndices:[0,1]});
    assert.equal(vertexBoundary.supported,false);assert.equal(vertexBoundary.interfaceKind,'vertex-feature-interface');
    vertexField.fallbackGeometry.dispose();
});

test('a soft coplanar seam is explicitly nonclassical for the individual triangle feature',()=>{
    const field=synthetic([standard,[[0,0,0],[2,0,0],[1,-2,0]]]),position=[.8,0,.6],radius=.2;
    const g=queryCompositeWallBvhPointGeometry({field,position,radius,localFaceIndices:[0,1]});
    assert.equal(g.supported,false);assert.equal(g.interfaceKind,'soft-coplanar-interface');
    assert.equal(g.reason,'nonclassical-feature-boundary');
    // The UNION can still be smooth: that requires a distinct union-feature
    // proof, not pretending the selected triangle's edge Hessian is classical.
    const plus=field.querySphere([.8,1e-5,.6],radius),minus=field.querySphere([.8,-1e-5,.6],radius);
    close(plus.signedGap,minus.signedGap,1e-12);vectorClose(plus.inward.values,minus.inward.values,1e-12);
    field.fallbackGeometry.dispose();
});

test('surface intersections and retained zero-distance normals are unsupported',()=>{
    const field=synthetic([standard,[[.5,-1,-1],[.5,2,-1],[.5,.5,2]]]);
    const g=queryCompositeWallBvhPointGeometry({field,position:[.5,.5,0],radius:.2,localFaceIndices:[0,1]});
    assert.equal(g.supported,false);assert.equal(g.interfaceKind,'surface-intersection');
    assert.equal(g.reason,'zero-distance-or-retained-provider-normal');
    assert.throws(()=>evaluateCompositeWallMixedRow({geometry:g,normalForce:1,penalty:1}),/supported/);
    field.fallbackGeometry.dispose();
});

test('unsupported sources, ambiguous predicates, invalid indices and unproven normal data cannot be accepted',()=>{
    const field=synthetic(),point=[.8,-.4,.6],radius=.2,contact=field.querySphere(point,radius),w=createCompositeWallBvhGeometryWorkspace(1);
    const evaluate=()=>differentiateCompositeWallBvhContact({field,contact,positions:[point],radius},w);
    contact.source='sparse-sdf';assert.equal(evaluate().reason,'unsupported-source:sparse-sdf');contact.source='sparse-sdf-bvh';
    contact.inward.values.set([1,0,0]);assert.equal(evaluate().reason,'normal-does-not-match-signed-feature-distance');
    field.querySphere(point,radius,contact);contact.faceIndex=100;assert.throws(evaluate,/triangle index/);
    field.querySphere(point,radius,contact);const original=field.fallbackGeometry.boundsTree;
    field.fallbackGeometry.boundsTree={geometry:field.fallbackGeometry,indirect:true};assert.equal(evaluate().reason,'unsupported-indirect-face-index-contract');
    field.fallbackGeometry.boundsTree=original;
    const boundary=queryCompositeWallBvhPointGeometry({field,position:[0,-.4,.6],radius});
    assert.equal(boundary.supported,false);assert.equal(boundary.interfaceKind,'unresolved-feature-interface');
    field.fallbackGeometry.dispose();
    const distant=synthetic([[[1e8,0,0],[1e8+32,0,0],[1e8,32,0]]]),position=[1e8+8,8,1e-7];
    const uncertain=distant.querySphere(position,radius);uncertain.inward.values.set([1,0,0]);
    const unresolved=differentiateCompositeWallBvhContact({field:distant,contact:uncertain,positions:[position],radius},createCompositeWallBvhGeometryWorkspace(1));
    assert.equal(unresolved.supported,false);assert.equal(unresolved.reason,'unresolved-distance-direction');
    distant.fallbackGeometry.dispose();
});

test('an unresolved incident triangle cannot silently certify a local feature',()=>{
    const field=synthetic([standard,[[0,0,0],[2,0,0],[1,0,0]]]),position=[.8,-.4,.6],radius=.2;
    const contact=field.querySphere(position,radius);assert.equal(contact.faceIndex,0);
    const g=differentiateCompositeWallBvhContact({field,contact,positions:[position],radius,localFaceIndices:[0,1]},createCompositeWallBvhGeometryWorkspace(1));
    assert.equal(g.supported,false);assert.equal(g.reason,'unresolved-local-face-predicates');
    field.fallbackGeometry.dispose();
});

test('feature derivatives are covariant under rotation, translation and uniform scale',()=>{
    // Exact integer coordinates after this scaled orthogonal map survive the
    // Float32 geometry storage; the oracle still uses fresh raw BVH results.
    const rotate=p=>[(p[0]-2*p[1]+2*p[2])/3,(2*p[0]+2*p[1]+p[2])/3,(-2*p[0]+p[1]+2*p[2])/3];
    const transform=p=>rotate(p).map((v,i)=>3*v+[7,-11,5][i]);
    const field=synthetic([standard.map(transform)],-1),radius=.4,eps=1e-5;
    for(const [feature,p] of [['face',[.4,.5,.7]],['edge',[.8,-.4,.6]],['vertex',[-.4,-.6,.8]]]){
        const position=transform(p),g=queryCompositeWallBvhPointGeometry({field,position,radius});
        assert.ok(g.supported,g.reason);assert.equal(g.featureType,feature);
        for(let j=0;j<3;j++){
            const plus=field.querySphere(position.map((v,i)=>v+(i===j?eps:0)),radius),minus=field.querySphere(position.map((v,i)=>v-(i===j?eps:0)),radius);
            close(g.gapGradient[j],(plus.signedGap-minus.signedGap)/(2*eps),1e-8);
            for(let i=0;i<3;i++)close(g.normalForceJacobian[3*i+j],(plus.inward.values[i]-minus.inward.values[i])/(2*eps),1e-8);
        }
    }
    field.fallbackGeometry.dispose();
});

let actual;
function realAnatomy() {
    if(actual)return actual;
    const buffer=name=>{const bytes=fs.readFileSync(url(`res/${name}`));return bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength);};
    const asset=decodeCollisionAsset(buffer('Aorta_plain.collision.bin')),geometry=new STLLoader().parse(buffer('Aorta_plain.stl'));
    const transform=transformAortaGeometry(geometry,generateVessel(140,0).vessel);
    close(transform.scale,asset.metadata.transform.scale,1e-12);vectorClose(transform.targetCenter,asset.metadata.transform.targetCenter,1e-12);
    geometry.boundsTree=new MeshBVH(geometry);
    const field=new VesselContactField(asset,{fallbackGeometry:geometry,bvhValidationDistance:.02,capsuleBvhValidation:-.1});
    actual={field,geometry};return actual;
}

for(const [feature,shift] of [['face',1],['edge',-4]])test(`actual anatomy with application BVH settings: stable capsule ${feature}`,t=>{
    const {field}=realAnatomy(),positions=[[65.00287246704102,-462.2578430175781+shift,-79.56869888305664],[64.95548751831055,-462.0397204589844+shift,-79.65612350463867]],radius=.4445;
    const raw=field.queryCapsuleCoordinates(...positions[0],...positions[1],radius,createContactResult());
    const g=queryCompositeWallBvhCapsuleGeometry({field,positions,radius});assert.ok(g.supported,g.reason);assert.equal(g.featureType,feature);
    assert.equal(g.source,'sparse-sdf-bvh');assert.equal(g.gap,raw.signedGap);assert.equal(g.sampleFraction,raw.segmentT);assert.equal(g.providerFaceIndex,raw.faceIndex);
    vectorClose(g.normal,raw.inward.values,0);vectorClose(g.closestPoint,raw.closestPoint.values,0);
    const eps=1e-4;let maxGapDerivativeError=0,maxNormalDerivativeError=0;
    for(let j=0;j<6;j++){
        const move=d=>positions.map((p,n)=>p.map((v,a)=>v+(3*n+a===j?d:0)));
        const plus=queryCompositeWallBvhCapsuleGeometry({field,positions:move(eps),radius}),minus=queryCompositeWallBvhCapsuleGeometry({field,positions:move(-eps),radius});
        assert.ok(plus.supported,plus.reason);assert.ok(minus.supported,minus.reason);
        assert.equal(plus.branchSignature,g.branchSignature);assert.equal(minus.branchSignature,g.branchSignature);
        const gapFD=(plus.gap-minus.gap)/(2*eps);close(g.gapGradient[j],gapFD,1e-6);
        maxGapDerivativeError=Math.max(maxGapDerivativeError,Math.abs(g.gapGradient[j]-gapFD));
        for(let i=0;i<6;i++){
            const normalFD=(plus.normalForceColumn[i]-minus.normalForceColumn[i])/(2*eps);
            close(g.normalForceJacobian[6*i+j],normalFD,1e-6);
            maxNormalDerivativeError=Math.max(maxNormalDerivativeError,Math.abs(g.normalForceJacobian[6*i+j]-normalFD));
        }
    }
    assert.equal(field.bvhValidationDistance,.02);assert.equal(field.capsuleBvhValidationGap,-.1);
    t.diagnostic(JSON.stringify({feature,faceIndex:g.providerFaceIndex,gap:g.gap,sampleFraction:g.sampleFraction,eps,maxGapDerivativeError,maxNormalDerivativeError}));
});

test('BVH physical mixed and gap-conjugate energy derivatives coincide on a proven unit-gradient branch',()=>{
    const field=synthetic(),position=[.8,-.4,.6],radius=.8,Fn=2,penalty=7;
    const evaluate=p=>{
        const geometry=queryCompositeWallBvhPointGeometry({field,position:p,radius});assert.ok(geometry.supported,geometry.reason);
        return {geometry,mixed:evaluateCompositeWallMixedRow({geometry,normalForce:Fn,penalty}),al:evaluateCompositeWallGapAugmented({geometry,gapMultiplier:Fn,penalty,mode:'gap-potential'})};
    };
    const base=evaluate(position),eps=1e-5;assert.equal(base.al.effectiveNormalForce,Fn);
    for(let j=0;j<3;j++){
        const plus=evaluate(position.map((v,i)=>v+(i===j?eps:0))),minus=evaluate(position.map((v,i)=>v-(i===j?eps:0)));
        close(base.al.gradient[j],(plus.al.energy-minus.al.energy)/(2*eps),1e-8);
        close(base.mixed.ncpGeometryRow[j],(plus.mixed.ncpResidual-minus.mixed.ncpResidual)/(2*eps),1e-8);
        for(let i=0;i<3;i++){
            close(base.al.hessian[3*i+j],(plus.al.gradient[i]-minus.al.gradient[i])/(2*eps),1e-8);
            close(base.mixed.mechanicalJacobian[3*i+j],(plus.mixed.mechanicalGradient[i]-minus.mixed.mechanicalGradient[i])/(2*eps),1e-8);
        }
    }
    field.fallbackGeometry.dispose();
});

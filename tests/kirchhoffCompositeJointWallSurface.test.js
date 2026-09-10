import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';
import { captureCompositeReferenceFrames } from '../src/physics/kirchhoffCompositeElement.js';
import { createCompositeChainLayout } from '../src/physics/kirchhoffCompositeChain.js';
import { createCompositeWallWorkspace, refreshCompositeWallContacts } from '../src/physics/kirchhoffCompositeWallContacts.js';
import { createCompositeWallEnvelopeWorkspace, refreshCompositeWallEnvelope } from '../src/physics/kirchhoffCompositeWallEnvelope.js';
import { createContactResult, VesselContactField } from '../src/physics/collision/vesselContactField.js';
import { decodeCollisionAsset } from '../src/physics/collision/collisionAssetFormat.js';
import { evaluateCompositeJointSurfaceMotion } from '../src/physics/kirchhoffCompositeJointSurfaceMotion.js';
import { createCompositeJointSurfacePullback, pullbackCompositeJointSurface, evaluateCompositeJointSurfaceLoads } from '../src/physics/kirchhoffCompositeJointSurfacePullback.js';
import { createCompositeJointWallSurfaceWorkspace, evaluateCompositeJointWallSurface } from '../src/physics/kirchhoffCompositeJointWallSurface.js';
import { createCompositeDiscreteWallPoint } from '../src/physics/compositeDiscreteWallPoint.js';
const dot=(a,b)=>a.reduce((s,x,k)=>s+x*b[k],0),norm=a=>Math.hypot(...a);
const add=(a,b)=>a.map((x,k)=>x+b[k]),sub=(a,b)=>a.map((x,k)=>x-b[k]),scale=(a,s)=>Array.from(a,x=>x*s);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const close=(a,b,t=3e-10)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b} (tol ${t})`);
const same=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((x,k)=>close(x,b[k],t));};
const layout=createCompositeChainLayout([['wire'],['wire']]);
function planeField(){return {calls:0,queryCapsuleCoordinates(ax,ay,az,bx,by,bz,r,out=createContactResult()){
    this.calls++;const t=ay<by?0:1,p=[ax+(bx-ax)*t,ay+(by-ay)*t,az+(bz-az)*t];
    out.signedDistance=p[1];out.signedGap=p[1]-r;out.segmentT=t;out.capsuleSampleCount=2;out.inward.values.set([0,1,0]);
    out.closestPoint.values.set([p[0],0,p[2]]);out.source='analytic-plane';out.faceIndex=0;return out;
}};}
const valley=([x,y,z])=>2+.5*(x-.125)*(y-.125)+.25*z,gradient=([x,y])=>[.5*(y-.125),.5*(x-.125),.25];
function sdfField(){
    const f={sdfOrigin:[-1,-1,-1],sdfDimensions:[3,3,3],brickSize:2,voxelSize:.5,sdfQuantization:1/1024,sdfBrickLookup:new Uint16Array(27),sdfDistances:new Uint32Array(216),calls:0};
    for(let bz=0;bz<3;bz++)for(let by=0;by<3;by++)for(let bx=0;bx<3;bx++){
        const brick=bx+3*(by+3*bz);f.sdfBrickLookup[brick]=brick;
        for(let z=0;z<2;z++)for(let y=0;y<2;y++)for(let x=0;x<2;x++)f.sdfDistances[brick*8+x+2*(y+2*z)]=valley([-1+.5*(2*bx+x),-1+.5*(2*by+y),-1+.5*(2*bz+z)])*1024;
    }
    f.queryCapsuleCoordinates=function(ax,ay,az,bx,by,bz,r,out=createContactResult()){
        this.calls++;const a=[ax,ay,az],b=[bx,by,bz];let best=Infinity,t=0;
        for(const s of [0,.5,1]){const g=valley(a.map((x,k)=>x+s*(b[k]-x)));if(g<best){best=g;t=s;}}
        const p=a.map((x,k)=>x+t*(b[k]-x)),n=scale(gradient(p),1/norm(gradient(p)));
        out.signedDistance=best;out.signedGap=best-r;out.segmentT=t;out.capsuleSampleCount=2;out.inward.values.set(n);out.closestPoint.values.set(sub(p,scale(n,best)));out.source='sparse-sdf';out.faceIndex=-1;return out;
    };return f;
}
function bvhField(){
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute([0,0,0,2,0,0,0,2,0],3));geometry.boundsTree=new MeshBVH(geometry);
    const f={fallbackGeometry:geometry,calls:0,bvhQueries:0},p=new THREE.Vector3(),hit={point:new THREE.Vector3(),distance:Infinity,faceIndex:-1};
    f.queryCapsuleCoordinates=function(ax,ay,az,bx,by,bz,r,out=createContactResult()){
        this.calls++;const a=[ax,ay,az],b=[bx,by,bz];let best=Infinity;
        for(const s of [0,.5,1]){const q=a.map((x,k)=>x+s*(b[k]-x));this.bvhQueries++;geometry.boundsTree.closestPointToPoint(p.fromArray(q),hit);
            if(hit.distance<best){best=hit.distance;out.signedDistance=best;out.signedGap=best-r;out.segmentT=s;out.closestPoint.values.set(hit.point.toArray());out.inward.values.set(scale(sub(q,hit.point.toArray()),1/best));out.faceIndex=hit.faceIndex;}}
        out.capsuleSampleCount=2;out.source='sparse-sdf-bvh';return out;
    };return f;
}
function refresh(input){
    const {field,collector,envelope}=input.current,positions=[...input.tool.positions,[9,9,9]],contactOwners={edges:[{edge:0,wall:{owner:'wire',radius:input.tool.radius}},{edge:1,wall:null}]};
    if(envelope)refreshCompositeWallEnvelope({positions,contactOwners,field},collector);else refreshCompositeWallContacts({positions,contactOwners,field},collector);
    input.current.row=envelope?collector.rows.find(r=>r.edge===0&&r.role==='proximal'):collector.rows[0];input.current.positions=structuredClone(input.tool.positions);
    return input;
}
function fixture(kind='plane',dt=.02){
    const positions=kind==='sdf'?[[-.625,-.625,.125],[.875,.875,.125]]:kind==='bvh'?[[.8,-.7,.3],[1,-.3,.7]]:[[-.8,.4,.1],[1.2,.45,.2]];
    const previousPositions=[sub(positions[0],[.01,.005,-.003]),sub(positions[1],[-.008,.003,.007])],field=kind==='sdf'?sdfField():kind==='bvh'?bvhField():planeField();
    const tool={id:'wire',edgeId:'wire:edge0',edge:0,radius:kind==='sdf'?2.2:.3,coordinates:[10,13],positions:structuredClone(positions),previousPositions,
        reference:captureCompositeReferenceFrames(previousPositions)[0],previousAngle:.2,angle:.4,trace:'right',
        materialMap:{sStart:20,dsDx:1.3,dsDt:.4},materialPath:{kind:'linear-affine-maps',previousEdgeId:'wire:edge0',previousMap:{sStart:20-dt*.4,dsDx:1.3}},
        positionRates:[[.2,-.1,.07],[-.15,.05,.2]],angleRate:2};
    return refresh({dt,tool,current:{field,collector:createCompositeWallWorkspace(layout),plane:kind==='plane'?{normal:[0,1,0],offset:0}:undefined},wall:{motion:'stationary-material',source:kind==='plane'?'analytic-plane':kind==='sdf'?'sparse-sdf':'sparse-sdf-bvh'}});
}
function copyInput(input){return {...input,tool:structuredClone(input.tool),current:{...input.current,positions:structuredClone(input.current.positions),row:structuredClone(input.current.row)},wall:{...input.wall}};}
function perturb(input,j,h){const x=copyInput(input);if(j===6)x.tool.angle+=h;else x.tool.positions[Math.floor(j/3)][j%3]+=h;return refresh(x);}
function instant(input,out){
    const tool={...input.tool,coordinate:input.tool.coordinates[0]+out.fraction*(input.tool.coordinates[1]-input.tool.coordinates[0])};
    return evaluateCompositeJointSurfaceMotion({tools:[tool],dt:input.dt,rateMode:'instantaneous',wall:{velocity:[0,0,0]},contact:{point:out.point,axes:out.physicalForce.axes}});
}

for(const kind of ['plane','sdf','bvh'])test(`fixed interior material point retains full tangential BE derivatives: ${kind}`,()=>{
    const input=fixture(kind),fraction=.37;
    input.wall.rateMode='backward-euler-grid';
    const sample=createCompositeDiscreteWallPoint({fraction});
    function pointRefresh(x){
        x.current.row=sample.refresh({field:x.current.field,positions:x.tool.positions,radius:x.tool.radius});
        x.current.row.owner='wire';
        x.current.positions=structuredClone(x.tool.positions);
        return x;
    }
    pointRefresh(input);
    const base=structuredClone(evaluateCompositeJointWallSurface(input,createCompositeJointWallSurfaceWorkspace()));
    assert.equal(base.fraction,fraction);
    assert.notEqual(input.current.row.rawContact.segmentT,fraction);
    const h=1e-6;
    for(let j=0;j<7;j++){
        const values=[];
        for(const sign of [1,-1]){
            const x=copyInput(input);
            if(j===6)x.tool.angle+=sign*h;else x.tool.positions[Math.floor(j/3)][j%3]+=sign*h;
            values.push(structuredClone(evaluateCompositeJointWallSurface(pointRefresh(x),createCompositeJointWallSurfaceWorkspace())));
        }
        for(let c=0;c<2;c++)close((values[0].increment[c]-values[1].increment[c])/(2*h),base.slipJacobian[c*7+j],3e-7);
        for(let i=0;i<14;i++)close((values[0].forceMap[i]-values[1].forceMap[i])/(2*h),base.DforceMap[i*7+j],3e-7);
    }
    if(kind==='bvh')input.current.field.fallbackGeometry.dispose();
});

for(const kind of ['plane','sdf','bvh'])test(`native implicit surface rate preserves full query-chain B/DB and slip derivative: ${kind}`,()=>{
    const input=fixture(kind);input.wall.rateMode='backward-euler-grid';
    // This label crosses its old native edge boundary; the declared local
    // one-sided rate remains defined without a finite hinge displacement.
    input.tool.materialMap.dsDt=-12;const out=evaluateCompositeJointWallSurface(input,createCompositeJointWallSurfaceWorkspace()),h=1e-6;
    assert.equal(out.motion.finiteStepSlipKnown,false);assert.equal(out.motion.includesHingeTransport,false);
    const B=Array.from(out.forceMap),DB=Array.from(out.DforceMap),G=Array.from(out.slipJacobian);
    for(let j=0;j<7;j++) {
        const plus=evaluateCompositeJointWallSurface(perturb(input,j,h),createCompositeJointWallSurfaceWorkspace()),minus=evaluateCompositeJointWallSurface(perturb(input,j,-h),createCompositeJointWallSurfaceWorkspace());
        for(let c=0;c<2;c++)close((plus.increment[c]-minus.increment[c])/(2*h),G[c*7+j],3e-7);
        for(let i=0;i<14;i++)close((plus.forceMap[i]-minus.forceMap[i])/(2*h),DB[i*7+j],3e-7);
    }
    // Virtual forces retain the same complete independent spin and lever-arm
    // map as the existing physical operator; only the declared rate law differs.
    same(B,Array.from(out.physicalForce.forceMap),1e-12);
    if(kind==='bvh')input.current.field.fallbackGeometry.dispose();
});
for(const kind of ['plane','sdf','bvh'])test(`${kind}: original selected row is composed without another query; all seven G/DB columns match re-detection FD`,context=>{
    const input=fixture(kind),ws=createCompositeJointWallSurfaceWorkspace(),calls=input.current.field.calls,base=structuredClone(evaluateCompositeJointWallSurface(input,ws));
    assert.equal(input.current.field.calls,calls);assert.equal(base.queryCount,0);assert.equal(base.operatorReady,true);same(base.forceMap,instant(input,base).forceMap,4e-14);
    assert.equal(base.identity.fraction,kind==='plane'?0:.5);
    let maximumG=0,maximumDB=0;
    for(let j=0;j<7;j++){
        const h=1e-6,a=structuredClone(evaluateCompositeJointWallSurface(perturb(input,j,h),ws)),b=evaluateCompositeJointWallSurface(perturb(input,j,-h),ws);
        for(let r=0;r<2;r++){const fd=(a.increment[r]-b.increment[r])/(2*h);maximumG=Math.max(maximumG,Math.abs(fd-base.slipJacobian[r*7+j]));close(fd,base.slipJacobian[r*7+j],3e-8);}
        for(let r=0;r<14;r++){const fd=(a.forceMap[r]-b.forceMap[r])/(2*h);maximumDB=Math.max(maximumDB,Math.abs(fd-base.DforceMap[r*7+j]));close(fd,base.DforceMap[r*7+j],3e-8);}
    }
    refresh(input);input.current.field.queryCapsuleCoordinates=()=>{throw Error('duplicate query');};
    if(kind==='bvh')input.current.field.fallbackGeometry.boundsTree.closestPointToPoint=()=>{throw Error('duplicate BVH query');};
    assert.ok(evaluateCompositeJointWallSurface(input,ws).supported);input.current.field.fallbackGeometry?.dispose();
    context.diagnostic(JSON.stringify({kind,maximumG,maximumDB,queryCount:base.queryCount}));
});

test('the SAME current stationary wall material point keeps tangential sliding instead of cancelling it with an old nearest point',()=>{
    const input=fixture();input.tool.positions=[[-.7,.3,.4],[1.3,.3,.4]];input.tool.previousPositions=[[-1,.3,0],[1,.3,0]];
    input.tool.reference=captureCompositeReferenceFrames(input.tool.previousPositions)[0];input.tool.angle=input.tool.previousAngle;input.tool.materialMap.dsDt=0;input.tool.materialPath.previousMap.sStart=20;input.tool.materialPath.previousTrace='left';input.tool.trace='left';refresh(input);
    const out=evaluateCompositeJointWallSurface(input);same(out.increment,[.3,.4],2e-14);assert.equal(out.previousNearestQueryUsed,false);
    same(out.point,[1.3,0,.4],0);assert.ok(norm(out.increment)>.49);
});

test('own feed, nonunit metric, affine label-rate map and unwrapped spin are retained independently against the stationary wall',()=>{
    const input=fixture();input.tool.positions=[[-1,.3,0],[1,.3,0]];input.tool.previousPositions=structuredClone(input.tool.positions);input.tool.reference=captureCompositeReferenceFrames(input.tool.positions)[0];
    // Tie selects the distal trace. Negative dsDt traces a material label back inside this same old edge.
    input.tool.trace='left';input.tool.materialMap.dsDt=-.65;input.tool.materialPath.previousMap.sStart=20-input.dt*-.65;input.tool.angle=input.tool.previousAngle+2*Math.PI;refresh(input);
    const out=evaluateCompositeJointWallSurface(input);same(out.increment,[input.dt/3,-.3*2*Math.PI],3e-14);
    input.tool.angle=input.tool.previousAngle-2*Math.PI;same(evaluateCompositeJointWallSurface(input).increment,[input.dt/3,.3*2*Math.PI],3e-14);
    input.tool.materialPath.previousMap.dsDx=1.301;input.tool.materialMap.dsDtEnds=[0,3].map(x=>((20-input.tool.materialPath.previousMap.sStart)+(1.3-1.301)*x)/input.dt);delete input.tool.materialMap.dsDt;
    const result=evaluateCompositeJointWallSurface(input);assert.ok(result.incrementValid);assert.ok(Number.isFinite(result.motion.tools[0].previousFraction));
});

test('finite stationary-wall rule tends to actual instantaneous virtual power for simultaneous feed, bending and spin',context=>{
    const errors=[],powers=[],traction=[.7,-.4];
    for(const dt of [.001,.0001,.00001]){
        const input=fixture('sdf',dt),t=input.tool;t.positions=t.previousPositions.map((q,e)=>add(q,scale(t.positionRates[e],dt)));t.angle=t.previousAngle+t.angleRate*dt;refresh(input);
        const out=structuredClone(evaluateCompositeJointWallSurface(input)),old=copyInput(input);old.tool.positions=structuredClone(t.previousPositions);old.tool.angle=t.previousAngle;refresh(old);
        const base=evaluateCompositeJointWallSurface(old),rate=instant(old,base),v=Array.from(out.increment,x=>x/dt),rates=t.positionRates.flat().concat(t.angleRate);
        const load=rates.map((_,j)=>dot(Array.from(base.forceMap.slice(j*2,j*2+2)),traction));
        errors.push(norm(sub(v,Array.from(rate.slipRate))));powers.push(Math.abs(dot(traction,v)-dot(load,rates)-dot(traction,rate.prescribedSlipRate)));
    }
    assert.ok(errors[1]<.11*errors[0]&&errors[2]<.11*errors[1],JSON.stringify(errors));assert.ok(errors[2]<1e-5,JSON.stringify(errors));
    assert.ok(powers[1]<.11*powers[0]&&powers[2]<.11*powers[1],JSON.stringify(powers));
    context.diagnostic(JSON.stringify({dt:[.001,.0001,.00001],rateErrors:errors,powerErrors:powers}));
});

test('physical B includes bending endpoint couples and own spin torque with exact virtual work and stationary-wall wrench',()=>{
    const input=fixture('bvh'),out=evaluateCompositeJointWallSurface(input),rate=instant(input,out),traction=[.7,-.4],rates=input.tool.positionRates.flat().concat(input.tool.angleRate);
    const load=rates.map((_,j)=>dot(Array.from(out.forceMap.slice(2*j,2*j+2)),traction));close(dot(load,rates)+dot(traction,rate.prescribedSlipRate),dot(traction,rate.slipRate),3e-14);
    const world=add(scale(out.physicalForce.axes[0],traction[0]),scale(out.physicalForce.axes[1],traction[1])),data=out.physicalForce.tools[0],tangent=Array.from(data.tangent);
    same(add(load.slice(0,3),load.slice(3,6)),world,3e-14);
    const connection=Array.from({length:6},(_,j)=>dot(tangent,[data.omegaMap[j],data.omegaMap[7+j],data.omegaMap[14+j]]));
    const physical=load.slice(0,6).map((x,j)=>x-load[6]*connection[j]);
    const moment=add(add(cross(input.tool.positions[0],physical.slice(0,3)),cross(input.tool.positions[1],physical.slice(3,6))),scale(tangent,load[6]));
    same(moment,cross(Array.from(out.point),world),3e-14);assert.ok(Math.abs(load[6])>.01);
    // Original normal Fn has the same wrench whether distributed at its axis
    // foot or this wall point, because their displacement is normal-parallel.
    const normalForce=scale(Array.from(out.normal),.9);same(cross(Array.from(out.point),normalForce),cross(Array.from(out.center),normalForce),3e-14);
    const mapping=createCompositeJointSurfacePullback({layout,modes:[],tools:out.tools});pullbackCompositeJointSurface(out,mapping);assert.ok(mapping.operatorReady);same(evaluateCompositeJointSurfaceLoads(traction,mapping).physical,load,0);
    input.current.field.fallbackGeometry.dispose();
});

test('explicit own-reference basis resolves normal incidence, differentiates its PT, and preserves isotropic power under gauge change',()=>{
    const input=fixture();input.tool.positions=[[0,.4,0],[0,2.4,0]];input.tool.previousPositions=input.tool.positions.map(p=>sub(p,[.03,0,.02]));input.tool.reference={tangent:[0,1,0],director:[0,0,1]};refresh(input);
    assert.throws(()=>evaluateCompositeJointWallSurface(input),/degenerate-wall-tangent-projection/);input.wall.tangentBasis='projected-own-reference-director';
    const ws=createCompositeJointWallSurfaceWorkspace(),base=structuredClone(evaluateCompositeJointWallSurface(input,ws));
    for(let j=0;j<7;j++){const h=1e-6,a=structuredClone(evaluateCompositeJointWallSurface(perturb(input,j,h),ws)),b=evaluateCompositeJointWallSurface(perturb(input,j,-h),ws);
        for(let r=0;r<2;r++)close((a.increment[r]-b.increment[r])/(2*h),base.slipJacobian[7*r+j],3e-8);
        for(let r=0;r<14;r++)close((a.forceMap[r]-b.forceMap[r])/(2*h),base.DforceMap[7*r+j],3e-8);}
    const gauge=copyInput(input);gauge.tool.reference.director=[1,0,0];gauge.tool.angle-=Math.PI/2;gauge.tool.previousAngle-=Math.PI/2;
    const rotated=evaluateCompositeJointWallSurface(gauge);assert.ok(norm(base.increment)>.03);close(norm(base.increment),norm(rotated.increment),3e-14);
    const v=input.tool.positionRates.flat().concat(input.tool.angleRate);const mapped=out=>[0,1].map(r=>v.reduce((s,x,j)=>s+x*out.forceMap[2*j+r],0));close(norm(mapped(base)),norm(mapped(rotated)),3e-14);
});

test('envelope proximal row uses its actual own fraction despite a degenerate detector raw fraction, with explicit material trace',()=>{
    const input=fixture();input.current.envelope=true;input.current.collector=createCompositeWallEnvelopeWorkspace(layout);refresh(input);
    assert.equal(input.current.row.role,'proximal');assert.equal(input.current.row.t,0);assert.equal(input.current.row.rawContact.segmentT,1);
    const out=evaluateCompositeJointWallSurface(input);assert.equal(out.fraction,0);assert.equal(out.identity.rawFraction,1);
    const noTrace=copyInput(input);delete noTrace.tool.trace;assert.throws(()=>evaluateCompositeJointWallSurface(noTrace),/trace/);
});

test('full/value are bit identical and reuse revokes stale maps on invalid sources, geometry, histories, seams and basis',()=>{
    const input=fixture('sdf'),before=structuredClone({tool:input.tool,row:input.current.row,positions:input.current.positions}),ws=createCompositeJointWallSurfaceWorkspace();
    const full=structuredClone(evaluateCompositeJointWallSurface(input,ws));
    const value=evaluateCompositeJointWallSurface({...input,order:'value'},ws);same(value.increment,full.increment,0);same(value.forceMap,full.forceMap,0);assert.equal(value.operatorReady,false);assert.equal(value.forceMapValid,true);
    for(const key of ['slipJacobian','DforceMap','currentQueryJacobian'])assert.ok(value[key].every(Number.isNaN));
    const cases=[x=>{x.wall.source='sparse-sdf-bvh';},x=>{x.current.row.normal[0]+=.1;},x=>{x.current.row.normalDerivative[0]+=.1;},x=>{x.current.row.rawContact.signedDistance+=.1;},
        x=>{x.current.positions[0][0]+=.1;},x=>{x.tool.edge=1;},x=>{x.tool.coordinate=999;},x=>{x.current.seam={};},x=>{x.wall.motion='moving';},
        x=>{x.tool.materialPath.previousEdgeId='other';},x=>{x.tool.materialPath.previousMap.sStart+=100;},x=>{x.wall.tangentBasis='world-x';},
        x=>{x.current.row.sampleCount++;},x=>{x.current.row.faceIndex++;}];
    for(const order of ['full','value'])for(const change of cases){const bad=copyInput(input);bad.order=order;change(bad);assert.throws(()=>evaluateCompositeJointWallSurface(bad,ws));assert.equal(ws.forceMapValid,false);assert.equal(ws.incrementValid,false);
        for(const key of ['increment','forceMap','slipJacobian','DforceMap','currentQueryJacobian'])assert.ok(ws[key].every(Number.isNaN));assert.deepEqual(evaluateCompositeJointWallSurface(input,ws),full);}
    assert.deepEqual({tool:input.tool,row:input.current.row,positions:input.current.positions},before);
});

test('actual anatomy smooth sparse-SDF selected record composes with zero extra field queries and explicit virtual projection metadata',()=>{
    const bytes=fs.readFileSync(new URL('../res/Aorta_plain.collision.bin',import.meta.url));const anatomy=decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
    const input=fixture('sdf');input.current.field=new VesselContactField(anatomy);input.tool.positions=[[65.00287246704102,-462.00980948623305,-79.56869888305664],[64.95548751831055,-461.7916869276393,-79.65612350463867]];
    input.tool.previousPositions=structuredClone(input.tool.positions);input.tool.reference=captureCompositeReferenceFrames(input.tool.positions)[0];input.tool.radius=.4445;input.tool.angle=input.tool.previousAngle;
    input.tool.materialMap.dsDt=0;input.tool.materialPath.previousMap.sStart=20;input.tool.materialPath.previousTrace='left';input.tool.trace='left';refresh(input);
    input.current.field.queryCapsuleCoordinates=()=>{throw Error('additional original query');};input.current.field.querySphere=()=>{throw Error('additional sphere query');};
    const out=evaluateCompositeJointWallSurface(input);assert.equal(out.identity.source,'sparse-sdf');assert.equal(out.identity.wallWitness,'provider-normal-projection-not-exact-isosurface');same(out.increment,[0,0],3e-13);assert.equal(out.queryCount,0);
});

// The prepared path owns the accepted and external poses. The candidate supplies
// only actual physical configuration columns; no fictitious reservoir DOF exists.
import { createCompositeJointSurfacePoseHistory, prepareCompositeJointSurfacePosePath } from '../src/physics/kirchhoffCompositeJointSurfacePoseHistory.js';
import { createCompositeJointPhysicalColumnPullback, pullbackCompositeJointPhysicalColumns, evaluateCompositeJointPhysicalColumnLoads } from '../src/physics/kirchhoffCompositeJointPhysicalColumnPullback.js';
function poseFixture(kind='plane',{adjacent=true,reservoir=true,permuted=false,flat=false,dt=.1}={}) {
    const input=fixture(kind,dt),t=input.tool;
    if(flat){t.positions=[[0,.3,0],[1,.3,0]];t.previousPositions=structuredClone(t.positions);t.reference=captureCompositeReferenceFrames(t.positions)[0];t.angle=t.previousAngle=0;t.coordinates=[0,1];}
    t.materialSegmentId=42;t.materialMap={sStart:20-.3*dt,dsDx:1.3,dsDt:-.3};
    if(flat)t.materialMap={sStart:-.3*dt,dsDx:1,dsDt:-.3};
    delete t.materialPath;
    input.current.envelope=true;input.current.collector=createCompositeWallEnvelopeWorkspace(layout);refresh(input);
    const old=t.previousPositions,dx=t.coordinates[1]-t.coordinates[0],label=flat?0:20,metric=t.materialMap.dsDx,count=adjacent?3:2;
    const oldOwn=[...structuredClone(old),...(adjacent?[add(old[1],sub(old[1],old[0]))]:[])];
    const source={toolId:t.id,reservoirIdentity:reservoir?'external-pose:17':null,
        nodes:[...(reservoir?[{id:'external',position:sub(old[0],sub(old[1],old[0]))}]:[]),...oldOwn.map((position,node)=>({id:node,node,position}))],
        edges:[...(reservoir?[{edgeId:'wire:reservoir',materialSegmentId:'42',source:'reservoir',nodeIds:['external',0],coordinates:[t.coordinates[0]-dx,t.coordinates[0]],labels:[label-dx*metric,label],reference:t.reference,angle:t.previousAngle}]:[]),
            ...Array.from({length:count-1},(_,edge)=>({edgeId:edge===0?t.edgeId:'wire:edge1',edge,materialSegmentId:42,source:'accepted',nodeIds:[edge,edge+1],
                coordinates:[t.coordinates[0]+edge*dx,t.coordinates[1]+edge*dx],labels:[label+edge*dx*metric,label+(edge+1)*dx*metric],reference:t.reference,angle:t.previousAngle}))],hinges:[]};
    for(let j=0;j<source.edges.length-1;j++)source.hinges.push({leftEdgeId:source.edges[j].edgeId,rightEdgeId:source.edges[j+1].edgeId,referenceTwist:0});
    const columns=[...Array.from({length:count},(_,node)=>[0,1,2].map(component=>({kind:'position',toolId:t.id,node,component}))).flat(),...Array.from({length:count-1},(_,edge)=>({kind:'angle',toolId:t.id,edge}))];
    if(permuted)columns.reverse();
    const col=(node,k)=>columns.findIndex(c=>c.kind==='position'&&c.node===node&&c.component===k),angleCol=edge=>columns.findIndex(c=>c.kind==='angle'&&c.edge===edge);
    const currentOwn=[...t.positions,...(adjacent?[add(add(t.positions[1],sub(t.positions[1],t.positions[0])),flat?[0,0,0]:[.01,-.015,.02])]:[])];
    input.configuration=Float64Array.from(columns,c=>c.kind==='position'?currentOwn[c.node][c.component]:c.edge===0?t.angle:flat?0:.17);
    const nodeBindings=source.nodes.map(n=>n.node===undefined?{nodeId:'external',offset:[0,0,0],terms:[0,1,2].flatMap(k=>
        (adjacent?[[0,1],[1,1],[2,-1]]:[[0,2],[1,-1]]).map(([node,weight])=>({column:col(node,k),weights:[0,1,2].map(axis=>axis===k?weight:0)})))}:
        {nodeId:n.id,offset:[0,0,0],terms:[0,1,2].map(k=>({column:col(n.node,k),weights:[0,1,2].map(axis=>axis===k?1:0)}))});
    const angleBindings=source.edges.map(e=>({edgeId:e.edgeId,offset:0,terms:[{column:angleCol(e.source==='reservoir'&&adjacent?1:e.edge??0),weight:1}]}));
    const preparation={history:createCompositeJointSurfacePoseHistory(source),targetEdgeId:t.edgeId,dt,
        currentMaps:source.edges.map(e=>({edgeId:e.edgeId,labels:e.labels.map(x=>x-.3*dt),dsDt:-.3})),configurationColumns:columns,nodeBindings,angleBindings};
    const path=prepareCompositeJointSurfacePosePath(preparation),ws=createCompositeJointWallSurfaceWorkspace({surfacePosePath:path});
    return {input,path,ws,source,preparation,col,angleCol};
}
function posePerturb(f,j,h) {
    const input=copyInput(f.input);input.configuration=Float64Array.from(f.input.configuration);input.configuration[j]+=h;
    const c=f.path.configurationColumns[j];
    if(c.kind==='position'&&c.node<=1)input.tool.positions[c.node][c.component]+=h;
    else if(c.kind==='angle'&&c.edge===0)input.tool.angle+=h;
    return refresh(input);
}

for(const kind of ['plane','sdf','bvh'])test(`${kind}: prepared proximal reservoir feed keeps all 11 physical G/DB columns through current wall re-detection`,context=>{
    const f=poseFixture(kind,{permuted:kind==='bvh'}),{input,ws}=f,calls=input.current.field.calls,base=structuredClone(evaluateCompositeJointWallSurface(input,ws)),N=f.path.configurationDofs;
    assert.equal(N,11);assert.equal(base.fraction,0);assert.equal(base.motion.events[0].terminal,true);assert.equal(base.currentTools[0].materialSegmentId,42);
    assert.equal(input.current.field.calls,calls);assert.equal(base.currentQueryJacobian.length,10*N);assert.equal(base.slipJacobian.length,2*N);assert.equal(base.forceMap.length,2*N);assert.equal(base.DforceMap.length,2*N*N);
    const extra=f.path.configurationColumns.map((c,j)=>c.kind==='position'&&c.node===2||c.kind==='angle'&&c.edge===1?j:-1).filter(j=>j>=0);
    for(const j of extra)same(base.forceMap.slice(2*j,2*j+2),[0,0],0);
    assert.ok(extra.some(j=>Math.abs(base.slipJacobian[j])+Math.abs(base.slipJacobian[N+j])>1e-7));
    let maxG=0,maxDB=0,maxQuery=0,previousChain=0;
    for(let j=0;j<N;j++){
        const h=1e-6,a=structuredClone(evaluateCompositeJointWallSurface(posePerturb(f,j,h),ws)),b=evaluateCompositeJointWallSurface(posePerturb(f,j,-h),ws);
        for(let r=0;r<2;r++){const error=Math.abs((a.increment[r]-b.increment[r])/(2*h)-base.slipJacobian[r*N+j]);maxG=Math.max(maxG,error);assert.ok(error<1e-7,`${kind} G ${r},${j}: ${error}`);
            let chain=0;for(let q=1;q<10;q++)chain+=base.motion.queryJacobian[r*19+q+9]*base.currentQueryJacobian[q*N+j];previousChain=Math.max(previousChain,Math.abs(chain));}
        for(let e=0;e<N*2;e++){const error=Math.abs((a.forceMap[e]-b.forceMap[e])/(2*h)-base.DforceMap[e*N+j]);maxDB=Math.max(maxDB,error);assert.ok(error<1e-7,`${kind} DB ${e},${j}: ${error}`);}
        for(let q=1;q<10;q++){const key=['point','normal','tangent'][Math.floor((q-1)/3)],axis=(q-1)%3,error=Math.abs((a[key][axis]-b[key][axis])/(2*h)-base.currentQueryJacobian[q*N+j]);maxQuery=Math.max(maxQuery,error);assert.ok(error<1e-8);}
    }
    assert.ok(previousChain>1e-5,'previous witness derivative is necessary');
    refresh(input);input.current.field.queryCapsuleCoordinates=()=>{throw Error('duplicate field query');};
    if(kind==='bvh')input.current.field.fallbackGeometry.boundsTree.closestPointToPoint=()=>{throw Error('duplicate BVH query');};
    assert.deepEqual(evaluateCompositeJointWallSurface(input,ws),base);input.current.field.fallbackGeometry?.dispose();
    context.diagnostic(JSON.stringify({kind,N,maxG,maxDB,maxQuery,previousChain,queryCount:base.queryCount}));
});

test('prepared dsDt=-.3 enters explicit proximal pose with nodal endpoint, complete own spin and mapper virtual work',()=>{
    const f=poseFixture('plane',{flat:true}),{input,ws}=f;
    const base=evaluateCompositeJointWallSurface(input,ws);same(base.increment,[.03,0],2e-14);assert.equal(input.current.row.role,'proximal');assert.equal(input.current.row.rawContact.segmentT,1);
    input.tool.angle=2*Math.PI;input.configuration[f.angleCol(0)]=input.tool.angle;
    const spun=evaluateCompositeJointWallSurface(input,ws);same(spun.increment,[.03,-.3*2*Math.PI],3e-14);
    const physicalLayout=createCompositeChainLayout([['wire'],['wire'],['wire']]),mapping=createCompositeJointPhysicalColumnPullback({layout:physicalLayout,modes:[],currentTools:spun.currentTools,configurationColumns:spun.configurationColumns});
    assert.ok(pullbackCompositeJointPhysicalColumns(spun,mapping).operatorReady);
    const traction=[.3,-.8],qdot=Array.from({length:11},(_,j)=>.1*Math.cos(j)),loads=evaluateCompositeJointPhysicalColumnLoads(traction,mapping).physical;
    close(dot(loads,qdot),dot(traction,[0,1].map(k=>qdot.reduce((s,v,j)=>s+v*spun.forceMap[j*2+k],0))),2e-14);
});

test('prepared accepted pose is authoritative despite absent or poisoned legacy previousPositions',()=>{
    const f=poseFixture(),base=structuredClone(evaluateCompositeJointWallSurface(f.input,f.ws));
    for(const old of [undefined,[[NaN,Infinity,999]],[[99,3,4],[95,2,1]]]){
        f.input.tool.previousPositions=old;assert.deepEqual(evaluateCompositeJointWallSurface(f.input,f.ws),base);
    }
    f.source.nodes[0].position[0]=999;f.preparation.nodeBindings[0].terms[0].weights[0]=999;f.preparation.currentMaps[0].labels[0]=999;
    assert.deepEqual(evaluateCompositeJointWallSurface(f.input,f.ws),base);
});

test('prepared path full/value owns N buffers and revokes failures before exact retry; stale identities/maps/configurations reject',()=>{
    const f=poseFixture('sdf',{permuted:true}),{input,ws}=f,before=structuredClone({tool:input.tool,configuration:input.configuration,row:input.current.row}),full=structuredClone(evaluateCompositeJointWallSurface(input,ws));
    const keys=['increment','slipJacobian','forceMap','DforceMap','currentQueryJacobian','point','normal','tangent','center'],buffers=keys.map(k=>ws[k]);
    const value=evaluateCompositeJointWallSurface({...input,order:'value'},ws);same(value.increment,full.increment,0);same(value.forceMap,full.forceMap,0);assert.equal(value.operatorReady,false);assert.equal(value.forceMapValid,true);
    for(const k of ['slipJacobian','DforceMap','currentQueryJacobian'])assert.ok(value[k].every(Number.isNaN));
    assert.throws(()=>{ws.forceMap=new Float64Array(22);},TypeError);assert.throws(()=>{ws.configurationColumns=[];},TypeError);
    const changes=[x=>{x.dt*=2;},x=>{x.tool.id='catheter';},x=>{x.tool.edge=1;},x=>{delete x.tool.edge;},x=>{x.tool.edgeId='wire:edge1';},x=>{x.tool.materialSegmentId='42';},x=>{delete x.tool.materialSegmentId;},
        x=>{x.configuration[f.col(0,0)]+=.1;},x=>{x.configuration[f.angleCol(0)]+=.1;},x=>{x.configuration[0]=NaN;},x=>{x.configuration=x.configuration.slice(1);},
        x=>{x.tool.coordinates[1]+=.1;},x=>{x.tool.materialMap.sStart+=.1;},x=>{x.tool.materialMap.dsDx+=.1;},x=>{x.tool.materialMap.dsDt=.3;},x=>{x.tool.materialMap.dsDtEnds=[-.3,.3];},
        x=>{x.tool.previousAngle+=.1;},x=>{x.tool.reference.director[0]+=.1;},x=>{x.tool.materialPath={kind:'linear-affine-maps',previousEdgeId:x.tool.edgeId,previousMap:{sStart:0,dsDx:1.3}};},
        x=>{x.current.row.normalDerivative[0]+=.1;},x=>{x.current.row.rawContact.signedDistance+=.1;},x=>{x.wall.motion='moving';},x=>{delete x.tool.trace;}];
    for(const order of ['value','full'])for(const change of changes){const bad=copyInput(input);bad.configuration=Float64Array.from(input.configuration);bad.order=order;change(bad);
        assert.throws(()=>evaluateCompositeJointWallSurface(bad,ws));assert.equal(ws.operatorReady,false);assert.equal(ws.forceMapValid,false);assert.equal(ws.motion,null);assert.equal(ws.physicalForce,null);
        keys.forEach(k=>assert.ok(ws[k].every(Number.isNaN),k));assert.deepEqual(evaluateCompositeJointWallSurface(input,ws),full);keys.forEach((k,j)=>assert.equal(ws[k],buffers[j]));}
    assert.deepEqual({tool:input.tool,configuration:input.configuration,row:input.current.row},before);
    assert.throws(()=>createCompositeJointWallSurfaceWorkspace({surfacePosePath:structuredClone(f.path)}),/prepared own surface pose path/);
});

test('same-edge history without external pose explicitly rejects proximal entering labels, then supports interior same-edge trace',()=>{
    const f=poseFixture('plane',{reservoir:false});
    assert.throws(()=>evaluateCompositeJointWallSurface(f.input,f.ws),{code:'surface-material-transport-required'});
    assert.equal(f.ws.supported,false);assert.ok(f.ws.requiredTransport);assert.ok(f.ws.increment.every(Number.isNaN));
    // Re-detect the original interior sparse-SDF capsule branch, without changing history.
    const s=poseFixture('sdf',{reservoir:false});s.input.current.envelope=false;s.input.current.collector=createCompositeWallWorkspace(layout);refresh(s.input);
    const pathResult=evaluateCompositeJointWallSurface(s.input,s.ws),legacy=copyInput(s.input);
    legacy.tool.materialPath={kind:'linear-affine-maps',previousEdgeId:legacy.tool.edgeId,previousMap:{sStart:20,dsDx:1.3}};
    const old=evaluateCompositeJointWallSurface(legacy),map=[0,1,2,3,4,5,s.angleCol(0)];same(pathResult.increment,old.increment,3e-14);
    for(let p=0;p<7;p++){same(pathResult.forceMap.slice(map[p]*2,map[p]*2+2),old.forceMap.slice(p*2,p*2+2),0);for(let k=0;k<2;k++)close(pathResult.slipJacobian[k*11+map[p]],old.slipJacobian[k*7+p],3e-13);}
});

for(const kind of ['plane','sdf'])test(`${kind}: prepared own-reference wall basis retains all derivatives under bending and spin`,()=>{
    const f=poseFixture(kind);f.input.wall.tangentBasis='projected-own-reference-director';
    const r=structuredClone(evaluateCompositeJointWallSurface(f.input,f.ws)),N=f.path.configurationDofs;
    for(let j=0;j<N;j++){const h=1e-6,a=structuredClone(evaluateCompositeJointWallSurface(posePerturb(f,j,h),f.ws)),b=evaluateCompositeJointWallSurface(posePerturb(f,j,-h),f.ws);
        for(let k=0;k<2;k++)close((a.increment[k]-b.increment[k])/(2*h),r.slipJacobian[k*N+j],1e-7);
        for(let e=0;e<2*N;e++)close((a.forceMap[e]-b.forceMap[e])/(2*h),r.DforceMap[e*N+j],1e-7);}
});

test('prepared proximal bending/spin and dsDt=-.3 converge to instantaneous stationary-wall virtual power',context=>{
    const errors=[],powerErrors=[],rates=[[.04,-.03,.02],[-.02,.05,.01]],spin=.7,traction=[.3,-.8];
    for(const dt of [1e-3,1e-4,1e-5]){
        const f=poseFixture('plane',{flat:true,adjacent:false,dt}),{input,ws}=f;
        const old=structuredClone(evaluateCompositeJointWallSurface(input,ws));
        rates.forEach((v,node)=>v.forEach((x,k)=>{input.tool.positions[node][k]+=dt*x;input.configuration[f.col(node,k)]=input.tool.positions[node][k];}));
        input.tool.angle=spin*dt;input.configuration[f.angleCol(0)]=input.tool.angle;refresh(input);
        const r=evaluateCompositeJointWallSurface(input,ws),v=Array.from(r.increment,x=>x/dt),qdot=rates.flat().concat(spin),expected=[0,1].map(k=>qdot.reduce((s,x,j)=>s+old.forceMap[2*j+k]*x,0)+(k===0?.3:0));
        errors.push(norm(sub(v,expected)));powerErrors.push(Math.abs(dot(traction,v)-dot(traction,expected)));
    }
    assert.ok(errors[1]<.11*errors[0]&&errors[2]<.11*errors[1]);assert.ok(powerErrors[1]<.11*powerErrors[0]&&powerErrors[2]<.11*powerErrors[1]);
    context.diagnostic(JSON.stringify({dt:[1e-3,1e-4,1e-5],errors,powerErrors}));
});

test('prepared full/value equality spans N=7/11, reordered columns, all smooth wall sources and both own tangent bases',()=>{
    for(const kind of ['plane','sdf','bvh'])for(const adjacent of [false,true])for(const basis of ['projected-own-tangent','projected-own-reference-director']){
        const f=poseFixture(kind,{adjacent,permuted:true});f.input.wall.tangentBasis=basis;
        const full=structuredClone(evaluateCompositeJointWallSurface(f.input,f.ws)),value=evaluateCompositeJointWallSurface({...f.input,order:'value'},f.ws);
        for(const k of ['increment','forceMap','point','normal','tangent','center'])assert.deepEqual(value[k],full[k]);assert.equal(value.slipJacobianValid,false);assert.equal(value.DforceMapValid,false);
        assert.deepEqual(evaluateCompositeJointWallSurface(f.input,f.ws),full);f.input.current.field.fallbackGeometry?.dispose();
    }
});

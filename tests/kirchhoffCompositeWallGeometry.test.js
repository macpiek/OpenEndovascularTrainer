import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import {pathToFileURL} from 'node:url';
import {
    evaluateSparseSdfTrilinearDerivatives, createCompositeWallGeometryWorkspace,
    differentiateCompositeWallContact, queryCompositeWallPointGeometry, queryCompositeWallCapsuleGeometry,
    evaluateCompositeWallMixedRow, evaluateCompositeWallGapAugmented,
} from '../src/physics/kirchhoffCompositeWallGeometry.js';

const providerRoot = process.env.OET_WALL_GEOMETRY_PROVIDER_ROOT;
const providerUrl = name => providerRoot ? pathToFileURL(`${providerRoot}/src/physics/collision/${name}.js`) : new URL(`../src/physics/collision/${name}.js`, import.meta.url);
const {VesselContactField, createContactResult} = await import(providerUrl('vesselContactField'));
const {decodeCollisionAsset} = await import(providerUrl('collisionAssetFormat'));
const close = (a, b, tol = 1e-8) => assert.ok(Math.abs(a - b) <= tol * Math.max(1, Math.abs(a), Math.abs(b)), `${a} != ${b}`);
const vectorClose = (a, b, tol) => { assert.equal(a.length, b.length); a.forEach((v, i) => close(v, b[i], tol)); };

const polynomial = ([x, y, z]) => 2 + .125*x + .25*y + .5*z + .125*x*y + .25*x*z + .5*y*z + .0625*x*y*z;
const polynomialGradient = ([x, y, z]) => [.125 + .125*y + .25*z + .0625*y*z, .25 + .125*x + .5*z + .0625*x*z, .5 + .25*x + .5*y + .0625*x*y];
const polynomialHessian = ([x, y, z]) => [0, .125+.0625*z, .25+.0625*y, .125+.0625*z, 0, .5+.0625*x, .25+.0625*y, .5+.0625*x, 0];
const valley = ([x, y, z]) => 2 + .5*(x-.125)*(y-.125) + .25*z;
const valleyGradient = ([x, y]) => [.5*(y-.125), .5*(x-.125), .25];
function grid(fn, origin = [0, 0, 0], dimensions = [2, 2, 2]) {
    const size = 2, h = .5, quantization = 1/1024, count = dimensions.reduce((a, b) => a*b, 1);
    const table = new Uint16Array(count), values = new Uint32Array(count*size**3);
    for (let bz=0; bz<dimensions[2]; bz++) for (let by=0; by<dimensions[1]; by++) for (let bx=0; bx<dimensions[0]; bx++) {
        const brick = bx + dimensions[0]*(by+dimensions[1]*bz); table[brick]=brick;
        for (let z=0; z<size; z++) for (let y=0; y<size; y++) for (let x=0; x<size; x++) {
            const position=[origin[0]+h*(bx*size+x),origin[1]+h*(by*size+y),origin[2]+h*(bz*size+z)];
            const encoded=fn(position)/quantization; assert.equal(encoded,Math.round(encoded));
            values[brick*size**3+x+size*(y+size*z)]=encoded;
        }
    }
    return {sdfOrigin:origin, sdfDimensions:dimensions, brickSize:size, voxelSize:h,
        sdfQuantization:quantization, sdfBrickLookup:table, sdfDistances:values};
}
function analyticField(sign = 1) {
    const field=grid(valley,[-1,-1,-1],[3,3,3]);field.pointCalls=0;field.capsuleCalls=0;
    const sample=(point,radius,out)=>{
        const gradient=valleyGradient(point),m=Math.hypot(...gradient);
        out.signedDistance=sign*valley(point);out.signedGap=out.signedDistance-radius;
        out.inward.values.set(gradient.map(v=>sign*v/m));out.source='sparse-sdf';out.faceIndex=-1;
        out.closestPoint.values.set(point.map((v,i)=>v-out.signedDistance*out.inward.values[i]));return out;
    };
    field.querySphere=(point,radius,out=createContactResult())=>{field.pointCalls++;return sample(point,radius,out);};
    field.queryCapsuleCoordinates=(ax,ay,az,bx,by,bz,radius,out=createContactResult())=>{
        field.capsuleCalls++; const a=[ax,ay,az],b=[bx,by,bz];let best=Infinity,t=0;
        for(const fraction of [0,.5,1]) {const p=a.map((v,i)=>v+(b[i]-v)*fraction),g=sign*valley(p)-radius;if(g<best){best=g;t=fraction;}}
        sample(a.map((v,i)=>v+(b[i]-v)*t),radius,out);out.segmentT=t;out.capsuleSampleCount=2;return out;
    };
    return field;
}
let anatomy;
function realField() {
    if(!anatomy){
        const path=providerRoot ? `${providerRoot}/res/Aorta_plain.collision.bin` : new URL('../res/Aorta_plain.collision.bin',import.meta.url);
        const bytes=fs.readFileSync(path);anatomy=decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
    }
    return new VesselContactField(anatomy);
}
const witness=[
    [65.00287246704102,-462.00980948623305,-79.56869888305664],
    [64.95548751831055,-461.7916869276393,-79.65612350463867],
];

test('the exact trilinear value, gradient and Hessian use quantized corners across brick boundaries',()=>{
    const field=grid(polynomial),point=[.625,.875,.7];
    const out=evaluateSparseSdfTrilinearDerivatives({field,position:point});
    assert.ok(out.supported);assert.deepEqual([...out.cell],[1,1,1]);
    close(out.value,polynomial(point),1e-14);vectorClose(out.gradient,polynomialGradient(point),1e-14);
    vectorClose(out.hessian,polynomialHessian(point),1e-14);
    const eps=1e-6;
    for(let j=0;j<3;j++){
        const plus=evaluateSparseSdfTrilinearDerivatives({field,position:point.map((v,i)=>v+(i===j?eps:0))});
        const minus=evaluateSparseSdfTrilinearDerivatives({field,position:point.map((v,i)=>v-(i===j?eps:0))});
        close(out.gradient[j],(plus.value-minus.value)/(2*eps));
        for(let i=0;i<3;i++)close(out.hessian[3*i+j],(plus.gradient[i]-minus.gradient[i])/(2*eps));
    }
});

for(const sign of [1,-1]) test(`point geometry preserves detection and separates signed gradient from unit normal (${sign})`,()=>{
    const field=analyticField(sign),point=[.25,.35,.2],radius=.4;
    const ws=createCompositeWallGeometryWorkspace(1),contact=createContactResult();
    const out=queryCompositeWallPointGeometry({field,position:point,radius,contactResult:contact},ws);
    assert.equal(field.pointCalls,1);assert.ok(out.supported,out.reason);assert.equal(out.branchSign,sign);
    close(out.gap,sign*valley(point)-radius);vectorClose(out.gapGradient,valleyGradient(point).map(v=>sign*v));
    close(Math.hypot(...out.normal),1);assert.ok(Math.abs(out.gradientNorm-1)>.5);
    const initial=structuredClone(out),eps=1e-6;
    for(let j=0;j<3;j++){
        const plus=queryCompositeWallPointGeometry({field,position:point.map((v,i)=>v+(i===j?eps:0)),radius});
        const minus=queryCompositeWallPointGeometry({field,position:point.map((v,i)=>v-(i===j?eps:0)),radius});
        close(initial.gapGradient[j],(plus.gap-minus.gap)/(2*eps));
        for(let i=0;i<3;i++){
            close(initial.gapHessian[3*i+j],(plus.gapGradient[i]-minus.gapGradient[i])/(2*eps));
            close(initial.normalForceJacobian[3*i+j],(plus.normal[i]-minus.normal[i])/(2*eps));
        }
    }
    const reused=queryCompositeWallPointGeometry({field,position:point,radius,contactResult:contact},ws);
    assert.equal(reused.gapGradient,out.gapGradient);assert.equal(reused.normalForceJacobian,out.normalForceJacobian);
});

test('the capsule pullback preserves the selected interior sample and differentiates both endpoints',()=>{
    const field=analyticField(),positions=[[-.625,-.625,.125],[.875,.875,.125]],radius=2.2;
    const base=queryCompositeWallCapsuleGeometry({field,positions,radius});
    assert.equal(field.capsuleCalls,1);assert.ok(base.supported,base.reason);assert.equal(base.sampleFraction,.5);assert.equal(base.sampleCount,2);
    vectorClose(base.weights,[.5,.5]);const eps=1e-6;
    for(let j=0;j<6;j++){
        const move=delta=>positions.map((p,node)=>p.map((v,axis)=>v+(3*node+axis===j?delta:0)));
        const plus=queryCompositeWallCapsuleGeometry({field,positions:move(eps),radius}),minus=queryCompositeWallCapsuleGeometry({field,positions:move(-eps),radius});
        assert.ok(plus.supported&&minus.supported);assert.equal(plus.branchSignature,base.branchSignature);assert.equal(minus.branchSignature,base.branchSignature);
        close(base.gapGradient[j],(plus.gap-minus.gap)/(2*eps));
        for(let i=0;i<6;i++){
            close(base.gapHessian[6*i+j],(plus.gapGradient[i]-minus.gapGradient[i])/(2*eps));
            close(base.normalForceJacobian[6*i+j],(plus.normalForceColumn[i]-minus.normalForceColumn[i])/(2*eps));
        }
    }
});

test('real anatomy point and capsule queries keep the original gap, normal and sampling policy',()=>{
    const field=realField(),scratch=createContactResult(),radius=.4445;
    const beforeOptions=[field.voxelSize,field.capsuleBvhValidationGap,field.bvhValidationDistance];
    const raw=field.queryCapsuleCoordinates(...witness[0],...witness[1],radius,scratch);
    const copy={gap:raw.signedGap,normal:[...raw.inward.values],t:raw.segmentT,samples:raw.capsuleSampleCount,source:raw.source};
    const capsule=queryCompositeWallCapsuleGeometry({field,positions:witness,radius,contactResult:scratch});
    assert.ok(capsule.supported,capsule.reason);assert.equal(capsule.gap,copy.gap);vectorClose(capsule.normal,copy.normal,0);
    assert.equal(capsule.sampleFraction,copy.t);assert.equal(capsule.sampleCount,copy.samples);assert.equal(capsule.source,copy.source);
    assert.deepEqual([field.voxelSize,field.capsuleBvhValidationGap,field.bvhValidationDistance],beforeOptions);
    for(const point of [witness[1],[65.00287246704102,-458.2578430175781,-79.56869888305664]]){
        const g=queryCompositeWallPointGeometry({field,position:point,radius});assert.ok(g.supported,g.reason);
        const eps=1e-4;
        for(let i=0;i<3;i++){
            const plus=field.querySphere(point.map((v,j)=>v+(i===j?eps:0)),radius,createContactResult());
            const minus=field.querySphere(point.map((v,j)=>v-(i===j?eps:0)),radius,createContactResult());
            close(g.gapGradient[i],(plus.signedGap-minus.signedGap)/(2*eps),1e-7);
        }
    }
});

test('the existing real-anatomy witness has the exact energy gradient in explicit gap-potential mode',()=>{
    const field=realField(),radius=.4445,options={gapMultiplier:2,penalty:10,mode:'gap-potential'};
    const evaluate=dx=>{
        const geometry=queryCompositeWallCapsuleGeometry({field,positions:witness.map(p=>p.map((v,i)=>v+(i===0?dx:0))),radius});
        assert.ok(geometry.supported,geometry.reason);return {geometry,al:evaluateCompositeWallGapAugmented({geometry,...options})};
    };
    const base=evaluate(0),eps=1e-4,plus=evaluate(eps),minus=evaluate(-eps),al=base.al,g=base.geometry;
    close(g.gap,-.02,1e-12);close(al.energy,.042,1e-12);
    const derivative=(plus.al.energy-minus.al.energy)/(2*eps),assembled=al.gradient[0]+al.gradient[3];
    close(assembled,derivative,1e-7);close(derivative,-.68952570255,1e-7);
    const oldGradient=-al.trialGapMultiplier*g.normal[0];assert.ok(Math.abs(oldGradient-derivative)>.98);
    close(al.effectiveNormalForce,2*g.gradientNorm);close(al.effectiveTrialNormalForce,2.2*g.gradientNorm);
    for(let i=0;i<6;i++)close(al.gradient[i],-al.effectiveTrialNormalForce*g.normalForceColumn[i]);
    assert.throws(()=>evaluateCompositeWallGapAugmented({geometry:g,gapMultiplier:2,penalty:10}),/explicit/);
    assert.throws(()=>evaluateCompositeWallGapAugmented({geometry:g,gapMultiplier:2,penalty:10,mode:'physical-normal'}),/physical-normal/);
});

for(const active of [true,false]) test(`mixed NCP uses actual Jg and physical normal, including Dn (${active?'active':'inactive'})`,()=>{
    const field=analyticField(),position=[.25,.35,.2],radius=active?2.3:.1,normalForce=.7,penalty=10,eps=1e-6;
    const evaluate=(p,force)=>{
        const geometry=queryCompositeWallPointGeometry({field,position:p,radius});assert.ok(geometry.supported,geometry.reason);
        return {geometry,row:evaluateCompositeWallMixedRow({geometry,normalForce:force,penalty})};
    };
    const {geometry:g,row:r}=evaluate(position,normalForce);assert.equal(r.active,active);assert.equal(r.symmetric,false);
    assert.ok(Math.abs(r.mechanicalJacobian[1]-r.mechanicalJacobian[3])>1e-4,'normalized-SDF physical force generally has curl');
    for(let j=0;j<3;j++){
        const plus=evaluate(position.map((v,i)=>v+(i===j?eps:0)),normalForce).row;
        const minus=evaluate(position.map((v,i)=>v-(i===j?eps:0)),normalForce).row;
        close(r.ncpGeometryRow[j],(plus.ncpResidual-minus.ncpResidual)/(2*eps));
        for(let i=0;i<3;i++)close(r.mechanicalJacobian[3*i+j],(plus.mechanicalGradient[i]-minus.mechanicalGradient[i])/(2*eps));
    }
    const plus=evaluate(position,normalForce+eps).row,minus=evaluate(position,normalForce-eps).row;
    close(r.ncpForceDerivative,(plus.ncpResidual-minus.ncpResidual)/(2*eps));
    r.forceColumn.forEach((v,i)=>close(v,(plus.mechanicalGradient[i]-minus.mechanicalGradient[i])/(2*eps)));
    close(Math.hypot(...r.mechanicalGradient),normalForce);assert.equal('converged' in r,false);
    if(active)assert.ok(Math.abs(r.ncpGeometryRow[0]/penalty+r.forceColumn[0])>.1);
    vectorClose(r.forceColumn,Array.from(g.normal,v=>-v));
});

test('the gap-potential Hessian includes the signed trilinear geometric term',()=>{
    const field=analyticField(),position=[.25,.35,.2],radius=2.3,options={gapMultiplier:2,penalty:7,mode:'gap-potential'};
    const evaluate=p=>evaluateCompositeWallGapAugmented({geometry:queryCompositeWallPointGeometry({field,position:p,radius}),...options});
    const base=evaluate(position),eps=1e-6;
    for(let j=0;j<3;j++){
        const plus=evaluate(position.map((v,i)=>v+(i===j?eps:0))),minus=evaluate(position.map((v,i)=>v-(i===j?eps:0)));
        close(base.gradient[j],(plus.energy-minus.energy)/(2*eps));
        for(let i=0;i<3;i++)close(base.hessian[3*i+j],(plus.gradient[i]-minus.gradient[i])/(2*eps));
    }
    assert.ok(Math.abs(base.hessian[1]-base.gaussNewton[1])>1);assert.equal(base.hessianType,'exact-fixed-branch');
});

test('the real-anatomy mixed row differentiates physical Fn and all six capsule coordinates',()=>{
    const field=realField(),radius=.4445,normalForce=2,penalty=10,eps=1e-4;
    const evaluate=(positions,force=normalForce)=>{
        const geometry=queryCompositeWallCapsuleGeometry({field,positions,radius});assert.ok(geometry.supported,geometry.reason);
        return {geometry,row:evaluateCompositeWallMixedRow({geometry,normalForce:force,penalty})};
    };
    const {geometry:g,row:r}=evaluate(witness);close(r.ncpResidual,-.2,1e-12);assert.ok(r.active);
    for(let j=0;j<6;j++){
        const move=delta=>witness.map((p,node)=>p.map((v,axis)=>v+(3*node+axis===j?delta:0)));
        const plus=evaluate(move(eps)),minus=evaluate(move(-eps));
        assert.equal(plus.geometry.branchSignature,g.branchSignature);assert.equal(minus.geometry.branchSignature,g.branchSignature);
        close(r.ncpGeometryRow[j],(plus.row.ncpResidual-minus.row.ncpResidual)/(2*eps),1e-7);
        for(let i=0;i<6;i++){
            close(r.mechanicalJacobian[6*i+j],(plus.row.mechanicalGradient[i]-minus.row.mechanicalGradient[i])/(2*eps),1e-7);
            close(g.gapHessian[6*i+j],(plus.geometry.gapGradient[i]-minus.geometry.gapGradient[i])/(2*eps),1e-7);
        }
    }
    const plus=evaluate(witness,normalForce+eps).row,minus=evaluate(witness,normalForce-eps).row;
    r.forceColumn.forEach((v,i)=>close(v,(plus.mechanicalGradient[i]-minus.mechanicalGradient[i])/(2*eps),1e-9));
    close(r.ncpForceDerivative,(plus.ncpResidual-minus.ncpResidual)/(2*eps),1e-9);
    const resultant=[0,1,2].map(a=>r.mechanicalGradient[a]+r.mechanicalGradient[3+a]);
    close(Math.hypot(...resultant),normalForce,1e-12);
});

test('length and force unit changes preserve Jg, physical normal and both operator conventions',()=>{
    const originalField=analyticField(),position=[.25,.35,.2],radius=2.3,L=1000,F=.01;
    const baseGeometry=queryCompositeWallPointGeometry({field:originalField,position,radius});
    const scaledField={...originalField,sdfOrigin:originalField.sdfOrigin.map(v=>v*L),voxelSize:originalField.voxelSize*L,sdfQuantization:originalField.sdfQuantization*L};
    const contact=originalField.querySphere(position,radius,createContactResult());
    contact.signedDistance*=L;contact.signedGap*=L;
    const scaledGeometry=differentiateCompositeWallContact({field:scaledField,contact,positions:[position.map(v=>v*L)],radius:radius*L},createCompositeWallGeometryWorkspace(1));
    assert.ok(scaledGeometry.supported,scaledGeometry.reason);
    vectorClose(scaledGeometry.gapGradient,baseGeometry.gapGradient,1e-12);vectorClose(scaledGeometry.normal,baseGeometry.normal,1e-12);
    vectorClose(Array.from(scaledGeometry.normalForceJacobian,v=>v*L),baseGeometry.normalForceJacobian,1e-12);
    const original=evaluateCompositeWallMixedRow({geometry:baseGeometry,normalForce:2,penalty:7});
    const scaled=evaluateCompositeWallMixedRow({geometry:scaledGeometry,normalForce:2*F,penalty:7*F/L});
    vectorClose(Array.from(scaled.mechanicalGradient,v=>v/F),original.mechanicalGradient,1e-12);
    vectorClose(Array.from(scaled.mechanicalJacobian,v=>v*L/F),original.mechanicalJacobian,1e-12);
    vectorClose(Array.from(scaled.ncpGeometryRow,v=>v*L/F),original.ncpGeometryRow,1e-12);close(scaled.ncpResidual/F,original.ncpResidual,1e-12);
    const a=evaluateCompositeWallGapAugmented({geometry:baseGeometry,gapMultiplier:2,penalty:7,mode:'gap-potential'});
    const b=evaluateCompositeWallGapAugmented({geometry:scaledGeometry,gapMultiplier:2*F,penalty:7*F/L,mode:'gap-potential'});
    close(b.energy/(F*L),a.energy,1e-12);close(b.effectiveNormalForce/F,a.effectiveNormalForce,1e-12);
    vectorClose(Array.from(b.gradient,v=>v/F),a.gradient,1e-12);vectorClose(Array.from(b.hessian,v=>v*L/F),a.hessian,1e-12);
});

test('unsupported geometry cannot be mistaken for a physical or energy certificate',()=>{
    const field=analyticField(),position=[.25,.35,.2],radius=.4,contact=field.querySphere(position,radius,createContactResult());
    const ws=createCompositeWallGeometryWorkspace(1),differentiate=()=>differentiateCompositeWallContact({field,contact,positions:[position],radius},ws);
    for(const source of ['sparse-sdf-bvh','centerline-safe-core','fallback','centerline-estimate']){
        contact.source=source;assert.equal(differentiate().supported,false);
        assert.throws(()=>evaluateCompositeWallMixedRow({geometry:ws,normalForce:1,penalty:10}),/supported/);
    }
    contact.source='sparse-sdf';contact.signedGap+=.1;assert.equal(differentiate().reason,'contact-does-not-match-sdf-polynomial');
    contact.signedGap-=.1;contact.inward.values.set([1,0,0]);assert.equal(differentiate().reason,'normal-is-not-normalized-signed-sdf-gradient');
    const boundary=queryCompositeWallPointGeometry({field,position:[.5,.35,.2],radius});assert.equal(boundary.reason,'sdf-cell-boundary');
    field.sdfBrickLookup.fill(0xffff);assert.equal(queryCompositeWallPointGeometry({field,position,radius}).reason,'missing-sdf-corner');
    const flat=analyticField();flat.sdfDistances.fill(2/flat.sdfQuantization);
    const flatContact=flat.querySphere(position,radius,createContactResult());flatContact.signedDistance=2;flatContact.signedGap=2-radius;
    const plateau=differentiateCompositeWallContact({field:flat,contact:flatContact,positions:[position],radius},createCompositeWallGeometryWorkspace(1));
    assert.equal(plateau.reason,'provider-fallback-or-unresolved-normal');
    assert.throws(()=>queryCompositeWallPointGeometry({field,position:[NaN,0,0],radius}),/finite/);
    const good=queryCompositeWallPointGeometry({field:analyticField(),position,radius});
    assert.throws(()=>evaluateCompositeWallMixedRow({geometry:good,normalForce:-1,penalty:10}),/Fn/);
    assert.throws(()=>evaluateCompositeWallMixedRow({geometry:good,normalForce:1,penalty:0}),/positive/);
});

import test from 'node:test';
import assert from 'node:assert/strict';
import {createCompositeDiscreteWallPointBranches,discoverCompositeDiscreteWallPointSeam} from '../src/physics/compositeDiscreteWallPointBranches.js';
const close=(a,b,t=1e-7)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<t,`${a} != ${b}`);
function field(){
    const f={sdfOrigin:[0,0,0],sdfDimensions:[2,2,2],brickSize:2,voxelSize:1,sdfQuantization:1/1024,
        sdfBrickLookup:new Uint16Array(8),sdfDistances:new Uint32Array(64),sdfInsideBits:new Uint8Array(8).fill(255),calls:0};
    const value=([x,y,z])=>2+.25*y+.125*z+.125*y*z-.5*Math.max(0,x-1);
    for(let bz=0;bz<2;bz++)for(let by=0;by<2;by++)for(let bx=0;bx<2;bx++){
        const brick=bx+2*(by+2*bz);f.sdfBrickLookup[brick]=brick;
        for(let z=0;z<2;z++)for(let y=0;y<2;y++)for(let x=0;x<2;x++)f.sdfDistances[8*brick+x+2*(y+2*z)]=1024*value([2*bx+x,2*by+y,2*bz+z]);
    }
    f.queryCapsuleCoordinates=function(ax,ay,az,bx,by,bz,r,out){
        assert.deepEqual([ax,ay,az],[bx,by,bz]);this.calls++;const p=[ax,ay,az],sd=value(p),g=[ax<1?0:-.5,.25+.125*az,.125+.125*ay],n=g.map(v=>v/Math.hypot(...g));
        out.source='sparse-sdf';out.signedDistance=sd;out.signedGap=sd-r;out.segmentT=1;out.capsuleSampleCount=1;
        out.point.values.set(p);out.inward.values.set(n);out.closestPoint.values.set(p.map((v,k)=>v-sd*n[k]));out.faceIndex=-1;return out;
    };return f;
}
const face={axis:0,gridIndex:1},domainBox={lower:[.7,.1,.1],upper:[1.3,.9,.9]};
test('seam discovery uses one existing point query and never alters it',()=>{
    const f=field(),helper=createCompositeDiscreteWallPointBranches({fraction:1}),p=[[.9,.4,.6],[.99,.4,.6]],radius=2.205,
        original=helper.point.refresh({field:f,positions:p,radius}),before=structuredClone(original.rawContact),calls=f.calls,
        result=discoverCompositeDiscreteWallPointSeam({field:f,original,radius});
    assert.equal(result.supported,true,result.reason);assert.deepEqual(result.sdfSeam.face,face);assert.equal(result.selectedBranch,0);
    assert.equal(f.calls,calls);assert.deepEqual(original.rawContact,before);
    assert.equal(helper.refresh({field:f,positions:p,radius,...result.sdfSeam}).supported,true);
    original.point[0]=.5;
    assert.equal(discoverCompositeDiscreteWallPointSeam({field:f,original,radius}).reason,'no-nearby-cell-face');
});
test('seam discovery refuses an unproved sign branch and a grid edge',()=>{
    const f=field(),helper=createCompositeDiscreteWallPointBranches({fraction:1}),radius=2,
        original=helper.point.refresh({field:f,positions:[[1,.4,.6],[1,.4,.6]],radius});
    f.sdfInsideBits.fill(0);
    assert.equal(discoverCompositeDiscreteWallPointSeam({field:f,original,radius}).supported,false);
    original.point.set([1,1,.6]);
    const edge=discoverCompositeDiscreteWallPointSeam({field:f,original,radius});
    assert.equal(edge.supported,false);assert.ok(edge.attempts.every(a=>a.reason==='multiple-cell-faces'));
});
test('seam discovery identifies the actual blocked anatomy face without a hardcoded axis',async()=>{
    const {readFileSync}=await import('node:fs'),{createCompositeAnatomyField}=await import('./helpers/compositeAnatomyField.js'),anatomy=createCompositeAnatomyField();
    try {
        const report=JSON.parse(readFileSync(new URL('../reports/composite-material-point-rejected-step361.json',import.meta.url),'utf8')),
            origin=JSON.parse(readFileSync(new URL('./fixtures/compositeNativeAppInitial.json',import.meta.url),'utf8')).context.sheaths[0].start,
            wire=new Map(report.result.diagnostics.rejectedConfiguration.positions).get('wire'),p=[38,39].map(i=>wire[i].map((v,k)=>v+origin[k])),
            radius=JSON.parse(report.result.diagnostics.certificate.wall.samples.find(s=>s.edge===38).key)[2],
            helper=createCompositeDiscreteWallPointBranches({fraction:1}),original=helper.point.refresh({field:anatomy.field,positions:p,radius}),
            result=discoverCompositeDiscreteWallPointSeam({field:anatomy.field,original,radius});
        assert.equal(result.supported,true,result.reason);assert.deepEqual(result.sdfSeam.face,{axis:2,gridIndex:215});
        assert.equal(helper.refresh({field:anatomy.field,positions:p,radius,...result.sdfSeam}).supported,true);
    } finally {anatomy.dispose();}
});
function positions(center,f){return [center.map((v,k)=>v-(k===1?.2:0)*f),center.map((v,k)=>v+(k===1?.2:0)*(1-f))];}
for(const fraction of [0,.37,1])test(`both wall branches pull back to fixed material point ${fraction}`,()=>{
    const source=field(),helper=createCompositeDiscreteWallPointBranches({fraction}),p=positions([.95,.4,.6],fraction),radius=2.205;
    const input={field:source,positions:p,radius,face,domainBox};let queries=0;
    const out=helper.refresh({...input,consumeQuery:()=>queries++});
    assert.equal(out.supported,true,out.reason);assert.equal(queries,1);assert.equal(out.selectedBranch,0);
    assert.equal(out.original.rawContact.source,'sparse-sdf');assert.equal(out.branches[1].source,'sparse-sdf-cell-polynomial');
    const base=structuredClone(out.branches),h=1e-6;
    for(let j=0;j<6;j++){
        const plus=structuredClone(p),minus=structuredClone(p);plus[Math.floor(j/3)][j%3]+=h;minus[Math.floor(j/3)][j%3]-=h;
        const a=structuredClone(helper.refresh({...input,positions:plus}).branches),b=structuredClone(helper.refresh({...input,positions:minus}).branches);
        for(let branch=0;branch<2;branch++){
            close((a[branch].gap-b[branch].gap)/(2*h),base[branch].gapJacobian[j]);
            for(let i=0;i<6;i++)close(-(a[branch].forceColumn[i]-b[branch].forceColumn[i])/(2*h),base[branch].normalDerivative[i*6+j]);
        }
    }
});

test('exact corner supports two normal reactions without changing original point data',()=>{
    const f=field(),helper=createCompositeDiscreteWallPointBranches({fraction:.5}),p=positions([1,.4,.6],.5),radius=2.205;
    const out=helper.refresh({field:f,positions:p,radius,face,domainBox});assert.equal(out.supported,true,out.reason);assert.equal(out.onSeam,true);
    const raw=structuredClone(out.original.rawContact),proof=helper.measure({forces:[2,3],penalty:10,gapTolerance:1e-8,forceTolerance:1e-7,workTolerance:1e-7});
    assert.equal(proof.converged,true);assert.deepEqual(out.original.rawContact,raw);
    proof.resultant.forEach((v,k)=>close(v,2*out.branches[0].normal[k]+3*out.branches[1].normal[k]));
    f.sdfInsideBits.fill(0);const invalid=helper.refresh({field:f,positions:p,radius,face,domainBox});assert.equal(invalid.supported,false);
    assert.ok(helper.branches.every(r=>!r.supported&&Number.isNaN(r.gap)));assert.throws(()=>helper.measure({}),/fresh supported/);
});

import assert from 'node:assert/strict';
import test from 'node:test';
import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames} from '../src/physics/kirchhoffCompositeElement.js';
import {createContactResult} from '../src/physics/collision/vesselContactField.js';
import {createCompositeJointWallRows} from '../src/physics/kirchhoffCompositeJointWallRows.js';
import {createCompositeJointWallFrictionRows as create,createCompositeJointWallFrictionWorkspace as workspace} from '../src/physics/kirchhoffCompositeJointWallFrictionRows.js';
const close=(a,b,t=1e-9)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b} (${t})`);
const vec=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const tol={force:1e-7,wallGap:1e-8,wallNcp:1e-8,wallWork:1e-8,frictionSlip:1e-8,frictionCone:1e-8,frictionWork:1e-8,frictionEquation:1e-9,linearConstraint:1e-10};
function plane() {
    return {calls:0,queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out=createContactResult()) {
        this.calls++;const t=ay<=by?0:1,p=t===0?[ax,ay,az]:[bx,by,bz];
        out.signedDistance=p[1];out.signedGap=p[1]-radius;out.segmentT=t;out.inward.values.set([0,1,0]);out.closestPoint.values.set([p[0],0,p[2]]);
        out.source='analytic-plane';out.faceIndex=0;out.capsuleSampleCount=2;return out;
    }};
}
function fixture({two=false,single=false,horizontal=false,mu=[.3,.6],semantic=17,state:given=null,field=plane(),rate=0,penalty=5,owner='catheter',positions=null,source='analytic-plane',radius=.25}={}) {
    const dt=.02,ids=single?['wire']:['wire','catheter'],layout=createCompositeChainLayout([ids,ids]),coordinates=[0,2,4];
    const points=new Map(ids.map(id=>[id,positions?structuredClone(positions):coordinates.map((x,i)=>[x,.25+(horizontal?0:.2*i),0])]));
    const modes=single?[]:coordinates.map((_,node)=>({node,basis:[[.8,.6,0],[-.6,.8,0],[0,0,1]],relativeDofs:[3*node,3*node+1,3*node+2]}));
    const state=given??{layout,coordinates,modes,relativeToolId:'wire',toolPositions:points,angles:new Map(ids.map(id=>[id,new Float64Array(2)])),
        relative:new Float64Array(3*modes.length),tools:[...points].map(([id,p])=>({id,reference:captureCompositeReferenceFrames(p),dsDx:1}))};
    const candidate=structuredClone(state),wall={mode:'wall-coulomb',chartId:'wall-friction-rows',field,forcePerLength:10,contactMode:'capsule',plane:{normal:[0,1,0],offset:0},
        friction:{law:'coulomb',mu,forcePerLength:penalty,materialPath:'linear-affine-maps',motion:'stationary-material',source,tangentBasis:'projected-own-tangent'},
        contactOwners:{edges:[{edge:0,wall:{owner:single?'wire':owner,radius,materialSegmentId:semantic}},
            {edge:1,wall:two?{owner:'wire',radius:.25,materialSegmentId:'wire:1'}:null}]}};
    const normal=createCompositeJointWallRows({...state,wall:{...wall,mode:'wall-normal',friction:'none'},tolerances:tol});
    normal.surfaceRecords.forEach(s=>normal.normalForces[s.base]=2);
    const prepared={dt,previousPositions:structuredClone(state.toolPositions),inertiaEdges:layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,
        materialMap:{sStart:20+2*e+(id==='catheter'?10:0)+dt*rate,dsDx:1,dsDt:rate}}))}))};
    return {state,candidate,wall,normal,prepared,dt,tolerances:tol,normalRowOffset:3,frictionRowOffset:7,queries:0};
}
function normalRefresh(f,order='full') {return f.normal.refresh({toolPositions:f.candidate.toolPositions,commonResidual:new Float64Array(f.state.layout.dofCount),relativeResidual:new Float64Array(f.candidate.relative.length),order,consumeQuery:()=>f.queries++});}
function ready(f,extra={}) {const m=create({...f,...extra});normalRefresh(f);m.prepare({toolPositions:f.candidate.toolPositions});return m;}
function refresh(f,m,{order='full',common=new Float64Array(f.state.layout.dofCount),relative=new Float64Array(f.candidate.relative.length),normal=true}={}) {
    if(normal)normalRefresh(f,order);
    return {certificate:m.refresh({toolPositions:f.candidate.toolPositions,commonResidual:common,relativeResidual:relative,order}),common,relative};
}
function loadSliding(f,m) {
    const first=refresh(f,m).certificate;
    first.samples.forEach((s,i)=>{const d=Math.hypot(...s.slip.map((v,k)=>s.mu[k]*v));
        for(let k=0;k<2;k++)m.tractions[2*i+k]=d===0?0:-s.Fn*s.mu[k]**2*s.slip[k]/d;
    });return refresh(f,m);
}
function perturb(f,row,column,h) {
    const L=f.state.layout;
    if(column<row.commonDofs.length) {
        const d=row.commonDofs[column],node=Array.from(L.positions).findIndex(v=>d>=v&&d<v+3);
        if(node>=0)for(const p of f.candidate.toolPositions.values())p[node][d-L.positions[node]]+=h;
        else for(const [id,spins] of L.spins){const edge=Array.from(spins).indexOf(d);if(edge>=0)f.candidate.angles.get(id)[edge]+=h;}
    } else {
        const d=row.relativeDofs[column-row.commonDofs.length],m=f.state.modes.find(m=>m.relativeDofs.includes(d)),axis=m.relativeDofs.indexOf(d);
        m.basis[axis].forEach((v,k)=>f.candidate.toolPositions.get('wire')[m.node][k]+=h*v);
    }
}
function snapshotRows(m) {return m.rows.map(r=>({jacobian:Array.from(r.jacobian),forceColumn:Array.from(r.forceColumn),H:Array.from(r.geometricTangent),residual:r.residual,
    diagonal:r.multiplierDerivative,cross:Array.from(r.multiplierJacobian)}));}

function ownerFrictionFixture({horizontal=true}={}){
    const f=fixture({two:true,horizontal});delete f.wall.friction.mu;
    f.wall.friction.muByOwner=[{owner:'wire',mu:[.006,.004]},{owner:'catheter',mu:[.002,.001]}];return f;
}

test('incoming material surface rates use the original wall lever and preserve rolling rest, sliding and pure normal motion',()=>{
    for(const [velocity,omega,expected] of [
        [[0,0,0],[4,0,0],'kinetic'], // rotation about x at y=.25 gives surface vz=-1
        [[0,0,1],[4,0,0],'static'],  // exact rolling cancellation at the wall
        [[0,2,0],[0,0,0],'static'],  // wall-normal motion is not tangent slip
        [[1e-30,0,1],[4,0,0],'kinetic'], // do not round a tiny axial rate to rest
    ]) {
        const f=fixture({horizontal:true});delete f.wall.friction.mu;
        Object.assign(f.wall.friction,{law:'coulomb-static-kinetic',muStatic:[.3,.6],muKinetic:[.1,.2],incomingSurfaceMotion:[{
            owner:'catheter',edge:0,interpretation:'physical-material-surface-velocity',materialLabels:[30,32],
            centerVelocities:[velocity.slice(),velocity.slice()],angularVelocity:omega.slice()}]});
        const m=ready(f),calls=f.wall.field.calls;
        const result=refresh(f,m,{normal:false});assert.equal(result.certificate.samples[0].mode,expected);assert.equal(f.wall.field.calls,calls);
        f.wall.friction.incomingSurfaceMotion[0].angularVelocity.fill(999);
        assert.equal(refresh(f,m,{normal:false}).certificate.samples[0].mode,expected,'prepared rates own their buffers');
    }
});

test('incoming material velocity is evaluated at the old label and old center, not the deformed spatial slot',()=>{
    const f=fixture({horizontal:true});delete f.wall.friction.mu;
    Object.assign(f.wall.friction,{law:'coulomb-static-kinetic',muStatic:[.3,.6],muKinetic:[.1,.2],incomingSurfaceMotion:[{
        owner:'catheter',edge:0,interpretation:'physical-material-surface-velocity',materialLabels:[30,32],
        centerVelocities:[[0,0,1],[0,0,1]],angularVelocity:[4,0,0]}]});
    // Move the current center outward. The old same-label center remains at
    // y=.25; using the new y=.5 would falsely turn rolling rest into sliding.
    f.candidate.toolPositions.get('catheter').forEach(p=>p[1]=.5);
    const m=ready(f);assert.equal(refresh(f,m).certificate.samples[0].mode,'static');
});

test('incoming material-rate records reject missing angular data, nonfinite rates and another material interval',()=>{
    for(const change of [m=>delete m.angularVelocity,m=>m.angularVelocity[2]=NaN,m=>m.centerVelocities[1][2]=Infinity,
        m=>m.materialLabels=[30,30],m=>m.materialLabels=[31,33],m=>m.interpretation='grid-velocity']) {
        const f=fixture({horizontal:true});delete f.wall.friction.mu;
        const motion={owner:'catheter',edge:0,interpretation:'physical-material-surface-velocity',materialLabels:[30,32],
            centerVelocities:[[0,0,0],[0,0,0]],angularVelocity:[0,0,0]};change(motion);
        Object.assign(f.wall.friction,{law:'coulomb-static-kinetic',muStatic:[.3,.6],muKinetic:[.1,.2],incomingSurfaceMotion:[motion]});
        assert.throws(()=>ready(f),/finite|increasing|material interval|interpretation/);
    }
});

test('a moved material label samples incoming endpoint rates at its old fraction during same-edge feed',()=>{
    const f=fixture({horizontal:true,rate:50});delete f.wall.friction.mu;
    Object.assign(f.wall.friction,{law:'coulomb-static-kinetic',muStatic:[.3,.6],muKinetic:[.1,.2],incomingSurfaceMotion:[{
        owner:'catheter',edge:0,interpretation:'physical-material-surface-velocity',materialLabels:[30,32],
        centerVelocities:[[-2,0,0],[0,0,0]],angularVelocity:[0,0,4]}]});
    // dt=.02 moves the current proximal label to 31. Its old velocity is -1
    // at old fraction .5, cancelling the +1 surface velocity from omega_z.
    // Reading the spatial proximal slot (-2) would falsely select sliding.
    const m=ready(f);assert.equal(refresh(f,m).certificate.samples[0].mode,'static');
});

test('different exposed tool surfaces use their own Coulomb axes, exact loads and dissipated work',()=>{
    const f=ownerFrictionFixture(),m=ready(f);
    f.candidate.toolPositions.get('wire').forEach(p=>p[0]-=.02);f.candidate.toolPositions.get('catheter').forEach(p=>p[0]+=.03);
    const r=loadSliding(f,m);assert.equal(r.certificate.converged,true);
    for(const s of r.certificate.samples){
        const isWire=s.owner==='wire',mu=isWire?[.006,.004]:[.002,.001];vec(s.mu,mu,0);
        vec(s.slip,[isWire?-.02:.03,0],2e-14);vec(s.traction,[isWire?.012:-.004,0],1e-14);
        close(s.work,(isWire?-.02:.03)*s.traction[0],1e-14);assert.ok(s.work<0);
        vec(m.nodalForces.get(s.owner).reduce((sum,p)=>sum.map((v,k)=>v+p[k]),[0,0,0]),[s.traction[0],0,0],1e-14);
    }
    const calls=f.wall.field.calls,saved=snapshotRows(m),history=m.commit();
    assert.equal(history.mu,undefined);assert.deepEqual(history.muByOwner,[{owner:'catheter',mu:[.002,.001]},{owner:'wire',mu:[.006,.004]}]);
    assert.deepEqual(refresh(f,m,{order:'gradient',normal:false}),r);assert.equal(f.wall.field.calls,calls);
    assert.deepEqual(refresh(f,m),r);assert.deepEqual(snapshotRows(m),saved);
    history.muByOwner[0].mu[0]=999;assert.equal(m.commit().muByOwner[0].mu[0],.002);
});

test('per-owner normal derivatives and full physical configuration derivatives match finite differences',()=>{
    const f=ownerFrictionFixture({horizontal:false}),m=ready(f);f.candidate.toolPositions.get('wire')[1][2]+=.001;f.candidate.angles.get('catheter')[0]=.03;
    f.candidate.toolPositions.get('catheter').forEach(p=>p[0]+=.01);refresh(f,m);
    const rows=snapshotRows(m),geometry=structuredClone(f.candidate.toolPositions),angles=structuredClone(f.candidate.angles),h=1e-6;
    for(let ri=0;ri<m.rows.length;ri++){
        const row=m.rows[ri],base=rows[ri];
        for(let j=0;j<row.jacobian.length;j++){
            const values=[];for(const sign of [-1,1]){f.candidate.toolPositions=structuredClone(geometry);f.candidate.angles=structuredClone(angles);perturb(f,row,j,sign*h);refresh(f,m);values.push(row.residual);}
            close((values[1]-values[0])/(2*h),base.jacobian[j],4e-6);
        }
        f.candidate.toolPositions=structuredClone(geometry);f.candidate.angles=structuredClone(angles);
        const slot=f.normal.surfaceRecords[Math.floor(ri/2)].base,Fn=f.normal.normalForces[slot],values=[];
        for(const sign of [-1,1]){f.normal.normalForces[slot]=Fn+sign*h;refresh(f,m);values.push(row.residual);}
        f.normal.normalForces[slot]=Fn;close((values[1]-values[0])/(2*h),base.cross[0],4e-8);
    }
});

test('per-owner history preserves coefficients across manager reuse and ignores only semantic list order',()=>{
    const f=ownerFrictionFixture(),w=workspace(),m=ready(f,{workspace:w});m.tractions.set([.001,0,.002,0]);refresh(f,m);
    const saved=m.commit(),next=structuredClone(f.state);next.wallFrictionState=saved;
    const second=fixture({two:true,horizontal:true,state:next,field:f.wall.field});delete second.wall.friction.mu;
    second.wall.friction.muByOwner=[{owner:'catheter',mu:[.002,.001]},{owner:'wire',mu:[.006,.004]}];
    const n=ready(second,{workspace:w});assert.deepEqual(Array.from(n.tractions),[.001,0,.002,0]);refresh(second,n);assert.deepEqual(n.commit(),saved);
    second.wall.friction.muByOwner.reverse();assert.doesNotThrow(()=>refresh(second,n));
    second.wall.friction.muByOwner[0].mu[0]+=.001;assert.throws(()=>refresh(second,n),/policy|provenance|configuration/i);
});

test('incomplete, mixed and unknown surface coefficient declarations reject instead of borrowing another tool law',()=>{
    for(const change of [f=>f.wall.friction.muByOwner.pop(),f=>f.wall.friction.muByOwner[0].owner='other',
        f=>f.wall.friction.muByOwner[0].owner='catheter',f=>f.wall.friction.mu=[.1,.1],f=>f.wall.friction.muByOwner[0].mu[0]=-1,
        f=>f.wall.friction.muByOwner[0].mu[1]=NaN]){
        const f=ownerFrictionFixture();change(f);assert.throws(()=>create(f),/coefficients|muByOwner/);
    }
    const f=fixture(),m=ready(f);f.wall.friction.muByOwner=[{owner:'catheter',mu:[.1,.1]}];assert.throws(()=>refresh(f,m),/declaration changed/);
});

test('coincident chart intervals keep both physical wall surfaces, separate Fn/Ft and own spin loads',()=>{
    for(const nodal of [false,true]) {
        const f=ownerFrictionFixture();
        f.wall.contactOwners.edges=[{edge:0,walls:[{owner:'catheter',radius:.25,materialSegmentId:'catheter:0'},
            {owner:'wire',radius:.25,materialSegmentId:17n}]},{edge:1,wall:null}];
        if(nodal) {
            f.wall.contactMode='nodal-endpoints';f.wall.pressureDiscretization='nodal-endpoints-one-sided-surface';
            f.wall.pressureSites=['catheter','wire'].flatMap(owner=>[{owner,edge:0,node:0,trace:'right'},{owner,edge:0,node:1,trace:'left'}]);
        }
        f.normal=createCompositeJointWallRows({...f.state,wall:{...f.wall,mode:'wall-normal',friction:'none'},tolerances:tol});
        f.normal.surfaceRecords.forEach(s=>f.normal.normalForces[s.base]=s.owner==='wire'?3:2);
        // Leave room for every normal row in the nodal chart.
        f.frictionRowOffset=10;
        const m=ready(f);f.candidate.toolPositions.get('wire').forEach(p=>p[0]-=.02);f.candidate.toolPositions.get('catheter').forEach(p=>p[0]+=.03);
        const result=loadSliding(f,m);assert.equal(result.certificate.converged,true);
        assert.equal(result.certificate.samples.length,nodal?4:2);assert.equal(f.normal.surfaceRecords.length,nodal?4:2);
        for(const s of result.certificate.samples){
            assert.equal(s.edge,0);assert.equal(s.Fn,s.owner==='wire'?3:2);
            vec(s.traction,[s.owner==='wire'?.018:-.004,0],1e-14);
            assert.equal(f.normal.surfaceRecords.find(v=>v.key===s.key).representative,null);
        }
        assert.equal(m.commit().tractions.length,nodal?8:4);
        f.wall.contactOwners.edges[0].walls.pop();assert.throws(()=>refresh(f,m),/ownership|chart|provenance/);
    }
});

test('multiple wall surfaces cannot duplicate, hide or borrow an owner',()=>{
    for(const surfaces of [[{owner:'wire',radius:.25},{owner:'wire',radius:.25}],[null],[{owner:'other',radius:.25}],
        [{owner:'wire',radius:0}]]) {
        const f=fixture();f.wall.contactOwners.edges[0]={edge:0,walls:surfaces};
        assert.throws(()=>createCompositeJointWallRows({...f.state,wall:{...f.wall,mode:'wall-normal',friction:'none'},tolerances:tol}),/occupy|radius/);
    }
    const f=fixture();f.wall.contactOwners.edges[0].walls=[];
    assert.throws(()=>createCompositeJointWallRows({...f.state,wall:{...f.wall,mode:'wall-normal',friction:'none'},tolerances:tol}),/never both/);
});


test('original wall records map ordinary Fn slots into two cross-coupled rows and scatter exact physical loads/own spin once',()=>{
    const f=fixture({two:true,horizontal:true}),before=structuredClone(f.state),w=workspace(),m=ready(f,{workspace:w});
    m.tractions.set([.1,-.2,-.08,.15]);const r=refresh(f,m);assert.equal(r.certificate.converged,true);assert.deepEqual(f.state,before);
    assert.equal(m.rows.length,4);assert.equal(m.tractions.length,4);assert.equal(m.diagnostics.preparationQueries,0);assert.equal(f.queries,4);
    assert.deepEqual(m.rows.map(r=>Array.from(r.multiplierDofs)),[[3,8],[3,7],[4,10],[4,9]]);
    for(const [i,id,edge] of [[0,'catheter',0],[1,'wire',1]]) {
        const F=m.nodalForces.get(id).reduce((sum,v)=>sum.map((x,k)=>x+v[k]),[0,0,0]);vec(F,[m.tractions[2*i],0,m.tractions[2*i+1]],1e-14);
        close(m.spinTorques.get(id)[edge],-.25*m.tractions[2*i+1],1e-14);
        close(r.common[f.state.layout.spins.get(id)[edge]],-m.spinTorques.get(id)[edge],1e-14);
    }
    for(const s of r.certificate.samples){close(s.work,0,0);close(s.minimumWork,0,0);close(s.wallWork,0,0);vec(s.equationResidual,[0,0],1e-15);}
    const history=m.commit(),owned=structuredClone(history);m.tractions.fill(0);assert.deepEqual(history,owned);
    assert.equal(w.diagnostics.surfaceWorkspaceBuilds,1);assert.equal(w.diagnostics.equationWorkspaceBuilds,1);
});

test('stationary wall axial sliding has exact displacement, force and dissipated work without any previous wall query',()=>{
    const f=fixture({horizontal:true}),m=ready(f);f.candidate.toolPositions.get('catheter').forEach(p=>p[0]+=.02);
    const r=loadSliding(f,m),s=r.certificate.samples[0];assert.equal(r.certificate.converged,true);vec(s.slip,[.02,0],1e-14);vec(s.traction,[-.6,0],1e-13);
    close(s.work,-.012,1e-14);close(s.minimumWork,-.012,1e-14);close(s.workGap,0,1e-14);close(s.wallWork,0,0);
    assert.equal(f.queries,3);assert.equal(f.wall.field.calls,3);assert.equal(m.diagnostics.preparationQueries,0);m.commit();
});

test('full wall operators retain independent G/B/DB and all normal/own/other traction derivatives under actual common/relative perturbations',()=>{
    for(const single of [true,false]) {
    const f=fixture({single,owner:'wire'}),m=ready(f);f.candidate.toolPositions.get('wire').forEach(p=>p[0]+=.02);f.candidate.angles.get('wire')[0]=.3;
    const base=loadSliding(f,m);assert.equal(base.certificate.converged,true);const rows=snapshotRows(m),support=m.rows[0],size=support.jacobian.length,h=1e-6;
    assert.ok(Math.abs(rows[0].cross[0])>1e-5);assert.ok(Math.abs(rows[0].cross[1])>1e-5);
    const geometry=structuredClone(f.candidate.toolPositions),angles=structuredClone(f.candidate.angles);
    const mechanical=r=>[...Array.from(support.commonDofs,d=>r.common[d]),...Array.from(support.relativeDofs,d=>r.relative[d])];
    for(let col=0;col<size;col++) {
        const result=[];
        for(const sign of [-1,1]){f.candidate.toolPositions=structuredClone(geometry);f.candidate.angles=structuredClone(angles);perturb(f,support,col,sign*h);
            const r=refresh(f,m);result.push({equation:m.rows.map(r=>r.residual),force:mechanical(r)});}
        for(let c=0;c<2;c++)close((result[1].equation[c]-result[0].equation[c])/(2*h),rows[c].jacobian[col],3e-6);
        for(let i=0;i<size;i++)close((result[1].force[i]-result[0].force[i])/(2*h),rows[0].H[i*size+col]+rows[1].H[i*size+col],3e-6);
    }
    f.candidate.toolPositions=geometry;f.candidate.angles=angles;refresh(f,m);
    for(let source=0;source<3;source++) {
        const values=source===0?f.normal.normalForces:m.tractions,index=source===0?0:source-1,old=values[index],answers=[];
        for(const sign of [-1,1]){values[index]=old+sign*h;refresh(f,m);answers.push(m.rows.map(r=>r.residual));}values[index]=old;
        for(let c=0;c<2;c++)close((answers[1][c]-answers[0][c])/(2*h),source===0?rows[c].cross[0]:source-1===c?rows[c].diagonal:rows[c].cross[1],3e-7);
    }
    }
});

test('value trials equal full values and loads bit for bit, revoke derivatives and permit original certified commit',()=>{
    const f=fixture({two:true}),m=ready(f);f.candidate.angles.get('catheter')[0]=.3;f.candidate.angles.get('wire')[1]=-.2;
    loadSliding(f,m);const full=refresh(f,m),loads=structuredClone(m.nodalForces),spins=structuredClone(m.spinTorques),rows=snapshotRows(m);
    const value=refresh(f,m,{order:'gradient'});assert.deepEqual(value,full);assert.deepEqual(m.nodalForces,loads);assert.deepEqual(m.spinTorques,spins);m.commit();
    for(const r of m.rows){assert.equal(r.geometricTangentValid,false);assert.ok(r.jacobian.every(Number.isNaN));assert.ok(r.geometricTangent.every(Number.isNaN));}
    refresh(f,m);assert.deepEqual(snapshotRows(m),rows);
});

test('negative signed Fn and infeasible zero-load Ft stay unclamped and never accept through forged diagnostic certificates',()=>{
    const f=fixture(),m=ready(f);f.candidate.angles.get('catheter')[0]=.2;m.tractions.set([.2,-.1]);
    for(const Fn of [-1e-30,0]) {f.normal.normalForces[0]=Fn;const c=refresh(f,m).certificate;assert.equal(c.converged,false);assert.ok(Number.isFinite(c.lineSearchMerit));
        assert.equal(c.samples[0].Fn,Fn);vec(m.tractions,[.2,-.1],0);c.converged=true;assert.throws(()=>m.commit(),/unchanged certified/);}
    m.tractions.fill(0);assert.equal(refresh(f,m).certificate.converged,true);m.commit();
});

test('second surface failure publishes no partial residual, invalidates all rows/loads and retries identically',()=>{
    const f=fixture({two:true}),m=ready(f);m.tractions.set([.1,.05,.05,-.1]);const first=refresh(f,m),rows=snapshotRows(m),saved=structuredClone(m.commit());
    const common=new Float64Array(f.state.layout.dofCount).fill(7),relative=new Float64Array(9).fill(9);
    f.normal.surfaceRecords[1].raw.rawContact.source='bad-source';assert.throws(()=>refresh(f,m,{normal:false,common,relative}),/declared source/);
    assert.ok(common.every(v=>v===7));assert.ok(relative.every(v=>v===9));assert.ok([...m.nodalForces.values()].flat(2).every(v=>v===0));
    for(const row of m.rows){assert.ok(row.forceColumn.every(Number.isNaN));assert.equal(row.geometricTangentValid,false);}assert.throws(()=>m.commit(),/unchanged certified/);
    assert.deepEqual(refresh(f,m),first);assert.deepEqual(snapshotRows(m),rows);assert.deepEqual(m.commit(),saved);
});

test('normal private generation and physical state freshness gate prepare, refresh, restore and commit',()=>{
    const f=fixture(),m=create(f);assert.throws(()=>m.prepare(),/matching original wall query/);normalRefresh(f);m.prepare({toolPositions:f.candidate.toolPositions});
    refresh(f,m);const checkpoint=f.normal.checkpoint();f.normal.restore(checkpoint);assert.throws(()=>m.commit(),/matching original wall query/);
    assert.throws(()=>refresh(f,m,{normal:false}),/matching original wall query/);refresh(f,m);
    for(const mutate of [()=>f.candidate.angles.get('catheter')[0]+=.1,()=>f.candidate.toolPositions.get('catheter')[0][0]+=.01,()=>m.tractions[0]+=.001,()=>f.normal.normalForces[0]+=.1]) {
        const p=structuredClone(f.candidate.toolPositions),a=structuredClone(f.candidate.angles),ft=m.tractions.slice(),fn=f.normal.normalForces.slice();mutate();
        assert.throws(()=>m.commit(),/unchanged certified/);f.candidate.toolPositions=p;f.candidate.angles=a;m.tractions.set(ft);f.normal.normalForces.set(fn);refresh(f,m);
    }
    f.candidate.tools[0].reference[0].director[0]=123;refresh(f,m);m.commit();
    f.wall.field.sourceVersion=2;assert.throws(()=>refresh(f,m,{normal:false}),/Frozen wall source/);
});

test('own material maps/semantic ID/history survive dt and penalty changes while stale leases and changed physical policies reject',()=>{
    const f=fixture({horizontal:true,rate:.2}),w=workspace(),m=ready(f,{workspace:w});loadSliding(f,m);const h=m.commit();
    const next=structuredClone(f.candidate);next.wallFrictionState=h;
    const g=fixture({horizontal:true,rate:.2,state:next,field:f.wall.field,penalty:50});
    g.prepared.inertiaEdges.forEach(e=>e.tools.forEach(t=>t.materialMap.sStart+=g.dt*.2));const other=ready(g,{workspace:w});loadSliding(g,other);other.commit();
    assert.equal(w.diagnostics.surfaceWorkspaceBuilds,1);assert.throws(()=>m.commit(),/Stale/);assert.equal(other.diagnostics.preparationQueries,0);
    const changed=fixture({state:next,field:f.wall.field,semantic:'17'});assert.throws(()=>create(changed),/history changed/);
    const wrong=fixture({state:next,field:f.wall.field});wrong.wall.friction.mu=[.4,.6];assert.throws(()=>create(wrong),/history changed/);
    const bad=fixture({state:next,field:f.wall.field});assert.throws(()=>create(bad),/Derived old material map/);
});

test('wall seam/representative/dependent reductions and active extra normal slots reject without rewriting trial forces',()=>{
    for(const mutation of ['seam','representative','dependent','extra','structure']) {
        const f=fixture(),normal=f.normal,views=normal.surfaceRecords.map(v=>({...v}));
        const altered={...normal,surfaceRecords:views};if(mutation==='extra')normal.normalForces[1]=.01;
        else if(mutation==='structure')altered.rowForceIndices=[1];else views[0][mutation]=mutation==='representative'?0:{};
        const forces=normal.normalForces.slice();assert.throws(()=>{const m=create({...f,normal:altered});normalRefresh(f);m.prepare({toolPositions:f.candidate.toolPositions});},/capsule|pressure|cone force/);
        assert.deepEqual(normal.normalForces,forces);
    }
    const f=fixture();f.wall.contactMode='envelope';assert.throws(()=>create(f),/capsule/);
});

test('current source/sample policy changes and finite material label crossing are explicit failures without previous queries',()=>{
    const f=fixture(),m=ready(f);f.wall.friction.motion='moving';assert.throws(()=>refresh(f,m,{normal:false}),/Frozen wall friction/);
    const crossed=fixture({rate:-.2});normalRefresh(crossed);const a=create(crossed);assert.throws(()=>a.prepare({toolPositions:crossed.candidate.toolPositions}),/crossed|outside|span|hinge|material transport|affine edge/i);
    assert.equal(a.diagnostics.preparationQueries,0);assert.equal(crossed.queries,1);
    const bad=fixture();bad.prepared.inertiaEdges[0].tools[1].materialMap.dsDt=[0,1000];assert.throws(()=>create(bad),/Old material slope/);
});

const norm=a=>Math.hypot(...a),scale=(a,s)=>Array.from(a,x=>x*s),sub=(a,b)=>a.map((x,k)=>x-b[k]);
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

for(const kind of ['sdf','bvh'])test(`${kind}: actual normal source routes current original operators and value loads with no extra query`,()=>{
    const field=kind==='sdf'?sdfField():bvhField(),positions=kind==='sdf'?[[-.625,-.625,.125],[.875,.875,.125],[2,2,.125]]:[[.8,-.7,.3],[1,-.3,.7],[2,.5,.8]];
    const f=fixture({single:true,field,positions,source:kind==='sdf'?'sparse-sdf':'sparse-sdf-bvh',radius:kind==='sdf'?2.2:.3}),m=ready(f);
    f.candidate.angles.get('wire')[0]=.2;f.candidate.toolPositions.get('wire')[0][0]+=.001;
    const full=loadSliding(f,m),forces=structuredClone(m.nodalForces),spins=structuredClone(m.spinTorques),calls=field.calls,bvhCalls=field.bvhQueries;
    assert.equal(full.certificate.converged,true);assert.equal(full.certificate.samples[0].surfaceWitness,kind==='sdf'?'provider-normal-projection-not-exact-isosurface':'original-geometric-wall-foot');
    const value=refresh(f,m,{normal:false,order:'gradient'});assert.deepEqual(value,full);assert.deepEqual(m.nodalForces,forces);assert.deepEqual(m.spinTorques,spins);
    assert.equal(field.calls,calls);assert.equal(field.bvhQueries,bvhCalls);assert.equal(m.diagnostics.preparationQueries,0);assert.ok(full.certificate.samples[0].work<0);m.commit();
    field.fallbackGeometry?.dispose();
});

test('one original field may provide SDF and BVH for different physical wall samples without changing either geometry law',()=>{
    const sdf=sdfField(),bvh=bvhField(),field={...sdf,fallbackGeometry:bvh.fallbackGeometry,bvhQueries:0,
        queryCapsuleCoordinates(...args){return (args[2]<.2?sdf:bvh).queryCapsuleCoordinates.call(this,...args);}};
    const f=fixture({two:true,field,source:'original-field',positions:[[-.625,-.625,.125],[.875,.875,.125],[2,2,.125]],radius:2.2});
    f.state.toolPositions.set('wire',[[.6,-1,.2],[.8,-.7,.3],[1,-.3,.7]]);
    f.state.tools.find(t=>t.id==='wire').reference=captureCompositeReferenceFrames(f.state.toolPositions.get('wire'));
    f.candidate=structuredClone(f.state);f.prepared.previousPositions=structuredClone(f.state.toolPositions);f.wall.contactOwners.edges[1].wall.radius=.3;
    f.normal=createCompositeJointWallRows({...f.state,wall:{...f.wall,mode:'wall-normal',friction:'none'},tolerances:tol});
    f.normal.surfaceRecords.forEach(s=>f.normal.normalForces[s.base]=2);
    const m=ready(f);f.candidate.angles.get('wire')[1]=.02;f.candidate.angles.get('catheter')[0]=.03;
    const full=loadSliding(f,m);assert.equal(full.certificate.converged,true);
    assert.deepEqual(f.normal.surfaceRecords.map(s=>s.raw.source),['sparse-sdf','sparse-sdf-bvh']);
    assert.deepEqual(full.certificate.samples.map(s=>s.surfaceWitness),['provider-normal-projection-not-exact-isosurface','original-geometric-wall-foot']);
    const calls=field.calls,queries=field.bvhQueries;assert.deepEqual(refresh(f,m,{normal:false,order:'gradient'}),full);
    assert.equal(field.calls,calls);assert.equal(field.bvhQueries,queries);m.commit();
    f.normal.surfaceRecords[0].raw.rawContact.source='sparse-sdf-bvh';assert.throws(()=>refresh(f,m,{normal:false}),/declared source/);
    bvh.fallbackGeometry.dispose();
});

function nodalFixture({sharedTrace='right',single=true,horizontal=true}={}) {
    const f=fixture({single,owner:'wire',horizontal});
    f.wall.contactOwners.edges[1].wall={owner:'wire',radius:.25,materialSegmentId:'wire:1'};
    f.wall.contactMode='nodal-endpoints';f.wall.pressureDiscretization='nodal-endpoints-one-sided-surface';
    f.wall.pressureSites=[{owner:'wire',node:0,edge:0,trace:'right'},
        {owner:'wire',node:1,edge:sharedTrace==='right'?1:0,trace:sharedTrace},{owner:'wire',node:2,edge:1,trace:'left'}];
    f.normal=createCompositeJointWallRows({...f.state,wall:{...f.wall,mode:'wall-normal',friction:'none'},tolerances:tol});
    f.normal.normalForces.set([1,2,3]);f.frictionRowOffset=9;return f;
}

test('explicit nodal pressure gives one physical Fn/Ft at the shared node and its selected one-sided own spin',()=>{
    for(const sharedTrace of ['right','left']) {
        const f=nodalFixture({sharedTrace}),before=structuredClone(f.state),m=ready(f);m.tractions.set([0,0,.1,.2,0,0]);
        const result=refresh(f,m);assert.equal(result.certificate.converged,true);assert.deepEqual(f.state,before);
        assert.equal(f.normal.surfaceRecords.length,3);assert.equal(f.normal.normalForces.length,3);assert.deepEqual(f.normal.rowForceIndices,[0,1,2]);
        assert.deepEqual(m.rows.map(r=>Array.from(r.multiplierDofs)),[[3,10],[3,9],[4,12],[4,11],[5,14],[5,13]]);
        assert.deepEqual(result.certificate.samples.map(s=>s.node),[0,1,2]);assert.equal(result.certificate.samples[1].trace,sharedTrace);
        vec(m.nodalForces.get('wire').reduce((sum,v)=>sum.map((x,k)=>x+v[k]),[0,0,0]),[.1,0,.2],1e-14);
        const chosen=sharedTrace==='right'?1:0,other=1-chosen;close(m.spinTorques.get('wire')[chosen],-.05,1e-14);close(m.spinTorques.get('wire')[other],0,0);
        close(result.common[f.state.layout.spins.get('wire')[chosen]],.05,1e-14);
        assert.equal(m.diagnostics.preparationQueries,0);const history=m.commit();assert.equal(history.contactMode,'nodal-endpoints');
        assert.equal(history.pressureDiscretization,'nodal-endpoints-one-sided-surface');assert.equal(history.currentMaps.length,2);
    }
});

test('flat capsule minimum changes endpoints while original nodal pressure and all three finite Coulomb laws remain defined',()=>{
    const f=nodalFixture(),m=ready(f),p=f.candidate.toolPositions.get('wire');p.forEach(v=>v[0]+=.02);
    p[0][1]=.25;p[1][1]=.2501;p[2][1]=.2502;const first=loadSliding(f,m);
    const originalFirst=normalRefresh(f).originalInequalities.filter(s=>s.role==='capsule');assert.deepEqual(originalFirst.map(s=>s.endpointFraction),[0,0]);assert.ok(originalFirst.every(s=>s.pressureDof===false));assert.equal(first.certificate.converged,true);
    const firstProof=first.certificate.samples.map(s=>({node:s.node,Fn:s.Fn,trace:s.trace,forceIndex:s.forceIndex}));
    p[0][1]=.2502;p[1][1]=.2501;p[2][1]=.25;const second=loadSliding(f,m);
    const originalSecond=normalRefresh(f).originalInequalities.filter(s=>s.role==='capsule');assert.deepEqual(originalSecond.map(s=>s.endpointFraction),[1,1]);assert.ok(originalSecond.every(s=>s.pressureDof===false));assert.equal(second.certificate.converged,true);
    assert.deepEqual(second.certificate.samples.map(s=>({node:s.node,Fn:s.Fn,trace:s.trace,forceIndex:s.forceIndex})),firstProof);
    assert.deepEqual(f.normal.rowForceIndices,[0,1,2]);vec(f.normal.normalForces,[1,2,3],0);assert.equal(m.rows.length,6);
    assert.ok(second.certificate.samples.every(s=>s.work<=0));m.commit();
});

test('nodal selected endpoint uses row.t and explicit trace even when degenerate raw query fraction is different',()=>{
    const f=nodalFixture();const original=f.wall.field.queryCapsuleCoordinates;let rawFraction=1;
    f.wall.field.queryCapsuleCoordinates=function(...args){const out=original.apply(this,args);if(args[0]===args[3]&&args[1]===args[4]&&args[2]===args[5])out.segmentT=rawFraction;return out;};
    // Source identity is frozen from construction by the normal owner.
    f.normal=createCompositeJointWallRows({...f.state,wall:{...f.wall,mode:'wall-normal',friction:'none'},tolerances:tol});f.normal.normalForces.set([1,2,3]);
    const m=ready(f),record=f.normal.surfaceRecords[0];assert.equal(record.raw.t,0);assert.equal(record.raw.rawContact.segmentT,1);
    f.candidate.toolPositions.get('wire').forEach(v=>v[0]+=.02);const r=loadSliding(f,m);assert.equal(r.certificate.converged,true);
    r.certificate.samples.forEach(s=>vec(s.slip,[.02,0],2e-14));const history=m.commit();
    rawFraction=0;const toggled=refresh(f,m);assert.equal(record.raw.t,0);assert.equal(record.raw.rawContact.segmentT,0);assert.deepEqual(toggled,r);assert.deepEqual(m.commit(),history);
});

test('nodal full/value, local derivatives and scalar torques use the chosen shared-node edge within a full two-tool chart',()=>{
    const f=nodalFixture({single:false,horizontal:false}),m=ready(f);f.candidate.toolPositions.get('wire').forEach(p=>p[0]+=.02);
    f.candidate.angles.get('wire')[0]=.2;f.candidate.angles.get('wire')[1]=-.3;loadSliding(f,m);
    const full=refresh(f,m),forces=structuredClone(m.nodalForces),spins=structuredClone(m.spinTorques),saved=snapshotRows(m),support=m.rows[2],size=support.jacobian.length;
    const geometry=structuredClone(f.candidate.toolPositions),angles=structuredClone(f.candidate.angles),h=1e-6;
    const keys=row=>[...Array.from(row.commonDofs,d=>`c:${d}`),...Array.from(row.relativeDofs,d=>`r:${d}`)],selected=keys(support),allKeys=m.rows.map(keys);
    const expectedH=(i,j)=>saved.reduce((sum,row,index)=>{const a=allKeys[index].indexOf(selected[i]),b=allKeys[index].indexOf(selected[j]);return sum+(a>=0&&b>=0?row.H[a*allKeys[index].length+b]:0);},0);
    for(let col=0;col<size;col++) {
        const residuals=[],mechanics=[];
        for(const sign of [-1,1]){f.candidate.toolPositions=structuredClone(geometry);f.candidate.angles=structuredClone(angles);perturb(f,support,col,sign*h);const r=refresh(f,m);residuals.push(m.rows.slice(2,4).map(r=>r.residual));mechanics.push([...Array.from(support.commonDofs,d=>r.common[d]),...Array.from(support.relativeDofs,d=>r.relative[d])]);}
        for(let c=0;c<2;c++)close((residuals[1][c]-residuals[0][c])/(2*h),saved[2+c].jacobian[col],3e-6);
        for(let i=0;i<size;i++)close((mechanics[1][i]-mechanics[0][i])/(2*h),expectedH(i,col),3e-6);
    }
    f.candidate.toolPositions=geometry;f.candidate.angles=angles;const value=refresh(f,m,{order:'gradient'});assert.deepEqual(value,full);assert.deepEqual(m.nodalForces,forces);assert.deepEqual(m.spinTorques,spins);
    assert.ok(m.rows.every(r=>!r.geometricTangentValid&&r.jacobian.every(Number.isNaN)));m.commit();
});

test('nodal pressure rejects missing/duplicate sites, wrong one-sided owner, radius jumps and silent history reinterpretation',()=>{
    for(const alter of [f=>delete f.wall.pressureDiscretization,f=>f.wall.pressureSites.pop(),f=>f.wall.pressureSites.push({...f.wall.pressureSites[1]}),
        f=>f.wall.pressureSites[1].trace='left',f=>f.wall.pressureSites[1].owner='catheter',f=>f.wall.contactOwners.edges[1].wall.radius=.3]) {
        const f=nodalFixture(),before=f.normal.normalForces.slice();alter(f);assert.throws(()=>create(f),/Nodal|nodal|one-sided|physical owned/);assert.deepEqual(f.normal.normalForces,before);
    }
    const f=nodalFixture(),m=ready(f);m.tractions.set([.1,0,.1,0,.1,0]);refresh(f,m);const history=m.commit();
    f.wall.pressureSites[1]={owner:'wire',node:1,edge:0,trace:'left'};assert.throws(()=>refresh(f,m,{normal:false}),/Frozen wall friction/);
    const changed=nodalFixture({sharedTrace:'left'});changed.state.wallFrictionState=history;assert.throws(()=>create(changed),/history changed/);
    const capsule=fixture({single:true});capsule.state.wallFrictionState=history;assert.throws(()=>create(capsule),/history changed/);
});

test('nodal second-site failure revokes every row and load, preserves original Fn/Ft/history and retries bit-identically',()=>{
    const f=nodalFixture(),m=ready(f);m.tractions.set([.1,.05,.05,-.1,.1,.1]);const full=refresh(f,m),saved=snapshotRows(m),history=structuredClone(m.commit()),fn=f.normal.normalForces.slice(),ft=m.tractions.slice();
    const common=new Float64Array(f.state.layout.dofCount).fill(7),relative=new Float64Array(0);f.normal.surfaceRecords[1].raw.rawContact.source='bad-source';
    assert.throws(()=>refresh(f,m,{normal:false,common,relative}),/declared source/);assert.ok(common.every(v=>v===7));assert.ok([...m.nodalForces.values()].flat(2).every(v=>v===0));
    assert.ok([...m.spinTorques.values()].every(v=>v.every(x=>x===0)));assert.deepEqual(f.normal.normalForces,fn);assert.deepEqual(m.tractions,ft);assert.throws(()=>m.commit(),/unchanged certified/);
    assert.deepEqual(refresh(f,m),full);assert.deepEqual(snapshotRows(m),saved);assert.deepEqual(m.commit(),history);
});

import {getCompositeJointWallFrictionEdgeId as poseEdgeId} from '../src/physics/kirchhoffCompositeJointWallFrictionRows.js';
import {createCompositeJointSurfacePoseHistory as poseHistory,prepareCompositeJointSurfacePosePath as preparePose} from '../src/physics/kirchhoffCompositeJointSurfacePoseHistory.js';
function prepareFixturePose(f,{adjacent=true,reservoir=true,permuted=false,alterSource,alterPreparation}={}) {
    const owner=f.wall.contactOwners.edges[0].wall.owner,tool=f.state.tools.find(t=>t.id===owner),p=f.state.toolPositions.get(owner),count=adjacent?3:2;
    const semantic=edge=>edge===0?f.wall.contactOwners.edges[0].wall.materialSegmentId:`adjacent:${owner}:${edge}`;
    const maps=Array.from({length:count-1},(_,edge)=>f.prepared.inertiaEdges[edge].tools.find(t=>t.id===owner).materialMap);
    const source={toolId:owner,reservoirIdentity:reservoir?'explicit-reservoir:17':null,nodes:[...(reservoir?[{id:'external',position:sub(p[0],sub(p[1],p[0]))}]:[]),
        ...p.slice(0,count).map((position,node)=>({id:node,node,position:position.slice()}))],edges:[],hinges:[]};
    const accepted=maps.map((m,edge)=>{const dx=f.state.coordinates[edge+1]-f.state.coordinates[edge],rates=typeof m.dsDt==='number'?[m.dsDt,m.dsDt]:m.dsDt;
        return {edgeId:poseEdgeId(semantic(edge)),materialSegmentId:semantic(edge),edge,source:'accepted',nodeIds:[edge,edge+1],coordinates:f.state.coordinates.slice(edge,edge+2),
            labels:[m.sStart-f.dt*rates[0],m.sStart+m.dsDx*dx-f.dt*rates[1]],reference:structuredClone(tool.reference[edge]),angle:f.state.angles.get(owner)[edge]};});
    if(reservoir){const a=accepted[0],dx=a.coordinates[1]-a.coordinates[0],ds=a.labels[1]-a.labels[0];source.edges.push({edgeId:'external:pose',materialSegmentId:'external:17',source:'reservoir',nodeIds:['external',0],
        coordinates:[a.coordinates[0]-dx,a.coordinates[0]],labels:[a.labels[0]-ds,a.labels[0]],reference:structuredClone(a.reference),angle:a.angle});}
    source.edges.push(...accepted);for(let j=0;j<source.edges.length-1;j++)source.hinges.push({leftEdgeId:source.edges[j].edgeId,rightEdgeId:source.edges[j+1].edgeId,referenceTwist:0});
    const configurationColumns=[...Array.from({length:count},(_,node)=>[0,1,2].map(component=>({kind:'position',toolId:owner,node,component}))).flat(),...Array.from({length:count-1},(_,edge)=>({kind:'angle',toolId:owner,edge}))];
    if(permuted)configurationColumns.reverse();const col=(node,k)=>configurationColumns.findIndex(c=>c.kind==='position'&&c.node===node&&c.component===k),spin=edge=>configurationColumns.findIndex(c=>c.kind==='angle'&&c.edge===edge);
    const nodeBindings=source.nodes.map(n=>n.node===undefined?{nodeId:n.id,offset:[0,0,0],terms:[0,1,2].flatMap(k=>(adjacent?[[0,1],[1,1],[2,-1]]:[[0,2],[1,-1]]).map(([node,w])=>({column:col(node,k),weights:[0,1,2].map(axis=>axis===k?w:0)})))}:
        {nodeId:n.id,offset:[0,0,0],terms:[0,1,2].map(k=>({column:col(n.node,k),weights:[0,1,2].map(axis=>axis===k?1:0)}))});
    const angleBindings=source.edges.map(e=>({edgeId:e.edgeId,offset:0,terms:[{column:spin(e.source==='reservoir'&&adjacent?1:e.edge??0),weight:1}]}));
    const currentMaps=accepted.map((e,j)=>({edgeId:e.edgeId,labels:[maps[j].sStart,maps[j].sStart+maps[j].dsDx*(e.coordinates[1]-e.coordinates[0])],dsDt:maps[j].dsDt}));
    if(reservoir)currentMaps.unshift({edgeId:'external:pose',labels:[2*currentMaps[0].labels[0]-currentMaps[0].labels[1],currentMaps[0].labels[0]],dsDt:maps[0].dsDt});
    alterSource?.(source);
    const preparation={history:poseHistory(source),targetEdgeId:accepted[0].edgeId,dt:f.dt,configurationColumns,nodeBindings,angleBindings,currentMaps};alterPreparation?.(preparation);
    const path=preparePose(preparation);f.wall.surfacePosePaths=[{owner,edge:0,path}];return {path,source,preparation};
}
function feedFixture({state,field=plane(),dt=.02,penalty=5,adjacent=true,permuted=false,source='analytic-plane',positions,radius=.25,rate=-.3}={}) {
    const f=fixture({owner:'catheter',horizontal:true,rate,state,field,penalty,source,positions,radius});
    for(const s of [f.state,f.candidate])s.tools.forEach(t=>{t.referenceTwists??=new Float64Array(1);});
    f.dt=f.prepared.dt=dt;
    f.prepared.inertiaEdges.forEach((e,edge)=>e.tools.forEach(t=>{const prior=state?.wallFrictionState?.currentMaps.find(m=>m.id===t.id&&m.edge===edge)?.currentMap;
        t.materialMap.sStart=(prior?.sStart??20+2*edge+(t.id==='catheter'?10:0))+dt*rate;}));
    f.wall.contactMode='nodal-endpoints';f.wall.pressureDiscretization='nodal-endpoints-one-sided-surface';
    f.wall.pressureSites=[{owner:'catheter',node:0,edge:0,trace:'right'},{owner:'catheter',node:1,edge:0,trace:'left'}];
    f.normal=createCompositeJointWallRows({...f.state,wall:{...f.wall,mode:'wall-normal',friction:'none'},tolerances:tol});f.normal.normalForces.set([2,3]);f.frictionRowOffset=11;
    f.pose=prepareFixturePose(f,{adjacent,permuted});return f;
}

test('explicit prepared proximal catheter feed dsDt=-.3 gives two original nodal Coulomb laws, all columns and physical loads without extra queries',()=>{
    const f=feedFixture(),before=structuredClone(f.state),w=workspace(),m=ready(f,{workspace:w}),queries=f.wall.field.calls;
    const full=loadSliding(f,m);assert.ok(full.certificate.converged);assert.deepEqual(f.state,before);assert.equal(m.rows.length,4);
    full.certificate.samples.forEach(s=>{vec(s.slip,[.006,0],4e-14);vec(s.traction,[-s.Fn*.3,0],1e-14);assert.ok(s.work<0);});
    assert.deepEqual(m.rows.map(r=>Array.from(r.multiplierDofs)),[[3,12],[3,11],[4,14],[4,13]]);
    assert.equal(m.rows[0].commonDofs.length,11);assert.equal(m.rows[0].relativeDofs.length,0);
    vec(m.nodalForces.get('catheter').reduce((sum,p)=>sum.map((x,k)=>x+p[k]),[0,0,0]),[-1.5,0,0],2e-14);
    assert.ok([...m.nodalForces.get('wire')].flat().every(v=>v===0));const saved=structuredClone(full),loads=structuredClone(m.nodalForces),spins=structuredClone(m.spinTorques),calls=f.wall.field.calls;
    assert.deepEqual(refresh(f,m,{normal:false,order:'gradient'}),saved);assert.equal(f.wall.field.calls,calls);assert.deepEqual(m.nodalForces,loads);assert.deepEqual(m.spinTorques,spins);
    assert.equal(m.diagnostics.preparationQueries,0);assert.equal(w.diagnostics.surfaceWorkspaceBuilds,1);assert.ok(calls>queries);const history=m.commit();assert.equal(history.currentMaps.length,2);
    const missing=feedFixture();delete missing.wall.surfacePosePaths;const unprepared=create(missing);normalRefresh(missing);
    assert.throws(()=>unprepared.prepare({toolPositions:missing.candidate.toolPositions}),{code:'surface-material-transport-required'});
});

for(const kind of ['plane','sdf','bvh'])test(`${kind}: N-physical wall manager equations and mechanics differentiate all expanded columns with simultaneous bending/spin`,context=>{
    const field=kind==='sdf'?sdfField():kind==='bvh'?bvhField():plane(),positions=kind==='sdf'?[[-.625,-.625,.125],[.875,.875,.125],[2.375,2.375,.125]]:kind==='bvh'?[[.8,-.7,.3],[1,-.3,.7],[1.2,.1,1.1]]:undefined;
    const f=feedFixture({field,positions,source:kind==='plane'?'analytic-plane':kind==='sdf'?'sparse-sdf':'sparse-sdf-bvh',radius:kind==='sdf'?2.2:.25,permuted:true}),m=ready(f);
    const p=f.candidate.toolPositions.get('catheter');p[0][0]+=.002;p[1][2]+=.003;p[2][1]+=.004;f.candidate.angles.get('catheter').set([.2,-.1]);
    loadSliding(f,m);const base=refresh(f,m),saved=snapshotRows(m),support=m.rows[0],size=support.jacobian.length,geometry=structuredClone(f.candidate.toolPositions),angles=structuredClone(f.candidate.angles),h=1e-6;
    const extra=Array.from(support.commonDofs).findIndex(d=>d===f.state.layout.positions[2]+1);assert.ok(extra>=0);
    close(saved[0].forceColumn[extra],0,0);assert.ok(saved.some(row=>Math.abs(row.jacobian[extra])>1e-10));
    let maxEquation=0,maxMechanics=0;
    const mechanics=r=>[...Array.from(support.commonDofs,d=>r.common[d]),...Array.from(support.relativeDofs,d=>r.relative[d])];
    for(let j=0;j<size;j++){
        const results=[];for(const sign of [-1,1]){f.candidate.toolPositions=structuredClone(geometry);f.candidate.angles=structuredClone(angles);perturb(f,support,j,sign*h);const r=refresh(f,m);results.push({residual:m.rows.map(r=>r.residual),force:mechanics(r)});}
        for(let c=0;c<m.rows.length;c++){const error=Math.abs((results[1].residual[c]-results[0].residual[c])/(2*h)-saved[c].jacobian[j]);maxEquation=Math.max(maxEquation,error);assert.ok(error<4e-6,`${kind} equation ${c},${j}: ${error}`);}
        for(let i=0;i<size;i++){const error=Math.abs((results[1].force[i]-results[0].force[i])/(2*h)-saved.reduce((sum,row)=>sum+row.H[i*size+j],0));maxMechanics=Math.max(maxMechanics,error);assert.ok(error<4e-6,`${kind} mechanics ${i},${j}: ${error}`);}
    }
    f.candidate.toolPositions=geometry;f.candidate.angles=angles;assert.deepEqual(refresh(f,m),base);
    const calls=field.calls,bvhCalls=field.bvhQueries;assert.deepEqual(refresh(f,m,{normal:false,order:'gradient'}),base);assert.equal(field.calls,calls);assert.equal(field.bvhQueries,bvhCalls);
    m.commit();field.fallbackGeometry?.dispose();context.diagnostic(JSON.stringify({kind,columns:size,maxEquation,maxMechanics}));
});

test('prepared old OWN nodes, frames, angles, labels and semantic IDs must match incoming state; explicit external pose is required',()=>{
    const changes=[
        s=>{s.nodes.filter(n=>n.node!==undefined).forEach(n=>n.position[0]+=.01);},
        s=>{s.edges.filter(e=>e.source==='accepted').forEach(e=>{e.reference.director=e.reference.director.map(x=>-x);e.angle+=Math.PI;});s.edges[0].reference.director=s.edges[0].reference.director.map(x=>-x);s.edges[0].angle+=Math.PI;},
        s=>{s.edges.forEach(e=>e.angle+=.1);}
    ];
    for(const alterSource of changes){const f=feedFixture();prepareFixturePose(f,{alterSource});assert.throws(()=>create(f),/incoming own physical state/);}
    const winding=feedFixture();prepareFixturePose(winding,{alterSource:s=>{s.hinges[1].referenceTwist=2*Math.PI;}});assert.throws(()=>create(winding),/hinge winding/);
    const old=feedFixture();old.state.angles.get('catheter')[1]=.1;assert.throws(()=>create(old),/incoming own physical state/);
    const wrongMap=feedFixture();wrongMap.prepared.inertiaEdges[1].tools.find(t=>t.id==='catheter').materialMap.sStart+=.01;assert.throws(()=>create(wrongMap),/maps must match/);
    const wrongSlope=feedFixture();wrongSlope.prepared.inertiaEdges[1].tools.find(t=>t.id==='catheter').materialMap.dsDx+=.01;assert.throws(()=>create(wrongSlope),/maps must match/);
    const wrongDt=feedFixture();wrongDt.dt*=2;wrongDt.prepared.dt=wrongDt.dt;assert.throws(()=>create(wrongDt),/identity\/dt/);
    const cloned=feedFixture();cloned.wall.surfacePosePaths[0].path=structuredClone(cloned.pose.path);assert.throws(()=>create(cloned),/prepared own surface pose path/);
    for(const mutate of [f=>f.wall.surfacePosePaths.push(f.wall.surfacePosePaths[0]),f=>f.wall.surfacePosePaths[0].owner='wire',f=>f.wall.surfacePosePaths[0].edge=1,f=>f.wall.surfacePosePaths=()=>{}]){
        const f=feedFixture();mutate(f);assert.throws(()=>create(f));}
    const unsupported=feedFixture();prepareFixturePose(unsupported,{reservoir:false});const m=create(unsupported);normalRefresh(unsupported);assert.throws(()=>m.prepare({toolPositions:unsupported.candidate.toolPositions}),{code:'surface-material-transport-required'});
    assert.equal(poseEdgeId(17),'number:17');assert.equal(poseEdgeId('17'),'string:2:17');assert.equal(poseEdgeId(17n),'bigint:17');assert.notEqual(poseEdgeId(17),poseEdgeId('17'));
});

test('two manager dt preserve their own Fn/Ft history with a newly prepared path and reject changed transport policy or support',()=>{
    const w=workspace(),f=feedFixture(),m=ready(f,{workspace:w});f.candidate.toolPositions.get('catheter').forEach(p=>{p[0]+=.001;p[2]+=.002;});f.candidate.angles.get('catheter').fill(.1);
    loadSliding(f,m);const history=m.commit(),next=structuredClone(f.candidate);next.wallFrictionState=history;
    const g=feedFixture({state:next,field:f.wall.field,dt:.013,penalty:50}),other=ready(g,{workspace:w});assert.notEqual(g.pose.path,f.pose.path);assert.equal(other.signature,m.signature);assert.deepEqual(other.tractions,history.tractions);
    assert.deepEqual(g.normal.normalForces,f.normal.normalForces);assert.throws(()=>m.commit(),/Stale/);
    g.candidate.angles.get('catheter').fill(.2);const full=loadSliding(g,other);assert.ok(full.certificate.converged);other.commit();
    const stale=feedFixture({state:next,field:f.wall.field,dt:.013});stale.wall.surfacePosePaths=f.wall.surfacePosePaths;assert.throws(()=>create(stale),/identity\/dt/);
    const sameName=feedFixture({state:next,field:f.wall.field,dt:f.dt});sameName.wall.surfacePosePaths=f.wall.surfacePosePaths;assert.throws(()=>create(sameName),/incoming own physical state|history/);
    for(const options of [
        {alterSource:s=>{s.reservoirIdentity='other';}},
        {alterPreparation:p=>{p.nodeBindings[0].terms[0].weights[0]+=.01;}},
        {adjacent:false},
        {permuted:true}
    ]){const changed=feedFixture({state:next,field:f.wall.field,dt:.013});prepareFixturePose(changed,options);const ft=history.tractions.slice(),fn=changed.normal.normalForces.slice();assert.throws(()=>create(changed),/history changed/);assert.deepEqual(history.tractions,ft);assert.deepEqual(changed.normal.normalForces,fn);}
    const without=feedFixture({state:next,field:f.wall.field});delete without.wall.surfacePosePaths;assert.throws(()=>create(without),/history changed/);
});

test('path workspace reuse across successive dt retains one legacy scratch arena and no accumulating path cache',()=>{
    const w=workspace(),field=plane();let state,prior;
    for(let i=0;i<12;i++){
        const f=feedFixture({state,field,dt:i%2?.013:.02}),m=ready(f,{workspace:w});loadSliding(f,m);const history=m.commit();
        if(prior)assert.throws(()=>prior.commit(),/Stale/);state=structuredClone(f.candidate);state.wallFrictionState=history;prior=m;
        assert.equal(w.diagnostics.surfaceWorkspaceBuilds,1);assert.equal(w.diagnostics.equationWorkspaceBuilds,1);assert.equal(w.diagnostics.managers,i+1);
        assert.equal(history.currentMaps.length,2);assert.ok(!('surfacePosePaths' in history));
    }
});

test('expanded wall manager late failure, current extra-node/spin freshness and path replacement revoke commit and retry exactly',()=>{
    const f=feedFixture(),m=ready(f);f.candidate.angles.get('catheter').set([.2,-.1]);loadSliding(f,m);const full=refresh(f,m),rows=snapshotRows(m),history=structuredClone(m.commit()),ft=m.tractions.slice(),fn=f.normal.normalForces.slice();
    const common=new Float64Array(f.state.layout.dofCount).fill(7),relative=new Float64Array(f.candidate.relative.length).fill(9);
    f.normal.surfaceRecords[1].raw.rawContact.source='bad';assert.throws(()=>refresh(f,m,{normal:false,common,relative}),/declared source/);
    assert.ok(common.every(v=>v===7));assert.ok(relative.every(v=>v===9));assert.ok([...m.nodalForces.values()].flat(2).every(v=>v===0));assert.ok([...m.spinTorques.values()].every(a=>a.every(v=>v===0)));
    assert.ok(m.rows.every(row=>!row.geometricTangentValid&&row.jacobian.every(Number.isNaN)));assert.deepEqual(m.tractions,ft);assert.deepEqual(f.normal.normalForces,fn);assert.throws(()=>m.commit(),/unchanged certified/);
    assert.deepEqual(refresh(f,m),full);assert.deepEqual(snapshotRows(m),rows);assert.deepEqual(m.commit(),history);
    for(const mutate of [()=>f.candidate.toolPositions.get('catheter')[2][0]+=.1,()=>f.candidate.angles.get('catheter')[1]+=.1]){
        const p=structuredClone(f.candidate.toolPositions),a=structuredClone(f.candidate.angles);mutate();assert.throws(()=>m.commit(),/unchanged certified/);f.candidate.toolPositions=p;f.candidate.angles=a;assert.deepEqual(refresh(f,m),full);
    }
    const list=f.wall.surfacePosePaths,entry=list[0];prepareFixturePose(f);const replacement=f.wall.surfacePosePaths[0];f.wall.surfacePosePaths=list;list[0]=replacement;
    assert.throws(()=>refresh(f,m,{normal:false}),/contracts changed/);assert.throws(()=>m.commit(),/contracts changed/);list[0]=entry;assert.deepEqual(refresh(f,m),full);assert.deepEqual(m.commit(),history);
    for(const bad of [null,{}, {owner:'other',edge:9}]){list[0]=bad;assert.throws(()=>refresh(f,m,{normal:false}),/contracts changed/);list[0]=entry;assert.deepEqual(refresh(f,m),full);}
});

test('a declared catheter pose path coexists with unchanged seven-column wire surface rows and exact global dual offsets',()=>{
    const f=feedFixture();f.wall.contactOwners.edges[1].wall={owner:'wire',radius:.25,materialSegmentId:'wire:1'};
    f.prepared.inertiaEdges.forEach(e=>{const map=e.tools.find(t=>t.id==='wire').materialMap;map.sStart=20+2*f.prepared.inertiaEdges.indexOf(e);map.dsDt=0;});
    f.wall.pressureSites.push({owner:'wire',node:1,edge:1,trace:'right'},{owner:'wire',node:2,edge:1,trace:'left'});
    f.normal=createCompositeJointWallRows({...f.state,wall:{...f.wall,mode:'wall-normal',friction:'none'},tolerances:tol});f.normal.normalForces.set([2,3,1,1]);
    const m=ready(f);f.candidate.angles.get('catheter').set([.2,-.1]);f.candidate.angles.get('wire')[1]=.1;const r=loadSliding(f,m);assert.ok(r.certificate.converged);assert.equal(m.rows.length,8);
    assert.equal(m.rows[0].commonDofs.length,11);assert.equal(m.rows[4].commonDofs.length,7);assert.equal(m.rows[4].relativeDofs.length,6);
    assert.deepEqual(m.rows.map(row=>Array.from(row.multiplierDofs)),[[3,12],[3,11],[4,14],[4,13],[5,16],[5,15],[6,18],[6,17]]);
    const rows=snapshotRows(m),loads=structuredClone(m.nodalForces),spins=structuredClone(m.spinTorques);assert.deepEqual(refresh(f,m,{order:'gradient'}),r);assert.deepEqual(m.nodalForces,loads);assert.deepEqual(m.spinTorques,spins);
    refresh(f,m);assert.deepEqual(snapshotRows(m),rows);m.commit();
});

test('prepared N=7 entry works, and an expanded relative wire owner retains all common/relative equation derivatives',()=>{
    const single=feedFixture({adjacent:false}),one=ready(single);assert.ok(loadSliding(single,one).certificate.converged);assert.equal(one.rows[0].commonDofs.length,7);one.commit();
    const f=feedFixture();f.wall.contactOwners.edges[0].wall.owner='wire';f.wall.pressureSites.forEach(s=>s.owner='wire');prepareFixturePose(f,{permuted:true});
    f.normal=createCompositeJointWallRows({...f.state,wall:{...f.wall,mode:'wall-normal',friction:'none'},tolerances:tol});f.normal.normalForces.set([2,3]);
    const m=ready(f);f.candidate.angles.get('wire').set([.2,-.1]);f.candidate.toolPositions.get('wire')[2][1]+=.003;loadSliding(f,m);
    const saved=snapshotRows(m),support=m.rows[0],geometry=structuredClone(f.candidate.toolPositions),angles=structuredClone(f.candidate.angles),h=1e-6;
    assert.equal(support.commonDofs.length,11);assert.equal(support.relativeDofs.length,9);
    for(let j=0;j<support.jacobian.length;j++){const result=[];for(const sign of [-1,1]){f.candidate.toolPositions=structuredClone(geometry);f.candidate.angles=structuredClone(angles);perturb(f,support,j,sign*h);refresh(f,m);result.push(m.rows.map(r=>r.residual));}
        for(let c=0;c<m.rows.length;c++)close((result[1][c]-result[0][c])/(2*h),saved[c].jacobian[j],4e-6);}
    f.candidate.toolPositions=geometry;f.candidate.angles=angles;refresh(f,m);m.commit();
});

test('prepared exact endpoint labels select the accepted trace despite noninteger metric cancellation',()=>{
    const metric=Math.hypot(1,.002),f=feedFixture({rate:0,positions:[[0,.25,0],[2,.254,0],[4,.258,0]]});
    f.prepared.inertiaEdges.forEach((e,edge)=>{const t=e.tools.find(t=>t.id==='catheter');t.materialMap={sStart:20+2*edge*metric,dsDx:metric,dsDt:0};});
    const {path}=prepareFixturePose(f),label=20+2*metric;assert.notEqual((label-20)/(2*metric),1);
    const m=ready(f),full=refresh(f,m);assert.equal(full.certificate.converged,true);full.certificate.samples.forEach(s=>vec(s.slip,[0,0],2e-14));
    assert.deepEqual(refresh(f,m,{normal:false,order:'gradient'}),full);m.commit();
    assert.equal(path.configurationDofs,11);assert.equal(m.diagnostics.preparationQueries,0);
});

test('fixed material points solve independent Coulomb laws at endpoints and interior without moving pressure ownership',()=>{
    const f=nodalFixture();
    f.wall.contactMode='material-points';f.wall.pressureDiscretization='fixed-material-points';f.wall.contactUpdate='current-query';
    f.wall.friction.rateMode='backward-euler-grid';f.wall.friction.slipModel='implicit-backward-euler-surface-rate';f.wall.friction.finiteStepSlipKnown=false;
    f.wall.pressureSites=[{owner:'wire',edge:0,fraction:0,trace:'right'},{owner:'wire',edge:0,fraction:1,trace:'left'},
        {owner:'wire',edge:1,fraction:1,trace:'left'},{owner:'wire',edge:0,fraction:.37}];
    f.normal=createCompositeJointWallRows({...f.state,wall:{...f.wall,mode:'wall-normal',friction:'none'},tolerances:tol});
    f.normal.normalForces.set([1,2,3,4]);f.frictionRowOffset=10;
    const m=ready(f);f.candidate.toolPositions.get('wire').forEach(p=>p[0]+=.02);
    const result=loadSliding(f,m);
    assert.equal(result.certificate.samples.length,4);
    assert.deepEqual(result.certificate.samples.map(s=>s.Fn),[1,2,3,4]);
    assert.ok(result.certificate.samples.every(s=>Math.hypot(...s.traction)>0));
    assert.equal(m.commit().contactMode,'material-points');
    const rows=snapshotRows(m),h=1e-6;
    for(let index=0;index<m.rows.length;index++){
        const row=m.rows[index];
        for(let column=0;column<row.commonDofs.length+row.relativeDofs.length;column++){
            perturb(f,row,column,h);refresh(f,m);const rp=m.rows[index].residual;
            perturb(f,row,column,-2*h);refresh(f,m);const rm=m.rows[index].residual;
            perturb(f,row,column,h);refresh(f,m);
            close((rp-rm)/(2*h),rows[index].jacobian[column],2e-6);
        }
    }
});

import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import {pathToFileURL} from 'node:url';

const root=process.env.OET_WALL_BVH_ROWS_ROOT;
const url=path=>root?pathToFileURL(`${root}/${path}`):new URL(`../${path}`,import.meta.url);
const {BufferGeometry,Float32BufferAttribute,Vector3}=await import(url('node_modules/three/build/three.module.js'));
const {MeshBVH}=await import(url('node_modules/three-mesh-bvh/src/index.js'));
const {STLLoader}=await import(url('node_modules/three/examples/jsm/loaders/STLLoader.js'));
const {VesselContactField,createContactResult}=await import(url('src/physics/collision/vesselContactField.js'));
const {decodeCollisionAsset}=await import(url('src/physics/collision/collisionAssetFormat.js'));
const {transformAortaGeometry}=await import(url('src/aortaTransform.js'));
const {generateVessel}=await import(url('src/vesselGeometry.js'));
const {createCompositeChainLayout,createCompositeChainWorkspace}=await import(url('src/physics/kirchhoffCompositeChain.js'));
const {createCompositeWallWorkspace,refreshCompositeWallContacts,assembleCompositeWallAugmented}=await import(url('src/physics/kirchhoffCompositeWallContacts.js'));
const {createCompositeWallEnvelopeWorkspace,refreshCompositeWallEnvelope,canonicalizeCompositeWallEnvelope}=await import(url('src/physics/kirchhoffCompositeWallEnvelope.js'));
const {differentiateCompositeWallRow,requireCompositeWallDifferentialRows}=await import(url('src/physics/kirchhoffCompositeWallDifferentialRows.js'));
const {createCompositeTimeStepState,advanceCompositeTimeStep}=await import(url('src/physics/kirchhoffCompositeTimeStep.js'));
const {captureCompositeReferenceFrames,compileCompositeMaterial}=await import(url('src/physics/kirchhoffCompositeElement.js'));
const close=(a,b,t=1e-8)=>assert.ok(Math.abs(a-b)<=t*Math.max(1,Math.abs(a),Math.abs(b)),`${a} != ${b}`);
const vectorClose=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const owners=(layout,radius)=>({edges:layout.edgeToolIds.map((_,edge)=>({edge,wall:{owner:'wire',radius}}))});
const raw=row=>({source:row.rawContact.source,gap:row.rawContact.signedGap,distance:row.rawContact.signedDistance,
    t:row.rawContact.segmentT,count:row.rawContact.capsuleSampleCount,face:row.rawContact.faceIndex,
    point:Array.from(row.rawContact.closestPoint.values),normal:Array.from(row.rawContact.inward.values)});

// A small actual MeshBVH, with an explicit fixed three-sample capsule policy.
// The collector must preserve that policy; only this fixture owns the queries.
function triangleField(vertices=[[0,0,0],[2,0,0],[0,2,0]]) {
    const geometry=new BufferGeometry();geometry.setAttribute('position',new Float32BufferAttribute(vertices.flat(),3));
    geometry.boundsTree=new MeshBVH(geometry);
    const point=new Vector3(),target={point:new Vector3(),faceIndex:-1,distance:Infinity};
    return {fallbackGeometry:geometry,calls:0,bvhCalls:0,
        queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out=createContactResult()) {
            this.calls++;let best=Infinity;
            for(const t of [.5,0,1]){
                const p=[ax+(bx-ax)*t,ay+(by-ay)*t,az+(bz-az)*t];this.bvhCalls++;
                geometry.boundsTree.closestPointToPoint(point.fromArray(p),target);
                if(target.distance<best){
                    best=target.distance;out.signedDistance=best;out.signedGap=best-radius;
                    out.source='sparse-sdf-bvh';out.faceIndex=target.faceIndex;out.segmentT=t;
                    out.closestPoint.values.set(target.point.toArray());
                    out.inward.values.set(best>1e-8?p.map((v,i)=>(v-target.point.getComponent(i))/best):[0,0,1]);
                }
            }
            out.capsuleSampleCount=2;return out;
        }};
}
function collect(field,positions,radius,envelope=false) {
    const layout=createCompositeChainLayout(positions.slice(1).map(()=>['wire'])),contactOwners=owners(layout,radius);
    const w=(envelope?refreshCompositeWallEnvelope:refreshCompositeWallContacts)({field,positions,contactOwners},
        (envelope?createCompositeWallEnvelopeWorkspace:createCompositeWallWorkspace)(layout));
    return {w,layout,contactOwners};
}

test('capsule collector retains the selected BVH feature and independent edge G/DB with no extra queries',()=>{
    const field=triangleField(),positions=[[.8,-.7,.3],[1,-.3,.7]],radius=.3;
    const {w}=collect(field,positions,radius),row=w.rows[0],saved=raw(row),eps=1e-5;
    assert.equal(field.calls,1);assert.equal(w.queries,1);assert.equal(field.bvhCalls,3);
    assert.equal(row.derivativeUnavailable,false);assert.equal(row.derivativeSource,'sparse-sdf-bvh');
    assert.equal(w.differentialScratch.bvhCapsule.featureType,'edge');assert.equal(row.t,.5);
    vectorClose(row.gapJacobian,Array.from(row.forceColumn,v=>-v),0);
    for(let j=0;j<6;j++){
        const move=d=>positions.map((p,n)=>p.map((v,a)=>v+(3*n+a===j?d:0)));
        const plus=field.queryCapsuleCoordinates(...move(eps).flat(),radius),minus=field.queryCapsuleCoordinates(...move(-eps).flat(),radius);
        assert.equal(plus.segmentT,.5);assert.equal(minus.segmentT,.5);
        close(row.gapJacobian[j],(plus.signedGap-minus.signedGap)/(2*eps),1e-8);
        for(let i=0;i<6;i++)close(row.normalDerivative[6*i+j],.5*(plus.inward.values[i%3]-minus.inward.values[i%3])/(2*eps),1e-8);
    }
    assert.deepEqual(raw(row),saved);const calls=field.calls,bvhCalls=field.bvhCalls;
    w.scratch.closestPoint.values.fill(NaN);w.scratch.faceIndex=-1;w.scratch.inward.values.fill(NaN);
    field.queryCapsuleCoordinates=()=>{throw new Error('Repeated collector query');};
    field.fallbackGeometry.boundsTree.closestPointToPoint=()=>{throw new Error('Repeated BVH query');};
    differentiateCompositeWallRow({field,positions,row},w.differentialScratch);
    requireCompositeWallDifferentialRows(w,[2]);assert.equal(row.derivativeUnavailable,false);
    assert.equal(field.calls,calls);assert.equal(field.bvhCalls,bvhCalls);assert.deepEqual(raw(row),saved);
    field.fallbackGeometry.dispose();
});

test('BVH endpoint cache owns its face/foot and scatters point G/DB into the correct 3-to-6 block',()=>{
    const field=triangleField(),positions=[[.5,-.4,.6],[.8,-.4,.6],[1.1,-.4,.6]],radius=.8;
    const {w,layout,contactOwners}=collect(field,positions,radius,true),saved=w.rows.map(raw);
    assert.equal(field.calls,5);assert.equal(w.queries,5);assert.equal(field.bvhCalls,15);
    assert.deepEqual(raw(w.rows[1]),raw(w.rows[3]));
    assert.notEqual(w.rows[1].rawContact.closestPoint.values,w.rows[3].rawContact.closestPoint.values);
    assert.notEqual(w.rows[1].rawContact.closestPoint.values,w.endpointScratch.closestPoint.values);
    const d=Math.hypot(.4,.6),n=[0,-.4/d,.6/d];
    for(const row of w.rows.filter(row=>row.role!=='capsule')){
        const end=row.role==='distal'?1:0;assert.equal(row.t,end);assert.equal(row.rawContact.segmentT,.5);
        assert.equal(row.derivativeUnavailable,false);assert.equal(row.rawContact.faceIndex,row.faceIndex);
        vectorClose(row.rawContact.closestPoint.values,row.closestPoint,0);
        for(let i=0;i<6;i++){
            const used=Math.floor(i/3)===end;close(row.gapJacobian[i],used?n[i%3]:0);
            close(row.forceColumn[i],used?-n[i%3]:0);
            for(let j=0;j<6;j++){
                const a=i%3,b=j%3,H=((a===b?1:0)-(a===0&&b===0?1:0)-n[a]*n[b])/d;
                close(row.normalDerivative[6*i+j],used&&Math.floor(j/3)===end?H:0);
            }
        }
    }
    w.endpointScratch.closestPoint.values.fill(99);w.endpointScratch.faceIndex=99;
    w.scratch.closestPoint.values.fill(99);w.scratch.faceIndex=99;
    field.queryCapsuleCoordinates=()=>{throw new Error('Repeated endpoint query');};
    field.fallbackGeometry.boundsTree.closestPointToPoint=()=>{throw new Error('Repeated endpoint BVH query');};
    for(const row of w.rows)differentiateCompositeWallRow({field,positions,row},w.differentialScratch);
    assert.deepEqual(w.rows.map(raw),saved);requireCompositeWallDifferentialRows(w,new Float64Array(6));
    const multipliers=new Float64Array([1,2,3,4,5,6]);
    const total=()=>w.rows.reduce((sum,row)=>sum+multipliers[row.index],0),before=total();
    const representatives=canonicalizeCompositeWallEnvelope(w,multipliers,contactOwners);close(total(),before,0);
    assert.ok(representatives.includes(2)&&representatives.includes(5),'edge DB cross blocks prevent a false endpoint dependence');
    assert.equal(layout.nodeCount,3);assert.equal(field.calls,5);field.fallbackGeometry.dispose();
});

function stepFixture(contactMode='envelope',y=1) {
    const positions=[[1,y,.48],[2,y,.48],[3,y,.48]],layout=createCompositeChainLayout([['wire'],['wire']]);
    const data={positions,coordinates:[0,1,2],reference:captureCompositeReferenceFrames(positions),
        tools:[{id:'wire',angles:new Float64Array(2),material:compileCompositeMaterial({EI1:.01,GJ:.01})}]};
    const state=createCompositeTimeStepState({data,layout}),field=triangleField([[0,0,0],[20,0,0],[0,20,0]]),dt=1/120;
    const options={dt,torsionMode:'quasi-static',elementBackend:'wasm-exact',constraintSolver:'mixed',
        prescribed:[{dof:layout.spins.get('wire')[0],value:0},{dof:layout.positions[0],value:1},
            ...Array.from(layout.positions,dof=>({dof:dof+1,value:y}))],
        inertiaEdges:layout.edgeToolIds.map((_,edge)=>({tools:[{id:'wire',massPerMaterialLength:.01,
            materialMap:{sStart:edge,dsDx:1,dsDt:0},oldMaterialVelocities:[[0,0,0],[0,0,0]]}]})),
        tolerances:{force:1e-7,torque:1e-8,length:1e-8,linear:1e-10},initialPenalty:1e4,
        budget:{directions:20,outerIterations:20,evaluations:150,lineSearchTrials:15},
        wall:{field,contactMode,friction:'frictionless',initialPenalty:1e4,
            tolerances:{gap:1e-8,force:1e-7,work:1e-7},contactOwners:owners(layout,.5)}};
    return {state,field,options};
}

for(const mode of ['capsule','envelope'])test(`collector to mixed whole dt resolves triangle face contact (${mode}) with physical momentum`,t=>{
    const {state,field,options}=stepFixture(mode),before=structuredClone(state),r=advanceCompositeTimeStep(state,options);
    assert.ok(r.accepted,JSON.stringify({status:r.status,...r.diagnostics}));assert.deepEqual(state,before);
    assert.equal(r.state.time,options.dt);assert.equal(r.state.step,1);assert.equal(r.diagnostics.historyCommits,1);
    assert.equal(r.diagnostics.wallQueries,field.calls);assert.equal(field.bvhCalls,3*field.calls);
    assert.ok(r.diagnostics.certificate.wall.converged);assert.ok(r.diagnostics.directions>0);
    const force=[0,0,0],rate=[0,0,0];
    for(const record of r.state.wallContactState.records){
        assert.ok(record.history.normalForce>=0);close(Math.hypot(...record.worldForce),record.history.normalForce,1e-12);
        record.worldForce.forEach((v,i)=>force[i]+=v);
    }
    for(const edge of r.state.materialVelocities)for(const tool of edge.tools)
        for(let i=0;i<3;i++)rate[i]+=.01/options.dt*.5*(tool.velocities[0][i]+tool.velocities[1][i]);
    for(let i=0;i<3;i++){
        const support=Array.from(r.state.layout.positions).reduce((sum,dof)=>sum+r.diagnostics.reactions[dof+i],0);
        close(force[i]+support,rate[i],2e-7);
    }
    assert.ok(force[2]>.1);
    for(let edge=0;edge<2;edge++){
        const a=r.state.data.positions[edge],b=r.state.data.positions[edge+1];
        close(Math.hypot(...b.map((v,i)=>v-a[i])),1,1e-9);
        const rawContact=field.queryCapsuleCoordinates(...a,...b,.5);assert.ok(rawContact.signedGap>=-options.wall.tolerances.gap);
        assert.equal(rawContact.source,'sparse-sdf-bvh');
    }
    t.diagnostic(JSON.stringify({mode,directions:r.diagnostics.directions,forceResidual:r.diagnostics.certificate.force,physicalWallForce:force}));
    field.fallbackGeometry.dispose();
});

test('BVH physical AL remains explicitly disabled before assembly and whole-dt state mutation',()=>{
    const {state,field,options}=stepFixture('capsule'),before=structuredClone(state);
    const {w,layout}=collect(field,state.data.positions,.5),chain=createCompositeChainWorkspace(layout);
    const saved=[chain.energy,chain.gradient.slice(),chain.hessian.slice()];
    assert.throws(()=>assembleCompositeWallAugmented(w,chain,{lambdas:[1,1],penalty:10}),/analytic-plane only.*BVH/);
    assert.equal(chain.energy,saved[0]);assert.deepEqual(chain.gradient,saved[1]);assert.deepEqual(chain.hessian,saved[2]);
    const r=advanceCompositeTimeStep(state,{...options,constraintSolver:'augmented'});
    assert.equal(r.accepted,false);assert.equal(r.state,state);assert.equal(r.diagnostics.historyCommits,0);
    assert.match(r.diagnostics.error,/analytic-plane only.*BVH/);assert.deepEqual(state,before);
    field.fallbackGeometry.dispose();
});

test('ambiguous triangle boundaries retain raw penetration and reject active mixed rows',()=>{
    const {state,field,options}=stepFixture('envelope',0),before=structuredClone(state);
    const {w}=collect(field,state.data.positions,.5,true);assert.equal(field.calls,5);
    for(const row of w.rows){
        assert.equal(row.source,'sparse-sdf-bvh');close(row.gap,-.02,1e-12);
        assert.equal(row.derivativeUnavailable,true);assert.equal(row.derivativeReason,'nonclassical-feature-boundary');
        assert.ok(row.gapJacobian.every(Number.isNaN));assert.ok(row.forceColumn.every(Number.isNaN));assert.ok(row.normalDerivative.every(Number.isNaN));
    }
    assert.throws(()=>requireCompositeWallDifferentialRows(w,new Float64Array(6)),/Unsupported active\/loaded/);
    const r=advanceCompositeTimeStep(state,options);assert.equal(r.accepted,false);assert.equal(r.state,state);
    assert.match(r.diagnostics.error,/Unsupported active\/loaded/);assert.deepEqual(state,before);assert.equal(r.diagnostics.historyCommits,0);
    field.fallbackGeometry.dispose();
});

let anatomy;
function actualAnatomy(){
    if(anatomy)return anatomy;
    const buffer=name=>{const b=fs.readFileSync(url(`res/${name}`));return b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength);};
    const asset=decodeCollisionAsset(buffer('Aorta_plain.collision.bin')),geometry=new STLLoader().parse(buffer('Aorta_plain.stl'));
    const transform=transformAortaGeometry(geometry,generateVessel(140,0).vessel);
    close(transform.scale,asset.metadata.transform.scale,1e-12);vectorClose(transform.targetCenter,asset.metadata.transform.targetCenter,1e-12);
    geometry.boundsTree=new MeshBVH(geometry);
    const field=new VesselContactField(asset,{fallbackGeometry:geometry,bvhValidationDistance:.02,capsuleBvhValidation:-.1});
    anatomy={field,geometry};return anatomy;
}
for(const [feature,shift] of [['face',1],['edge',-4]])test(`actual anatomy ${feature}: raw collector G/DB matches independent provider differences`,t=>{
    const {field}=actualAnatomy(),positions=[[65.00287246704102,-462.2578430175781+shift,-79.56869888305664],
        [64.95548751831055,-462.0397204589844+shift,-79.65612350463867]],radius=.4445;
    const query=field.queryCapsuleCoordinates.bind(field);let calls=0;field.queryCapsuleCoordinates=(...args)=>{calls++;return query(...args);};
    const {w}=collect(field,positions,radius),row=w.rows[0],saved=raw(row),eps=1e-4;
    assert.equal(calls,1);assert.equal(w.queries,1);assert.equal(row.derivativeUnavailable,false);
    assert.equal(w.differentialScratch.bvhCapsule.featureType,feature);assert.equal(row.source,'sparse-sdf-bvh');
    vectorClose(row.closestPoint,row.rawContact.closestPoint.values,0);assert.equal(row.faceIndex,row.rawContact.faceIndex);
    let maxG=0,maxDB=0;
    for(let j=0;j<6;j++){
        const move=d=>positions.map((p,n)=>p.map((v,a)=>v+(3*n+a===j?d:0)));
        const plus=query(...move(eps).flat(),radius,createContactResult()),minus=query(...move(-eps).flat(),radius,createContactResult());
        assert.equal(plus.source,row.source);assert.equal(minus.source,row.source);assert.equal(plus.segmentT,row.t);assert.equal(minus.segmentT,row.t);
        const gapFD=(plus.signedGap-minus.signedGap)/(2*eps);close(row.gapJacobian[j],gapFD,1e-6);maxG=Math.max(maxG,Math.abs(row.gapJacobian[j]-gapFD));
        for(let i=0;i<6;i++){
            const weight=i<3?1-row.t:row.t,normalFD=weight*(plus.inward.values[i%3]-minus.inward.values[i%3])/(2*eps);
            close(row.normalDerivative[6*i+j],normalFD,1e-6);maxDB=Math.max(maxDB,Math.abs(row.normalDerivative[6*i+j]-normalFD));
        }
    }
    assert.deepEqual(raw(row),saved);assert.equal(calls,1);requireCompositeWallDifferentialRows(w,[2]);assert.equal(calls,1);
    assert.equal(field.bvhValidationDistance,.02);assert.equal(field.capsuleBvhValidationGap,-.1);
    t.diagnostic(JSON.stringify({feature,faceIndex:row.faceIndex,gap:row.gap,t:row.t,maxG,maxDB}));
    field.queryCapsuleCoordinates=query;
});

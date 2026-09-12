import assert from 'node:assert/strict';
import test from 'node:test';
import fs from 'node:fs';
import {pathToFileURL} from 'node:url';
const root=process.env.OET_JOINT_WALL_ROWS_ROOT;
const url=p=>root?pathToFileURL(`${root}/${p}`):new URL(`../${p}`,import.meta.url);
const {createCompositeJointWallRows:create}=await import(url('src/physics/kirchhoffCompositeJointWallRows.js'));
const {createCompositeChainLayout}=await import(url('src/physics/kirchhoffCompositeChain.js'));
const {VesselContactField,createContactResult}=await import(url('src/physics/collision/vesselContactField.js'));
const {decodeCollisionAsset}=await import(url('src/physics/collision/collisionAssetFormat.js'));
const {BufferGeometry,Float32BufferAttribute,Vector3}=await import(url('node_modules/three/build/three.module.js'));
const {MeshBVH}=await import(url('node_modules/three-mesh-bvh/src/index.js'));
const close=(a,b,t=1e-9)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t*Math.max(1,Math.abs(a),Math.abs(b)),`${a} != ${b}`);
const vectorClose=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const zero=(n=3)=>new Array(n).fill(0);
const tolerances={force:1e-7,wallGap:1e-8,wallNcp:1e-8,wallWork:1e-7,linearConstraint:5e-11};
function fixture({edges=[['wire','catheter'],['wire']],positions=[[0,.3,0],[2,.6,0],[4,.1,0]],field=plane(),owners=null}={}) {
    const layout=createCompositeChainLayout(edges),coordinates=positions.map((_,i)=>2*i),nodes=positions.map((_,i)=>i).filter(i=>new Set([...(edges[i-1]??[]),...(edges[i]??[])]).size>1);
    const modes=nodes.map((node,i)=>({node,basis:[[.8,.6,0],[-.6,.8,0],[0,0,1]],relativeDofs:[3*i,3*i+1,3*i+2]}));
    const relative=Float64Array.from(modes.flatMap(()=>[.25,-.1,.07]));
    const wall={mode:'wall-normal',friction:'none',chartId:'joint-wall-test',field,forcePerLength:10,
        contactOwners:{edges:edges.map((ids,edge)=>({edge,wall:owners?owners[edge]:{owner:ids.includes('catheter')?'catheter':'wire',radius:edge===0?.3:.1}}))}};
    return {layout,coordinates,modes,relativeToolId:'wire',positions:structuredClone(positions),relative,wall,tolerances};
}
function physical(f) {
    const p=new Map([...f.layout.spins.keys()].map(id=>[id,structuredClone(f.positions)]));
    for(const m of f.modes)for(let a=0;a<3;a++)for(let k=0;k<3;k++)p.get('wire')[m.node][k]+=m.basis[a][k]*f.relative[m.relativeDofs[a]];
    return p;
}
function measure(adapter,f,{positions=physical(f),query=true,order='full',consumeQuery=()=>{},common=null,relative=null}={}) {
    const commonResidual=common??new Float64Array(f.layout.dofCount),relativeResidual=relative??new Float64Array(f.relative.length);
    const certificate=adapter.refresh({toolPositions:positions,commonResidual,relativeResidual,order,consumeQuery,query});
    return {certificate,common:commonResidual,relative:relativeResidual,positions};
}
function plane() {
    return {calls:[],source:'analytic-plane',queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out=createContactResult()) {
        this.calls.push([ax,ay,az,bx,by,bz,radius]);const t=ay<by?0:1,p=t===0?[ax,ay,az]:[bx,by,bz];
        out.signedDistance=p[1];out.signedGap=p[1]-radius;out.segmentT=t;out.inward.values.set([0,1,0]);out.closestPoint.values.set([p[0],0,p[2]]);
        out.source=this.source;out.faceIndex=0;out.capsuleSampleCount=2;return out;
    }};
}
const valley=([x,y,z])=>2+.5*(x-.125)*(y-.125)+.25*z;
const gradient=([x,y])=>[.5*(y-.125),.5*(x-.125),.25];
function smoothField() {
    const brickSize=2,dimensions=[3,3,3],origin=[-1,-1,-1],voxelSize=.5,q=1/1024,lookup=new Uint16Array(27),distances=new Uint32Array(216);
    for(let bz=0;bz<3;bz++)for(let by=0;by<3;by++)for(let bx=0;bx<3;bx++){
        const brick=bx+3*(by+3*bz);lookup[brick]=brick;
        for(let z=0;z<2;z++)for(let y=0;y<2;y++)for(let x=0;x<2;x++){
            const p=[origin[0]+voxelSize*(2*bx+x),origin[1]+voxelSize*(2*by+y),origin[2]+voxelSize*(2*bz+z)],v=valley(p)/q;
            assert.equal(v,Math.round(v));distances[8*brick+x+2*(y+2*z)]=v;
        }
    }
    return {sdfOrigin:origin,sdfDimensions:dimensions,brickSize,voxelSize,sdfQuantization:q,sdfBrickLookup:lookup,sdfDistances:distances,
        sdfInsideBits:new Uint8Array(27).fill(255),calls:0,queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out=createContactResult()) {
            this.calls++;const a=[ax,ay,az],b=[bx,by,bz];let selected=null;
            for(const t of [0,.5,1]){const p=a.map((v,i)=>v+t*(b[i]-v)),d=valley(p);if(selected===null||d<selected.d)selected={t,p,d};}
            const {t,p,d}=selected,G=gradient(p),m=Math.hypot(...G);out.signedDistance=d;out.signedGap=d-radius;out.segmentT=t;out.capsuleSampleCount=2;
            out.inward.values.set(G.map(v=>v/m));out.closestPoint.values.set(p.map((v,i)=>v-d*out.inward.values[i]));out.source='sparse-sdf';out.faceIndex=-1;return out;
        }};
}
function triangleField() {
    const geometry=new BufferGeometry();geometry.setAttribute('position',new Float32BufferAttribute([0,0,0,2,0,0,0,2,0],3));geometry.boundsTree=new MeshBVH(geometry);
    const point=new Vector3(),target={point:new Vector3(),faceIndex:-1,distance:Infinity};
    return {fallbackGeometry:geometry,calls:0,bvhCalls:0,queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out=createContactResult()) {
        this.calls++;let best=Infinity;
        for(const t of [.5,0,1]){const p=[ax+t*(bx-ax),ay+t*(by-ay),az+t*(bz-az)];this.bvhCalls++;
            geometry.boundsTree.closestPointToPoint(point.fromArray(p),target);
            if(target.distance<best){best=target.distance;out.signedDistance=best;out.signedGap=best-radius;out.source='sparse-sdf-bvh';out.faceIndex=target.faceIndex;out.segmentT=t;
                out.closestPoint.values.set(target.point.toArray());out.inward.values.set(p.map((v,i)=>(v-target.point.getComponent(i))/best));}}
        out.capsuleSampleCount=2;return out;
    }};
}

test('per-tool collectors use actual catheter overlap and exposed wire geometry with one physical force each',()=>{
    const f=fixture();Object.freeze(f.wall.field);const a=create(f);a.normalForces[0]=2;a.normalForces[3]=3;
    let queries=0;const r=measure(a,f,{consumeQuery:()=>queries++}),p=physical(f);
    assert.equal(queries,2);assert.equal(a.diagnostics.queries,2);assert.equal(f.wall.field.calls.length,2);assert.equal(r.certificate.converged,true);
    assert.deepEqual(f.wall.field.calls[0],p.get('wire')[1].concat(p.get('wire')[2],[.1]));
    assert.deepEqual(f.wall.field.calls[1],p.get('catheter')[0].concat(p.get('catheter')[1],[.3]));
    assert.notDeepEqual(p.get('wire')[1],p.get('catheter')[1]);
    vectorClose(a.nodalForces.get('catheter')[0],[0,2,0],0);vectorClose(a.nodalForces.get('wire')[2],[0,3,0],0);
    assert.ok(a.nodalForces.get('wire').slice(0,2).flat().every(v=>v===0));assert.ok(r.relative.every(v=>v===0));
    close(r.common[f.layout.positions[0]+1],-2,0);close(r.common[f.layout.positions[2]+1],-3,0);
    const saved=a.commit(),before=structuredClone(saved);a.normalForces[0]=4;assert.deepEqual(saved,before);assert.throws(()=>a.commit(),/fresh original query/);
});

test('surface consumers borrow the same original rows only after a fresh successful normal evaluation',()=>{
    const f=fixture(),a=create(f),p=physical(f),records=a.surfaceRecords;
    assert.equal(records.length,2);assert.ok(Object.isFrozen(records));assert.ok(records.every(Object.isFrozen));
    assert.throws(()=>a.assertCurrentSurface({toolPositions:p}),/matching original wall query/);
    const checkpoint=a.checkpoint();let queries=0;
    measure(a,f,{positions:p,consumeQuery:()=>queries++});
    const proof=a.assertCurrentSurface({toolPositions:p});assert.equal(queries,2);
    assert.equal(a.surfaceRecords,records);
    records.forEach((r,i)=>{
        assert.equal(r.index,i);assert.equal(r.base,3*i);assert.equal(r.raw.owner,r.owner);assert.equal(r.raw.edge,r.edge);
        assert.equal(r.raw.source,'analytic-plane');assert.equal(r.raw.derivativeUnavailable,false);
        assert.equal(r.seam,null);assert.equal(r.representative,null);assert.equal(r.dependent,null);
    });
    a.normalForces[0]=.1;
    assert.throws(()=>a.assertCurrentSurface({toolPositions:p}),/unchanged normal forces/);
    a.normalForces[0]=0;p.get('catheter')[0][0]+=.01;
    assert.throws(()=>a.assertCurrentSurface({toolPositions:p}),/stale/);
    p.get('catheter')[0][0]-=.01;a.restore(checkpoint);
    assert.throws(()=>a.assertCurrentSurface({toolPositions:p}),/matching original wall query/);
    measure(a,f,{positions:p,consumeQuery:()=>queries++});
    assert.ok(a.assertCurrentSurface({toolPositions:p}).generation>proof.generation);assert.equal(queries,4);
    const calls=f.wall.field.calls.length;measure(a,f,{positions:p,query:false});
    a.assertCurrentSurface({toolPositions:p});assert.equal(f.wall.field.calls.length,calls);
});

test('smooth sparse-SDF preserves distinct G/B, full DB, virtual work and material ownership under independent q/rho changes',()=>{
    const f=fixture({field:smoothField(),owners:[null,{owner:'wire',radius:2.2}],positions:[[-1.5,-1.5,0],[-.835,-.595,.055],[.925,.975,.125]]}),a=create(f);
    a.normalForces[0]=.7;const baseline=measure(a,f),row=a.rows[0],defs={common:Array.from(row.commonDofs),relative:Array.from(row.relativeDofs)};
    const G=Array.from(row.jacobian),B=Array.from(row.forceColumn,v=>-v),DB=Array.from(row.geometricTangent,v=>-v/.7),h=1e-6;
    defs.common.forEach((d,i)=>close(baseline.common[d],-.7*B[i],1e-15));defs.relative.forEach((d,i)=>close(baseline.relative[d],-.7*B[defs.common.length+i],1e-15));
    assert.ok(G.some((v,i)=>Math.abs(v-B[i])>.1));assert.ok(DB.some((v,i)=>Math.abs(v-DB[(i%G.length)*G.length+Math.floor(i/G.length)])>.01));
    assert.equal(baseline.certificate.samples[0].source,'sparse-sdf');assert.equal(f.wall.field.calls,1);
    assert.ok(a.nodalForces.get('catheter').flat().every(v=>v===0));
    const move=(col,delta)=>{
        const changed={...f,positions:structuredClone(f.positions),relative:f.relative.slice()};
        if(col<defs.common.length){const d=defs.common[col],node=Array.from(f.layout.positions).findIndex(v=>d>=v&&d<v+3);changed.positions[node][d-f.layout.positions[node]]+=delta;}
        else changed.relative[defs.relative[col-defs.common.length]]+=delta;
        return physical(changed);
    };
    for(let col=0;col<G.length;col++){
        const plus=measure(a,f,{positions:move(col,h)}),bp=Array.from(a.rows[0].forceColumn,v=>-v);
        const minus=measure(a,f,{positions:move(col,-h)}),bm=Array.from(a.rows[0].forceColumn,v=>-v);
        close(G[col],(plus.certificate.samples[0].gap-minus.certificate.samples[0].gap)/(2*h),2e-8);
        for(let i=0;i<G.length;i++)close(DB[i*G.length+col],(bp[i]-bm[i])/(2*h),2e-7);
    }
    measure(a,f);const direction=G.map((_,i)=>.125*Math.sin(i+.5)),base=physical(f),moved=structuredClone(base);
    for(let col=0;col<G.length;col++){const p=move(col,direction[col]);for(const [id,points] of p)for(let i=0;i<points.length;i++)for(let k=0;k<3;k++)moved.get(id)[i][k]+=points[i][k]-base.get(id)[i][k];}
    const worldWork=[...a.nodalForces].reduce((sum,[id,p])=>sum+p.reduce((s,v,i)=>s+v.reduce((z,F,k)=>z+F*(moved.get(id)[i][k]-base.get(id)[i][k]),0),0),0);
    close(worldWork,.7*B.reduce((s,v,i)=>s+v*direction[i],0),2e-13);
    const calls=f.wall.field.calls;measure(a,f,{positions:base,query:false});assert.equal(f.wall.field.calls,calls);assert.equal(a.rows[0].geometricTangentValid,true);
    measure(a,f,{order:'gradient'});assert.equal(a.rows[0].geometricTangentValid,false);
});

test('actual MeshBVH edge feature enters catheter q only without any derivative query',()=>{
    const field=triangleField(),f=fixture({field,positions:[[.8,-.7,.3],[1,-.3,.7],[2,.4,.8]],owners:[{owner:'catheter',radius:.8},null]}),a=create(f);
    a.normalForces[0]=2;const r=measure(a,f),row=a.rows[0];assert.equal(field.calls,1);assert.equal(field.bvhCalls,3);assert.equal(r.certificate.samples[0].source,'sparse-sdf-bvh');
    assert.equal(row.relativeDofs.length,0);assert.ok(r.relative.every(v=>v===0));assert.ok(a.nodalForces.get('wire').flat().every(v=>v===0));
    vectorClose(row.jacobian,Array.from(row.forceColumn,v=>-v),0);assert.ok(row.geometricTangent.some(v=>Math.abs(v)>.1));
    const saved=Array.from(row.geometricTangent);measure(a,f,{positions:r.positions,query:false});assert.equal(field.calls,1);assert.equal(field.bvhCalls,3);vectorClose(a.rows[0].geometricTangent,saved,0);
    field.fallbackGeometry.dispose();
});

test('unknown open exact-zero rows eliminate without a made-up derivative; loaded and penetrated unsupported rows reject atomically',()=>{
    const field=plane();field.source='unknown-source';const f=fixture({field,positions:[[0,1,0],[2,2,0],[4,3,0]],owners:[{owner:'catheter',radius:.3},null]}),a=create(f);
    const open=measure(a,f);assert.equal(open.certificate.converged,true);assert.equal(open.certificate.samples[0].eliminated,true);assert.ok(a.rows[0].forceColumn.every(v=>v===0));a.commit();
    a.normalForces[0]=.1;const common=new Float64Array(f.layout.dofCount).fill(7),relative=new Float64Array(f.relative.length).fill(9);
    assert.throws(()=>measure(a,f,{common,relative}),/Unsupported active\/loaded/);assert.ok(common.every(v=>v===7));assert.ok(relative.every(v=>v===9));assert.throws(()=>a.commit(),/fresh original query/);
    a.normalForces[0]=0;const p=physical(f);p.get('catheter')[0][1]=.2;assert.throws(()=>measure(a,f,{positions:p}),/Unsupported active\/loaded/);
});

test('signed private Fn is measured unchanged but cannot commit; query:false cannot certify stale geometry or mutated force',()=>{
    const f=fixture({owners:[{owner:'catheter',radius:.3},null]}),a=create(f);a.normalForces[0]=-1e-30;const r=measure(a,f);
    assert.equal(a.normalForces[0],-1e-30);assert.equal(r.certificate.converged,false);assert.ok(r.certificate.negativeForce>0);assert.throws(()=>a.commit(),/fresh original query/);
    a.normalForces[0]=2;measure(a,f);a.commit();const p=physical(f);p.get('catheter')[0][1]+=.01;
    assert.throws(()=>measure(a,f,{positions:p,query:false}),/stale/);assert.throws(()=>a.commit(),/fresh original query/);
    const current=measure(a,f);measure(a,f,{positions:current.positions,query:false});assert.throws(()=>a.commit(),/fresh original query/);
    measure(a,f);a.normalForces[0]=3;assert.throws(()=>a.commit(),/fresh original query/);
    const current2=measure(a,f);current2.positions.get('catheter')[0][1]+=.001;assert.throws(()=>a.commit(),/stale/);
});

test('actual chart indices, modes, owner support, field identity/policy and cloneable history are validated',()=>{
    const f=fixture({owners:[{owner:'catheter',radius:.3},null]}),a=create(f);a.normalForces[0]=2;measure(a,f);const history=a.commit();
    assert.doesNotThrow(()=>create({...f,history:structuredClone(history)}));
    const rescaled=create({...f,wall:{...f.wall,forcePerLength:100},history});assert.equal(rescaled.normalForces[0],2,'NCP scale cannot rescale physical history');
    assert.throws(()=>create({...f,wall:{...f.wall,field:plane()},history}),/provenance changed/);
    const changed=structuredClone(f.wall.contactOwners);changed.edges[0].wall.radius=.4;assert.throws(()=>create({...f,wall:{...f.wall,contactOwners:changed},history}),/provenance changed/);
    assert.throws(()=>create({...f,layout:{...f.layout,positions:Int32Array.from([0,100,200])}}),/Layout indices/);
    assert.throws(()=>create({...f,modes:f.modes.map((m,i)=>i?{...m,node:99}:m)}),/Full ordered modes/);
    assert.throws(()=>create({...f,wall:{...f.wall,contactOwners:{edges:[{edge:99,wall:{owner:'catheter',radius:.3}},{edge:1,wall:null}]}}}),/edge order/);
    assert.throws(()=>create({...f,wall:{...f.wall,contactOwners:{edges:[{edge:0,wall:{owner:'missing',radius:.3}},{edge:1,wall:null}]}}}),/occupy/);
    f.coordinates[1]+=.01;assert.throws(()=>measure(a,f),/Frozen wall source/);f.coordinates[1]-=.01;
    f.wall.field.bvhValidationDistance=.123;assert.throws(()=>measure(a,f),/Frozen wall source/);
    const s=fixture({field:smoothField(),owners:[null,{owner:'wire',radius:2.2}],positions:[[-1.5,-1.5,0],[-.885,-.695,.055],[.875,.875,.125]]}),b=create(s);
    s.wall.field.sdfDistances=s.wall.field.sdfDistances.slice();assert.throws(()=>measure(b,s),/Frozen wall source/);
});

test('actual query budget, failed-query invalidation and loaded source/sample changes remain explicit',()=>{
    const f=fixture(),a=create(f);let used=0;
    assert.throws(()=>measure(a,f,{consumeQuery:()=>{if(used===1)throw Error('query-budget');used++;}}),/query-budget/);
    assert.equal(a.diagnostics.queries,1);assert.equal(f.wall.field.calls.length,1);assert.throws(()=>measure(a,f,{query:false}),/matching original/);
    measure(a,f);a.normalForces[0]=2;measure(a,f);const p=physical(f);p.get('catheter')[0][1]=1;assert.throws(()=>measure(a,f,{positions:p}),/source\/sample\/feature changed/);
    f.wall.field.source='new-source';assert.throws(()=>measure(a,f),/source\/sample\/feature changed/);
});

test('exact duplicate endpoint reactions share one row and checkpoints restore the prior force/structure',()=>{
    const f=fixture({edges:[['wire','catheter'],['wire','catheter']],positions:[[0,1,0],[2,.3,0],[4,1,0]],owners:[{owner:'catheter',radius:.3},{owner:'catheter',radius:.3}]}),a=create(f),checkpoint=a.checkpoint();
    a.normalForces[0]=2;a.normalForces[3]=3;const r=measure(a,f);assert.equal(r.certificate.converged,true);assert.equal(r.certificate.rowStructureChanged,true);
    assert.equal(a.rows.length,1);assert.equal(a.normalForces[0],5);assert.equal(a.normalForces[3],0);close(a.nodalForces.get('catheter')[1][1],5,0);
    a.normalForces[0]=NaN;const restored=a.restore(checkpoint);assert.equal(restored.rowStructureChanged,true);assert.equal(a.rows.length,2);assert.ok(a.normalForces.every(v=>v===0));assert.throws(()=>measure(a,f,{query:false}),/matching original/);
});

test('Coulomb sample reactions retain separate coincident endpoint forces and restore their full private state',()=>{
    const make=()=>fixture({edges:[['wire','catheter'],['wire','catheter']],positions:[[0,1,0],[2,.3,0],[4,1,0]],
        owners:[{owner:'catheter',radius:.3},{owner:'catheter',radius:.3}]});
    const normalFixture=make(),normal=create(normalFixture);
    normal.normalForces[0]=2;normal.normalForces[3]=3;
    const aggregate=measure(normal,normalFixture);
    assert.equal(normal.rows.length,1);assert.deepEqual(Array.from(normal.normalForces),[5,0,0,0,0,0]);

    const f=make();
    const a=create({...f,preserveSampleReactions:true});a.normalForces[0]=2;a.normalForces[3]=3;
    const measured=measure(a,f),checkpoint=a.checkpoint();
    assert.equal(measured.certificate.converged,true);assert.equal(measured.certificate.rowStructureChanged,false);
    assert.deepEqual(a.rowForceIndices,[0,3]);assert.deepEqual(Array.from(a.normalForces),[2,0,0,3,0,0]);
    assert.ok(a.surfaceRecords.every(s=>s.representative===null));
    vectorClose(measured.common,aggregate.common,0);vectorClose(measured.relative,aggregate.relative,0);
    assert.deepEqual(a.nodalForces,normal.nodalForces);close(a.nodalForces.get('catheter')[1][1],5,0);
    // Equal normal loads do not establish equal tangential wrenches: these
    // adjacent edges still own distinct one-sided directors and spin DOFs.
    assert.notEqual(f.layout.spins.get('catheter')[0],f.layout.spins.get('catheter')[1]);
    a.normalForces[0]=7;a.normalForces[3]=11;measure(a,f);
    assert.equal(a.restore(checkpoint).rowStructureChanged,false);
    assert.deepEqual(Array.from(a.normalForces),[2,0,0,3,0,0]);assert.deepEqual(a.rowForceIndices,[0,3]);
    assert.throws(()=>measure(a,f,{query:false}),/matching original/);
    const restored=measure(a,f);vectorClose(restored.common,measured.common,0);
    vectorClose(restored.relative,measured.relative,0);assert.deepEqual(a.nodalForces,normal.nodalForces);
    const history=a.commit();assert.throws(()=>create({...f,preserveSampleReactions:false,history}),/history|provenance/);
});

const P1={initial:[[65.00287246704102,-462.00980948623305,-79.56869888305664],[64.95548751831055,-461.7916869276393,-79.65612350463867]],
    first:[[64.97836989399555,-462.0296690503204,-79.55471756171002],[65,-461.81336005090367,-79.6557542069334],[64.90596731869704,-461.6053450112134,-79.72892145713489]],
    firstFn:[2.1924778704896255,.5980074238507306],
    second:[[64.96934865187995,-462.05074231537515,-79.53995104240776],[65.01337321578661,-461.8433842109076,-79.65188987694235],[64.92012710276462,-461.63224461704743,-79.71662844893282]],secondFn:[0,.5801722357399944]};
function realFixture() {
    const bytes=fs.readFileSync(url('res/Aorta_plain.collision.bin')),anatomy=decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
    const field=new VesselContactField(anatomy),query=field.queryCapsuleCoordinates.bind(field);let calls=0;
    field.queryCapsuleCoordinates=(...args)=>{calls++;return query(...args);};
    const [a,b]=P1.initial,positions=[a,b,b.map((v,i)=>2*v-a[i])];
    const f=fixture({field,edges:[['wire','catheter'],['wire','catheter']],positions,owners:[{owner:'catheter',radius:.4445},null]});
    return {f,calls:()=>calls};
}
function catPositions(f,p){const out=physical(f);out.set('catheter',structuredClone(p));return out;}

test('real P1 discovers both min branches without re-query; rejected discovery restores charts and retries cleanly',()=>{
    const {f,calls}=realFixture(),a=create(f),base=measure(a,f),checkpoint=a.checkpoint(),before=calls();
    assert.equal(a.rows.length,1);close(base.certificate.samples[0].gap,-.02,1e-12);
    const delta=new Map([['catheter',[zero(),[.1,0,0],zero()]]]);
    const discovery=a.discoverCharts({deltaPhysical:delta});assert.equal(discovery.rowStructureChanged,true);assert.equal(a.rows.length,2);assert.equal(calls(),before);
    assert.deepEqual(a.rowForceIndices,[1,2]);assert.ok(a.rows.every(r=>r.sdfBranch!==null));
    const onChart=measure(a,f,{positions:base.positions,query:false});assert.equal(onChart.certificate.samples.length,2);assert.equal(calls(),before);
    a.normalForces.set([0,2,1]);const restore=a.restore(checkpoint);assert.equal(restore.rowStructureChanged,true);assert.equal(a.rows.length,1);assert.ok(a.normalForces.every(v=>v===0));
    assert.throws(()=>measure(a,f,{positions:base.positions,query:false}),/matching original/);assert.throws(()=>a.commit(),/fresh original/);
    const retry=measure(a,f);a.discoverCharts({deltaPhysical:delta});measure(a,f,{positions:retry.positions,query:false});
    const clean=create(f),fresh=measure(clean,f);clean.discoverCharts({deltaPhysical:delta});measure(clean,f,{positions:fresh.positions,query:false});
    assert.deepEqual(a.rowForceIndices,clean.rowForceIndices);assert.deepEqual(a.normalForces,clean.normalForces);
    assert.throws(()=>a.restore(clean.checkpoint()),/Foreign/);
});

test('real P1 source query certifies the cone and release witnesses with literal sign/domain gates and owned history',()=>{
    const {f,calls}=realFixture(),a=create(f),first=catPositions(f,P1.first);let counted=0;
    measure(a,f,{positions:first,consumeQuery:()=>counted++});assert.equal(a.rows.length,2);a.normalForces.set([0,...P1.firstFn]);
    const loaded=measure(a,f,{positions:first,consumeQuery:()=>counted++});assert.equal(loaded.certificate.converged,true);assert.equal(loaded.certificate.domainAdmissible,true);
    const saved=a.commit(),before=structuredClone(saved),clone=create({...f,history:structuredClone(saved)}),second=catPositions(f,P1.second);
    const wrongDomain=measure(clone,f,{positions:second});assert.equal(wrongDomain.certificate.domainAdmissible,false);assert.equal(wrongDomain.certificate.converged,false);
    clone.normalForces.set([0,-1e-30,P1.secondFn[1]]);const signed=measure(clone,f,{positions:second});assert.equal(signed.certificate.converged,false);assert.equal(clone.normalForces[1],-1e-30);assert.throws(()=>clone.commit(),/fresh original/);
    clone.normalForces.set([0,...P1.secondFn]);const released=measure(clone,f,{positions:second});assert.equal(released.certificate.converged,true);assert.equal(released.certificate.domainAdmissible,true);
    const result=clone.commit();assert.ok(Object.is(result.normalForces[1],0));assert.ok(result.normalForces[2]>0);assert.deepEqual(saved,before);
    const total=clone.nodalForces.get('catheter').reduce((v,p)=>v.map((x,i)=>x+p[i]),[0,0,0]);close(Math.hypot(...total),P1.secondFn[1],1e-12);
    assert.ok(clone.nodalForces.get('wire').flat().every(v=>v===0));assert.equal(counted,2);assert.equal(a.diagnostics.queries,2);assert.equal(calls(),a.diagnostics.queries+clone.diagnostics.queries);
    const bad=structuredClone(result);bad.records[0].seam.face.gridIndex++;const stale=create({...f,history:bad});assert.throws(()=>measure(stale,f,{positions:second}),/Unsupported SDF cone/);
});

function flatPlane() {
    return {calls:[],queryCapsuleCoordinates(ax,ay,az,bx,by,bz,radius,out=createContactResult()) {
        this.calls.push([ax,ay,az,bx,by,bz,radius]);
        const t=ay===by?.5:ay<by?0:1,p=[ax+t*(bx-ax),ay+t*(by-ay),az+t*(bz-az)];
        out.signedDistance=p[1];out.signedGap=p[1]-radius;out.segmentT=t;out.capsuleSampleCount=2;
        out.inward.values.set([0,1,0]);out.closestPoint.values.set([p[0],0,p[2]]);out.source='analytic-plane';out.faceIndex=0;return out;
    }};
}
function envelopeFixture(extra={}) {
    const f=fixture({field:flatPlane(),positions:[[0,.3,0],[2,.3,0],[4,.4,0]],owners:[{owner:'catheter',radius:.3},null],...extra});
    f.wall.contactMode='envelope';return f;
}
function wrench(adapter,id,positions) {
    return adapter.nodalForces.get(id).reduce((out,F,i)=>{
        const p=positions.get(id)[i];out.force=out.force.map((v,k)=>v+F[k]);
        const m=[p[1]*F[2]-p[2]*F[1],p[2]*F[0]-p[0]*F[2],p[0]*F[1]-p[1]*F[0]];out.moment=out.moment.map((v,k)=>v+m[k]);return out;
    },{force:[0,0,0],moment:[0,0,0]});
}

test('envelope solves the flat loaded plane sample change through retained endpoints, original gaps and full rollback',()=>{
    const f=envelopeFixture(),a=create(f);a.normalForces[2]=6;const base=measure(a,f),rows=a.rows;
    assert.equal(f.wall.field.calls.length,3);assert.deepEqual(a.rowForceIndices,[0,1]);assert.deepEqual(rows.map(r=>r.role),['proximal','distal']);
    vectorClose(a.normalForces,[3,3,0],0);assert.equal(base.certificate.samples.length,3);assert.ok(base.certificate.samples.every(r=>r.gap===0));
    const original=base.certificate.samples[2];assert.equal(original.querySampleFraction,.5);assert.deepEqual(original.dependent.weights,[.5,.5]);
    assert.equal(base.certificate.converged,true);assert.deepEqual(wrench(a,'catheter',base.positions),{force:[0,6,0],moment:[0,0,6]});
    const force=a.nodalForces.get('catheter');close(force[0][1]*1+force[1][1]*3,6*2,0); // arbitrary endpoint virtual work, original t=.5
    const checkpoint=a.checkpoint(),history=a.commit(),before=structuredClone(history),tilted=physical(f);tilted.get('catheter')[1][1]=.31;
    const trial=measure(a,f,{positions:tilted});assert.equal(trial.certificate.samples[2].querySampleFraction,0);assert.deepEqual(a.rowForceIndices,[0,1]);
    assert.equal(trial.certificate.converged,false);assert.ok(trial.certificate.complementarity>0);vectorClose(a.normalForces,[3,3,0],0);
    assert.equal(trial.certificate.samples[1].endpointFraction,1);assert.equal(trial.certificate.samples[1].querySampleFraction,.5);
    a.normalForces[1]=-1e-30;const signed=measure(a,f,{positions:tilted});assert.equal(signed.certificate.converged,false);assert.equal(a.normalForces[1],-1e-30);
    a.normalForces[1]=0;const released=measure(a,f,{positions:tilted});assert.equal(released.certificate.converged,true);a.commit();
    assert.deepEqual(wrench(a,'catheter',tilted),{force:[0,3,0],moment:[0,0,0]});assert.ok(a.nodalForces.get('wire').flat().every(v=>v===0));
    a.restore(checkpoint);vectorClose(a.normalForces,[3,3,0],0);assert.throws(()=>measure(a,f,{positions:base.positions,query:false}),/matching original/);
    const retry=measure(a,f),restored=a.commit();assert.equal(retry.certificate.converged,true);assert.deepEqual(restored,history);assert.deepEqual(history,before);
    const cloned=create({...f,history:structuredClone(history)});assert.equal(measure(cloned,f).certificate.converged,true);
});

test('envelope signed exact linear transfer preserves force, tangent and wrench without clipping',()=>{
    const f=envelopeFixture(),a=create(f);a.normalForces.set([3,4,-2]);const checkpoint=a.checkpoint(),r=measure(a,f);
    vectorClose(a.normalForces,[2,3,0],0);assert.equal(r.certificate.converged,true);assert.deepEqual(a.rowForceIndices,[0,1]);
    assert.deepEqual(wrench(a,'catheter',r.positions),{force:[0,5,0],moment:[0,0,6]}); // 4*2 - 2*1
    assert.ok(a.rows.every(row=>row.geometricTangent.every(v=>v===0)));
    a.restore(checkpoint);vectorClose(a.normalForces,[3,4,-2],0);a.normalForces.set([1,1,-10]);
    const negative=measure(a,f);vectorClose(a.normalForces,[-4,-4,0],0);assert.equal(negative.certificate.converged,false);assert.equal(negative.certificate.negativeForce,4);assert.throws(()=>a.commit(),/fresh original/);
});

test('envelope combines exact shared endpoint constraints after local transfer and checkpoints every role/gauge',()=>{
    const f=envelopeFixture({edges:[['wire','catheter'],['wire','catheter']],positions:[[0,.3,0],[2,.3,0],[4,.3,0]],owners:[{owner:'catheter',radius:.3},{owner:'catheter',radius:.3}]}),a=create(f);
    a.normalForces.set([1,2,4,3,5,6]);const checkpoint=a.checkpoint(),r=measure(a,f);
    assert.equal(f.wall.field.calls.length,5);assert.equal(r.certificate.samples.length,6);assert.deepEqual(a.rowForceIndices,[0,1,4]);vectorClose(a.normalForces,[3,10,0,0,8,0],0);
    assert.equal(r.certificate.converged,true);assert.deepEqual(wrench(a,'catheter',r.positions),{force:[0,21,0],moment:[0,0,52]});
    const saved=a.commit();assert.equal(saved.contactMode,'envelope');assert.deepEqual(saved.records.map(r=>r.role),['proximal','distal','capsule','proximal','distal','capsule']);
    assert.equal(saved.records[3].representative,1);assert.equal(saved.records[2].dependent.kind,'endpoint-combination');
    assert.equal(a.restore(checkpoint).rowStructureChanged,true);vectorClose(a.normalForces,[1,2,4,3,5,6],0);assert.equal(a.rows.length,6);
    measure(a,f);assert.deepEqual(a.commit(),saved);
});

test('envelope SDF endpoint POINT derivatives preserve raw query t versus physical fraction and retain nonlinear capsule DB',()=>{
    const f=envelopeFixture({field:smoothField(),owners:[null,{owner:'wire',radius:2.5}],positions:[[-1.5,-1.5,0],[-.835,-.595,.055],[.925,.975,.125]]}),a=create(f);
    a.normalForces.set([1,2,3]);const r=measure(a,f),[proximal,distal,capsule]=a.rows;
    assert.deepEqual(a.rowForceIndices,[0,1,2]);assert.equal(f.wall.field.calls,3);assert.equal(r.certificate.samples[2].querySampleFraction,.5);
    assert.equal(r.certificate.samples[0].endpointFraction,0);assert.equal(r.certificate.samples[1].endpointFraction,1);
    assert.equal(r.certificate.samples[1].querySampleFraction,0,'the degenerate original query selected t=0, not physical endpoint 0');
    assert.ok(proximal.jacobian.slice(3,6).every(v=>v===0));assert.ok(distal.jacobian.slice(0,3).every(v=>v===0));
    assert.ok(capsule.geometricTangent.some(v=>Math.abs(v)>1e-3));assert.ok(a.nodalForces.get('catheter').flat().every(v=>v===0));
    const saved=Array.from(distal.jacobian),DB=Array.from(distal.geometricTangent,v=>-v/2),h=1e-6,p=physical(f),q=structuredClone(p);
    // Physical distal wire node2 is common q (no rho mode). Perturb only it.
    q.get('wire')[2][0]+=h;const plus=measure(a,f,{positions:q}),bp=Array.from(a.rows[1].forceColumn,v=>-v);
    q.get('wire')[2][0]-=2*h;const minus=measure(a,f,{positions:q}),bm=Array.from(a.rows[1].forceColumn,v=>-v);
    close(saved[3],(plus.certificate.samples[1].gap-minus.certificate.samples[1].gap)/(2*h),2e-8);
    for(let i=0;i<saved.length;i++)close(DB[saved.length*i+3],(bp[i]-bm[i])/(2*h),2e-7);
    measure(a,f,{positions:p});const calls=f.wall.field.calls;measure(a,f,{positions:p,query:false});assert.equal(f.wall.field.calls,calls);
});

test('envelope BVH endpoint query fractions remain owned and a flat face capsule can transfer exactly to both endpoints',()=>{
    const field=triangleField(),f=envelopeFixture({field,positions:[[.25,.25,.5],[.75,.25,.5],[1,.5,.5]],owners:[{owner:'catheter',radius:.5},null]}),a=create(f);
    a.normalForces[2]=2;const r=measure(a,f);assert.equal(r.certificate.converged,true);assert.deepEqual(a.rowForceIndices,[0,1]);vectorClose(a.normalForces,[1,1,0],0);
    assert.equal(field.calls,3);assert.equal(field.bvhCalls,9);assert.equal(r.certificate.samples[1].querySampleFraction,.5);assert.equal(r.certificate.samples[1].endpointFraction,1);
    assert.equal(r.certificate.samples[2].source,'sparse-sdf-bvh');assert.ok(a.rows[1].forceColumn.slice(0,3).every(v=>v===0));
    const history=a.commit();assert.equal(history.records[1].provenance.sampleFraction,.5);assert.equal(history.records[1].provenance.endpointFraction,1);
    const p=physical(f);p.get('catheter')[1][2]=.6;const opened=measure(a,f,{positions:p});assert.equal(opened.certificate.samples[2].querySampleFraction,0);assert.equal(opened.certificate.converged,false);
    a.normalForces[1]=0;assert.equal(measure(a,f,{positions:p}).certificate.converged,true);a.commit();field.fallbackGeometry.dispose();
});

test('contact modes cannot reinterpret history; independent envelope capsule sample switches and new seam cases still reject',()=>{
    const f=fixture({owners:[{owner:'catheter',radius:.3},null]}),capsule=create(f);measure(capsule,f);const history=capsule.commit();
    assert.throws(()=>create({...f,wall:{...f.wall,contactMode:'envelope'},history}),/provenance changed/);
    const e=envelopeFixture(),envelope=create(e);measure(envelope,e);const saved=envelope.commit();assert.throws(()=>create({...e,wall:{...e.wall,contactMode:'capsule'},history:saved}),/provenance changed/);
    assert.throws(()=>create({...f,wall:{...f.wall,contactMode:'unknown'}}),/contactMode/);
    const nonlinear=envelopeFixture({field:smoothField(),owners:[null,{owner:'wire',radius:2.5}],positions:[[-1.5,-1.5,0],[-.835,-.595,.055],[.925,.975,.125]]}),n=create(nonlinear);
    n.normalForces[2]=1;measure(n,nonlinear);const switched=physical(nonlinear);switched.get('wire')[2]=[-.6,-.55,.125];
    assert.throws(()=>measure(n,nonlinear,{positions:switched}),/Loaded wall source\/sample\/feature changed/);
    const {f:real}=realFixture();real.wall.contactMode='envelope';const seam=create(real);
    assert.throws(()=>measure(seam,real,{positions:catPositions(real,P1.first)}),/Unsupported active\/loaded wall derivative/);
});

function nodalFixture({field=plane(),positions=[[0,.3,0],[2,.3,0],[4,.3,0]],owners=null,sites=null}={}) {
    const f=fixture({edges:[['wire','catheter'],['wire','catheter']],field,positions,owners:owners??[{owner:'catheter',radius:.3},{owner:'catheter',radius:.3}]});
    f.wall.contactMode='nodal-endpoints';f.wall.pressureDiscretization='nodal-endpoints-one-sided-surface';
    f.wall.pressureSites=sites??[{owner:'catheter',node:0,edge:0,trace:'right'},{owner:'catheter',node:1,edge:0,trace:'left'},{owner:'catheter',node:2,edge:1,trace:'left'}];
    return f;
}

function materialPointFixture(options={}) {
    const f=nodalFixture(options);f.wall.contactMode='material-points';f.wall.contactUpdate='current-query';f.wall.pressureDiscretization='fixed-material-points';
    f.wall.pressureSites=f.wall.pressureSites.map(s=>({owner:s.owner,edge:s.edge,fraction:s.node-s.edge,trace:s.trace}));
    for(const e of f.wall.contactOwners.edges)if(e.wall)f.wall.pressureSites.push({owner:e.wall.owner,edge:e.edge,fraction:.5});
    return f;
}

test('material point pressure owns endpoint and interior reactions while checking every original capsule',()=>{
    const f=materialPointFixture(),a=create(f);a.normalForces.set([.3,.2,.4,.1,.5]);let queries=0;
    const r=measure(a,f,{consumeQuery:()=>queries++});assert.equal(r.certificate.converged,true);assert.equal(queries,10);
    assert.deepEqual(a.rowForceIndices,[0,1,2,3,4]);assert.equal(r.certificate.originalInequalities.length,6);
    assert.ok(a.surfaceRecords.every(s=>s.role==='material-point'&&s.representative===null));
    assert.deepEqual(a.surfaceRecords.map(s=>s.fraction),[0,1,1,.5,.5]);
    a.nodalForces.get('catheter').forEach((p,i)=>vectorClose(p,[0,[.35,.5,.65][i],0],1e-14));
    const history=a.commit(),b=create({...f,history});measure(b,f);assert.deepEqual(b.commit(),history);
    const checkpoint=a.checkpoint();a.normalForces.fill(0);measure(a,f);a.restore(checkpoint);measure(a,f);
    assert.deepEqual(a.commit(),history);
});

test('capsule winner changes never transfer a loaded material endpoint reaction',()=>{
    const f=materialPointFixture(),a=create(f);a.normalForces.set([2,0,3,0,0]);const original=measure(a,f),positions=physical(f);
    positions.get('catheter')[1][1]+=.001;const next=measure(a,f,{positions});
    assert.equal(next.certificate.converged,true);assert.equal(next.certificate.rowStructureChanged,false);
    assert.notEqual(original.certificate.originalInequalities.find(s=>s.edge===0&&s.role==='capsule').querySampleFraction,
        next.certificate.originalInequalities.find(s=>s.edge===0&&s.role==='capsule').querySampleFraction);
    assert.deepEqual(Array.from(a.normalForces),[2,0,3,0,0]);
    assert.deepEqual(a.nodalForces.get('catheter'),[[0,2,0],[0,0,0],[0,3,0]]);
});

test('material point law rejects missing, duplicate or reinterpreted pressure sites',()=>{
    for(const change of [f=>f.wall.pressureSites.shift(),f=>f.wall.pressureSites.push({...f.wall.pressureSites[3]}),
        f=>f.wall.pressureSites.push({owner:'catheter',edge:1,fraction:0,trace:'right'}),
        f=>f.wall.pressureSites[3].trace='left',f=>f.wall.pressureSites[3].fraction=1.1,
        f=>f.wall.contactOwners.edges[1].wall.radius=.31]) {
        const f=materialPointFixture();change(f);assert.throws(()=>create(f),/site|endpoint|fraction|radius|trace/i);
    }
    const f=materialPointFixture(),a=create(f);measure(a,f);const history=a.commit();
    f.wall.pressureSites[3].fraction=.4;assert.throws(()=>measure(a,f),/Frozen/);assert.throws(()=>create({...f,history}),/history|provenance/);
});

test('fixed pressure samples cannot certify an uncovered original capsule interior penetration',()=>{
    const f=materialPointFixture({field:smoothField(),positions:[[-.375,-.375,0],[.625,.625,0],[2,.625,0]],
        owners:[{owner:'catheter',radius:2.05},null],sites:[{owner:'catheter',node:0,edge:0,trace:'right'},{owner:'catheter',node:1,edge:0,trace:'left'}]});
    f.wall.pressureSites[2].fraction=.1;const a=create(f),r=measure(a,f);
    assert.ok(r.certificate.samples.every(s=>s.gap>0&&s.Fn===0));assert.equal(r.certificate.converged,false);
    const coverage=r.certificate.originalInequalities.find(s=>s.role==='capsule');close(coverage.gap,-.05,1e-12);
    assert.ok(r.certificate.coverageMerit>0);assert.throws(()=>a.commit(),/fresh original/);
});

test('capsule argmin switching moves a finite loaded reaction at arbitrarily small displacement; nodal pressure does not',()=>{
    // Characterize the failed anatomy corrector: this is a force-location
    // discontinuity, even for a smooth plane with a constant normal.
    for(const epsilon of [1e-6,1e-9,1e-12]) {
        for(const nodal of [false,true]) {
            const f=nodalFixture();
            if(!nodal)f.wall.contactMode='capsule';
            f.wall.contactUpdate='current-query';
            const a=create({...f,preserveSampleReactions:true});
            a.normalForces[nodal?1:0]=196;
            const p=physical(f);p.get('catheter')[0][1]+=-epsilon;
            measure(a,f,{positions:p});
            const before=a.nodalForces.get('catheter').map(v=>Array.from(v));
            p.get('catheter')[0][1]+=2*epsilon;
            measure(a,f,{positions:p});
            const after=a.nodalForces.get('catheter');
            const change=Math.hypot(...after.flatMap((v,i)=>Array.from(v,(x,k)=>x-before[i][k])));
            close(change,nodal?0:196*Math.SQRT2,1e-12);
            close(after.reduce((sum,v)=>sum+v[1],0),196,1e-12);
        }
    }
});

test('explicit nodal pressure owns one integrated force per shared node and retains every original envelope inequality',()=>{
    const f=nodalFixture(),a=create(f);a.normalForces.set([.3,.4,.5]);let queries=0;
    const measured=measure(a,f,{consumeQuery:()=>queries++});assert.equal(queries,5);assert.equal(a.normalForces.length,3);assert.equal(a.rows.length,3);
    assert.equal(measured.certificate.converged,true);assert.equal(measured.certificate.originalInequalities.length,6);
    assert.equal(measured.certificate.originalInequalities.filter(r=>r.pressureDof===false).length,2);
    assert.deepEqual(a.rowForceIndices,[0,1,2]);assert.deepEqual(a.surfaceRecords.map(r=>[r.node,r.edge,r.role,r.trace]),[[0,0,'proximal','right'],[1,0,'distal','left'],[2,1,'distal','left']]);
    a.nodalForces.get('catheter').forEach((v,i)=>vectorClose(v,[0,[.3,.4,.5][i],0],0));
    assert.ok(a.nodalForces.get('wire').flat().every(v=>v===0));assert.ok(a.surfaceRecords.every(r=>r.dependent===null&&r.representative===null&&r.seam===null));
    const state=a.commit(),b=create({...f,history:state});measure(b,f);assert.deepEqual(b.commit(),state);
});

test('loaded original capsule sample switches do not change nodal force coordinates or one-sided material ownership',()=>{
    const f=nodalFixture(),a=create(f);a.normalForces.set([.3,.4,.5]);const first=measure(a,f),history=a.commit(),checkpoint=a.checkpoint(),rows=a.rows;
    const p=physical(f);p.get('catheter')[1][1]+=.001;
    const changed=measure(a,f,{positions:p}),old=first.certificate.originalInequalities.find(r=>r.edge===0&&r.role==='capsule'),next=changed.certificate.originalInequalities.find(r=>r.edge===0&&r.role==='capsule');
    assert.notEqual(next.querySampleFraction,old.querySampleFraction);assert.equal(changed.certificate.rowStructureChanged,false);
    assert.deepEqual(a.rows,rows);assert.deepEqual(Array.from(a.normalForces),[.3,.4,.5]);assert.equal(a.diagnostics.discoveries,0);
    a.assertCurrentSurface({toolPositions:p});a.restore(checkpoint);measure(a,f);assert.deepEqual(a.commit(),history);
});

test('nodal coverage rejects a true sparse-SDF interior penetration even with open pressure endpoints and zero reactions',()=>{
    const f=nodalFixture({field:smoothField(),positions:[[-.375,-.375,0],[.625,.625,0],[2,.625,0]],owners:[{owner:'catheter',radius:2.05},null],
        sites:[{owner:'catheter',node:0,edge:0,trace:'right'},{owner:'catheter',node:1,edge:0,trace:'left'}]}),a=create(f),r=measure(a,f);
    assert.equal(a.rows.length,2);assert.ok(r.certificate.samples.every(s=>s.Fn===0&&s.gap>0&&s.ncp===0));
    const interior=r.certificate.originalInequalities.find(s=>s.role==='capsule');close(interior.gap,-.05,1e-12);assert.equal(interior.pressureDof,false);
    assert.equal(r.certificate.converged,false);close(r.certificate.penetration,.05,1e-12);assert.ok(r.certificate.coverageMerit>0);
    assert.throws(()=>a.commit(),/fresh original query/);
});

test('nodal pressure law and the incident material trace cannot reinterpret accepted capsule or nodal history',()=>{
    const f=nodalFixture(),a=create(f);a.normalForces.set([.3,.4,.5]);measure(a,f);const history=a.commit();
    const other=nodalFixture();other.wall.pressureSites[1]={owner:'catheter',node:1,edge:1,trace:'right'};
    assert.throws(()=>create({...other,history}),/provenance changed/);
    const capsule=fixture({edges:[['wire','catheter'],['wire','catheter']],positions:f.positions,field:f.wall.field,owners:[{owner:'catheter',radius:.3},{owner:'catheter',radius:.3}]}),c=create(capsule);
    measure(c,capsule);assert.throws(()=>create({...f,history:c.commit()}),/provenance changed/);
    f.wall.pressureSites[1].edge=1;f.wall.pressureSites[1].trace='right';
    assert.throws(()=>measure(a,f),/Frozen nodal pressure/);assert.throws(()=>a.commit(),/Frozen nodal pressure/);
});

test('nodal sites require complete unique endpoints, correct own-edge traces and a continuous radius at shared nodes',()=>{
    const variants=[f=>delete f.wall.pressureDiscretization,f=>f.wall.pressureSites.pop(),f=>f.wall.pressureSites[1]={...f.wall.pressureSites[0]},
        f=>f.wall.pressureSites[1].trace='right',f=>f.wall.pressureSites[1].owner='wire',f=>f.wall.pressureSites[1].edge=7,
        f=>f.wall.contactOwners.edges[1].wall.radius=.4];
    for(const change of variants){const f=nodalFixture();change(f);assert.throws(()=>create(f),/pressure|Pressure|site|trace|radius/);}
});

test('raw zero-length endpoint query fractions do not change the fixed physical nodal trace',()=>{
    const field=plane(),original=field.queryCapsuleCoordinates;let flip=false;
    field.queryCapsuleCoordinates=function(...args){const out=original.apply(this,args);if(args[0]===args[3]&&args[1]===args[4]&&args[2]===args[5])out.segmentT=flip?0:1;return out;};
    const f=nodalFixture({field}),a=create(f);a.normalForces.set([.3,.4,.5]);measure(a,f);const history=a.commit();flip=true;
    const r=measure(a,f);assert.equal(r.certificate.converged,true);assert.deepEqual(a.commit(),history);
    assert.deepEqual(a.surfaceRecords.map(r=>r.raw.t),[0,1,1]);assert.ok(a.surfaceRecords.every(r=>r.raw.rawContact.segmentT===0));
});

test('normal wall history preserves bigint material identities without aliasing numeric/string IDs',()=>{
    const field=plane(),make=id=>{
        const f=nodalFixture({field});f.wall.contactOwners.edges.forEach(e=>e.wall.materialSegmentId=id);return f;
    };
    const f=make(17n),a=create(f);a.normalForces.set([.3,.4,.5]);measure(a,f);const history=a.commit();
    const sameIds=create({...make(17n),history});measure(sameIds,f);assert.deepEqual(sameIds.commit(),history);
    for(const id of [17,'17',18n])assert.throws(()=>create({...make(id),history}),/provenance changed/);
    for(const id of [{type:'joint-wall-bigint-id',value:'17'},NaN,Infinity])assert.throws(()=>create(make(id)),/materialSegmentId/);
    f.wall.contactOwners.edges[0].wall.materialSegmentId=17;assert.throws(()=>measure(a,f),/Frozen wall source/);
    const capsule=make(17n);capsule.wall.contactMode='capsule';const c=create(capsule);measure(c,capsule);assert.equal(c.commit().contactMode,'capsule');
});

test('explicit current-query SDF policy refreshes a loaded cell derivative and preserves rollback and final residual checks',()=>{
    for(const current of [false,true]) {
        const f=fixture({edges:[['wire'],['wire']],field:smoothField(),
            positions:[[.1,.2,.2],[.2,.2,.2],[.3,.3,.3]],owners:[null,{owner:'wire',radius:2.5}]});
        if(current)f.wall.contactUpdate='current-query';
        const a=create(f);a.normalForces[0]=3;
        const before=measure(a,f),saved=a.checkpoint(),p=physical(f);
        assert.throws(()=>a.commit(),/certified wall forces/);
        p.get('wire').forEach(v=>v[0]+=.4);
        if(!current){assert.throws(()=>measure(a,f,{positions:p}),/Loaded SDF cell changed/);continue;}
        const next=measure(a,f,{positions:p});
        assert.equal(a.normalForces[0],3);assert.equal(next.certificate.samples[0].source,'sparse-sdf');
        assert.ok(next.certificate.penetration>0,'A fresh derivative cannot erase the violated contact');
        assert.notDeepEqual(next.common,before.common,'Current physical normal changes the force residual');
        a.restore(saved);const restored=measure(a,f);assert.deepEqual(restored.certificate,{...before.certificate,queryGeneration:restored.certificate.queryGeneration});
        assert.deepEqual(restored.common,before.common);assert.throws(()=>a.commit(),/certified wall forces/);
    }
});

test('current-query contact follows the original capsule witness while retaining its force and action location',()=>{
    const f=fixture({edges:[['wire'],['wire']],positions:[[0,.8,0],[2,.6,0],[4,.1,0]],owners:[null,{owner:'wire',radius:1}]});
    f.wall.contactUpdate='current-query';const a=create(f);a.normalForces[0]=2;
    const before=measure(a,f),p=physical(f);p.get('wire')[2][1]=.9;
    const after=measure(a,f,{positions:p});
    assert.equal(before.certificate.samples[0].querySampleFraction,1);
    assert.equal(after.certificate.samples[0].querySampleFraction,0);
    assert.equal(a.normalForces[0],2);
    close(before.common[f.layout.positions[2]+1],after.common[f.layout.positions[1]+1]);
    close(after.common[f.layout.positions[2]+1],0);
    assert.equal(after.certificate.converged,false);
    f.wall.contactUpdate='frozen-chart';assert.throws(()=>measure(a,f),/Frozen wall source/);
});

test('declared local seam keeps two normal reactions on the actual blocked material point',async()=>{
    const {createCompositeAnatomyField}=await import('./helpers/compositeAnatomyField.js'),anatomy=createCompositeAnatomyField();
    try {
        const report=JSON.parse(fs.readFileSync(new URL('../reports/composite-material-point-rejected-step361.json',import.meta.url),'utf8')),
            origin=JSON.parse(fs.readFileSync(new URL('./fixtures/compositeNativeAppInitial.json',import.meta.url),'utf8')).context.sheaths[0].start,
            wire=new Map(report.result.diagnostics.rejectedConfiguration.positions).get('wire'),positions=[38,39,40].map(i=>wire[i].map((v,k)=>v+origin[k])),
            radius=JSON.parse(report.result.diagnostics.certificate.wall.samples.find(s=>s.edge===38).key)[2],p=positions[1],
            f=nodalFixture({field:anatomy.field,positions,owners:[{owner:'catheter',radius},{owner:'catheter',radius}]});
        f.wall.contactMode='material-points';f.wall.pressureDiscretization='fixed-material-points';f.wall.contactUpdate='current-query';
        f.wall.pressureSites=[{owner:'catheter',edge:0,fraction:0,trace:'right'},
            {owner:'catheter',edge:0,fraction:1,trace:'left',sdfSeam:{face:{axis:2,gridIndex:215},domainBox:{lower:p.map(v=>v-.02),upper:p.map(v=>v+.02)}}},
            {owner:'catheter',edge:1,fraction:1,trace:'left'}];
        const a=create(f);a.normalForces.set([0,2,3,0]);const result=measure(a,f);
        assert.equal(a.rows.length,4);assert.equal(result.certificate.domainAdmissible,false,'A nonselected branch cannot carry an accepted force off the seam');
        assert.deepEqual(a.surfaceRecords.map(s=>s.sdfBranch?.branchIndex??null),[null,0,1,null]);
        const samples=result.certificate.samples.filter(s=>s.edge===0&&s.endpointFraction===1);
        assert.equal(samples.length,2);assert.deepEqual(samples.map(s=>s.Fn),[2,3]);assert.notEqual(samples[0].gap,samples[1].gap);
        assert.equal(result.certificate.converged,false,'Both branch gaps still require a physical correction');
        const saved=a.checkpoint(),forces=a.normalForces.slice();a.normalForces[1]=0;assert.equal(measure(a,f).certificate.domainAdmissible,true);a.restore(saved);assert.deepEqual(a.normalForces,forces);
    } finally {anatomy.dispose();}
});

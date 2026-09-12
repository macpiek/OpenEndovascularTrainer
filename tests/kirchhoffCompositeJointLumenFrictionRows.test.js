import assert from 'node:assert/strict';
import test from 'node:test';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeJointLumenRows} from '../src/physics/kirchhoffCompositeJointLumenRows.js';
import {createCompositeJointLumenFrictionRows as create,createCompositeJointLumenFrictionWorkspace as workspace} from '../src/physics/kirchhoffCompositeJointLumenFrictionRows.js';
const close=(a,b,t=1e-9)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b} (${t})`);
const vec=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const tol={force:1e-7,lumenGap:1e-8,lumenNcp:1e-8,lumenWork:1e-8,frictionSlip:1e-8,frictionCone:1e-8,frictionWork:1e-8,frictionEquation:1e-9,linearConstraint:1e-10};

test('sequential friction managers reuse only the owned chart and bind fresh sample identities and force history',()=>{
    const w=workspace(),a=fixture({quadrature:[.25,.75]}),first=ready(a,{workspace:w});first.tractions.set([.1,-.2,-.08,.15]);refresh(a,first);
    const b=fixture({quadrature:[.25,.75],semantic:['new-wire','new-catheter']}),second=ready(b,{workspace:w}),cold=ready(b);
    assert.equal(w.diagnostics.chartBuilds,1);assert.equal(w.diagnostics.chartHits,1);
    assert.equal(w.diagnostics.support.builds,1);assert.equal(w.diagnostics.support.hits,3);
    assert.ok(second.tractions.every(v=>v===0));assert.throws(()=>first.commit(),/Stale/);
    assert.deepEqual(refresh(b,second),refresh(b,cold));assert.deepEqual(snapshotRows(second),snapshotRows(cold));
    const c=fixture();c.state.modes.forEach(m=>m.basis=[[1,0,0],[0,1,0],[0,0,1]]);
    const changed=ready(c,{workspace:w}),changedCold=ready(c);assert.equal(w.diagnostics.chartBuilds,2);
    assert.deepEqual(refresh(c,changed),refresh(c,changedCold));assert.deepEqual(snapshotRows(changed),snapshotRows(changedCold));
    c.state.layout.positions[0]+=1;assert.throws(()=>create({...c,workspace:w}),/Layout positions/);
    c.state.layout.positions[0]-=1;const retry=ready(c,{workspace:w});assert.deepEqual(refresh(c,retry),refresh(c,changedCold));
});
function fixture({quadrature=[.25],mu=[.3,.6],history=null,semantic=['wire:0','catheter:0'],state:given=null,rate=0}={}) {
    const dt=.02,layout=createCompositeChainLayout([['wire','catheter'],['wire','catheter']]),coordinates=[0,2,4];
    const points=new Map([['catheter',coordinates.map(x=>[x,0,0])],['wire',coordinates.map(x=>[x+.125,.25,0])]]);
    const modes=coordinates.map((_,node)=>({node,basis:[[.8,.6,0],[-.6,.8,0],[0,0,1]],relativeDofs:[3*node,3*node+1,3*node+2]}));
    const state=given??{layout,coordinates,modes,relativeToolId:'wire',toolPositions:points,angles:new Map([['wire',new Float64Array(2)],['catheter',new Float64Array(2)]]),
        relative:new Float64Array(9),tools:[...points].map(([id,p])=>({id,reference:captureCompositeReferenceFrames(p),dsDx:1}))};
    if(history)state.lumenFrictionState=structuredClone(history);
    const candidate=structuredClone(state),contacts={mode:'lumen-coulomb',chartId:'friction-rows',forcePerLength:1,
        friction:{law:'coulomb',mu,forcePerLength:5,materialPath:'linear-affine-maps'},
        pairs:[{id:'side0',innerToolId:'wire',outerToolId:'catheter',innerEdge:0,outerEdge:0,innerMaterialSegmentId:semantic[0],outerMaterialSegmentId:semantic[1],
            lumenRadius:.5,innerRadius:.25,quadrature,openDistal:false,portalFilletRadius:0}]};
    const normal=createCompositeJointLumenRows({...state,contacts:{...contacts,mode:'lumen-normal',friction:'none'},tolerances:tol});
    normal.normalForces.fill(2);normal.samples.forEach(s=>{if(s.redundant)normal.normalForces[s.index]=0;});
    const prepared={dt,previousPositions:structuredClone(state.toolPositions),inertiaEdges:layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,
        materialMap:{sStart:20+2*e+(id==='catheter'?10:0)+dt*rate,dsDx:1,dsDt:rate}}))}))};
    normal.prepareGauge({toolPositions:state.toolPositions,consumeQuery:()=>{throw Error('Unexpected normal gauge query');}});
    return {state,candidate,contacts,normal,prepared,dt,tolerances:tol,normalRowOffset:3,frictionRowOffset:7,queries:0};
}
function ready(f,extra={}) {const m=create({...f,...extra});m.prepare({consumeQuery:()=>f.queries++});return m;}
function refresh(f,m,{order='full',common=new Float64Array(f.state.layout.dofCount),relative=new Float64Array(f.candidate.relative.length),normal=true}={}) {
    if(normal)f.normal.refresh({toolPositions:f.candidate.toolPositions,commonResidual:new Float64Array(common.length),relativeResidual:new Float64Array(relative.length),order,consumeQuery:()=>f.queries++});
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

test('one/two original samples own distinct two-component rows, actual nodal forces/spin torques and original stick certificate',()=>{
    const f=fixture({quadrature:[.25,.75]}),before=structuredClone(f.state),w=workspace(),m=ready(f,{workspace:w});
    m.tractions.set([.1,-.2,-.08,.15]);const r=refresh(f,m);assert.equal(r.certificate.converged,true);assert.deepEqual(f.state,before);
    assert.equal(m.rows.length,4);assert.equal(m.tractions.length,4);assert.equal(m.diagnostics.preparationQueries,2);assert.equal(f.queries,4);
    assert.deepEqual(m.rows.map(r=>Array.from(r.multiplierDofs)),[[3,8],[3,7],[4,10],[4,9]]);
    const force=[0,0,0];for(const p of m.nodalForces.values())p.forEach(v=>v.forEach((x,k)=>force[k]+=x));vec(force,[0,0,0],1e-14);
    for(const [id,spins] of f.state.layout.spins)spins.forEach((d,e)=>close(r.common[d],-m.spinTorques.get(id)[e],1e-14));
    assert.ok(Math.abs(m.spinTorques.get('wire')[0])>1e-5);assert.ok(Math.abs(m.spinTorques.get('catheter')[0])>1e-5);
    for(const s of r.certificate.samples){close(s.work,0,0);close(s.minimumWork,0,0);close(s.workGap,0,0);close(s.coneViolation,0,0);vec(s.equationResidual,[0,0],1e-15);}
    const history=m.commit(),owned=structuredClone(history);m.tractions.fill(0);assert.deepEqual(history,owned);
    assert.equal(w.diagnostics.surfaceWorkspaceBuilds,1);assert.equal(w.diagnostics.equationWorkspaceBuilds,1);
});

for(const implicitRate of [false,true])test(`${implicitRate?'implicit fed':'finite'} sliding rows retain all primal G/B/DB and Fn/other-Ft derivatives in the physical support`,()=>{
    const f=fixture({rate:implicitRate?-50:0});
    if(implicitRate)Object.assign(f.contacts.friction,{rateMode:'backward-euler-grid',slipModel:'implicit-backward-euler-surface-rate',finiteStepSlipKnown:false});
    const m=ready(f);if(implicitRate)assert.equal(f.queries,0);
    f.candidate.toolPositions.get('wire').forEach(p=>p[0]+=.02);f.candidate.angles.get('wire')[0]=.3;f.candidate.angles.get('catheter')[0]=-.15;
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
        for(let c=0;c<2;c++){const expected=source===0?rows[c].cross[0]:source-1===c?rows[c].diagonal:rows[c].cross[1];close((answers[1][c]-answers[0][c])/(2*h),expected,3e-7);}
    }
});

test('signed private normal extension and infeasible zero-load Ft remain unclamped and cannot gain authority through mutated diagnostics',()=>{
    const f=fixture(),m=ready(f);f.candidate.angles.get('wire')[0]=.2;m.tractions.set([.2,-.1]);
    for(const Fn of [-1e-30,0]) {
        f.normal.normalForces[0]=Fn;const c=refresh(f,m).certificate;assert.equal(c.converged,false);assert.ok(Number.isFinite(c.lineSearchMerit));
        assert.equal(c.samples[0].Fn,Fn);vec(m.tractions,[.2,-.1],0);c.converged=true;c.samples[0].converged=true;
        assert.throws(()=>m.commit(),/unchanged certified/);
    }
    m.tractions.fill(0);const c=refresh(f,m).certificate;assert.equal(c.converged,true);m.commit();
});

test('failed current query/surface refresh revokes all rows and loads without partially publishing residuals, then retries identically',()=>{
    const f=fixture({quadrature:[.25,.75]}),m=ready(f);m.tractions.set([.1,.05,.05,-.1]);const first=refresh(f,m),expected=snapshotRows(m),saved=structuredClone(m.commit());
    const common=new Float64Array(f.state.layout.dofCount).fill(7),relative=new Float64Array(9).fill(9);
    f.normal.samples[1].geometry.rawContact.kind='distal-rim';assert.throws(()=>refresh(f,m,{normal:false,common,relative}),/fresh original/);
    assert.ok(common.every(v=>v===7));assert.ok(relative.every(v=>v===9));assert.ok(m.rows.every(r=>!r.geometricTangentValid&&r.jacobian.every(Number.isNaN)));
    assert.ok([...m.nodalForces.values()].flat(2).every(v=>v===0));assert.ok([...m.spinTorques.values()].every(v=>v.every(x=>x===0)));
    assert.throws(()=>m.commit(),/unchanged certified/);const retry=refresh(f,m);assert.deepEqual(retry,first);assert.deepEqual(snapshotRows(m),expected);assert.deepEqual(m.commit(),saved);
    f.candidate.toolPositions.get('wire')[0][0]+=.001;assert.throws(()=>refresh(f,m,{normal:false}),/fresh original/);
});

test('private commit checks positions, live own angles and forces, but permits root candidate reference-frame commit',()=>{
    const f=fixture(),m=ready(f);m.tractions.set([.1,-.1]);refresh(f,m);m.commit();
    f.candidate.tools[0].reference[0].director=[0,0,1];assert.doesNotThrow(()=>m.commit());
    f.candidate.angles.get('wire')[0]+=.001;assert.throws(()=>m.commit(),/unchanged certified/);f.candidate.angles.get('wire')[0]-=.001;
    m.tractions[0]+=.001;assert.throws(()=>m.commit(),/unchanged certified/);m.tractions[0]-=.001;
    f.normal.normalForces[0]+=.001;assert.throws(()=>m.commit(),/unchanged certified/);f.normal.normalForces[0]-=.001;
    refresh(f,m);const c=refresh(f,m,{order:'gradient'}).certificate;assert.equal(c.converged,true);assert.ok(m.rows.every(r=>!r.geometricTangentValid));m.commit();
    assert.ok(m.rows.every(r=>r.jacobian.every(Number.isNaN)&&r.geometricTangent.every(Number.isNaN)&&r.forceColumn.every(Number.isFinite)));
    refresh(f,m);assert.ok(m.rows.every(r=>r.geometricTangentValid&&r.jacobian.every(Number.isFinite)&&r.geometricTangent.every(Number.isFinite)));
});

test('accepted maps/Ft are owned across dt and numerical k changes; inconsistent labels or physical mu/chart cannot rebind history',()=>{
    const f=fixture(),w=workspace(),m=ready(f,{workspace:w});m.tractions.set([.1,-.1]);refresh(f,m);const history=m.commit(),saved=structuredClone(history);
    const next=fixture({history,rate:.2});next.contacts.friction.forcePerLength=50;const m2=ready(next,{workspace:w});
    assert.throws(()=>m.commit(),/Stale/);assert.deepEqual(history,saved);vec(m2.tractions,history.tractions,0);
    loadSliding(next,m2);const nextHistory=m2.commit();assert.equal(w.diagnostics.surfaceWorkspaceBuilds,1);
    const current=nextHistory.currentMaps.find(m=>m.id==='wire').currentMap;close(current.sStart,20+.02*.2,0);
    const bad=fixture({history});bad.prepared.inertiaEdges[0].tools[0].materialMap.sStart+=.01;assert.throws(()=>create(bad),/old material map disagrees/);
    assert.throws(()=>create(fixture({history,mu:[.31,.6]})),/chart\/coefficient/);
    const missing=structuredClone(history);missing.currentMaps.pop();assert.throws(()=>create(fixture({history:missing})),/old material map disagrees/);
});

test('old affine-map derivation supports nonuniform endpoint rates and typed semantic IDs without stale moving-foot scalars',()=>{
    const f=fixture({semantic:[17n,'17']}),mappings=f.prepared.inertiaEdges[0].tools;
    mappings[0].materialMap.dsDt=[-.2,.1];mappings[1].materialMap.dsDt=[.15,-.1];
    const m=ready(f);f.candidate.toolPositions.get('wire')[1][0]+=.002;loadSliding(f,m);const history=m.commit();
    assert.notEqual(history.currentMaps[0].edgeId,history.currentMaps[1].edgeId);assert.ok(history.currentMaps.some(t=>t.edgeId==='bigint:17'));
    const before=structuredClone(history);m.tractions[0]+=1;assert.deepEqual(history,before);
});

test('normal-only interior endpoint gauge, tip features and pressure transfers reject explicitly before mutating original data',()=>{
    const f=fixture({quadrature:[.25,.5,.75]}),before=structuredClone(f.state);f.normal.normalForces.set([0,1,0]);const forces=f.normal.normalForces.slice();
    assert.throws(()=>create(f),/interior normal-force endpoint gauge/);assert.deepEqual(f.normal.normalForces,forces);assert.deepEqual(f.state,before);
    const tip=fixture();tip.normal.samples[0].feature='distal-fillet';assert.throws(()=>create(tip),/tip features cannot be omitted/);
    const transfer=fixture();transfer.normal.gauge.transfers.push({Fn:1});assert.throws(()=>create(transfer),/endpoint gauge/);
});

test('previous query budget failures and invalid preparation never publish data or alter incoming history',()=>{
    const f=fixture({quadrature:[.25,.75]}),m=create(f),before=structuredClone(f.state);let queries=0;
    assert.throws(()=>m.prepare({consumeQuery:()=>{if(queries===1)throw Error('query-budget');queries++;}}),/query-budget/);
    assert.equal(queries,1);assert.deepEqual(f.state,before);assert.throws(()=>refresh(f,m),/original previous queries/);
    m.prepare({consumeQuery:()=>queries++});assert.equal(queries,3);assert.equal(refresh(f,m).certificate.converged,true);
    const invalid=fixture();invalid.prepared.inertiaEdges[0].tools[0].materialMap.dsDt=[0,1000];assert.throws(()=>create(invalid),/Old material slope/);
});

function openZero(f) {
    f.normal.normalForces.fill(0);
    f.candidate.toolPositions.get('wire').forEach(p=>p[1]=.2);
}
function assertZeroCone(m,result,order) {
    assert.equal(result.certificate.converged,true);assert.equal(result.certificate.merit,0);assert.equal(result.certificate.lineSearchMerit,0);
    assert.ok(result.common.every(v=>v===0));assert.ok(result.relative.every(v=>v===0));
    for(const s of result.certificate.samples){assert.equal(s.slip,null);assert.equal(s.slipRequired,false);assert.match(s.zeroConeProof,/deltaFn-zero-implies-deltaFt-zero/);
        for(const key of ['Fn','work','minimumWork','workGap','coneViolation','slipResidual'])assert.equal(s[key],0);assert.deepEqual(s.equationResidual,[0,0]);}
    for(const r of m.rows){assert.equal(r.knownZeroCone,true);assert.equal(r.residual,0);assert.equal(r.multiplierDerivative,.2);
        assert.ok(r.multiplierJacobian.every(v=>v===0));assert.ok(r.forceColumn.every(v=>v===0));assert.equal(r.geometricTangentValid,order==='full');
        assert.ok(r.jacobian.every(order==='full'?v=>v===0:Number.isNaN));assert.ok(r.geometricTangent.every(order==='full'?v=>v===0:Number.isNaN));}
    assert.ok([...m.nodalForces.values()].flat(2).every(v=>v===0));assert.ok([...m.spinTorques.values()].every(v=>v.every(x=>x===0)));
}

test('strict open exact zero cone eliminates only known-zero increments in full/value while retaining original queries and maps',()=>{
    const f=fixture({quadrature:[.25,.75],rate:.2}),before=structuredClone(f.state),w=workspace(),m=ready(f,{workspace:w});openZero(f);
    f.candidate.angles.get('wire')[0]=4*Math.PI;f.candidate.angles.get('catheter')[0]=-2*Math.PI;
    for(const order of ['full','gradient','full'])assertZeroCone(m,refresh(f,m,{order}),order);
    assert.equal(f.queries,8);assert.equal(m.diagnostics.surfaceEvaluations,0);assert.equal(m.diagnostics.zeroConeEliminations,6);
    assert.equal(w.diagnostics.zeroConeEliminations,6);assert.deepEqual(f.state,before);assert.deepEqual(Array.from(m.commit().tractions),[0,0,0,0]);
});
test('shared previous geometry scratch retains each sample normal and fresh edge eligibility on later configurations',()=>{
    const f=fixture({quadrature:[.25,.75]}),w=workspace();
    f.state.toolPositions.get('wire').forEach((p,i)=>{p[1]=.05;p[2]=-.16+.32*i;});
    for(const t of f.state.tools)t.reference=captureCompositeReferenceFrames(f.state.toolPositions.get(t.id));
    f.candidate.toolPositions=structuredClone(f.state.toolPositions);f.prepared.previousPositions=structuredClone(f.state.toolPositions);
    f.normal.normalForces.fill(0);const m=ready(f,{workspace:w}),first=refresh(f,m);assertZeroCone(m,first,'full');
    const [a,b]=f.normal.samples.map(s=>Array.from(s.geometry.normal));
    assert.ok(a.reduce((sum,v,k)=>sum+v*b[k],0)<.99,'The two old sample normals cannot substitute for one another');
    assert.equal(w.diagnostics.previousGeometryWorkspaceBuilds,1);
    const initial=structuredClone(f.candidate.toolPositions),angle=.3,c=Math.cos(angle),s=Math.sin(angle);
    for(const points of f.candidate.toolPositions.values())for(const p of points){const [x,y]=p;p[0]=c*x-s*y;p[1]=s*x+c*y;}
    const turned=refresh(f,m);assert.ok(turned.certificate.samples.every(s=>s.slipRequired));
    f.candidate.toolPositions=initial;assert.deepEqual(refresh(f,m),first);assertZeroCone(m,refresh(f,m),'full');
    assert.deepEqual(refresh(f,ready(f,{workspace:w})),first);
});

test('active Fn=0 has nonzero normal-to-Ft response; closed gap and any nonzero Fn/Ft retain their full original operator',()=>{
    const f=fixture(),m=ready(f);openZero(f);f.candidate.toolPositions.get('wire').forEach(p=>{p[0]+=.02;p[1]=.3;});f.candidate.angles.get('wire')[0]=.3;
    const active=refresh(f,m);assert.equal(f.normal.samples[0].active,true);assert.equal(f.normal.samples[0].row.multiplierDerivative,0);
    assert.equal(active.certificate.samples[0].slipRequired,true);assert.ok(m.rows.some(r=>Math.abs(r.multiplierJacobian[0])>1e-5));
    const deltaFn=.125,deltaFt=m.rows.map(r=>-r.multiplierJacobian[0]*deltaFn/r.multiplierDerivative);
    assert.ok(Math.hypot(...deltaFt)>1e-3);for(let c=0;c<2;c++)close(m.rows[c].multiplierDerivative*deltaFt[c]+m.rows[c].multiplierJacobian[0]*deltaFn,0,1e-16);
    assert.ok(m.rows.some(r=>r.forceColumn.some(v=>v!==0))); // discarding this block would alter the mechanical direction
    f.candidate.toolPositions.get('wire').forEach(p=>p[1]=.25);const closed=refresh(f,m);assert.equal(f.normal.samples[0].gap,0);assert.equal(closed.certificate.samples[0].slipRequired,true);
    openZero(f);
    f.normal.normalForces[0]=Number.MIN_VALUE;assert.throws(()=>refresh(f,m),/Unresolved nonzero friction ellipse radius/);
    assert.equal(m.diagnostics.zeroConeEliminations,0); // retain the original underflow rejection, never round this force to zero
    for(const [Fn,Ft] of [[1e-30,0],[0,Number.MIN_VALUE],[0,1e-30],[0,-1e-30]]){
        f.normal.normalForces[0]=Fn;m.tractions[0]=Ft;const c=refresh(f,m).certificate;
        assert.equal(c.samples[0].slipRequired,true);assert.equal(c.samples[0].Fn,Fn);assert.equal(c.samples[0].traction[0],Ft);assert.ok(m.rows.every(r=>r.knownZeroCone===false));
    }
    assert.equal(m.diagnostics.zeroConeEliminations,0);assert.equal(m.diagnostics.surfaceEvaluations,7);
});

test('activation release reactivation never reuses a reduced physical B or loses accepted history; cold and reused managers match',()=>{
    const f=fixture(),w=workspace(),m=ready(f,{workspace:w});m.tractions.set([.1,-.1]);const loaded=refresh(f,m),loadedRows=snapshotRows(m);assert.equal(loaded.certificate.converged,true);
    openZero(f);m.tractions.fill(0);const released=refresh(f,m);assertZeroCone(m,released,'full');const history=m.commit();
    f.candidate.toolPositions.get('wire').forEach(p=>p[1]=.25);f.normal.normalForces[0]=2;m.tractions.set([.1,-.1]);assert.deepEqual(refresh(f,m),loaded);assert.deepEqual(snapshotRows(m),loadedRows);
    openZero(f);m.tractions.fill(0);refresh(f,m);assert.deepEqual(m.commit(),history);
    const coldFixture=fixture({history}),warmFixture=fixture({history}),cold=ready(coldFixture),warm=ready(warmFixture,{workspace:w});openZero(coldFixture);openZero(warmFixture);
    assert.throws(()=>m.commit(),/Stale/);assert.deepEqual(refresh(coldFixture,cold),refresh(warmFixture,warm));assert.deepEqual(snapshotRows(cold),snapshotRows(warm));assert.deepEqual(cold.commit(),warm.commit());
});

test('known-zero proof cannot hide unsupported material transport or failed current query and identical retry restores it',()=>{
    const f=fixture({quadrature:[.25,.75]}),m=ready(f);openZero(f);const good=refresh(f,m),history=m.commit(),rows=snapshotRows(m);
    const common=new Float64Array(f.state.layout.dofCount).fill(3),relative=new Float64Array(9).fill(7);
    f.normal.samples[1].geometry.rawContact.kind='distal-rim';assert.throws(()=>refresh(f,m,{normal:false,common,relative}),/fresh original/);
    assert.ok(common.every(v=>v===3));assert.ok(relative.every(v=>v===7));assert.ok(m.rows.every(r=>!r.knownZeroCone&&r.jacobian.every(Number.isNaN)));
    assert.throws(()=>m.commit(),/unchanged certified/);assert.deepEqual(refresh(f,m),good);assert.deepEqual(snapshotRows(m),rows);assert.deepEqual(m.commit(),history);
    const crossing=fixture({rate:100}),before=structuredClone(crossing.state),crossingManager=ready(crossing);openZero(crossing);
    assert.throws(()=>refresh(crossing,crossingManager),/material|transport|cross/i);assert.deepEqual(crossing.state,before);
    assert.equal(crossingManager.diagnostics.zeroConeEliminations,0);assert.equal(crossingManager.diagnostics.surfaceEvaluations,1);
    assert.ok(crossingManager.rows.every(r=>!r.geometricTangentValid&&!r.knownZeroCone&&r.forceColumn.every(Number.isNaN)));
});

test('regularity eligibility only falls back: large supported contact-frame changes retain the original surface evaluation',()=>{
    const f=fixture(),m=ready(f);openZero(f);f.candidate.toolPositions.get('wire').forEach(p=>{p[1]=0;p[2]=.2;});
    const r=refresh(f,m);assert.equal(r.certificate.converged,true);assert.equal(r.certificate.samples[0].slipRequired,true);
    assert.equal(m.diagnostics.zeroConeEliminations,0);assert.equal(m.diagnostics.surfaceEvaluations,1);assert.ok(m.rows.every(r=>r.geometricTangentValid&&!r.knownZeroCone));
});

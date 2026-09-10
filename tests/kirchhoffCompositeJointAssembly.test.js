import assert from 'node:assert/strict';
import test from 'node:test';
import * as oracle from '../src/physics/kirchhoffCompositeElement.js';
import { assembleCompositeTranslationalInertia } from '../src/physics/kirchhoffCompositeKinematics.js';
import { createCompositeChainLayout } from '../src/physics/kirchhoffCompositeChain.js';
import { createCompositeJointAssembly, createCompositeJointAssemblyWorkspace, invalidateCompositeJointAssemblyWorkspace } from '../src/physics/kirchhoffCompositeJointAssembly.js';
import { createCompositeToolLengthWorkspace, evaluateCompositeToolLengths } from '../src/physics/kirchhoffCompositeToolLengths.js';
import { createCompositeRelativeDirectionWorkspace, solveCompositeRelativeDirection } from '../src/physics/kirchhoffCompositeRelativeDirection.js';

const close=(a,b,t=2e-9)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t*(1+Math.max(Math.abs(a),Math.abs(b))),`${a} != ${b}`);
const vectorClose=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
function fixture({count=5}={}) {
    const layout=createCompositeChainLayout(Array.from({length:count-1},(_,e)=>e<count-2?['wire','catheter']:['wire']));
    const coordinates=Array.from({length:count},(_,i)=>2*i),positions=coordinates.map((x,i)=>[x,.13*Math.sin(.4*i),.1*Math.cos(.6*i)]);
    const modes=Array.from({length:count-2},(_,i)=>({node:i+1,basis:[[.8,.6,0],[-.6,.8,0],[0,0,1]]}));
    const relative=Float64Array.from({length:3*modes.length},(_,i)=>.03*Math.sin(.6+i));
    const previousPositions=new Map(),angles=new Map(),tools=[];
    for(const id of layout.spins.keys()) {
        const wire=id==='wire',p=positions.map((p,i)=>p.map((v,k)=>v+(wire?.011*Math.cos(i+k):-.007*Math.sin(i+2*k))));
        previousPositions.set(id,p);
        angles.set(id,Float64Array.from({length:count-1},(_,i)=>layout.spins.get(id)[i]<0?NaN:.12*Math.sin(i+.3+(wire?0:1))));
        tools.push({id,reference:oracle.captureCompositeReferenceFrames(p),referenceTwists:Float64Array.from({length:count-2},()=>wire?2*Math.PI:0),dsDx:wire?1.2:.85,
            material:oracle.compileCompositeMaterial({stiffness:wire?[[2,.1,.2],[.1,4,-.1],[.2,-.1,1.5]]:[[12,-.2,.1],[-.2,16,.3],[.1,.3,9]],
                intrinsic:wire?[.01,-.02,.13]:[-.02,.03,-.06],energyOffset:.002})});
    }
    const inertia={dt:.06,previousPositions,inertiaEdges:layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,massPerMaterialLength:id==='wire'?.13:.24,
        materialMap:{sStart:20+2*e,dsDx:id==='wire'?1.2:.85,dsDt:id==='wire'?[-.8,.4]:[.3,-.2]},
        oldMaterialVelocities:id==='wire'?[[.2,-.1,.3],[.3,.1,-.2]]:[[-.1,.2,.1],[.2,-.3,.4]]}))}))};
    return {layout,coordinates,positions,modes,relative,angles,tools,inertia};
}
function physicalPositions(f,positions=f.positions,relative=f.relative) {
    const wire=positions.map(p=>[...p]);let offset=0;
    f.modes.forEach(m=>{m.basis.forEach((b,j)=>b.forEach((v,k)=>wire[m.node][k]+=v*relative[offset+j]));offset+=m.basis.length;});
    return new Map([['wire',wire],['catheter',positions.map(p=>[...p])]]);
}
// Original complete material energies and gradients, independent of joint
// packed scatter, selected relative stencils or diagnostic common blocks.
function independent(f,{positions=f.positions,relative=f.relative,angles=f.angles}={}) {
    const p=physicalPositions(f,positions,relative),g=new Float64Array(f.layout.dofCount+relative.length),wireGradient=Array.from({length:f.layout.nodeCount},()=>[0,0,0]),energies=[];
    const perTool=new Map();
    for(const tool of f.tools) {
        const x=p.get(tool.id),spins=f.layout.spins.get(tool.id);let energy=0;
        for(const h of f.layout.hinges)if(h.tools.includes(tool.id)) {
            const i=h.vertex,a=angles.get(tool.id),r=oracle.evaluateCompositeElement({positions:x.slice(i-1,i+2),reference:tool.reference.slice(i-1,i+1),
                referenceLength:(f.coordinates[i+1]-f.coordinates[i-1])/2,
                tools:[{angles:[a[i-1],a[i]],dsDx:tool.dsDx,referenceTwist:tool.referenceTwists[i-1],material:tool.material}]});
            energy+=r.energy;energies.push(r.energy);
            const ids=[i-1,i,i+1].flatMap(node=>Array.from({length:3},(_,k)=>f.layout.positions[node]+k)).concat([spins[i-1],spins[i]]);
            ids.forEach((d,j)=>g[d]+=r.gradient[j]);
            if(tool.id==='wire')for(let j=0;j<9;j++)wireGradient[i-1+Math.floor(j/3)][j%3]+=r.gradient[j];
        }
        if(f.inertia!==null)for(let e=0;e<f.layout.nodeCount-1;e++)if(spins[e]>=0) {
            const record=f.inertia.inertiaEdges[e].tools.find(t=>t.id===tool.id);
            const r=assembleCompositeTranslationalInertia({coordinates:f.coordinates.slice(e,e+2),positions:x.slice(e,e+2),previousPositions:f.inertia.previousPositions.get(tool.id).slice(e,e+2),dt:f.inertia.dt,tools:[record]});
            energy+=r.energy;energies.push(r.energy);
            for(let j=0;j<6;j++){g[f.layout.positions[e+Math.floor(j/3)]+j%3]+=r.gradient[j];if(tool.id==='wire')wireGradient[e+Math.floor(j/3)][j%3]+=r.gradient[j];}
        }
        perTool.set(tool.id,energy);
    }
    let offset=f.layout.dofCount;
    for(const m of f.modes)for(const b of m.basis)g[offset++]=b.reduce((sum,v,k)=>sum+v*wireGradient[m.node][k],0);
    return {energy:energies.reduce((s,v)=>s+v,0),energies,g,perTool};
}
function snapshot(output) {
    const {chain,cluster}=output,n=chain.gradient.length,r=cluster.relative.dofCount,N=n+r,g=Float64Array.from([...chain.gradient,...cluster.relative.gradient]),H=new Float64Array(N*N);
    for(let i=0;i<n;i++)for(let j=0;j<n;j++)if(Math.abs(i-j)<chain.layout.band)H[N*i+j]=chain.hessian[Math.max(i,j)*chain.layout.band+Math.abs(i-j)];
    for(let i=0;i<r;i++)for(let j=0;j<r;j++)if(Math.abs(i-j)<cluster.relative.band)H[N*(n+i)+n+j]=cluster.relative.hessian[Math.max(i,j)*cluster.relative.band+Math.abs(i-j)];
    const c=cluster.coupling;
    for(let row=0;row<c.commonDofs.length;row++)for(let i=c.rowOffsets[row];i<c.rowOffsets[row+1];i++){const a=c.commonDofs[row],b=n+c.columns[i];H[N*a+b]=H[N*b+a]=c.values[i];}
    return {g,H,N,energy:output.energy};
}
function displaced(f,col,h) {
    const positions=f.positions.map(p=>[...p]),relative=f.relative.slice(),angles=new Map([...f.angles].map(([id,a])=>[id,a.slice()]));
    if(col>=f.layout.dofCount)relative[col-f.layout.dofCount]+=h;
    else {
        const node=Array.from(f.layout.positions).findIndex(d=>col>=d&&col<d+3);
        if(node>=0)positions[node][col-f.layout.positions[node]]+=h;
        else for(const [id,indices] of f.layout.spins){const edge=indices.indexOf(col);if(edge>=0)angles.get(id)[edge]+=h;}
    }
    return {positions,relative,angles};
}

test('symbolic assembly reuse refreshes all physical data and matches cold E/g/H/kinetic/momentum exactly',()=>{
    const f=fixture(),workspace=createCompositeJointAssemblyWorkspace(f);
    const first=createCompositeJointAssembly(f,{workspace}),old=first.evaluate(f),arrays=[old.chain.gradient,old.chain.hessian,old.cluster.relative.gradient,old.cluster.relative.hessian,old.cluster.coupling.values];
    const initial=snapshot(old),stats=workspace.diagnostics;
    assert.deepEqual(initial,snapshot(createCompositeJointAssembly(f).evaluate(f)));
    const changed=fixture();changed.tools[0].material.stiffness[0]*=1.7;changed.tools[0].material.intrinsic[2]+=.021;changed.tools[0].material={...changed.tools[0].material,energyOffset:.005};
    for(const frame of changed.tools[0].reference) {
        const [x,y,z]=frame.tangent,[a,b,c]=frame.director,cross=[y*c-z*b,z*a-x*c,x*b-y*a];
        frame.director=frame.director.map((v,k)=>Math.cos(.17)*v+Math.sin(.17)*cross[k]);
    }
    changed.tools[0].referenceTwists[1]+=2*Math.PI;changed.relative[3]+=.016;
    for(const entry of changed.inertia.inertiaEdges)for(const t of entry.tools) {t.massPerMaterialLength*=1.13;t.materialMap.dsDt=[.7,-.4];t.oldMaterialVelocities[0][2]+=.23;}
    changed.inertia.dt*=.8;changed.inertia.previousPositions.get('wire')[2][1]+=.09;
    const second=createCompositeJointAssembly(changed,{workspace});assert.equal(old.hessianValid,false);assert.equal(old.cluster.operatorReady,false);
    assert.throws(()=>first.evaluate(f),/Stale/);
    const a=second.evaluate(changed),b=createCompositeJointAssembly(changed).evaluate(changed);
    assert.deepEqual(snapshot(a),snapshot(b));assert.deepEqual(a.perTool,b.perTool);assert.deepEqual(a.evaluatedReferenceTwists,b.evaluatedReferenceTwists);
    [a.chain.gradient,a.chain.hessian,a.cluster.relative.gradient,a.cluster.relative.hessian,a.cluster.coupling.values].forEach((v,i)=>assert.equal(v,arrays[i]));
    assert.equal(workspace.diagnostics.structureBuilds,1);assert.equal(workspace.diagnostics.scatterPlansBuilt,stats.scatterPlansBuilt);
    assert.notDeepEqual(snapshot(a),initial);invalidateCompositeJointAssemblyWorkspace(workspace);assert.equal(a.hessianValid,false);assert.throws(()=>second.evaluate(changed),/Stale/);
});

test('failed preparation and structural changes invalidate old handles; fresh retry never sees stale material state',()=>{
    const f=fixture(),workspace=createCompositeJointAssemblyWorkspace(f),first=createCompositeJointAssembly(f,{workspace});first.evaluate(f);
    const bad=fixture();bad.tools[0].material.stiffness[0]=NaN;
    assert.throws(()=>createCompositeJointAssembly(bad,{workspace}),/finite/);assert.throws(()=>first.evaluate(f),/Stale/);
    const retry=createCompositeJointAssembly(f,{workspace});assert.deepEqual(snapshot(retry.evaluate(f)),snapshot(createCompositeJointAssembly(f).evaluate(f)));
    assert.throws(()=>createCompositeJointAssembly({...f,coordinates:f.coordinates.map((x,i)=>x+(i===2?.1:0))},{workspace}),/Frozen/);
    assert.throws(()=>retry.evaluate(f),/Stale/);
    const different=fixture();different.modes[0].basis=[[1,0,0],[0,1,0],[0,0,1]];
    assert.throws(()=>createCompositeJointAssembly(different,{workspace}),/Frozen/);
    const recursive=fixture();recursive.tools[0].materialAt=()=>createCompositeJointAssembly(f,{workspace});
    assert.throws(()=>createCompositeJointAssembly(recursive,{workspace}),/busy/);
    assert.deepEqual(snapshot(createCompositeJointAssembly(f,{workspace}).evaluate(f)),snapshot(createCompositeJointAssembly(f).evaluate(f)));
});

test('reused scatter stencils do not freeze material providers or dynamic exact frame sharing',()=>{
    const f=fixture({count:9});f.modes=[f.modes[1]];f.relative=f.relative.slice(0,3);
    const workspace=createCompositeJointAssemblyWorkspace(f),ownReference=structuredClone(f.tools[1].reference),material=f.tools[0].material;
    let providerCalls=0,scale=1;
    f.tools[0].materialAt=()=>{providerCalls++;return {...material,stiffness:Float64Array.from(material.stiffness,v=>v*scale)};};
    for(let i=0;i<3;i++) {
        f.tools[1].reference=i===1?structuredClone(f.tools[0].reference):structuredClone(ownReference);scale=1+.2*i;
        const before=providerCalls,warm=createCompositeJointAssembly(f,{workspace}),after=providerCalls,a=warm.evaluate(f),cold=createCompositeJointAssembly(f).evaluate(f);
        assert.equal(after-before,7,'each current material hinge is compiled for this dt');
        assert.deepEqual(snapshot(a),snapshot(cold));assert.deepEqual(a.perTool,cold.perTool);
        assert.equal(a.statistics.sharedGeometryHinges,i===1?3:0);
    }
    assert.equal(workspace.diagnostics.structureBuilds,1);
});

test('finite relative displacement uses each material current geometry, history and constitutive/inertial energy exactly once',()=>{
    const f=fixture(),assembly=createCompositeJointAssembly(f),output=assembly.evaluate(f),expected=independent(f),a=snapshot(output);
    close(a.energy,expected.energy,1e-12);vectorClose(a.g,expected.g,2e-11);
    for(const [id,p] of output.perTool)close(p.elasticEnergy+p.inertialEnergy,expected.perTool.get(id),1e-12);
    assert.deepEqual(output.toolPositions,physicalPositions(f));
    assert.equal(output.statistics.elementEvaluations,5);assert.equal(output.statistics.inertiaEvaluations,7);
    assert.equal(output.certified,false);assert.equal(output.cluster.additionalInertia,null);
    const zero=independent(f,{relative:new Float64Array(f.relative.length)});
    assert.ok(Math.abs(zero.energy-expected.energy)>.01,'a zero-offset assembly cannot substitute for the current physical geometry');
});

test('the exact common/relative/spin tangent matches FD of independently summed nonlinear material gradients',()=>{
    const f=fixture(),assembly=createCompositeJointAssembly(f),base=snapshot(assembly.evaluate(f)),h=2e-6;
    for(let col=0;col<base.N;col++) {
        const plus=independent(f,displaced(f,col,h)),minus=independent(f,displaced(f,col,-h));
        const derivative=plus.energies.reduce((sum,e,i)=>sum+e-minus.energies[i],0)/(2*h);
        close(base.g[col],derivative,8e-8);
        for(let row=0;row<base.N;row++)close(base.H[base.N*row+col],(plus.g[row]-minus.g[row])/(2*h),8e-8);
    }
});

test('declared common-axis spans with identical frames evaluate geometry once without dropping either material or spin',()=>{
    const f=fixture({count:9});f.modes=[f.modes[1]];f.relative=f.relative.slice(0,3);
    f.tools[1].reference=structuredClone(f.tools[0].reference);
    f.inertia.previousPositions.set('catheter',structuredClone(f.inertia.previousPositions.get('wire')));
    const assembly=createCompositeJointAssembly(f),out=assembly.evaluate(f),base=snapshot(out),expected=independent(f);
    assert.equal(out.statistics.sharedGeometryHinges,3);assert.equal(out.statistics.materialHinges,13);assert.equal(out.statistics.elementEvaluations,10);
    vectorClose(base.g,expected.g,2e-11);close(base.energy,expected.energy,2e-12);
    for(const [id,p] of out.perTool)close(p.elasticEnergy+p.inertialEnergy,expected.perTool.get(id),2e-12);
    const h=2e-6;
    for(const col of [f.layout.positions[5]+1,f.layout.spins.get('wire')[5],f.layout.spins.get('catheter')[5],f.layout.dofCount+1]) {
        const plus=independent(f,displaced(f,col,h)),minus=independent(f,displaced(f,col,-h));
        for(let row=0;row<base.N;row++)close(base.H[base.N*row+col],(plus.g[row]-minus.g[row])/(2*h),8e-8);
    }
    const different=fixture({count:9});different.modes=[different.modes[1]];different.relative=different.relative.slice(0,3);
    const separate=createCompositeJointAssembly(different).evaluate(different);
    assert.equal(separate.statistics.sharedGeometryHinges,0);assert.equal(separate.statistics.elementEvaluations,13);
    vectorClose(snapshot(separate).g,independent(different).g,2e-11);
});

test('original finite material length rows enter the same band solve as both nonlinear material reactions',()=>{
    const f=fixture(),assembly=createCompositeJointAssembly(f),out=assembly.evaluate(f),p=physicalPositions(f);
    const restLengths=new Map([...p].map(([id,x])=>[id,Float64Array.from({length:f.layout.nodeCount-1},(_,e)=>Math.hypot(...x[e+1].map((v,k)=>v-x[e][k])))]));
    const lengths=createCompositeToolLengthWorkspace({layout:f.layout,modes:out.cluster.modes}),multipliers=Float64Array.from({length:lengths.rows.length},(_,i)=>.001*Math.sin(i));
    evaluateCompositeToolLengths({toolPositions:out.toolPositions,restLengths,multipliers,tolerance:1e-9},lengths);
    const w=createCompositeRelativeDirectionWorkspace(f.layout,out.cluster,lengths.rows),fixed=new Uint8Array(f.layout.dofCount);
    // Independent proximal spins are prescribed; material positions retain
    // their inertia. No artificial pin or stiffness is applied to rho.
    for(const [id,spin] of f.layout.spins)fixed[spin[0]]=1;
    const commonResidual=Float64Array.from(out.chain.gradient,(v,i)=>v+lengths.commonGradient[i]);
    const relativeResidual=Float64Array.from(out.cluster.relative.gradient,(v,i)=>v+lengths.relativeGradient[i]);
    const result=solveCompositeRelativeDirection(w,out.chain,{cluster:out.cluster,commonResidual,relativeResidual,fixed,rows:lengths.rows,tolerances:{force:1e-8,torque:1e-8},maxCorrections:2});
    assert.equal(result.converged,true);assert.ok(result.relativeIncrement.some(v=>Math.abs(v)>.001));
    assert.ok(w.count>f.layout.dofCount+f.relative.length,'both independent original material length rows occupy this one solve');
    lengths.rows[0].geometricTangentValid=false;
    assert.throws(()=>solveCompositeRelativeDirection(w,out.chain,{cluster:out.cluster,commonResidual,relativeResidual,fixed,rows:lengths.rows,
        tolerances:{force:1e-8,torque:1e-8},maxCorrections:2}),/fresh full physical geometric tangent/);
});

test('gradient-only evaluation does not touch poisoned Hessians and full evaluation rebuilds every physical block',()=>{
    const f=fixture(),assembly=createCompositeJointAssembly(f),out=assembly.evaluate(f),full=snapshot(out);
    const storage=[out.chain.hessian,out.cluster.common.hessian,out.cluster.relative.hessian,out.cluster.coupling.values];storage.forEach(a=>a.fill(NaN));
    const g=assembly.evaluate(f,{order:'gradient'});
    assert.equal(g,out);assert.equal(g.hessianValid,false);assert.equal(g.cluster.hessianValid,false);assert.ok(storage.every(a=>a.every(Number.isNaN)));
    assert.equal(g.energy,full.energy);assert.deepEqual(Float64Array.from([...g.chain.gradient,...g.cluster.relative.gradient]),full.g);
    assert.deepEqual(snapshot(assembly.evaluate(f)),full);
});

test('prepared material data and histories are owned and trial failure cannot leave a valid old tangent',()=>{
    const f=fixture(),assembly=createCompositeJointAssembly(f),base=snapshot(assembly.evaluate(f));
    f.coordinates.fill(100);f.tools.forEach(t=>{t.reference[0].director.fill(8);t.referenceTwists.fill(90);t.material.stiffness.fill(700);});
    f.inertia.previousPositions.get('wire')[0].fill(900);f.inertia.inertiaEdges[0].tools[0].oldMaterialVelocities[0].fill(99);
    f.modes[0].basis[0].fill(-2);
    assert.deepEqual(snapshot(assembly.evaluate(f)),base);
    assert.throws(()=>assembly.evaluate({...f,relative:new Float64Array([NaN])}),/relative coordinates/);
    assert.equal(assembly.cluster.hessianValid,false);assert.equal(assembly.cluster.operatorReady,false);
    assert.deepEqual(snapshot(assembly.evaluate(f)),base);
});

test('independent history, active spins and complete modes are explicit requirements',()=>{
    const f=fixture();
    assert.throws(()=>createCompositeJointAssembly({...f,inertia:undefined}),/explicit inertia/);
    assert.throws(()=>createCompositeJointAssembly({...f,tools:f.tools.map(t=>({...t,reference:undefined}))}),/own explicit/);
    assert.throws(()=>createCompositeJointAssembly({...f,modes:f.modes.map(m=>({...m,basis:m.basis.slice(1)}))}),/three-coordinate/);
    const assembly=createCompositeJointAssembly(f),angles=new Map(f.angles);angles.set('wire',new Float64Array(f.layout.nodeCount-1).fill(NaN));
    assert.throws(()=>assembly.evaluate({...f,angles}),/Finite current active/);
});

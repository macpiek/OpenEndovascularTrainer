import assert from 'node:assert/strict';
import test from 'node:test';
import * as oracle from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {assembleCompositeRelativePatch} from '../src/physics/kirchhoffCompositeRelativePatch.js';
import {assembleCompositeRelativeCluster as assemble,createCompositeRelativeClusterStructure as createStructure} from '../src/physics/kirchhoffCompositeRelativeCluster.js';
import {assembleCompositeTranslationalInertia} from '../src/physics/kirchhoffCompositeKinematics.js';
const close=(a,b,t=2e-10)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t*(1+Math.max(Math.abs(a),Math.abs(b))),`${a} != ${b}`);
const vectorClose=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],t));};
const H=(block,i,j)=>Math.abs(i-j)>=block.band?0:block.hessian[Math.max(i,j)*block.band+Math.abs(i-j)];
const C=(coupling,row,col)=>{for(let i=coupling.rowOffsets[row];i<coupling.rowOffsets[row+1];i++)if(coupling.columns[i]===col)return coupling.values[i];return 0;};
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const unit=v=>{const l=Math.hypot(...v);return v.map(x=>x/l);};
function fixture({count=11,curved=false,nodes=[4,5,6],dimensions=null}={}) {
    const rest=Array.from({length:count},(_,i)=>[2*i,curved?.17*Math.sin(i*.3):0,curved?.12*Math.cos(i*.27):0]);
    const positions=rest.map((p,i)=>p.map((v,j)=>v+(curved?.03*Math.sin(i*.2+j):0))),coordinates=Array.from({length:count},(_,i)=>2*i);
    const layout=createCompositeChainLayout(Array.from({length:count-1},()=>['wire','catheter']));
    const data={positions,coordinates,reference:oracle.captureCompositeReferenceFrames(rest),tools:[
        {id:'wire',angles:Float64Array.from({length:count-1},(_,i)=>curved?.13*Math.sin(i*.7):0),dsDx:1.3,
            referenceTwists:new Float64Array(count-2).fill(curved?2*Math.PI:0),material:oracle.compileCompositeMaterial(curved?
                {stiffness:[[3,.3,.2],[.3,5,-.1],[.2,-.1,2]],intrinsic:[.03,-.07,.13],energyOffset:.2}:{EI1:2,EI2:7,GJ:3})},
        {id:'catheter',angles:new Float64Array(count-1),material:oracle.compileCompositeMaterial({EI1:20,GJ:8})}]};
    const modes=nodes.map((node,index)=>{
        if(dimensions?.[index]===3)return {node,basis:[[1,0,0],[0,1,0],[0,0,1]]};
        const t=unit(positions[node+1].map((v,i)=>v-positions[node-1][i])),b=unit(cross([0,0,1],t));return{node,basis:[b,cross(t,b)]};});
    return {data,layout,modes};
}
// Independent complete summed wire energy/gradient: all original JS elements
// are evaluated, not the cluster's selected stencils or packed matrix.
function independent(f,rho=new Float64Array(f.modes.reduce((sum,m)=>sum+m.basis.length,0)),includeInertia=false) {
    const positions=f.data.positions.map(p=>[...p]);
    let offset=0;
    f.modes.forEach(mode=>{mode.basis.forEach((basis,axis)=>basis.forEach((v,k)=>positions[mode.node][k]+=v*rho[offset+axis]));offset+=mode.basis.length;});
    const wire=f.data.tools.find(t=>t.id==='wire'),spins=f.layout.spins.get('wire'),g=new Float64Array(f.layout.dofCount);
    let energy=0;const energies=[];
    for(const hinge of f.layout.hinges)if(hinge.tools.includes('wire')) {
        const i=hinge.vertex,x=f.data.coordinates,material=wire.materialAt?wire.materialAt({vertex:i,coordinate:x[i],start:(x[i-1]+x[i])/2,end:(x[i]+x[i+1])/2}):wire.material;
        const response=oracle.evaluateCompositeElement({positions:[positions[i-1],positions[i],positions[i+1]],reference:[f.data.reference[i-1],f.data.reference[i]],
            referenceLength:(x[i+1]-x[i-1])/2,tools:[{angles:[wire.angles[i-1],wire.angles[i]],referenceTwist:wire.referenceTwists?.[i-1]??0,
                dsDx:typeof wire.dsDx==='function'?wire.dsDx(x[i]):wire.dsDx??1,material}]});
        energy+=response.energy;energies.push(response.energy);
        const dofs=[i-1,i,i+1].flatMap(node=>[f.layout.positions[node],f.layout.positions[node]+1,f.layout.positions[node]+2]).concat([spins[i-1],spins[i]]);
        dofs.forEach((dof,j)=>g[dof]+=response.gradient[j]);
    }
    if(includeInertia)for(let e=0;e<positions.length-1;e++)if(spins[e]>=0) {
        const response=assembleCompositeTranslationalInertia({coordinates:f.data.coordinates.slice(e,e+2),positions:positions.slice(e,e+2),
            previousPositions:f.inertia.previousPositions.slice(e,e+2),dt:f.inertia.dt,tools:f.inertia.inertiaEdges[e].tools.filter(t=>t.id==='wire')});
        energy+=response.energy;energies.push(response.energy);for(let row=0;row<6;row++)g[f.layout.positions[e+Math.floor(row/3)]+row%3]+=response.gradient[row];
    }
    const relative=Float64Array.from(f.modes.flatMap(mode=>mode.basis.map(basis=>basis.reduce((sum,v,k)=>sum+v*g[f.layout.positions[mode.node]+k],0))));
    return {energy,energies,gradient:g,relative};
}
// Sum paired local energy differences before division; subtracting complete
// large prestressed energies first would create an avoidable FD roundoff floor.
const energyDerivative=(a,b,h)=>a.energies.reduce((sum,value,i)=>sum+(value-b.energies[i]),0)/(2*h);
function addInertia(f) {
    f.inertia={dt:.13,previousPositions:f.data.positions.map((p,i)=>p.map((v,j)=>v-.003*Math.cos(i+j))),
        inertiaEdges:f.layout.edgeToolIds.map((ids,e)=>({tools:ids.map(id=>({id,massPerMaterialLength:id==='wire'?2.4:1000,
            materialMap:{sStart:100+e,dsDx:id==='wire'?1.3:.8,dsDt:id==='wire'?[-.7,-2.1]:[.4,1.2]},
            oldMaterialVelocities:id==='wire'?[[.3,-.2,.1],[.8,.4,-.3]]:[[-.7,.1,.2],[-.4,-.1,.6]]}))}))};return f;
}

test('overlapping supports evaluate each affected wire hinge once and preserve sparse joint stencils',()=>{
    const f=fixture(),seen=[],material=f.data.tools[0].material;f.data.tools[0].materialAt=({vertex})=>(seen.push(vertex),material);
    const a=assemble(f);
    assert.deepEqual(a.affectedHinges,[3,4,5,6,7]);assert.deepEqual(seen,a.affectedHinges);assert.equal(new Set(seen).size,5);
    assert.equal(a.relativeDofCount,6);assert.ok(a.common.band<=11);assert.equal(a.relative.band,6);
    assert.ok(a.common.hessian.length<a.common.dofs.length**2);assert.ok(a.coupling.values.length<a.common.dofs.length*6);
    assert.equal(a.stencils.length,5);assert.equal(a.certified,false);assert.equal(a.condensed,false);assert.equal(a.hessianKind,'gauss-newton');
    assert.equal(a.commonContributionRole,'diagnostic-only-already-in-common');assert.ok(!('normalForce'in a));
});

test('single-mode cluster equals existing RelativePatch including all position and wire-spin coupling rows',()=>{
    for(const curved of [false,true]) {
        const f=fixture({curved,nodes:[5]}),a=assemble(f),p=assembleCompositeRelativePatch({...f,node:5,basis:f.modes[0].basis});
        assert.deepEqual(a.affectedHinges,p.affectedHinges);assert.deepEqual(a.common.dofs,p.dofs);close(a.energy,p.energy,2e-12);
        vectorClose(a.common.gradient,p.gradient,2e-12);vectorClose(a.relative.gradient,p.modeGradient,2e-12);
        for(let row=0;row<p.dofs.length;row++) {
            for(let col=0;col<p.dofs.length;col++)close(H(a.common,row,col),p.hessian[row*p.dofs.length+col],2e-12);
            for(let axis=0;axis<2;axis++)close(C(a.coupling,row,axis),p.coupling[2*row+axis],2e-12);
        }
        vectorClose([H(a.relative,0,0),H(a.relative,0,1),H(a.relative,1,1)],p.modeStiffness,2e-12);
    }
});

test('joint GN blocks independently pull back the affected common diagnostic matrix, including spin rows and cross-node modes',()=>{
    const f=fixture({curved:true}),a=assemble(f),indices=new Map(Array.from(a.common.dofs,(dof,index)=>[dof,index]));
    for(let r=0;r<a.relativeDofCount;r++) {
        const mode=f.modes[Math.floor(r/2)],basis=mode.basis[r%2],at=f.layout.positions[mode.node];
        close(a.relative.gradient[r],basis.reduce((sum,v,k)=>sum+v*a.common.gradient[indices.get(at+k)],0));
        for(let row=0;row<a.common.dofs.length;row++)close(C(a.coupling,row,r),basis.reduce((sum,v,k)=>sum+H(a.common,row,indices.get(at+k))*v,0));
        for(let s=0;s<a.relativeDofCount;s++) {
            const other=f.modes[Math.floor(s/2)],b=other.basis[s%2],otherAt=f.layout.positions[other.node];let value=0;
            for(let j=0;j<3;j++)for(let k=0;k<3;k++)value+=basis[j]*H(a.common,indices.get(at+j),indices.get(otherAt+k))*b[k];
            close(H(a.relative,r,s),value);
        }
    }
    const wireSpins=new Set(Array.from(f.layout.spins.get('wire')));
    assert.ok(Array.from(a.common.dofs).some((dof,row)=>wireSpins.has(dof)&&Array.from({length:6},(_,r)=>C(a.coupling,row,r)).some(v=>Math.abs(v)>1e-3)));
    const b=assemble({...f,elementBackend:'javascript'});vectorClose(a.common.gradient,b.common.gradient,2e-12);vectorClose(a.relative.hessian,b.relative.hessian,2e-12);
});

test('full exact cross-mode and common-relative Hessians match FD of independently summed wire energy/gradient, without mirror-derived references',()=>{
    const f=fixture({curved:true}),a=assemble({...f,elementBackend:'wasm-exact'}),base=independent(f),h=2e-6;
    vectorClose(a.relative.gradient,base.relative,2e-12);
    for(let col=0;col<6;col++) {
        const plus=new Float64Array(6),minus=new Float64Array(6);plus[col]=h;minus[col]=-h;
        const pa=independent(f,plus),pb=independent(f,minus);close(a.relative.gradient[col],energyDerivative(pa,pb,h),2e-8);
        for(let row=0;row<6;row++)close(H(a.relative,row,col),(pa.relative[row]-pb.relative[row])/(2*h),2e-8);
        for(let row=0;row<a.common.dofs.length;row++)close(C(a.coupling,row,col),(pa.gradient[a.common.dofs[row]]-pb.gradient[a.common.dofs[row]])/(2*h),2e-8);
    }
    const direction=[.3,-.2,.7,.1,-.4,.6],pa=independent(f,direction.map(v=>h*v)),pb=independent(f,direction.map(v=>-h*v));
    for(let row=0;row<6;row++)close(direction.reduce((sum,v,col)=>sum+H(a.relative,row,col)*v,0),(pa.relative[row]-pb.relative[row])/(2*h),2e-8);
});

test('zeroing cross-mode terms gives a measurably wrong simultaneous-mode response',()=>{
    const f=fixture(),a=assemble(f),v=[.3,-.2,.7,.1,-.4,.6],h=1e-6,pa=independent(f,v.map(x=>h*x)),pb=independent(f,v.map(x=>-h*x));
    let error=0;
    for(let row=0;row<6;row++) {
        const full=v.reduce((sum,x,col)=>sum+H(a.relative,row,col)*x,0),separate=v.reduce((sum,x,col)=>sum+(Math.floor(row/2)===Math.floor(col/2)?H(a.relative,row,col)*x:0),0);
        close(full,(pa.relative[row]-pb.relative[row])/(2*h),2e-8);error=Math.max(error,Math.abs(full-separate));
    }
    assert.ok(error>.5,'independent disks must miss a substantial physical cross-mode response');
    assert.ok(Math.abs(H(a.relative,0,2))>.1&&Math.abs(H(a.relative,0,4))>.1,'neighbor and second-neighbor modes share energy elements');
});

test('physical convective inertia contributes complete common/relative/cross blocks once, including neighboring material velocity coupling',()=>{
    const f=addInertia(fixture({curved:true})),a=assemble({...f,elementBackend:'wasm-exact'}),elastic=assemble({...f,elementBackend:'wasm-exact',inertia:null}),extra=a.additionalInertia;
    assert.deepEqual(a.affectedInertiaEdges,[3,4,5,6]);assert.equal(a.stencils.filter(s=>s.kind==='wire-inertia').length,4);
    close(a.energy,elastic.energy+extra.energy);close(a.inertialEnergy,extra.energy);
    for(const key of ['common','relative'])for(const field of ['gradient','hessian'])a[key][field].forEach((v,i)=>close(v,elastic[key][field][i]+extra[key][field][i]));
    // CSR patterns coincide because every inertia edge lies in an affected hinge.
    assert.deepEqual(a.coupling.columns,elastic.coupling.columns);a.coupling.values.forEach((v,i)=>close(v,elastic.coupling.values[i]+extra.coupling.values[i]));
    assert.ok(Math.abs(H(extra.relative,0,2))>1,'consistent inertia has cross-node terms');
    // Independently scatter every full original 6x6 edge inertia into a dense
    // TEST oracle, with no packed indexing or mirrored triangle reference.
    const N=f.layout.dofCount,expectedG=new Float64Array(N),expectedH=new Float64Array(N*N);let expectedEnergy=0;
    for(const e of a.affectedInertiaEdges) {
        const response=assembleCompositeTranslationalInertia({coordinates:f.data.coordinates.slice(e,e+2),positions:f.data.positions.slice(e,e+2),
            previousPositions:f.inertia.previousPositions.slice(e,e+2),dt:f.inertia.dt,tools:f.inertia.inertiaEdges[e].tools.filter(t=>t.id==='wire')});
        const dofs=[e,e+1].flatMap(node=>[f.layout.positions[node],f.layout.positions[node]+1,f.layout.positions[node]+2]);
        expectedEnergy+=response.energy;
        for(let i=0;i<6;i++){expectedG[dofs[i]]+=response.gradient[i];for(let j=0;j<6;j++)expectedH[dofs[i]*N+dofs[j]]+=response.hessian[i*6+j];}
    }
    close(extra.energy,expectedEnergy,2e-12);
    for(let row=0;row<a.common.dofs.length;row++) {
        close(extra.common.gradient[row],expectedG[a.common.dofs[row]],2e-12);
        for(let col=0;col<a.common.dofs.length;col++)close(H(extra.common,row,col),expectedH[a.common.dofs[row]*N+a.common.dofs[col]],2e-12);
    }
    const wireSpins=new Set(Array.from(f.layout.spins.get('wire')));
    a.common.dofs.forEach((dof,row)=>{if(wireSpins.has(dof)){close(extra.common.gradient[row],0);for(let col=0;col<6;col++)close(C(extra.coupling,row,col),0);}});
    // The larger FD step resolves small axial basis projections against the
    // nonzero convective momentum. The comparison tolerance remains2e-8.
    const h=2e-5,base=independent(f,undefined,true);vectorClose(a.relative.gradient,base.relative);
    for(let col=0;col<6;col++) {
        const plus=new Float64Array(6),minus=new Float64Array(6);plus[col]=h;minus[col]=-h;
        const pa=independent(f,plus,true),pb=independent(f,minus,true);close(a.relative.gradient[col],energyDerivative(pa,pb,h),2e-8);
        for(let row=0;row<6;row++)close(H(a.relative,row,col),(pa.relative[row]-pb.relative[row])/(2*h),2e-8);
        for(let row=0;row<a.common.dofs.length;row++)close(C(a.coupling,row,col),(pa.gradient[a.common.dofs[row]]-pb.gradient[a.common.dofs[row]])/(2*h),2e-8);
    }
    f.inertia.inertiaEdges.forEach(edge=>{edge.tools.find(t=>t.id==='catheter').massPerMaterialLength*=100;});
    const changed=assemble({...f,elementBackend:'wasm-exact'});vectorClose(a.relative.gradient,changed.relative.gradient,2e-12);vectorClose(a.relative.hessian,changed.relative.hessian,2e-12);
});

test('local support storage is independent of a distant wire extension and inputs/bases remain owned unchanged',()=>{
    const short=fixture(),long=fixture({count:201}),before=structuredClone(short),a=assemble(short),b=assemble(long);
    assert.deepEqual(short,before);assert.deepEqual(a.common.dofs,b.common.dofs);assert.deepEqual(a.affectedHinges,b.affectedHinges);
    vectorClose(a.relative.hessian,b.relative.hessian);assert.equal(a.coupling.values.length,b.coupling.values.length);
    short.modes[0].basis[0][1]=12;assert.notEqual(a.modes[0].basis[0][1],12);assert.ok(Object.isFrozen(a.modes[0].basis[0]));
    assert.throws(()=>assemble({...fixture(),modes:[fixture().modes[0],fixture().modes[0]]}),/unique/);
    const bad=fixture();bad.modes[0].basis=[[1,0,0],[0,0,1]];assert.throws(()=>assemble(bad),/transverse/);
    const broken=addInertia(fixture());broken.inertia.inertiaEdges[3].tools=broken.inertia.inertiaEdges[3].tools.filter(t=>t.id!=='wire');assert.throws(()=>assemble(broken),/exactly one/);
});

test('invalid material ownership, constitutive tensors and physical inertia fail instead of yielding a partial cluster',()=>{
    const f=fixture(),broken=fixture();broken.data.tools[0].material.stiffness[0]=Infinity;
    assert.throws(()=>assemble(broken),/Nonfinite/);
    assert.throws(()=>assemble({...f,modes:[{node:0,basis:[[0,1,0],[0,0,1]]}]}),/interior/);
    const missing=fixture();missing.layout.spins.get('wire')[4]=-1;assert.throws(()=>assemble(missing),/both sides/);
    const inertia=addInertia(fixture());inertia.inertia.inertiaEdges[3].tools[0].massPerMaterialLength=-1;
    assert.throws(()=>assemble(inertia),/positive/);
    assert.throws(()=>assemble({...fixture(),elementBackend:'unknown'}),/backend/);
});

test('many adjacent modes retain bounded bands and CSR row support instead of a dense cluster matrix',()=>{
    const f=fixture({count:35,nodes:Array.from({length:20},(_,i)=>i+3)}),a=assemble(f);
    assert.equal(a.relativeDofCount,40);assert.equal(a.relative.band,6);assert.equal(a.common.band,11);
    assert.ok(a.relative.hessian.length<40*40/4);
    for(let row=0;row<a.common.dofs.length;row++)assert.ok(a.coupling.rowOffsets[row+1]-a.coupling.rowOffsets[row]<=10);
    assert.equal(a.affectedHinges.length,22);assert.equal(new Set(a.affectedHinges).size,22);
});

test('three-coordinate and mixed 2/3 pullbacks match independent summed-energy FD with axial, spin and inertial coupling',()=>{
    for(const scenario of [
        {dimensions:[3,3,3],curved:false,elementBackend:'wasm',inertia:false},
        {dimensions:[3,3,3],curved:true,elementBackend:'wasm-exact',inertia:false},
        {dimensions:[2,3,2],curved:true,elementBackend:'wasm-exact',inertia:true},
    ]) {
        const f=fixture({curved:scenario.curved});if(scenario.inertia)addInertia(f);
        f.modes.forEach((m,i)=>{if(scenario.dimensions[i]===3)m.basis=[[1,0,0],[0,1,0],[0,0,1]];});
        const a=assemble({...f,elementBackend:scenario.elementBackend}),r=scenario.dimensions.reduce((sum,v)=>sum+v,0),h=scenario.inertia?2e-5:2e-6;
        assert.equal(a.relativeDofCount,r);assert.equal(a.fullRankOnRepresentedNodes,scenario.dimensions.every(d=>d===3));
        assert.equal(a.relativeRepresentation,scenario.dimensions.every(d=>d===3)?'full-rank-on-represented-nodes':'mixed-full-and-transverse-reduced');
        let next=0;a.modes.forEach((m,i)=>{assert.equal(m.dimension,scenario.dimensions[i]);assert.deepEqual(m.relativeDofs,Array.from({length:m.dimension},()=>next++));});
        const base=independent(f,undefined,scenario.inertia);vectorClose(a.relative.gradient,base.relative,2e-12);
        for(let col=0;col<r;col++) {
            const plus=new Float64Array(r),minus=new Float64Array(r);plus[col]=h;minus[col]=-h;
            const pa=independent(f,plus,scenario.inertia),pb=independent(f,minus,scenario.inertia);
            close(a.relative.gradient[col],energyDerivative(pa,pb,h),2e-8);
            for(let row=0;row<r;row++)close(H(a.relative,row,col),(pa.relative[row]-pb.relative[row])/(2*h),2e-8);
            for(let row=0;row<a.common.dofs.length;row++)close(C(a.coupling,row,col),(pa.gradient[a.common.dofs[row]]-pb.gradient[a.common.dofs[row]])/(2*h),2e-8);
        }
        const direction=Float64Array.from({length:r},(_,i)=>.3*Math.sin(.7*i+.4));
        const pa=independent(f,direction.map(v=>h*v),scenario.inertia),pb=independent(f,direction.map(v=>-h*v),scenario.inertia);
        for(let row=0;row<r;row++)close(direction.reduce((sum,v,col)=>sum+H(a.relative,row,col)*v,0),(pa.relative[row]-pb.relative[row])/(2*h),2e-8);
    }
});

test('a structure-only constructor evaluates no constitutive/inertia operator and owns full bases independently of the chord',()=>{
    const f=addInertia(fixture({curved:true}));f.modes.forEach(m=>{m.basis=[[0,0,1],[1,0,0],[0,1,0]];});
    const material=f.data.tools[0].material;let calls=0;
    f.data.tools[0].materialAt=()=>{calls++;return material;};
    const structure=createStructure({...f,elementBackend:'wasm-exact'});
    assert.equal(calls,0);assert.equal(structure.operatorReady,false);assert.equal(structure.hessianValid,false);
    assert.ok(structure.common.hessian.every(v=>v===0)&&structure.relative.hessian.every(v=>v===0)&&structure.coupling.values.every(v=>v===0));
    assert.equal(structure.fullRankOnRepresentedNodes,true);assert.equal(structure.relativeDofCount,9);
    const a=assemble({...f,elementBackend:'wasm-exact'});assert.equal(calls,a.affectedHinges.length);
    assert.equal(a.operatorReady,true);assert.equal(a.hessianValid,true);assert.deepEqual(structure.stencils,a.stencils);
    assert.deepEqual(structure.coupling.columns,a.coupling.columns);assert.deepEqual(structure.coupling.rowOffsets,a.coupling.rowOffsets);
    f.modes[0].basis[0][0]=.2;assert.equal(structure.modes[0].basis[0][0],0);assert.throws(()=>createStructure(f),/orthonormal/);
    const reduced=fixture();assert.equal(assemble(reduced).relativeRepresentation,'transverse-reduced');
    reduced.modes[0].basis=[[1,0,0],[0,0,1]];assert.throws(()=>createStructure(reduced),/transverse/);
    f.modes[0].basis=[[1,0,0],[0,1,0],[0,1,0]];assert.throws(()=>createStructure(f),/orthonormal/);
    const alone=fixture();alone.layout=createCompositeChainLayout(Array.from({length:alone.layout.nodeCount-1},()=>['wire']));alone.data.tools=alone.data.tools.slice(0,1);
    assert.equal(createStructure(alone).relativeRepresentation,'transverse-reduced','existing 2D compatibility stays explicit');
    alone.modes.forEach(m=>{m.basis=[[1,0,0],[0,1,0],[0,0,1]];});assert.throws(()=>createStructure(alone),/two materials/);
});

test('full three-coordinate modes retain local bands for many neighboring modes',()=>{
    const f=fixture({count:45,nodes:Array.from({length:30},(_,i)=>i+3)});f.modes.forEach(m=>{m.basis=[[1,0,0],[0,1,0],[0,0,1]];});
    const a=assemble(f);assert.equal(a.relativeDofCount,90);assert.equal(a.relative.band,9);assert.equal(a.common.band,11);
    for(let row=0;row<a.common.dofs.length;row++)assert.ok(a.coupling.rowOffsets[row+1]-a.coupling.rowOffsets[row]<=15);
    assert.ok(a.relative.hessian.length<90*90/8);assert.equal(a.fullRankOnRepresentedNodes,true);
});

test('the third axial coordinate permits independent inextensible lengths where two transverse coordinates cannot',()=>{
    const h=2,clearance=.0405,q=[[0,0,0],[h,0,0],[2*h,0,0]],offset=[0,clearance,0];
    const reduced=q.map((p,i)=>[p[0],offset[i],0]),shortening=Math.sqrt(h*h-clearance*clearance)-h;
    const complete=q.map((p,i)=>[p[0]+i*shortening,offset[i],0]);
    const length=(positions,edge)=>Math.hypot(...positions[edge+1].map((v,axis)=>v-positions[edge][axis]));
    for(let edge=0;edge<2;edge++) {
        close(length(q,edge),h,1e-13);assert.ok(length(reduced,edge)-h>4e-4);
        close(length(complete,edge),h,1e-13);
    }
    assert.ok(complete[2][0]-q[2][0]<-8e-4,'a free distal wire boundary must permit the required axial slip');
    // At the straight state the two original edge-length equations are
    // identical if only transverse rho is present. Adding axial rho gives
    // wire length derivative [dq1-dq0] + [da1-da0], independent of catheter.
    const catheterJacobian=[-1,1,0,0],wireJacobian=[-1,1,-1,1];
    assert.equal(catheterJacobian[0]*wireJacobian[2]-catheterJacobian[2]*wireJacobian[0],1);
});

test('global 3D endpoints and interior material start/tip modes match independent energy-gradient FD without ghost edges',()=>{
    for(const scenario of ['global-ends','material-ends']) {
        const nodes=scenario==='global-ends'?[0,6]:[1,4],f=fixture({count:7,curved:true,nodes,dimensions:[3,3]});
        if(scenario==='material-ends')f.layout=createCompositeChainLayout(Array.from({length:6},(_,e)=>e>=1&&e<4?['wire','catheter']:['catheter']));
        addInertia(f);
        // Reading a nonexisting endpoint edge is a bug, not missing material.
        const edges=f.inertia.inertiaEdges;f.inertia.inertiaEdges=new Proxy(edges,{get(target,key,receiver){
            if(key==='-1'||key==='6')throw new Error('ghost inertia edge was queried');return Reflect.get(target,key,receiver);
        }});
        const a=assemble({...f,elementBackend:'wasm-exact'}),h=2e-5;
        assert.deepEqual(a.affectedHinges,scenario==='global-ends'?[1,5]:[2,3]);
        assert.deepEqual(a.affectedInertiaEdges,scenario==='global-ends'?[0,5]:[1,3]);
        assert.equal(a.fullRankOnRepresentedNodes,true);assert.equal(a.relativeDofCount,6);
        for(let col=0;col<6;col++) {
            const plus=new Float64Array(6),minus=new Float64Array(6);plus[col]=h;minus[col]=-h;
            const pa=independent(f,plus,true),pb=independent(f,minus,true);
            close(a.relative.gradient[col],energyDerivative(pa,pb,h),2e-8);
            for(let row=0;row<6;row++)close(H(a.relative,row,col),(pa.relative[row]-pb.relative[row])/(2*h),2e-8);
            for(let row=0;row<a.common.dofs.length;row++)close(C(a.coupling,row,col),(pa.gradient[a.common.dofs[row]]-pb.gradient[a.common.dofs[row]])/(2*h),2e-8);
        }
        const active=a.affectedInertiaEdges[0];edges[active].tools=edges[active].tools.filter(t=>t.id!=='wire');
        assert.throws(()=>createStructure(f),/exactly one prepared relative material/,'missing active material inertia remains an error');
    }
});

test('a one-edge relative material gets only its physical inertia support and preserves reduced 2D guards',()=>{
    const f=fixture({count:6,nodes:[2,3],dimensions:[3,3]});
    f.layout=createCompositeChainLayout(Array.from({length:5},(_,e)=>e===2?['wire','catheter']:['catheter']));
    assert.throws(()=>createStructure(f),/no affected wire energy support/);
    addInertia(f);const a=assemble(f);
    assert.deepEqual(a.affectedHinges,[]);assert.deepEqual(a.affectedInertiaEdges,[2]);assert.equal(a.stencils.length,1);
    assert.ok(a.relative.hessian.some(v=>v!==0));assert.equal(a.elasticEnergy,0);assert.ok(a.inertialEnergy>0);
    const reduced=fixture();
    assert.throws(()=>createStructure({...reduced,modes:[{node:0,basis:[[0,1,0],[0,0,1]]}]}),/interior/);
    assert.throws(()=>createStructure({...f,modes:[{node:2,basis:[[0,1,0],[0,0,1]]}]}),/both sides/);
    assert.throws(()=>createStructure({...f,modes:[{node:0,basis:[[1,0,0],[0,1,0],[0,0,1]]}]}),/active incident edge/);
});

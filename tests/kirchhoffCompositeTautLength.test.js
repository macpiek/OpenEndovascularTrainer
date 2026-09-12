import test from 'node:test';
import assert from 'node:assert/strict';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {createCompositeContinuousGeometry} from '../src/physics/kirchhoffCompositeContinuousGeometry.js';
import {createCompositeToolLengthWorkspace} from '../src/physics/kirchhoffCompositeToolLengths.js';
import {createCompositeTautLengthBlock as create} from '../src/physics/kirchhoffCompositeTautLength.js';

const ids=['catheter','wire'],cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],
    close=(a,b,t=1e-9)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}`);
function fixture(count=4,fixed=[1,2]) {
    const geometry=createCompositeContinuousGeometry({coordinates:Array.from({length:count},(_,j)=>j)}),
        layout=createCompositeChainLayout(Array.from({length:count-1},()=>ids),{positionSupports:geometry.edges.map(e=>e.nodeIndices)}),
        modes=Array.from({length:count},(_,node)=>({node,basis:[[0,1,0],[0,0,1],[1,0,0]],relativeDofs:[3*node,3*node+1,3*node+2]})),
        p=new Map(ids.map(id=>[id,Array.from({length:count},(_,j)=>[j,id==='wire'?.04:0,0])])),
        lengths=createCompositeToolLengthWorkspace({layout,modes,geometryByTool:new Map(ids.map(id=>[id,geometry]))}),
        args={lengthRows:lengths.rows,positionBoundaries:new Map(ids.flatMap(toolId=>fixed.map(node=>[JSON.stringify([toolId,node]),{toolId,node,value:p.get(toolId)[node].slice()}]))),
            restLengths:new Map(ids.map(id=>[id,new Float64Array(count-1).fill(1)])),tolerance:1e-9,linearTolerance:1e-11};
    return {layout,modes,p,lengths,args};
}
function evaluate(f,block,p,order='full') {
    const n=f.layout.dofCount,r=3*f.layout.nodeCount,cg=new Float64Array(n),rg=new Float64Array(r),N=n+r,H=new Float64Array(N*N),
        certificate=block.refresh({toolPositions:p,commonResidual:cg,relativeResidual:rg,order});
    if(order==='full')for(const row of block.rows) {
        const indices=[...row.commonDofs,...Array.from(row.relativeDofs,j=>n+j)],size=indices.length;
        for(let i=0;i<size;i++)for(let j=0;j<size;j++)H[indices[i]*N+indices[j]]+=row.geometricTangent[i*size+j];
    }
    return {work:block.rows.reduce((sum,row)=>sum+row.lambda*row.residual,0),g:Float64Array.from([...cg,...rg]),H,N,certificate};
}
function perturb(f,p,column,h) {
    const common=f.layout.dofCount;
    if(column>=common){const d=column-common,node=Math.floor(d/3),basis=f.modes[node].basis[d%3];basis.forEach((v,k)=>p.get('wire')[node][k]+=h*v);return;}
    for(let node=0;node<f.layout.nodeCount;node++){const k=column-f.layout.positions[node];if(k>=0&&k<3){for(const id of ids)p.get(id)[node][k]+=h;return;}}
    // A spin has no contribution to a geometric length constraint.
}

test('taut normal-form work and complete signed tangents agree with independent differences, including both prescribed endpoints and rotated relative bases',()=>{
    const f=fixture(),block=create(f.args);assert.equal(block.rows.length,8);assert.equal(block.replaced.size,2);
    block.rows.forEach((row,j)=>row.lambda=.2*(j-3));f.p.get('wire')[0][2]=.02;f.p.get('catheter')[3][1]=-.01;
    const original=evaluate(f,block,f.p),h=1e-6;
    for(let j=0;j<original.N;j++) {
        const plus=structuredClone(f.p),minus=structuredClone(f.p);perturb(f,plus,j,h);perturb(f,minus,j,-h);
        const a=evaluate(f,block,plus,'gradient'),b=evaluate(f,block,minus,'gradient');
        close((a.work-b.work)/(2*h),original.g[j],2e-9);
        for(let i=0;i<original.N;i++)close((a.g[i]-b.g[i])/(2*h),original.H[i*original.N+j],2e-9);
    }
    assert.ok(block.rows.every(row=>!row.geometricTangentValid));
    const fresh=evaluate(f,block,f.p);assert.deepEqual(fresh.H,original.H);
});

test('feasible taut reactions have zero resultant and moment, retain axial freedom and preserve world-vector warm starts',()=>{
    const f=fixture(),block=create(f.args);block.rows.forEach((row,j)=>row.lambda=.13*(j-2));
    for(const id of ids){f.p.get(id)[0][0]=-.03;f.p.get(id)[3][0]=3.02;}
    const r=evaluate(f,block,f.p);assert.equal(r.certificate.converged,true);
    for(const row of block.rows) {
        const force=[0,0,0],moment=[0,0,0],p=f.p.get(row.toolId);
        row.nodes.forEach((node,j)=>{const F=[0,1,2].map(k=>row.lambda*row.J[3*j+k]);F.forEach((v,k)=>force[k]+=v);cross(p[node],F).forEach((v,k)=>moment[k]+=v);});
        force.forEach(v=>close(v,0,1e-14));moment.forEach(v=>close(v,0,1e-14));
    }
    const restored=create({...f.args,history:block.commit()});assert.deepEqual(evaluate(f,restored,f.p).g,r.g);
    f.p.get('wire')[0][1]+=.001;assert.equal(evaluate(f,restored,f.p).certificate.converged,false);
});

test('only exact taut endpoints are reparameterized; polynomial rank, prescribed shape and duplicate line constraints are checked',()=>{
    const f=fixture();for(const rest of f.args.restLengths.values())rest[1]+=1e-12;
    assert.equal(create(f.args).rows.length,0,'arbitrarily small true slack must not be locked');
    const repeated=fixture(5,[0,1,3,4]),block=create(repeated.args);
    assert.equal(block.replaced.size,4);assert.equal(block.rows.length,4,'coincident prescribed lines share a reaction gauge at their common free node');
    for(const id of ids)for(const node of [3,4])repeated.args.positionBoundaries.get(JSON.stringify([id,node])).value[1]+=.1;
    assert.throws(()=>create(repeated.args),/incompatible intersections/);
    const conflict=fixture(4,[0,1,2]);conflict.args.positionBoundaries.get(JSON.stringify(['wire',2])).value[1]=.08;
    assert.throws(()=>create(conflict.args),/conflict/);
    assert.throws(()=>create({...f.args,tolerance:NaN}),/tolerances/);
});

test('world-vector reactions rotate with a declared taut line and remain independent of the transverse chart basis',()=>{
    const f=fixture(),block=create(f.args);block.rows.forEach((row,j)=>row.lambda=.13*(j-2));evaluate(f,block,f.p);
    const forces=b=>{
        const out=new Map(ids.map(id=>[id,Array.from({length:4},()=>[0,0,0])]));
        for(const row of b.rows)row.nodes.forEach((node,j)=>[0,1,2].forEach(k=>out.get(row.toolId)[node][k]+=row.lambda*row.J[3*j+k]));
        return out;
    },before=forces(block),R=[[.36,-.8,-.48],[.48,.6,-.64],[.8,0,.6]],rotate=v=>R.map(row=>row.reduce((sum,x,k)=>sum+x*v[k],0)),offset=[4,-3,2],rotated=fixture();
    for(const id of ids)rotated.p.set(id,f.p.get(id).map(p=>rotate(p).map((v,k)=>v+offset[k])));
    for(const b of rotated.args.positionBoundaries.values())b.value=rotated.p.get(b.toolId)[b.node].slice();
    // The declaration uses the represented endpoint metric. Floating point
    // subtraction after a world translation need not reproduce literal 1.
    for(const id of ids)rotated.args.restLengths.get(id)[1]=Math.hypot(...rotated.p.get(id)[2].map((v,k)=>v-rotated.p.get(id)[1][k]));
    const after=create({...rotated.args,history:block.commit().map(r=>({...r,normalMultiplier:rotate(r.normalMultiplier)}))});
    assert.equal(after.rows.length,block.rows.length);evaluate(rotated,after,rotated.p);const actual=forces(after);
    for(const id of ids)before.get(id).forEach((F,node)=>rotate(F).forEach((v,k)=>close(actual.get(id)[node][k],v,2e-14)));
});

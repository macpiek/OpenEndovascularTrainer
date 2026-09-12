import test from 'node:test';
import assert from 'node:assert/strict';
import {createCompositeChainLayout} from '../src/physics/kirchhoffCompositeChain.js';
import {captureCompositeReferenceFrames,compileCompositeMaterial} from '../src/physics/kirchhoffCompositeElement.js';
import {createCompositeJointAssembly,createCompositeJointAssemblyWorkspace} from '../src/physics/kirchhoffCompositeJointAssembly.js';
import {createCompositeJointTimeStepState,createCompositeJointTimeStepWorkspace,advanceCompositeJointTimeStep as advance} from '../src/physics/kirchhoffCompositeJointTimeStep.js';
import {createCompositeJointMaterialHistory} from '../src/physics/kirchhoffCompositeJointMaterialHistory.js';
import {evaluateCompositeContinuousGeometry} from '../src/physics/kirchhoffCompositeContinuousGeometry.js';
import {createCompositeToolLengthWorkspace} from '../src/physics/kirchhoffCompositeToolLengths.js';
import {createCompositeRelativeDirectionWorkspace} from '../src/physics/kirchhoffCompositeRelativeDirection.js';

const close=(a,b,t=1e-8)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=t,`${a} != ${b}; tol ${t}`);
const same=(a,b,t)=>{assert.equal(a.length,b.length);a.forEach((v,j)=>close(v,b[j],t));};
const dt=1/120,ids=['wire','catheter'],zero=[0,0,0];
const accepted=r=>assert.equal(r.accepted,true,JSON.stringify({status:r.status,error:r.error,certificate:r.diagnostics?.certificate}));
function fixture({coordinates=[0,1,2.4,3.5,5.1,6],tip=coordinates.length-1,interfaces=[]}={}) {
    const n=coordinates.length,
        layout=createCompositeChainLayout(coordinates.slice(1).map((_,e)=>e<tip?ids:['wire'])),
        positions=coordinates.map((x,node)=>[x,node>tip?.03:0,0]),wire=coordinates.map(x=>[x,.03,0]),
        modes=coordinates.flatMap((_,node)=>node<=tip?[{node,basis:[[1,0,0],[0,1,0],[0,0,1]]}]:[]),
        tools=ids.map(id=>({id,dsDx:1,reference:captureCompositeReferenceFrames(id==='wire'?wire:positions),referenceTwists:new Float64Array(n-2),
            material:compileCompositeMaterial({EI1:id==='wire'?2:9,EI2:id==='wire'?3:12,GJ:id==='wire'?1:4})})),
        state=createCompositeJointTimeStepState({layout,coordinates,positions,tools,modes,relative:modes.flatMap(()=>[0,.03,0]),
            angles:new Map(ids.map(id=>[id,new Float64Array(n-1)])),restLengths:new Map(ids.map(id=>[id,coordinates.slice(1).map((x,j)=>x-coordinates[j])])),
            inertiaGeometryByTool:new Map(ids.map(id=>[id,{interfaces:id==='catheter'?interfaces:[]}])),materialCoordinate:'reference-arclength'});
    return {state,input:{dt,torsionMode:'quasi-static',contacts:'none',inertia:inertiaFor(state),
        boundaries:{positions:[],spins:ids.map(toolId=>({toolId,edge:0,value:0}))}}};
}
function inertiaFor(state,velocity={wire:zero,catheter:zero},feed={wire:0,catheter:0},shift={wire:0,catheter:0}) {
    return {dt,previousPositions:structuredClone(state.toolPositions),inertiaEdges:state.layout.edgeToolIds.map((tools,e)=>({tools:tools.map(id=>({id,
        massPerMaterialLength:id==='wire'?.13:.24,materialMap:{sStart:20+state.coordinates[e]+shift[id],dsDx:1,dsDt:feed[id]},
        oldMaterialVelocities:[velocity[id].slice(),velocity[id].slice()]}))}))};
}
function packed(output) {
    const {chain,cluster}=output,n=chain.gradient.length,r=cluster.relative.dofCount,N=n+r,H=new Float64Array(N*N),
        g=Float64Array.from([...chain.gradient,...cluster.relative.gradient]);
    for(let a=0;a<n;a++)for(let b=0;b<n;b++)if(Math.abs(a-b)<chain.layout.band)H[a*N+b]=chain.hessian[Math.max(a,b)*chain.layout.band+Math.abs(a-b)];
    for(let a=0;a<r;a++)for(let b=0;b<r;b++)if(Math.abs(a-b)<cluster.relative.band)H[(n+a)*N+n+b]=cluster.relative.hessian[Math.max(a,b)*cluster.relative.band+Math.abs(a-b)];
    const c=cluster.coupling;
    for(let row=0;row<c.commonDofs.length;row++)for(let at=c.rowOffsets[row];at<c.rowOffsets[row+1];at++) {
        const a=c.commonDofs[row],b=n+c.columns[at];H[a*N+b]=H[b*N+a]=c.values[at];
    }
    return {energy:output.energy,g,H,N};
}
function perturb(state,j,h) {
    if(j>=state.layout.dofCount){state.relative[j-state.layout.dofCount]+=h;return;}
    for(let node=0;node<state.positions.length;node++)if(j>=state.layout.positions[node]&&j<state.layout.positions[node]+3){state.positions[node][j-state.layout.positions[node]]+=h;return;}
    for(const [id,spin] of state.layout.spins){const edge=spin.indexOf(j);if(edge>=0){state.angles.get(id)[edge]+=h;return;}}
    throw Error('Unknown coordinate');
}

test('continuous inertia assembles every wider common/relative term and its full nonlinear tangent matches independent finite differences',()=>{
    const {state,input}=fixture({tip:4}),args={...state,inertia:input.inertia};
    for(const entry of args.inertia.inertiaEdges)for(const t of entry.tools){
        delete t.oldMaterialVelocities;
        t.oldVelocityPieces=[{fractions:[0,1],interpretation:'quintic-bernstein-material-velocity',bernsteinVelocities:Array.from({length:6},(_,j)=>[.001*j*j,.01*j,-.004*j])}];
        t.materialMap.dsDt=t.id==='wire'?[-.4,.2]:[.3,-.1];
    }
    const assembly=createCompositeJointAssembly(args),current=structuredClone(state);
    current.positions[2][1]+=.012;current.relative[6]+=.004;current.angles.get('wire')[2]=.05;
    const original=packed(assembly.evaluate(current)),h=1e-6;
    assert.ok(assembly.layout.band>createCompositeChainLayout(state.layout.edgeToolIds).band);
    const p=assembly.layout.positions;
    assert.ok(Math.abs(original.H[p[0]*original.N+p[3]])>1e-6,'four-node inertia coefficient must survive the native hinge band');
    for(let col=0;col<original.N;col++) {
        const plus=structuredClone(current),minus=structuredClone(current);perturb(plus,col,h);perturb(minus,col,-h);
        const a=packed(assembly.evaluate(plus)),b=packed(assembly.evaluate(minus));
        close((a.energy-b.energy)/(2*h),original.g[col],2e-7);
        for(let row=0;row<original.N;row++)close((a.g[row]-b.g[row])/(2*h),original.H[row*original.N+col],3e-6);
    }
    const workspace=createCompositeJointAssemblyWorkspace(args),warm=createCompositeJointAssembly(args,{workspace});
    assert.deepEqual(packed(warm.evaluate(current)),original);
    const changed={...args,inertia:structuredClone(args.inertia)};changed.inertia.inertiaEdges[0].tools[0].massPerMaterialLength*=2;
    assert.deepEqual(packed(createCompositeJointAssembly(changed,{workspace}).evaluate(current)),packed(createCompositeJointAssembly(changed).evaluate(current)));
    assert.equal(workspace.diagnostics.structureBuilds,1);
});

test('tool tips and declared interfaces bound support; a changed chart cannot reuse a stale scatter pattern',()=>{
    const {state,input}=fixture({tip:4,interfaces:[2]}),cat=state.inertiaGeometryByTool.get('catheter');
    assert.deepEqual(cat.interfaces,[2,4]);
    assert.deepEqual(cat.edges[3].nodeIndices,[2,3,4]);
    for(const e of [0,1])assert.ok(cat.edges[e].nodeIndices.every(node=>node<=2));
    const args={...state,inertia:input.inertia};
    for(const entry of args.inertia.inertiaEdges)for(const t of entry.tools){t.oldVelocityPieces=[{fractions:[0,1],oldMaterialVelocities:t.oldMaterialVelocities}];delete t.oldMaterialVelocities;}
    const workspace=createCompositeJointAssemblyWorkspace(args),assembly=createCompositeJointAssembly(args,{workspace}),first=structuredClone(assembly.evaluate(state).perTool.get('catheter')),
        current=structuredClone(state);current.positions[5][1]=.7;
    assert.deepEqual(assembly.evaluate(current).perTool.get('catheter'),first);
    const bad={...args,inertiaGeometryByTool:new Map(ids.map(id=>[id,{interfaces:[]}]))};
    assert.throws(()=>createCompositeJointAssembly(bad,{workspace}),/Frozen/);
    assert.throws(()=>assembly.evaluate(state),/Stale/);
    assert.deepEqual(createCompositeJointAssembly(args,{workspace}).evaluate(state).perTool.get('catheter'),first);
    assert.throws(()=>createCompositeJointTimeStepState({...state,inertiaGeometryByTool:new Map([['wire',{}]])}),/every material/);
});

test('one joint step preserves independent translations, opposite feed and independent rotation with polynomial accepted histories',()=>{
    const f=fixture(),grid={wire:[.04,-.02,.01],catheter:[-.03,.01,-.02]},feed={wire:-.2,catheter:.15},
        physical=Object.fromEntries(ids.map(id=>[id,grid[id].map((v,k)=>v-(k===0?feed[id]:0))]));
    f.input.inertia=inertiaFor(f.state,physical,feed);f.input.boundaries.spins[0].value=.3;f.input.boundaries.spins[1].value=-.2;
    const result=advance(f.state,f.input);accepted(result);
    for(const id of ids){
        f.state.toolPositions.get(id).forEach((p,j)=>same(result.state.toolPositions.get(id)[j],p.map((v,k)=>v+dt*grid[id][k]),2e-10));
        for(const entry of result.state.materialVelocities){const record=entry.tools.find(t=>t.id===id);assert.equal(record.velocities,undefined);record.bernsteinVelocities.forEach(v=>same(v,physical[id],1e-9));}
        same(result.balances.get(id).residual,zero,3e-7);
        result.state.angles.get(id).forEach(v=>close(v,id==='wire'?.3:-.2,2e-10));
    }
    assert.equal(result.diagnostics.inertiaGeometry,'continuous-quintic');assert.equal(result.diagnostics.lengthGeometry,'native-chords');
});

test('nonlinear bending commits full polynomial velocity then transports opposite feeds across old nodes into the next joint solve',()=>{
    const f=fixture(),workspace=createCompositeJointTimeStepWorkspace(f.state);
    f.input.boundaries.positions=ids.map(toolId=>({toolId,node:0,value:f.state.toolPositions.get(toolId)[0]}));
    f.input.loads={forces:[{toolId:'wire',node:5,value:[0,.012,.006]},{toolId:'catheter',node:4,value:[0,-.02,.004]}],torques:[{toolId:'wire',edge:4,value:.001}]};
    const first=advance(f.state,{...f.input,workspace});accepted(first);
    const history=createCompositeJointMaterialHistory({materialVelocities:first.state.materialVelocities,reservoir:({toolId})=>({id:toolId,sStart:0,sEnd:100,
        velocities:[zero,zero],interpretation:'physical-material-velocity'})}),feed={wire:-.2,catheter:.15},shift={wire:-.2*dt,catheter:.15*dt},
        inertia=inertiaFor(first.state,undefined,feed,shift),transported=history.prepare({coordinates:first.state.coordinates,inertiaEdges:inertia.inertiaEdges});
    const expected=new Map(ids.map(id=>[id,[0,0,0]]));
    for(let e=0;e<inertia.inertiaEdges.length;e++)for(const t of inertia.inertiaEdges[e].tools){
        const old=transported.inertiaEdges[e].tools.find(v=>v.id===t.id);delete t.oldMaterialVelocities;
        t.oldVelocityPieces=old.pieces.map(p=>({fractions:p.fractions,...(p.bernsteinVelocities?{interpretation:p.interpretation,bernsteinVelocities:p.bernsteinVelocities}:{oldMaterialVelocities:p.oldMaterialVelocities})}));
        for(const piece of old.pieces)for(const sample of piece.samples)sample.oldMaterialVelocity.forEach((v,k)=>expected.get(t.id)[k]+=t.massPerMaterialLength*sample.materialWeight*v);
    }
    assert.ok(transported.requiredCuts.some(c=>c.toolId==='wire'));assert.ok(transported.requiredCuts.some(c=>c.toolId==='catheter'));
    const input={...f.input,inertia,workspace},before=structuredClone(first.state),limited=advance(first.state,{...input,budget:{directions:0}});
    assert.equal(limited.accepted,false);assert.deepEqual(first.state,before);
    const second=advance(first.state,input),cold=advance(first.state,{...input,workspace:null});accepted(second);accepted(cold);
    assert.deepEqual(second.state,cold.state);assert.equal(second.state.step,2);
    for(const id of ids){same(second.perTool.get(id).oldMomentum,expected.get(id),1e-12);same(second.balances.get(id).residual,zero,3e-7);}
    for(const entry of second.state.materialVelocities)for(const t of entry.tools){
        const g=second.state.inertiaGeometryByTool.get(t.id).edges[entry.edge],record=inertia.inertiaEdges[entry.edge].tools.find(v=>v.id===t.id),
            evaluated=evaluateCompositeContinuousGeometry(g,{positions:g.nodeIndices.map(node=>second.state.toolPositions.get(t.id)[node]),
                previousPositions:g.nodeIndices.map(node=>first.state.toolPositions.get(t.id)[node]),dt,materialMap:record.materialMap,fraction:.37});
        const nextHistory=createCompositeJointMaterialHistory({materialVelocities:second.state.materialVelocities});
        same(nextHistory.sample(t.id,t.sStart+.37*(t.sEnd-t.sStart)),evaluated.materialVelocity,1e-11);
    }
    assert.equal(workspace.diagnostics.assembly.structureBuilds,1);
});

test('polynomial history cannot be downgraded to affine inertia or certified against the old contact geometry',()=>{
    const f=fixture(),record=f.input.inertia.inertiaEdges[0].tools[0];delete record.oldMaterialVelocities;
    record.oldVelocityPieces=[{fractions:[0,1],interpretation:'quintic-bernstein-material-velocity',bernsteinVelocities:Array.from({length:6},()=>zero.slice())}];
    const affine=createCompositeJointTimeStepState({...f.state,inertiaGeometryByTool:null});
    assert.throws(()=>advance(affine,f.input),/requires continuous/);
    assert.throws(()=>advance(f.state,{...f.input,wall:{mode:'wall-normal'}}),/matching curved contact/);
    const bad=structuredClone(f.input);bad.inertia.inertiaEdges[0].tools[0].oldVelocityPieces[0].bernsteinVelocities.pop();
    assert.throws(()=>advance(f.state,bad),/six velocity/);
});

test('continuous frame elasticity and inertia solve both tools together through consecutive loaded steps with separate rotation and exact retry',()=>{
    const f=fixture({coordinates:[0,1,2]});
    f.state=createCompositeJointTimeStepState({...f.state,elasticityGeometry:'continuous-material-frame'});
    f.input.boundaries.positions=ids.map(toolId=>({toolId,node:0,value:f.state.toolPositions.get(toolId)[0]}));
    f.input.boundaries.spins=ids.map(toolId=>({toolId,edge:0,value:toolId==='wire'?.03:-.02}));
    f.input.loads={forces:[{toolId:'wire',node:2,value:[0,.008,.003]},{toolId:'catheter',node:2,value:[0,-.012,.004]}]};
    const workspace=createCompositeJointTimeStepWorkspace(f.state),before=structuredClone(f.state),first=advance(f.state,{...f.input,workspace});accepted(first);
    assert.deepEqual(f.state,before);assert.equal(first.diagnostics.elasticityGeometry,'continuous-material-frame');
    assert.equal(first.diagnostics.inertiaGeometry,'continuous-quintic');assert.equal(first.diagnostics.lengthGeometry,'native-chords');
    assert.ok(first.state.toolPositions.get('wire')[2][1]>f.state.toolPositions.get('wire')[2][1]);
    assert.ok(first.state.toolPositions.get('catheter')[2][1]<f.state.toolPositions.get('catheter')[2][1]);
    for(const id of ids){same(first.balances.get(id).residual,zero,3e-7);close(first.state.angles.get(id)[0],id==='wire'?.03:-.02,1e-12);}
    const history=createCompositeJointMaterialHistory({materialVelocities:first.state.materialVelocities}),inertia=inertiaFor(first.state),
        prepared=history.prepare({coordinates:first.state.coordinates,inertiaEdges:inertia.inertiaEdges});
    for(let e=0;e<inertia.inertiaEdges.length;e++)for(const t of inertia.inertiaEdges[e].tools){
        delete t.oldMaterialVelocities;t.oldVelocityPieces=prepared.inertiaEdges[e].tools.find(v=>v.id===t.id).pieces;
    }
    const input={...f.input,inertia,workspace},rejected=advance(first.state,{...input,budget:{directions:0}});
    assert.equal(rejected.accepted,false);assert.equal(rejected.state,first.state);
    const second=advance(first.state,input),cold=advance(first.state,{...input,workspace:null});accepted(second);accepted(cold);
    assert.deepEqual(second.state,cold.state);assert.deepEqual(second.balances,cold.balances);assert.equal(second.state.step,2);
    for(const id of ids)same(second.balances.get(id).residual,zero,3e-7);
    assert.equal(workspace.diagnostics.assembly.structureBuilds,1);
    assert.equal(workspace.diagnostics.assembly.continuousFrames.arenaAllocations,1,'one WASM derivative tape for gradient/full orders, both tools and all steps');
});

test('continuous elastic position/spin support enters the same band and CSR and matches finite differences across its wider coupling',()=>{
    const f=fixture({tip:4});f.state=createCompositeJointTimeStepState({...f.state,elasticityGeometry:'continuous-material-frame'});
    const args={...f.state,inertia:structuredClone(f.input.inertia)};
    for(const entry of args.inertia.inertiaEdges)for(const t of entry.tools){t.oldVelocityPieces=[{fractions:[0,1],oldMaterialVelocities:t.oldMaterialVelocities}];delete t.oldMaterialVelocities;}
    const current=structuredClone(f.state);current.positions[2][1]+=.03;current.positions[3][2]+=.04;current.angles.get('wire')[1]=.1;
    const assembly=createCompositeJointAssembly(args),r=packed(assembly.evaluate(current)),position=f.state.layout.positions,
        remote=position[0],far=position[4];
    assert.ok(Math.abs(r.H[remote*r.N+far])>1e-8||Math.abs(r.H[(remote+1)*r.N+far+1])>1e-8,'wider physical elastic terms cannot be truncated to the inertia stencil');
    const j=far+1,h=1e-6,plus=structuredClone(current),minus=structuredClone(current);perturb(plus,j,h);perturb(minus,j,-h);
    const a=packed(assembly.evaluate(plus,{order:'gradient'})),b=packed(assembly.evaluate(minus,{order:'gradient'}));
    for(let row=0;row<r.N;row++)close((a.g[row]-b.g[row])/(2*h),r.H[row*r.N+j],3e-6);
    const workspace=createCompositeJointAssemblyWorkspace(args);
    assert.deepEqual(packed(createCompositeJointAssembly(args,{workspace}).evaluate(current)),r);
    assert.throws(()=>createCompositeJointAssembly({...args,elasticityGeometry:'native-discrete-rod'},{workspace}),/Frozen/);
    assert.throws(()=>createCompositeJointTimeStepState({...f.state,inertiaGeometryByTool:null}),/requires the same/);
    const split=fixture({tip:4,interfaces:[2]});
    assert.throws(()=>createCompositeJointTimeStepState({...split.state,elasticityGeometry:'continuous-material-frame'}),/orientation-continuous/);
});

function bowedFixture() {
    const height=.12,arc=.5*Math.sqrt(1+4*height*height)+Math.asinh(2*height)/(4*height),f=fixture({coordinates:[0,arc,2*arc]}),
        positions=[[0,0,0],[1,height,0],[2,0,0]],tools=f.state.tools.map(t=>{
            const own=positions.map(p=>p.map((v,k)=>v+(t.id==='wire'&&k===1?.03:0))),material=structuredClone(t.material);
            return {...t,reference:captureCompositeReferenceFrames(own,[0,0,1]),materialAt:({coordinate})=>({...material,
                intrinsic:[-2*height/(1+4*height*height*(1-coordinate/arc)**2)/arc,0,0]})};
        });
    f.state=createCompositeJointTimeStepState({...f.state,positions,tools,elasticityGeometry:'continuous-material-frame',lengthGeometry:'continuous-arclength'});
    f.input.inertia=inertiaFor(f.state);return f;
}
// Independent integration of the exact quadratic through three equally
// spaced nodes. This does not use the solver's Bernstein basis or Gauss rule.
function quadraticLength(p,edge) {
    const A=p[0].map((v,k)=>(p[2][k]-2*p[1][k]+v)/2),B=p[0].map((v,k)=>p[1][k]-v-A[k]),segments=1024;
    let sum=0;for(let j=0;j<=segments;j++){const x=edge+j/segments,speed=Math.hypot(...A.map((v,k)=>2*v*x+B[k]));sum+=(j===0||j===segments?1:j%2?4:2)*speed;}
    return sum/(3*segments);
}

test('continuous material frame, inertia and arclength close in one loaded solve and preserve independent curved lengths through a second dt',()=>{
    const f=bowedFixture(),workspace=createCompositeJointTimeStepWorkspace(f.state),before=structuredClone({...f.state,tools:f.state.tools.map(({materialAt,...t})=>t)});
    f.input.boundaries.positions=ids.flatMap(toolId=>[0,1].map(node=>({toolId,node,value:f.state.toolPositions.get(toolId)[node]})));
    f.input.loads={forces:[{toolId:'wire',node:2,value:[0,.004,.002]},{toolId:'catheter',node:2,value:[0,-.003,-.002]}]};
    const first=advance(f.state,{...f.input,workspace});accepted(first);
    assert.equal(first.diagnostics.lengthGeometry,'continuous-arclength');assert.equal(first.diagnostics.retainedLengthRows,4);
    assert.deepEqual(first.diagnostics.suppressedPrescribedLengthRows,[],'two prescribed endpoints cannot discard neighboring curve support');
    assert.deepEqual({...f.state,tools:f.state.tools.map(({materialAt,...t})=>t)},before);
    for(const id of ids) {
        const p=first.state.toolPositions.get(id);for(let e=0;e<2;e++){
            close(quadraticLength(p,e),first.state.restLengths.get(id)[e],1e-8);
            assert.ok(Math.hypot(...p[e+1].map((v,k)=>v-p[e][k]))<first.state.restLengths.get(id)[e]-1e-4);
        }
        same(first.balances.get(id).residual,zero,3e-7);
    }
    assert.equal(first.diagnostics.certificate.pointwiseInextensibility,false);assert.ok(first.diagnostics.certificate.parameterMetricDeviationBound>0);
    const inertia=inertiaFor(first.state),history=createCompositeJointMaterialHistory({materialVelocities:first.state.materialVelocities}),
        prepared=history.prepare({coordinates:first.state.coordinates,inertiaEdges:inertia.inertiaEdges});
    for(let e=0;e<inertia.inertiaEdges.length;e++)for(const t of inertia.inertiaEdges[e].tools){delete t.oldMaterialVelocities;t.oldVelocityPieces=prepared.inertiaEdges[e].tools.find(v=>v.id===t.id).pieces;}
    const input={...f.input,inertia,workspace},limited=advance(first.state,{...input,budget:{directions:0}});assert.equal(limited.accepted,false);assert.equal(limited.state,first.state);
    const second=advance(first.state,input),cold=advance(first.state,{...input,workspace:null});accepted(second);accepted(cold);assert.deepEqual(second.state,cold.state);
    for(const id of ids){for(let e=0;e<2;e++)close(quadraticLength(second.state.toolPositions.get(id),e),second.state.restLengths.get(id)[e],1e-8);same(second.balances.get(id).residual,zero,3e-7);}
});

test('fully prescribed continuous support suppresses only redundant length duals and checks curved targets before acceptance',()=>{
    const f=bowedFixture();f.input.boundaries.positions=ids.flatMap(toolId=>f.state.toolPositions.get(toolId).map((value,node)=>({toolId,node,value})));
    const result=advance(f.state,f.input);accepted(result);assert.equal(result.diagnostics.retainedLengthRows,0);assert.equal(result.diagnostics.suppressedPrescribedLengthRows.length,4);
    const bad=structuredClone(f.input.boundaries);bad.positions.find(p=>p.toolId==='wire'&&p.node===1).value[1]+=.002;
    assert.throws(()=>advance(f.state,{...f.input,boundaries:bad}),/conflict/);
    const workspace=createCompositeJointTimeStepWorkspace(f.state),different=createCompositeJointTimeStepState({...f.state,lengthGeometry:'native-chords'});
    assert.throws(()=>advance(different,{...f.input,workspace}),/length geometry changed/);
    assert.throws(()=>createCompositeJointTimeStepState({...f.state,elasticityGeometry:'native-discrete-rod',inertiaGeometryByTool:null}),/Continuous lengths/);
});

test('four-node continuous length rows use explicitly declared wider support while arbitrary distant rows remain rejected',()=>{
    const f=fixture({coordinates:[0,1,2,3]});f.state=createCompositeJointTimeStepState({...f.state,lengthGeometry:'continuous-arclength'});
    const result=advance(f.state,f.input);accepted(result);
    const inertia=structuredClone(f.input.inertia);for(const entry of inertia.inertiaEdges)for(const t of entry.tools){t.oldVelocityPieces=[{fractions:[0,1],oldMaterialVelocities:t.oldMaterialVelocities}];delete t.oldMaterialVelocities;}
    const assembly=createCompositeJointAssembly({...f.state,inertia}),lengths=createCompositeToolLengthWorkspace({layout:assembly.layout,modes:assembly.cluster.modes,geometryByTool:f.state.inertiaGeometryByTool}),
        wide=lengths.rows.findIndex(row=>row.nodeIndices.length===4);
    assert.ok(wide>=0);createCompositeRelativeDirectionWorkspace(assembly.layout,assembly.cluster,lengths.rows);
    const untagged=lengths.rows.slice();untagged[wide]={...untagged[wide],constraintSupport:undefined};
    assert.throws(()=>createCompositeRelativeDirectionWorkspace(assembly.layout,assembly.cluster,untagged),/two-edge/);
    const wrong=lengths.rows.slice();wrong[wide]={...wrong[wide],constraintSupport:{...wrong[wide].constraintSupport,nodeIndices:[0,1,3]}};
    assert.throws(()=>createCompositeRelativeDirectionWorkspace(assembly.layout,assembly.cluster,wrong),/exact declared/);
});

test('a loaded straight continuous root with two prescribed nodes advances both tools, retains arc lengths and balances its regular taut reactions',()=>{
    const f=fixture({coordinates:[0,1,2,3,4,5]});
    f.state=createCompositeJointTimeStepState({...f.state,elasticityGeometry:'continuous-material-frame',lengthGeometry:'continuous-arclength'});
    f.input.boundaries.positions=ids.flatMap(toolId=>[0,1].map(node=>({toolId,node,value:f.state.toolPositions.get(toolId)[node]})));
    f.input.loads={forces:[{toolId:'wire',node:5,value:[0,.004,.002]},{toolId:'catheter',node:5,value:[0,-.003,-.002]}]};
    const workspace=createCompositeJointTimeStepWorkspace(f.state),before=structuredClone(f.state),first=advance(f.state,{...f.input,workspace});accepted(first);
    assert.deepEqual(f.state,before);assert.equal(first.diagnostics.tautLengthRows,4);assert.equal(first.diagnostics.reparameterizedTautLengths.length,2);
    assert.ok(first.diagnostics.certificate.taut.converged);assert.equal(first.diagnostics.certificate.taut.axialMultiplierGauge,0);
    assert.equal(first.state.tautLengthState.length,2);assert.ok(first.state.tautLengthState.some(r=>r.normalMultiplier.some(v=>Math.abs(v)>1e-8)));
    for(const id of ids) {
        same(first.balances.get(id).residual,zero,3e-7);
        close(first.state.toolPositions.get(id)[2][1],f.state.toolPositions.get(id)[2][1],1e-10);
        assert.ok(Math.abs(first.state.toolPositions.get(id)[5][1]-f.state.toolPositions.get(id)[5][1])>1e-8,'the remote rod must still bend');
        for(let edge=0;edge<5;edge++) {
            const geometry=first.state.inertiaGeometryByTool.get(id).edges[edge],positions=geometry.nodeIndices.map(node=>first.state.toolPositions.get(id)[node]);let length=0;
            for(let j=0;j<=128;j++){const q=evaluateCompositeContinuousGeometry(geometry,{positions,fraction:j/128});length+=(j===0||j===128?1:j%2?4:2)*Math.hypot(...q.positionDx);}
            close(length/384,first.state.restLengths.get(id)[edge],1e-8);
        }
    }
    const inertia=inertiaFor(first.state),history=createCompositeJointMaterialHistory({materialVelocities:first.state.materialVelocities}),
        prepared=history.prepare({coordinates:first.state.coordinates,inertiaEdges:inertia.inertiaEdges});
    for(let e=0;e<inertia.inertiaEdges.length;e++)for(const t of inertia.inertiaEdges[e].tools){delete t.oldMaterialVelocities;t.oldVelocityPieces=prepared.inertiaEdges[e].tools.find(v=>v.id===t.id).pieces;}
    const input={...f.input,inertia,workspace,boundaries:{...f.input.boundaries,spins:ids.map(toolId=>({toolId,edge:0,value:toolId==='wire'?.03:-.02}))}},
        snapshot=structuredClone(first.state),rejected=advance(first.state,{...input,budget:{directions:0}});
    assert.equal(rejected.accepted,false);assert.equal(rejected.state,first.state);assert.deepEqual(first.state,snapshot);
    const second=advance(first.state,input),cold=advance(first.state,{...input,workspace:null});accepted(second);accepted(cold);
    assert.deepEqual(second.state,cold.state);assert.deepEqual(second.boundaryForces,cold.boundaryForces);assert.deepEqual(first.state,snapshot);
    for(const id of ids){same(second.balances.get(id).residual,zero,3e-7);close(second.state.angles.get(id)[0],id==='wire'?.03:-.02,1e-14);}
    const own=createCompositeJointTimeStepState(second.state);own.tautLengthState[0].normalMultiplier[0]+=1;
    assert.notDeepEqual(own.tautLengthState,second.state.tautLengthState);
    assert.throws(()=>createCompositeJointTimeStepState({...second.state,tautLengthState:[...second.state.tautLengthState,second.state.tautLengthState[0]]}),/distinct/);
});

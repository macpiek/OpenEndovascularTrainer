import test from 'node:test';
import assert from 'node:assert/strict';
import {solveCoulombNewton} from '../src/physics/kirchhoffCoulombNewtonSolver.js';
import {condenseKirchhoffSections as condense} from '../src/physics/kirchhoffSectionCondensation.js';
function pack(A){const n=A.length;return {values:Float64Array.from(A.flat()),starts:new Int32Array(n),ends:new Int32Array(n).fill(n-1),offsets:Int32Array.from({length:n},(_,i)=>i*n)};}
function solve(A,b){A=A.map(r=>r.slice());b=Array.from(b);const n=b.length;
 for(let k=0;k<n;k++){let p=k;for(let i=k+1;i<n;i++)if(Math.abs(A[i][k])>Math.abs(A[p][k]))p=i;
 [A[k],A[p]]=[A[p],A[k]];[b[k],b[p]]=[b[p],b[k]];
 assert.ok(Math.abs(A[k][k])>1e-14);for(let i=k+1;i<n;i++){const f=A[i][k]/A[k][k];for(let j=k+1;j<n;j++)A[i][j]-=f*A[k][j];b[i]-=f*b[k];}}
 for(let i=n-1;i>=0;i--){for(let j=i+1;j<n;j++)b[i]-=A[i][j]*b[j];b[i]/=A[i][i];}return b;}
const unpack=b=>Array.from({length:b.count},(_,i)=>Array.from({length:b.count},(_,j)=>j<b.matrix.starts[i]||j>b.matrix.ends[i]?0:b.matrix.values[b.matrix.offsets[i]+j]));
const close=(a,b)=>a.forEach((v,i)=>assert.ok(Math.abs(v-b[i])<1e-11,`${i}: ${v} vs ${b[i]}`));

test('two nonsymmetric indefinite section interiors transfer loads and preserve full solution',()=>{
 const A=[[8,1,2,0,0,0,0],[2,0,3,-1,0,0,0],[1,2,-1,2,0,0,0],[0,3,-2,9,2,1,0],[0,0,0,1,0,4,1],[0,0,0,-1,2,-2,3],[0,0,0,0,2,1,8]];
 const rhs=[1,3,-2,4,5,-1,2],input=pack(A),sections=[[1,2],[4,5]];
 const block=condense({matrix:input,rhs,sections}),x=solve(unpack(block),block.rhs),full=block.recover(x);
 close(full,solve(A,rhs));assert.equal(block.count,3);assert.equal(block.diagnostics.localCount,4);
 A.forEach((row,i)=>assert.ok(Math.abs(row.reduce((s,v,j)=>s+v*full[j],0)-rhs[i])<1e-11));
 input.values.fill(0);rhs.fill(0);sections[0].fill(0);block.matrix.values.fill(0);block.retainedIndices.fill(0);
 close(block.recover(x),full);
});

test('empty and complete interiors retain exact solutions',()=>{
 for(const sections of [[],[[],[0,1]],[[0,1]]]){
 const A=[[0,2],[3,-1]],rhs=[4,5],block=condense({matrix:pack(A),rhs,sections});
 close(block.recover(solve(unpack(block),block.rhs)),solve(A,rhs));}
});

test('cross-section edges and singular interiors are rejected rather than approximated',()=>{
 assert.throws(()=>condense({matrix:pack([[2,1],[1,2]]),rhs:[0,0],sections:[[0],[1]]}),/coupled/);
 assert.throws(()=>condense({matrix:pack([[2,1],[1,0]]),rhs:[0,0],sections:[[1]]}),/Singular/);
 assert.throws(()=>condense({matrix:pack([[2,1],[1,2]]),rhs:[0,0],sections:[[0],[0]]}),/repeated/);
});

for(const load of [1,-1])test(`local contact Newton preserves normal/friction coupling and boundary reaction, load=${load}`,()=>{
 const matrix=Float64Array.from([2,.4,0,.1,.4,2,0,0,0,0,2,0,.1,0,0,3]);
 const rhs=Float64Array.from([load,-4,0,.3]),lower=Float64Array.from([0,-Infinity,-Infinity,-Infinity]),upper=new Float64Array(4).fill(Infinity);
 const groups=[{rows:[1,2],lambda:[0,0],normalLambda:0,normalRow:0,mu:[.2,.2]}];
 const options={matrixFormat:'row-major',coulombLinearSolver:'band-lu',tolerance:1e-10};
 const full=solveCoulombNewton(matrix,rhs,lower,upper,4,4,groups,options);
 const local=solveCoulombNewton(matrix,rhs,lower,upper,4,4,groups,{...options,localSections:[[0,1,2]]});
 assert.ok(full.diagnostics.converged,JSON.stringify(full.diagnostics));
 assert.ok(local.diagnostics.converged,JSON.stringify(local.diagnostics));close(local.increment,full.increment);
 assert.ok(local.diagnostics.sectionSolves>0);assert.equal(local.diagnostics.globalRowCount,1);
 assert.equal(local.diagnostics.localRowCount,3);
});

test('section factors reuse only identical interiors, and prior recovery remains owned',()=>{
 const workspace={},A=[[8,1,2],[2,0,3],[1,2,-1]],matrix=pack(A),sections=[[1,2]];
 const first=condense({matrix,rhs:[1,3,-2],sections,workspace});
 const initial=solve(unpack(first),first.rhs),original=first.recover(initial);
 assert.equal(first.diagnostics.localFactorizations,1);
 const second=condense({matrix,rhs:[2,4,1],sections,workspace});
 assert.equal(second.diagnostics.localFactorizations,0);assert.equal(second.diagnostics.localFactorReuses,1);
 close(second.recover(solve(unpack(second),second.rhs)),solve(A,[2,4,1]));
 A[1][2]=3.5;
 const changed=condense({matrix:pack(A),rhs:[2,4,1],sections,workspace});
 assert.equal(changed.diagnostics.localFactorizations,1);assert.equal(changed.diagnostics.localFactorReuses,0);
 close(changed.recover(solve(unpack(changed),changed.rhs)),solve(A,[2,4,1]));
 close(first.recover(initial),original);
});

test('iterative refinement repairs cancellation in local reconstruction without relaxing the original linear gate',async()=>{
 const {createCoulombBandLayout}=await import('../src/physics/kirchhoffCoulombBandLU.js');
 const {createCoulombSectionLU}=await import('../src/physics/kirchhoffCoulombSectionLU.js');
 const matrix=pack([[1e-12,1],[1,0]]),layout=createCoulombBandLayout(matrix,2,2,[],'general-band');
 const J=new Float64Array(layout.entries);
 for(let i=0;i<2;i++)for(let j=layout.starts[i];j<=layout.ends[i];j++)J[layout.offsets[i]+j]=matrix.values[matrix.offsets[i]+j];
 const solver=createCoulombSectionLU(layout,2,[[0]]),direction=new Float64Array(2);
 assert.ok(solver.solve(J,Float64Array.from([-.3,-.7]),Float64Array.from([1,1]),0,direction));
 close(direction,[.7,.3-.7e-12]);
 assert.ok(solver.diagnostics.linearRefinementSteps>0);
 assert.ok(solver.diagnostics.maximumAcceptedLinearBackwardError<=128*Number.EPSILON);
});

test('boundary responses reuse only unchanged outgoing coupling and still update incoming reactions',()=>{
 const workspace={},A=[[8,1,2],[2,0,3],[1,2,-1]],sections=[[1,2]];
 const run=()=>condense({matrix:pack(A),rhs:[1,3,-2],sections,workspace});
 const first=run(),initial=solve(unpack(first),first.rhs),original=first.recover(initial);
 assert.equal(first.diagnostics.localResponseSolves,1);
 const second=run();assert.equal(second.diagnostics.localResponseReuses,1);
 assert.equal(second.diagnostics.localSchurReuses,1);assert.equal(second.diagnostics.localSchurProducts,0);
 A[0][1]=1.3; // Incoming reaction changes; outgoing response can still be reused.
 const incoming=run();assert.equal(incoming.diagnostics.localResponseReuses,1);
 assert.equal(incoming.diagnostics.localSchurProducts,1);assert.equal(incoming.diagnostics.localSchurReuses,0);
 close(incoming.recover(solve(unpack(incoming),incoming.rhs)),solve(A,[1,3,-2]));
 A[1][0]=2.5; // Outgoing coupling changes without changing the local LU.
 const outgoing=run();assert.equal(outgoing.diagnostics.localFactorReuses,1);
 assert.equal(outgoing.diagnostics.localSchurProducts,1);assert.equal(outgoing.diagnostics.localSchurReuses,0);
 assert.equal(outgoing.diagnostics.localResponseSolves,1);assert.equal(outgoing.diagnostics.localResponseReuses,0);
 close(outgoing.recover(solve(unpack(outgoing),outgoing.rhs)),solve(A,[1,3,-2]));
 close(first.recover(initial),original);
});

test('failed refactorization cannot leave a stale factor in a reused WASM workspace',()=>{
 const workspace={},A=[[8,1,2],[2,0,3],[1,2,-1]],sections=[[1,2]],rhs=[1,3,-2];
 const first=condense({matrix:pack(A),rhs,sections,workspace});
 const boundary=solve(unpack(first),first.rhs),original=first.recover(boundary);
 const singular=A.map(row=>row.slice());singular[2][1]=0;
 assert.throws(()=>condense({matrix:pack(singular),rhs,sections,workspace}),/Singular/);
 const restored=condense({matrix:pack(A),rhs,sections,workspace});
 assert.equal(restored.diagnostics.localFactorizations,1);
 close(restored.recover(solve(unpack(restored),restored.rhs)),solve(A,rhs));
 close(first.recover(boundary),original);
});

test('independent Newton solver instances safely share the section workspace',async()=>{
 const {createCoulombBandLayout}=await import('../src/physics/kirchhoffCoulombBandLU.js');
 const {createCoulombSectionLU}=await import('../src/physics/kirchhoffCoulombSectionLU.js');
 const matrix=pack([[3,1],[1,2]]),layout=createCoulombBandLayout(matrix,2,2,[],'general-band'),workspace={};
 const first=createCoulombSectionLU(layout,2,[[0]],workspace),second=createCoulombSectionLU(layout,2,[[0]],workspace);
 const x=new Float64Array(2),unit=Float64Array.from([1,1]);
 assert.ok(first.solve(matrix.values,Float64Array.from([-1,-2]),unit,0,x));close(x,[0,1]);
 assert.ok(second.solve(matrix.values,Float64Array.from([-2,-1]),unit,0,x));close(x,[.6,.2]);
 assert.equal(second.diagnostics.localFactorReuses,1);assert.equal(second.diagnostics.localResponseReuses,1);
 const changed=Float64Array.from([4,1,1,2]);
 assert.ok(second.solve(changed,Float64Array.from([-2,-1]),unit,0,x));close(x,[3/7,2/7]);
 assert.ok(first.solve(matrix.values,Float64Array.from([-1,-2]),unit,0,x));close(x,[0,1]);
});

test('profiled triangular solves preserve pivot fill and repeated right-hand sides',async()=>{
 const {createKirchhoffLinearKernel}=await import('../src/physics/kirchhoffLinearKernel.js');
 const n=24,kernel=createKirchhoffLinearKernel(8*n*n+40*n+128);
 const A=Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>i===j?4:j===i+1?-.7:j===i-1?1.2:0));
 // Row permutation forces nonlocal pivots and moves stored L coefficients.
 [A[0],A[17]]=[A[17],A[0]];[A[3],A[22]]=[A[22],A[3]];
 const lu=kernel.alloc(Float64Array,n*n),piv=kernel.alloc(Int32Array,n),rhs=kernel.alloc(Float64Array,n),dense=kernel.alloc(Float64Array,n);
 const starts=kernel.alloc(Int32Array,n),ends=kernel.alloc(Int32Array,n);
 lu.set(A.flat());assert.equal(kernel.factorDenseLU(lu.byteOffset,piv.byteOffset,n),0);
 assert.ok(piv.some((v,i)=>v!==i));
 for(let i=0;i<n;i++){let first=0,last=n-1;while(first<i&&lu[i*n+first]===0)first++;while(last>i&&lu[i*n+last]===0)last--;starts[i]=first;ends[i]=last;}
 for(let load=0;load<8;load++){
  const input=Float64Array.from({length:n},(_,i)=>Math.sin(i+load));rhs.set(input);dense.set(input);
  kernel.solveDenseProfileLU(lu.byteOffset,piv.byteOffset,rhs.byteOffset,n,starts.byteOffset,ends.byteOffset);
  kernel.solveDenseLU(lu.byteOffset,piv.byteOffset,dense.byteOffset,n);
  close(rhs,dense);close(rhs,solve(A,input));
 }
});

test('topology cache updates values, detects zero support changes, and rechecks section ownership',()=>{
 const workspace={},rhs=[1,2,3,4],A=[[5,1,0,0],[2,4,1,0],[0,1,5,2],[0,0,1,4]],sections=[[1,2]];
 const run=()=>condense({matrix:pack(A),rhs,sections,workspace});
 const first=run(),boundary=solve(unpack(first),first.rhs),original=first.recover(boundary);
 assert.equal(first.diagnostics.topologyReused,false);
 A[1][1]=6;const changed=run();assert.equal(changed.diagnostics.topologyReused,true);
 close(changed.recover(solve(unpack(changed),changed.rhs)),solve(A,rhs));
 A[0][2]=.3;const added=run();assert.equal(added.diagnostics.topologyReused,false);
 close(added.recover(solve(unpack(added),added.rhs)),solve(A,rhs));
 A[0][2]=0;const removed=run();assert.equal(removed.diagnostics.topologyReused,false);
 close(removed.recover(solve(unpack(removed),removed.rhs)),solve(A,rhs));
 assert.throws(()=>condense({matrix:pack(A),rhs,sections:[[1],[2]],workspace}),/coupled/);
 const restored=run();assert.equal(restored.diagnostics.topologyReused,true);
 close(restored.recover(solve(unpack(restored),restored.rhs)),solve(A,rhs));
 close(first.recover(boundary),original);
});

test('many small sections and repartition retain independent factors in one arena',()=>{
 const n=128,A=Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>i===j?i+2:0)),rhs=Array.from({length:n},(_,i)=>i+1),workspace={};
 const singles=Array.from({length:n},(_,i)=>[i]);
 const first=condense({matrix:pack(A),rhs,sections:singles,workspace});
 const original=first.recover([]);close(original,rhs.map((v,i)=>v/(i+2)));
 const pairs=Array.from({length:n/2},(_,i)=>[2*i,2*i+1]);
 const second=condense({matrix:pack(A),rhs,sections:pairs,workspace});
 close(second.recover([]),original);close(first.recover([]),original);
 A[0][0]=7;
 const third=condense({matrix:pack(A),rhs,sections:singles,workspace});
 close(third.recover([]),rhs.map((v,i)=>v/A[i][i]));close(second.recover([]),original);
});

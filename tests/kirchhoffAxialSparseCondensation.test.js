import test from 'node:test';
import assert from 'node:assert/strict';
import {condenseKirchhoffAxialSparse as condense} from '../src/physics/kirchhoffAxialSparseCondensation.js';

function solve(A,b) {
 const n=b.length;A=A.map(row=>row.slice());b=b.slice();
 for(let k=0;k<n;k++) {
  let p=k;for(let i=k+1;i<n;i++) if(Math.abs(A[i][k])>Math.abs(A[p][k]))p=i;
  [A[k],A[p]]=[A[p],A[k]];[b[k],b[p]]=[b[p],b[k]];
  assert.ok(Math.abs(A[k][k])>1e-15);
  for(let i=k+1;i<n;i++) {const f=A[i][k]/A[k][k];for(let j=k+1;j<n;j++)A[i][j]-=f*A[k][j];b[i]-=f*b[k];}
 }
 for(let i=n-1;i>=0;i--){for(let j=i+1;j<n;j++)b[i]-=A[i][j]*b[j];b[i]/=A[i][i];}return b;
}
const dense=block=>Array.from({length:block.count},(_,i)=>Array.from({length:block.count},(_,j)=>
 j<block.matrix.starts[i]||j>block.matrix.ends[i]?0:block.matrix.values[block.matrix.offsets[i]+j]));
const close=(a,b)=>a.forEach((v,i)=>assert.ok(Math.abs(v-b[i])<1e-11,`${i}: ${v} != ${b[i]}`));
function fixture(coupling=0) {
 const H=[[8,2,-1],[2,4,coupling],[-1,coupling,2]];
 return {hRows:H.map(row=>new Map(row.map((v,i)=>[i,v]))),b:[3,7,-5],
 gradients:[[[0,1],[1,-2],[2,3]]],explicitRows:[{gradientIndex:0,alpha:.1,rhs:2,lower:-Infinity,upper:Infinity}],
 eliminatedIndices:[1,2],axialCoordinates:[2,0,1]};
}
for(const coupling of [0,.2])test(`sparse mixed elimination matches independent full equations, coupling=${coupling}`,()=>{
 const input=fixture(coupling),block=condense(input);
 const H=input.hRows.map(row=>[...Array(3)].map((_,j)=>row.get(j)??0));
 const J=[1,-2,3];const A=H.map((row,i)=>[...row,-J[i]]);A.push([...J,.1]);
 const full=solve(A,[...input.b,2]),mixed=solve(dense(block),Array.from(block.rhs));
 close(block.recover(mixed),full.slice(0,3));close([mixed[block.explicitIndices[0]]],[full[3]]);
 assert.equal(block.diagnostics.eliminatedOffsetDofs,coupling?0:2);
 assert.equal(block.diagnostics.coupledEliminationFallback,!!coupling);
 const original=block.recover(mixed);
 input.hRows.forEach(row=>row.clear());input.b.fill(999);input.gradients[0].length=0;
 block.retainedIndices.fill(999);block.retainedSolutionIndices.fill(999);block.explicitIndices.fill(999);
 block.matrix.values.fill(999);block.rhs.fill(999);
 assert.deepEqual(block.recover(mixed),original);
});

test('local chain band storage grows linearly without dropping nonzeros',()=>{
 const sizes=[32,64,128].map(N=>{
  const hRows=Array.from({length:2*N},(_,i)=>new Map([[i,2]]));
  const gradients=Array.from({length:N-1},(_,i)=>[[2*i,1],[2*i+1,-1],[2*i+2,-1],[2*i+3,1]]);
  const block=condense({hRows,b:new Float64Array(2*N),gradients,
   explicitRows:gradients.map((_,i)=>({gradientIndex:i,alpha:.01,rhs:0,lower:-Infinity,upper:Infinity})),
   eliminatedIndices:Array.from({length:N},(_,i)=>2*i+1),axialCoordinates:Array.from({length:2*N},(_,i)=>Math.floor(i/2))});
  assert.ok(block.diagnostics.lowerBandwidth<=3);assert.ok(block.diagnostics.upperBandwidth<=3);
  assert.ok(block.matrix.values.length<block.count*7);
  return block.matrix.values.length;
 });
 assert.ok(sizes[1]<=2*sizes[0]+20);assert.ok(sizes[2]<=2*sizes[1]+20);
});

test('invalid symmetry, diagonal and nonfinite recovery are rejected',()=>{
 const input=fixture();input.hRows[1].set(0,1);assert.throws(()=>condense(input),/symmetric/);
 const zero=fixture();zero.hRows[1].set(1,0);assert.throws(()=>condense(zero),/positive/);
 const block=condense(fixture());assert.throws(()=>block.recover([Infinity,0]),/Invalid/);
});

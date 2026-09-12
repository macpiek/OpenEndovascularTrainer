import test from 'node:test';import assert from 'node:assert/strict';
import {deleteKirchhoffCholeskyRow as remove} from '../src/physics/kirchhoffCholeskyDelete.js';
function cholesky(A,n){
 const L=new Float64Array(n*n);
 for(let i=0;i<n;i++)for(let j=0;j<=i;j++){
  let sum=A[i*n+j];for(let k=0;k<j;k++)sum-=L[i*n+k]*L[j*n+k];
  L[i*n+j]=i===j?Math.sqrt(sum):sum/L[j*n+j];
 }
 return L;
}
function pack(L,n,band=n){const factor=new Float64Array(n*band);for(let i=0;i<n;i++)for(let j=Math.max(0,i-band+1);j<=i;j++)factor[i*band+i-j]=L[i*n+j];return factor;}
function gram(L,n){const A=new Float64Array(n*n);for(let i=0;i<n;i++)for(let j=0;j<n;j++)for(let k=0;k<=Math.min(i,j);k++)A[i*n+j]+=L[i*n+k]*L[j*n+k];return A;}
function assertPrincipal(A,n,index,out){
 const size=n-1;
 for(let i=0;i<size;i++)for(let j=0;j<=i;j++){
  let actual=0,magnitude=0;for(let k=0;k<=j;k++){const product=out[i*size+i-k]*out[j*size+j-k];actual+=product;magnitude+=Math.abs(product);}
  const expected=A[(i<index?i:i+1)*n+(j<index?j:j+1)];
  assert.ok(Math.abs(actual-expected)<=128*n*Number.EPSILON*Math.max(Number.MIN_VALUE,Math.abs(expected),magnitude),`${n}/${index} row${i},${j}: ${actual} != ${expected}`);
 }
}
for(const n of [1,2,7,31])test(`arbitrary SPD principal deletion reconstructs original matrix (${n})`,()=>{
 const B=Float64Array.from({length:n*n},(_,k)=>Math.sin(k*1.37+.2));const A=new Float64Array(n*n);
 for(let i=0;i<n;i++)for(let j=0;j<n;j++){for(let k=0;k<n;k++)A[i*n+j]+=B[i*n+k]*B[j*n+k];if(i===j)A[i*n+j]+=.05;}
 const factor=pack(cholesky(A,n),n),before=factor.slice(),workspace={};
 for(let index=0;index<n;index++){const out=new Float64Array((n-1)**2);assert.equal(remove(factor,n,n,index,out,workspace),true);assertPrincipal(A,n,index,out);assert.deepEqual(factor,before);}
});
test('band input, near-null positive shift and repeated deletion preserve the original principal matrix',()=>{
 const n=12,L=new Float64Array(n*n);for(let i=0;i<n;i++){L[i*n+i]=1e-4;for(let j=Math.max(0,i-2);j<i;j++)L[i*n+j]=Math.sin(i+j)*.3;}
 const A=gram(L,n),out=new Float64Array((n-1)**2),workspace={};assert.ok(remove(pack(L,n,3),n,3,5,out,workspace));assertPrincipal(A,n,5,out);
 const tiny=Float64Array.from([1e-160,0,1e-160,1]);const tinyA=gram(Float64Array.from([1e-160,0,1,1e-160]),2),result=new Float64Array(1);
 assert.ok(remove(tiny,2,2,0,result));assertPrincipal(tinyA,2,0,result);
});
test('warm workspace buffers are reused across sizes and consecutive deletes',()=>{
 const workspace={},n=9,L=new Float64Array(n*n);for(let i=0;i<n;i++){L[i*n+i]=1;for(let j=0;j<i;j++)L[i*n+j]=.01*(i+j);}
 let source=pack(L,n),A=gram(L,n),count=n;const destination=new Float64Array((n-1)**2);
 assert.ok(remove(source,count,count,3,destination,workspace));const factorBuffer=workspace.factor,vectorBuffer=workspace.vector;
 for(const index of [3,0,2,1]){
  assert.ok(remove(source,count,count,index,destination,workspace));assertPrincipal(A,count,index,destination);
  const next=new Float64Array((count-1)**2);for(let i=0;i<count-1;i++)for(let j=0;j<count-1;j++)next[i*(count-1)+j]=A[(i<index?i:i+1)*count+(j<index?j:j+1)];
  count--;source=destination.slice(0,count*count);A=next;
  assert.equal(workspace.factor,factorBuffer);assert.equal(workspace.vector,vectorBuffer);
 }
});
test('invalid pivots, nonfinite data and overflow leave input and output untouched',()=>{
 for(const source of [Float64Array.from([0,0,1,.2]),Float64Array.from([1,0,-1,.2]),Float64Array.from([1,0,1,NaN]),Float64Array.from([1,0,Number.MAX_VALUE,Number.MAX_VALUE])]){
  const saved=source.slice(),out=Float64Array.of(123);assert.equal(remove(source,2,2,0,out),false);assert.deepEqual(source,saved);assert.deepEqual(out,Float64Array.of(123));
 }
});

test('a rank-deficient matrix with its original tiny positive shift keeps that same shift after deletion',()=>{
 const n=8,shift=1e-8,v=Float64Array.from({length:n},(_,i)=>Math.cos(i*.4)),A=new Float64Array(n*n);
 for(let i=0;i<n;i++)for(let j=0;j<n;j++)A[i*n+j]=v[i]*v[j]+(i===j?shift:0);
 const source=pack(cholesky(A,n),n),out=new Float64Array((n-1)**2);
 for(const index of [0,3,n-1]){assert.ok(remove(source,n,n,index,out));assertPrincipal(A,n,index,out);}
});
test('output may overlap input only after successful complete computation',()=>{
 const source=Float64Array.from([2,0,0,3,.2,0,4,.3,.4]),rowMajor=Float64Array.from([2,0,0,.2,3,0,.4,.3,4]),A=gram(rowMajor,3);
 assert.ok(remove(source,3,3,1,source));assertPrincipal(A,3,1,source);
});

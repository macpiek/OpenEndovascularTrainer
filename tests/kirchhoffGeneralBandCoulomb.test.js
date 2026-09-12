import test from 'node:test';
import assert from 'node:assert/strict';
import {createCoulombBandLayout} from '../src/physics/kirchhoffCoulombBandLU.js';
import {solveCoulombNewton} from '../src/physics/kirchhoffCoulombNewtonSolver.js';
import {measureCoupledLoadKKT} from '../src/physics/kirchhoffCoupledLoadSolver.js';

test('directed row envelopes retain tiny couplings without inventing a transpose',()=>{
    const n=8,a=new Float64Array(n*n);for(let i=0;i<n;i++)a[i*n+i]=1;
    a[1*n+6]=1e-30;
    const p=createCoulombBandLayout(a,n,n,[],'row-major');
    assert.equal(p.ends[1],6);assert.equal(p.starts[6],6);assert.equal(p.entries,13);
});

test('general axial band LU solves the same nonsymmetric force-dependent friction system as dense LU',()=>{
    const n=60,a=new Float64Array(n*n),rhs=new Float64Array(n),lower=new Float64Array(n),upper=new Float64Array(n).fill(Infinity),groups=[];
    for(let i=0;i<n;i++){
        a[i*n+i]=2+(i%3)*.1;
        if(i>0)a[i*n+i-1]=-.09;
        if(i+1<n)a[i*n+i+1]=.04;
        rhs[i]=i%3===0?1:Math.sin(i)*.8;
        if(i%3!==0)lower[i]=-Infinity;
    }
    for(let i=0;i<n;i+=3)groups.push({rows:[i+1,i+2],normalRow:i,normalLambda:0,lambda:[0,0],mu:[.2,.3],radii:[0,0]});
    const inputs=[a,rhs,lower,upper,n,n,groups],options={matrixFormat:'row-major',tolerance:1e-8},
        dense=solveCoulombNewton(...inputs,options),band=solveCoulombNewton(...inputs,{...options,coulombLinearSolver:'band-lu'});
    assert.equal(dense.diagnostics.converged,true);assert.equal(band.diagnostics.converged,true,JSON.stringify(band.diagnostics));
    assert.equal(band.diagnostics.linearSolver,'band-lu');assert.ok(band.diagnostics.jacobianEntries<n*n/5);
    const residual=Float64Array.from(rhs,(v,i)=>v-band.increment.reduce((sum,x,j)=>sum+a[i*n+j]*x,0));
    const actualGroups=groups.map(g=>({...g,radii:g.mu.map(mu=>mu*Math.max(0,band.increment[g.normalRow]))}));
    const kkt=measureCoupledLoadKKT(residual,band.increment,lower,upper,actualGroups);
    assert.ok(kkt.maximumResidual<=1e-8,JSON.stringify(kkt));
    for(let i=0;i<n;i++){assert.ok(Math.abs(band.increment[i]-dense.increment[i])<1e-7);assert.ok(Math.abs(residual[i]-band.residual[i])<1e-12);}
});

function pack(a,n){
 const starts=new Int32Array(n),ends=new Int32Array(n),offsets=new Int32Array(n);let size=0;
 for(let i=0;i<n;i++){starts[i]=ends[i]=i;for(let j=0;j<n;j++)if(a[i*n+j]!==0){starts[i]=Math.min(starts[i],j);ends[i]=Math.max(ends[i],j);}
 offsets[i]=size-starts[i];size+=ends[i]-starts[i]+1;}
 const values=new Float64Array(size);for(let i=0;i<n;i++)for(let j=starts[i];j<=ends[i];j++)values[offsets[i]+j]=a[i*n+j];
 return {values,starts,ends,offsets};
}
test('compact general-band keeps directed tiny couplings and expands friction envelopes without dense input',()=>{
 const n=12,a=new Float64Array(n*n);for(let i=0;i<n;i++)a[i*n+i]=1;a[2*n+8]=1e-30;
 const input=pack(a,n),layout=createCoulombBandLayout(input,n,0,[{rows:[2,3],normalRow:10}],'general-band');
 assert.equal(layout.ends[2],10);assert.equal(layout.ends[3],10);assert.equal(layout.starts[8],8);assert.equal(layout.starts[10],10);
 assert.ok(input.values.length<n*n/2);
});
test('compact general-band matches original dense Coulomb equations and defaults to band LU',()=>{
 const n=48,a=new Float64Array(n*n),rhs=new Float64Array(n),lo=new Float64Array(n).fill(-Infinity),hi=new Float64Array(n).fill(Infinity),groups=[];
 for(let i=0;i<n;i++){a[i*n+i]=2;if(i)a[i*n+i-1]=-.09;if(i+1<n)a[i*n+i+1]=.04;rhs[i]=i%3===0?1:Math.sin(i)*.8;}
 for(let i=0;i<n;i+=3){lo[i]=0;groups.push({rows:[i+1,i+2],normalRow:i,normalLambda:0,lambda:[0,0],mu:[.2,.3],radii:[0,0]});}
 const packed=pack(a,n),before=structuredClone({packed,groups});
 for(const normalMap of ['projection','fischer-burmeister']){
 const options={normalMap,tolerance:1e-9},dense=solveCoulombNewton(a,rhs,lo,hi,n,n,groups,{...options,matrixFormat:'row-major'}),
 compact=solveCoulombNewton(packed,rhs,lo,hi,n,0,groups,{...options,matrixFormat:'general-band'});
 assert.equal(compact.diagnostics.converged,true,JSON.stringify(compact.diagnostics));assert.equal(dense.diagnostics.converged,true);
 assert.equal(compact.diagnostics.matrixFormat,'general-band');assert.equal(compact.diagnostics.linearSolver,'band-lu');
 assert.ok(compact.diagnostics.jacobianEntries<n*n/5);assert.ok(compact.diagnostics.factorEntries<n*n/3);
 for(let i=0;i<n;i++){assert.ok(Math.abs(compact.increment[i]-dense.increment[i])<1e-8);
 const residual=rhs[i]-compact.increment.reduce((v,x,j)=>v+a[i*n+j]*x,0);assert.ok(Math.abs(residual-compact.residual[i])<1e-12);}
 }assert.deepEqual({packed,groups},before);
});
test('compact general-band preserves nonsymmetric zero and negative diagonal mobility branches',()=>{
 for(const a of [Float64Array.of(0,1,2,3),Float64Array.of(-1,.2,.1,2),Float64Array.of(0,0,0,1)]){
 const rhs=a[0]===0&&a[1]===0?[0,.2]:[.3,-.2],lo=[-Infinity,-Infinity],hi=[Infinity,Infinity],options={tolerance:1e-9};
 const dense=solveCoulombNewton(a,rhs,lo,hi,2,2,[],{...options,matrixFormat:'row-major'}),compact=solveCoulombNewton(pack(a,2),rhs,lo,hi,2,0,[],{...options,matrixFormat:'general-band'});
 assert.equal(compact.diagnostics.converged,dense.diagnostics.converged);compact.increment.forEach((v,i)=>assert.ok(Math.abs(v-dense.increment[i])<1e-8));
 }
});
test('compact input rejects incomplete, nonfinite, out-of-range or diagonal-free row metadata',()=>{
 const valid=()=>pack(Float64Array.of(1,.1,0,2),2);
 for(const mutate of [m=>m.values[0]=NaN,m=>m.values[0]=Infinity,m=>m.values=Array.from(m.values),m=>m.starts=[0,1],
 m=>m.starts=new Int32Array(1),m=>m.ends=new Int32Array(3),m=>m.offsets=new Float64Array(2),m=>m.starts[0]=-1,
 m=>m.starts[0]=1,m=>m.ends[1]=0,m=>m.ends[0]=2,m=>m.offsets[0]=-1,m=>m.offsets[1]=999]){
 const m=valid();mutate(m);assert.throws(()=>solveCoulombNewton(m,[0,0],[-Infinity,-Infinity],[Infinity,Infinity],2,0,[],{matrixFormat:'general-band'}),/general-band/);
 }
 assert.throws(()=>createCoulombBandLayout(valid(),-1,0,[],'general-band'),/general-band/);
 assert.throws(()=>createCoulombBandLayout(valid(),2,0,[{rows:[0,2]}],'general-band'),/valid distinct/);
});

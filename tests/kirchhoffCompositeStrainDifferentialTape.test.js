import test from 'node:test';
import assert from 'node:assert/strict';
import {createCompositeStrainDifferentialTape as tape} from '../src/physics/kirchhoffCompositeStrainDifferentialTape.js';
import {createCompositeSurfaceDifferentialArena as forward} from '../src/physics/kirchhoffCompositeJointSurfaceMotion.js';

const close=(a,b)=>assert.ok(Number.isFinite(a)&&Number.isFinite(b)&&Math.abs(a-b)<=2e-12*(1+Math.abs(a)+Math.abs(b)),`${a} != ${b}`);
function expression(a, values) {
    const q=values.map((x,j)=>a.variable(x,j));let length=a.constant(1),product=a.constant(.7);
    q.forEach((x,j)=>{length=a.add(length,a.mul(x,x));product=a.add(product,a.scale(a.sin(a.mul(x,q[(j+1)%q.length])),1/(j+1)));});
    const s=a.sqrt(length),r=a.reciprocal(length),angle=a.atan2(product,s),extra=a.variable(values[0],0);
    return [a.add(s,a.mul(r,a.cos(angle))),a.sub(a.mul(angle,product),a.scale(length,3)),a.add(a.mul(extra,q[0]),a.sub(product,product))];
}

test('reverse strain Hessians agree with full forward differentiation for nonlinear shared graphs, aliased parents and repeated inputs',()=>{
    for(const N of [2,7,29]) {
        const a=forward(N,true,2048),b=tape(N),values=Array.from({length:N},(_,j)=>.17*Math.sin(j+1)-.04*j),
            left=expression(a,values),right=expression(b,values),H=b.hessians(right);
        for(let k=0;k<3;k++) {
            close(a.data[left[k]],b.data[right[k]]);
            for(let i=0;i<N;i++) {
                close(a.data[left[k]+1+i],b.data[right[k]+1+i]);
                for(let j=0;j<N;j++) {
                    close(a.data[left[k]+1+N+i*N+j],H[(k*N+i)*N+j]);
                    close(H[(k*N+i)*N+j],H[(k*N+j)*N+i]);
                }
            }
        }
        assert.ok(H[N*N]<0,'negative geometric stiffness must not be projected away');
    }
});

test('reversed polynomial derivatives match analytic Hessians and each output has its own cleared adjoints',()=>{
    const a=tape(2),x=a.variable(.3,0),y=a.variable(-.4,1),xx=a.mul(x,x),xy=a.mul(x,y),
        cubic=a.mul(xx,y),constant=a.constant(7),negative=a.scale(xx,-2),H=a.hessians([cubic,constant,negative,cubic]);
    const expected=[-.8,.6,.6,0, 0,0,0,0, -4,0,0,0, -.8,.6,.6,0];
    H.forEach((v,j)=>close(v,expected[j]));
    assert.deepEqual(a.hessians([xy]),new Float64Array([0,1,1,0]));
});

test('checkpoint replay, reset, dimension changes and rejected evaluations retain no old derivatives and keep scratch bounded',()=>{
    const a=tape(7,128),x=a.variable(.2,0),checkpoint=a.checkpoint(),first=a.hessians([a.sin(x)]);
    a.rewind(checkpoint);const other=a.hessians([a.cos(x)]);close(other[0],-Math.cos(.2));
    a.rewind(checkpoint);assert.deepEqual(a.hessians([a.sin(x)]),first);
    assert.throws(()=>a.rewind(1),/checkpoint/);assert.throws(()=>a.hessians([a.checkpoint()]),/live/);
    assert.throws(()=>a.variable(1,7),/index/);
    while(a.used()<128)a.constant(0);assert.throws(()=>a.constant(0),/capacity/);
    a.reset();const next=a.variable(-.3,1);close(a.hessians([a.mul(next,next)])[8],2);
    a.constant(NaN);assert.equal(a.finite(),false);
    a.reset();const tiny=a.variable(1e-150,0),inverse=a.reciprocal(tiny);assert.equal(a.finite(),true);
    assert.throws(()=>a.hessians([inverse]),/Nonfinite/);
    a.reset();const retry=a.variable(.2,0);assert.deepEqual(a.hessians([a.sin(retry)]),first);
    const small=tape(2,128,a.storage);assert.equal(small.storage,a.storage);assert.equal(small.retainedBytes,a.retainedBytes);
    const big=tape(29,128,small.storage);assert.notEqual(big.storage,small.storage);
    assert.ok(big.retainedBytes<128*(1+29+29*29)*8/10+65536,'linear derivative storage plus bounded WASM page rounding');
    assert.throws(()=>tape(0),/dimension/);
});

test('WASM finalization computes only the changed suffix, keeps fixed memory and never returns partially finalized derivatives',()=>{
    const a=tape(3,128),buffer=a.storage.memory.buffer,x=a.variable(.2,0),prefix=a.mul(x,x),checkpoint=a.checkpoint(),first=a.sin(prefix);
    assert.equal(a.diagnostics.forwardCalls,0);assert.equal(a.finalize(),true);
    const ready=a.diagnostics;assert.equal(ready.forwardNodes,a.used());
    close(a.data[first+1],.4*Math.cos(.04));a.hessians([first]);
    assert.equal(a.diagnostics.forwardCalls,ready.forwardCalls,'reverse evaluation must use the already-current first derivatives');
    a.rewind(checkpoint);const second=a.cos(prefix);assert.equal(a.finite(),true);
    assert.equal(a.diagnostics.forwardNodes,ready.forwardNodes+1,'the shared native prefix is still current');
    close(a.data[second+1],-.4*Math.sin(.04));
    a.constant(NaN);assert.equal(a.finalize(),false);assert.throws(()=>a.hessians([second]),/Nonfinite/);
    a.rewind(checkpoint);const retry=a.sin(prefix);assert.equal(a.finalize(),true);close(a.data[retry+1],.4*Math.cos(.04));
    assert.equal(a.storage.memory.buffer,buffer);assert.throws(()=>a.storage.memory.grow(1),RangeError);assert.equal(a.data.buffer,buffer);
    const b=tape(2,64,a.storage),variable=b.variable(.3,1),square=b.mul(variable,variable);assert.equal(b.finalize(),true);
    close(b.data[square+2],.6);assert.deepEqual(b.hessians([square]),new Float64Array([0,0,0,2]));
    assert.equal(b.storage.memory.buffer,buffer);
});

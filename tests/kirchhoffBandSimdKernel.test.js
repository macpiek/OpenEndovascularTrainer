import test from 'node:test';
import assert from 'node:assert/strict';
import {createKirchhoffLinearKernel} from '../src/physics/kirchhoffLinearKernel.js';
import {bandSimdKernelBytes} from '../src/physics/kirchhoffBandSimdKernelBytes.js';

function workspace(n,kl,ku,simd){
    const stride=2*kl+ku+1,kernel=createKirchhoffLinearKernel(8*n*stride+16*n+1024,{simd});
    const guards=[];
    const alloc=(Type,count)=>{const guard=kernel.alloc(Float64Array,4);guard.fill(789123.5);guards.push(guard);return kernel.alloc(Type,count);};
    const factor=alloc(Float64Array,n*stride),rhs=alloc(Float64Array,n),right=alloc(Int32Array,n);
    alloc(Float64Array,0);
    return {n,kl,ku,stride,kernel,factor,rhs,right,guards};
}
function run(w){return w.kernel.solveGeneralBandLU(w.factor.byteOffset,w.rhs.byteOffset,w.right.byteOffset,w.n,w.kl,w.ku);}
function fill(w,seed,mode){
    const {n,kl,ku,stride,factor,rhs,right}=w;factor.fill(0);
    for(let i=0;i<n;i++){
        right[i]=Math.min(n-1,i+(mode==='ragged'?i%Math.max(1,ku+1):ku));
        rhs[i]=Math.cos(i+seed);
        for(let j=Math.max(0,i-kl);j<=right[i];j++){
            const index=i*stride+kl+j-i;
            factor[index]=i===j?(mode==='pivot'?1e-5:2+kl+ku):Math.sin(seed+i*7+j*11)*.4;
            if((i*3+j+seed)%7===0&&i!==j)factor[index]=-0;
            if(mode==='singular'&&j===Math.floor(n/2))factor[index]=0;
        }
    }
}

test('optional SIMD elimination preserves scalar factors, RHS, pivots and rejection over odd and ragged bands',()=>{
    assert.equal(WebAssembly.validate(bandSimdKernelBytes),true,'This regression runner must exercise SIMD');
    let swaps=0,rejections=0;
    for(const n of [0,1,2,3,4,9,17,32,63,128])for(const width of [0,1,2,3,7])for(const mode of ['ordinary','ragged','pivot','singular']){
        const kl=Math.min(width,n-1),ku=Math.min((width*3+1)%9,n-1);
        if(kl<0){if(width!==0)continue;}
        const a=workspace(n,Math.max(0,kl),Math.max(0,ku),false),b=workspace(n,Math.max(0,kl),Math.max(0,ku),true);
        fill(a,n+width,mode);b.factor.set(a.factor);b.rhs.set(a.rhs);b.right.set(a.right);
        const ref=run(a),actual=run(b);
        assert.equal(actual,ref,`${n}/${kl}/${ku}/${mode}`);
        assert.deepEqual(b.factor,a.factor);assert.deepEqual(b.rhs,a.rhs);assert.deepEqual(b.right,a.right);
        for(const guard of [...a.guards,...b.guards])assert.deepEqual(Array.from(guard),[789123.5,789123.5,789123.5,789123.5]);
        if(actual<0)rejections++;else swaps+=actual;
    }
    assert.ok(swaps>0);assert.ok(rejections>0);
});

test('repeated SIMD calls keep independent scratch and preserve scalar rounding at extreme finite scales',()=>{
    const a=workspace(61,13,8,false),b=workspace(61,13,8,true),unrelated=workspace(33,7,5,true);
    for(const scale of [1e-12,1,1e100,1e-10,1e60]){
        fill(a,27,'pivot');
        for(let i=0;i<a.factor.length;i++)a.factor[i]*=scale;
        for(let i=0;i<a.rhs.length;i++)a.rhs[i]*=scale;
        b.factor.set(a.factor);b.rhs.set(a.rhs);b.right.set(a.right);
        const expected=run(a);fill(unrelated,19,'ordinary');assert.ok(run(unrelated)>=0);
        assert.equal(run(b),expected);assert.deepEqual(b.factor,a.factor);assert.deepEqual(b.rhs,a.rhs);
    }
});

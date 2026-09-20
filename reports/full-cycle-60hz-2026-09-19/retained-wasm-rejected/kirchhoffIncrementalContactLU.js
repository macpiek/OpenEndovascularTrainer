import { createKirchhoffLinearKernel } from './kirchhoffLinearKernel.js';

// Leases remain exclusive across generator yields. Releasing a completed or
// cancelled linearization makes its WASM memory reusable without allocating
// a new instance for every Newton direction. The idle pool is bounded.
const idle=[];
function acquire(bytes) {
    const index=idle.findIndex(v=>v.buffer.byteLength>=bytes);
    if(index>=0)return idle.splice(index,1)[0];
    let capacity=65536;while(capacity<bytes)capacity*=2;
    const kernel=createKirchhoffLinearKernel(capacity);
    return {kernel,buffer:kernel.alloc(Uint8Array,capacity).buffer};
}

// General row updates A = B + U V^T; retain a pivoted band LU of B.
// This workspace belongs to exactly one immutable linearization.
export function createIncrementalContactLU(layout,count,{maxRank=8,wasmAssembly=false}={}) {
    if(!Number.isInteger(maxRank)||maxRank<0||maxRank>16)throw new RangeError('Invalid contact update rank');
    const {starts,ends,offsets,kl,ku}=layout,stride=2*kl+ku+1;
    const lease=acquire(8*count*(stride+kl+4)+128+(wasmAssembly?8*layout.entries+64*count+64:0)),{kernel,buffer}=lease;
    let cursor=0,disposed=false;
    const alloc=(Type,length)=>{cursor=Math.ceil(cursor/8)*8;const v=new Type(buffer,cursor,length);cursor+=v.byteLength;return v;};
    const factor=alloc(Float64Array,count*stride),rhs=alloc(Float64Array,count),
        right=alloc(Int32Array,count),pivots=alloc(Int32Array,count),lower=alloc(Float64Array,count*kl);
    const base=new Float64Array(layout.entries),scale=new Float64Array(count),x=new Float64Array(count);
    const assembly=wasmAssembly?{matrix:alloc(Float64Array,layout.entries),input:alloc(Float64Array,count),
        solution:alloc(Float64Array,count),error:alloc(Float64Array,count),scales:alloc(Float64Array,count),
        starts:alloc(Int32Array,count),ends:alloc(Int32Array,count),offsets:alloc(Int32Array,count),out:alloc(Float64Array,4)}:null;
    if(assembly){assembly.starts.set(starts);assembly.ends.set(ends);assembly.offsets.set(offsets);}
    let valid=false;const columns=new Map();
    const diagnostics={factorizations:0,updates:0,reuses:0,backsolves:0,updateFailures:0,rankResets:0,
        changedRows:{},maximumBackwardError:0};
    function backsolve(out) {
        kernel.solveRetainedGeneralBandLU(factor.byteOffset,rhs.byteOffset,right.byteOffset,count,kl,ku,pivots.byteOffset,lower.byteOffset);
        for(let i=0;i<count;i++)out[i]=rhs[i]*scale[i];
        diagnostics.backsolves++;
    }
    function certify(J,F) {
        let err=0,normA=0,normF=0,normX=0;
        if(assembly) {
            assembly.matrix.set(J);assembly.input.set(F);assembly.solution.set(x);
            kernel.measureOriginalBandBackwardError(assembly.matrix.byteOffset,assembly.input.byteOffset,assembly.solution.byteOffset,
                assembly.starts.byteOffset,assembly.ends.byteOffset,assembly.offsets.byteOffset,assembly.out.byteOffset,count);
            [err,normA,normF,normX]=assembly.out;
        } else {
        for(let i=0;i<count;i++) {
            let v=F[i],row=0;
            for(let j=starts[i];j<=ends[i];j++){const a=J[offsets[i]+j];v+=a*x[j];row+=Math.abs(a);}
            err=Math.max(err,Math.abs(v));normA=Math.max(normA,row);normF=Math.max(normF,Math.abs(F[i]));normX=Math.max(normX,Math.abs(x[i]));
        }
        }
        const error=err/Math.max(Number.MIN_VALUE,normA*normX+normF);
        if(Number.isFinite(error))diagnostics.maximumBackwardError=Math.max(diagnostics.maximumBackwardError,error);
        return Number.isFinite(error)&&error<=64*Math.max(1,count)*Number.EPSILON&&x.every(Number.isFinite);
    }
    function fresh(J,F,scales) {
        valid=false;columns.clear();base.set(J);scale.set(scales);factor.fill(0);right.set(ends);
        if(assembly) {
            assembly.matrix.set(J);assembly.input.set(F);assembly.scales.set(scales);
            kernel.prepareGeneralBandLU(assembly.matrix.byteOffset,assembly.input.byteOffset,assembly.scales.byteOffset,
                assembly.starts.byteOffset,assembly.ends.byteOffset,assembly.offsets.byteOffset,factor.byteOffset,
                rhs.byteOffset,right.byteOffset,count,stride,kl,0);
        } else {
        for(let i=0;i<count;i++) {
            rhs[i]=-F[i]*scale[i];const target=i*stride+kl-i;
            for(let j=starts[i];j<=ends[i];j++)factor[target+j]=J[offsets[i]+j]*scale[i]*scale[j];
        }
        }
        diagnostics.factorizations++;
        if(kernel.factorRetainedGeneralBandLU(factor.byteOffset,rhs.byteOffset,right.byteOffset,count,kl,ku,pivots.byteOffset,lower.byteOffset)<0)return false;
        for(let i=0;i<count;i++)x[i]=rhs[i]*scale[i];
        valid=certify(J,F);return valid;
    }
    function update(J,F,changed) {
        for(let i=0;i<count;i++)rhs[i]=-F[i]*scale[i];backsolve(x);
        if(!changed.length){diagnostics.reuses++;return certify(J,F);}
        const n=changed.length,z=changed.map(row=>{
            let col=columns.get(row);if(col)return col;
            rhs.fill(0);rhs[row]=scale[row];col=new Float64Array(count);backsolve(col);
            if(columns.size<32)columns.set(row,col);return col;
        });
        const small=new Float64Array(n*n),c=new Float64Array(n);
        for(let i=0;i<n;i++) {
            const row=changed[i];small[i*n+i]=1;
            for(let j=starts[row];j<=ends[row];j++) {
                const delta=J[offsets[row]+j]-base[offsets[row]+j];if(!delta)continue;
                c[i]+=delta*x[j];for(let k=0;k<n;k++)small[i*n+k]+=delta*z[k][j];
            }
        }
        // Pivot the small nonsymmetric system; a singular/unstable update
        // always returns to a fresh full LU, never substitutes a diagonal.
        for(let k=0;k<n;k++) {
            let p=k;for(let i=k+1;i<n;i++)if(Math.abs(small[i*n+k])>Math.abs(small[p*n+k]))p=i;
            if(!Number.isFinite(small[p*n+k])||Math.abs(small[p*n+k])<1e-14)return false;
            if(p!==k){for(let j=k;j<n;j++){const v=small[k*n+j];small[k*n+j]=small[p*n+j];small[p*n+j]=v;}const v=c[k];c[k]=c[p];c[p]=v;}
            for(let i=k+1;i<n;i++) {const q=small[i*n+k]/small[k*n+k];for(let j=k+1;j<n;j++)small[i*n+j]-=q*small[k*n+j];c[i]-=q*c[k];}
        }
        for(let i=n-1;i>=0;i--){for(let j=i+1;j<n;j++)c[i]-=small[i*n+j]*c[j];c[i]/=small[i*n+i];}
        for(let i=0;i<count;i++)for(let j=0;j<n;j++)x[i]-=z[j][i]*c[j];
        if(!certify(J,F))return false;
        diagnostics.updates++;return true;
    }
    return {diagnostics,measureOriginalResidual(solution,error,originalRhs) {
        if(!assembly)return null;
        assembly.solution.set(solution);assembly.input.set(originalRhs);
        const residual=kernel.measureOriginalBandResidual(assembly.matrix.byteOffset,assembly.input.byteOffset,
            assembly.solution.byteOffset,assembly.starts.byteOffset,assembly.ends.byteOffset,assembly.offsets.byteOffset,assembly.error.byteOffset,count);
        error.set(assembly.error);return residual;
    },dispose() {if(!disposed){disposed=true;valid=false;if(idle.length<2&&lease.buffer.byteLength<=4*1024*1024)idle.push(lease);}},inverse(input,out) {
        if(!valid)return false;
        for(let i=0;i<count;i++)rhs[i]=input[i]*scale[i];
        backsolve(out);return out.every(Number.isFinite);
    },solve(J,F,scales,shift,direction) {
        if(disposed)throw new Error('Released incremental LU lease');
        if(shift!==0)throw new RangeError('Incremental contact prototype requires an unshifted matrix');
        const changed=[];
        if(valid)for(let i=0;i<count;i++) {
            for(let j=starts[i];j<=ends[i];j++)if(J[offsets[i]+j]!==base[offsets[i]+j]){changed.push(i);break;}
        }
        if(valid)diagnostics.changedRows[changed.length]=(diagnostics.changedRows[changed.length]??0)+1;
        let ok=false;
        if(valid&&changed.length<=maxRank) {ok=update(J,F,changed);if(!ok)diagnostics.updateFailures++;}
        else if(valid)diagnostics.rankResets++;
        if(!ok)ok=fresh(J,F,scales);
        if(ok)for(let i=0;i<count;i++)direction[i]=x[i]/scales[i];
        return ok;
    }};
}

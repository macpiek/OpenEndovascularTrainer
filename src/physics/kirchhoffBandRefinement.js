import {createKirchhoffLinearKernel} from './kirchhoffLinearKernel.js';

// Separate from the ordinary LU arena: measuring the original equations must
// not overwrite these factors between refinement RHSs. A lease is synchronous
// and invalidated on the next acquisition; never keep it across a yield.
let storage=null;
export function createBandRefinement(layout,count,matrixValues,scales) {
    const {starts,ends,offsets,kl,ku,entries}=layout,stride=2*kl+ku+1;
    const bytes=8*count*(stride+kl+6)+8*entries+128;
    if(!Number.isSafeInteger(bytes)||bytes<128||bytes>0x100000000)
        throw new RangeError('Invalid band refinement workspace');
    if(!storage||storage.buffer.byteLength<bytes) {
        let size=65536;while(size<bytes)size*=2;
        const kernel=createKirchhoffLinearKernel(size);
        storage={kernel,buffer:kernel.alloc(Uint8Array,size).buffer,views:new Map(),owner:null};
    }
    const arena=storage,owner={},key=`${count}/${kl}/${ku}/${entries}`;
    arena.owner=owner;
    let views=arena.views.get(key);
    if(!views) {
        let cursor=0;
        const take=(Type,n)=>{cursor=Math.ceil(cursor/8)*8;const v=new Type(arena.buffer,cursor,n);cursor+=v.byteLength;return v;};
        views={factor:take(Float64Array,count*stride),rhs:take(Float64Array,count),right:take(Int32Array,count),
            pivots:take(Int32Array,count),lower:take(Float64Array,count*kl),matrix:take(Float64Array,entries),
            input:take(Float64Array,count),scale:take(Float64Array,count),first:take(Int32Array,count),
            last:take(Int32Array,count),offset:take(Int32Array,count),out:take(Float64Array,5)};
        if(arena.views.size>=32)arena.views.delete(arena.views.keys().next().value);
        arena.views.set(key,views);
    }
    const {kernel}=arena,{factor,rhs,right,pivots,lower,matrix,input,scale,first,last,offset,out}=views;
    matrix.set(matrixValues);scale.set(scales);first.set(starts);last.set(ends);offset.set(offsets);
    let valid=false,disposed=false;
    const diagnostics={factorizations:0,backsolves:0};
    return {diagnostics,
        dispose(){disposed=true;valid=false;if(arena.owner===owner)arena.owner=null;},
        solve(F,direction) {
            if(disposed||storage!==arena||arena.owner!==owner)throw new Error('Expired band refinement lease');
            input.set(F);
            if(!valid) {
                factor.fill(0);right.set(last);
                kernel.prepareGeneralBandLU(matrix.byteOffset,input.byteOffset,scale.byteOffset,first.byteOffset,last.byteOffset,
                    offset.byteOffset,factor.byteOffset,rhs.byteOffset,right.byteOffset,count,stride,kl,0);
                diagnostics.factorizations++;
                if(kernel.factorRetainedGeneralBandLU(factor.byteOffset,rhs.byteOffset,right.byteOffset,count,kl,ku,pivots.byteOffset,lower.byteOffset)<0)return false;
                valid=true;
            } else {
                for(let i=0;i<count;i++)rhs[i]=-F[i]*scale[i];
                kernel.solveRetainedGeneralBandLU(factor.byteOffset,rhs.byteOffset,right.byteOffset,count,kl,ku,pivots.byteOffset,lower.byteOffset);
                diagnostics.backsolves++;
            }
            // Same scaled backward-error certificate as the ordinary band LU.
            // The caller still certifies/refines the ORIGINAL physical equations.
            kernel.measureGeneralBandLU(matrix.byteOffset,input.byteOffset,scale.byteOffset,first.byteOffset,last.byteOffset,
                offset.byteOffset,factor.byteOffset,rhs.byteOffset,right.byteOffset,count,stride,kl,out.byteOffset,0);
            const error=out[0]/Math.max(Number.MIN_VALUE,out[1]*out[3]+out[2]);
            if(!Number.isFinite(error)||error>64*Math.max(1,count)*Number.EPSILON||!rhs.every(Number.isFinite)) {
                valid=false;return false;
            }
            direction.set(rhs);return true;
        }
    };
}

import {createCoulombBandLU} from './kirchhoffCoulombBandLU.js';

// Exact substitution in the ALREADY assembled numerical equations. A row
// a*deltaLambda=0 with a!=0 determines deltaLambda=0. Its column may load any
// other row: that contribution is still exactly zero. Propagate only through
// previously proved zero increments; an unresolved dual cycle stays present.
// This selects no contact branch and removes no physical certificate row.
export function prepareCompositeZeroDualSolve(w,operator) {
    const {count,dual,residual,scales}=w,{values:matrix,offsets:rowOffsets,columns,diagonal}=operator;
    const known=w.zeroDualKnown??=new Uint8Array(count);known.set(w.fixedMask);
    let removed=0;
    for(let pass=0;pass<w.maxRowsPerNode;pass++) {
        let changed=false;
        for(const d of dual) {
            if(known[d]||residual[d]!==0||matrix[diagonal[d]]===0)continue;
            let determined=true;
            for(let k=rowOffsets[d];k<rowOffsets[d+1];k++)if(columns[k]!==d&&!known[columns[k]]&&matrix[k]!==0){determined=false;break;}
            if(determined){known[d]=1;removed++;changed=true;}
        }
        if(!changed)break;
    }
    if(!removed)return null;
    // Fixed primal coordinates keep their existing identity equations. Only
    // the proven dual coordinates are compressed out of the band.
    const excluded=w.zeroDualExcluded??=new Uint8Array(count);excluded.fill(0);
    for(const d of dual)if(known[d])excluded[d]=1;
    const key=Array.from(dual,d=>excluded[d]).join('');
    const cache=w.zeroDualSystems??=new Map();let system=cache.get(key);
    if(!system) {
        const original=Int32Array.from(Array.from({length:count},(_,i)=>i).filter(i=>!excluded[i])),index=new Int32Array(count).fill(-1);
        original.forEach((d,i)=>index[d]=i);
        let bandwidth=0;
        // Compile the ORIGINAL symbolic supports, including geometric
        // tangents from eliminated rows. A currently zero matrix coefficient
        // cannot shrink the reusable pattern or hide a future coefficient.
        for(const a of original)for(let k=rowOffsets[a];k<rowOffsets[a+1];k++)if(index[columns[k]]>=0)
            bandwidth=Math.max(bandwidth,Math.abs(index[a]-index[columns[k]]));
        const size=original.length,starts=new Int32Array(size),ends=new Int32Array(size),offsets=new Int32Array(size);let entries=0;
        for(let i=0;i<size;i++){starts[i]=Math.max(0,i-bandwidth);ends[i]=Math.min(size-1,i+bandwidth);offsets[i]=entries-starts[i];entries+=ends[i]-starts[i]+1;}
        const packedLayout={starts,ends,offsets,entries,kl:bandwidth,ku:bandwidth};
        system={original,index,count:size,removed,packedLayout,matrix:new Float64Array(entries),residual:new Float64Array(size),
            scales:new Float64Array(size),correction:new Float64Array(size),lu:createCoulombBandLU(packedLayout,size)};
        cache.set(key,system);if(cache.size>4)cache.delete(cache.keys().next().value);
    } else {cache.delete(key);cache.set(key,system);}
    const q=system.packedLayout;system.matrix.fill(0);
    for(let i=0;i<system.count;i++) {
        const a=system.original[i];system.scales[i]=scales[a];
        for(let k=rowOffsets[a];k<rowOffsets[a+1];k++) {
            const j=system.index[columns[k]];if(j>=0)system.matrix[q.offsets[i]+j]=matrix[k];
        }
    }
    return system;
}

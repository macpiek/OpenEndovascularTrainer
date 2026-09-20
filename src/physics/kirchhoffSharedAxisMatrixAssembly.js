import {sharedAxisEffectiveGap} from './kirchhoffSharedAxisCompliance.js';
// Numeric data belongs to one immutable linear call. Address maps belong to
// its bounded layout workspace; changing row supports are read on every assembly.
// No matrix values survive a new pose, reaction update, or cancelled call.
function commonPrimal(chain, context) {
    if(context.primal)return context.primal;
    const n=chain.layout.dofCount,half=chain.layout.band-1,width=2*half+1;
    const values=new Float64Array(n*width),included=new Set();
    if(chain.tangent) {
        for(let i=0;i<n;i++)for(let j=Math.max(0,i-half);j<=Math.min(n-1,i+half);j++) {
            const at=i*width+j-i+half,v=chain.tangent[at];if(v)values[at]+=v;
        }
    } else {
        for(let i=0;i<n;i++)for(let j=Math.max(0,i-half);j<=i;j++) {
            const v=chain.hessian[i*chain.layout.band+i-j];
            if(v){values[i*width+j-i+half]+=v;if(i!==j)values[j*width+i-j+half]+=v;}
        }
    }
    // Only the invariant Hessian prefix can be moved here without reordering
    // floating-point additions. Walls without Hessians contribute nothing;
    // the first wall Hessian stops the prefix even if currently inactive.
    for(const r of context.rows) {
        if(!r.geometricHessian)continue;
        if(r.kind!=='length'||r.dofs.some(p=>r.dofs.some(q=>Math.abs(p-q)>half)))break;
        const m=r.dofs.length;
        for(let i=0;i<m;i++)for(let j=0;j<m;j++) {
            const v=r.geometricHessian[i*m+j];if(v)values[r.dofs[i]*width+r.dofs[j]-r.dofs[i]+half]+=v;
        }
        included.add(r);
    }
    return context.primal={values,included,n,half,width};
}
function mapsFor(w,base) {
    // Only row starts and physical-to-packed indices are needed for a band
    // matrix. Expanding every 6x6 Hessian into a separate address array costs
    // more than its writes, especially for the many rows with no Hessian.
    let maps=w.assemblyMaps;
    if(!maps||maps.n!==base.n||maps.half!==base.half) {
        maps=w.assemblyMaps={n:base.n,half:base.half,
            primalOffsets:Int32Array.from(w.primal,p=>w.band.offsets[p]),
            dualOffsets:Int32Array.from(w.dual,d=>w.band.offsets[d])};
    }
    return maps;
}
function addHessian(w,map,r) {
    const {matrix:A,primal,band:{starts,ends}}=w,m=r.dofs.length;
    for(let i=0;i<m;i++) {
        const p=r.dofs[i],base=map.primalOffsets[p],row=primal[p];
        for(let j=0;j<m;j++) {
            const v=r.geometricHessian[i*m+j];if(!v)continue;
            const column=primal[r.dofs[j]];
            if(column<starts[row]||column>ends[row])throw new RangeError('Shared axis matrix support overflow');
            A[base+column]+=v;
        }
    }
}
function add(A,at,v) {
    if(!v)return;
    // A zero outside the band was legal in the original add() path. Validate
    // only nonzero writes, including a support that was formerly all zeros.
    if(at<0)throw new RangeError('Shared axis matrix support overflow');
    A[at]+=v;
}
export function assemblePreparedSharedAxisMatrix(w,chain,{rows,gradient,fixed,tolerance,activeSet,inactiveRows=[]},context) {
    const base=commonPrimal(chain,context),maps=mapsFor(w,base);
    const A=w.matrix,F=w.residual;A.fill(0);F.fill(0);w.fixed.fill(0);
    for(let i=0;i<w.primal.length;i++){const p=w.primal[i];F[p]=gradient[i];w.fixed[p]=fixed[i];}
    // This map is determined by the packed layout, not by changing geometry
    // or Jacobian values. Read current supports directly; no copied row graph.
    const {starts,ends}=w.band,primal=w.primal;
    for(let i=0;i<base.n;i++) {
        const row=primal[i],offset=maps.primalOffsets[i],source=i*base.width-i+base.half;
        for(let j=Math.max(0,i-base.half);j<=Math.min(base.n-1,i+base.half);j++) {
            const v=base.values[source+j];if(!v)continue;
            const column=primal[j];
            if(column<starts[row]||column>ends[row])throw new RangeError('Shared axis matrix support overflow');
            A[offset+column]+=v;
        }
    }
    for(let index=0;index<rows.length;index++) {
        const r=rows[index],d=w.dual[index],sign=r.kind==='wall'?-1:1,dualOffset=maps.dualOffsets[index];
        const active=r.kind==='length'||activeSet[index];let movable=false;
        for(let k=0;k<r.dofs.length;k++) {
            const p=r.dofs[k],row=primal[p];
            if(!fixed[p]&&r.jacobian[k]!==0)movable=true;
            add(A,d<starts[row]||d>ends[row]?-1:maps.primalOffsets[p]+d,sign*r.jacobian[k]);
        }
        for(let k=0;k<(r.extraForceDofs?.length??0);k++) {
            const p=r.extraForceDofs[k],row=primal[p];
            add(A,d<starts[row]||d>ends[row]?-1:maps.primalOffsets[p]+d,r.extraForceJacobian[k]);
        }
        if(!active||(!movable&&!r.compliance)) {
            if(active&&(r.kind==='length'?Math.abs(r.gap)>tolerance:r.gap < -tolerance))throw new RangeError('Incompatible fixed shared axis constraint');
            add(A,dualOffset+d,1);F[d]=r.multiplier;
        } else {
            F[d]=sharedAxisEffectiveGap(r);
            if(r.compliance)add(A,dualOffset+d,r.compliance);
            for(let k=0;k<r.dofs.length;k++) {
                const column=primal[r.dofs[k]];
                add(A,column<starts[d]||column>ends[d]?-1:dualOffset+column,r.jacobian[k]);
            }
        }
        if(r.geometricHessian&&!base.included.has(r))addHessian(w,maps,r);
    }
    for(const r of inactiveRows)if(r.geometricHessian)addHessian(w,maps,r);
}

// Fixed nodes usually occupy only the sheath entrance. Cache their exact band
// write locations instead of scanning every matrix entry for every active set.
export function applyPreparedSharedAxisFixedMask(w) {
    let map=w.fixedAssemblyMap,same=!!map&&map.mask.length===w.fixed.length;
    if(same)for(let i=0;i<w.fixed.length;i++)if(map.mask[i]!==w.fixed[i]){same=false;break;}
    if(!same) {
        const zeros=[],ones=[],rhs=[],{starts,ends,offsets}=w.band;
        for(let i=0;i<w.count;i++) {
            if(w.fixed[i])rhs.push(i);
            for(let j=starts[i];j<=ends[i];j++)if(w.fixed[i]||w.fixed[j])
                (i===j?ones:zeros).push(offsets[i]+j);
        }
        map=w.fixedAssemblyMap={mask:w.fixed.slice(),zeros:Int32Array.from(zeros),ones:Int32Array.from(ones),rhs:Int32Array.from(rhs)};
    }
    for(let k=0;k<map.zeros.length;k++)w.matrix[map.zeros[k]]=0;
    for(let k=0;k<map.ones.length;k++)w.matrix[map.ones[k]]=1;
    for(let k=0;k<map.rhs.length;k++)w.residual[map.rhs[k]]=0;
}

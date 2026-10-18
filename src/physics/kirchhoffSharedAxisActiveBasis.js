import {createSharedAxisBasisKernel} from './kirchhoffSharedAxisBasisKernel.js';
const sign = row => row.kind === 'wall' ? -1 : 1;

// Rank preparation is synchronous: reuse scratch across remeshed states.
// Numerical prefixes remain private to their owner and immutable Jacobian token.
const linearizations=new WeakMap();
let basisWorkspace=null;

// Only normalized Jacobians are reusable across solves. Gaps, loads, Hessians
// and dual iterates still participate in every current active-set decision.
// Compare values, not object identities: callers may update arrays in place.
export function sharedAxisBasisLinearization(rows,fixed) {
    let saved=linearizations.get(fixed),same=!!saved&&saved.rows.length===rows.length&&saved.fixed.length===fixed.length;
    if(same)for(let i=0;i<fixed.length;i++)if(saved.fixed[i]!==fixed[i]){same=false;break;}
    if(same)for(let i=0;i<rows.length;i++) {
        const a=rows[i],b=saved.rows[i];
        if(a.kind!==b.kind||a.compliance!==b.compliance||a.dofs.length!==b.dofs.length||a.jacobian.length!==b.jacobian.length){same=false;break;}
        for(let j=0;j<a.dofs.length;j++)if(a.dofs[j]!==b.dofs[j]||!Object.is(a.jacobian[j],b.jacobian[j])){same=false;break;}
        if(!same)break;
    }
    if(same)return saved.token;
    // The token is immutable, but its private comparison buffers need not be
    // allocated again for every Newton pose. No caller receives these arrays.
    saved??={fixed:fixed.slice(),rows:[]};
    saved.token=Symbol('basis-jacobians');
    if(saved.fixed.length!==fixed.length)saved.fixed=fixed.slice();
    else for(let i=0;i<fixed.length;i++)saved.fixed[i]=fixed[i];
    saved.rows.length=rows.length;
    for(let i=0;i<rows.length;i++) {
        const r=rows[i];let copy=saved.rows[i];
        if(!copy||copy.dofs.length!==r.dofs.length||copy.jacobian.length!==r.jacobian.length)
            copy=saved.rows[i]={kind:r.kind,compliance:r.compliance,dofs:r.dofs.slice(),jacobian:r.jacobian.slice()};
        else {
            copy.kind=r.kind;copy.compliance=r.compliance;
            for(let j=0;j<r.dofs.length;j++)copy.dofs[j]=r.dofs[j];
            for(let j=0;j<r.jacobian.length;j++)copy.jacobian[j]=r.jacobian[j];
        }
    }
    linearizations.set(fixed,saved);return saved.token;
}

function workspaceFor(fixed,rowCount,key=fixed) {
    let w=basisWorkspace;
    if(!w||w.rowCapacity<rowCount||w.dofCount<fixed.length) {
        const rowCapacity=Math.max(w?.rowCapacity??0,Math.ceil(rowCount/128)*128),dofCount=Math.max(w?.dofCount??0,Math.ceil(fixed.length/128)*128);
        w=basisWorkspace={rowCount,rowCapacity,dofCount,owner:key,pool:[],order:[],basis:[],normValues:[],stamp:0,
            pivotAt:new Int32Array(dofCount),pivotQueue:[],queued:new Uint32Array(rowCapacity),
            orderKinds:new Uint8Array(rowCapacity),orderActive:new Uint8Array(rowCapacity),
            cachedOrder:[],cachedBasisCounts:[],cachedCount:0,basisCache:null,
            spatialMarks:new Uint32Array(dofCount),reactionMarks:new Uint32Array(rowCapacity),
            next:new Float64Array(rowCapacity),forceError:new Float64Array(dofCount)};
    } else if(w.owner!==key||w.rowCount!==rowCount) {
        // Rank preparation is synchronous and returns no scratch views. Reuse
        // numeric storage across remeshed private states, but never reuse their
        // numerical prefix. This also makes interleaved solvers independent.
        w.owner=key;w.rowCount=rowCount;w.cachedCount=0;w.basisCache=null;
        w.orderKinds.fill(0);w.orderActive.fill(0);w.order.length=0;
    }
    return w;
}

function queuePivot(w,pivot,stamp) {
    if(pivot<0||w.queued[pivot]===stamp)return;
    w.queued[pivot]=stamp;
    const heap=w.pivotQueue;let i=heap.length;heap.push(pivot);
    while(i>0){const parent=(i-1)>>>1;if(heap[parent]<=pivot)break;heap[i]=heap[parent];i=parent;}
    heap[i]=pivot;
}
function nextPivot(heap) {
    const result=heap[0],last=heap.pop();
    if(heap.length) {
        let i=0;
        while(2*i+1<heap.length) {
            let child=2*i+1;if(child+1<heap.length&&heap[child+1]<heap[child])child++;
            if(heap[child]>=last)break;heap[i]=heap[child];i=child;
        }
        heap[i]=last;
    }
    return result;
}
function prepareOrder(w,rows,activeSet) {
    let changed=false;
    for(let i=0;i<rows.length;i++) {
        const kind=rows[i].kind==='length'?1:rows[i].kind==='wall'&&!rows[i].compliance?2:0,active=activeSet[i]?1:0;
        if(w.orderKinds[i]!==kind||w.orderActive[i]!==active)changed=true;
        w.orderKinds[i]=kind;w.orderActive[i]=active;
    }
    if(changed) {
        w.order.length=0;
        for(const kind of [1,2])for(let i=0;i<rows.length;i++)if(w.orderKinds[i]===kind&&w.orderActive[i])w.order.push(i);
    }
}
function nextRow(w,index) {
    let r=w.pool[index];
    if(!r)r=w.pool[index]={v:w.kernel?.vector(index)??new Float64Array(w.dofCount),c:new Float64Array(w.rowCapacity),
        support:w.kernel?.indices(index)??new Int32Array(w.dofCount),supportCount:0,coefficients:[],pivot:0};
    // Every nonzero belongs to its recorded support. Exact zeros removed by
    // compaction are already zero, so global padding needs no clearing.
    for(let k=0;k<r.supportCount;k++)r.v[r.support[k]]=0;
    for(const i of r.coefficients)r.c[i]=0;
    r.supportCount=r.coefficients.length=0;r.coefficientsValid=false;
    r.numericRevision=(r.numericRevision??0)+1;
    return r;
}
function compactExactNonzeros(indices,values) {
    let count=0;for(let i=0;i<indices.length;i++)if(values[indices[i]]!==0)indices[count++]=indices[i];
    indices.length=count;
}
function supportedNorm(w,support,values,count) {
    const scratch=w.normValues;scratch.length=count;
    for(let i=0;i<count;i++)scratch[i]=values[support[i]];
    // Keep the native Math.hypot scaling and argument order. A naive squared
    // sum would change rank decisions on extreme-magnitude rows.
    return Math.hypot(...scratch);
}

// Record elementary eliminations cheaply. The reaction-space coefficients
// are needed only for a dependent row. Reconstruct them in the original order
// on that uncommon path, preserving the same floating-point operations.
function materializeDependentCoefficients(w,basis,dependent) {
    // A dependent contact usually touches only a small part of the chain.
    // Reconstruct its elimination ancestors, not every unrelated basis row.
    // A normalized prefix's coefficients stay valid until nextRow rebuilds
    // that row (including changes of owner, Jacobians or active-set order).
    const needed=w.coefficientNeeded??=new Uint8Array(w.rowCapacity);
    needed.fill(0,0,basis.length+1);needed[basis.length]=1;
    for(let i=basis.length;i>=0;i--) {
        const r=i===basis.length?dependent:basis[i];
        if(!needed[i]||r.coefficientsValid)continue;
        for(let k=0;k<r.eliminations.length;k+=2)needed[r.eliminations[k]]=1;
    }
    for(let i=0;i<=basis.length;i++) {
        const r=i===basis.length?dependent:basis[i];
        if(!needed[i]||r.coefficientsValid)continue;
        for(const j of r.coefficients)r.c[j]=0;
        r.coefficients.length=0;r.coefficients.push(r.source);r.c[r.source]=1;
        if(++w.stamp>=0xffffffff){w.spatialMarks.fill(0);w.reactionMarks.fill(0);w.queued.fill(0);w.stamp=1;}
        const stamp=w.stamp;w.reactionMarks[r.source]=stamp;
        for(let k=0;k<r.eliminations.length;k+=2) {
            const q=basis[r.eliminations[k]],projection=r.eliminations[k+1];
            for(const j of q.coefficients) {
                r.c[j]-=projection*q.c[j];
                if(r.c[j]!==0&&w.reactionMarks[j]!==stamp){w.reactionMarks[j]=stamp;r.coefficients.push(j);}
            }
        }
        if(r!==dependent) {
            for(const j of r.coefficients)r.c[j]/=r.factor;
            compactExactNonzeros(r.coefficients,r.c);
        }
        r.coefficientsValid=true;
    }
    return dependent.c;
}

/** Repair only the linear solver's working dual representation. The physical
 * incoming multipliers and Hessian are untouched. A null-space pivot preserves
 * generalized force while replacing dependent active equalities; released rows
 * remain inequalities in the active-set search. Length reactions are signed,
 * wall reactions nonnegative. Fixed degrees of freedom cannot restrict motion.
 */
export function prepareSharedAxisActiveBasis({rows,fixed,activeSet,dual,trace,reuseStructure=true,basisCache=null,basisWorkspaceKey=fixed,lazyBasisCoefficients=false,wasmBasis=false}) {
    let pivots=0;
    // A reduced identity projection can borrow the owning state's larger
    // basis buffers. Only storage is shared: each linearization token resets
    // cached coefficients before a different indexing/fixed mask is used.
    const w=workspaceFor(fixed,rows.length,basisWorkspaceKey),{spatialMarks,reactionMarks,order,basis}=w;
    let kernel=null;
    if(wasmBasis&&lazyBasisCoefficients&&w.dofCount&&w.rowCapacity) {
        if(!w.kernel) {
            w.kernel=createSharedAxisBasisKernel(w.dofCount,w.rowCapacity);
            if(w.kernel)for(let i=0;i<w.pool.length;i++)w.kernel.attach(i,w.pool[i]);
        }
        kernel=w.kernel;
    }
    if(!reuseStructure||!basisCache||w.basisCache!==basisCache||w.lazyBasisCoefficients!==lazyBasisCoefficients){w.cachedCount=0;w.basisCache=basisCache;w.lazyBasisCoefficients=lazyBasisCoefficients;}
    for(let pass=0;pass<=rows.length;pass++) {
        // Sparse row echelon elimination: a Kirchhoff length/contact row
        // has local support. Orthogonalizing every row against the entire
        // growing chain destroys that locality and costs cubic work.

        if(reuseStructure)prepareOrder(w,rows,activeSet);
        else {
            order.length=0;
            for(let i=0;i<rows.length;i++)if(rows[i].kind==='length'&&activeSet[i])order.push(i);
            for(let i=0;i<rows.length;i++)if(rows[i].kind==='wall'&&!rows[i].compliance&&activeSet[i])order.push(i);
            w.orderKinds.fill(0);w.orderActive.fill(0);
        }
        // This token is scoped to one immutable linearization. Reuse only
        // the common input prefix; a changed active row invalidates everything
        // after it. Dual values/gaps select pivots but never enter these cached
        // normalized Jacobian rows. New linearizations get a new token.
        let prefix=0;
        if(reuseStructure&&basisCache)while(prefix<w.cachedCount&&prefix<order.length&&w.cachedOrder[prefix]===order[prefix])prefix++;
        const kept=prefix?w.cachedBasisCounts[prefix-1]:0;
        basis.length=kept;for(let i=0;i<kept;i++)basis[i]=w.pool[i];
        w.pivotAt.fill(-1);for(let i=0;i<kept;i++)w.pivotAt[basis[i].pivot]=i;
        if(kernel){kernel.pivotAt.set(w.pivotAt);for(let i=0;i<kept;i++)kernel.store(i,basis[i],basis[i].numericRevision);}
        w.cachedCount=prefix;
        const remember=(position,index)=>{if(reuseStructure&&basisCache){w.cachedOrder[position]=index;w.cachedBasisCounts[position]=basis.length;w.cachedCount=position+1;}};
        let dependent=null;
        for(let position=prefix;position<order.length;position++) {
            const index=order[position];
            const row=rows[index],working=nextRow(w,basis.length),{v,c,support,coefficients}=working;
            if(lazyBasisCoefficients){working.source=index;(working.eliminations??=[]).length=0;}
            else coefficients.push(index);
            if(++w.stamp>=0xffffffff){spatialMarks.fill(0);reactionMarks.fill(0);w.queued.fill(0);w.stamp=1;}
            const stamp=w.stamp,rowSign=sign(row);
            for(let k=0;k<row.dofs.length;k++){const p=row.dofs[k];if(!fixed[p]) {
                v[p]+=rowSign*row.jacobian[k];
                if(v[p]!==0&&spatialMarks[p]!==stamp){spatialMarks[p]=stamp;support[working.supportCount++]=p;}
            }}
            const originalNorm=supportedNorm(w,support,v,working.supportCount);if(originalNorm===0){remember(position,index);continue;}
            if(!lazyBasisCoefficients){c[index]=1;reactionMarks[index]=stamp;}
            // Long chains have mostly local support. Visit only nonzero
            // pivots, in the same increasing order as the ordinary scan.
            // Include fill introduced by preceding eliminations; never drop
            // small coefficients or change the numerical rank threshold.
            const sparseElimination=basis.length>64;
            if(kernel)kernel.eliminate(working,basis.length,stamp);
            else {
            w.pivotQueue.length=0;
            if(sparseElimination)for(let k=0;k<working.supportCount;k++) {
                const j=support[k];if(v[j]!==0)queuePivot(w,w.pivotAt[j],stamp);
            }
            for(let scan=0;sparseElimination?w.pivotQueue.length:scan<basis.length;scan++) {
                const qi=sparseElimination?nextPivot(w.pivotQueue):scan;
                const q=basis[qi];
                const projection=v[q.pivot];if(projection===0)continue;
                // One f64 update per coefficient, in the original support
                // order. Reuse its value for both sparse bookkeeping checks.
                const qv=q.v,qs=q.support,pivotAt=w.pivotAt,queued=w.queued;
                for(let k=0;k<q.supportCount;k++) {
                    const j=qs[k],value=v[j]-projection*qv[j];v[j]=value;
                    if(value!==0) {
                        if(spatialMarks[j]!==stamp){spatialMarks[j]=stamp;support[working.supportCount++]=j;}
                        const pivot=pivotAt[j];
                        if(sparseElimination&&pivot>qi&&queued[pivot]!==stamp)queuePivot(w,pivot,stamp);
                    }
                }
                v[q.pivot]=0;
                if(lazyBasisCoefficients)working.eliminations.push(qi,projection);
                else for(const j of q.coefficients) {
                    c[j]-=projection*q.c[j];
                    if(c[j]!==0&&reactionMarks[j]!==stamp){reactionMarks[j]=stamp;coefficients.push(j);}
                }
            }
            }
            let pivot=0,maximum=Math.abs(v[0]);
            for(let k=0;k<working.supportCount;k++) {
                const j=support[k];
                const magnitude=Math.abs(v[j]);
                if(magnitude>maximum||magnitude===maximum&&j<pivot){pivot=j;maximum=magnitude;}
            }
            if(supportedNorm(w,support,v,working.supportCount)<=1e-11*originalNorm){dependent=lazyBasisCoefficients?materializeDependentCoefficients(w,basis,working):c;break;}
            const factor=v[pivot];
            for(let k=0;k<working.supportCount;k++)v[support[k]]/=factor;
            if(lazyBasisCoefficients)working.factor=factor;
            else for(const j of coefficients)c[j]/=factor;
            // Only exact zeros disappear. Small terms remain available for
            // rank detection and the unchanged generalized-force certificate.
            working.pivot=pivot;
            let count=0;for(let k=0;k<working.supportCount;k++)if(v[support[k]]!==0)support[count++]=support[k];
            working.supportCount=count;compactExactNonzeros(coefficients,c);
            w.pivotAt[pivot]=basis.length;
            if(kernel){kernel.pivotAt[pivot]=basis.length;kernel.store(basis.length,working,working.numericRevision);}
            basis.push(working);remember(position,index);
        }
        if(!dependent)return {converged:true,pivots};
        // Along this null direction the force is constant. Choose the sign
        // improving the dual objective, not an arbitrary contact to delete.
        let slope=0;
        for(let i=0;i<rows.length;i++)slope+=sign(rows[i])*rows[i].gap*dependent[i];
        let orientation=slope>=0?1:-1;
        const bound=direction=>{
            let step=Infinity,drop=-1;
            for(let i=0;i<rows.length;i++)if(activeSet[i]&&rows[i].kind==='wall'&&direction*dependent[i]<-1e-12) {
                const limit=Math.max(0,dual[i])/(-direction*dependent[i]);
                if(limit<step){step=limit;drop=i;}
            }
            return {step,drop};
        };
        let {step,drop}=bound(orientation);
        if(drop<0&&Math.abs(slope)<1e-12) {orientation=-orientation;({step,drop}=bound(orientation));}
        if(drop<0){
            if(trace?.captureConflicts)trace.push({kind:'incompatible-active-constraints',slope,orientation,
                rows:rows.flatMap((r,i)=>Math.abs(dependent[i])>1e-12?[{index:i,id:r.id,kind:r.kind,edge:r.edge,
                    coefficient:dependent[i],gap:r.gap,multiplier:r.multiplier,jacobian:Array.from(r.jacobian)}]:[])});
            return {converged:false,pivots,failure:'incompatible-active-constraints'};
        }
        const next=w.next.subarray(0,rows.length);next.set(dual);
        for(let i=0;i<rows.length;i++) {
            next[i]+=orientation*step*dependent[i];
            if(rows[i].kind==='wall')next[i]=Math.max(0,next[i]);
        }
        next[drop]=0;
        const forceError=w.forceError;forceError.fill(0);
        let forceScale=1;
        for(let i=0;i<rows.length;i++) {
            const delta=next[i]-dual[i];
            for(let k=0;k<rows[i].dofs.length;k++){const p=rows[i].dofs[k];if(!fixed[p]) {
                const f=sign(rows[i])*delta*rows[i].jacobian[k];forceError[p]+=f;forceScale+=Math.abs(f);
            }}
        }
        let maximumForceError=0;for(let i=0;i<forceError.length;i++)maximumForceError=Math.max(maximumForceError,Math.abs(forceError[i]));
        if(maximumForceError>1e-10*forceScale)
            return {converged:false,pivots,failure:'active-basis-force-error'};
        dual.set(next);activeSet[drop]=0;pivots++;
        trace?.push({kind:'basis-pivot',drop:rows[drop].id??drop,step,slope,
            maximumForceError});
    }
    return {converged:false,pivots,failure:'active-basis-limit'};
}

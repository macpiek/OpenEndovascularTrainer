const sign = row => row.kind === 'wall' ? -1 : 1;

// A fixed mask identifies the lifetime of one spatial solve. Repeated active
// sets reuse numerical storage, but no rank decision or coefficient survives
// the next call. Weak ownership lets a discarded spatial state release it.
const workspaces=new WeakMap();
function workspaceFor(fixed,rowCount) {
    let w=workspaces.get(fixed);
    if(!w||w.rowCount!==rowCount||w.dofCount!==fixed.length) {
        w={rowCount,dofCount:fixed.length,pool:[],order:[],basis:[],normValues:[],stamp:0,
            spatialMarks:new Uint32Array(fixed.length),reactionMarks:new Uint32Array(rowCount),
            next:new Float64Array(rowCount),forceError:new Float64Array(fixed.length)};
        workspaces.set(fixed,w);
    }
    return w;
}
function nextRow(w,index,dofCount) {
    let r=w.pool[index];
    if(!r)r=w.pool[index]={v:new Float64Array(dofCount),c:new Float64Array(w.rowCount),support:[],coefficients:[],pivot:0};
    // Every nonzero belongs to its recorded support. Exact zeros removed by
    // compaction are already zero, so global padding needs no clearing.
    for(const i of r.support)r.v[i]=0;
    for(const i of r.coefficients)r.c[i]=0;
    r.support.length=r.coefficients.length=0;
    return r;
}
function compactExactNonzeros(indices,values) {
    let count=0;for(let i=0;i<indices.length;i++)if(values[indices[i]]!==0)indices[count++]=indices[i];
    indices.length=count;
}
function supportedNorm(w,support,values) {
    const scratch=w.normValues;scratch.length=support.length;
    for(let i=0;i<support.length;i++)scratch[i]=values[support[i]];
    // Keep the native Math.hypot scaling and argument order. A naive squared
    // sum would change rank decisions on extreme-magnitude rows.
    return Math.hypot(...scratch);
}

/** Repair only the linear solver's working dual representation. The physical
 * incoming multipliers and Hessian are untouched. A null-space pivot preserves
 * generalized force while replacing dependent active equalities; released rows
 * remain inequalities in the active-set search. Length reactions are signed,
 * wall reactions nonnegative. Fixed degrees of freedom cannot restrict motion.
 */
export function prepareSharedAxisActiveBasis({rows,fixed,activeSet,dual,trace}) {
    let pivots=0;
    const w=workspaceFor(fixed,rows.length),{spatialMarks,reactionMarks,order,basis}=w;
    for(let pass=0;pass<=rows.length;pass++) {
        // Sparse row echelon elimination: a Kirchhoff length/contact row
        // has local support. Orthogonalizing every row against the entire
        // growing chain destroys that locality and costs cubic work.
        basis.length=order.length=0;
        for(let i=0;i<rows.length;i++)if(rows[i].kind==='length'&&activeSet[i])order.push(i);
        for(let i=0;i<rows.length;i++)if(rows[i].kind==='wall'&&activeSet[i])order.push(i);
        let dependent=null;
        for(const index of order) {
            const row=rows[index],working=nextRow(w,basis.length,fixed.length),{v,c,support,coefficients}=working;
            coefficients.push(index);
            if(++w.stamp>=0xffffffff){spatialMarks.fill(0);reactionMarks.fill(0);w.stamp=1;}
            const stamp=w.stamp,rowSign=sign(row);
            for(let k=0;k<row.dofs.length;k++){const p=row.dofs[k];if(!fixed[p]) {
                v[p]+=rowSign*row.jacobian[k];
                if(v[p]!==0&&spatialMarks[p]!==stamp){spatialMarks[p]=stamp;support.push(p);}
            }}
            const originalNorm=supportedNorm(w,support,v);if(originalNorm===0)continue;
            c[index]=1;reactionMarks[index]=stamp;
            for(const q of basis) {
                const projection=v[q.pivot];if(projection===0)continue;
                for(const j of q.support) {
                    v[j]-=projection*q.v[j];
                    if(v[j]!==0&&spatialMarks[j]!==stamp){spatialMarks[j]=stamp;support.push(j);}
                }
                v[q.pivot]=0;
                for(const j of q.coefficients) {
                    c[j]-=projection*q.c[j];
                    if(c[j]!==0&&reactionMarks[j]!==stamp){reactionMarks[j]=stamp;coefficients.push(j);}
                }
            }
            let pivot=0;
            for(const j of support)if(Math.abs(v[j])>Math.abs(v[pivot])||Math.abs(v[j])===Math.abs(v[pivot])&&j<pivot)pivot=j;
            if(supportedNorm(w,support,v)<=1e-11*originalNorm){dependent=c;break;}
            const factor=v[pivot];
            for(const j of support)v[j]/=factor;
            for(const j of coefficients)c[j]/=factor;
            // Only exact zeros disappear. Small terms remain available for
            // rank detection and the unchanged generalized-force certificate.
            working.pivot=pivot;compactExactNonzeros(support,v);compactExactNonzeros(coefficients,c);basis.push(working);
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
        if(drop<0)return {converged:false,pivots,failure:'incompatible-active-constraints'};
        const next=w.next;next.set(dual);
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

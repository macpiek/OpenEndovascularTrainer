const sign = row => row.kind === 'wall' ? -1 : 1;

/** Repair only the linear solver's working dual representation. The physical
 * incoming multipliers and Hessian are untouched. A null-space pivot preserves
 * generalized force while replacing dependent active equalities; released rows
 * remain inequalities in the active-set search. Length reactions are signed,
 * wall reactions nonnegative. Fixed degrees of freedom cannot restrict motion.
 */
export function prepareSharedAxisActiveBasis({rows,fixed,activeSet,dual,trace}) {
    let pivots=0,stamp=0;
    // Reuse membership markers while each basis row owns its numerical values.
    // This keeps typed-array arithmetic without scanning global zero padding.
    const spatialMarks=new Uint32Array(fixed.length),reactionMarks=new Uint32Array(rows.length);
    for(let pass=0;pass<=rows.length;pass++) {
        // Sparse row echelon elimination: a Kirchhoff length/contact row
        // has local support. Orthogonalizing every row against the entire
        // growing chain destroys that locality and costs cubic work.
        const basis=[];
        const order=rows.flatMap((r,i)=>r.kind==='length'&&activeSet[i]?[i]:[])
            .concat(rows.flatMap((r,i)=>r.kind==='wall'&&activeSet[i]?[i]:[]));
        let dependent=null;
        for(const index of order) {
            const row=rows[index],v=new Float64Array(fixed.length),c=new Float64Array(rows.length),support=[],coefficients=[index];
            stamp++;
            row.dofs.forEach((p,k)=>{if(!fixed[p]) {
                v[p]+=sign(row)*row.jacobian[k];
                if(v[p]!==0&&spatialMarks[p]!==stamp){spatialMarks[p]=stamp;support.push(p);}
            }});
            const originalNorm=Math.hypot(...support.map(j=>v[j]));if(originalNorm===0)continue;
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
            if(Math.hypot(...support.map(j=>v[j]))<=1e-11*originalNorm){dependent=c;break;}
            const factor=v[pivot];
            for(const j of support)v[j]/=factor;
            for(const j of coefficients)c[j]/=factor;
            // Only exact zeros disappear. Small terms remain available for
            // rank detection and the unchanged generalized-force certificate.
            basis.push({v,c,pivot,support:support.filter(j=>v[j]!==0),coefficients:coefficients.filter(j=>c[j]!==0)});
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
        const next=dual.slice();
        for(let i=0;i<rows.length;i++) {
            next[i]+=orientation*step*dependent[i];
            if(rows[i].kind==='wall')next[i]=Math.max(0,next[i]);
        }
        next[drop]=0;
        const forceError=new Float64Array(fixed.length);
        let forceScale=1;
        for(let i=0;i<rows.length;i++) {
            const delta=next[i]-dual[i];
            rows[i].dofs.forEach((p,k)=>{if(!fixed[p]) {
                const f=sign(rows[i])*delta*rows[i].jacobian[k];forceError[p]+=f;forceScale+=Math.abs(f);
            }});
        }
        if(Math.max(...forceError.map(Math.abs))>1e-10*forceScale)
            return {converged:false,pivots,failure:'active-basis-force-error'};
        dual.set(next);activeSet[drop]=0;pivots++;
        trace?.push({kind:'basis-pivot',drop:rows[drop].id??drop,step,slope,
            maximumForceError:Math.max(...forceError.map(Math.abs))});
    }
    return {converged:false,pivots,failure:'active-basis-limit'};
}

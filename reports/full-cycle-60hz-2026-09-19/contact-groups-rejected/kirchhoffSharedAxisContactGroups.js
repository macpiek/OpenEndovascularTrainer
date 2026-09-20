/** A private direction model for nearly parallel finite-face witnesses at
 * one material point. Physical definitions and final nonlinear certificates
 * remain ungrouped. Reaction weights preserve their incoming resultant. */
export function groupSharedAxisContacts(rows,fixed,{gapTolerance,angleTolerance=.001}={}) {
    if(!(gapTolerance>0)||!Number.isFinite(gapTolerance)||!(angleTolerance>0&&angleTolerance<Math.PI/2))
        throw new RangeError('Positive finite contact grouping tolerances required');
    const buckets=new Map(),groups=[],cosine=Math.cos(angleTolerance);
    for(let i=0;i<rows.length;i++) {
        const r=rows[i],norm=Math.hypot(...r.jacobian);
        if(r.kind!=='wall'||!r.witness||!(norm>1e-12)||r.multiplier<0||
            !r.dofs.some((p,k)=>!fixed[p]&&r.jacobian[k]!==0)) {
            groups.push({indices:[i]});continue;
        }
        const key=`${r.edge}/${r.witness.t}/${r.witness.owner??''}/${r.dofs.join('.')}`;
        let bucket=buckets.get(key);if(!bucket)buckets.set(key,bucket=[]);
        const sameForceColumn=(a,b)=>{
            if((a.extraForceDofs?.length??0)!==(b.extraForceDofs?.length??0))return false;
            return !a.extraForceDofs||a.extraForceDofs.every((p,k)=>p===b.extraForceDofs[k]&&Math.abs(a.extraForceJacobian[k]-b.extraForceJacobian[k])<1e-9);
        };
        const group=bucket.find(g=>sameForceColumn(r,rows[g.indices[0]])&&Math.max(g.maxGap,r.gap)-Math.min(g.minGap,r.gap)<=gapTolerance&&
            g.indices.every(j=>r.jacobian.reduce((sum,v,k)=>sum+v*rows[j].jacobian[k],0)>=cosine*norm*g.norms.get(j)));
        if(group){group.indices.push(i);group.norms.set(i,norm);group.minGap=Math.min(group.minGap,r.gap);group.maxGap=Math.max(group.maxGap,r.gap);}
        else {const next={indices:[i],norms:new Map([[i,norm]]),minGap:r.gap,maxGap:r.gap};bucket.push(next);groups.push(next);}
    }
    const grouped=groups.map(group=>{
        const indices=group.indices,first=rows[indices[0]];
        if(indices.length===1){group.weights=[1];return first;}
        const load=indices.reduce((sum,i)=>sum+rows[i].multiplier,0);
        let closest=indices[0];for(const i of indices)if(rows[i].gap<rows[closest].gap)closest=i;
        const weights=group.weights=indices.map(i=>load>0?rows[i].multiplier/load:Number(i===closest));
        const jacobian=new Array(first.dofs.length).fill(0),extra=new Map();
        let gap=0,hessian;
        for(let at=0;at<indices.length;at++) {
            const r=rows[indices[at]],weight=weights[at];gap+=weight*r.gap;
            for(let k=0;k<jacobian.length;k++)jacobian[k]+=weight*r.jacobian[k];
            for(let k=0;k<(r.extraForceDofs?.length??0);k++) {
                const p=r.extraForceDofs[k];extra.set(p,(extra.get(p)??0)+weight*r.extraForceJacobian[k]);
            }
            if(r.geometricHessian) {
                hessian??=new Float64Array(jacobian.length**2);
                for(let k=0;k<hessian.length;k++)hessian[k]+=r.geometricHessian[k];
            }
        }
        const extraForceDofs=[...extra.keys()].sort((a,b)=>a-b);
        return {...first,id:`contact-group/${indices.join('.')}`,jacobian,gap,multiplier:load,geometricHessian:hessian,
            extraForceDofs:extraForceDofs.length?extraForceDofs:undefined,
            extraForceJacobian:extraForceDofs.length?extraForceDofs.map(p=>extra.get(p)):undefined};
    });
    return {rows:grouped,groups,removedRows:rows.length-grouped.length,
        expand(delta) {
            const result=new Float64Array(rows.length);
            groups.forEach((g,j)=>g.indices.forEach((i,k)=>{
                result[i]=g.indices.length===1?delta[j]:g.weights[k]*delta[j];
            }));
            return result;
        }};
}

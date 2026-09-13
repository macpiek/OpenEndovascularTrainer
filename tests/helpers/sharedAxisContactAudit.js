import { assembleSharedAxisNative } from '../../src/physics/kirchhoffSharedAxisNative.js';
import { solveSharedAxisLinear } from '../../src/physics/kirchhoffSharedAxisLinear.js';

const dot=(a,b)=>a.reduce((sum,x,i)=>sum+x*b[i],0);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];

// Independent row elimination, used only as a diagnostic oracle, never by
// the physical solver. Rows are dimensionless length-constraint gradients.
function rank(matrix) {
    const a=matrix.map(r=>r.slice());let result=0;
    if(!a.length)return 0;
    for(let j=0;j<a[0].length&&result<a.length;j++) {
        let pivot=result;
        for(let i=result+1;i<a.length;i++)if(Math.abs(a[i][j])>Math.abs(a[pivot][j]))pivot=i;
        if(Math.abs(a[pivot][j])<1e-9)continue;
        [a[result],a[pivot]]=[a[pivot],a[result]];
        const scale=a[result][j];for(let k=j;k<a[0].length;k++)a[result][k]/=scale;
        for(let i=result+1;i<a.length;i++) {
            const v=a[i][j];for(let k=j;k<a[0].length;k++)a[i][k]-=v*a[result][k];
        }
        result++;
    }
    return result;
}

export function auditSharedAxisContactRows(s,rows) {
    const free=Array.from(s.fixed).flatMap((v,i)=>v?[]:[i]);
    const active=rows.filter(r=>(r.kind==='length'||r.multiplier>1e-7)&&r.dofs.some((d,i)=>!s.fixed[d]&&r.jacobian[i]));
    const jacobian=active.map(r=>free.map(d=>r.dofs.includes(d)?r.jacobian[r.dofs.indexOf(d)]:0));
    const groups=new Map();
    for(const r of active)if(r.witness) {
        const key=`${r.edge}/${r.witness.t}`;
        if(!groups.has(key))groups.set(key,[]);groups.get(key).push(r);
    }
    const dependentGroups=[];
    for(const [point,group] of groups)if(group.length===4) {
        const [a,b,c,d]=group.map(r=>r.jacobian.slice(3)),det=dot(a,cross(b,c));
        if(Math.abs(det)<1e-12)continue;
        const coefficients=[dot(d,cross(b,c))/det,dot(a,cross(d,c))/det,dot(a,cross(b,d))/det];
        const normalResidual=d.map((v,j)=>v-coefficients[0]*a[j]-coefficients[1]*b[j]-coefficients[2]*c[j]);
        const gapResidual=group[3].gap-coefficients.reduce((sum,v,i)=>sum+v*group[i].gap,0);
        dependentGroups.push({point,ids:group.map(r=>r.id),gaps:group.map(r=>r.gap),reactions:group.map(r=>r.multiplier),
            determinant:det,coefficients,normalResidual,gapResidual});
    }
    return {activeEquations:active.length,jacobianRank:rank(jacobian),
        augmentedRank:rank(jacobian.map((r,i)=>[...r,active[i].gap])),dependentGroups};
}

export function auditSharedAxisDirection(s,mode) {
    const base=assembleSharedAxisNative(s,{tangentMode:mode});
    const rows=mode==='gauss-newton'?base.rows.map(r=>({...r,geometricHessian:undefined})):base.rows;
    const direction=solveSharedAxisLinear(s.mixed,s.chain,{rows,gradient:s.chain.gradient,fixed:s.fixed,tolerance:1e-7});
    return {mode,converged:direction.converged,failure:direction.failure??null,factorizations:direction.factorizations,
        ...auditSharedAxisContactRows(s,base.rows)};
}

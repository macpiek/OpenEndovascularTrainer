import {createSharedAxisLinear,iterateSharedAxisLinear} from './kirchhoffSharedAxisLinear.js';

// The correction has an identity primal block, not a material tangent.
// Keep the same spatial/dual ordering and active-set algorithm, but allocate
// only the band actually required by its constraint Jacobians. In reduced
// mode eliminate uncoupled spin variables exactly (their correction is zero);
// retain all positional unknowns and solve their reactions globally.
export function* iterateSharedAxisProjection(s,base,{reuseStructure=true,mode='reduced',reuseConstraintWork=false,reuseMatrixAssembly=false,maxActiveSetAttempts=undefined}={}) {
    let cache=s.projectionWorkspace;
    if(!cache||cache.layout!==s.layout||cache.definitions!==s.definitions||cache.zeroReactions.length!==s.definitions.length||cache.mode!==mode) {
        const positionOnly=mode==='reduced';
        const originalDofs=positionOnly?Array.from(s.layout.positions).flatMap(p=>[p,p+1,p+2]):null;
        const inverse=positionOnly?new Int32Array(s.layout.dofCount).fill(-1):null;
        originalDofs?.forEach((p,i)=>{inverse[p]=i;});
        const layout=mode==='reference'?s.layout:{...s.layout,band:1,dofCount:originalDofs?.length??s.layout.dofCount};
        const hessian=new Float64Array(layout.dofCount*layout.band);
        for(let i=0;i<layout.dofCount;i++)hessian[i*layout.band]=1;
        cache=s.projectionWorkspace={layout:s.layout,definitions:s.definitions,mode,
            chain:{layout,hessian},gradient:new Float64Array(layout.dofCount),
            originalDofs,inverse,fixed:positionOnly?new Uint8Array(layout.dofCount):s.fixed,
            increment:positionOnly?new Float64Array(s.layout.dofCount):null,
            zeroReactions:new Float64Array(s.definitions.length),rows:[],
            mixed:createSharedAxisLinear(layout,s.definitions,{lazy:true})};
    }
    const {rows}=cache;rows.length=base.rows.length;
    for(let i=0;i<rows.length;i++) {
        const source=base.rows[i];let r=rows[i];
        // Supports are immutable within a layout. Keep only fields used by
        // this identity correction; never copy physical force/Hessian data.
        if(!r||r.sourceDofs!==source.dofs||r.kind!==source.kind||r.id!==source.id) {
            const dofs=cache.inverse?source.dofs.map(p=>{
                if(cache.inverse[p]<0)throw new RangeError('Projection constraint unexpectedly depends on spin');
                return cache.inverse[p];
            }):source.dofs;
            r=rows[i]={kind:source.kind,id:source.id,edge:source.edge,sample:source.sample,
                dofs,sourceDofs:source.dofs,multiplier:0,penalty:1};
        }
        // Current geometry and gaps are mandatory even at unchanged support.
        r.jacobian=source.jacobian;r.gap=source.gap;
    }
    if(cache.originalDofs)cache.originalDofs.forEach((p,i)=>{cache.fixed[i]=s.fixed[p];});
    const result=yield* iterateSharedAxisLinear(cache.mixed,cache.chain,
        {fixed:cache.fixed,gradient:cache.gradient,rows,tolerance:1e-10,reuseStructure,reuseConstraintWork,reuseMatrixAssembly,identityProjection:mode,basisWorkspaceKey:s.fixed,maxActiveSetAttempts});
    if(cache.originalDofs)cache.originalDofs.forEach((p,i)=>{cache.increment[p]=result.increment?.[i]??0;});
    return {...result,increment:cache.increment??result.increment,zeroReactions:cache.zeroReactions};
}

import { createCoulombBandLU } from './kirchhoffCoulombBandLU.js';

const finite=(x,name)=>{if(!Number.isFinite(x))throw new RangeError(`${name} must be finite`);return x;};
const positive=(x,name)=>{if(!(finite(x,name)>0))throw new RangeError(`${name} must be positive`);return x;};

/** Interleave local length/wall reactions with the ONE chain's primal DOFs.
 * No global response columns, dense mobility matrix or contact Schur matrix.
 * Constraint rows have a fixed edge support; their values/active branch change.
 * The general band LU retains pivot fill and accepts indefinite tangents.
 */
export function createCompositeMixedWorkspace(layout,definitions) {
    const primal=new Int32Array(layout.dofCount),dual=new Int32Array(definitions.length),rows=definitions.map(r=>({...r,dofs:[...r.dofs]}));
    const isPosition=new Uint8Array(layout.dofCount);
    for(const first of layout.positions)for(let axis=0;axis<3;axis++)isPosition[first+axis]=1;
    const byEdge=Array.from({length:layout.nodeCount-1},()=>[]);
    rows.forEach((row,i)=>{
        if(!['length','wall'].includes(row.kind)||!Number.isInteger(row.edge)||row.edge<0||row.edge>=byEdge.length)
            throw new RangeError('A mixed row needs a length/wall kind and a valid edge');
        const legal=[layout.positions[row.edge],layout.positions[row.edge+1]].flatMap(s=>[s,s+1,s+2]);
        if(row.dofs.length!==6||row.dofs.some((d,j)=>d!==legal[j]))throw new RangeError('Mixed constraints must use the six ordered edge positions');
        byEdge[row.edge].push(i);
    });
    let count=0;
    for(let node=0;node<layout.nodeCount;node++) {
        const start=layout.positions[node],end=node+1<layout.nodeCount?layout.positions[node+1]:layout.dofCount;
        for(let dof=start;dof<end;dof++)primal[dof]=count++;
        if(node<byEdge.length)for(const row of byEdge[node])dual[row]=count++;
    }
    let bandwidth=0;
    for(let i=0;i<layout.dofCount;i++)for(let j=Math.max(0,i-layout.band+1);j<=i;j++)
        bandwidth=Math.max(bandwidth,primal[i]-primal[j]);
    rows.forEach((row,i)=>row.dofs.forEach(d=>{bandwidth=Math.max(bandwidth,Math.abs(primal[d]-dual[i]));}));
    const starts=new Int32Array(count),ends=new Int32Array(count),offsets=new Int32Array(count);
    let entries=0;
    for(let i=0;i<count;i++){starts[i]=Math.max(0,i-bandwidth);ends[i]=Math.min(count-1,i+bandwidth);offsets[i]=entries-starts[i];entries+=ends[i]-starts[i]+1;}
    const packedLayout={starts,ends,offsets,entries,kl:bandwidth,ku:bandwidth};
    return {layout,rows,primal,dual,count,isPosition,packedLayout,lu:createCoulombBandLU(packedLayout,count),
        matrix:new Float64Array(entries),residual:new Float64Array(count),scales:new Float64Array(count),
        increment:new Float64Array(count),correction:new Float64Array(count),linearResidual:new Float64Array(count),
        primalIncrement:new Float64Array(layout.dofCount),multiplierIncrement:new Float64Array(rows.length)};
}

/** One unshifted mixed Newton direction. chain must hold elastic/inertial
 * tangent, and gradient is the ORIGINAL physical stationarity residual with
 * J_length^T lambda - J_wall^T normalForce, not an augmented gradient.
 *
 * Length rows: F=g, dF=J dq. Wall: F=(max(0,lambda-penalty*g)-lambda)/penalty.
 * A wall's penalty only chooses/scales its exact NCP equation, never softens g.
 * At contact its Jacobian is -J; at an open branch it is -dLambda/penalty.
 * An optional signed wall forceColumn is the mechanical derivative per unit
 * PHYSICAL normal force. It is -B for a distributed unit normal, whereas J
 * differentiates gap. They differ for a normalized sparse-SDF gradient.
 * Without an explicit column, the true-distance contract defaults to -J.
 * A supplied geometricHessian may include the nonsymmetric -Fn*DB term.
 * An explicitly defined SDF cone branch may set allowSignedWallIterate:true
 * for its PRIVATE NCP unknown. The NCP equations and linear proof are unchanged;
 * physical nonnegativity remains mandatory in the original final certificate.
 * Ordinary wall definitions retain the nonnegative-iterate contract.
 * All original linear equations are measured in explicit force/torque/length
 * units after LU. Optional corrections refactor the SAME unshifted matrix;
 * counts report every factor/solve. This direction does not accept a dt.
 */
export function solveCompositeMixedDirection(w,chain,{rows,gradient,fixed,tolerances,maxCorrections=1}) {
    if(chain.hessianValid===false)throw new RangeError('A fresh full Hessian assembly is required before a mixed direction');
    const {layout,primal,dual,count,matrix,residual,scales,increment,correction,linearResidual,packedLayout:p}=w;
    if(chain.layout!==layout||rows.length!==w.rows.length||gradient.length!==layout.dofCount||fixed.length!==layout.dofCount)
        throw new RangeError('Mixed inputs must match the frozen layout');
    if(!Number.isInteger(maxCorrections)||maxCorrections<0||maxCorrections>2)throw new RangeError('maxCorrections must be 0..2');
    const tol={force:positive(tolerances?.force,'linear force tolerance'),torque:positive(tolerances?.torque,'linear torque tolerance'),
        constraint:positive(tolerances?.constraint,'linear constraint tolerance')};
    if(!gradient.every(Number.isFinite)||!chain.hessian.every(Number.isFinite))throw new RangeError('Finite original operator required');
    matrix.fill(0);residual.fill(0);increment.fill(0);
    const at=(i,j)=>p.offsets[i]+j;
    const add=(i,j,v)=>{if(v===0)return;if(j<p.starts[i]||j>p.ends[i])throw new RangeError('Mixed stencil exceeds local band');matrix[at(i,j)]+=v;};
    for(let i=0;i<layout.dofCount;i++) {
        const a=primal[i];residual[a]=fixed[i]?0:gradient[i];
        if(fixed[i]){add(a,a,1);continue;}
        for(let j=Math.max(0,i-layout.band+1);j<=i;j++)if(!fixed[j]) {
            const b=primal[j],v=chain.hessian[i*layout.band+i-j];add(a,b,v);if(i!==j)add(b,a,v);
        }
    }
    let fixedConstraintResidual=0;
    rows.forEach((r,index)=>{
        const definition=w.rows[index],d=dual[index],J=r.jacobian;
        if(r.kind!==definition.kind||r.edge!==definition.edge||!J||J.length!==6||!J.every(Number.isFinite))throw new RangeError('Row values must match the frozen local support');
        const gap=finite(r.gap,'constraint gap'),lambda=finite(r.multiplier,'physical multiplier');
        const isWall=r.kind==='wall',penalty=isWall?positive(r.penalty,'wall NCP scale'):null;
        const forceColumn=r.forceColumn;
        if(forceColumn!==undefined&&(!isWall||forceColumn.length!==6||!forceColumn.every(Number.isFinite)))
            throw new RangeError('An explicit wall force column needs six finite signed coefficients');
        const signedSdfTrial=r.allowSignedWallIterate===true&&isWall&&(definition.sdfBranch===0||definition.sdfBranch===1);
        if(r.allowSignedWallIterate===true&&!signedSdfTrial)throw new RangeError('Signed wall iterates require an explicit SDF cone definition');
        if(isWall&&lambda<0&&!signedSdfTrial)throw new RangeError('Physical wall reaction must be nonnegative');
        const active=!isWall||lambda-penalty*gap>0,sign=isWall?-1:1;
        const noMotion=definition.dofs.every((dof,i)=>fixed[dof]||J[i]===0);
        // An exactly satisfied constraint on entirely prescribed positions
        // carries an undetermined reaction. Keep that physical reaction fixed.
        // Incompatible prescribed geometry remains a reported failure.
        if(noMotion && (!isWall || active)) {
            fixedConstraintResidual=Math.max(fixedConstraintResidual,Math.abs(gap));add(d,d,1);return;
        }
        residual[d]=isWall?(Math.max(0,lambda-penalty*gap)-lambda)/penalty:gap;
        if(!active)add(d,d,-1/penalty);
        definition.dofs.forEach((dof,i)=>{if(fixed[dof])return;
            add(primal[dof],d,forceColumn===undefined?sign*J[i]:forceColumn[i]);if(active)add(d,primal[dof],sign*J[i]);
        });
        // The exact stress tangent of a length multiplier can be indefinite
        // under compression. Preserve it when the caller supplies it.
        if(r.geometricHessian!==undefined) {
            if(r.geometricHessian.length!==36||!r.geometricHessian.every(Number.isFinite))throw new RangeError('Finite 6x6 stress tangent required');
            definition.dofs.forEach((a,i)=>{if(!fixed[a])definition.dofs.forEach((b,j)=>{if(!fixed[b])add(primal[a],primal[b],r.geometricHessian[6*i+j]);});});
        }
    });
    if(!matrix.every(Number.isFinite)||!residual.every(Number.isFinite))throw new RangeError('Nonfinite original mixed equations');
    for(let i=0;i<layout.dofCount;i++) {
        const row=primal[i];let magnitude=Math.abs(matrix[at(row,row)]);
        if(magnitude===0)for(let j=p.starts[row];j<=p.ends[row];j++)magnitude=Math.max(magnitude,Math.abs(matrix[at(row,j)]));
        scales[row]=magnitude>0?1/Math.sqrt(magnitude):1;
    }
    w.rows.forEach((row,i)=>{
        let magnitude=0;for(const dof of row.dofs)if(!fixed[dof])magnitude=Math.max(magnitude,Math.abs(matrix[at(primal[dof],dual[i])]*scales[primal[dof]]));
        scales[dual[i]]=magnitude>0?1/magnitude:1;
    });
    const measure=()=>{
        for(let i=0;i<count;i++){
            let sum=residual[i],compensation=0;
            for(let j=p.starts[i];j<=p.ends[i];j++){
                const value=matrix[at(i,j)]*increment[j],next=sum+value;
                compensation+=Math.abs(sum)>=Math.abs(value)?sum-next+value:value-next+sum;sum=next;
            }
            linearResidual[i]=sum+compensation;
        }
        let force=0,torque=0,constraint=fixedConstraintResidual;
        for(let i=0;i<layout.dofCount;i++)if(!fixed[i]){
            const value=Math.abs(linearResidual[primal[i]]);
            if(w.isPosition[i])force=Math.max(force,value);else torque=Math.max(torque,value);
        }
        for(const row of dual)constraint=Math.max(constraint,Math.abs(linearResidual[row]));
        return {force,torque,constraint};
    };
    let factorizations=0,linearSolves=0,proof=null,solverAccepted=true;
    for(let iteration=0;iteration<=maxCorrections;iteration++){
        factorizations++;linearSolves++;
        solverAccepted=w.lu.solve(matrix,iteration===0?residual:linearResidual,scales,0,correction);
        if(!solverAccepted)break;
        for(let i=0;i<count;i++)increment[i]+=scales[i]*correction[i];
        proof=measure();
        if(proof.force<=tol.force&&proof.torque<=tol.torque&&proof.constraint<=tol.constraint)break;
    }
    w.primalIncrement.forEach((_,i)=>{w.primalIncrement[i]=fixed[i]?0:increment[primal[i]];});
    w.multiplierIncrement.forEach((_,i)=>{w.multiplierIncrement[i]=increment[dual[i]];});
    return {increment:w.primalIncrement,multiplierIncrement:w.multiplierIncrement,linearResidual,proof,factorizations,linearSolves,
        converged:solverAccepted&&proof!==null&&linearResidual.every(Number.isFinite)&&
            proof.force<=tol.force&&proof.torque<=tol.torque&&proof.constraint<=tol.constraint,
        count,bandwidth:p.kl,matrixEntries:matrix.length,factorEntries:w.lu.diagnostics.factorEntries,
        scope:'one-chain-original-mixed-linear-direction'};
}

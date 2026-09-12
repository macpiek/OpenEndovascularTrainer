import {createContactResult} from './collision/vesselContactField.js';
import {initializeCompositeWallDifferentialRow,createCompositeWallDifferentialWorkspace,captureCompositeWallDifferentialContact,
    differentiateCompositeWallRow,requireCompositeWallDifferentialRows,equalCompositeWallGlobalDifferentials} from './kirchhoffCompositeWallDifferentialRows.js';

const finite=(value,name)=>{if(!Number.isFinite(value))throw new TypeError(`${name} must be finite`);return value;};
const positive=(value,name)=>{if(!(finite(value,name)>0))throw new RangeError(`${name} must be positive`);return value;};

/** One capsule query per existing mesh edge and its OUTERMOST material.
 * Reuses the application's VesselContactField, including its sampling and
 * refinement policy. This is a discrete contact collector, not a new CCD or
 * geometric certificate. Sheath and internal lumen contacts are separate.
 */
export function createCompositeWallWorkspace(layout) {
    return {layout,scratch:createContactResult(),queries:0,differentialScratch:createCompositeWallDifferentialWorkspace(),
        rows:Array.from({length:layout.nodeCount-1},(_,edge)=>initializeCompositeWallDifferentialRow({edge,index:edge,included:false,owner:null,
            gap:Infinity,t:0,normal:new Float64Array(3),closestPoint:new Float64Array(3),
            faceIndex:-1,branchId:-1,source:null,sampleCount:0},layout)),
        physicalGradient:new Float64Array(layout.dofCount),trialNormalForces:new Float64Array(layout.nodeCount-1)};
}

export function refreshCompositeWallContacts({positions,contactOwners,field,lambdas},workspace) {
    if(positions.length!==workspace.layout.nodeCount||contactOwners.edges.length!==workspace.rows.length)
        throw new RangeError('Wall geometry/ownership must match the frozen mesh');
    if(typeof field?.queryCapsuleCoordinates!=='function') throw new TypeError('The vessel capsule-query interface is required');
    for(const p of positions) if(p.length!==3||!p.every(Number.isFinite)) throw new TypeError('Finite centerline positions are required');
    workspace.queries=0;
    for(const row of workspace.rows) {
        const edgeOwner=contactOwners.edges[row.edge],wall=edgeOwner.wall;
        if(edgeOwner.edge!==row.edge) throw new RangeError('Contact ownership must follow mesh edges');
        row.included=wall!==null&&wall!==undefined;row.owner=wall?.owner??null;
        if(!row.included) {row.gap=Infinity;differentiateCompositeWallRow({field,positions,row},workspace.differentialScratch);continue;}
        if(!workspace.layout.edgeToolIds[row.edge].includes(wall.owner)) throw new RangeError('Wall owner must occupy this edge');
        const radius=positive(wall.radius,'outer material radius'),a=positions[row.edge],b=positions[row.edge+1];
        const result=field.queryCapsuleCoordinates(...a,...b,radius,workspace.scratch);
        workspace.queries++;
        row.gap=finite(result.signedGap,'wall gap');row.t=finite(result.segmentT,'capsule fraction');
        if(row.t<0||row.t>1) throw new RangeError('A capsule fraction must lie on its edge');
        const normal=result.inward.values,closest=result.closestPoint.values;
        if(!normal.every(Number.isFinite)||!closest.every(Number.isFinite)||Math.abs(Math.hypot(...normal)-1)>1e-8)
            throw new RangeError('A finite unit inward normal and closest surface point are required');
        row.normal.set(normal);row.closestPoint.set(closest);row.faceIndex=result.faceIndex;
        row.branchId=result.branchId;row.source=result.source;row.sampleCount=result.capsuleSampleCount;
        captureCompositeWallDifferentialContact(row,result,radius);
        differentiateCompositeWallRow({field,positions,row},workspace.differentialScratch);
    }
    if(lambdas!==undefined)requireCompositeWallDifferentialRows(workspace,lambdas);
    return workspace;
}

function inputs(w,lambdas,penalty) {
    if(lambdas?.length!==w.rows.length || typeof penalty!=='number'&&penalty?.length!==w.rows.length)
        throw new RangeError('Multiplier and penalty data must match wall rows');
    for(const row of w.rows) {
        if(finite(lambdas[row.index],'normal multiplier')<0) throw new RangeError('Normal multipliers must be nonnegative');
        positive(typeof penalty==='number'?penalty:penalty[row.index],'wall penalty');
        if(!row.included && lambdas[row.index]!==0) throw new RangeError('Removed wall ownership requires an explicit traction transfer');
    }
}

/** PHR augmented energy for g>=0, lambda>=0:
 * p=max(0,lambda-mu*g), E=(p²-lambda²)/(2mu), gradient=-p*Jg.
 * The added tangent is the stated PSD GN mu*Jg^T*Jg, not the exact
 * geometry Hessian. p is a TRIAL reaction, not an accepted friction budget.
 * Physical-normal AL is supported here for analytic-plane (G=B) only;
 * sparse-SDF physical normals do not define this scalar potential.
 * Call after elastic/inertial assembly; refresh geometry on each trial.
 */
export function assembleCompositeWallAugmented(workspace,chain,{lambdas,penalty}) {
    inputs(workspace,lambdas,penalty);
    if(chain.layout!==workspace.layout) throw new RangeError('Wall and chain must share one layout');
    if(workspace.rows.some(row=>row.included&&row.source==='sparse-sdf'))
        throw new RangeError('Physical-normal AL has an inconsistent potential for sparse-SDF; use physical mixed rows or explicit gap-potential mode');
    if(workspace.rows.some(row=>row.included&&row.source==='sparse-sdf-bvh'))
        throw new RangeError('Physical-normal AL is enabled for analytic-plane only; BVH derivatives enable physical mixed rows, not implicit BVH AL');
    requireCompositeWallDifferentialRows(workspace,lambdas);
    workspace.trialNormalForces.fill(0);let energy=0;
    for(const row of workspace.rows) if(row.included) {
        const mu=typeof penalty==='number'?penalty:penalty[row.index],lambda=lambdas[row.index];
        const p=Math.max(0,lambda-mu*row.gap),delta=p-lambda;
        // Factored difference avoids squaring two large nearly equal forces.
        energy+=delta*(p/2+lambda/2)/mu;workspace.trialNormalForces[row.index]=p;
        if(p===0)continue;
        const dofs=row.dofs,J=row.gapJacobian;
        for(let i=0;i<6;i++) {
            chain.gradient[dofs[i]]-=p*J[i];
            for(let j=0;j<=i;j++) {
                const hi=Math.max(dofs[i],dofs[j]),lo=Math.min(dofs[i],dofs[j]),value=mu*J[i]*J[j];
                if(value!==0&&hi-lo>=chain.layout.band) throw new RangeError('Wall stencil exceeds the local chain band');
                if(value!==0)chain.hessian[hi*chain.layout.band+hi-lo]+=value;
            }
        }
    }
    chain.energy+=energy;
    if(!Number.isFinite(chain.energy)||!chain.gradient.every(Number.isFinite)||!chain.hessian.every(Number.isFinite)||
        !workspace.trialNormalForces.every(Number.isFinite))throw new RangeError('Nonfinite composite wall operator');
    return {energy,trialNormalForces:workspace.trialNormalForces,scope:'discrete-wall-augmented-operator'};
}

/** Evaluate the ORIGINAL unilateral conditions and physical -Fn*B, where B
 * distributes the unit normal. B is generally NOT the true gap gradient G.
 * Force residual compares lambda with its projected update; complementarity
 * is checked in force*length units with a separate caller-supplied tolerance.
 * Caller must additionally check the entire mechanical force residual and
 * the continuous/swept geometry. This does not accept a timestep by itself.
 */
export function measureCompositeWallConstraints(workspace,{lambdas,penalty,gapTolerance,forceTolerance,workTolerance}) {
    inputs(workspace,lambdas,penalty);
    positive(gapTolerance,'gap tolerance');positive(forceTolerance,'force tolerance');positive(workTolerance,'work tolerance');
    requireCompositeWallDifferentialRows(workspace,lambdas);
    const gradient=workspace.physicalGradient;gradient.fill(0);
    let maximumPenetration=0,maximumProjectedResidual=0,maximumComplementarity=0;
    for(const row of workspace.rows) if(row.included) {
        const mu=typeof penalty==='number'?penalty:penalty[row.index],lambda=lambdas[row.index];
        maximumPenetration=Math.max(maximumPenetration,-row.gap);
        maximumProjectedResidual=Math.max(maximumProjectedResidual,Math.abs(Math.max(0,lambda-mu*row.gap)-lambda));
        maximumComplementarity=Math.max(maximumComplementarity,Math.abs(lambda*row.gap));
        if(lambda===0)continue;
        const dofs=row.dofs;
        for(let i=0;i<6;i++)gradient[dofs[i]]+=lambda*row.forceColumn[i];
    }
    if(!gradient.every(Number.isFinite)||![maximumPenetration,maximumProjectedResidual,maximumComplementarity].every(Number.isFinite))
        throw new RangeError('Nonfinite physical wall residual');
    return {physicalGradient:gradient,maximumPenetration,maximumProjectedResidual,maximumComplementarity,
        converged:maximumPenetration<=gapTolerance&&maximumProjectedResidual<=forceTolerance&&maximumComplementarity<=workTolerance,
        scope:'discrete-wall-KKT-only'};
}

/** Exact duplicate endpoint constraints share ONE physical reaction. Adjacent
 * capsules can select the same mesh vertex, with the same outer radius, gap,
 * normal and surface witness. Their source and original g/G/B/DB must also
 * match exactly after global scatter; equal normals alone are insufficient.
 * Sum the physical force on the first row and zero the redundant copies;
 * force and moment on the common chain are unchanged. Interior feet, distinct
 * normals/owners/radii or merely nearby contacts are NEVER merged. This is a
 * transaction-local multiplier reparameterization, not geometric snapping or
 * approximate rank truncation. Returns the representative edge indices.
 */
export function canonicalizeCompositeWallReactions(workspace,lambdas,contactOwners) {
    if(lambdas.length!==workspace.rows.length)throw new RangeError('Wall reactions must match the workspace');
    requireCompositeWallDifferentialRows(workspace,lambdas);
    const endpoints=new Map(),representatives=[];
    for(const row of workspace.rows)if(row.included) {
        if(!Number.isFinite(lambdas[row.index])||lambdas[row.index]<0)throw new RangeError('Finite nonnegative physical reactions required');
        if(row.derivativeUnavailable||row.t!==0&&row.t!==1){representatives.push(row.index);continue;}
        const node=row.edge+row.t,candidates=endpoints.get(node)??[];
        const previous=candidates.find(candidate=>candidate.owner===row.owner&&
            contactOwners.edges[candidate.edge].wall.radius===contactOwners.edges[row.edge].wall.radius&&
            candidate.normal.every((v,i)=>v===row.normal[i])&&candidate.closestPoint.every((v,i)=>v===row.closestPoint[i])&&
            equalCompositeWallGlobalDifferentials(candidate,row,workspace.layout));
        if(previous===undefined){candidates.push(row);endpoints.set(node,candidates);representatives.push(row.index);}
        else {
            const sum=lambdas[previous.index]+lambdas[row.index];
            if(!Number.isFinite(sum))throw new RangeError('Combined physical wall reaction overflowed');
            lambdas[previous.index]=sum;lambdas[row.index]=0;
        }
    }
    return representatives;
}

import {prepareCompositeZeroDualSolve} from './kirchhoffCompositeZeroDuals.js';
import {createCompositeSparseDirectionOperator} from './kirchhoffCompositeSparseDirection.js';

const finite=(v,name)=>{if(!Number.isFinite(v))throw new RangeError(`${name} must be finite`);return v;};
const positive=(v,name)=>{if(!(finite(v,name)>0))throw new RangeError(`${name} must be positive`);return v;};
const vector=(v,n,name)=>{
    if(!v||v.length!==n)throw new RangeError(`${name} requires ${n} finite entries`);
    for(let i=0;i<n;i++)if(!Number.isFinite(v[i]))throw new RangeError(`${name} requires ${n} finite entries`);
};
const same=(a,b)=>a?.length===b.length&&b.every((v,i)=>v===a[i]);
const indexList=(v,count,name)=>{
    if(!(v instanceof Int32Array)||v.some(d=>d<0||d>=count)||new Set(v).size!==v.length)
        throw new RangeError(`${name} must be unique in-range Int32Array indices`);
    return v.slice();
};

function validateClusterStructure(w,c,requireOperator=true) {
    const s=w.structure;
    if(!c||(requireOperator&&(c.hessianValid===false||c.operatorReady===false))||c.commonContributionRole!=='diagnostic-only-already-in-common'||
        c.relative?.dofCount!==s.relativeCount||c.relative.band!==s.relativeBand||
        c.toolId!==s.toolId||c.hessianKind!==s.hessianKind||
        !same(c.coupling?.commonDofs,s.commonDofs)||!same(c.coupling?.rowOffsets,s.rowOffsets)||
        !same(c.coupling?.columns,s.columns)||c.modes?.length!==s.modeNodes.length)
        throw new RangeError('A fresh cluster with the frozen structure and Hessian kind is required');
    c.modes.forEach((m,i)=>{
        const count=s.modeOffsets[i+1]-s.modeOffsets[i];
        if(m.node!==s.modeNodes[i]||m.basis?.length!==count||m.relativeDofs?.length!==count||
            m.relativeDofs.some((d,j)=>d!==s.modeOffsets[i]+j)||m.basis.some((b,j)=>!same(b,s.bases[i][j])))
            throw new RangeError('Cluster modes and frozen transverse bases changed; rebuild the workspace');
    });
    if(!requireOperator)return;
    vector(c.relative.hessian,s.relativeCount*s.relativeBand,'relative Hessian');
    vector(c.coupling.values,s.columns.length,'common-relative tangent');
    for(const slot of w.remoteRelativeSlots)if(c.relative.hessian[slot]!==0)
        throw new RangeError('Relative coupling exceeds the local two-edge stencil');
}

/** Structure-only workspace for ONE original common/rho/local-dual system.
 * Original equations use a complete symbolic CSR stencil. A numerical band
 * and its factor are prepared only for the unknowns that remain in the solve.
 * originalMatrix/matrix expose on-demand full-band diagnostic views; the
 * compressed path never materializes them. Reading lu explicitly allocates
 * the uncompressed factor for callers that need that path.
 * Recreate if topology, basis/dimension, Hessian kind or row supports change.
 * Every solve MUST supply the current cluster values; no H/C are cached here.
 * An unassembled createCompositeRelativeClusterStructure result is sufficient
 * to create this workspace, but solve requires an assembled valid operator.
 * The explicit single-material empty cluster contributes no rho unknowns or
 * support. Common equations and local duals use the identical band solve.
 *
 * Row definition (owned copies):
 * {anchorNode, commonDofs:Int32Array, relativeDofs:Int32Array, unit:string,
 *  multiplierDofs?:Int32Array}. Optional multiplierDofs names OTHER row
 * indices in one local contact block, with the same anchor and a combined
 * two-edge support. It retains the normal/tangential and tangential/tangential
 * derivatives of that block; no normal load is frozen by this solver.
 * Local ordered support is commonDofs followed by relativeDofs. Indices are
 * global Chain DOFs and Cluster rho DOFs, respectively. All support nodes and
 * anchor must lie within two edges, the actual wire-hinge stencil. There may
 * be several rows per node; storage stays O(N) for a bounded local row count.
 * Diagonal and declared off-diagonal dual derivatives are supplied at solve
 * time. The constructor never selects contact branches or disk normals.
 */
export function createCompositeRelativeDirectionWorkspace(layout,cluster,rowDefinitions=[]) {
    if(!layout||!Number.isInteger(layout.nodeCount)||layout.nodeCount<3||layout.positions?.length!==layout.nodeCount||
        !Number.isInteger(layout.dofCount)||!Number.isInteger(layout.band)||layout.band<1)
        throw new RangeError('A valid common Chain layout is required');
    const n=layout.dofCount,r=cluster?.relative?.dofCount;
    const empty=r===0&&layout.spins?.size===1&&layout.spins.has(cluster.toolId)&&cluster.modes?.length===0&&
        cluster.relativeRepresentation==='none-single-material';
    if(!Number.isInteger(r)||(!empty&&r<2)||!Array.isArray(cluster.modes)||(!empty&&!cluster.modes.length)||
        !Number.isInteger(cluster.relative.band)||cluster.relative.band<1)
        throw new RangeError('A joint relative Cluster or explicit empty single-material Cluster is required');
    const commonNode=new Int32Array(n),positionMask=new Uint8Array(n),relativeNode=new Int32Array(r);
    for(let node=0;node<layout.nodeCount;node++) {
        const start=layout.positions[node],end=node+1<layout.nodeCount?layout.positions[node+1]:n;
        if(!Number.isInteger(start)||start<0||end-start<3||(node===0&&start!==0)||end>n)
            throw new RangeError('Common DOFs must follow the Chain node ordering');
        for(let d=start;d<end;d++)commonNode[d]=node;
        positionMask.fill(1,start,start+3);
    }
    const modeNodes=Int32Array.from(cluster.modes,m=>m.node),modeOffsets=new Int32Array(cluster.modes.length+1);
    let relativeOffset=0;
    const bases=cluster.modes.map((m,i)=>{
        const dimension=m.basis?.length;
        if(!Number.isInteger(m.node)||m.node<0||m.node>=layout.nodeCount||(i&&m.node<=modeNodes[i-1])||
            (dimension===2&&(m.node===0||m.node===layout.nodeCount-1))||
            ![2,3].includes(dimension)||m.relativeDofs?.length!==dimension||m.relativeDofs.some((d,j)=>d!==relativeOffset+j))
            throw new RangeError('Cluster mode nodes must be ordered and in range; reduced two-coordinate modes remain interior');
        modeOffsets[i]=relativeOffset;
        for(let axis=0;axis<dimension;axis++)relativeNode[relativeOffset++]=m.node;
        m.basis.forEach(b=>vector(b,3,'frozen basis'));
        return m.basis.map(b=>Float64Array.from(b));
    });
    modeOffsets[cluster.modes.length]=relativeOffset;
    if(relativeOffset!==r)throw new RangeError('Relative DOF count must equal the sum of mode dimensions');
    const widerPositionPairs=new Set();
    for(const nodes of layout.positionSupports??[]) {
        if(!Array.isArray(nodes)||nodes.some((node,j)=>!Number.isInteger(node)||node<0||node>=layout.nodeCount||(j&&node<=nodes[j-1])))
            throw new RangeError('Declared position support must have ordered physical nodes');
        for(const a of nodes)for(const b of nodes)widerPositionPairs.add(a*layout.nodeCount+b);
    }
    const materialCouplingPairs=new Set();
    for(const support of layout.materialSupports??[])if(support.toolId===cluster.toolId) {
        const dofs=support.positionNodes.flatMap(node=>[layout.positions[node],layout.positions[node]+1,layout.positions[node]+2])
            .concat(support.angleEdges.map(edge=>layout.spins.get(support.toolId)[edge]));
        for(const a of support.positionNodes)for(const b of support.positionNodes)widerPositionPairs.add(a*layout.nodeCount+b);
        for(const d of dofs)for(const node of support.positionNodes)materialCouplingPairs.add(d*layout.nodeCount+node);
    }
    const positionPairAllowed=(a,b)=>Math.abs(a-b)<=2||widerPositionPairs.has(a*layout.nodeCount+b);
    const commonDofs=indexList(cluster.coupling?.commonDofs,n,'Cluster common support'),
        rowOffsets=cluster.coupling?.rowOffsets,columns=cluster.coupling?.columns;
    if(empty&&(commonDofs.length!==0||cluster.common?.dofs?.length!==0))
        throw new RangeError('An empty single-material Cluster must have zero common support');
    if(!(rowOffsets instanceof Int32Array)||rowOffsets.length!==commonDofs.length+1||rowOffsets[0]!==0||
        !(columns instanceof Int32Array)||rowOffsets.at(-1)!==columns.length||
        rowOffsets.some((v,i)=>v<0||v>columns.length||(i&&v<rowOffsets[i-1]))||columns.some(v=>v<0||v>=r))
        throw new RangeError('Valid local Cluster CSR coupling is required');
    for(let row=0;row<commonDofs.length;row++) {
        const seen=new Set();
        for(let i=rowOffsets[row];i<rowOffsets[row+1];i++) {
            const a=commonNode[commonDofs[row]],b=relativeNode[columns[i]];
            if(seen.has(columns[i])||(Math.abs(a-b)>2&&!materialCouplingPairs.has(commonDofs[row]*layout.nodeCount+b)&&(!positionMask[commonDofs[row]]||!positionPairAllowed(a,b))))
                throw new RangeError('Cluster coupling must have unique local two-edge or declared position support');
            seen.add(columns[i]);
        }
    }
    const rows=rowDefinitions.map((def,index)=>{
        const commonDofs=indexList(def.commonDofs,n,'Row common support'),relativeDofs=indexList(def.relativeDofs,r,'Row relative support');
        const multiplierDofs=indexList(def.multiplierDofs??new Int32Array(),rowDefinitions.length,'Row multiplier support');
        if(multiplierDofs.includes(index))throw new RangeError('Use multiplierDerivative for the diagonal of a local dual block');
        if(!Number.isInteger(def.anchorNode)||def.anchorNode<0||def.anchorNode>=layout.nodeCount||typeof def.unit!=='string'||!def.unit.trim()||
            commonDofs.length+relativeDofs.length===0)throw new RangeError('A local row needs anchorNode, nonempty support and an explicit unit');
        const nodes=[def.anchorNode,...Array.from(commonDofs,d=>commonNode[d]),...Array.from(relativeDofs,d=>relativeNode[d])];
        let constraintSupport=null;
        if(def.constraintSupport?.kind==='native-tool-pair') {
            const s=def.constraintSupport,tools=s.tools;
            if(typeof s.pairId!=='string'||!s.pairId||!Array.isArray(tools)||tools.length!==2||tools[0].id===tools[1].id||typeof s.spins!=='boolean'||
                tools.some(t=>!Number.isInteger(t.edge)||!layout.edgeToolIds[t.edge]?.includes(t.id))||def.unit!=='mm')throw new RangeError('Native contact support needs exactly its two original material edges');
            const expectedCommon=[...new Set(tools.flatMap(t=>[t.edge,t.edge+1].flatMap(node=>[layout.positions[node],layout.positions[node]+1,layout.positions[node]+2]).concat(s.spins?[layout.spins.get(t.id)[t.edge]]:[])))].sort((a,b)=>a-b),
                relativeNodes=tools.filter(t=>t.id===cluster.toolId).flatMap(t=>[t.edge,t.edge+1]),
                expectedRelative=cluster.modes.filter(m=>relativeNodes.includes(m.node)).flatMap(m=>m.relativeDofs);
            if(def.anchorNode!==Math.min(...tools.map(t=>t.edge))||!same(commonDofs,expectedCommon)||!same(relativeDofs,expectedRelative))throw new RangeError('Native pair rows must retain their exact physical force and own-spin support');
            constraintSupport={kind:s.kind,pairId:s.pairId,tools:tools.map(t=>({id:t.id,edge:t.edge})),spins:s.spins};
        } else if(def.constraintSupport?.kind==='continuous-wall-friction') {
            const s=def.constraintSupport,material=(layout.materialSupports??[]).find(m=>m.toolId===s.toolId&&m.edge===s.edge),
                expectedCommon=material?[...material.positionNodes.flatMap(node=>[layout.positions[node],layout.positions[node]+1,layout.positions[node]+2]),...material.angleEdges.map(edge=>layout.spins.get(s.toolId)[edge])].sort((a,b)=>a-b):[],
                expectedRelative=s.toolId===cluster.toolId?cluster.modes.filter(m=>material?.positionNodes.includes(m.node)).flatMap(m=>m.relativeDofs):[];
            if(!material||def.toolId!==s.toolId||def.edge!==s.edge||def.anchorNode!==s.edge||def.unit!=='mm'||!same(s.positionNodes,material.positionNodes)||!same(s.angleEdges,material.angleEdges)||
                !same(commonDofs,expectedCommon)||!same(relativeDofs,expectedRelative))throw new RangeError('Continuous friction rows need the exact declared own material frame support');
            constraintSupport={kind:s.kind,toolId:s.toolId,edge:s.edge,positionNodes:s.positionNodes.slice(),angleEdges:s.angleEdges.slice()};
        } else if(def.constraintSupport!==undefined) {
            const s=def.constraintSupport,p=s?.nodeIndices,
                declared=Array.isArray(p)&&(layout.positionSupports??[]).some(v=>same(v,p)),
                expectedCommon=declared?p.flatMap(node=>[layout.positions[node],layout.positions[node]+1,layout.positions[node]+2]):[],
                expectedRelative=s?.toolId===cluster.toolId?cluster.modes.filter(m=>p?.includes(m.node)).flatMap(m=>m.relativeDofs):[];
            if(!['continuous-arclength','continuous-taut-arclength','continuous-wall-point'].includes(s?.kind)||def.toolId!==s.toolId||def.edge!==s.edge||def.anchorNode!==s.edge||def.unit!=='mm'||
                !layout.edgeToolIds[s.edge]?.includes(s.toolId)||!declared||!same(commonDofs,expectedCommon)||!same(relativeDofs,expectedRelative)||multiplierDofs.length)
                throw new RangeError('Continuous length rows require their exact declared physical position support');
            constraintSupport={kind:s.kind,toolId:s.toolId,edge:s.edge,nodeIndices:p.slice()};
        }
        if(Math.max(...nodes)-Math.min(...nodes)>2&&constraintSupport===null)throw new RangeError('Constraint support exceeds the local two-edge stencil');
        return {anchorNode:def.anchorNode,commonDofs,relativeDofs,multiplierDofs,unit:def.unit,nodes,constraintSupport};
    });
    rows.forEach(row=>row.multiplierDofs.forEach(index=>{
        const other=rows[index],nodes=[...row.nodes,...other.nodes];
        const s=row.constraintSupport,t=other.constraintSupport,nativePair=s?.kind==='native-tool-pair'&&t?.kind==='native-tool-pair'&&s.pairId===t.pairId&&JSON.stringify(s.tools)===JSON.stringify(t.tools),continuous=s?.kind==='continuous-wall-friction'&&t&&s.toolId===t.toolId&&s.edge===t.edge&&
            (t.kind==='continuous-wall-friction'&&same(s.positionNodes,t.positionNodes)&&same(s.angleEdges,t.angleEdges)||t.kind==='continuous-wall-point'&&t.nodeIndices.every(node=>s.positionNodes.includes(node)));
        if(other.anchorNode!==row.anchorNode||Math.max(...nodes)-Math.min(...nodes)>2&&!continuous&&!nativePair)
            throw new RangeError('Coupled multipliers must share one local contact anchor and declared physical support');
    }));
    const common=new Int32Array(n),relative=new Int32Array(r),dual=new Int32Array(rows.length),rowsByNode=Array.from({length:layout.nodeCount},()=>[]);
    rows.forEach((row,i)=>rowsByNode[row.anchorNode].push(i));
    let count=0,modeIndex=0;
    for(let node=0;node<layout.nodeCount;node++) {
        const end=node+1<layout.nodeCount?layout.positions[node+1]:n;
        for(let dof=layout.positions[node];dof<end;dof++)common[dof]=count++;
        if(modeNodes[modeIndex]===node){for(let dof=modeOffsets[modeIndex];dof<modeOffsets[modeIndex+1];dof++)relative[dof]=count++;modeIndex++;}
        for(const row of rowsByNode[node])dual[row]=count++;
    }
    let bandwidth=0;
    const visit=(a,b)=>{bandwidth=Math.max(bandwidth,Math.abs(a-b));};
    for(let i=0;i<n;i++)for(let j=Math.max(0,i-layout.band+1);j<=i;j++)visit(common[i],common[j]);
    const relativeScatter=[],remoteRelativeSlots=[],couplingScatter=[];
    for(let i=0;i<r;i++)for(let j=Math.max(0,i-cluster.relative.band+1);j<=i;j++) {
        const slot=i*cluster.relative.band+i-j;
        if(!positionPairAllowed(relativeNode[i],relativeNode[j])){remoteRelativeSlots.push(slot);continue;}
        relativeScatter.push(slot,relative[i],relative[j]);visit(relative[i],relative[j]);
    }
    for(let row=0;row<commonDofs.length;row++)for(let i=rowOffsets[row];i<rowOffsets[row+1];i++) {
        couplingScatter.push(i,common[commonDofs[row]],relative[columns[i]]);visit(common[commonDofs[row]],relative[columns[i]]);
    }
    rows.forEach((row,index)=>{
        row.mixedDofs=Int32Array.from([...Array.from(row.commonDofs,d=>common[d]),...Array.from(row.relativeDofs,d=>relative[d])]);
        for(const a of row.mixedDofs){visit(a,dual[index]);for(const b of row.mixedDofs)visit(a,b);}
        row.multiplierDofs.forEach(other=>visit(dual[index],dual[other]));
    });
    const starts=new Int32Array(count),ends=new Int32Array(count),offsets=new Int32Array(count);let entries=0;
    for(let i=0;i<count;i++){starts[i]=Math.max(0,i-bandwidth);ends[i]=Math.min(count-1,i+bandwidth);offsets[i]=entries-starts[i];entries+=ends[i]-starts[i]+1;}
    const packedLayout={starts,ends,offsets,entries,kl:bandwidth,ku:bandwidth};
    const w={layout,rows,common,relative,dual,count,positionMask,packedLayout,
        structure:{relativeCount:r,relativeBand:cluster.relative.band,toolId:cluster.toolId,hessianKind:cluster.hessianKind,
            commonDofs,rowOffsets:rowOffsets.slice(),columns:columns.slice(),modeNodes,modeOffsets,bases},
        relativeScatter:Int32Array.from(relativeScatter),remoteRelativeSlots:Int32Array.from(remoteRelativeSlots),couplingScatter:Int32Array.from(couplingScatter),
        residual:new Float64Array(count),originalResidual:new Float64Array(count),scales:new Float64Array(count),
        increment:new Float64Array(count),correction:new Float64Array(count),linearResidual:new Float64Array(count),originalLinearResidual:new Float64Array(count),
        fixedMask:new Uint8Array(count),heldMultiplierRows:new Uint8Array(rows.length),freeGeometryRows:new Uint8Array(rows.length),
        commonIncrement:new Float64Array(n),relativeIncrement:new Float64Array(r),multiplierIncrement:new Float64Array(rows.length),
        fixedReactionIncrement:new Float64Array(n),fixedStationarity:new Float64Array(n),constraintResidual:new Float64Array(rows.length),
        maxRowsPerNode:Math.max(0,...rowsByNode.map(a=>a.length))};
    validateClusterStructure(w,cluster,false);w.sparseOperator=createCompositeSparseDirectionOperator(w);return w;
}

/** Original linear equations only, at the caller's already prepared branch:
 *   [ H_common + T   C     forceColumn ] [dq]       [commonResidual  ]
 *   [ C^T           H_rho forceColumn ] [drho] = - [relativeResidual]
 *   [ geometry Jacobian   dF/dlambda  ] [dlambda]   [row.residual    ]
 * T is optional row.geometricTangent over the full ordered primal support;
 * it may be nonsymmetric/indefinite. It is added ONCE, as are H_rho and C.
 * Chain ALREADY includes Cluster.common, including physical wire inertia.
 * Supplied common/relative residuals ALREADY include current physical loads
 * and reactions; no multiplier force or Cluster gradient is added again.
 *
 * rows: {residual, jacobian, forceColumn, multiplierDerivative=0,
 *        multiplierJacobian?,
 *        geometricTangent?, tolerance}. Both signed columns are mandatory
 * finite vectors of support size; each row tolerance uses definition.unit.
 * tolerances.force gates common positions AND orthonormal relative forces;
 * tolerances.torque separately gates independent tool spins.
 * Fixed common increments are zero. With the residual convention inertia +
 * elastic gradient - applied/wall force, the boundary force ON THE CHAIN is
 * +original residual at fixed DOFs. Its increment is +A_fixed,*delta; the
 * opposite sign is the chain's force on the handle, not the force on the chain.
 * multiplierJacobian follows the frozen definition.multiplierDofs ordering;
 * its values are added once to the nonsymmetric original dual block.
 * An undetermined row with no free geometry derivative and no free dual derivative
 * holds dlambda=0; its ORIGINAL incompatibility remains in the certificate.
 * An optional numericalShift adds a diagonal only to the scaled numerical
 * search system. The original matrix, residual and proof remain separate;
 * `converged` ALWAYS refers to the original unshifted linear equations.
 * `numericalConverged` certifies only that search system, never a physical dt.
 * With eliminateZeroDuals=true (default), an unshifted solve substitutes only
 * dual increments proved exactly zero by the assembled numerical equations.
 * The original full residual, held-row incompatibilities and geometric
 * tangents remain. count describes original unknowns; solvedCount and
 * eliminatedZeroDuals describe the actual factorized system.
 * No pivot floor, disk normal, branch decision or nonlinear dt approval.
 * Returned typed arrays belong to workspace and are overwritten next solve.
 */
export function solveCompositeRelativeDirection(w,chain,{cluster,commonResidual,relativeResidual,fixed,rows=[],tolerances,maxCorrections=1,numericalShift=0,numericalDualRows=null,holdDuplicateConstraints=false,eliminateZeroDuals=true}) {
    if(chain.layout!==w.layout||chain.hessianValid===false)throw new RangeError('A fresh full common Hessian with the frozen layout is required');
    validateClusterStructure(w,cluster);
    if(chain.elementBackend!==undefined&&(chain.elementBackend==='wasm-exact')!==(cluster.hessianKind==='exact'))
        throw new RangeError('Common and relative elastic Hessian kinds must agree');
    const {layout,common,relative,dual,count,packedLayout:p,originalResidual:F,residual,increment,scales,sparseOperator:s}=w;
    const A=s.original,matrix=s.values;
    vector(chain.hessian,layout.dofCount*layout.band,'original common Hessian');
    vector(commonResidual,common.length,'original common residual');vector(relativeResidual,relative.length,'original relative residual');
    if(!fixed||fixed.length!==common.length||!fixed.every(v=>v===0||v===1)||rows.length!==w.rows.length)
        throw new RangeError('Fixed coordinates and rows must match the frozen layout');
    if(!Number.isInteger(maxCorrections)||maxCorrections<0||maxCorrections>2)throw new RangeError('maxCorrections must be 0..2');
    if(!Number.isFinite(numericalShift)||numericalShift<0)throw new RangeError('numericalShift must be finite and nonnegative');
    if(numericalDualRows!==null&&(!Array.isArray(numericalDualRows)||new Set(numericalDualRows).size!==numericalDualRows.length||
        numericalDualRows.some(index=>!Number.isInteger(index)||index<0||index>=rows.length)))
        throw new RangeError('numericalDualRows must be null or unique valid row indices');
    if(typeof holdDuplicateConstraints!=='boolean')throw new TypeError('holdDuplicateConstraints must be boolean');
    if(typeof eliminateZeroDuals!=='boolean')throw new TypeError('eliminateZeroDuals must be boolean');
    const tol={force:positive(tolerances?.force,'linear force tolerance'),torque:positive(tolerances?.torque,'linear torque tolerance')};
    s.generation++;A.fill(0);F.fill(0);increment.fill(0);w.fixedMask.fill(0);w.heldMultiplierRows.fill(0);w.freeGeometryRows.fill(0);
    const add=(slot,v)=>{if(v!==0)A[slot]+=v;};
    const scatter=(plan,values)=>{for(let i=0;i<plan.length;i+=3){const value=values[plan[i]];add(plan[i+1],value);if(plan[i+2]>=0)add(plan[i+2],value);}};
    for(let i=0;i<common.length;i++) {
        F[common[i]]=commonResidual[i];w.fixedMask[common[i]]=fixed[i];
    }
    scatter(s.commonScatter,chain.hessian);
    for(let i=0;i<relative.length;i++)F[relative[i]]=relativeResidual[i];
    scatter(s.relativeScatter,cluster.relative.hessian);scatter(s.couplingScatter,cluster.coupling.values);
    for(let index=0;index<rows.length;index++) {
        const row=rows[index];
        const def=w.rows[index],support=def.mixedDofs,d=dual[index],size=support.length,plan=s.rowPlans[index];
        if(def.constraintSupport!==null||row.constraintSupport!==undefined) {
            const a=def.constraintSupport,b=row.constraintSupport;
            const unchanged=a&&b&&a.kind===b.kind&&a.toolId===b.toolId&&a.edge===b.edge&&
                (a.kind==='native-tool-pair'?a.pairId===b.pairId&&a.spins===b.spins&&JSON.stringify(a.tools)===JSON.stringify(b.tools):a.kind==='continuous-wall-friction'?same(a.positionNodes,b.positionNodes)&&same(a.angleEdges,b.angleEdges):same(a.nodeIndices,b.nodeIndices));
            if(!unchanged)throw new RangeError('Frozen continuous physical support changed');
        }
        vector(row.jacobian,size,'geometry Jacobian');vector(row.forceColumn,size,'signed physical force column');
        positive(row.tolerance,`row ${index} tolerance (${def.unit})`);F[d]=finite(row.residual,'original constraint residual');
        const diagonal=finite(row.multiplierDerivative??0,'original multiplier derivative');add(plan.diagonal,diagonal);
        if(row.multiplierDofs!==undefined&&!same(row.multiplierDofs,def.multiplierDofs))throw new RangeError('Frozen local multiplier support changed');
        if(def.multiplierDofs.length||row.multiplierJacobian!==undefined) {
            vector(row.multiplierJacobian,def.multiplierDofs.length,'original off-diagonal multiplier Jacobian');
            for(let j=0;j<plan.multiplierSlots.length;j++)add(plan.multiplierSlots[j],row.multiplierJacobian[j]);
        }
        let hasFreeGeometry=false;
        for(let i=0;i<size;i++){
            add(plan.jacobianSlots[i],row.jacobian[i]);add(plan.forceSlots[i],row.forceColumn[i]);
            if(!w.fixedMask[support[i]]&&row.jacobian[i]!==0)hasFreeGeometry=true;
        }
        w.freeGeometryRows[index]=hasFreeGeometry?1:0;
        if(row.geometricTangent!==undefined){
            if(row.geometricTangentValid===false)throw new RangeError('A fresh full physical geometric tangent is required');
            vector(row.geometricTangent,size*size,'physical geometric tangent');
            for(let i=0;i<plan.tangentSlots.length;i++)add(plan.tangentSlots[i],row.geometricTangent[i]);}
    }
    // A coupling to another free dual is an equation, even when this row's
    // own diagonal and all free geometry coefficients vanish. Only propagate
    // the existing zero-increment gauge through already held duals. Cycles
    // remain in the original block and are solved without a diagonal shift.
    for(let pass=0;pass<w.maxRowsPerNode;pass++) {
        let changed=false;
        rows.forEach((row,index)=>{
            if(w.heldMultiplierRows[index]||w.freeGeometryRows[index]||(row.multiplierDerivative??0)!==0)return;
            if(w.rows[index].multiplierDofs.some((other,j)=>row.multiplierJacobian[j]!==0&&!w.heldMultiplierRows[other]))return;
            w.heldMultiplierRows[index]=1;changed=true;
        });
        if(!changed)break;
    }
    if(!A.every(Number.isFinite)||!F.every(Number.isFinite))throw new RangeError('Nonfinite original joint equations');
    matrix.set(A);residual.set(F);
    for(let i=0;i<count;i++)for(let k=s.offsets[i];k<s.offsets[i+1];k++)if(w.fixedMask[i]||w.fixedMask[s.columns[k]])matrix[k]=i===s.columns[k]?1:0;
    for(let i=0;i<count;i++)if(w.fixedMask[i])residual[i]=0;
    // Exact duplicate equations need only one copy in the search system.
    // Keep both physical force columns and all friction rows. Prescribing one
    // multiplier INCREMENT selects a basic linear solution; it neither merges
    // nor deletes current reactions. The full original proof below must pass.
    const duplicateConstraints=[];
    if(holdDuplicateConstraints&&numericalShift===0) {
        const equations=new Map();
        rows.forEach((row,index)=>{
            if(w.heldMultiplierRows[index]||!w.freeGeometryRows[index])return;
            const d=dual[index],entries=[];
            for(let k=s.offsets[d];k<s.offsets[d+1];k++)if(matrix[k]!==0)entries.push(s.columns[k],matrix[k]);
            const key=JSON.stringify([w.rows[index].unit,F[d],entries]);
            const representative=equations.get(key);
            if(representative===undefined)equations.set(key,index);
            else {w.heldMultiplierRows[representative]=1;equations.set(key,index);duplicateConstraints.push({index:representative,representative:index});}
        });
    }
    w.heldMultiplierRows.forEach((held,index)=>{if(held){const d=dual[index];for(let k=s.offsets[d];k<s.offsets[d+1];k++)matrix[k]=s.columns[k]===d?1:0;residual[d]=0;}});
    for(const indices of [common,relative])for(const row of indices) {
        let magnitude=Math.abs(matrix[s.diagonal[row]]);
        if(magnitude===0)for(let k=s.offsets[row];k<s.offsets[row+1];k++)magnitude=Math.max(magnitude,Math.abs(matrix[k]));
        scales[row]=magnitude>0?1/Math.sqrt(magnitude):1;
    }
    w.rows.forEach((row,index)=>{
        let magnitude=0;row.mixedDofs.forEach((dof,i)=>{if(!w.fixedMask[dof])magnitude=Math.max(magnitude,Math.abs(matrix[s.rowPlans[index].forceSlots[i]]*scales[dof]));});
        scales[dual[index]]=magnitude>0?1/magnitude:1;
    });
    if(numericalShift>0){
        for(const indices of [common,relative])for(const d of indices)if(!w.fixedMask[d])matrix[s.diagonal[d]]+=numericalShift/(scales[d]*scales[d]);
        dual.forEach((d,index)=>{if(!w.heldMultiplierRows[index]&&(numericalDualRows===null||numericalDualRows.includes(index)))matrix[s.diagonal[d]]+=numericalShift/(scales[d]*scales[d]);});
        if(!matrix.every(Number.isFinite))throw new RangeError('Nonfinite numerical search regularization');
    }
    const multiplyResidual=(operator,original,out)=>{
        for(let i=0;i<count;i++) {
            let sum=original[i],compensation=0;
            for(let k=s.offsets[i];k<s.offsets[i+1];k++){
                const value=operator[k]*increment[s.columns[k]],next=sum+value;
                compensation+=Math.abs(sum)>=Math.abs(value)?sum-next+value:value-next+sum;sum=next;
            }
            out[i]=sum+compensation;
        }
    };
    const measure=(initial=false,numerical=false)=>{
        const measured=numerical?w.linearResidual:w.originalLinearResidual;
        if(initial)measured.set(F);
        else multiplyResidual(numerical?matrix:A,F,measured);
        // Fixed primal and held dual increments are exactly zero. Free rows
        // of the modified system therefore have the SAME original residual.
        // Keep incompatible original held rows visible to the physical proof.
        if(!numerical)w.linearResidual.set(measured);
        for(let i=0;i<count;i++)if(w.fixedMask[i])w.linearResidual[i]=0;
        w.heldMultiplierRows.forEach((held,index)=>{if(held)w.linearResidual[dual[index]]=0;});
        let commonForce=0,relativeForce=0,torque=0;
        for(let i=0;i<common.length;i++)if(!fixed[i]){
            const value=Math.abs(measured[common[i]]);
            if(w.positionMask[i])commonForce=Math.max(commonForce,value);else torque=Math.max(torque,value);
        }
        for(const dof of relative)relativeForce=Math.max(relativeForce,Math.abs(measured[dof]));
        const constraints=rows.map((row,index)=>{
            // A held row's original incompatibility is never hidden by its
            // numerical identity row or the zero correction RHS.
            const value=numerical&&w.heldMultiplierRows[index]?F[dual[index]]:measured[dual[index]];w.constraintResidual[index]=value;
            return {unit:w.rows[index].unit,residual:value,tolerance:row.tolerance,converged:Math.abs(value)<=row.tolerance};
        });
        return {commonForce,relativeForce,force:Math.max(commonForce,relativeForce),torque,constraints};
    };
    const passes=proof=>proof.force<=tol.force&&proof.torque<=tol.torque&&proof.constraints.every(c=>c.converged);
    // Remove only dual increments determined to be exactly zero by these
    // assembled equations. Keep every coefficient of the sparse original
    // operator and its full certificate,
    // and preserve the established numerical-shift policy unchanged.
    const compressed=eliminateZeroDuals&&numericalShift===0?prepareCompositeZeroDualSolve(w,s):null;
    const nativeMatrix=compressed?null:w.matrix;
    let factorizations=0,linearSolves=0,solverAccepted=false,proof=measure(true,numericalShift>0);
    for(let attempt=0;attempt<=maxCorrections;attempt++) {
        factorizations++;linearSolves++;
        const rhs=attempt===0?residual:w.linearResidual;
        if(compressed) {
            compressed.original.forEach((d,i)=>compressed.residual[i]=rhs[d]);
            solverAccepted=compressed.lu.solve(compressed.matrix,compressed.residual,compressed.scales,0,compressed.correction);
            w.correction.fill(0);compressed.original.forEach((d,i)=>w.correction[d]=compressed.correction[i]);
        } else solverAccepted=w.lu.solve(nativeMatrix,rhs,scales,0,w.correction);
        if(!solverAccepted)break;
        for(let i=0;i<count;i++)increment[i]=w.fixedMask[i]?0:increment[i]+scales[i]*w.correction[i];
        // Held reaction increments are prescribed exactly, not a tiny LU error.
        w.heldMultiplierRows.forEach((held,index)=>{if(held)increment[dual[index]]=0;});
        proof=measure(false,numericalShift>0);if(passes(proof))break;
    }
    const numericalProof=proof,numericalConverged=solverAccepted&&passes(numericalProof);
    if(numericalShift>0)proof=measure();
    w.fixedReactionIncrement.fill(0);w.fixedStationarity.fill(0);
    common.forEach((d,i)=>{w.commonIncrement[i]=fixed[i]?0:increment[d];if(fixed[i]){
        w.fixedStationarity[i]=w.originalLinearResidual[d];
        // Boundary force ON THE CHAIN: R=F_fixed. The increment is
        // dR=+(H*dq+C*drho+forceColumn*dlambda), independent of initial F.
        let sum=0,compensation=0;
        for(let k=s.offsets[d];k<s.offsets[d+1];k++){
            const v=A[k]*increment[s.columns[k]],next=sum+v;compensation+=Math.abs(sum)>=Math.abs(v)?sum-next+v:v-next+sum;sum=next;
        }
        w.fixedReactionIncrement[i]=sum+compensation;
    }});
    relative.forEach((d,i)=>{w.relativeIncrement[i]=increment[d];});dual.forEach((d,i)=>{w.multiplierIncrement[i]=increment[d];});
    return {commonIncrement:w.commonIncrement,relativeIncrement:w.relativeIncrement,multiplierIncrement:w.multiplierIncrement,
        fixedReactionIncrement:w.fixedReactionIncrement,fixedStationarity:w.fixedStationarity,
        originalLinearResidual:w.originalLinearResidual,constraintResidual:w.constraintResidual,heldMultiplierRows:w.heldMultiplierRows,
        proof,numericalProof,numericalShift,numericalConverged,duplicateConstraints,factorizations,linearSolves,converged:solverAccepted&&w.originalLinearResidual.every(Number.isFinite)&&passes(proof),
        count,bandwidth:p.kl,matrixEntries:compressed?.matrix.length??nativeMatrix.length,
        factorEntries:compressed?.lu.diagnostics.factorEntries??w.lu.diagnostics.factorEntries,maxRowsPerNode:w.maxRowsPerNode,
        originalStorage:'csr',originalMatrixEntries:A.length,originalBandEntries:p.entries,
        materializedOriginalBandEntries:s.materializedOriginalEntries,materializedNumericalBandEntries:s.materializedNumericalEntries,
        solvedCount:compressed?.count??count,eliminatedZeroDuals:compressed?.removed??0,
        solvedBandwidth:compressed?.packedLayout.kl??p.kl,solvedFactorEntries:compressed?.lu.diagnostics.factorEntries??w.lu.diagnostics.factorEntries,
        scope:'one-common-axis-and-joint-relative-original-linear-direction',relativeRepresentation:cluster.relativeRepresentation,
        fullRankOnRepresentedNodes:cluster.fullRankOnRepresentedNodes===true,certified:false,nonlinearStepAccepted:false};
}

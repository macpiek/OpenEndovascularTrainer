import { createCompositeElementWorkspace, evaluateCompositeElement } from './kirchhoffCompositeElement.js';
import * as fastElement from './kirchhoffCompositeElementFast.js';
import * as exactElement from './kirchhoffCompositeElementExact.js';
import { createKirchhoffLinearKernel } from './kirchhoffLinearKernel.js';

/** One spatially ordered chain. Each node owns its position ONCE; an outgoing
 * edge owns one independent scalar spin for each material actually present.
 * Material tips must coincide with mesh boundaries supplied by topology.
 */
export function createCompositeChainLayout(edgeToolIds,{positionSupports=[],materialSupports=[]}={}) {
    if (!Array.isArray(edgeToolIds) || !edgeToolIds.length) throw new RangeError('A chain needs at least one edge');
    const count = edgeToolIds.length + 1, ids = new Set();
    const edges = edgeToolIds.map(list => {
        if (!Array.isArray(list) || !list.length || list.length > 2 || new Set(list).size !== list.length ||
            list.some(id => typeof id !== 'string' || !id)) throw new TypeError('Each edge needs one or two distinct tool ids');
        list.forEach(id => ids.add(id)); return [...list];
    });
    if (ids.size > 2) throw new RangeError('A catheter/guidewire chain contains at most two materials');
    const positions = new Int32Array(count), spins = new Map([...ids].map(id => [id,new Int32Array(count-1).fill(-1)]));
    for (const id of ids) {
        const covered = edges.map((e,i) => e.includes(id) ? i : -1).filter(i => i >= 0);
        if (covered.at(-1)-covered[0]+1 !== covered.length) throw new RangeError('Each tool must occupy one connected material interval');
    }
    let dofCount = 0;
    for (let node = 0; node < count; node++) {
        positions[node] = dofCount; dofCount += 3;
        for (const id of edges[node] ?? []) spins.get(id)[node] = dofCount++;
    }
    const hinges = []; let band = 1;
    for (let vertex = 1; vertex + 1 < count; vertex++) {
        const tools = edges[vertex-1].filter(id => edges[vertex].includes(id));
        if (!tools.length) throw new RangeError('Adjacent sections require a continuous material through their interface');
        const dofs = [vertex-1,vertex,vertex+1].flatMap(node => [positions[node],positions[node]+1,positions[node]+2]);
        for (const id of tools) dofs.push(spins.get(id)[vertex-1],spins.get(id)[vertex]);
        band = Math.max(band,Math.max(...dofs)-Math.min(...dofs)+1);
        hinges.push({vertex,tools,dofs:Int32Array.from(dofs)});
    }
    // Continuous interpolation can couple more position nodes than a native
    // three-node bending hinge. Reserve those actual entries without adding
    // unknowns or silently writing an out-of-band coefficient into another row.
    if(!Array.isArray(positionSupports))throw new RangeError('Position supports must be explicit node lists');
    const supports=positionSupports.map(nodes=>{
        if(!Array.isArray(nodes)||!nodes.length||nodes.some((node,j)=>!Number.isInteger(node)||node<0||node>=count||(j&&node<=nodes[j-1])))
            throw new RangeError('Position support nodes must increase within the chain');
        band=Math.max(band,positions[nodes.at(-1)]+2-positions[nodes[0]]+1);return nodes.slice();
    });
    if(!Array.isArray(materialSupports))throw new RangeError('Material supports must be explicit physical position/spin lists');
    const physicalSupports=materialSupports.map(s=>{
        const spin=spins.get(s.toolId),nodes=s.positionNodes,angles=s.angleEdges;
        if(!spin||!Number.isInteger(s.edge)||spin[s.edge]<0||spin[s.edge]===undefined||!Array.isArray(nodes)||!nodes.length||!Array.isArray(angles)||!angles.length||
            nodes.some((node,j)=>!Number.isInteger(node)||node<0||node>=count||(j&&node<=nodes[j-1])||!([edges[node-1],edges[node]].some(ids=>ids?.includes(s.toolId))))||
            angles.some((edge,j)=>!Number.isInteger(edge)||edge<0||edge>=count-1||spin[edge]<0||(j&&edge<=angles[j-1])))
            throw new RangeError('Material support must contain ordered active position and spin nodes');
        const dofs=nodes.flatMap(node=>[positions[node],positions[node]+1,positions[node]+2]).concat(angles.map(edge=>spin[edge]));
        band=Math.max(band,Math.max(...dofs)-Math.min(...dofs)+1);
        return {toolId:s.toolId,edge:s.edge,positionNodes:nodes.slice(),angleEdges:angles.slice()};
    });
    return { nodeCount:count, edgeToolIds:edges, positions, spins, hinges, dofCount, band,positionSupports:supports,materialSupports:physicalSupports };
}

export function createCompositeChainWorkspace(layout,{elementBackend='wasm'}={}) {
    if(!['wasm','javascript','wasm-exact'].includes(elementBackend)) throw new TypeError('Unknown composite element backend');
    const elementWorkspace=elementBackend==='wasm-exact'?exactElement.createCompositeElementWorkspace:
        elementBackend==='wasm'?fastElement.createCompositeElementWorkspace:createCompositeElementWorkspace;
    const n=layout.dofCount, band=layout.band;
    const kernel=createKirchhoffLinearKernel(8*(3*n*band+6*n)+256);
    return {layout,kernel,energy:0,elementBackend,hessianValid:true,evaluationOrder:null,
        evaluateElement:elementBackend==='wasm-exact'?exactElement.evaluateCompositeElement:
            elementBackend==='wasm'?fastElement.evaluateCompositeElement:evaluateCompositeElement,
        evaluatedReferenceTwists:new Map([...layout.spins.keys()].map(id=>[id,new Float64Array(Math.max(0,layout.nodeCount-2)).fill(NaN)])),
        gradient:kernel.alloc(Float64Array,n), hessian:kernel.alloc(Float64Array,n*band),
        matrix:kernel.alloc(Float64Array,n*band), factor:kernel.alloc(Float64Array,n*band),
        rhs:kernel.alloc(Float64Array,n), increment:kernel.alloc(Float64Array,n),
        residual:kernel.alloc(Float64Array,n), reactions:kernel.alloc(Float64Array,n), scales:kernel.alloc(Float64Array,n),
        local:[null,elementWorkspace(1),elementWorkspace(2)]};
}

/** Assemble energy, exact gradient and the selected tangent directly into a
 * local lower band. Default backends retain GN; explicit wasm-exact includes
 * strain second derivatives and may be indefinite. No two-rod constraint matrix, global contact Schur
 * complement or equality-response columns are constructed. This is the
 * constitutive operator; inertia, length and contact belong to its caller.
 * Explicit order:'gradient' (wasm-exact only) skips Hessian evaluation/scatter,
 * leaves its bytes stale and marks hessianValid:false. Full is the default;
 * direction solvers must reject an invalid Hessian regardless of its bytes.
 */
export function assembleCompositeChain({positions,coordinates,reference,tools},workspace,{order='full'}={}) {
    workspace.hessianValid=false;workspace.evaluationOrder=order;
    if(!['full','gradient'].includes(order))throw new TypeError('Chain order must be full or gradient');
    if(order==='gradient'&&workspace.elementBackend!=='wasm-exact')throw new TypeError('Gradient-only assembly requires the wasm-exact backend');
    const evaluation={order},withHessian=order==='full';
    const {layout,gradient,hessian}=workspace;
    if(positions.length!==layout.nodeCount || coordinates.length!==layout.nodeCount || reference.length!==layout.nodeCount-1)
        throw new RangeError('Geometry and reference frames must match the chain');
    for(let i=0;i<coordinates.length;i++) if(!Number.isFinite(coordinates[i]) || (i && coordinates[i]<=coordinates[i-1]))
        throw new RangeError('Centerline material coordinates must increase');
    const byId=new Map(tools.map(tool=>[tool.id,tool]));
    if(byId.size!==tools.length || [...layout.spins.keys()].some(id=>!byId.has(id))) throw new TypeError('Unique material data for every tool is required');
    gradient.fill(0); if(withHessian)hessian.fill(0); workspace.energy=0;
    for(const {vertex:i,tools:ids,dofs} of layout.hinges) {
        const samples=ids.map(id=> {
            const tool=byId.get(id);
            return {angles:[tool.angles[i-1],tool.angles[i]],dsDx:typeof tool.dsDx==='function'?tool.dsDx(coordinates[i]):tool.dsDx??1,
                referenceTwist:tool.referenceTwists?.[i-1] ?? 0,
                material:tool.materialAt?tool.materialAt({vertex:i,coordinate:coordinates[i],
                    start:(coordinates[i-1]+coordinates[i])/2,end:(coordinates[i]+coordinates[i+1])/2}):tool.material};
        });
        const local=workspace.evaluateElement({positions:[positions[i-1],positions[i],positions[i+1]],
            reference:[reference[i-1],reference[i]],referenceLength:(coordinates[i+1]-coordinates[i-1])/2,tools:samples},workspace.local[ids.length],evaluation);
        workspace.energy+=local.energy;
        ids.forEach((id,j)=>{workspace.evaluatedReferenceTwists.get(id)[i-1]=local.referenceTwists[j];});
        for(let row=0;row<dofs.length;row++) {
            gradient[dofs[row]]+=local.gradient[row];
            if(withHessian)for(let col=0;col<=row;col++) {
                const hi=Math.max(dofs[row],dofs[col]),lo=Math.min(dofs[row],dofs[col]);
                hessian[hi*layout.band+hi-lo]+=local.hessian[row*dofs.length+col];
            }
        }
    }
    workspace.hessianValid=withHessian;
    return workspace;
}

// Cholesky of the unshifted, equilibrated operator. A pivot floor changes
// null modes into artificial supports, even when the right-hand side is zero.
// Reject pivots within the local floating-point subtraction error instead.
function factorCheckedBand(factor,n,band) {
    for(let i=0;i<n;i++) for(let j=Math.max(0,i-band+1);j<=i;j++) {
        const at=i*band+i-j;
        let value=factor[at], magnitude=Math.abs(value);
        for(let k=Math.max(0,i-band+1);k<j;k++) {
            const product=factor[i*band+i-k]*factor[j*band+j-k];
            value-=product; magnitude+=Math.abs(product);
        }
        if(i===j) {
            if(!Number.isFinite(value) || !(value>32*Number.EPSILON*band*magnitude))
                throw new RangeError('An unconstrained zero-energy mode or numerically singular operator needs inertia or a physical boundary');
            factor[at]=Math.sqrt(value);
        } else {
            factor[at]=value/factor[j*band];
            if(!Number.isFinite(factor[at])) throw new RangeError('Nonfinite composite factor');
        }
    }
}

const productSplitter=134217729;
const productSplitBits=new DataView(new ArrayBuffer(8));
function truncatedProductHead(value) {
    // Clear the bottom 27 significand bits without overflowing a splitter
    // multiplication for very large finite matrix entries.
    productSplitBits.setFloat64(0,value,false);
    productSplitBits.setUint32(4,productSplitBits.getUint32(4,false)&0xf8000000,false);
    return productSplitBits.getFloat64(0,false);
}
function productRoundingError(a,b,product) {
    const splitA=productSplitter*a,splitB=productSplitter*b;
    let ah=splitA-(splitA-a),bh=splitB-(splitB-b);
    if(!Number.isFinite(ah)||!Number.isFinite(bh)||!Number.isFinite(ah*bh)) {
        ah=truncatedProductHead(a);bh=truncatedProductHead(b);
    }
    const al=a-ah,bl=b-bh;
    return ((ah*bh-product)+ah*bl+al*bh)+al*bl;
}
function evaluateOriginalResidual(matrix,rhs,increment,residual,reactions,n,band,fixed) {
    let maximumResidual=0;
    for(let i=0;i<n;i++) {
        let sum=rhs[i],compensation=0;
        for(let j=Math.max(0,i-band+1);j<Math.min(n,i+band);j++) {
            const value=matrix[Math.max(i,j)*band+Math.abs(i-j)],product=value*increment[j],term=-product;
            const next=sum+term;
            compensation+=(Math.abs(sum)>=Math.abs(term)?(sum-next)+term:(term-next)+sum)
                -productRoundingError(value,increment[j],product);
            sum=next;
        }
        // Compensate both products and summation in ORIGINAL physical units.
        // A small backward/scaled residual is never substituted for this norm.
        residual[i]=sum+compensation;
        if(!Number.isFinite(residual[i])) throw new RangeError('Nonfinite composite original residual');
        reactions[i]=fixed?.[i]?-residual[i]:0;
        if(!fixed?.[i]) maximumResidual=Math.max(maximumResidual,Math.abs(residual[i]));
    }
    return maximumResidual;
}

/** One banded primal direction. diagonal and extraGradient can supply the
 * implicit kinetic term or an augmented constraint term. fixed holds EXACT
 * zero increments; their reactions are reconstructed from the original
 * operator after solving. At most maxRefinementSteps residual corrections use
 * the SAME unshifted factor. Both stopping and returned reactions use a
 * compensated original-matrix residual; failure is never scaled into success.
 * This is not a complete nonlinear/contact step.
 */
export function solveCompositeChainIncrement(workspace,{diagonal,extraGradient,fixed,tolerance=1e-9,maxRefinementSteps=2}={}) {
    if(workspace.hessianValid===false)throw new RangeError('A fresh full Hessian assembly is required before a Chain direction');
    const {layout,kernel,matrix,factor,rhs,increment,residual,reactions,scales,gradient,hessian}=workspace;
    const n=layout.dofCount,band=layout.band;
    if(!Number.isFinite(tolerance)||tolerance<=0) throw new RangeError('A positive residual tolerance is required');
    if(!Number.isInteger(maxRefinementSteps)||maxRefinementSteps<0||maxRefinementSteps>3)
        throw new RangeError('maxRefinementSteps must be an integer from 0 to 3');
    if([diagonal,extraGradient,fixed].some(v=>v && v.length!==n)) throw new RangeError('Direction inputs must match the reduced DOFs');
    if(!hessian.every(Number.isFinite)||!gradient.every(Number.isFinite)) throw new RangeError('Nonfinite original composite operator');
    matrix.set(hessian);
    for(let i=0;i<n;i++) {
        const d=diagonal?.[i]??0,g=extraGradient?.[i]??0;
        if(!Number.isFinite(d)||d<0||!Number.isFinite(g)) throw new RangeError('Finite nonnegative inertia and finite applied gradients are required');
        matrix[i*band]+=d; rhs[i]=-gradient[i]-g;
        if(!Number.isFinite(matrix[i*band])||!Number.isFinite(rhs[i])) throw new RangeError('Nonfinite composite direction inputs');
        const pivot=fixed?.[i]?1:matrix[i*band];
        if(!(pivot>0)) throw new RangeError('An unconstrained zero-energy mode needs inertia or a physical boundary');
        scales[i]=1/Math.sqrt(pivot);
    }
    factor.fill(0);
    for(let i=0;i<n;i++) {
        for(let j=Math.max(0,i-band+1);j<=i;j++) factor[i*band+i-j]=fixed?.[i]||fixed?.[j]
            ?(i===j?1:0):matrix[i*band+i-j]*scales[i]*scales[j];
        increment[i]=fixed?.[i]?0:rhs[i]*scales[i];
    }
    factorCheckedBand(factor,n,band);
    kernel.solveBand(factor.byteOffset,increment.byteOffset,n,band);
    for(let i=0;i<n;i++) increment[i]*=scales[i];
    let maximumResidual=evaluateOriginalResidual(matrix,rhs,increment,residual,reactions,n,band,fixed);
    const initialMaximumResidual=maximumResidual;
    let refinementSteps=0;
    while(maximumResidual>tolerance&&refinementSteps<maxRefinementSteps) {
        // Borrow residual's existing kernel storage for the correction. RHS,
        // original matrix, fixed mask and factor remain untouched.
        for(let i=0;i<n;i++) residual[i]=fixed?.[i]?0:residual[i]*scales[i];
        kernel.solveBand(factor.byteOffset,residual.byteOffset,n,band);
        for(let i=0;i<n;i++) increment[i]=fixed?.[i]?0:increment[i]+residual[i]*scales[i];
        refinementSteps++;
        maximumResidual=evaluateOriginalResidual(matrix,rhs,increment,residual,reactions,n,band,fixed);
    }
    if([increment,residual,reactions].some(values=>!values.every(Number.isFinite))) throw new RangeError('Nonfinite composite solution or reactions');
    return {increment,residual,reactions,converged:Number.isFinite(maximumResidual)&&maximumResidual<=tolerance,
        maximumResidual,initialMaximumResidual,refinementSteps,linearSolves:1+refinementSteps,
        factorizations:1,dofCount:n,band,matrixEntries:n*band};
}

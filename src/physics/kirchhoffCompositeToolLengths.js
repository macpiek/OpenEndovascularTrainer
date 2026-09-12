import {createCompositeContinuousLength} from './kirchhoffCompositeContinuousLength.js';

const positive = (value, name) => { if (!(value > 0) || !Number.isFinite(value)) throw new RangeError(`${name} must be positive and finite`); return value; };
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

/** Separate physical rest metrics for both materials in common/relative
 * coordinates. A wire position has the already-declared affine pullback
 * x_wire(node)=q(node)+B(node)*rho(node); other materials use q. Bases freeze
 * during a prepared solve. Physical toolPositions are supplied by the SAME
 * reconstruction used for elasticity/inertia, not inferred from rest lengths.
 *
 * Each material edge contributes its own original |x1-x0|-restLength row,
 * or, with explicit geometryByTool, integral |q'(x)| dx-restLength for that
 * same continuous curve. Continuous rows retain all neighboring shape nodes
 * and report their parameter-speed bounds; integrated length preservation
 * does not certify pointwise inextensibility of a coarse polynomial element.
 * Two transverse coordinates alone do not generally admit independent
 * inextensible material lengths. This operator does not hide that obstruction
 * by discarding the wire row or replacing its rest metric with the common one.
 */
export function createCompositeToolLengthWorkspace({ layout, modes = [], relativeToolId = 'wire',geometryByTool=null }) {
    if (!layout || layout.nodeCount < 2 || layout.edgeToolIds?.length !== layout.nodeCount - 1)
        throw new RangeError('A fixed common layout is required');
    if(geometryByTool!==null&&(!(geometryByTool instanceof Map)||geometryByTool.size!==layout.spins.size||[...layout.spins.keys()].some(id=>!geometryByTool.has(id))))
        throw new RangeError('Continuous lengths require matching geometry for every material');
    const byNode = new Map(); let relativeCount = 0, previousNode = -1;
    for (const mode of modes) {
        const dimension = mode.basis?.length;
        if (!Number.isInteger(mode.node) || mode.node <= previousNode || mode.node >= layout.nodeCount ||
            ![2, 3].includes(dimension) || mode.relativeDofs?.length !== dimension ||
            mode.relativeDofs.some((dof, i) => dof !== relativeCount + i))
            throw new RangeError('Ordered modes need distinct nodes, two/three basis vectors and contiguous relative DOFs');
        const basis = mode.basis.map(v => {
            if (v?.length !== 3 || !v.every(Number.isFinite)) throw new RangeError('Finite frozen basis vectors are required');
            return Array.from(v);
        });
        for (let i = 0; i < dimension; i++) for (let j = 0; j <= i; j++)
            if (Math.abs(dot(basis[i], basis[j]) - (i === j ? 1 : 0)) > 1e-10) throw new RangeError('Frozen mode basis must be orthonormal');
        const incident = [mode.node - 1, mode.node].filter(edge => edge >= 0 && edge < layout.nodeCount - 1);
        if (!incident.some(edge => layout.edgeToolIds[edge].includes(relativeToolId))) throw new RangeError('A relative mode needs its material at that node');
        byNode.set(mode.node, { basis, relativeDofs: Array.from(mode.relativeDofs) }); relativeCount += dimension; previousNode = mode.node;
    }
    const rows = [];
    for (let edge = 0; edge < layout.nodeCount - 1; edge++) for (const toolId of layout.edgeToolIds[edge]) {
        if(geometryByTool!==null) {
            const geometry=geometryByTool.get(toolId)?.edges?.[edge],compiled=createCompositeContinuousLength({geometry}),nodes=geometry.nodeIndices,
                commonDofs=Int32Array.from(nodes.flatMap(node=>[layout.positions[node],layout.positions[node]+1,layout.positions[node]+2])),
                terms=Array.from({length:3*nodes.length},(_,j)=>[j,1]),relativeDofs=[];
            if(geometry.edge!==edge||!nodes.includes(edge)||!nodes.includes(edge+1))throw new RangeError('Continuous length geometry must belong to its declared edge');
            if(nodes.some(node=>!([layout.edgeToolIds[node-1],layout.edgeToolIds[node]].some(ids=>ids?.includes(toolId)))))
                throw new RangeError('Continuous length support must stay within its physical material');
            if(toolId===relativeToolId)nodes.forEach((node,index)=>{
                const mode=byNode.get(node);if(mode)mode.basis.forEach((basis,axis)=>{
                    relativeDofs.push(mode.relativeDofs[axis]);terms.push(basis.flatMap((value,k)=>value===0?[]:[3*index+k,value]));
                });
            });
            const size=terms.length;
            rows.push({index:rows.length,toolId,edge,anchorNode:edge,unit:'mm',commonDofs,relativeDofs:Int32Array.from(relativeDofs),
                nodeIndices:nodes,compiled,terms,lengthGeometry:'continuous-arclength',
                constraintSupport:{kind:'continuous-arclength',toolId,edge,nodeIndices:nodes.slice()},
                length:0,residual:0,multiplier:0,multiplierDerivative:0,tolerance:null,geometricTangentValid:false,
                jacobian:new Float64Array(size),forceColumn:new Float64Array(size),geometricTangent:new Float64Array(size*size),
                speedBounds:null,parameterMetricDeviationBound:Infinity});
            continue;
        }
        const commonDofs = Int32Array.from([layout.positions[edge], layout.positions[edge + 1]].flatMap(first => [first, first + 1, first + 2]));
        const vectors = [[-1, 0, 0], [0, -1, 0], [0, 0, -1], [1, 0, 0], [0, 1, 0], [0, 0, 1]], relativeDofs = [];
        if (toolId === relativeToolId) for (let end = 0; end < 2; end++) {
            const mode = byNode.get(edge + end); if (!mode) continue;
            mode.basis.forEach((basis, axis) => {
                relativeDofs.push(mode.relativeDofs[axis]); vectors.push(basis.map(v => end ? v : -v));
            });
        }
        const size = vectors.length, metric = new Float64Array(size * size);
        for (let i = 0; i < size; i++) for (let j = 0; j < size; j++) metric[size * i + j] = dot(vectors[i], vectors[j]);
        rows.push({ index: rows.length, toolId, edge, anchorNode: edge, unit: 'mm', commonDofs,
            nodeIndices:[edge,edge+1],lengthGeometry:'native-chords',
            relativeDofs: Int32Array.from(relativeDofs), vectors, metric, length: 0, residual: 0,
            multiplier: 0, multiplierDerivative: 0, tolerance: null, geometricTangentValid: false, direction: new Float64Array(3),
            jacobian: new Float64Array(size), forceColumn: new Float64Array(size), geometricTangent: new Float64Array(size * size) });
    }
    const incidence=new Map();
    for(const row of rows)for(const id of [...row.commonDofs].map(d=>`q:${d}`).concat([...row.relativeDofs].map(d=>`r:${d}`)))
        incidence.set(id,(incidence.get(id)??0)+1);
    const maximumIncidentRows=Math.max(1,...incidence.values());
    return { layout, relativeToolId, relativeCount, rows, maximumIncidentRows,commonGradient: new Float64Array(layout.dofCount),
        relativeGradient: new Float64Array(relativeCount), maximumResidual: Infinity,
        converged: false, operatorReady: false, hessianValid: false, certified: false, scope: geometryByTool===null?'original-separate-material-edge-lengths':'separate-continuous-material-arclengths' };
}

/** Current physical positions and each material's explicit immutable rest
 * lengths are required. Multipliers are signed bilateral forces, not penalties.
 * Returned residual/column/stress blocks go into the SAME joint direction.
 * Common and relative gradients already contain +lambda*J. No force is added
 * to a material twice. order:'gradient' keeps the same gap/G/physical reaction,
 * leaves geometric tangent bytes stale and invalidates them explicitly. A
 * fresh full evaluation is required before a direction. This measurement
 * does not accept an entire timestep.
 */
export function evaluateCompositeToolLengths({ toolPositions, restLengths, multipliers, tolerance, forceTolerance=1e-7, order = 'full' }, workspace) {
    workspace.converged = workspace.operatorReady = workspace.hessianValid = false;
    workspace.quadrature=null;
    for (const row of workspace.rows) row.geometricTangentValid = false;
    workspace.maximumResidual = Infinity;
    if (order !== 'full' && order !== 'gradient') throw new TypeError('Material length order must be full or gradient');
    const full = order === 'full';
    positive(tolerance, 'Length tolerance');
    positive(forceTolerance,'Physical force tolerance');
    if (!(toolPositions instanceof Map) || !(restLengths instanceof Map) || multipliers?.length !== workspace.rows.length)
        throw new RangeError('Explicit physical tool position/rest maps and one multiplier per material edge are required');
    workspace.maximumResidual = 0; workspace.converged = false;
    workspace.commonGradient.fill(0); workspace.relativeGradient.fill(0);
    // Reserve 2% of the existing nonlinear force gate for all length-reaction
    // quadrature combined. Incident rows, all three components and rho's
    // linear pullback are accounted for; no final force gate is relaxed.
    // Hessians receive the corresponding force/length budget at the own
    // reference edge scale. They guide Newton; fresh forces still decide dt.
    const forceBudget=.02*forceTolerance,componentBudget=forceBudget/Math.sqrt(3),
        perRowBudget=componentBudget/workspace.maximumIncidentRows;
    let maximumEstimatedReactionError=0,maximumEstimatedTangentError=0;
    for (const row of workspace.rows) {
        const positions = toolPositions.get(row.toolId), metric = restLengths.get(row.toolId), lambda = multipliers[row.index];
        if (positions?.length !== workspace.layout.nodeCount || metric?.length !== workspace.layout.nodeCount - 1 || !Number.isFinite(lambda))
            throw new RangeError('Physical positions, independent rest lengths and signed forces must match the fixed material topology');
        const rest = positive(metric[row.edge], 'Material rest length');
        if(row.lengthGeometry==='continuous-arclength') {
            const precision=Math.max(1,Math.abs(lambda)),pullback=Math.max(1,...row.terms.map(term=>term.reduce((sum,v,j)=>sum+(j%2?Math.abs(v):0),0))),
                gradientTolerance=perRowBudget/(precision*pullback),hessianTolerance=perRowBudget/(precision*pullback*pullback*rest),
                requested={order,lengthTolerance:.02*tolerance,gradientTolerance,hessianTolerance};
            let response;
            try {response=row.compiled.evaluate(row.nodeIndices.map(node=>positions[node]),requested);}
            catch(error) {if(error?.details)error.details={...error.details,toolId:row.toolId,edge:row.edge,multiplier:lambda,
                forceTolerance,forceBudget,maximumIncidentRows:workspace.maximumIncidentRows,pullback,referenceLength:rest};throw error;}
            const
                size=row.terms.length,N=response.gradient.length,commonCount=row.commonDofs.length;
            row.quadrature={...response.quadrature,forceBudget:perRowBudget,gradientTolerance,hessianTolerance,
                estimatedReactionError:Math.abs(lambda)*pullback*response.quadrature.estimatedError.gradient,
                estimatedTangentError:full?Math.abs(lambda)*pullback*pullback*response.quadrature.estimatedError.hessian:null};
            maximumEstimatedReactionError=Math.max(maximumEstimatedReactionError,row.quadrature.estimatedReactionError);
            if(full)maximumEstimatedTangentError=Math.max(maximumEstimatedTangentError,row.quadrature.estimatedTangentError);
            row.length=response.length;row.residual=row.length-rest;row.multiplier=lambda;row.tolerance=tolerance;
            row.speedBounds={...response.speedBounds};row.parameterMetricDeviationBound=Math.max(Math.abs(response.speedBounds.lower/(rest/row.compiled.coordinateLength)-1),Math.abs(response.speedBounds.upper/(rest/row.compiled.coordinateLength)-1));
            workspace.maximumResidual=Math.max(workspace.maximumResidual,Math.abs(row.residual));
            for(let i=0;i<size;i++) {
                const left=row.terms[i];let J=0;for(let at=0;at<left.length;at+=2)J+=left[at+1]*response.gradient[left[at]];
                row.jacobian[i]=row.forceColumn[i]=J;
                if(i<commonCount)workspace.commonGradient[row.commonDofs[i]]+=lambda*J;
                else workspace.relativeGradient[row.relativeDofs[i-commonCount]]+=lambda*J;
                if(full)for(let j=0;j<size;j++) {
                    const right=row.terms[j];let H=0;
                    for(let a=0;a<left.length;a+=2)for(let b=0;b<right.length;b+=2)H+=left[a+1]*right[b+1]*response.hessian[left[a]*N+right[b]];
                    row.geometricTangent[size*i+j]=lambda*H;
                }
            }
            if(!row.jacobian.every(Number.isFinite)||(full&&!row.geometricTangent.every(Number.isFinite)))throw new RangeError('Nonfinite continuous length reaction');
            continue;
        }
        const a = positions[row.edge], b = positions[row.edge + 1];
        if (a?.length !== 3 || b?.length !== 3 || !a.every(Number.isFinite) || !b.every(Number.isFinite)) throw new RangeError('Finite physical material endpoints are required');
        for (let axis = 0; axis < 3; axis++) row.direction[axis] = b[axis] - a[axis];
        row.length = positive(Math.hypot(...row.direction), 'Physical material edge length');
        for (let axis = 0; axis < 3; axis++) row.direction[axis] /= row.length;
        row.residual = row.length - rest; row.multiplier = lambda; row.tolerance = tolerance;
        workspace.maximumResidual = Math.max(workspace.maximumResidual, Math.abs(row.residual));
        const size = row.vectors.length;
        for (let i = 0; i < size; i++) row.jacobian[i] = row.forceColumn[i] = dot(row.vectors[i], row.direction);
        for (let i = 0; i < size; i++) {
            if (i < 6) workspace.commonGradient[row.commonDofs[i]] += lambda * row.jacobian[i];
            else workspace.relativeGradient[row.relativeDofs[i - 6]] += lambda * row.jacobian[i];
            if (full) for (let j = 0; j < size; j++) row.geometricTangent[size * i + j] = lambda / row.length *
                (row.metric[size * i + j] - row.jacobian[i] * row.jacobian[j]);
        }
        if (!row.jacobian.every(Number.isFinite) || (full && !row.geometricTangent.every(Number.isFinite))) throw new RangeError('Nonfinite material length operator');
    }
    if (!workspace.commonGradient.every(Number.isFinite) || !workspace.relativeGradient.every(Number.isFinite)) throw new RangeError('Nonfinite length reaction');
    workspace.converged = workspace.maximumResidual <= tolerance;
    workspace.operatorReady = true;
    workspace.quadrature={forceTolerance,forceBudget,maximumIncidentRows:workspace.maximumIncidentRows,
        estimatedForceError:Math.sqrt(3)*workspace.maximumIncidentRows*maximumEstimatedReactionError,
        estimatedMaximumTangentEntryError:full?workspace.maximumIncidentRows*maximumEstimatedTangentError:null,rigorousErrorBound:false};
    workspace.hessianValid = full;
    for (const row of workspace.rows) row.geometricTangentValid = full;
    return workspace;
}

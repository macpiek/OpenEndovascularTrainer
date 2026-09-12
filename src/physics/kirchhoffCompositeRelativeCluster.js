import * as gn from './kirchhoffCompositeElementFast.js';
import * as exact from './kirchhoffCompositeElementExact.js';
import * as oracle from './kirchhoffCompositeElement.js';
import {createCompositeInertiaWorkspace,assembleCompositeTranslationalInertia} from './kirchhoffCompositeKinematics.js';
import {createCompositeContinuousMaterialInertiaEdge} from './kirchhoffCompositeContinuousInertia.js';

const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const finiteVector=(v,n,name)=>{if(!v||v.length!==n||!v.every(Number.isFinite))throw new TypeError(`${name} needs ${n} finite entries`);};
const packed=(row,col,band)=>Math.max(row,col)*band+Math.abs(row-col);

function prepareStructure({data,layout,modes,toolId=layout?.spins?.size===1?layout.spins.keys().next().value:'wire',elementBackend='wasm',inertia=null}) {
    const implementation=elementBackend==='wasm'?gn:elementBackend==='wasm-exact'?exact:elementBackend==='javascript'?oracle:null;
    if(!implementation)throw new TypeError('Relative cluster backend must be wasm, wasm-exact or javascript');
    if(!layout||data?.positions?.length!==layout.nodeCount||data?.coordinates?.length!==layout.nodeCount||
        data?.reference?.length!==layout.nodeCount-1)throw new RangeError('Cluster needs matching common data/layout');
    const empty=Array.isArray(modes)&&modes.length===0&&layout.spins.size===1&&data.tools?.length===1;
    if(!Array.isArray(modes)||(!modes.length&&!empty))throw new RangeError('At least one relative mode is required unless the chart has one material');
    const tool=data.tools.find(t=>t.id===toolId),spin=layout.spins.get(toolId);
    if(!tool||!spin)throw new RangeError('The relative material must exist in the common layout');
    let relativeCount=0;
    const ownModes=modes.map(({node,basis},index)=>{
        if(!Number.isInteger(node)||node<0||node>=layout.nodeCount||(index&&node<=modes[index-1].node))
            throw new RangeError('Mode nodes must be unique, strictly increasing and within the common chain');
        if(!basis||![2,3].includes(basis.length))throw new TypeError('Two transverse or three complete basis vectors are required');
        const incidentEdges=[node-1,node].filter(edge=>edge>=0&&edge<layout.nodeCount-1);
        if(basis.length===2) {
            if(node===0||node===layout.nodeCount-1)throw new RangeError('Reduced two-coordinate modes must remain interior');
            if(spin[node-1]<0||spin[node]<0)throw new RangeError('Each reduced mode requires its material on both sides');
        } else {
            if(!incidentEdges.some(edge=>spin[edge]>=0))throw new RangeError('A full mode needs an active incident edge of its relative material');
            if(!incidentEdges.some(edge=>layout.edgeToolIds[edge].some(id=>id!==toolId)))
                throw new RangeError('Full relative coordinates require two materials at each represented overlap node');
        }
        basis.forEach(v=>finiteVector(v,3,'basis vector'));
        for(let i=0;i<basis.length;i++)for(let j=0;j<=i;j++)
            if(Math.abs(dot(basis[i],basis[j])-(i===j?1:0))>1e-10)throw new RangeError('The frozen basis must be orthonormal');
        if(basis.length===2) {
            finiteVector(data.positions[node-1],3,'common position');finiteVector(data.positions[node+1],3,'common position');
            const chord=data.positions[node+1].map((v,i)=>v-data.positions[node-1][i]),length=Math.hypot(...chord);
            if(!(length>0)||!Number.isFinite(length)||basis.some(v=>Math.abs(dot(v,chord)/length)>1e-10))
                throw new RangeError('Reduced two-coordinate modes must be transverse to the centered common chord');
        }
        const relativeDofs=Array.from({length:basis.length},()=>relativeCount++);
        return Object.freeze({node,dimension:basis.length,basis:Object.freeze(basis.map(v=>Object.freeze(Array.from(v)))),relativeDofs:Object.freeze(relativeDofs)});
    });
    const positionDofs=node=>[layout.positions[node],layout.positions[node]+1,layout.positions[node]+2];
    const descriptors=data.elasticityGeometry==='continuous-material-frame'?(layout.materialSupports??[]).filter(s=>s.toolId===toolId&&ownModes.some(mode=>s.positionNodes.includes(mode.node)))
        .map(s=>({kind:'wire-continuous-elastic',edge:s.edge,nodes:s.positionNodes,dofs:s.positionNodes.flatMap(positionDofs).concat(s.angleEdges.map(edge=>spin[edge]))}))
        :layout.hinges.filter(h=>h.tools.includes(toolId)&&ownModes.some(mode=>Math.abs(mode.node-h.vertex)<=1))
        .map(h=>({kind:'wire-hinge',vertex:h.vertex,nodes:[h.vertex-1,h.vertex,h.vertex+1],
            dofs:[h.vertex-1,h.vertex,h.vertex+1].flatMap(positionDofs).concat([spin[h.vertex-1],spin[h.vertex]])}));
    const affectedHinges=descriptors.map(d=>d.vertex),affectedInertiaEdges=[];
    if(inertia!==null) {
        if(inertia.previousPositions?.length!==layout.nodeCount||inertia.inertiaEdges?.length!==layout.nodeCount-1)
            throw new RangeError('Physical inertia needs previous positions and prepared input for every common edge');
        const ownGeometry=inertia.geometryByTool?.get(toolId),nodesFor=edge=>ownGeometry?.edges[edge].nodeIndices??[edge,edge+1],modeNodes=new Set(ownModes.map(mode=>mode.node)),
            incident=Array.from({length:layout.nodeCount-1},(_,edge)=>edge)
                .filter(edge=>spin[edge]>=0&&nodesFor(edge).some(node=>modeNodes.has(node)));
        for(const edge of incident) {
            const tools=inertia.inertiaEdges[edge]?.tools?.filter(t=>t.id===toolId);
            if(tools?.length!==1)throw new RangeError('Each incident inertia edge needs exactly one prepared relative material');
            const nodes=nodesFor(edge);
            descriptors.push({kind:'wire-inertia',edge,nodes,dofs:nodes.flatMap(positionDofs),inertiaTool:tools[0],geometry:ownGeometry?.edges[edge]});
            affectedInertiaEdges.push(edge);
        }
    }
    const dofs=Int32Array.from([...new Set(descriptors.flatMap(d=>d.dofs))].sort((a,b)=>a-b)),n=dofs.length,r=relativeCount;
    if(!n&&!empty)throw new RangeError('The cluster has no affected wire energy support');
    const localIndex=new Map(Array.from(dofs,(dof,index)=>[dof,index])),columnSets=Array.from({length:n},()=>new Set());
    let commonBand=1,relativeBand=1;
    for(const descriptor of descriptors) {
        descriptor.commonIndices=Int32Array.from(descriptor.dofs,dof=>localIndex.get(dof));
        descriptor.columns=ownModes.flatMap(mode=>descriptor.nodes.includes(mode.node)?
            mode.relativeDofs.map((relativeDof,axis)=>({relativeDof,at:3*descriptor.nodes.indexOf(mode.node),basis:mode.basis[axis]})):[]);
        for(const a of descriptor.commonIndices) {
            for(const b of descriptor.commonIndices)commonBand=Math.max(commonBand,Math.abs(a-b)+1);
            descriptor.columns.forEach(column=>columnSets[a].add(column.relativeDof));
        }
        for(const a of descriptor.columns)for(const b of descriptor.columns)relativeBand=Math.max(relativeBand,Math.abs(a.relativeDof-b.relativeDof)+1);
    }
    const rowOffsets=new Int32Array(n+1),columns=[];
    columnSets.forEach((set,row)=>{rowOffsets[row]=columns.length;columns.push(...[...set].sort((a,b)=>a-b));});rowOffsets[n]=columns.length;
    const columnIndices=Int32Array.from(columns),couplingIndex=columnSets.map((_,row)=>new Map(Array.from(columnIndices.subarray(rowOffsets[row],rowOffsets[row+1]),(column,j)=>[column,rowOffsets[row]+j])));
    const block=()=>({energy:0,common:{dofs,band:commonBand,gradient:new Float64Array(n),hessian:new Float64Array(n*commonBand)},
        relative:{dofCount:r,band:relativeBand,gradient:new Float64Array(r),hessian:new Float64Array(r*relativeBand)},
        coupling:{rowOffsets,columns:columnIndices,values:new Float64Array(columns.length),commonDofs:dofs,relativeDofCount:r}});
    const total=block(),additionalInertia=inertia===null?null:block();
    const fullRank=ownModes.every(mode=>mode.dimension===3),reduced=ownModes.every(mode=>mode.dimension===2);
    const operator={...total,toolId,modes:Object.freeze(ownModes),elementBackend,hessianKind:elementBackend==='wasm-exact'?'exact':'gauss-newton',
        elasticEnergy:0,inertialEnergy:0,additionalInertia,affectedHinges,affectedInertiaEdges,
        stencils:descriptors.map(d=>({kind:d.kind,vertex:d.vertex,edge:d.edge,commonDofs:Int32Array.from(d.dofs),commonIndices:d.commonIndices,
            relativeDofs:Int32Array.from(d.columns,column=>column.relativeDof)})),
        commonContributionRole:'diagnostic-only-already-in-common',scope:'joint-uncondensed-relative-linearization-at-zero-offset',
        relativeRepresentation:empty?'none-single-material':fullRank?'full-rank-on-represented-nodes':reduced?'transverse-reduced':'mixed-full-and-transverse-reduced',
        fullRankOnRepresentedNodes:!empty&&fullRank,operatorReady:false,hessianValid:false,
        certified:false,condensed:false,relativeDofCount:r};
    return {operator,descriptors,couplingIndex,commonBand,relativeBand,implementation,tool,spin,data,layout,inertia};
}

/** Compile ONLY frozen modes, affected physical stencils, bands/CSR and zeroed
 * numeric storage. No materialAt, Element or Kinematics evaluation occurs.
 * modes own contiguous variable-sized relativeDofs. Two-coordinate modes are
 * explicitly transverse reductions; three orthonormal vectors are a complete
 * fixed coordinate basis and need not be transverse to the current chord.
 * Full modes include common endpoints and material starts/tips, provided
 * both materials meet at the node and the relative material has an incident
 * edge. Reduced 2D modes keep their interior/two-sided support requirements.
 * All returned H/g bytes are scratch, NOT an assembled physical operator:
 * operatorReady and hessianValid remain false until the caller assembles it.
 * One actual material with explicit modes:[] produces an empty cluster:
 * zero relative DOFs, zero common support/energy and CSR offsets [0]. Its
 * physical energies remain exclusively in the common material assembly.
 */
export function createCompositeRelativeClusterStructure(args) {
    return prepareStructure(args).operator;
}

/** Joint UNCONDENSED linearization at rho=0 of wire positions q+B*rho.
 * Every affected hinge/inertia edge is evaluated ONCE. Bases stay fixed.
 * Three relative coordinates are full rank only on represented overlap nodes;
 * two coordinates remain an explicit reduction. No endpoint transfer, finite
 * rho assembly, independent length solve, nonlinear dt or CCD is performed.
 * Default GN and optional exact tangents preserve all spin/cross-mode terms.
 * common energy/g/H are DIAGNOSTICS already present in Chain, never additive.
 * Optional inertia includes FULL physical common/relative/coupling terms;
 * additionalInertia reports a subset, not an additional contribution to add.
 */
export function assembleCompositeRelativeCluster(args) {
    if(args.data?.elasticityGeometry==='continuous-material-frame')throw new RangeError('Continuous material elasticity must be evaluated by the physical joint assembly');
    const {operator:total,descriptors,couplingIndex,commonBand,relativeBand,implementation,tool,spin,data,layout,inertia}=prepareStructure(args);
    const {additionalInertia}=total;
    function add(target,response,descriptor) {
        const size=descriptor.dofs.length,H=response.hessian;
        finiteVector(response.gradient,size,'physical local gradient');finiteVector(H,size*size,'physical local Hessian');
        if(!Number.isFinite(response.energy))throw new RangeError('Finite physical local energy is required');
        target.energy+=response.energy;
        for(let row=0;row<size;row++) {
            const commonRow=descriptor.commonIndices[row];target.common.gradient[commonRow]+=response.gradient[row];
            for(let col=0;col<=row;col++)target.common.hessian[packed(commonRow,descriptor.commonIndices[col],commonBand)]+=H[row*size+col];
            for(const column of descriptor.columns) {
                let value=0;for(let k=0;k<3;k++)value+=H[row*size+column.at+k]*column.basis[k];
                target.coupling.values[couplingIndex[commonRow].get(column.relativeDof)]+=value;
            }
        }
        for(let a=0;a<descriptor.columns.length;a++) {
            const left=descriptor.columns[a],ri=left.relativeDof;
            for(let j=0;j<3;j++)target.relative.gradient[ri]+=left.basis[j]*response.gradient[left.at+j];
            for(let b=0;b<=a;b++) {
                const right=descriptor.columns[b];let value=0;
                for(let j=0;j<3;j++)for(let k=0;k<3;k++)value+=left.basis[j]*H[(left.at+j)*size+right.at+k]*right.basis[k];
                target.relative.hessian[packed(ri,right.relativeDof,relativeBand)]+=value;
            }
        }
    }
    const elementWorkspace=implementation.createCompositeElementWorkspace(1),inertiaWorkspace=inertia===null?null:createCompositeInertiaWorkspace(1);
    let elasticEnergy=0,inertialEnergy=0;
    for(const descriptor of descriptors) {
        let response;
        if(descriptor.kind==='wire-hinge') {
            const i=descriptor.vertex,x=data.coordinates;
            const material=tool.materialAt?tool.materialAt({vertex:i,coordinate:x[i],start:(x[i-1]+x[i])/2,end:(x[i]+x[i+1])/2}):tool.material;
            response=implementation.evaluateCompositeElement({positions:[data.positions[i-1],data.positions[i],data.positions[i+1]],
                reference:[data.reference[i-1],data.reference[i]],referenceLength:(x[i+1]-x[i-1])/2,
                tools:[{angles:[tool.angles[i-1],tool.angles[i]],referenceTwist:tool.referenceTwists?.[i-1]??0,
                    dsDx:typeof tool.dsDx==='function'?tool.dsDx(x[i]):tool.dsDx??1,material}]},elementWorkspace);
            elasticEnergy+=response.energy;
        } else {
            const e=descriptor.edge;
            response=descriptor.geometry?createCompositeContinuousMaterialInertiaEdge({geometry:descriptor.geometry,
                previousPositions:descriptor.nodes.map(node=>inertia.previousPositions[node]),dt:inertia.dt,tool:descriptor.inertiaTool})
                .evaluate(descriptor.nodes.map(node=>data.positions[node])):assembleCompositeTranslationalInertia({coordinates:[data.coordinates[e],data.coordinates[e+1]],
                positions:[data.positions[e],data.positions[e+1]],previousPositions:[inertia.previousPositions[e],inertia.previousPositions[e+1]],
                dt:inertia.dt,tools:[descriptor.inertiaTool]},inertiaWorkspace);
            add(additionalInertia,response,descriptor);inertialEnergy+=response.energy;
        }
        add(total,response,descriptor);
    }
    for(const target of [total,...(additionalInertia?[additionalInertia]:[])])
        if(!Number.isFinite(target.energy)||[target.common.gradient,target.common.hessian,target.relative.gradient,target.relative.hessian,target.coupling.values].some(v=>!v.every(Number.isFinite)))
            throw new RangeError('Nonfinite joint relative cluster operator');
    return Object.assign(total,{elasticEnergy,inertialEnergy,operatorReady:true,hessianValid:true});
}

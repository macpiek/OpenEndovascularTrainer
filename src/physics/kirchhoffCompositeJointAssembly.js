import * as gn from './kirchhoffCompositeElementFast.js';
import * as exact from './kirchhoffCompositeElementExact.js';
import * as oracle from './kirchhoffCompositeElement.js';
import { createCompositeRelativeClusterStructure } from './kirchhoffCompositeRelativeCluster.js';
import { createCompositeMaterialInertiaEdge } from './kirchhoffCompositeMaterialInertia.js';
import { createCompositePiecewiseMaterialInertiaEdge } from './kirchhoffCompositePiecewiseMaterialInertia.js';
import { createCompositeChainLayout } from './kirchhoffCompositeChain.js';
import { ownCompositeJointInertiaGeometry } from './kirchhoffCompositeJointGeometry.js';
import { createCompositeContinuousMaterialInertiaEdge } from './kirchhoffCompositeContinuousInertia.js';
import { createCompositeContinuousFrame,createCompositeContinuousFrameWorkspace } from './kirchhoffCompositeContinuousFrame.js';
import { createCompositeContinuousElasticEdge } from './kirchhoffCompositeContinuousElasticity.js';

const packed = (i,j,band) => Math.max(i,j)*band+Math.abs(i-j);
const vector = (v,n,name) => { if(v?.length!==n||!v.every(Number.isFinite))throw new RangeError(`${name} requires ${n} finite values`); };
const positive = (v,name) => { if(!(v>0)||!Number.isFinite(v))throw new RangeError(`${name} must be positive and finite`);return v; };
const ownFrame = frame => {
    vector(frame?.tangent,3,'Material reference tangent');vector(frame?.director,3,'Material reference director');
    return {tangent:Array.from(frame.tangent),director:Array.from(frame.director)};
};
const assemblyWorkspaces=new WeakMap();
function structureKey({layout,coordinates,modes,inertiaGeometryByTool=null,elasticityGeometry='native-discrete-rod',relativeToolId=layout?.spins?.size===1?layout.spins.keys().next().value:'wire',elementBackend='wasm-exact'},physicalInertia) {
    return JSON.stringify({edges:layout.edgeToolIds,positions:Array.from(layout.positions),spins:[...layout.spins].map(([id,v])=>[id,Array.from(v)]),
        hinges:layout.hinges.map(h=>[h.vertex,h.tools,Array.from(h.dofs)]),band:layout.band,dofs:layout.dofCount,n:layout.nodeCount,
        positionSupports:layout.positionSupports??[],materialSupports:layout.materialSupports??[],elasticityGeometry,inertiaGeometry:inertiaGeometryByTool===null?null:[...inertiaGeometryByTool].map(([id,g])=>[id,g.interfaces]),
        coordinates:Array.from(coordinates),modes:modes.map(m=>({node:m.node,basis:m.basis.map(b=>Array.from(b))})),relativeToolId,elementBackend,physicalInertia});
}

/** Reusable SYMBOLIC layout/scatter maps and numeric scratch, not a prepared
 * material state. Frames, winding, material tensors, maps, mass and velocities
 * are freshly owned on every createCompositeJointAssembly(args,{workspace}).
 * A later prepare (including a failed one) invalidates earlier borrowed
 * assembly handles. The physical-inertia presence, chart, modes/bases and
 * backend cannot change in this handle. No H or factor is cached as current.
 */
export function createCompositeJointAssemblyWorkspace({layout,coordinates,modes,inertiaGeometryByTool=null,elasticityGeometry='native-discrete-rod',relativeToolId=layout?.spins?.size===1?layout.spins.keys().next().value:'wire',elementBackend='wasm-exact',physicalInertia=true}) {
    if(!['wasm-exact','wasm','javascript'].includes(elementBackend)||typeof physicalInertia!=='boolean')throw new RangeError('Explicit backend and inertia presence are required');
    const geometry=ownCompositeJointInertiaGeometry({layout,coordinates,inertiaGeometryByTool,elasticityGeometry});
    if(inertiaGeometryByTool!==null){layout=geometry.layout;inertiaGeometryByTool=geometry.inertiaGeometryByTool;}
    const owned={layout:createCompositeChainLayout(layout.edgeToolIds,{positionSupports:layout.positionSupports??[],materialSupports:layout.materialSupports??[]}),inertiaGeometryByTool,elasticityGeometry,coordinates:Float64Array.from(coordinates),
        modes:modes.map(m=>({node:m.node,basis:m.basis.map(b=>Array.from(b))})),relativeToolId,elementBackend};
    if(structureKey(owned,physicalInertia)!==structureKey({layout,coordinates,modes,inertiaGeometryByTool,elasticityGeometry,relativeToolId,elementBackend},physicalInertia))throw new RangeError('Canonical common layout is required');
    const stats={prepares:0,structureBuilds:0,scatterPlansBuilt:0},cache={...owned,key:structureKey(owned,physicalInertia),stats,generation:0,busy:false,
        plans:new Map(),cluster:null,chain:null,local:null,toolPositions:null,output:null};
    const handle=Object.freeze({elementBackend,physicalInertia,nodeCount:layout.nodeCount,get diagnostics(){return {...stats,continuousFrames:cache.continuousFrames?.diagnostics??null};}});
    assemblyWorkspaces.set(handle,cache);return handle;
}
export function invalidateCompositeJointAssemblyWorkspace(handle) {
    const cache=assemblyWorkspaces.get(handle);if(!cache)throw new RangeError('Unknown joint assembly workspace');
    if(cache.busy)throw new RangeError('Joint assembly workspace is busy');
    invalidate(cache);
}
function invalidate(cache) {
    cache.generation++;
    for(const block of [cache.cluster,cache.chain,cache.output])if(block){block.hessianValid=false;block.operatorReady=false;}
}

/** Prepared, fixed-topology constitutive/inertial assembly in ONE coordinate
 * system: x_cat=q, x_wire=q+B*rho on explicitly represented overlap nodes;
 * exposed single-tool nodes use q. Each material retains its own time-reference
 * frames, winding, profile, feed map and old physical velocities/positions.
 * Every material hinge/edge is evaluated exactly once at its CURRENT physical
 * geometry. There is no rho=0 wire energy plus a second correction energy.
 * One material has explicit modes:[] and an empty relative cluster; its
 * physical hinges and inertia still use this same common assembly.
 *
 * This is a nonlinear operator, not a timestep solver. Lengths, clearance,
 * walls, friction, boundary conditions and acceptance belong to its caller.
 * Modes absent at an overlap node imply a common-axis reduction there; this
 * module does not certify that reduction or transfer moving material tips.
 * Explicit inertia:null permits elastic-only diagnostics; a physical dt must
 * supply {dt,previousPositions:Map<id,points>,inertiaEdges:[{tools}]}.
 */
export function createCompositeJointAssembly(args,{workspace=null}={}) {
    if(workspace===null)return prepareAssembly(args.inertiaGeometryByTool==null?args:{...args,...ownCompositeJointInertiaGeometry(args)},null,0);
    const cache=assemblyWorkspaces.get(workspace);if(!cache)throw new RangeError('Unknown joint assembly workspace');
    if(cache.busy)throw new RangeError('Joint assembly workspace is busy');
    invalidate(cache);cache.busy=true;
    try {
        if(args.inertiaGeometryByTool!=null)args={...args,...ownCompositeJointInertiaGeometry(args)};
        if(structureKey(args,args.inertia!==null)!==cache.key)throw new RangeError('Frozen joint chart/layout/bases/backend or inertia presence changed');
        cache.stats.prepares++;
        return prepareAssembly({...args,layout:cache.layout,coordinates:cache.coordinates,modes:cache.modes,inertiaGeometryByTool:cache.inertiaGeometryByTool},cache,cache.generation);
    } finally {cache.busy=false;}
}
function prepareAssembly({layout,coordinates,modes,tools,inertia,angles,inertiaGeometryByTool=null,elasticityGeometry='native-discrete-rod',relativeToolId=layout?.spins?.size===1?layout.spins.keys().next().value:'wire',elementBackend='wasm-exact'},cache,generation) {
    const implementation=elementBackend==='wasm-exact'?exact:elementBackend==='wasm'?gn:elementBackend==='javascript'?oracle:null;
    if(!implementation)throw new TypeError('Joint assembly backend must be wasm-exact, wasm or javascript');
    if(!['native-discrete-rod','continuous-material-frame'].includes(elasticityGeometry))throw new RangeError('Unknown elastic geometry');
    const continuousElasticity=elasticityGeometry==='continuous-material-frame';
    if(continuousElasticity&&(inertiaGeometryByTool===null||inertia===null||!(angles instanceof Map)||elementBackend!=='wasm-exact'))
        throw new RangeError('Continuous elasticity needs own continuous geometry, old positions/angles and an exact joint tangent');
    if(!layout||coordinates?.length!==layout.nodeCount||!Array.isArray(tools)||inertia===undefined)
        throw new RangeError('Fixed layout, coordinates, material data and explicit inertia (or null) are required');
    const x=cache?coordinates:Float64Array.from(coordinates),n=layout.nodeCount;
    x.forEach((v,i)=>{if(!Number.isFinite(v)||(i&&v<=x[i-1]))throw new RangeError('Common chart coordinates must increase');});
    const byId=new Map(tools.map(tool=>[tool.id,tool]));
    if(byId.size!==tools.length||tools.length!==layout.spins.size||[...layout.spins.keys()].some(id=>!byId.has(id)))
        throw new RangeError('Exactly one independent material record per tool is required');
    const reference=new Map(),anchors=new Map();
    for(const [id,tool] of byId) {
        if(tool.reference?.length!==n-1||tool.referenceTwists?.length!==n-2)
            throw new RangeError('Every material needs its own explicit reference frames and winding history');
        const active=layout.spins.get(id);
        reference.set(id,tool.reference.map((f,e)=>active[e]>=0?ownFrame(f):null));
        const winding=new Float64Array(n-2).fill(NaN);
        for(const h of layout.hinges)if(h.tools.includes(id)) {
            const value=tool.referenceTwists[h.vertex-1];if(!Number.isFinite(value))throw new RangeError('Finite active material winding history is required');
            winding[h.vertex-1]=value;
        }
        anchors.set(id,winding);
    }
    const toolPositions=cache?.toolPositions??new Map([...byId.keys()].map(id=>[id,Array.from({length:n},()=>[0,0,0])]));
    // The symbolic compiler requires a chart only to validate reduced 2D
    // bases. Complete 3D bases do not depend on current chord directions.
    if(!Array.isArray(modes)||modes.some(m=>m.basis?.length!==3)||(layout.spins.size===1&&modes.length!==0))
        throw new RangeError('Nonlinear joint assembly currently requires complete three-coordinate modes');
    if(inertia!==null&&(!(inertia.previousPositions instanceof Map)||inertia.inertiaEdges?.length!==n-1))
        throw new RangeError('Inertia needs independent old tool geometries and prepared material edges');
    const symbolicPositions=Array.from(x,v=>[v,0,0]);
    const cluster=cache?.cluster??createCompositeRelativeClusterStructure({layout,modes,toolId:relativeToolId,elementBackend,
        data:{positions:symbolicPositions,coordinates:x,reference:reference.get(relativeToolId),tools,elasticityGeometry},
        inertia:inertia===null?null:{...inertia,geometryByTool:inertiaGeometryByTool,previousPositions:inertia.previousPositions.get(relativeToolId)}});
    cluster.scope='nonlinear-common-relative-material-assembly';
    // The old zero-offset helper exposes a second diagnostic inertia block.
    // This assembly reports current physical inertia in perTool instead.
    cluster.additionalInertia=null;
    const modeByNode=cache?.modeByNode??new Map(cluster.modes.map(m=>[m.node,m]));
    const commonIndex=cache?.commonIndex??new Map(Array.from(cluster.common.dofs,(d,i)=>[d,i]));
    const couplingIndex=cache?.couplingIndex??new Map();
    if(!cache?.couplingIndex)for(let row=0;row<cluster.coupling.commonDofs.length;row++) {
        const columns=new Map();
        for(let i=cluster.coupling.rowOffsets[row];i<cluster.coupling.rowOffsets[row+1];i++)columns.set(cluster.coupling.columns[i],i);
        couplingIndex.set(cluster.coupling.commonDofs[row],columns);
    }
    const chain=cache?.chain??{layout,elementBackend,gradient:new Float64Array(layout.dofCount),hessian:new Float64Array(layout.dofCount*layout.band),
        energy:0,hessianValid:false,evaluationOrder:null};
    const local=cache?.local??[null,implementation.createCompositeElementWorkspace(1),null],elements=[],edges=[],continuousElements=[];
    if(cache&&!cache.cluster) {
        Object.assign(cache,{cluster,chain,local,toolPositions,modeByNode,commonIndex,couplingIndex});cache.stats.structureBuilds++;
    }
    function scatterPlan(id,nodes,dofs) {
        const key=`${id}:${nodes.join(',')}:${dofs.join(',')}`,cached=cache?.plans.get(key);if(cached)return cached;
        const columns=[];
        if(id===relativeToolId)for(let index=0;index<nodes.length;index++) {
            const mode=modeByNode.get(nodes[index]);if(mode)mode.basis.forEach((b,j)=>columns.push({at:3*index,basis:b,dof:mode.relativeDofs[j],
                terms:b.flatMap((value,k)=>value===0?[]:[3*index+k,value])}));
        }
        const size=dofs.length,commonSlots=new Int32Array(size*size),crossSlots=new Int32Array(size*columns.length);
        for(let i=0;i<size;i++)for(let j=0;j<=i;j++){
            if(Math.abs(dofs[i]-dofs[j])>=layout.band)throw new RangeError('Material contribution exceeds the common position band');
            commonSlots[size*i+j]=packed(dofs[i],dofs[j],layout.band);
        }
        for(let i=0;i<size;i++)for(let j=0;j<columns.length;j++) {
            const slot=couplingIndex.get(dofs[i])?.get(columns[j].dof);
            if(slot===undefined)throw new RangeError('Material contribution lies outside the symbolic relative support');
            crossSlots[columns.length*i+j]=slot;
        }
        // Frozen bases make this pullback a constant sparse map. Compile its
        // exact nonzero coefficients once; an identity basis requires one H
        // lookup per pair instead of nine multiplications on every trial.
        const relativePairs=[];
        for(let a=0;a<columns.length;a++)for(let b=0;b<=a;b++) {
            const left=columns[a],right=columns[b],terms=[];
            for(let i=0;i<left.terms.length;i+=2)for(let j=0;j<right.terms.length;j+=2)
                terms.push(size*left.terms[i]+right.terms[j],left.terms[i+1]*right.terms[j+1]);
            relativePairs.push({slot:packed(left.dof,right.dof,cluster.relative.band),terms});
        }
        const plan={dofs:Int32Array.from(dofs),size,columns,commonSlots,crossSlots,relativePairs,
            diagnosticIndices:columns.length?Int32Array.from(dofs,d=>commonIndex.get(d)):null};
        if(cache){cache.plans.set(key,plan);cache.stats.scatterPlansBuilt++;}return plan;
    }
    const positionDofs=node=>[layout.positions[node],layout.positions[node]+1,layout.positions[node]+2];
    for(const hinge of continuousElasticity?[]:layout.hinges)for(const id of hinge.tools) {
        const i=hinge.vertex,tool=byId.get(id),spin=layout.spins.get(id);
        const raw=tool.materialAt?tool.materialAt({vertex:i,coordinate:x[i],start:(x[i-1]+x[i])/2,end:(x[i]+x[i+1])/2}):tool.material;
        vector(raw?.stiffness,9,'Compiled material stiffness');vector(raw?.intrinsic,3,'Compiled material intrinsic strain');
        const material={stiffness:Float64Array.from(raw.stiffness),intrinsic:Float64Array.from(raw.intrinsic),energyOffset:raw.energyOffset??0};
        if(!Number.isFinite(material.energyOffset))throw new RangeError('Material energy offset must be finite');
        const sample={angles:[0,0],referenceTwist:anchors.get(id)[i-1],
            dsDx:positive(typeof tool.dsDx==='function'?tool.dsDx(x[i]):tool.dsDx,'Independent material dsDx'),material};
        const p=toolPositions.get(id);
        const input={positions:[p[i-1],p[i],p[i+1]],reference:[reference.get(id)[i-1],reference.get(id)[i]],
            referenceLength:(x[i+1]-x[i-1])/2,tools:[sample]};
        elements.push({ids:[id],vertex:i,input,samples:[sample],plan:scatterPlan(id,[i-1,i,i+1],[i-1,i,i+1].flatMap(positionDofs).concat([spin[i-1],spin[i]]))});
    }
    const materialHingeCount=elements.length;
    if(continuousElasticity) {
        const scratch=cache?.continuousFrames??createCompositeContinuousFrameWorkspace();if(cache)cache.continuousFrames=scratch;
        const materialLengths=new Map([...byId].map(([id,tool])=>{
            const active=layout.edgeToolIds.flatMap((ids,e)=>ids.includes(id)?[e]:[]);
            if(tool.materialBreaks!==undefined&&(!Array.isArray(tool.materialBreaks)||tool.materialBreaks.some(v=>!Number.isFinite(v))||new Set(tool.materialBreaks).size!==tool.materialBreaks.length))
                throw new RangeError('Material breaks must be distinct finite physical coordinates');
            return [id,x[active.at(-1)+1]-x[active[0]]];
        }));
        for(const support of layout.materialSupports) {
            const {toolId:id,edge}=support,tool=byId.get(id),geometry=inertiaGeometryByTool.get(id),
                fraction=(x[edge+1]-x[edge])/materialLengths.get(id),settings=tool.elasticQuadrature??{},
                frame=createCompositeContinuousFrame({geometry,edge,toolId:id,previousPositions:inertia.previousPositions.get(id),previousAngles:angles.get(id),reference:reference.get(id),referenceTwists:anchors.get(id)}),
                compiled=createCompositeContinuousElasticEdge({frame,dsDx:tool.dsDx,
                    ...(tool.materialAt?{materialAt:tool.materialAt}:{material:tool.material}),
                    materialBreaks:(tool.materialBreaks??[]).filter(at=>at>x[edge]&&at<x[edge+1]),workspace:scratch,
                    quadrature:{...settings,energy:(settings.energy??1e-9)*fraction,gradient:(settings.gradient??1e-8)*fraction,hessian:(settings.hessian??1e-7)*fraction}}),
                p=toolPositions.get(id),spin=layout.spins.get(id);
            continuousElements.push({id,frame,compiled,input:{positions:frame.positionNodeIndices.map(node=>p[node]),angles:new Array(frame.angleEdgeIndices.length).fill(0)},
                plan:scatterPlan(id,frame.positionNodeIndices,frame.positionNodeIndices.flatMap(positionDofs).concat(frame.angleEdgeIndices.map(edge=>spin[edge])))});
        }
    }
    // On already-declared common-axis spans the two constitutive laws share
    // one geometry calculation. This is exact only with identical frozen
    // reference frames. Different material histories keep separate elements;
    // neither proximity nor small rho is used to guess a reduction here.
    const sameFrames=(a,b)=>a.every((f,i)=>['tangent','director'].every(key=>f[key].every((v,k)=>v===b[i][key][k])));
    for(let i=0;i+1<elements.length;i++) {
        const a=elements[i],b=elements[i+1];
        if(a.vertex!==b.vertex||a.plan.columns.length||b.plan.columns.length||!sameFrames(a.input.reference,b.input.reference))continue;
        const samples=[...a.samples,...b.samples],ids=[...a.ids,...b.ids],otherId=ids.find(id=>id!==relativeToolId);
        const dofs=[...a.plan.dofs,...b.plan.dofs.slice(9)];
        elements.splice(i,2,{ids,vertex:a.vertex,samples,input:{...a.input,tools:samples},plan:scatterPlan(otherId,[a.vertex-1,a.vertex,a.vertex+1],dofs)});
        local[2]??=implementation.createCompositeElementWorkspace(2);
    }
    if(inertia!==null)for(let edge=0;edge<n-1;edge++) {
        const records=inertia.inertiaEdges[edge]?.tools,ids=layout.edgeToolIds[edge];
        if(!Array.isArray(records)||records.length!==ids.length||new Set(records.map(t=>t.id)).size!==ids.length||records.some(t=>!ids.includes(t.id)))
            throw new RangeError('Each physical material edge needs exactly one independent inertia record');
        for(const tool of records) {
            const previous=inertia.previousPositions.get(tool.id);
            if(previous?.length!==n)throw new RangeError('Explicit old geometry for every material is required');
            const continuous=inertiaGeometryByTool?.get(tool.id).edges[edge],nodes=continuous?.nodeIndices??[edge,edge+1],
                factory=continuous?createCompositeContinuousMaterialInertiaEdge:tool.oldVelocityPieces===undefined?createCompositeMaterialInertiaEdge:createCompositePiecewiseMaterialInertiaEdge;
            const compiled=factory({geometry:continuous,coordinates:[x[edge],x[edge+1]],previousPositions:nodes.map(node=>previous[node]),dt:inertia.dt,tool});
            const p=toolPositions.get(tool.id);
            edges.push({id:tool.id,edge,compiled,positions:nodes.map(node=>p[node]),plan:scatterPlan(tool.id,nodes,nodes.flatMap(positionDofs))});
        }
    }
    const perTool=cache?.output?.perTool??new Map([...byId.keys()].map(id=>[id,{elasticEnergy:0,inertialEnergy:0,mass:0,kineticEnergy:0,
        momentum:new Float64Array(3),oldMomentum:new Float64Array(3),momentumIncrement:new Float64Array(3)}]));
    const evaluatedReferenceTwists=cache?.output?.evaluatedReferenceTwists??new Map([...byId.keys()].map(id=>[id,new Float64Array(n-2).fill(NaN)]));
    const output=cache?.output??{chain,cluster,toolPositions,perTool,evaluatedReferenceTwists,energy:0,elasticEnergy:0,inertialEnergy:0,
        hessianValid:false,operatorReady:false,certified:false,includesAngularInertia:false,inertiaGeometry:inertiaGeometryByTool===null?'affine':'continuous-quintic',elasticityGeometry,
        scope:'fixed-topology-nonlinear-joint-material-operator',statistics:{materialHinges:materialHingeCount,sharedGeometryHinges:materialHingeCount-elements.length,
            materialInertiaEdges:edges.length,continuousElasticEdges:continuousElements.length,elementEvaluations:0,inertiaEvaluations:0}};
    output.statistics.sharedGeometryHinges=materialHingeCount-elements.length;
    if(cache)cache.output=output;
    const fullOptions={order:'full'},gradientOptions={order:'gradient'};
    function add(response,plan,full) {
        const {dofs,size,columns,commonSlots,crossSlots,diagnosticIndices}=plan,H=response.hessian,g=response.gradient;
        for(let row=0;row<size;row++) {
            chain.gradient[dofs[row]]+=g[row];
            if(full)for(let col=0;col<=row;col++)chain.hessian[commonSlots[size*row+col]]+=H[size*row+col];
            if(diagnosticIndices) {
                cluster.common.gradient[diagnosticIndices[row]]+=g[row];
                if(full)for(let col=0;col<=row;col++)cluster.common.hessian[packed(diagnosticIndices[row],diagnosticIndices[col],cluster.common.band)]+=H[size*row+col];
            }
            if(full)for(let c=0;c<columns.length;c++) {
                const column=columns[c];let value=0;
                for(let k=0;k<column.terms.length;k+=2)value+=H[size*row+column.terms[k]]*column.terms[k+1];
                cluster.coupling.values[crossSlots[columns.length*row+c]]+=value;
            }
        }
        for(let a=0;a<columns.length;a++) {
            const left=columns[a];
            for(let k=0;k<left.terms.length;k+=2)cluster.relative.gradient[left.dof]+=left.terms[k+1]*g[left.terms[k]];
        }
        if(full)for(const {slot,terms} of plan.relativePairs){let value=0;for(let k=0;k<terms.length;k+=2)value+=terms[k+1]*H[terms[k]];cluster.relative.hessian[slot]+=value;}
        if(columns.length)cluster.energy+=response.energy;
    }
    function evaluate({positions,relative,angles},{order='full'}={}) {
        if(cache&&(cache.generation!==generation||cache.busy))throw new RangeError('Stale or busy prepared joint assembly handle');
        output.hessianValid=output.operatorReady=chain.hessianValid=cluster.hessianValid=cluster.operatorReady=false;
        chain.evaluationOrder=order;
        if(!['full','gradient'].includes(order)||(order==='gradient'&&elementBackend!=='wasm-exact'))throw new TypeError('Gradient-only joint assembly requires wasm-exact');
        if(positions?.length!==n||!(angles instanceof Map))throw new RangeError('Current common positions and independent material angles are required');
        vector(relative,cluster.relative.dofCount,'Current relative coordinates');
        for(let i=0;i<n;i++) {
            vector(positions[i],3,'Current common position');
            for(const p of toolPositions.values())for(let k=0;k<3;k++)p[i][k]=positions[i][k];
        }
        const wire=toolPositions.get(relativeToolId);
        for(const m of cluster.modes)for(let a=0;a<3;a++)for(let k=0;k<3;k++)wire[m.node][k]+=m.basis[a][k]*relative[m.relativeDofs[a]];
        for(const [id,spins] of layout.spins) {
            const values=angles.get(id);if(values?.length!==n-1)throw new RangeError('Current independent spins must match their material edges');
            for(let e=0;e<n-1;e++)if(spins[e]>=0&&!Number.isFinite(values[e]))throw new RangeError('Finite current active material spins are required');
        }
        const full=order==='full',options=full?fullOptions:gradientOptions;
        chain.energy=output.energy=output.elasticEnergy=output.inertialEnergy=cluster.energy=cluster.elasticEnergy=cluster.inertialEnergy=0;
        for(const block of [chain,cluster.common,cluster.relative]){block.gradient.fill(0);if(full)block.hessian.fill(0);}
        if(full)cluster.coupling.values.fill(0);
        for(const p of perTool.values()){p.elasticEnergy=p.inertialEnergy=p.mass=p.kineticEnergy=0;p.momentum.fill(0);p.oldMomentum.fill(0);p.momentumIncrement.fill(0);}
        output.statistics.elementEvaluations=output.statistics.inertiaEvaluations=0;
        for(const e of elements) {
            for(let j=0;j<e.ids.length;j++){const a=angles.get(e.ids[j]);e.samples[j].angles[0]=a[e.vertex-1];e.samples[j].angles[1]=a[e.vertex];}
            const response=implementation.evaluateCompositeElement(e.input,local[e.ids.length],options);add(response,e.plan,full);
            for(let j=0;j<e.ids.length;j++) {
                evaluatedReferenceTwists.get(e.ids[j])[e.vertex-1]=response.referenceTwists[j];
                perTool.get(e.ids[j]).elasticEnergy+=response.toolEnergy[j];
            }
            output.elasticEnergy+=response.energy;
            if(e.plan.columns.length)cluster.elasticEnergy+=response.energy;
            output.statistics.elementEvaluations++;
        }
        for(const e of continuousElements) {
            const values=angles.get(e.id);e.frame.angleEdgeIndices.forEach((edge,j)=>e.input.angles[j]=values[edge]);
            const response=e.compiled.evaluate(e.input,options);add(response,e.plan,full);
            perTool.get(e.id).elasticEnergy+=response.energy;output.elasticEnergy+=response.energy;
            if(e.plan.columns.length)cluster.elasticEnergy+=response.energy;
            output.statistics.elementEvaluations++;
        }
        for(const e of edges) {
            const response=e.compiled.evaluate(e.positions,options),p=perTool.get(e.id);add(response,e.plan,full);
            output.inertialEnergy+=response.energy;p.inertialEnergy+=response.energy;p.mass+=response.mass;p.kineticEnergy+=response.kineticEnergy;
            if(e.plan.columns.length)cluster.inertialEnergy+=response.energy;
            for(let k=0;k<3;k++){p.momentum[k]+=response.tools[0].momentum[k];p.oldMomentum[k]+=response.tools[0].oldMomentum[k];p.momentumIncrement[k]+=response.momentumIncrement[k];}
            output.statistics.inertiaEvaluations++;
        }
        output.energy=chain.energy=output.elasticEnergy+output.inertialEnergy;
        if(!Number.isFinite(output.energy)||![chain.gradient,cluster.common.gradient,cluster.relative.gradient,
            ...(full?[chain.hessian,cluster.common.hessian,cluster.relative.hessian,cluster.coupling.values]:[])].every(v=>v.every(Number.isFinite)))
            throw new RangeError('Nonfinite original joint material operator');
        output.operatorReady=cluster.operatorReady=true;
        output.hessianValid=chain.hessianValid=cluster.hessianValid=full;
        return output;
    }
    return Object.freeze({layout,cluster,evaluate,scope:output.scope,includesPhysicalInertia:inertia!==null});
}

import {createCompositeChainLayout} from './kirchhoffCompositeChain.js';

const plans=new WeakMap();
const vector=(v,n,name)=>{if(v?.length!==n||!Array.from(v).every(Number.isFinite))throw new RangeError(`${name} needs ${n} finite entries`);};
const same=(a,b)=>a?.length===b.length&&b.every((v,i)=>v===a[i]);
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
const layoutKey=l=>JSON.stringify({edges:l.edgeToolIds,positions:Array.from(l.positions),spins:[...l.spins].map(([id,v])=>[id,Array.from(v)]),
    nodeCount:l.nodeCount,dofCount:l.dofCount,band:l.band,hinges:l.hinges.map(h=>[h.vertex,h.tools,Array.from(h.dofs)])});


const columnKey=c=>c?.kind==='position'?JSON.stringify(['position',c.toolId,c.node,c.component]):
    c?.kind==='angle'?JSON.stringify(['angle',c.toolId,c.edge]):null;
const toolKey=t=>JSON.stringify([t?.id,t?.edge,t?.edgeId]);
const finiteArray=n=>new Float64Array(n).fill(NaN);

/** Own physical coordinates y=T*z may visit the preceding/following edge.
 * configurationColumns fixes their exact order. Each currentTools edge MUST
 * contribute all six endpoint coordinates and its OWN scalar spin, even if
 * fixed by the caller's Dirichlet conditions. Extra visited coordinates may
 * have B=0 and nonzero finite G/DB; no such columns are dropped or condensed.
 *
 * All full 3D overlap modes are validated. The physical support, including
 * both endpoints of every listed angle, must fit at most two adjacent edges.
 * Only this constant chart map is prepared; no contact/transport law is here.
 */
export function createCompositeJointPhysicalColumnPullback({layout,modes,currentTools,configurationColumns,
    relativeToolId=layout?.spins?.size===1?layout.spins.keys().next().value:'wire'}) {
    if(!layout||!(layout.spins instanceof Map)||layout.nodeCount<3)throw new RangeError('A canonical joint layout with at least three nodes is required');
    const canonical=createCompositeChainLayout(layout.edgeToolIds);
    if(layoutKey(layout)!==layoutKey(canonical))throw new RangeError('Layout positions/spins/hinges must match actual material edges');
    if(!canonical.spins.has(relativeToolId))throw new RangeError('The relative material must exist in the chart');
    const overlap=Array.from({length:canonical.nodeCount},(_,node)=>node).filter(node=>new Set([
        ...(canonical.edgeToolIds[node-1]??[]),...(canonical.edgeToolIds[node]??[])]).size===2);
    if(!Array.isArray(modes)||modes.length!==overlap.length)throw new RangeError('Full ordered modes must cover every overlap node; one material requires modes:[]');
    const byNode=new Map();
    modes.forEach((m,i)=>{
        if(m.node!==overlap[i]||m.basis?.length!==3||m.relativeDofs?.length!==3||m.relativeDofs.some((d,j)=>d!==3*i+j))
            throw new RangeError('Complete ordered 3D modes need contiguous relative DOFs');
        const basis=m.basis.map(v=>{vector(v,3,'Frozen basis');return Array.from(v);});
        for(let a=0;a<3;a++)for(let b=0;b<=a;b++)if(Math.abs(dot(basis[a],basis[b])-(a===b?1:0))>1e-10)
            throw new RangeError('Frozen surface bases must be orthonormal');
        byNode.set(m.node,{basis,dofs:Array.from(m.relativeDofs)});
    });
    if(!Array.isArray(currentTools)||![1,2].includes(currentTools.length)||new Set(currentTools.map(t=>t.id)).size!==currentTools.length)
        throw new RangeError('One or two distinct actual current tools are required');
    const ownedTools=Object.freeze(currentTools.map(t=>{
        if(typeof t.id!=='string'||!Number.isInteger(t.edge)||t.edge<0||t.edge>=canonical.nodeCount-1||
            !canonical.edgeToolIds[t.edge].includes(t.id)||!(canonical.spins.get(t.id)?.[t.edge]>=0))
            throw new RangeError('Each current tool needs its actual material edge and own spin');
        if(typeof t.edgeId!=='string'||!t.edgeId)throw new RangeError('A current physical provider edge ID is required');
        return Object.freeze({id:t.id,edge:t.edge,edgeId:t.edgeId});
    }));
    const ids=new Set(ownedTools.map(t=>t.id)),keys=new Set(),supportNodes=[];
    if(!Array.isArray(configurationColumns)||configurationColumns.length===0)throw new RangeError('Explicit physical configuration columns are required');
    const ownedColumns=Object.freeze(configurationColumns.map(c=>{
        if(!ids.has(c?.toolId))throw new RangeError('A physical column must belong to an actual current tool');
        let value;
        if(c.kind==='position') {
            if(!Number.isInteger(c.node)||c.node<0||c.node>=canonical.nodeCount||!Number.isInteger(c.component)||c.component<0||c.component>2||
                ![...(canonical.edgeToolIds[c.node-1]??[]),...(canonical.edgeToolIds[c.node]??[])].includes(c.toolId))
                throw new RangeError('Position column needs an active own physical node and xyz component');
            value={kind:'position',toolId:c.toolId,node:c.node,component:c.component};supportNodes.push(c.node);
        } else if(c.kind==='angle') {
            if(!Number.isInteger(c.edge)||c.edge<0||c.edge>=canonical.nodeCount-1||!(canonical.spins.get(c.toolId)?.[c.edge]>=0))
                throw new RangeError('Angle column needs an active OWN material edge spin');
            value={kind:'angle',toolId:c.toolId,edge:c.edge};supportNodes.push(c.edge,c.edge+1);
        } else throw new RangeError('A physical column kind must be position or angle');
        const key=columnKey(value);if(keys.has(key))throw new RangeError('Duplicate physical configuration column');keys.add(key);return Object.freeze(value);
    }));
    for(const t of ownedTools) {
        for(const node of [t.edge,t.edge+1])for(let component=0;component<3;component++)
            if(!keys.has(columnKey({kind:'position',toolId:t.id,node,component})))throw new RangeError('Every current body endpoint column must be present, including fixed coordinates');
        if(!keys.has(columnKey({kind:'angle',toolId:t.id,edge:t.edge})))throw new RangeError('Every current body OWN spin column must be present');
    }
    const anchorNode=Math.min(...supportNodes);
    if(Math.max(...supportNodes)-anchorNode>2)throw new RangeError('Physical column union exceeds the local two-adjacent-edge stencil');
    const commonSet=new Set(),relativeSet=new Set();
    const physicalRows=ownedColumns.map(c=>{
        const common=c.kind==='angle'?canonical.spins.get(c.toolId)[c.edge]:canonical.positions[c.node]+c.component;
        commonSet.add(common);const relative=[];
        if(c.kind==='position'&&c.toolId===relativeToolId)byNode.get(c.node)?.basis.forEach((basis,a)=>{
            const weight=basis[c.component];if(weight!==0){const d=byNode.get(c.node).dofs[a];relativeSet.add(d);relative.push([d,weight]);}
        });
        return {common,relative};
    });
    const commonDofs=Int32Array.from([...commonSet].sort((a,b)=>a-b)),relativeDofs=Int32Array.from([...relativeSet].sort((a,b)=>a-b));
    const commonIndex=new Map(Array.from(commonDofs,(d,i)=>[d,i])),relativeIndex=new Map(Array.from(relativeDofs,(d,i)=>[d,commonDofs.length+i]));
    const physicalDofCount=ownedColumns.length,dofCount=commonDofs.length+relativeDofs.length,entries=Array.from({length:dofCount},()=>[]);
    physicalRows.forEach((r,physical)=>{entries[commonIndex.get(r.common)].push([physical,1]);for(const [d,weight] of r.relative)entries[relativeIndex.get(d)].push([physical,weight]);});
    const offsets=new Int32Array(dofCount+1);entries.forEach((v,i)=>offsets[i+1]=offsets[i]+v.length);
    const physicalIndices=new Int32Array(offsets[dofCount]),weights=new Float64Array(offsets[dofCount]);
    entries.forEach((v,i)=>v.forEach(([physical,weight],j)=>{physicalIndices[offsets[i]+j]=physical;weights[offsets[i]+j]=weight;}));
    const rows=[0,1].map(component=>({component,anchorNode,commonDofs,relativeDofs,unit:'mm',
        jacobian:finiteArray(dofCount),forceColumn:finiteArray(dofCount),forceDerivative:finiteArray(dofCount*dofCount),
        operatorReady:false,jacobianValid:false,forceColumnValid:false,forceDerivativeValid:false}));
    const toolLoads=ownedTools.map(t=>{
        const own=ownedColumns.filter(c=>c.toolId===t.id),nodes=Int32Array.from([...new Set(own.filter(c=>c.kind==='position').map(c=>c.node))].sort((a,b)=>a-b)),
            edges=Int32Array.from([...new Set(own.filter(c=>c.kind==='angle').map(c=>c.edge))].sort((a,b)=>a-b));
        return Object.freeze({...t,nodes,edges,nodalForces:Object.freeze(Array.from(nodes,()=>finiteArray(3))),spinTorques:finiteArray(edges.length)});
    });
    const loadScatter=ownedColumns.map(c=>{
        const tool=ownedTools.findIndex(t=>t.id===c.toolId),t=toolLoads[tool];
        return {tool,node:c.kind==='position'?Array.from(t.nodes).indexOf(c.node):-1,component:c.component,
            spin:c.kind==='angle'?Array.from(t.edges).indexOf(c.edge):-1};
    });
    const loads={valid:false,Ft:finiteArray(2),physical:finiteArray(physicalDofCount),common:finiteArray(commonDofs.length),relative:finiteArray(relativeDofs.length),tools:Object.freeze(toolLoads)};
    const workspace={anchorNode,currentTools:ownedTools,configurationColumns:ownedColumns,commonDofs,relativeDofs,physicalDofCount,dofCount,rows,loads,
        slipJacobian:finiteArray(2*dofCount),forceMap:finiteArray(2*dofCount),DforceMap:finiteArray(2*dofCount*dofCount),
        operatorReady:false,slipJacobianValid:false,forceMapValid:false,DforceMapValid:false,certified:false,
        scope:'fixed-own-physical-column-joint-pullback'};
    plans.set(workspace,{offsets,physicalIndices,weights,physicalDofCount,dofCount,commonDofs:commonDofs.slice(),relativeDofs:relativeDofs.slice(),
        currentTools:ownedTools,configurationColumns:ownedColumns,toolKeys:ownedTools.map(toolKey),columnKeys:ownedColumns.map(columnKey),
        loadScatter,loadTools:toolLoads.map(t=>({id:t.id,nodes:t.nodes.slice(),edges:t.edges.slice()})),
        physicalForceMap:new Float64Array(2*physicalDofCount),forceReady:false,busy:false});
    return workspace;
}
function planFor(w) {const p=plans.get(w);if(!p)throw new TypeError('Use a prepared physical-column pullback workspace');if(p.busy)throw new RangeError('Physical-column pullback is busy');return p;}
function invalidateLoads(loads) {
    loads.valid=false;for(const v of [loads.Ft,loads.physical,loads.common,loads.relative])v.fill(NaN);
    for(const t of loads.tools){t.spinTorques.fill(NaN);t.nodalForces.forEach(v=>v.fill(NaN));}
}
function invalidate(w,p) {
    p.forceReady=false;w.operatorReady=w.slipJacobianValid=w.forceMapValid=w.DforceMapValid=false;
    for(const v of [w.slipJacobian,w.forceMap,w.DforceMap])v.fill(NaN);
    for(const row of w.rows){row.operatorReady=row.jacobianValid=row.forceColumnValid=row.forceDerivativeValid=false;row.jacobian.fill(NaN);row.forceColumn.fill(NaN);row.forceDerivative.fill(NaN);}
    invalidateLoads(w.loads);
}
function verifySupport(w,p) {
    if(!same(w.commonDofs,p.commonDofs)||!same(w.relativeDofs,p.relativeDofs)||w.currentTools!==p.currentTools||w.configurationColumns!==p.configurationColumns||
        w.loads.tools.length!==p.loadTools.length||w.loads.tools.some((t,i)=>t.id!==p.loadTools[i].id||!same(t.nodes,p.loadTools[i].nodes)||!same(t.edges,p.loadTools[i].edges)))
        throw new RangeError('Prepared physical-column support was modified');
}

/** Gphysical[2*N], Bphysical[N*2], DBphysical[N*2*N] already include ALL
 * producer geometry/history derivatives in the exact listed physical order.
 * DB[(physicalRow*2+component)*N+configurationColumn] need not be symmetric.
 * We compute G*T, T^T*B and T^T*DB*T exactly, with no force-support pruning.
 * Optional invalid derivatives MUST be omitted. Full operatorReady requires
 * both G and DB; their individual validity remains independent. Force-only
 * value input publishes true B/loads and invalidates absent derivatives.
 */
export function pullbackCompositeJointPhysicalColumns(input,workspace) {
    const p=planFor(workspace);p.busy=true;invalidate(workspace,p);
    try {
        verifySupport(workspace,p);
        const {currentTools,configurationColumns,slipJacobian,slipJacobianValid,forceMap,forceMapValid,DforceMap,DforceMapValid}=input??{};
        if(!Array.isArray(currentTools)||!same(currentTools.map(toolKey),p.toolKeys)||!Array.isArray(configurationColumns)||!same(configurationColumns.map(columnKey),p.columnKeys))
            throw new RangeError('Physical producer tool/edge identity and configuration column order must match the prepared mapping');
        const {physicalDofCount:N,dofCount:M,offsets,physicalIndices:ix,weights:a}=p;
        if(forceMapValid!==true)throw new RangeError('A current valid physical forceMap is required');vector(forceMap,2*N,'Physical forceMap');
        const hasG=slipJacobian!==undefined&&slipJacobian!==null,hasDB=DforceMap!==undefined&&DforceMap!==null;
        if(hasG){if(slipJacobianValid!==true)throw new RangeError('Finite slipJacobian validity is required');vector(slipJacobian,2*N,'Physical finite slipJacobian');}
        else if(slipJacobianValid===true)throw new RangeError('Missing valid finite slipJacobian');
        if(hasDB){if(DforceMapValid!==true)throw new RangeError('Physical DforceMap validity is required');vector(DforceMap,2*N*N,'Physical DforceMap');}
        else if(DforceMapValid===true)throw new RangeError('Missing valid physical DforceMap');
        for(let i=0;i<M;i++)for(let c=0;c<2;c++) {
            let g=0,b=0;for(let k=offsets[i];k<offsets[i+1];k++){if(hasG)g+=a[k]*slipJacobian[c*N+ix[k]];b+=a[k]*forceMap[2*ix[k]+c];}
            workspace.forceMap[2*i+c]=b;workspace.rows[c].forceColumn[i]=-b;
            if(hasG)workspace.slipJacobian[c*M+i]=workspace.rows[c].jacobian[i]=g;
            if(hasDB)for(let j=0;j<M;j++) {
                let d=0;for(let k=offsets[i];k<offsets[i+1];k++)for(let l=offsets[j];l<offsets[j+1];l++)d+=a[k]*DforceMap[(2*ix[k]+c)*N+ix[l]]*a[l];
                workspace.DforceMap[(2*i+c)*M+j]=workspace.rows[c].forceDerivative[i*M+j]=d;
            }
        }
        vector(workspace.forceMap,2*M,'Mapped physical forceMap');
        if(hasG)vector(workspace.slipJacobian,2*M,'Mapped finite slipJacobian');if(hasDB)vector(workspace.DforceMap,2*M*M,'Mapped physical DforceMap');
        p.physicalForceMap.set(forceMap);p.forceReady=workspace.forceMapValid=true;
        workspace.slipJacobianValid=hasG;workspace.DforceMapValid=hasDB;workspace.operatorReady=hasG&&hasDB;
        for(const row of workspace.rows){row.operatorReady=hasG&&hasDB;row.jacobianValid=hasG;row.forceColumnValid=true;row.forceDerivativeValid=hasDB;}
        return workspace;
    } catch(error){invalidate(workspace,p);throw error;} finally{p.busy=false;}
}

/** Exact signed physical N-vector and local common/relative loads. Every
 * listed physical coordinate is scattered to its actual node or OWN edge
 * spin. Zero B on history-only coordinates stays zero; nonzero B is never
 * silently dropped. No Dirichlet elimination, wall reaction removal, Coulomb
 * law, finite power approximation or angular-history inference occurs here.
 * The PRIVATE current B snapshot owns load validity, not public output bytes.
 */
export function evaluateCompositeJointPhysicalColumnLoads(Ft,workspace) {
    const p=planFor(workspace);p.busy=true;const loads=workspace.loads,length=Ft?.length,f0=Ft?.[0],f1=Ft?.[1];invalidateLoads(loads);
    try {
        verifySupport(workspace,p);if(!p.forceReady||workspace.forceMapValid!==true)throw new RangeError('Loads require a current valid physical forceMap');
        if(length!==2||!Number.isFinite(f0)||!Number.isFinite(f1))throw new RangeError('Signed traction needs two finite entries');loads.Ft.set([f0,f1]);
        for(let i=0;i<p.physicalDofCount;i++)loads.physical[i]=p.physicalForceMap[2*i]*f0+p.physicalForceMap[2*i+1]*f1;
        for(let i=0;i<p.dofCount;i++) {
            let value=0;for(let k=p.offsets[i];k<p.offsets[i+1];k++)value+=p.weights[k]*loads.physical[p.physicalIndices[k]];
            if(i<loads.common.length)loads.common[i]=value;else loads.relative[i-loads.common.length]=value;
        }
        for(const v of [loads.physical,loads.common,loads.relative])if(!v.every(Number.isFinite))throw new RangeError('Nonfinite mapped physical load');
        for(const t of loads.tools){t.nodalForces.forEach(v=>v.fill(0));t.spinTorques.fill(0);}
        p.loadScatter.forEach((s,i)=>{const t=loads.tools[s.tool];if(s.node>=0)t.nodalForces[s.node][s.component]=loads.physical[i];else t.spinTorques[s.spin]=loads.physical[i];});
        loads.valid=true;return loads;
    } catch(error){invalidateLoads(loads);throw error;} finally{p.busy=false;}
}

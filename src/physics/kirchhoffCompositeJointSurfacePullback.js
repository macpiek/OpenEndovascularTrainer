import {createCompositeChainLayout} from './kirchhoffCompositeChain.js';

const plans=new WeakMap();
const vector=(v,n,name)=>{if(v?.length!==n||!Array.from(v).every(Number.isFinite))throw new RangeError(`${name} needs ${n} finite entries`);};
const same=(a,b)=>a?.length===b.length&&b.every((v,i)=>v===a[i]);
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
const layoutKey=l=>JSON.stringify({edges:l.edgeToolIds,positions:Array.from(l.positions),spins:[...l.spins].map(([id,v])=>[id,Array.from(v)]),
    nodeCount:l.nodeCount,dofCount:l.dofCount,band:l.band,hinges:l.hinges.map(h=>[h.vertex,h.tools,Array.from(h.dofs)])});

/** Constant local map y=T*z. Physical y is [qA.xyz,qB.xyz,theta] per
 * declared tool, in tools order; z is commonDofs followed by relativeDofs.
 * Each {id,edge,edgeId} binds a physical provider edge ID to its actual
 * layout edge. Only the relative material's positions use q+B*rho. Every
 * theta maps to its OWN layout.spins.get(id)[edge], without interpolation.
 *
 * One tool is a tool/wall contact (also valid within a two-material chart).
 * Two tools must be distinct. Full 3D modes cover ALL chart overlap nodes;
 * a one-material chart has modes:[]. Support must fit two adjacent edges.
 * The compiled chart/bases/edge identities are owned, frozen snapshots;
 * rebuild this mapping when they change. The factory validates/copies the
 * full chart once and shares compiled columns for equal ordered tool edges;
 * each invocation builds independent local numeric storage. Provider edge
 * identities are rebound on every invocation, never taken from that cache.
 * matches() checks the actual layout and basis values before sequential reuse.
 * Geometry, material maps, spins, forces and history are never cached here.
 * Local numeric arrays are read-only scratch, overwritten/invalidated by the
 * next pullback call on that particular mapping.
 */
export function createCompositeJointSurfacePullbackFactory({layout,modes,
    relativeToolId=layout?.spins?.size===1?layout.spins.keys().next().value:'wire',supportPolicy='local-two-edge'}) {
    if(!layout||!(layout.spins instanceof Map)||layout.nodeCount<3)throw new RangeError('A canonical joint layout with at least three nodes is required');
    if(!['local-two-edge','declared-tool-pair'].includes(supportPolicy))throw new RangeError('Unknown surface support policy');
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
    const key=surfaceChartKey({layout,modes,relativeToolId,supportPolicy});
    const context={canonical,byNode,relativeToolId,supportPolicy},cache=new Map(),capacity=4*canonical.nodeCount;
    let builds=0,hits=0;
    const factory=tools=>{
        const ownedTools=surfaceTools(canonical,tools),supportKey=JSON.stringify(ownedTools.map(t=>[t.id,t.edge]));
        let plan=cache.get(supportKey);
        if(plan){hits++;cache.delete(supportKey);}
        else {plan=compileSurfaceMapping(context,ownedTools);builds++;}
        cache.set(supportKey,plan);if(cache.size>capacity)cache.delete(cache.keys().next().value);
        return createSurfaceMapping(plan,ownedTools);
    };
    factory.matches=args=>surfaceChartKey(args)===key;
    Object.defineProperty(factory,'diagnostics',{get:()=>({builds,hits,retainedPlans:cache.size,capacity})});
    return Object.freeze(factory);
}
function surfaceChartKey({layout,modes,relativeToolId=layout?.spins?.size===1?layout.spins.keys().next().value:'wire',supportPolicy='local-two-edge'}) {
    return JSON.stringify({supportPolicy,layout:layoutKey(layout),relativeToolId,modes:modes.map(m=>[m.node,m.basis.map(v=>Array.from(v)),Array.from(m.relativeDofs)])});
}
export function createCompositeJointSurfacePullback(args) {
    return createCompositeJointSurfacePullbackFactory(args)(args.tools);
}
function surfaceTools(canonical,tools) {
    if(!Array.isArray(tools)||![1,2].includes(tools.length)||new Set(tools.map(t=>t.id)).size!==tools.length)
        throw new RangeError('One actual tool against a wall or two distinct actual tools are required');
    return Object.freeze(tools.map(t=>{
        if(typeof t.id!=='string'||!Number.isInteger(t.edge)||t.edge<0||t.edge>=canonical.nodeCount-1||
            !canonical.edgeToolIds[t.edge].includes(t.id)||!(canonical.spins.get(t.id)?.[t.edge]>=0))
            throw new RangeError('Each physical surface needs its actual material edge and own spin DOF');
        if(typeof t.edgeId!=='string'||!t.edgeId)throw new RangeError('An explicit physical provider edge ID is required');
        return Object.freeze({id:t.id,edge:t.edge,edgeId:t.edgeId});
    }));
}
function compileSurfaceMapping({canonical,byNode,relativeToolId,supportPolicy},ownedTools) {
    const nodes=ownedTools.flatMap(t=>[t.edge,t.edge+1]),anchorNode=Math.min(...nodes);
    if(Math.max(...nodes)-anchorNode>2&&!(supportPolicy==='declared-tool-pair'&&ownedTools.length===2))throw new RangeError('Surface support exceeds the local two-edge stencil; rebuild the chart');
    const commonDofs=Int32Array.from([...new Set(ownedTools.flatMap(t=>[
        ...[t.edge,t.edge+1].flatMap(node=>[canonical.positions[node],canonical.positions[node]+1,canonical.positions[node]+2]),canonical.spins.get(t.id)[t.edge]]))].sort((a,b)=>a-b));
    const relativeDofs=Int32Array.from([...new Set(ownedTools.flatMap(t=>t.id===relativeToolId?
        [t.edge,t.edge+1].flatMap(node=>byNode.get(node)?.dofs??[]):[]))].sort((a,b)=>a-b));
    const commonIndex=new Map(Array.from(commonDofs,(d,i)=>[d,i])),relativeIndex=new Map(Array.from(relativeDofs,(d,i)=>[d,commonDofs.length+i]));
    const physicalDofCount=7*ownedTools.length,dofCount=commonDofs.length+relativeDofs.length,columns=Array.from({length:dofCount},()=>[]);
    ownedTools.forEach((t,i)=>{
        for(let end=0;end<2;end++) {
            const node=t.edge+end;
            for(let axis=0;axis<3;axis++)columns[commonIndex.get(canonical.positions[node]+axis)].push([7*i+3*end+axis,1]);
            if(t.id===relativeToolId)byNode.get(node)?.basis.forEach((b,a)=>b.forEach((v,k)=>{
                if(v!==0)columns[relativeIndex.get(byNode.get(node).dofs[a])].push([7*i+3*end+k,v]);
            }));
        }
        columns[commonIndex.get(canonical.spins.get(t.id)[t.edge])].push([7*i+6,1]);
    });
    return {anchorNode,commonDofs,relativeDofs,physicalDofCount,dofCount,columns};
}
function createSurfaceMapping(compiled,ownedTools) {
    const {anchorNode,physicalDofCount,dofCount}=compiled,commonDofs=compiled.commonDofs.slice(),relativeDofs=compiled.relativeDofs.slice();
    const rows=[0,1].map(component=>({component,anchorNode,commonDofs,relativeDofs,unit:'mm',
        jacobian:new Float64Array(dofCount).fill(NaN),forceColumn:new Float64Array(dofCount).fill(NaN),
        forceDerivative:new Float64Array(dofCount*dofCount).fill(NaN),
        operatorReady:false,jacobianValid:false,forceColumnValid:false,forceDerivativeValid:false}));
    const loads={valid:false,Ft:new Float64Array(2).fill(NaN),physical:new Float64Array(physicalDofCount).fill(NaN),
        common:new Float64Array(commonDofs.length).fill(NaN),relative:new Float64Array(relativeDofs.length).fill(NaN),
        tools:ownedTools.map(t=>({...t,nodes:Int32Array.of(t.edge,t.edge+1),nodalForces:[new Float64Array(3).fill(NaN),new Float64Array(3).fill(NaN)],scalarTorque:NaN}))};
    const workspace={anchorNode,tools:ownedTools,commonDofs,relativeDofs,physicalDofCount,dofCount,rows,loads,
        slipJacobian:new Float64Array(2*dofCount).fill(NaN),forceMap:new Float64Array(2*dofCount).fill(NaN),
        DforceMap:new Float64Array(2*dofCount*dofCount).fill(NaN),
        operatorReady:false,slipJacobianValid:false,forceMapValid:false,DforceMapValid:false,certified:false,
        scope:'fixed-affine-joint-surface-pullback'};
    plans.set(workspace,{...compiled,tools:ownedTools,
        physicalForceMap:new Float64Array(2*physicalDofCount),forceReady:false,busy:false});
    return workspace;
}
function planFor(workspace) {
    const plan=plans.get(workspace);if(!plan)throw new TypeError('Use a prepared joint surface pullback workspace');
    if(plan.busy)throw new RangeError('Joint surface pullback is busy');return plan;
}
function invalidateLoads(loads) {
    loads.valid=false;for(const v of [loads.Ft,loads.physical,loads.common,loads.relative])v.fill(NaN);
    for(const t of loads.tools){t.scalarTorque=NaN;t.nodalForces.forEach(v=>v.fill(NaN));}
}
function invalidate(workspace,plan) {
    plan.forceReady=false;workspace.operatorReady=workspace.slipJacobianValid=workspace.forceMapValid=workspace.DforceMapValid=false;
    for(const v of [workspace.slipJacobian,workspace.forceMap,workspace.DforceMap])v.fill(NaN);
    for(const row of workspace.rows){row.operatorReady=row.jacobianValid=row.forceColumnValid=row.forceDerivativeValid=false;
        row.jacobian.fill(NaN);row.forceColumn.fill(NaN);row.forceDerivative.fill(NaN);}
    invalidateLoads(workspace.loads);
}
function verifySupport(workspace,plan) {
    if(!same(workspace.commonDofs,plan.commonDofs)||!same(workspace.relativeDofs,plan.relativeDofs))
        throw new RangeError('Prepared joint surface support was modified');
}

/** Input operators are already composed with ALL physical geometry chain
 * rules by their provider. Their independent validity flags must be true.
 * slipJacobian is the finite-slip derivative G (2 x physicalDofCount), NOT
 * substituted from instantaneous B. It may be omitted for force/power-only
 * use; then rows/operatorReady remain false. forceMap B is mandatory.
 * Optional DB index: ((physicalRow*2+component)*physicalDofCount+configCol).
 * Output uses the same packing with joint dofCount. No detector derivatives,
 * numeric FD, constitutive law, traction sign selection or solver lives here.
 *
 * Gjoint=G*T; Bjoint=T^T*B; DBjoint=T^T*DB*T. forceMap and row.forceDerivative
 * are positive physical maps. row.forceColumn=-B follows the mechanical
 * residual convention internal - applied. No Ft-weighted Hessian is claimed.
 * Any failed refresh invalidates every old map/row/load before rethrowing.
 */
export function pullbackCompositeJointSurface(input,workspace) {
    const plan=planFor(workspace);plan.busy=true;invalidate(workspace,plan);
    try {
        verifySupport(workspace,plan);
        const {tools,slipJacobian,slipJacobianValid,forceMap,forceMapValid,DforceMap,DforceMapValid}=input;
        const {physicalDofCount:p,dofCount:n,columns}=plan;
        if(!Array.isArray(tools)||tools.length!==plan.tools.length||tools.some((t,i)=>t.id!==plan.tools[i].id||t.edgeId!==plan.tools[i].edgeId||
            (t.edge!==undefined&&t.edge!==plan.tools[i].edge)))throw new RangeError('Physical operator tool/edge order must match the prepared mapping');
        if(forceMapValid!==true)throw new RangeError('A current valid instantaneous physical forceMap is required');
        vector(forceMap,2*p,'Physical instantaneous forceMap');
        const hasG=slipJacobian!==undefined&&slipJacobian!==null,hasDB=DforceMap!==undefined&&DforceMap!==null;
        if(hasG){if(slipJacobianValid!==true)throw new RangeError('Finite slipJacobian validity is required');vector(slipJacobian,2*p,'Physical finite slipJacobian');}
        else if(slipJacobianValid===true)throw new RangeError('Missing valid finite slipJacobian');
        if(hasDB){if(DforceMapValid!==true)throw new RangeError('Physical DforceMap validity is required');vector(DforceMap,2*p*p,'Physical DforceMap');}
        else if(DforceMapValid===true)throw new RangeError('Missing valid physical DforceMap');
        for(let i=0;i<n;i++)for(let component=0;component<2;component++) {
            let g=0,b=0;for(const [row,weight] of columns[i]){if(hasG)g+=weight*slipJacobian[component*p+row];b+=weight*forceMap[2*row+component];}
            workspace.forceMap[2*i+component]=b;workspace.rows[component].forceColumn[i]=-b;
            if(hasG)workspace.slipJacobian[component*n+i]=workspace.rows[component].jacobian[i]=g;
            if(hasDB)for(let j=0;j<n;j++) {
                let d=0;for(const [row,wr] of columns[i])for(const [col,wc] of columns[j])d+=wr*DforceMap[(2*row+component)*p+col]*wc;
                workspace.DforceMap[(2*i+component)*n+j]=workspace.rows[component].forceDerivative[i*n+j]=d;
            }
        }
        vector(workspace.forceMap,2*n,'Mapped instantaneous forceMap');
        if(hasG)vector(workspace.slipJacobian,2*n,'Mapped finite slipJacobian');
        if(hasDB)vector(workspace.DforceMap,2*n*n,'Mapped DforceMap');
        plan.physicalForceMap.set(forceMap);plan.forceReady=workspace.forceMapValid=true;
        workspace.operatorReady=workspace.slipJacobianValid=hasG;workspace.DforceMapValid=hasDB;
        for(const row of workspace.rows){row.operatorReady=row.jacobianValid=hasG;row.forceColumnValid=true;row.forceDerivativeValid=hasDB;}
        return workspace;
    } catch(error){invalidate(workspace,plan);throw error;} finally{plan.busy=false;}
}

/** Physical load for the caller's signed Ft, with no Coulomb policy/clipping.
 * common/relative are in the workspace's local support order. tools reports
 * each actual edge's two nodal forces and OWN scalar spin torque; no fictitious
 * wall body/torque is created. Prescribed wall/feed power belongs to the motion
 * provider. The returned loads are scratch invalidated by the next map/load.
 */
export function evaluateCompositeJointSurfaceLoads(Ft,workspace) {
    const plan=planFor(workspace);plan.busy=true;const loads=workspace.loads;invalidateLoads(loads);
    try {
        verifySupport(workspace,plan);
        if(!plan.forceReady||workspace.forceMapValid!==true)throw new RangeError('Loads require a current valid mapped physical forceMap');
        vector(Ft,2,'Signed tangential traction');loads.Ft.set(Ft);
        for(let i=0;i<plan.physicalDofCount;i++)loads.physical[i]=plan.physicalForceMap[2*i]*Ft[0]+plan.physicalForceMap[2*i+1]*Ft[1];
        for(let i=0;i<plan.dofCount;i++) {
            let value=0;for(const [row,weight] of plan.columns[i])value+=weight*loads.physical[row];
            if(i<loads.common.length)loads.common[i]=value;else loads.relative[i-loads.common.length]=value;
        }
        for(const v of [loads.physical,loads.common,loads.relative])if(!v.every(Number.isFinite))throw new RangeError('Nonfinite mapped tangential load');
        loads.tools.forEach((t,i)=>{t.nodalForces[0].set(loads.physical.subarray(7*i,7*i+3));t.nodalForces[1].set(loads.physical.subarray(7*i+3,7*i+6));t.scalarTorque=loads.physical[7*i+6];});
        loads.valid=true;return loads;
    } catch(error){invalidateLoads(loads);throw error;} finally{plan.busy=false;}
}

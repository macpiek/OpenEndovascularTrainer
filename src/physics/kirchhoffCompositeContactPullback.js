const plans = new WeakMap();
const finiteVector = (v,n,name) => { if(v?.length!==n||!v.every(Number.isFinite))throw new RangeError(`${name} requires ${n} finite entries`); };

/** Frozen local map from physical tool endpoint coordinates to common q and
 * relative rho. `points` gives the EXACT physical ordering of a geometry
 * operator, e.g. [wire0,wire1,cat0,cat1], each {toolId,node}. Duplicate common
 * q coordinates are summed, never treated as independent copies. Frames are
 * frozen affine bases; no basis derivative or contact search occurs here.
 * The support must fit the joint solver's existing two-edge local stencil.
 * An omitted relative mode remains an explicit common-axis reduction whose
 * admissibility is the caller's responsibility.
 * The factory validates/copies the chart once and builds independent local
 * numeric workspaces for each points list. Compiled columns/term pairs are
 * shared by equal ordered supports; mutable outputs are always independent.
 * matches() compares only the frozen
 * coordinate map; positions, contact identities, radii and forces are not cached.
 */
export function createCompositeContactPullbackFactory({layout,modes,relativeToolId='wire',supportPolicy='local-two-edge'}) {
    if(!layout||!Array.isArray(modes))
        throw new RangeError('A fixed layout, explicit modes and ordered physical points are required');
    if(!['local-two-edge','declared-tool-pair'].includes(supportPolicy))throw new RangeError('Unknown contact support policy');
    const hasTool=(id,node)=>[node-1,node].some(edge=>edge>=0&&edge<layout.nodeCount-1&&layout.edgeToolIds[edge].includes(id));
    const byNode=new Map();let relativeCount=0,previousNode=-1;
    for(const m of modes) {
        const dimension=m.basis?.length;
        if(!Number.isInteger(m.node)||m.node<=previousNode||m.node>=layout.nodeCount||![2,3].includes(dimension)||
            m.relativeDofs?.length!==dimension||m.relativeDofs.some((d,i)=>d!==relativeCount+i)||!hasTool(relativeToolId,m.node))
            throw new RangeError('Ordered relative modes must belong to their material with contiguous DOFs');
        const basis=m.basis.map(b=>{finiteVector(b,3,'Frozen basis');return Array.from(b);});
        for(let a=0;a<dimension;a++)for(let b=0;b<=a;b++)if(Math.abs(basis[a].reduce((sum,v,k)=>sum+v*basis[b][k],0)-(a===b?1:0))>1e-10)
            throw new RangeError('Frozen contact bases must be orthonormal');
        byNode.set(m.node,{basis,dofs:Array.from(m.relativeDofs)});relativeCount+=dimension;previousNode=m.node;
    }
    const ownedLayout={nodeCount:layout.nodeCount,positions:Int32Array.from(layout.positions),edgeToolIds:layout.edgeToolIds.map(ids=>ids.slice())};
    const key=contactChartKey({layout,modes,relativeToolId,supportPolicy});
    const context={layout:ownedLayout,byNode,relativeToolId,supportPolicy},cache=new Map(),capacity=4*layout.nodeCount;
    let builds=0,hits=0;
    const factory=points=>{
        const ownedPoints=contactPoints(ownedLayout,points),supportKey=JSON.stringify(ownedPoints);
        let plan=cache.get(supportKey);
        if(plan){hits++;cache.delete(supportKey);}
        else {plan=compileContactMapping(context,ownedPoints);builds++;}
        cache.set(supportKey,plan);if(cache.size>capacity)cache.delete(cache.keys().next().value);
        return createContactMapping(plan,ownedPoints);
    };
    factory.matches=args=>contactChartKey(args)===key;
    Object.defineProperty(factory,'diagnostics',{get:()=>({builds,hits,retainedPlans:cache.size,capacity})});
    return Object.freeze(factory);
}
function contactChartKey({layout,modes,relativeToolId='wire',supportPolicy='local-two-edge'}) {
    return JSON.stringify({supportPolicy,nodes:layout.nodeCount,positions:Array.from(layout.positions),edges:layout.edgeToolIds,relativeToolId,
        modes:modes.map(m=>[m.node,m.basis.map(v=>Array.from(v)),Array.from(m.relativeDofs)])});
}
export function createCompositeContactPullback(args) {
    return createCompositeContactPullbackFactory(args)(args.points);
}
function contactPoints(layout,points) {
    if(!Array.isArray(points)||!points.length)throw new RangeError('A fixed layout, explicit modes and ordered physical points are required');
    const hasTool=(id,node)=>[node-1,node].some(edge=>edge>=0&&edge<layout.nodeCount-1&&layout.edgeToolIds[edge].includes(id));
    return points.map(p=>{
        if(!Number.isInteger(p.node)||p.node<0||p.node>=layout.nodeCount||typeof p.toolId!=='string'||!hasTool(p.toolId,p.node))
            throw new RangeError('Each physical contact point must belong to a present tool');
        return {toolId:p.toolId,node:p.node};
    });
}
function compileContactMapping({layout,byNode,relativeToolId,supportPolicy},ownedPoints) {
    const nodes=ownedPoints.map(p=>p.node),anchorNode=Math.min(...nodes);
    const declaredPair=supportPolicy==='declared-tool-pair'&&ownedPoints.length===4&&ownedPoints[0].toolId!==ownedPoints[2].toolId&&[0,2].every(i=>ownedPoints[i+1].toolId===ownedPoints[i].toolId&&ownedPoints[i+1].node===ownedPoints[i].node+1);
    if(Math.max(...nodes)-anchorNode>2&&!declaredPair)throw new RangeError('Contact correspondence exceeds the local two-edge stencil; rebuild its chart');
    const commonDofs=Int32Array.from([...new Set(nodes.flatMap(node=>Array.from({length:3},(_,k)=>layout.positions[node]+k)))].sort((a,b)=>a-b));
    const relativeDofs=Int32Array.from([...new Set(ownedPoints.flatMap(p=>p.toolId===relativeToolId?(byNode.get(p.node)?.dofs??[]):[]))].sort((a,b)=>a-b));
    const commonIndices=new Map(Array.from(commonDofs,(d,i)=>[d,i])),relativeIndices=new Map(Array.from(relativeDofs,(d,i)=>[d,i+commonDofs.length]));
    const physicalCount=3*ownedPoints.length,count=commonDofs.length+relativeDofs.length,columns=Array.from({length:count},()=>[]);
    ownedPoints.forEach((p,i)=>{
        for(let k=0;k<3;k++)columns[commonIndices.get(layout.positions[p.node]+k)].push([3*i+k,1]);
        const mode=p.toolId===relativeToolId?byNode.get(p.node):null;
        mode?.basis.forEach((b,a)=>b.forEach((value,k)=>{if(value!==0)columns[relativeIndices.get(mode.dofs[a])].push([3*i+k,value]);}));
    });
    const pairs=[];
    // Keep both matrix halves: normalized physical normals need not be the
    // gradient of the reported gap, and DB need not be symmetric.
    for(let i=0;i<count;i++)for(let j=0;j<count;j++) {
        const terms=[];
        for(const [a,wa] of columns[i])for(const [b,wb] of columns[j])terms.push(physicalCount*a+b,wa*wb);
        pairs.push(terms);
    }
    return {anchorNode,commonDofs,relativeDofs,physicalCount,count,columns,pairs};
}
function createContactMapping(plan,ownedPoints) {
    const {anchorNode,physicalCount,count}=plan;
    const workspace={anchorNode,unit:'mm',commonDofs:plan.commonDofs.slice(),relativeDofs:plan.relativeDofs.slice(),points:ownedPoints,physicalDofCount:physicalCount,dofCount:count,
        gap:NaN,gapJacobian:new Float64Array(count),normalForceColumn:new Float64Array(count),forceColumn:new Float64Array(count),
        normalDerivative:new Float64Array(count*count),supported:false,operatorReady:false,hessianValid:false,certified:false,
        scope:'fixed-affine-physical-tool-contact-pullback'};
    plans.set(workspace,plan);return workspace;
}

/** G_joint=T^T G_physical, B_joint=T^T B_physical,
 * DB_joint=T^T DB_physical T. Mechanical contact contribution is -Fn*B,
 * its force column -B, and its geometric tangent -Fn*DB, each ONCE.
 * This function retains the distinction G!=B and does not choose an NCP
 * branch, enforce complementarity, scale Fn, or certify a complete step.
 * Geometry must already have validated its current source/sample/feature.
 * order:'gradient' keeps exact G/B without reading DB; it invalidates the
 * Hessian output. A full pullback requires a fresh physical derivative.
 */
export function pullbackCompositeContact(geometry,workspace,{order='full'}={}) {
    const plan=plans.get(workspace);if(!plan)throw new TypeError('Use a prepared contact pullback workspace');
    workspace.supported=workspace.operatorReady=workspace.hessianValid=false;workspace.gap=NaN;
    for(const a of [workspace.gapJacobian,workspace.normalForceColumn,workspace.forceColumn,workspace.normalDerivative])a.fill(NaN);
    if(!['full','gradient'].includes(order))throw new RangeError('Contact pullback order must be full or gradient');
    const full=order==='full';
    if(geometry?.supported!==true||!Number.isFinite(geometry.gap))throw new RangeError('A current supported physical contact geometry is required');
    const {physicalCount,count,columns,pairs}=plan;
    finiteVector(geometry.gapJacobian,physicalCount,'Physical gap Jacobian');
    finiteVector(geometry.normalForceColumn,physicalCount,'Physical normal force column');
    if(full){
        if(geometry.hessianValid===false)throw new RangeError('A fresh physical normal derivative is required');
        finiteVector(geometry.normalDerivative,physicalCount*physicalCount,'Physical normal derivative');
    }
    if(geometry.forceColumn!==undefined) {
        finiteVector(geometry.forceColumn,physicalCount,'Signed physical force column');
        if(geometry.forceColumn.some((v,i)=>v!==-geometry.normalForceColumn[i]))throw new RangeError('Physical force column must be the negative normal load column');
    }
    for(let i=0;i<count;i++) {
        let g=0,b=0;
        for(const [d,w] of columns[i]){g+=w*geometry.gapJacobian[d];b+=w*geometry.normalForceColumn[d];}
        workspace.gapJacobian[i]=g;workspace.normalForceColumn[i]=b;workspace.forceColumn[i]=-b;
    }
    if(full)for(let i=0;i<pairs.length;i++){const terms=pairs[i];let value=0;for(let k=0;k<terms.length;k+=2)value+=terms[k+1]*geometry.normalDerivative[terms[k]];workspace.normalDerivative[i]=value;}
    if(![workspace.gapJacobian,workspace.normalForceColumn,...(full?[workspace.normalDerivative]:[])].every(a=>a.every(Number.isFinite)))
        throw new RangeError('Nonfinite pulled-back physical contact operator');
    workspace.gap=geometry.gap;workspace.supported=workspace.operatorReady=true;workspace.hessianValid=full;return workspace;
}

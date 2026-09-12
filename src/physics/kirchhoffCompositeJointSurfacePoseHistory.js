const histories = new WeakMap(), paths = new WeakMap();
const finite = (x, name) => { if (!Number.isFinite(x)) throw new RangeError(`${name} must be finite`); return x; };
const id = (x, name) => { if (!(typeof x === 'string' && x.length || typeof x === 'number' && Number.isFinite(x))) throw new TypeError(`${name} must be a string or finite number`); return x; };
const name = (x, label) => { if (typeof x !== 'string' || !x) throw new TypeError(`${label} must be a nonempty string`); return x; };
const vector = (x, n, label) => { if (x?.length !== n) throw new RangeError(`${label} needs ${n} entries`); return Array.from(x, v => finite(v, label)); };
const dot = (a, b) => a.reduce((s, v, j) => s + v * b[j], 0);
const cross = (a, b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const freeze = value => { if (value && typeof value === 'object') { Object.values(value).forEach(freeze); Object.freeze(value); } return value; };
const interval = (x, label) => { const a=vector(x,2,label); if (!(a[1]>a[0]) || !Number.isFinite(a[1]-a[0])) throw new RangeError(`${label} must increase finitely`); return a; };
const unique = (list, key, label) => { const set=new Set();for(const x of list){const k=key(x);if(set.has(k))throw new RangeError(`Duplicate ${label}`);set.add(k);} };
function unsupported(reason, details = {}) { const e=new RangeError(reason);e.code='surface-material-transport-required';e.requiredTransport={reason,...details};throw e; }

/** Immutable OWN accepted pose history. Material segment and node identities
 * preserve their primitive type. A reservoir is an explicit external pose
 * span, not a velocity record or an extrapolation of the first modeled edge.
 * Frames, angles, label spans and directed reference lifts are copied.
 */
export function createCompositeJointSurfacePoseHistory({ toolId, nodes, edges, hinges = [], reservoirIdentity = null } = {}) {
    name(toolId,'Own tool ID');
    if (!Array.isArray(nodes)||nodes.length<2||!Array.isArray(edges)||!edges.length||!Array.isArray(hinges)) throw new TypeError('Explicit own nodes, edges and hinge records are required');
    const ownNodes=nodes.map(n=>({id:id(n?.id,'Node identity'),position:vector(n.position,3,'Accepted own position'),...(n.node===undefined?{}:{node:n.node})}));unique(ownNodes,n=>n.id,'node identity');
    const nodeIndex=new Map(ownNodes.map((n,i)=>[n.id,i]));
    const ownEdges=edges.map((e,i)=>{
        name(e?.edgeId,'Own edge ID');if(e.toolId!==undefined&&e.toolId!==toolId)throw new RangeError('Pose history cannot mix physical tools');
        if(!['accepted','reservoir'].includes(e.source))throw new RangeError('Explicit accepted/reservoir pose source is required');
        if(e.source==='reservoir'&&i!==0)unsupported('only-proximal-reservoir-history-supported',{toolId,edgeId:e.edgeId});
        if(!Array.isArray(e.nodeIds)||e.nodeIds.length!==2||e.nodeIds.some(n=>!nodeIndex.has(n))||e.nodeIds[0]===e.nodeIds[1])throw new RangeError('Each edge needs two distinct declared own node identities');
        const indices=e.nodeIds.map(n=>nodeIndex.get(n)),chord=ownNodes[indices[1]].position.map((x,k)=>x-ownNodes[indices[0]].position[k]),length=Math.hypot(...chord);
        if(!(length>1e-12)||!Number.isFinite(length))throw new RangeError('Accepted pose edge must be nondegenerate');
        const reference={tangent:vector(e.reference?.tangent,3,'Accepted own tangent'),director:vector(e.reference?.director,3,'Accepted own director')},tangent=chord.map(x=>x/length);
        if(Math.abs(dot(reference.tangent,reference.tangent)-1)>1e-10||Math.abs(dot(reference.director,reference.director)-1)>1e-10||Math.abs(dot(reference.tangent,reference.director))>1e-10||Math.hypot(...reference.tangent.map((x,k)=>x-tangent[k]))>1e-10)throw new RangeError('Accepted frame must belong to its own physical edge');
        if(e.source==='accepted'&&(!Number.isInteger(e.edge)||e.edge<0||ownNodes[indices[0]].node!==e.edge||ownNodes[indices[1]].node!==e.edge+1))throw new RangeError('Accepted edges must identify their actual own layout edge and endpoint node indices');
        if(e.source==='reservoir'&&e.edge!==undefined)throw new RangeError('External reservoir is not an actual free rod edge');
        return{edgeId:e.edgeId,...(e.source==='accepted'?{edge:e.edge}:{}),materialSegmentId:id(e.materialSegmentId,'Own material segment identity'),source:e.source,nodeIds:e.nodeIds.slice(),nodes:indices,
            coordinates:interval(e.coordinates,'Own edge coordinates'),labels:interval(e.labels,'Accepted own labels'),reference,angle:finite(e.angle,'Accepted own unwrapped angle')};
    });unique(ownEdges,e=>e.edgeId,'own edge ID');
    const usedNodeIds=new Set(ownEdges.flatMap(e=>e.nodeIds));if(ownNodes.some(n=>!usedNodeIds.has(n.id)))throw new RangeError('Pose history cannot contain unreferenced external nodes');
    const actualEdges=ownEdges.filter(e=>e.source==='accepted');unique(actualEdges,e=>e.edge,'actual physical edge');
    const physicalNodeIds=new Set(actualEdges.flatMap(e=>e.nodeIds));
    for(const n of ownNodes)if(physicalNodeIds.has(n.id)){if(!Number.isInteger(n.node)||n.node<0)throw new RangeError('Actual nodes require their physical layout index');}
    else if(n.node!==undefined)throw new RangeError('External reservoir nodes cannot declare free physical columns');
    unique(ownNodes.filter(n=>physicalNodeIds.has(n.id)),n=>n.node,'actual physical node');
    for(let i=1;i<ownEdges.length;i++){
        const left=ownEdges[i-1],right=ownEdges[i];
        if(left.nodeIds[1]!==right.nodeIds[0]||left.coordinates[1]!==right.coordinates[0]||left.labels[1]!==right.labels[0])throw new RangeError('Adjacent own poses must share one node identity and continuous coordinates/labels');
    }
    if(ownEdges.some(e=>e.source==='reservoir'))id(reservoirIdentity,'Explicit reservoir identity');
    else if(reservoirIdentity!==null)throw new RangeError('A reservoir identity requires an actual reservoir pose span');
    const ownHinges=hinges.map(h=>{
        const left=ownEdges.findIndex(e=>e.edgeId===h?.leftEdgeId),right=ownEdges.findIndex(e=>e.edgeId===h?.rightEdgeId);
        if(left<0||right!==left+1)throw new RangeError('Hinge references must identify adjacent own edges');
        const a=ownEdges[left].reference,b=ownEdges[right].reference,den=1+dot(a.tangent,b.tangent),referenceTwist=finite(h.referenceTwist,'Accepted unwrapped hinge reference lift');
        if(!(den>1e-10))return{leftEdgeId:h.leftEdgeId,rightEdgeId:h.rightEdgeId,referenceTwist,antiparallel:true};
        const v=cross(a.tangent,b.tangent),w=cross(v,a.director),ww=cross(v,w),carried=a.director.map((x,k)=>x+w[k]+ww[k]/den),phase=Math.atan2(dot(b.tangent,cross(carried,b.director)),dot(carried,b.director));
        if(Math.abs(Math.sin(referenceTwist-phase))>1e-10||Math.abs(Math.cos(referenceTwist-phase)-1)>1e-10)throw new RangeError('Accepted hinge reference lift disagrees with its own frames');
        return{leftEdgeId:h.leftEdgeId,rightEdgeId:h.rightEdgeId,referenceTwist,antiparallel:false};
    });unique(ownHinges,h=>h.leftEdgeId,'directed hinge');
    const data=freeze({toolId,nodes:ownNodes,edges:ownEdges,hinges:ownHinges,reservoirIdentity});
    const history=freeze({scope:'own-accepted-affine-surface-pose-history',toolId,reservoirIdentity,nodes:ownNodes,edges:ownEdges,hinges:ownHinges,includesAngularHistory:true});
    histories.set(history,data);return history;
}

/** Prepare a fixed linear-in-time map/pose dependency contract. Bindings mean
 * currentValue = offset + J * configuration, with a CONSTANT explicitly
 * supplied Jacobian. Empty terms declare a prescribed value; identity terms
 * attach an endpoint to a physical node. A shared junction has ONE node ID
 * and ONE binding. Candidate values are never cached or inferred from rates.
 */
export function prepareCompositeJointSurfacePosePath({history,targetEdgeId,dt,currentMaps,configurationColumns,nodeBindings,angleBindings,reservoirIdentity=history?.reservoirIdentity}={}) {
    const accepted=histories.get(history);if(!accepted)throw new TypeError('Use an owned surface pose history');
    if(!(finite(dt,'Pose path dt')>0))throw new RangeError('Pose path dt must be positive');
    if(reservoirIdentity!==accepted.reservoirIdentity)throw new RangeError('Reservoir identity changed during preparation');
    const target=accepted.edges.findIndex(e=>e.edgeId===targetEdgeId);if(target<0||accepted.edges[target].source!=='accepted')throw new RangeError('Current target must be an original accepted own edge');
    if(!Array.isArray(currentMaps)||currentMaps.length!==accepted.edges.length)throw new RangeError('Current label maps must cover all declared pose edges');
    unique(currentMaps,m=>m.edgeId,'current edge map');
    const maps=accepted.edges.map(e=>{const m=currentMaps.find(m=>m.edgeId===e.edgeId);if(!m)throw new RangeError('Missing own current material map');const labels=interval(m.labels,'Current own labels');
        const rates=labels.map((s,j)=>finite((s-e.labels[j])/dt,'Implied own label rate'));
        if(m.dsDt!==undefined){const supplied=typeof m.dsDt==='number'?[m.dsDt,m.dsDt]:vector(m.dsDt,2,'Own endpoint label rates');supplied.forEach((r,j)=>{finite(r,'Own label rate');const bound=64*Number.EPSILON*(Math.abs(labels[j])+Math.abs(e.labels[j]))/dt;if(Math.abs(r-rates[j])>bound)throw new RangeError('Label rate disagrees with accepted/current linear maps');});}
        return{edgeId:e.edgeId,labels,rates};});
    for(let i=1;i<maps.length;i++)if(maps[i-1].labels[1]!==maps[i].labels[0])throw new RangeError('Current own material labels must be continuous');
    if(!Array.isArray(configurationColumns)||!configurationColumns.length)throw new RangeError('Explicit physical configuration column identities are required');
    const physicalNodes=new Set(accepted.edges.filter(e=>e.source==='accepted').flatMap(e=>e.nodes.map(j=>accepted.nodes[j].node))),physicalEdges=new Set(accepted.edges.filter(e=>e.source==='accepted').map(e=>e.edge));
    const columns=configurationColumns.map(c=>{
        if(c?.toolId!==accepted.toolId)throw new RangeError('Configuration columns must belong to the same own physical tool');
        if(c.kind==='position'&&Number.isInteger(c.node)&&physicalNodes.has(c.node)&&Number.isInteger(c.component)&&c.component>=0&&c.component<3)return{kind:c.kind,toolId:c.toolId,node:c.node,component:c.component};
        if(c.kind==='angle'&&Number.isInteger(c.edge)&&physicalEdges.has(c.edge))return{kind:c.kind,toolId:c.toolId,edge:c.edge};
        throw new RangeError('Configuration columns identify actual own node components or edge angles');
    });unique(columns,c=>JSON.stringify(c),'physical configuration column');
    const terms=(list,vectorTerm)=>{
        if(!Array.isArray(list))throw new RangeError('Binding terms must explicitly declare their dependency, including an empty prescribed list');
        const own=list.map(t=>{if(!Number.isInteger(t?.column)||t.column<0||t.column>=columns.length)throw new RangeError('Binding column lies outside the declared physical support');return vectorTerm?{column:t.column,weights:vector(t.weights,3,'Node binding weights')}:{column:t.column,weight:finite(t.weight,'Angle binding weight')};});
        unique(own,t=>t.column,'binding derivative column');return own;
    };
    if(!Array.isArray(nodeBindings)||nodeBindings.length!==accepted.nodes.length||!Array.isArray(angleBindings)||angleBindings.length!==accepted.edges.length)throw new RangeError('One explicit current binding per unique node and own edge angle is required');
    unique(nodeBindings,b=>b.nodeId,'node binding');unique(angleBindings,b=>b.edgeId,'angle binding');
    const positions=accepted.nodes.map(n=>{const b=nodeBindings.find(b=>b.nodeId===n.id);if(!b)throw new RangeError('Missing own node dependency contract');return{nodeId:n.id,offset:vector(b.offset,3,'Current node binding offset'),terms:terms(b.terms,true)};});
    const angles=accepted.edges.map(e=>{const b=angleBindings.find(b=>b.edgeId===e.edgeId);if(!b)throw new RangeError('Missing own angle dependency contract');return{edgeId:e.edgeId,offset:finite(b.offset,'Current unwrapped angle binding offset'),terms:terms(b.terms,false)};});
    // Actual physical poses retain every reaction DOF. Dirichlet elimination
    // belongs to the parent solver, never to the surface dependency contract.
    accepted.nodes.forEach((node,j)=>{if(node.node===undefined)return;const b=positions[j];
        if(b.offset.some(x=>x!==0)||b.terms.length!==3)throw new RangeError('Actual own endpoint must bind identically to its physical configuration columns');
        for(let k=0;k<3;k++){const index=columns.findIndex(c=>c.kind==='position'&&c.node===node.node&&c.component===k),term=b.terms.find(t=>t.column===index);
            if(index<0||!term||term.weights.some((v,axis)=>v!==(axis===k?1:0)))throw new RangeError('Actual own endpoint must bind identically to its physical configuration columns');}
    });
    accepted.edges.forEach((e,j)=>{if(e.source!=='accepted')return;const b=angles[j],index=columns.findIndex(c=>c.kind==='angle'&&c.edge===e.edge);
        if(index<0||b.offset!==0||b.terms.length!==1||b.terms[0].column!==index||b.terms[0].weight!==1)throw new RangeError('Actual own angle must bind identically to its physical configuration column');
    });
    const queryColumns=[{kind:'coordinate',toolId:accepted.toolId,edgeId:targetEdgeId},...['current','previous'].flatMap(time=>['point','normal','tangent'].flatMap(kind=>[0,1,2].map(component=>({time,kind,component}))))];
    const data=freeze({...accepted,target,targetEdgeId,dt,maps,configurationColumns:columns,nodeBindings:positions,angleBindings:angles,queryColumns});
    const path=freeze({scope:'prepared-own-affine-reservoir-surface-path',toolId:accepted.toolId,targetEdgeId,materialSegmentId:accepted.edges[target].materialSegmentId,reservoirIdentity,
        dt,configurationColumns:columns,queryColumns,configurationDofs:columns.length,queryDofs:19,includesInternalHingeTransport:false});
    paths.set(path,data);return path;
}

// Shared module-internal reader: the WeakMap establishes preparation provenance,
// and the recursively frozen result cannot be changed by consumers.
export function readCompositeJointSurfacePosePath(path) {
    const data=paths.get(path);if(!data)throw new TypeError('Use a prepared own surface pose path');return data;
}

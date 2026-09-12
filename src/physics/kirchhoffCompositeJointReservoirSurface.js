import { readCompositeJointSurfacePosePath } from './kirchhoffCompositeJointSurfacePoseHistory.js';
import { createCompositeJointSurfaceForceMapWorkspace, evaluateCompositeJointSurfaceForceMap } from './kirchhoffCompositeJointSurfaceMotion.js';

const plans = new WeakMap();
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0), add=(a,b)=>a.map((v,i)=>v+b[i]), sub=(a,b)=>a.map((v,i)=>v-b[i]), scale=(a,s)=>a.map(v=>v*s);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],unit=a=>scale(a,1/Math.hypot(...a));
const pt=(d,a,t)=>{const v=cross(a,t),w=cross(v,d);return add(add(d,w),scale(cross(v,w),1/(1+dot(a,t))));};
const phase=(a,b,t)=>Math.atan2(dot(t,cross(a,b)),dot(a,b));
function reject(reason,details={}){const e=new RangeError(reason);e.code='surface-material-transport-required';e.requiredTransport={reason,...details};throw e;}
function vector(value,n,label){if(value?.length!==n||!Array.from(value).every(Number.isFinite))throw new RangeError(`${label} needs ${n} finite entries`);return Array.from(value);}

// First-order AD of the stated finite rule, never a numerical difference.
// Value mode uses the same scalar operations with zero derivative dimensions.
function dualAlgebra(n) {
    const constant = x => ({ x, d: new Float64Array(n) });
    const variable = (x,i) => { const a=constant(x);if(n)a.d[i]=1;return a; };
    const unary=(a,x,p)=>({x,d:Float64Array.from(a.d,v=>p*v)});
    const binary=(a,b,x,p,q)=>({x,d:Float64Array.from(a.d,(v,i)=>p*v+q*b.d[i])});
    const plus=(a,b)=>binary(a,b,a.x+b.x,1,1), minus=(a,b)=>binary(a,b,a.x-b.x,1,-1), mul=(a,b)=>binary(a,b,a.x*b.x,b.x,a.x);
    const times=(a,s)=>unary(a,a.x*s,s), inverse=a=>unary(a,1/a.x,-1/(a.x*a.x));
    const sqrt=a=>{if(!(a.x>0))reject('degenerate-path-tangent');return unary(a,Math.sqrt(a.x),.5/Math.sqrt(a.x));};
    const atan2=(y,x)=>{const d=y.x*y.x+x.x*x.x;if(!(d>0))reject('unresolved-path-phase');return binary(y,x,Math.atan2(y.x,x.x),x.x/d,-y.x/d);};
    const vadd=(a,b)=>a.map((v,i)=>plus(v,b[i])), vsub=(a,b)=>a.map((v,i)=>minus(v,b[i])), vmul=(a,s)=>a.map(v=>mul(v,s));
    const vdot=(a,b)=>a.reduce((s,v,i)=>plus(s,mul(v,b[i])),constant(0));
    const vcross=(a,b)=>[minus(mul(a[1],b[2]),mul(a[2],b[1])),minus(mul(a[2],b[0]),mul(a[0],b[2])),minus(mul(a[0],b[1]),mul(a[1],b[0]))];
    const vunit=a=>vmul(a,inverse(sqrt(vdot(a,a))));
    const transport=(d,a,t)=>{const den=plus(constant(1),vdot(a,t));if(!(den.x>1e-10))reject('antiparallel-path-chart');const k=vcross(a,t),w=vcross(k,d);return vadd(vadd(d,w),vmul(vcross(k,w),inverse(den)));};
    const signedPhase=(a,b,t)=>atan2(vdot(t,vcross(a,b)),vdot(a,b));
    const interpolate=(a,b,f)=>vadd(a,vmul(vsub(b,a),f));
    const rotateT=(d,t,angle)=>vadd(vmul(d,unary(angle,Math.cos(angle.x),-Math.sin(angle.x))),vmul(vcross(t,d),unary(angle,Math.sin(angle.x),Math.cos(angle.x))));
    return {constant,variable,plus,minus,mul,times,inverse,sqrt,atan2,vadd,vsub,vmul,vdot,vcross,vunit,transport,signedPhase,interpolate,rotateT};
}

function evaluatePathCore(input, { full = true } = {}) {
    const {nodes,edges,hinges,query,geometry}=input, count=nodes.current.length, config=3*count+edges.length, n=config+19, a=dualAlgebra(full?n:0);
    const {constant:C,variable:V,plus:P,minus:M,mul:U,times:S,inverse:I,vadd:A,vsub:D,vmul:T,vdot:O,vcross:X,vunit:N,transport:PT,interpolate:L,rotateT:RT}=a;
    if (!(input.dt>0)||input.toolId!==edges[0].toolId||edges.some(e=>e.toolId!==input.toolId)) reject('own-path-tool-identity');
    edges.forEach((e,i)=>{
        if (!['accepted','reservoir'].includes(e.source) || e.source==='reservoir' && e.poseProvided!==true) reject('explicit-reservoir-pose-required');
        if (!e.frame || !Number.isFinite(e.oldAngle)||!Number.isFinite(e.angle)) reject('explicit-own-orientation-required');
        for(const key of ['oldLabels','newLabels']) if(!(e[key]?.[1]>e[key]?.[0]))reject('increasing-own-label-span-required');
        if(i && (edges[i-1].nodes[1]!==e.nodes[0]||edges[i-1].oldLabels[1]!==e.oldLabels[0]||edges[i-1].newLabels[1]!==e.newLabels[0]))reject('nonadjacent-own-material-history');
        const oldT=unit(sub(nodes.old[e.nodes[1]],nodes.old[e.nodes[0]]));
        if(Math.hypot(...sub(oldT,e.frame.tangent))>1e-10||Math.abs(dot(e.frame.director,oldT))>1e-10||Math.abs(dot(e.frame.director,e.frame.director)-1)>1e-10)reject('stale-own-path-frame');
    });
    const frame=(q,start)=>{const p=q.point.map((x,k)=>V(x,start+k)),t=N(q.tangent.map((x,k)=>V(x,start+6+k))),raw=q.normal.map((x,k)=>V(x,start+3+k)),normal=N(D(raw,T(t,O(raw,t))));return{p,axes:[normal,X(t,normal),t]};};
    const current=frame(geometry.current,config+1),old=frame(geometry.previous,config+10),toFrame=(v,f)=>f.axes.map(d=>O(d,v));
    const q=nodes.current.map((p,i)=>p.map((x,k)=>V(x,3*i+k))), coordinate=V(query.coordinate,config), target=edges.findIndex(e=>e.id===query.edgeId);
    if(target<0)reject('missing-target-edge');
    const targetEdge=edges[target], f=S(M(coordinate,C(targetEdge.coordinates[0])),1/(targetEdge.coordinates[1]-targetEdge.coordinates[0]));
    if(f.x<0||f.x>1||f.x===0&&query.trace!=='right'||f.x===1&&query.trace!=='left')reject('explicit-target-trace-required');
    const label=P(C(targetEdge.newLabels[0]),S(f,targetEdge.newLabels[1]-targetEdge.newLabels[0]));
    if(edges.some(e=>label.x===e.oldLabels[0]||label.x===e.oldLabels[1])){
        if(!['left','right'].includes(query.previousTrace))reject('explicit-accepted-material-trace-required',{toolId:input.toolId,materialLabel:label.x});
        const covered=edges.some(e=>query.previousTrace==='right'?label.x>=e.oldLabels[0]&&label.x<e.oldLabels[1]:label.x>e.oldLabels[0]&&label.x<=e.oldLabels[1]);
        if(!covered)reject('accepted-material-trace-outside-own-history',{toolId:input.toolId,materialLabel:label.x,trace:query.previousTrace});
    }
    const edgeData=edges.map((e,i)=>{
        const po=e.nodes.map(j=>toFrame(D(nodes.old[j].map(C),old.p),old)),pn=e.nodes.map(j=>toFrame(D(q[j],current.p),current)),
            oldT=toFrame(e.frame.tangent.map(C),old),oldD=toFrame(e.frame.director.map(C),old),
            worldT=N(D(q[e.nodes[1]],q[e.nodes[0]])),newT=toFrame(worldT,current),
            newD=toFrame(PT(e.frame.director.map(C),e.frame.tangent.map(C),worldT),current),
            referencePhase=a.signedPhase(PT(oldD,oldT,newT),newD,newT);
        if(Math.PI-Math.abs(referencePhase.x)<=1e-10)reject('time-reference-lift-required');
        return{e,po,pn,oldT,oldD,referencePhase,theta:V(e.angle,3*count+i)};
    });
    function pose(i,tau,sideFraction=null){
        const d=edgeData[i],e=d.e,ends=d.po.map((p,j)=>L(p,d.pn[j],tau)),t=N(D(ends[1],ends[0])),
            ref=RT(PT(d.oldD,d.oldT,t),t,U(tau,d.referencePhase)),theta=P(C(e.oldAngle),U(tau,M(d.theta,C(e.oldAngle)))),
            sa=P(C(e.oldLabels[0]),S(tau,e.newLabels[0]-e.oldLabels[0])),sb=P(C(e.oldLabels[1]),S(tau,e.newLabels[1]-e.oldLabels[1])),
            fraction=sideFraction===null?U(M(label,sa),I(M(sb,sa))):C(sideFraction);
        return{t,ref,theta,center:L(ends[0],ends[1],fraction),fraction};
    }
    function angular(from,to,anchor=null){
        const c=O(from.t,to.t),v=X(from.t,to.t),den=P(C(1),c);if(!(den.x>1e-10))reject('antiparallel-path-chart');
        const delta=M(C(1),c);let k;
        if(Math.abs(delta.x)<1e-3){const coeff=[1];for(let j=1;j<=6;j++)coeff.push(coeff[j-1]*j/(2*j+1));k=C(coeff[6]);for(let j=5;j>=0;j--)k=P(C(coeff[j]),U(delta,k));}
        else{const sine=a.sqrt(O(v,v));k=U(a.atan2(sine,c),I(sine));}
        let phase=a.signedPhase(PT(from.ref,from.t,to.t),to.ref,to.t);
        if(anchor!==null){if(!Number.isFinite(anchor))reject('explicit-hinge-reference-lift-required');const winding=2*Math.PI*Math.round((anchor-phase.x)/(2*Math.PI));phase=P(phase,C(winding));if(Math.PI-Math.abs(phase.x-anchor)<=1e-10)reject('hinge-reference-lift-ambiguous');}
        else if(Math.PI-Math.abs(phase.x)<=1e-10)reject('short-segment-reference-lift-required');
        const alpha=P(M(to.theta,from.theta),phase),swing=T(v,k),meanT=T(A(from.t,to.t),I(U(k,den)));
        return {omega:A(swing,T(meanT,alpha)),alpha};
    }
    function increment(from,to,anchor=null){const {omega,alpha}=angular(from,to,anchor),lever=Svec(A(from.center,to.center),-.5);return{value:A(D(to.center,from.center),X(omega,lever)),omega,alpha,lever};}
    function Svec(v,s){return v.map(x=>S(x,s));}
    // All event locations come from label = interpolated boundary label.
    const events=[]; let initialIndex=null, persistentIndex=null;
    for(let i=0;i<edges.length-1;i++){
        const oldB=edges[i].oldLabels[1],newB=edges[i].newLabels[1],change=newB-oldB;
        if(change===0){
            if(label.x===oldB){
                if(!['left','right'].includes(query.previousTrace)||query.previousTrace!==query.trace)reject('persistent-material-hinge-needs-matching-traces');
                persistentIndex=i+(query.trace==='right'?1:0);if(persistentIndex!==target)reject('persistent-hinge-trace-owner');
            }continue;
        }
        let tau=S(M(label,C(oldB)),1/change);
        // Exact target boundary equality establishes the event at the end of
        // the step. Retain its one-sided event derivative and the final zero-
        // duration target segment; otherwise proximal-entry G loses a term.
        if(label.x===newB)tau.x=1; if(label.x===oldB)tau.x=0;
        const direction=change<0?1:-1;let leavesInitial=false;
        if(tau.x===0){
            if(!['left','right'].includes(query.previousTrace))reject('explicit-initial-event-trace-path-required');
            initialIndex=i+(query.previousTrace==='right'?1:0);leavesInitial=direction>0?initialIndex===i:initialIndex===i+1;
        }
        const entersTarget=direction>0?target===i+1:target===i;
        if(tau.x>0&&tau.x<1||tau.x===0&&leavesInitial||tau.x===1&&entersTarget){events.push({tau,boundary:i,direction,terminal:tau.x===1,initial:tau.x===0});}
    }
    events.sort((a,b)=>a.tau.x-b.tau.x);
    for(let i=1;i<events.length;i++)if(events[i].tau.x===events[i-1].tau.x)reject('simultaneous-path-events');
    const cuts=[C(0),...events.map(e=>e.tau),C(1)],segments=[],jumps=[];
    let total=[0,0,0].map(C),previousIndex=null;
    for(let j=0;j<cuts.length-1;j++){
        const midpoint=(cuts[j].x+cuts[j+1].x)/2;
        let index=cuts[j].x===0&&cuts[j+1].x===0?initialIndex:cuts[j].x===1&&cuts[j+1].x===1?target:persistentIndex!==null?persistentIndex:edges.findIndex(e=>{const lo=e.oldLabels[0]+midpoint*(e.newLabels[0]-e.oldLabels[0]),hi=e.oldLabels[1]+midpoint*(e.newLabels[1]-e.oldLabels[1]);return label.x>lo&&label.x<hi;});
        if(index<0&&query.trace===query.previousTrace&&['left','right'].includes(query.trace)){
            const e=edges[target],boundary=query.trace==='right'?0:1;
            if(label.x===e.oldLabels[boundary]&&label.x===e.newLabels[boundary])index=target;
        }
        if(index<0)reject('missing-own-pose-history-or-reservoir',{toolId:input.toolId,materialLabel:label.x,tau:midpoint,currentEdgeId:query.edgeId});
        if(previousIndex!==null){
            const event=events[j-1];if(index-previousIndex!==event.direction)reject('invalid-path-adjacency');
            const hinge=hinges.find(h=>h.left===edges[event.boundary].id&&h.right===edges[event.boundary+1].id);
            if(!hinge)reject('explicit-hinge-rotation-path-required',{toolId:input.toolId,materialLabel:label.x,tau:cuts[j].x});
            if(edges[previousIndex].source!=='reservoir'||edges[index].source!=='accepted'||index!==target||event.direction!==1)
                reject('internal-hinge-surface-reconstruction-required',{toolId:input.toolId,materialLabel:label.x,tau:cuts[j].x,fromEdgeId:edges[previousIndex].id,toEdgeId:edges[index].id,direction:event.direction});
            const entry=edges[previousIndex],own=edges[index],oldMismatch=own.oldAngle-entry.oldAngle+hinge.referenceTwist;
            if(Math.hypot(...sub(entry.frame.tangent,own.frame.tangent))>1e-10||Math.abs(oldMismatch)>1e-10)
                reject('incompatible-accepted-reservoir-orientation',{toolId:input.toolId,materialLabel:label.x,tau:cuts[j].x,fromEdgeId:entry.id,toEdgeId:own.id,unwrappedSpinMismatch:oldMismatch});
            const leftFrame=edges[event.boundary].frame,rightFrame=edges[event.boundary+1].frame,
                acceptedPhase=phase(pt(leftFrame.director,leftFrame.tangent,rightFrame.tangent),rightFrame.director,rightFrame.tangent);
            if(!Number.isFinite(hinge.referenceTwist)||Math.abs(Math.sin(hinge.referenceTwist-acceptedPhase))>1e-10||Math.abs(Math.cos(hinge.referenceTwist-acceptedPhase)-1)>1e-10)reject('stale-accepted-hinge-reference-lift');
            const from=pose(previousIndex,cuts[j],event.direction>0?1:0),to=pose(index,cuts[j],event.direction>0?0:1),
                jump=increment(from,to,event.direction*hinge.referenceTwist);
            total=A(total,jump.value);jumps.push({event,from:previousIndex,to:index,...jump});
        }
        const from=pose(index,cuts[j]),to=pose(index,cuts[j+1]),piece=increment(from,to);
        total=A(total,piece.value);segments.push({edgeId:edges[index].id,tau:[cuts[j],cuts[j+1]],from,to,...piece});previousIndex=index;
    }
    if(previousIndex!==target)reject('wrong-current-target-trace');
    const projected=[total[2],total[1]],values=v=>v.map(x=>x.x);
    for(const x of total)if(!Number.isFinite(x.x)||!x.d.every(Number.isFinite))throw new RangeError('Nonfinite own surface path value/derivative');
    return {increment:values(projected),relativeIncrement:values(total),jacobian:projected.map(x=>Array.from(x.d)),configurationDofs:config,dofCount:n,
        label:label.x,events:events.map(e=>({tau:e.tau.x,derivative:Array.from(e.tau.d),boundary:e.boundary,direction:e.direction,terminal:e.terminal,initial:e.initial})),
        segments:segments.map(s=>({edgeId:s.edgeId,tau:values(s.tau),from:values(s.from.center),to:values(s.to.center),increment:values(s.value)})),
        jumps:jumps.map(j=>({from:j.from,to:j.to,omega:values(j.omega),alpha:j.alpha.x,lever:values(j.lever),increment:values(j.value)})),
        reconstruction:'own-affine-space-time-compatible-reservoir-entry;short-swing-own-spin;mean-lever-per-piece',certified:false};
}

function invalidate(out, plan, reason='not-evaluated') {
    for(const key of ['supported','operatorReady','incrementValid','configurationJacobianValid','queryJacobianValid','GValid','forceMapValid','DforceMapValid','forceMapQueryDerivativeValid','physicalForceMapValid','physicalDforceMapValid','physicalForceMapQueryDerivativeValid'])out[key]=false;
    out.reason=reason;out.requiredTransport=null;out.order=null;out.physicalForce=null;out.materialLabel=NaN;out.events=[];out.segments=[];out.jumps=[];
    for(const array of Object.values(plan.buffers))array.fill(NaN);
}

/** Reusable owned output; every call first revokes previous validity, including
 * calls that reject before evaluation. G is [component,configurationColumn],
 * B is [configurationColumn,component] so F=B*Ft. DB is [column,component,
 * derivativeColumn]. Prescribed external derivatives are already contracted.
 */
export function createCompositeJointReservoirSurfaceWorkspace(path) {
    const data=readCompositeJointSurfacePosePath(path),N=data.configurationColumns.length,Q=19,rawN=3*data.nodes.length+data.edges.length;
    const buffers={increment:new Float64Array(2),relativeIncrement:new Float64Array(3),configurationJacobian:new Float64Array(2*N),queryJacobian:new Float64Array(2*Q),
        forceMap:new Float64Array(N*2),DforceMap:new Float64Array(N*2*N),forceMapQueryDerivative:new Float64Array(N*2*Q),physicalForceMap:new Float64Array(14),physicalDforceMap:new Float64Array(14*7),physicalForceMapQueryDerivative:new Float64Array(14*10)};
    const matrix=new Float64Array(rawN*N);
    data.nodeBindings.forEach((b,i)=>b.terms.forEach(t=>t.weights.forEach((w,k)=>matrix[(3*i+k)*N+t.column]=w)));
    data.angleBindings.forEach((b,i)=>b.terms.forEach(t=>matrix[(3*data.nodes.length+i)*N+t.column]=t.weight));
    const target=data.edges[data.target],physicalRows=[...target.nodes.flatMap(j=>[3*j,3*j+1,3*j+2]),3*data.nodes.length+data.target],
        physicalToConfiguration=new Float64Array(7*N);
    physicalRows.forEach((r,j)=>physicalToConfiguration.set(matrix.subarray(r*N,(r+1)*N),j*N));
    const currentTools=Object.freeze([Object.freeze({id:data.toolId,edge:target.edge,edgeId:target.edgeId,materialSegmentId:target.materialSegmentId,
        nodeIds:Object.freeze(target.nodeIds.slice()),nodes:Object.freeze(target.nodes.map(j=>data.nodes[j].node))})]);
    const out={scope:'own-affine-compatible-proximal-reservoir-surface',...buffers,configurationDofs:N,queryDofs:Q,dofCount:N+Q,
        configurationColumns:data.configurationColumns,queryColumns:data.queryColumns,currentTools,tools:currentTools,
        physicalDofCount:7,physicalQueryDofs:10,physicalQueryColumns:Object.freeze(data.queryColumns.slice(0,10)),reservoirIdentity:data.reservoirIdentity,includesInternalHingeTransport:false,certified:false,
        instantaneousRatesRequired:false,velocityKnown:false,prescribedFeedAndWallPowerKnown:false,
        derivativeConvention:'G=d(finite increment)/d(configuration);F=B*Ft;G is not B',
        dependencies:'current external pose = explicit offset + constant supplied Jacobian * actual own configuration'};
    out.slipJacobian=out.G=out.configurationJacobian;
    out.physicalConfigurationColumns=Object.freeze([...currentTools[0].nodes.flatMap(node=>[0,1,2].map(component=>Object.freeze({kind:'position',toolId:data.toolId,node,component}))),Object.freeze({kind:'angle',toolId:data.toolId,edge:target.edge})]);
    // Public scratch contents are reusable; their column identities, shape and
    // buffer references cannot be replaced after successful preparation.
    for(const key of [...Object.keys(buffers),'slipJacobian','G','configurationColumns','queryColumns','physicalConfigurationColumns','currentTools','tools','configurationDofs','queryDofs','dofCount','physicalDofCount','physicalQueryDofs','physicalQueryColumns','reservoirIdentity'])Object.defineProperty(out,key,{writable:false,configurable:false});
    const plan={data,N,Q,rawN,matrix,physicalRows,physicalToConfiguration,buffers,busy:false,forceWorkspace:createCompositeJointSurfaceForceMapWorkspace(1)};
    plans.set(out,plan);invalidate(out,plan);return out;
}

export function evaluateCompositeJointReservoirSurface(input, out) {
    const plan=plans.get(out);if(!plan)throw new TypeError('Use a prepared own reservoir surface workspace');
    if(plan.busy)throw new RangeError('Own reservoir surface workspace is busy');plan.busy=true;invalidate(out,plan);
    try{
        const {data,N,Q,rawN,matrix,physicalToConfiguration:T}=plan,order=input?.order??'full',full=order==='full';
        if(!['full','value'].includes(order))throw new RangeError('Surface order must be full or value');
        const configuration=vector(input?.configuration,N,'Actual own physical configuration'),query=input?.query;
        if(!query||!Number.isFinite(query.coordinate))throw new RangeError('Current own material coordinate must be finite');
        if(query.trace!==undefined&&!['left','right'].includes(query.trace)||query.previousTrace!==undefined&&!['left','right'].includes(query.previousTrace))throw new RangeError('Own endpoint traces must be left or right');
        const geometry=input?.finiteGeometry;if(geometry?.kind!=='explicit-affine-side-queries')throw new TypeError('Explicit current and previous objective affine-side queries are required');
        const queries=Object.fromEntries(['current','previous'].map(time=>{const q=geometry[time];return[time,Object.fromEntries(['point','normal','tangent'].map(key=>[key,vector(q?.[key],3,`${time} query ${key}`)]))];}));
        for(const q of Object.values(queries)){
            const length=Math.hypot(...q.tangent);if(!(length>1e-12)||!Number.isFinite(length))throw new RangeError('Nondegenerate finite query tangent is required');
            const t=q.tangent.map(x=>x/length),projected=q.normal.map((x,k)=>x-dot(q.normal,t)*t[k]);if(!(Math.hypot(...projected)>1e-12))throw new RangeError('Nondegenerate finite query normal is required');
        }
        // Affine contracts are evaluated from the supplied current configuration
        // on EVERY retry; no stale reservoir/candidate samples are retained.
        const positions=data.nodeBindings.map(b=>b.offset.map((x,k)=>b.terms.reduce((s,t)=>s+t.weights[k]*configuration[t.column],x))),
            angles=data.angleBindings.map(b=>b.terms.reduce((s,t)=>s+t.weight*configuration[t.column],b.offset));
        if(!positions.flat().every(Number.isFinite)||!angles.every(Number.isFinite))throw new RangeError('Nonfinite current pose dependency evaluation');
        const coreInput={dt:data.dt,toolId:data.toolId,nodes:{old:data.nodes.map(n=>n.position),current:positions},
            edges:data.edges.map((e,j)=>({id:e.edgeId,toolId:data.toolId,source:e.source,poseProvided:e.source==='reservoir',nodes:e.nodes,coordinates:e.coordinates,
                oldLabels:e.labels,newLabels:data.maps[j].labels,frame:e.reference,oldAngle:e.angle,angle:angles[j]})),
            hinges:data.hinges.map(h=>({left:h.leftEdgeId,right:h.rightEdgeId,referenceTwist:h.referenceTwist})),
            query:{edgeId:data.targetEdgeId,coordinate:query.coordinate,trace:query.trace,previousTrace:query.previousTrace},geometry:queries};
        const r=evaluatePathCore(coreInput,{full}),target=data.edges[data.target];
        const force=evaluateCompositeJointSurfaceForceMap({order,tools:[{id:data.toolId,edgeId:target.edgeId,coordinates:target.coordinates,coordinate:query.coordinate,trace:query.trace,
            positions:target.nodes.map(j=>positions[j]),previousPositions:target.nodes.map(j=>data.nodes[j].position),reference:target.reference,angle:angles[data.target]}],
            forceGeometry:{kind:'explicit-affine-side-query',...queries.current}},plan.forceWorkspace);
        out.increment.set(r.increment);out.relativeIncrement.set(r.relativeIncrement);out.physicalForceMap.set(force.forceMap);
        for(let column=0;column<N;column++)for(let k=0;k<2;k++){
            let value=0;for(let physical=0;physical<7;physical++)value+=T[physical*N+column]*force.forceMap[physical*2+k];out.forceMap[column*2+k]=value;
        }
        if(full){
            out.physicalDforceMap.set(force.configurationDerivative);out.physicalForceMapQueryDerivative.set(force.queryDerivative);
            for(let k=0;k<2;k++){
                for(let j=0;j<N;j++){let v=0;for(let raw=0;raw<rawN;raw++)v+=r.jacobian[k][raw]*matrix[raw*N+j];out.configurationJacobian[k*N+j]=v;}
                out.queryJacobian.set(r.jacobian[k].slice(rawN),k*Q);
            }
            for(let column=0;column<N;column++)for(let k=0;k<2;k++){
                const entry=column*2+k;
                for(let j=0;j<N;j++){
                    let value=0;for(let p=0;p<7;p++)if(T[p*N+column]!==0)for(let d=0;d<7;d++)value+=T[p*N+column]*force.configurationDerivative[(p*2+k)*7+d]*T[d*N+j];
                    out.DforceMap[entry*N+j]=value;
                }
                for(let j=0;j<Q;j++){
                    let value=0;if(j<10)for(let p=0;p<7;p++)value+=T[p*N+column]*force.queryDerivative[(p*2+k)*10+j];
                    out.forceMapQueryDerivative[entry*Q+j]=value;
                }
            }
        }
        for(const array of full?Object.values(plan.buffers):[out.increment,out.relativeIncrement,out.forceMap,out.physicalForceMap])if(!array.every(Number.isFinite))throw new RangeError('Nonfinite contracted own surface operator');
        out.events=r.events.map(event=>{
            const {derivative,...metadata}=event;
            const from=data.edges[event.boundary+(event.direction>0?0:1)],to=data.edges[event.boundary+(event.direction>0?1:0)];
            return{...metadata,fromEdgeId:from.edgeId,toEdgeId:to.edgeId,fromMaterialSegmentId:from.materialSegmentId,toMaterialSegmentId:to.materialSegmentId,configurationDerivative:full?Float64Array.from({length:N},(_,j)=>derivative.slice(0,rawN).reduce((s,v,raw)=>s+v*matrix[raw*N+j],0)):null,
                queryDerivative:full?Float64Array.from(derivative.slice(rawN)):null};
        });
        out.segments=r.segments;out.jumps=r.jumps.map(j=>({...j,fromEdgeId:data.edges[j.from].edgeId,toEdgeId:data.edges[j.to].edgeId}));out.materialLabel=r.label;out.physicalForce=force;out.order=order;out.reason=null;
        out.supported=out.incrementValid=out.forceMapValid=out.physicalForceMapValid=true;
        out.operatorReady=out.configurationJacobianValid=out.queryJacobianValid=out.GValid=out.DforceMapValid=out.forceMapQueryDerivativeValid=out.physicalDforceMapValid=out.physicalForceMapQueryDerivativeValid=full;
        return out;
    }catch(error){invalidate(out,plan,error.message);out.requiredTransport=error.requiredTransport?structuredClone(error.requiredTransport):null;throw error;}
    finally{plan.busy=false;}
}

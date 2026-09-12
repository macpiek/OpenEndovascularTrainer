import assert from 'node:assert/strict';
import test from 'node:test';
import { evaluateCompositeJointSurfaceIncrement, evaluateCompositeJointSurfaceForceMap } from '../src/physics/kirchhoffCompositeJointSurfaceMotion.js';

// TEST-ONLY executable reconstruction proposal. This is not a production
// provider or permission to widen any existing contact/transport validity gate.
const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
const add = (a, b) => a.map((v, i) => v + b[i]), sub = (a, b) => a.map((v, i) => v - b[i]), scale = (a, s) => a.map(v => s * v);
const cross = (a, b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const unit = a => scale(a, 1 / Math.hypot(...a));
const close = (a, b, eps = 2e-10) => assert.ok(Number.isFinite(a) && Number.isFinite(b) && Math.abs(a-b) <= eps, `${a} != ${b}`);
const same = (a,b,eps) => { assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],eps)); };
const rotate = (v,a,angle) => add(add(scale(v,Math.cos(angle)),scale(cross(a,v),Math.sin(angle))),scale(a,dot(a,v)*(1-Math.cos(angle))));
const pt = (d,a,t) => { const v=cross(a,t), w=cross(v,d); return add(add(d,w),scale(cross(v,w),1/(1+dot(a,t)))); };
const phase = (a,b,t) => Math.atan2(dot(t,cross(a,b)),dot(a,b));
function reject(code) { const e = new RangeError(code); e.code=code; throw e; }

// Small independent first-order dual algebra, with no production helper reuse.
function dualAlgebra(n) {
    const constant = x => ({ x, d: new Float64Array(n) });
    const variable = (x,i) => { const a=constant(x);a.d[i]=1;return a; };
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

/** Concrete test-only API:
 * nodes: accepted/current physical world endpoints; edges: OWN node indices,
 * accepted frame, old/current unwrapped angle, old/new physical label endpoints.
 * query: current target edge, coordinate and explicit endpoint one-sided trace.
 * hinges: accepted directed reference-twist lifts, never inferred from theta.
 * Reservoir edges MUST be explicit pose records with the same data, plus source.
 * The path traverses the SAME current label through linear-in-time material
 * maps. Each crossing has its actual event tau. Each interval uses the stated
 * symmetric-mean-lever short-swing/own-spin rule; a zero-length hinge is a
 * SEPARATE angular jump, not an averaged frame or a smooth surface claim.
 * G columns include ALL declared current physical node coordinates and own
 * edge angles, current query coordinate, current and previous point/n/t.
 */
function piecewisePath(input, { freezeEventDerivatives = false } = {}) {
    const {nodes,edges,hinges,query,geometry}=input, count=nodes.current.length, config=3*count+edges.length, n=config+19, a=dualAlgebra(n);
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
        if(tau.x>0&&tau.x<1||tau.x===0&&leavesInitial||tau.x===1&&entersTarget){if(freezeEventDerivatives)tau=C(tau.x);events.push({tau,boundary:i,direction,terminal:tau.x===1,initial:tau.x===0});}
    }
    events.sort((a,b)=>a.tau.x-b.tau.x);
    for(let i=1;i<events.length;i++)if(events[i].tau.x===events[i-1].tau.x)reject('simultaneous-path-events');
    const cuts=[C(0),...events.map(e=>e.tau),C(1)],segments=[],jumps=[];
    let total=[0,0,0].map(C),previousIndex=null;
    for(let j=0;j<cuts.length-1;j++){
        const midpoint=(cuts[j].x+cuts[j+1].x)/2;
        const index=cuts[j].x===0&&cuts[j+1].x===0?initialIndex:cuts[j].x===1&&cuts[j+1].x===1?target:persistentIndex!==null?persistentIndex:edges.findIndex(e=>{const lo=e.oldLabels[0]+midpoint*(e.newLabels[0]-e.oldLabels[0]),hi=e.oldLabels[1]+midpoint*(e.newLabels[1]-e.oldLabels[1]);return label.x>lo&&label.x<hi;});
        if(index<0)reject('missing-own-pose-history-or-reservoir');
        if(previousIndex!==null){
            const event=events[j-1];if(index-previousIndex!==event.direction)reject('invalid-path-adjacency');
            const hinge=hinges.find(h=>h.left===edges[event.boundary].id&&h.right===edges[event.boundary+1].id);
            if(!hinge)reject('explicit-hinge-rotation-path-required');
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
    return {increment:values(projected),relativeIncrement:values(total),jacobian:projected.map(x=>Array.from(x.d)),configurationDofs:config,dofCount:n,
        label:label.x,events:events.map(e=>({tau:e.tau.x,derivative:Array.from(e.tau.d),boundary:e.boundary,direction:e.direction,terminal:e.terminal,initial:e.initial})),
        segments:segments.map(s=>({edgeId:s.edgeId,tau:values(s.tau),from:values(s.from.center),to:values(s.to.center),increment:values(s.value)})),
        jumps:jumps.map(j=>({from:j.from,to:j.to,omega:values(j.omega),alpha:j.alpha.x,lever:values(j.lever),increment:values(j.value)})),
        reconstruction:'piecewise-own-affine-space-time;short-swing-own-spin;mean-lever-per-piece;explicit-zero-length-hinge-jump',certified:false};
}

function bent({dt=.1,oldShift=0,newShift=-.1,coordinate=1.05,edge=1,twist=.5,radius=.2}={}){
    const positions=[[-1,0,0],[0,0,0],[0,1,0]],frames=[{tangent:[1,0,0],director:[0,0,1]},{tangent:[0,1,0],director:[0,0,1]}],
        edges=[0,1].map(i=>({id:`wire:e${i}`,toolId:'wire',source:'accepted',nodes:[i,i+1],coordinates:[i,i+1],oldLabels:[i+oldShift,i+1+oldShift],newLabels:[i+newShift,i+1+newShift],frame:frames[i],oldAngle:i*twist,angle:i*twist})),
        query={point:[radius,0,0],normal:[1,0,0],tangent:[0,1,0]};
    return {dt,toolId:'wire',nodes:{old:structuredClone(positions),current:structuredClone(positions)},edges,hinges:[{left:edges[0].id,right:edges[1].id,referenceTwist:0}],query:{edgeId:edges[edge].id,coordinate},geometry:{current:structuredClone(query),previous:structuredClone(query)}};
}
function perturb(input,j,amount){const count=input.nodes.current.length,config=3*count+input.edges.length;
    if(j<3*count)input.nodes.current[Math.floor(j/3)][j%3]+=amount;
    else if(j<config)input.edges[j-3*count].angle+=amount;
    else if(j===config)input.query.coordinate+=amount;
    else{const local=j-config-1,q=input.geometry[local<9?'current':'previous'];q[['point','normal','tangent'][Math.floor((local%9)/3)]][local%3]+=amount;}
}
function productionInput(input){const e=input.edges.find(e=>e.id===input.query.edgeId),i=input.edges.indexOf(e),dx=e.coordinates[1]-e.coordinates[0],f=(input.query.coordinate-e.coordinates[0])/dx,
    oldSx=(e.oldLabels[1]-e.oldLabels[0])/dx,sx=(e.newLabels[1]-e.newLabels[0])/dx,label=e.newLabels[0]+f*(e.newLabels[1]-e.newLabels[0]),oldF=(label-e.oldLabels[0])/(e.oldLabels[1]-e.oldLabels[0]);
    return{tools:[{id:input.toolId,edgeId:e.id,coordinates:e.coordinates,coordinate:input.query.coordinate,trace:input.query.trace,
        positions:e.nodes.map(j=>input.nodes.current[j]),previousPositions:e.nodes.map(j=>input.nodes.old[j]),reference:e.frame,angle:e.angle,previousAngle:e.oldAngle,
        materialMap:{sStart:e.newLabels[0],dsDx:sx,dsDt:((e.newLabels[0]-e.oldLabels[0])+(sx-oldSx)*dx*f)/input.dt},
        materialPath:{kind:'linear-affine-maps',previousEdgeId:e.id,previousMap:{sStart:e.oldLabels[0],dsDx:oldSx},previousTrace:oldF===0?'right':oldF===1?'left':undefined}}],
        dt:input.dt,wall:{velocity:[0,0,0]},rotationPath:'short-contact-frame-own-unwrapped-spins',finiteGeometry:{kind:'explicit-affine-side-queries',...input.geometry},
        forceGeometry:{kind:'explicit-affine-side-query',...input.geometry.current}};
}

test('sharp bent own two-edge path traces the same label, retains swing/twist work and reverses feed exactly',context=>{
    const input=bent(),r=piecewisePath(input),reverse=bent({oldShift:-.1,newShift:0,coordinate:.95,edge:0}),back=piecewisePath(reverse);
    close(r.label,.95);close(back.label,r.label);close(r.events[0].tau,.5);assert.equal(r.events[0].direction,1);assert.equal(back.events[0].direction,-1);
    same(r.increment,[.05+.2*Math.PI/2,.2/Math.PI],2e-14);same(back.increment,scale(r.increment,-1),2e-14);
    assert.deepEqual(r.segments.map(s=>s.edgeId),['wire:e0','wire:e1']);assert.equal(r.jumps.length,1);
    const fullTurn=piecewisePath(bent({twist:.5+2*Math.PI}));close(fullTurn.increment[1]-r.increment[1],.8,2e-14);
    assert.throws(()=>evaluateCompositeJointSurfaceIncrement(productionInput(input)),{code:'surface-material-transport-required'});
    context.diagnostic(JSON.stringify({forward:r.increment,reverse:back.increment,jump:r.jumps[0],oldLabel:r.label,eventTau:r.events[0].tau}));
});

test('piecewise G differentiates all visited current nodes, own spins, moving label/event time and both contact queries',context=>{
    const input=bent();input.nodes.current[0]=[-1.02,.03,.04];input.nodes.current[1]=[.02,-.01,.03];input.nodes.current[2]=[.04,1.03,-.02];input.edges[0].angle+=.15;input.edges[1].angle-=.07;
    const r=piecewisePath(input),h=1e-6;let maximum=0;
    for(let j=0;j<r.dofCount;j++){const p=structuredClone(input),m=structuredClone(input);perturb(p,j,h);perturb(m,j,-h);const a=piecewisePath(p),b=piecewisePath(m);
        for(let k=0;k<2;k++){const fd=(a.increment[k]-b.increment[k])/(2*h),error=Math.abs(fd-r.jacobian[k][j]);maximum=Math.max(maximum,error);close(fd,r.jacobian[k][j],3e-8);}}
    const wrong=piecewisePath(input,{freezeEventDerivatives:true}),eventColumn=r.configurationDofs;
    const eventError=Math.max(...r.jacobian.map((row,k)=>Math.abs(row[eventColumn]-wrong.jacobian[k][eventColumn])));
    assert.ok(eventError>.01); // Freezing the segment partition silently loses G.
    assert.ok(Math.max(...r.jacobian.flatMap(row=>row.slice(0,3).map(Math.abs)))>.001); // previous own edge's CURRENT nodes enter G
    context.diagnostic(JSON.stringify({columns:r.dofCount,maxGError:maximum,frozenEventGError:eventError,nonlocalConfigurationDofs:r.configurationDofs}));
});

test('each own reference gauge transforms its angle and directed hinge lift together without losing winding',()=>{
    const input=bent({twist:4*Math.PI+.5}),r=piecewisePath(input),g=[2*Math.PI+.7,-4*Math.PI+.4];
    input.edges.forEach((e,i)=>{e.frame.director=rotate(e.frame.director,e.frame.tangent,g[i]);e.angle-=g[i];e.oldAngle-=g[i];});input.hinges[0].referenceTwist+=g[1]-g[0];
    const changed=piecewisePath(input);same(changed.increment,r.increment,3e-14);changed.jacobian.forEach((row,k)=>same(row,r.jacobian[k],3e-13));
});

test('the explicit path is objective under observer rotation/translation and common finite motion of geometry and query',()=>{
    const base=bent(),result=piecewisePath(base),axis=unit([.3,-.4,.8]),angle=.83,shift=[.3,-.1,.7],R=v=>rotate(v,axis,angle),transform=p=>add(R(p),shift);
    const observer=structuredClone(base);for(const key of ['old','current'])observer.nodes[key]=observer.nodes[key].map(transform);
    observer.edges.forEach(e=>{e.frame.tangent=R(e.frame.tangent);e.frame.director=R(e.frame.director);});
    for(const q of Object.values(observer.geometry)){q.point=transform(q.point);q.normal=R(q.normal);q.tangent=R(q.tangent);}
    same(piecewisePath(observer).increment,result.increment,3e-14);
    const moving=structuredClone(base);moving.nodes.current=moving.nodes.current.map(transform);const q=moving.geometry.current;q.point=transform(q.point);q.normal=R(q.normal);q.tangent=R(q.tangent);
    moving.edges.forEach(e=>{const t=R(e.frame.tangent),d=pt(e.frame.director,e.frame.tangent,t);e.angle+=phase(d,R(e.frame.director),t);});
    same(piecewisePath(moving).increment,result.increment,3e-14);
    const noFeed=bent({newShift:0,coordinate:1.3});noFeed.nodes.current=noFeed.nodes.current.map(transform);const c=noFeed.geometry.current;c.point=transform(c.point);c.normal=R(c.normal);c.tangent=R(c.tangent);
    noFeed.edges.forEach(e=>{const t=R(e.frame.tangent),d=pt(e.frame.director,e.frame.tangent,t);e.angle+=phase(d,R(e.frame.director),t);});same(piecewisePath(noFeed).increment,[0,0],3e-14);
});

test('no-crossing specialization agrees with frozen finite rule and reaches the current physical B plus prescribed-feed power',context=>{
    const values=[];const rates=[[.1,.04,-.03],[-.04,.03,.02],[.02,-.05,.04]],spins=[.4,-.7],u=.3,traction=[.3,-.8];
    for(const dt of [1e-3,1e-4,1e-5]){
        const input=bent({dt,newShift:-u*dt,coordinate:1.4});input.nodes.current=input.nodes.old.map((p,i)=>add(p,scale(rates[i],dt)));input.edges.forEach((e,i)=>e.angle+=spins[i]*dt);
        const own=piecewisePath(input),direct=productionInput(input),finite=evaluateCompositeJointSurfaceIncrement(direct),force=evaluateCompositeJointSurfaceForceMap(direct);
        same(own.increment,Array.from(finite.increment),3e-14);
        const columns=[3,4,5,6,7,8,10,...Array.from({length:19},(_,j)=>11+j)];
        for(let row=0;row<2;row++)columns.forEach((column,j)=>close(own.jacobian[row][column],finite.jacobian[row*finite.dofCount+j],3e-13));
        const e=input.edges[1],configurationRate=e.nodes.flatMap(j=>rates[j]).concat(spins[1]),
            feed=scale(sub(direct.tools[0].positions[1],direct.tools[0].positions[0]),u),expected=force.axes.map((axis,k)=>dot(axis,feed)+configurationRate.reduce((s,v,j)=>s+force.forceMap[j*2+k]*v,0)),velocity=scale(own.increment,1/dt),error=Math.max(...sub(velocity,expected).map(Math.abs));
        const forcePower=configurationRate.reduce((s,v,j)=>s+v*(force.forceMap[j*2]*traction[0]+force.forceMap[j*2+1]*traction[1]),0)+dot(traction,force.axes.map(axis=>dot(axis,feed)));
        close(dot(traction,velocity),forcePower,dt*.3);values.push({dt,error,powerError:Math.abs(dot(traction,velocity)-forcePower)});
    }
    assert.ok(values[2].error<.011*values[0].error);context.diagnostic(JSON.stringify(values));
});

test('a zero-length sharp hinge carries finite angular slip, so no bounded current-edge B limit exists at the crossing',context=>{
    const values=[];for(const dt of [.1,.01,.001]){const input=bent({dt,newShift:-dt,coordinate:1+dt/2}),r=piecewisePath(input);values.push({dt,slip:r.increment,rate:scale(r.increment,1/dt)});close(r.increment[0]-dt/2,.2*Math.PI/2,2e-13);close(r.increment[1],.2/Math.PI,2e-13);}
    assert.ok(values[2].rate[0]>100*values[0].rate[0]*.8);context.diagnostic(JSON.stringify(values));
});

function reservoirEntry(dt=.1){
    const input=bent({dt,newShift:-.3*dt,coordinate:1,edge:1,twist:0});input.nodes.old=[[-1,0,0],[0,0,0],[1,0,0]];input.nodes.current=structuredClone(input.nodes.old);
    input.edges.forEach(e=>e.frame={tangent:[1,0,0],director:[0,1,0]});input.edges[0].source='reservoir';input.edges[0].poseProvided=true;input.query.trace='right';
    // Material labels and chart coordinates are independent: shift this test's
    // complete chart by -1 so the actual proximal point is coordinate zero.
    input.edges.forEach(e=>{e.coordinates=e.coordinates.map(x=>x-1);e.oldLabels=e.oldLabels.map(x=>x-1);e.newLabels=e.newLabels.map(x=>x-1);});input.query.coordinate=0;
    const query={point:[0,.2,0],normal:[0,1,0],tangent:[1,0,0]};input.geometry={current:structuredClone(query),previous:structuredClone(query)};return input;
}

test('known straight reservoir pose closes real negative-label-rate proximal advancement with an exact finite-power limit',context=>{
    const values=[];for(const dt of [.1,.01,1e-4]){
        const input=reservoirEntry(dt),r=piecewisePath(input),direct=productionInput(input);assert.throws(()=>evaluateCompositeJointSurfaceIncrement(direct),{code:'surface-material-transport-required'});
        same(r.increment,[.3*dt,0],3e-14);assert.equal(r.segments[0].edgeId,'wire:e0');close(r.label,-.3*dt,3e-14);
        assert.equal(r.events.length,1); assert.equal(r.events[0].terminal,true); close(r.events[0].tau,1);
        same(r.jumps[0].increment,[0,0,0],3e-14); // Explicitly compatible reservoir/target frames make the terminal jump zero.
        const map=evaluateCompositeJointSurfaceForceMap(direct);assert.equal(map.forceMap.length,14);
        values.push({dt,label:r.label,slip:r.increment,rate:scale(r.increment,1/dt)});
    }context.diagnostic(JSON.stringify(values));
});

test('missing reservoir pose, unknown hinge path, stale frame, absent history and own tool mismatch reject explicitly',()=>{
    const missing=reservoirEntry();delete missing.edges[0].poseProvided;assert.throws(()=>piecewisePath(missing),{code:'explicit-reservoir-pose-required'});
    const angular=reservoirEntry();delete angular.edges[0].frame;assert.throws(()=>piecewisePath(angular),{code:'explicit-own-orientation-required'});
    const hinge=bent();hinge.hinges=[];assert.throws(()=>piecewisePath(hinge),{code:'explicit-hinge-rotation-path-required'});
    const badLift=bent();badLift.hinges[0].referenceTwist=.2;assert.throws(()=>piecewisePath(badLift),{code:'stale-accepted-hinge-reference-lift'});
    const stale=bent();stale.edges[0].frame.tangent=[0,1,0];assert.throws(()=>piecewisePath(stale),{code:'stale-own-path-frame'});
    const own=bent();own.edges[0].toolId='catheter';assert.throws(()=>piecewisePath(own),{code:'own-path-tool-identity'});
    const absent=bent({newShift:-4});assert.throws(()=>piecewisePath(absent),{code:'missing-own-pose-history-or-reservoir'});
});

test('reservoir entry retains own current bending/spin G and full turns; reverse feed traces accepted own material',context=>{
    const u=.3,rates=[[0,0,0],[.04,-.03,.02],[-.02,.05,.01]],spins=[.25,.7],traction=[.3,-.8],errors=[];
    for(const dt of [1e-3,1e-4,1e-5]){
        const input=reservoirEntry(dt);input.nodes.current=input.nodes.old.map((p,j)=>add(p,scale(rates[j],dt)));input.edges.forEach((e,j)=>e.angle+=dt*spins[j]);
        const r=piecewisePath(input),force=evaluateCompositeJointSurfaceForceMap(productionInput(input)),ownRate=[...rates[1],...rates[2],spins[1]],feed=scale(sub(input.nodes.current[2],input.nodes.current[1]),u),
            expected=force.axes.map((axis,k)=>dot(axis,feed)+ownRate.reduce((s,v,j)=>s+force.forceMap[j*2+k]*v,0)),velocity=scale(r.increment,1/dt),error=Math.max(...sub(expected,velocity).map(Math.abs));
        close(dot(traction,velocity),dot(traction,expected),dt*.1);errors.push({dt,error,powerError:Math.abs(dot(traction,velocity)-dot(traction,expected))});
        assert.ok(Math.abs(r.jacobian[1][10])>.19); // The target's own spin remains in G.
    }
    assert.ok(errors[2].error<.011*errors[0].error);
    const winding=reservoirEntry(),zero=piecewisePath(winding);winding.edges[1].angle+=2*Math.PI;const turn=piecewisePath(winding);close(turn.increment[1]-zero.increment[1],.4*Math.PI,3e-14);
    const reverse=reservoirEntry();reverse.edges.forEach(e=>e.newLabels=e.oldLabels.map(x=>x+.3*reverse.dt));
    const back=piecewisePath(reverse);same(back.increment,[-.3*reverse.dt,0],3e-14);assert.deepEqual(back.segments.map(s=>s.edgeId),['wire:e1']);assert.equal(back.jumps.length,0);
    context.diagnostic(JSON.stringify({bendingSpinLimit:errors,fullTurnIncrement:turn.increment,reverseFeed:back.increment}));
});

test('terminal reservoir G keeps one-sided event timing and all pose/query derivatives',context=>{
    const input=reservoirEntry();input.nodes.current[1]=[.01,.02,-.01];input.nodes.current[2]=[1.03,-.01,.04];input.edges[0].angle=.03;input.edges[1].angle=.09;
    const r=piecewisePath(input),h=1e-6;let maxError=0;
    for(let j=0;j<r.dofCount;j++){
        const p=structuredClone(input),m=structuredClone(input);perturb(p,j,h);
        let difference,denominator;
        if(j===r.configurationDofs){difference=sub(piecewisePath(p).increment,r.increment);denominator=h;} // outgoing right trace at the proximal endpoint
        else{perturb(m,j,-h);difference=sub(piecewisePath(p).increment,piecewisePath(m).increment);denominator=2*h;}
        for(let k=0;k<2;k++){const fd=difference[k]/denominator;maxError=Math.max(maxError,Math.abs(fd-r.jacobian[k][j]));close(fd,r.jacobian[k][j],j===r.configurationDofs?2e-5:3e-8);}
    }
    context.diagnostic(JSON.stringify({columns:r.dofCount,maxTerminalGError:maxError,queryDerivative:'right one-sided at target endpoint'}));
});

test('exact initial and persistent hinge labels require explicit accepted traces and retain endpoint jumps',()=>{
    const incoming=bent({coordinate:1.1});incoming.query.previousTrace='left';const fromLeft=piecewisePath(incoming);
    assert.equal(fromLeft.events[0].initial,true);close(fromLeft.events[0].tau,0);same(fromLeft.increment,[.1+.2*Math.PI/2,.2/Math.PI],3e-14);
    const outgoing=structuredClone(incoming);outgoing.query.previousTrace='right';const fromRight=piecewisePath(outgoing);assert.equal(fromRight.events.length,0);same(fromRight.increment,[.1,0],3e-14);
    const missing=structuredClone(incoming);delete missing.query.previousTrace;assert.throws(()=>piecewisePath(missing),{code:'explicit-initial-event-trace-path-required'});
    const reverse=bent({oldShift:-.1,newShift:0,coordinate:1,edge:0});reverse.query.trace='left';const back=piecewisePath(reverse);same(back.increment,scale(fromLeft.increment,-1),3e-14);close(back.label,fromLeft.label);
    const persistent=bent({newShift:0,coordinate:1});persistent.query.trace='right';assert.throws(()=>piecewisePath(persistent),{code:'persistent-material-hinge-needs-matching-traces'});
    persistent.query.previousTrace='right';same(piecewisePath(persistent).increment,[0,0],3e-14);
});

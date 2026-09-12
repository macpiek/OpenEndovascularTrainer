import {kirchhoffComponentBodies} from './kirchhoffComponentBodies.js';
import {createKirchhoffWallWitnessGeometryWorkspace,evaluateKirchhoffWallWitnessGeometry} from './kirchhoffWallWitnessGeometry.js';
import {createKirchhoffWallReactionLedger,captureKirchhoffWallReaction,recordKirchhoffWallReaction,
    buildKirchhoffWallReactionRelease,commitKirchhoffWallReactionRelease} from './kirchhoffWallReactionLedger.js';

function stateFor(component){return component._wallWitnessRows??={bodies:kirchhoffComponentBodies(component),witnesses:[],lookup:new Map(),rows:[],field:null,begun:false,pending:false};}
function loaded(w){return w.ledger.lambda!==0||w.ledger.wrenches.some(v=>['fx','fy','fz','mx','my','mz'].some(k=>v[k]!==0));}
function point(w){const b=w.body,n=w.node;for(let axis=0;axis<3;axis++){const p=b[['x','y','z'][axis]];w.point[axis]=(1-w.t)*p[n]+w.t*p[n+1];}return w.point;}
function evaluate(w,state){
 const g=evaluateKirchhoffWallWitnessGeometry({geometry:state.field.fallbackGeometry,faceIndex:w.face,point:point(w)},w.geometry);
 if(w.triangleKey!==null&&w.triangleKey!==g.triangleKey)throw new Error('Wall witness geometry changed; retained face identity is invalid');
 if(!g.normalDefined)throw new Error('Wall witness at zero distance has unsupported signed normal');
 const a=g.triangleVertices;
 const ux=a[3]-a[0],uy=a[4]-a[1],uz=a[5]-a[2],vx=a[6]-a[0],vy=a[7]-a[1],vz=a[8]-a[2];
 const nx=uy*vz-uz*vy,ny=uz*vx-ux*vz,nz=ux*vy-uy*vx;
 const side=(w.point[0]-a[0])*nx+(w.point[1]-a[1])*ny+(w.point[2]-a[2])*nz;
 if(w.orientation===0){if(side===0)throw new Error('Wall witness has unresolved interior side');w.orientation=Math.sign(side);}
 if(!(w.orientation*side>0))throw new Error('Wall witness crossed retained interior branch');
 w.triangleKey=g.triangleKey;return g;
}
function support(w){const b=w.body;return w.node>=Math.max(b.activeStart,b.collisionStartSegment,0)&&w.node<Math.min(b.activeEnd,b.collisionEndSegment+1,b.segmentCount);}
function identity(w){return w.body.materialCoordinate[w.node]===w.materialA&&w.body.materialCoordinate[w.node+1]===w.materialB;}

function tuple(side,node,a,b,t,face){return `${side}:${node}:${a}:${b}:${t}:${face}`;}
function current(w){const b=w.body;return support(w)&&identity(w)&&b.wallActive[w.node]!==0&&b.wallFaceIndex[w.node]===w.face&&b.wallT[w.node]===w.t;}
function prune(state){
 let kept=0;state.lookup.clear();
 for(const w of state.witnesses){
  // Zero scalar load alone is insufficient: applied wrench contributions can
  // remain after changing normal directions and must be explicitly released.
  if(!loaded(w)&&!current(w)&&!w.discoveredForStep)continue;
  state.witnesses[kept++]=w;
  state.lookup.set(tuple(w.side,w.node,w.materialA,w.materialB,w.t,w.face),w);
 }
 state.witnesses.length=kept;
}

export function beginKirchhoffWallWitnessStep(component){
 const state=stateFor(component),bodies=kirchhoffComponentBodies(component);
 if(state.bodies.length!==bodies.length||bodies.some((b,i)=>b!==state.bodies[i]))throw new Error('Wall witness component changed');
 // Consistent with the existing world: positional forces start at zero each
 // physical step; feature identity survives, no old force is warm-started.
 for(const w of state.witnesses){w.ledger.lambda=0;w.ledger.wrenches.length=0;w.ledger.normalWrenches.length=0;w.ledger.tangentLambda.fill(0);w.ledger.retiring=false;w.discoveredForStep=false;}
 prune(state);state.rows.length=0;state.begun=true;state.pending=false;return state;
}

export function collectKirchhoffWallWitnessRows(component,field,rows,dt){
 const state=stateFor(component);
 if(!state.begun)throw new Error('Begin wall witness step before collecting');
 if(!Number.isFinite(dt)||dt<=0||!Array.isArray(rows)||rows===state.rows)throw new TypeError('Positive timestep and independent row collector required');
 if(state.field&&state.field!==field)throw new Error('Wall witness field changed');
 state.field=field;state.rows.length=0;prune(state);
 for(let side=0;side<state.bodies.length;side++){
  const body=state.bodies[side];
  for(let node=Math.max(0,body.activeStart,body.collisionStartSegment);node<Math.min(body.activeEnd,body.collisionEndSegment+1,body.segmentCount);node++){
   if(!body.wallActive[node])continue;
   if(body.wallFaceIndex[node]<0)throw new Error('Active wall has no represented finite face: '+JSON.stringify({body:body.id,node,t:body.wallT[node],gap:body.wallGap[node],a:[body.x[node],body.y[node],body.z[node]],b:[body.x[node+1],body.y[node+1],body.z[node+1]],radius:Math.max(body.nodeRadius[node],body.nodeRadius[node+1])}));
   const radius=Math.max(body.nodeRadius[node],body.nodeRadius[node+1]),t=body.wallT[node],face=body.wallFaceIndex[node];
   if(!Number.isFinite(t)||t<0||t>1||!(body.wallGap[node]+radius>0))throw new Error('Unsupported exterior or zero-distance wall discovery');
   const materialA=body.materialCoordinate[node],materialB=body.materialCoordinate[node+1];
   if(!Number.isFinite(materialA)||!Number.isFinite(materialB))throw new Error('Wall witness material identity is invalid');
   const key=tuple(side,node,materialA,materialB,t,face);
   if(state.lookup.has(key))continue;
   const w={body,side,node,materialA,materialB,t,face,radius,triangleKey:null,orientation:0,point:new Float64Array(3),
    geometry:createKirchhoffWallWitnessGeometryWorkspace(),ledger:createKirchhoffWallReactionLedger(body),row:{},release:{},frozen:[]};
   evaluate(w,state);state.witnesses.push(w);state.lookup.set(key,w);
  }
 }
 for(const w of state.witnesses){
  if(!identity(w)){if(loaded(w))throw new Error('Release wall reaction before changing material identity');continue;}
  if(!support(w)){
   if(!loaded(w))continue;
   const row=buildKirchhoffWallReactionRelease(w.ledger,w.side,w.release);row.additionalIndex=rows.length;row.witness=w;
   rows.push(row);state.rows.push(row);continue;
  }
  if(Math.max(w.body.nodeRadius[w.node],w.body.nodeRadius[w.node+1])!==w.radius)throw new Error('Retained wall radius changed');
  const g=evaluate(w,state),row=w.row;
  Object.assign(row,{kind:'wall-witness',side:w.side,node:w.node,owner:w.ledger,witness:w,strain:g.distance-w.radius,
   alpha:w.body.wallCompliance/(dt*dt),lambda:w.ledger.lambda,lower:0,upper:Infinity,additionalIndex:rows.length});
  const gradients=row.gradients??=[];gradients.length=0;const pool=row.gradientPool??=[];
  for(let end=0;end<2;end++)for(let axis=0;axis<3;axis++){
   const value=(end?w.t:1-w.t)*g.direction[axis];if(!value)continue;
   const entry=pool[gradients.length]??=Object.seal({side:0,dof:0,value:0});entry.side=w.side;entry.dof=(w.node+end)*6+axis;entry.value=value;gradients.push(entry);
  }
  captureKirchhoffWallReaction(w.body,gradients,w.side,w.frozen);rows.push(row);state.rows.push(row);
 }
 state.pending=true;return rows;
}

export function commitKirchhoffWallWitnessMultipliers(component,increments,scale){
 const state=stateFor(component);
 if(!state.pending||!Number.isFinite(scale)||scale<0||scale>1)throw new Error('Unapplied wall witness rows and common scale required');
 for(const row of state.rows){const delta=increments[row.additionalIndex]*scale;
  if(!Number.isFinite(delta)||row.lambda+delta<0||row.kind==='wall-release'&&row.lambda+delta>1)throw new RangeError('Invalid wall witness increment');}
 for(const row of state.rows){
  const delta=increments[row.additionalIndex]*scale,w=row.witness;
  if(row.kind==='wall-release')commitKirchhoffWallReactionRelease(row,delta);
  else {
   recordKirchhoffWallReaction(w.ledger,w.frozen,delta);
   for(const g of row.gradients){const n=Math.floor(g.dof/6),axis=g.dof%6;w.body[['wallProjectionX','wallProjectionY','wallProjectionZ'][axis]][n]+=w.body.inverseMass[n]*g.value*delta;}
  }
 }
 const sums=new Map();for(const w of state.witnesses){let nodes=sums.get(w.body);if(!nodes)sums.set(w.body,nodes=new Map());nodes.set(w.node,(nodes.get(w.node)??0)+w.ledger.lambda);}
 for(const [body,nodes]of sums)for(const [node,sum]of nodes)body.wallLambda[node]=sum;
 state.pending=false;
}

export function measureKirchhoffWallWitnessResidual(component){
 const state=stateFor(component),out=state.measurement??={};out.maximumResidual=0;out.pending=0;out.pendingReleases=0;out.pendingDiscoveries=0;out.finite=true;
 // Measurement cannot create rows or overwrite the frozen unapplied batch.
 // A newly queried witness prevents certification until collect represents it.
 for(let side=0;side<state.bodies.length;side++){const body=state.bodies[side];for(let node=Math.max(0,body.activeStart,body.collisionStartSegment);node<Math.min(body.activeEnd,body.collisionEndSegment+1,body.segmentCount);node++){
  if(!body.wallActive[node])continue;
  if(body.wallFaceIndex[node]<0)throw new Error('Active wall has no represented finite face: '+JSON.stringify({body:body.id,node,t:body.wallT[node],gap:body.wallGap[node],a:[body.x[node],body.y[node],body.z[node]],b:[body.x[node+1],body.y[node+1],body.z[node+1]],radius:Math.max(body.nodeRadius[node],body.nodeRadius[node+1])}));
  if(!state.lookup.has(tuple(side,node,body.materialCoordinate[node],body.materialCoordinate[node+1],body.wallT[node],body.wallFaceIndex[node]))){out.pending++;out.pendingDiscoveries++;}
 }}
 for(const w of state.witnesses){
  if(!identity(w)){if(loaded(w))throw new Error('Loaded wall material identity changed');continue;}
  if(!support(w)){if(loaded(w)){out.pending++;out.pendingReleases++;}continue;}
  const g=evaluate(w,state),r=g.distance-w.radius+(w.row.alpha??0)*w.ledger.lambda;
  out.maximumResidual=Math.max(out.maximumResidual,w.ledger.lambda>0?Math.abs(r):Math.max(0,-r));
  out.finite&&=Number.isFinite(r)&&Number.isFinite(w.ledger.lambda);
 }
 return out;
}

// A rejected nonlinear trial may reveal a missing constraint. Preserve only
// its geometry identity, never the rejected pose or applied force history.
export function captureKirchhoffWallDiscoveries(component,known){
 return stateFor(component).witnesses.filter(w=>!known.has(w)).map(w=>({
  body:w.body,side:w.side,node:w.node,materialA:w.materialA,materialB:w.materialB,
  t:w.t,face:w.face,radius:w.radius,orientation:w.orientation,triangleKey:w.triangleKey
 }));
}
export function retainKirchhoffWallDiscoveries(component,candidates){
 const state=stateFor(component);let added=0;
 for(const candidate of candidates){
  const key=tuple(candidate.side,candidate.node,candidate.materialA,candidate.materialB,candidate.t,candidate.face);
  if(state.lookup.has(key))continue;
  const w={...candidate,discoveredForStep:true,point:new Float64Array(3),
   geometry:createKirchhoffWallWitnessGeometryWorkspace(),ledger:createKirchhoffWallReactionLedger(candidate.body),row:{},release:{},frozen:[]};
  if(!identity(w)||!support(w))continue;
  evaluate(w,state);state.witnesses.push(w);state.lookup.set(key,w);added++;
 }
 return added;
}

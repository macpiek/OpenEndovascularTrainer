import {selectKirchhoffWallWitnessFrictionCoefficient} from './kirchhoffWallWitnessFrictionMode.js';
import {kirchhoffComponentBodies} from './kirchhoffComponentBodies.js';
import {buildKirchhoffSurfaceFriction,evaluateKirchhoffSurfaceFrictionKKT} from './kirchhoffSurfaceFriction.js';
import {rotateVectorByQuaternion} from './discreteKirchhoffRod.js';
import {buildKirchhoffWallReactionRelease,captureKirchhoffWallSurfaceReaction,recordKirchhoffWallSurfaceReaction} from './kirchhoffWallReactionLedger.js';
import {measureKirchhoffFrictionMerit} from './kirchhoffFrictionMerit.js';
const XYZ=['X','Y','Z'];
function environment(){
 const b={count:2,segmentCount:1,activeStart:0,activeEnd:1,nodeRadius:new Float64Array(2),inverseMass:new Float64Array(2),orientationControlSegment:-1};
 for(const axis of XYZ){b[axis.toLowerCase()]=new Float64Array(2);b['previous'+axis]=new Float64Array(2);b['velocity'+axis]=new Float64Array(2);b['angularVelocity'+axis]=new Float64Array(1);}
 for(const axis of [...XYZ,'W'])for(const prefix of ['orientation','previousOrientation'])b[prefix+axis]=new Float64Array([axis==='W'?1:0]);
 for(let axis=1;axis<=3;axis++)b['inverseInertia'+axis]=new Float64Array(1);
 return b;
}
function entry(w,out){
 const pool=out.pool??=new WeakMap();let e=pool.get(w);
 if(!e){e={wall:environment(),record:{kind:'side',innerWeights:new Float64Array(2),outerWeights:new Float64Array([1,0]),_outerSegmentIndex:0,
  normal:new Float64Array(3),surfaceContactPoint:new Float64Array(3),surfaceAxialTangent:new Float64Array(3)},constraint:{},surface:{},rows:[{gradients:[]},{gradients:[]}],frozen:[[],[]],lambda:new Float64Array(2),residual:{}};pool.set(w,e);}
 return e;
}
const wrenchAxes=['fx','fy','fz','mx','my','mz'];
// The journal is authoritative: merely projecting lambda into a new tangent
// plane must never erase its normal component or its previously applied moment.
function transport(e,side){
 const ledger=e.witness.ledger, t=e.transport??={body:e.witness.body,wrenches:[],normalWrenches:[]};
 t.wrenches.length=0;let error=0,magnitude=0;
 for(const old of ledger.wrenches){
  const normal=ledger.normalWrenches.find(v=>v.node===old.node);
  const value={node:old.node};
  for(const axis of wrenchAxes){
   let represented=0;for(let i=0;i<2;i++)represented+=(e.frozen[i].find(v=>v.node===old.node)?.[axis]??0)*e.rows[i].lambda;
   const actual=old[axis]-(normal?.[axis]??0);
   value[axis]=actual-represented;error=Math.max(error,Math.abs(value[axis]));magnitude=Math.max(magnitude,Math.abs(actual),Math.abs(represented),Math.abs(old[axis]),Math.abs(normal?.[axis]??0));
  }
  t.wrenches.push(value);
 }
 // Only arithmetic cancellation is ignored, never a force-sized threshold.
 e.hasTransport=error>64*Number.EPSILON*magnitude;
 if(e.hasTransport){buildKirchhoffWallReactionRelease(t,side,e.transportRow??={});e.transportRow.kind='wall-witness-friction-transport';}
 return e.hasTransport;
}
/** One physical surface law per retained wall witness. The environment object
 * supplies fixed kinematics only; none of its DOFs enter the actual component.
 * Unequal coefficients require the opt-in static-candidate/retry controller.
 */
export function buildKirchhoffWallWitnessFriction(component,normalRows,dt,out={}){
 if(!(dt>0)||!Number.isFinite(dt))throw new RangeError('Positive wall friction timestep required');
 out.rows??=[];out.groups??=[];out.entries??=[];out.rows.length=out.groups.length=out.entries.length=0;
 out.version=(out.version??0)+1;
 out.component=component;out.normalRows=normalRows;out.dt=dt;out.committed=false;out.appended=false;out.rowOffset=0;
 const bodies=kirchhoffComponentBodies(component);
 for(const normal of normalRows){
  if(normal.kind!=='wall-witness')continue;
  const w=normal.witness,b=w.body,side=normal.side;
  if(bodies[side]!==b)throw new Error('Wall friction witness does not belong to component');
  const selected=selectKirchhoffWallWitnessFrictionCoefficient(component,w,dt),mu=selected.mu;
  if(mu===0&&b.wallStaticFriction===0){
   if(w.ledger.tangentLambda.some(value=>value!==0))throw new Error('Release loaded wall tangent history before disabling friction');
   continue;
  }
  const contact=w.friction??={tangentLambda:w.ledger.tangentLambda,tangentU:new Float64Array([1,0,0]),tangentV:new Float64Array([0,1,0]),twistLambda:0,
   get normalLambda(){return w.ledger.lambda;}};
  if(contact.tangentLambda!==w.ledger.tangentLambda)throw new Error('Wall tangent history must belong to reaction ledger');
  const e=entry(w,out),r=e.record;r._innerSegmentIndex=w.node;r.innerWeights.set([1-w.t,w.t]);r.normal.set(w.geometry.direction);r.manifoldContact=contact;
  const q={x:b.orientationX[w.node],y:b.orientationY[w.node],z:b.orientationZ[w.node],w:b.orientationW[w.node]};
  let tangent=rotateVectorByQuaternion(q,{x:0,y:0,z:1});
  if(Math.abs(XYZ.reduce((sum,a,i)=>sum+tangent[a.toLowerCase()]*r.normal[i],0))>1-1e-8)tangent=rotateVectorByQuaternion(q,{x:1,y:0,z:0});
  for(let axis=0;axis<3;axis++){
   const key=XYZ[axis],center=(1-w.t)*b[key.toLowerCase()][w.node]+w.t*b[key.toLowerCase()][w.node+1];
   r.surfaceContactPoint[axis]=center-w.radius*r.normal[axis];r.surfaceAxialTangent[axis]=tangent[key.toLowerCase()];
   e.wall[key.toLowerCase()].fill(r.surfaceContactPoint[axis]);e.wall['previous'+key].fill(r.surfaceContactPoint[axis]);
  }
  e.wall.z[1]+=1;e.wall.previousZ[1]+=1;
  Object.assign(e.constraint,{innerBody:b,outerBody:e.wall,axialFriction:mu,circumferentialFriction:mu});
  const s=buildKirchhoffSurfaceFriction(e.constraint,r,dt,e.surface);
  if(!s.supported)throw new Error('Unsupported wall witness surface friction: '+s.reason);
  e.modeKey=selected.key;e.mode=selected.mode;e.witness=w;e.contact=contact;e.rowStart=out.rows.length;e.normal=normal;
  for(let axis=0;axis<2;axis++){
   const src=s.rows[axis],row=e.rows[axis];Object.assign(row,{kind:'wall-witness-friction',alpha:0,lambda:src.lambda,strain:src.strain,lower:-Infinity,upper:Infinity});
   row.gradients=src.gradients.filter(g=>g.side===0).map(g=>({side,dof:g.dof,value:g.value}));
   captureKirchhoffWallSurfaceReaction(b,row.gradients,side,e.frozen[axis]);out.rows.push(row);
  }
  if(transport(e,side)){e.transportIndex=out.rows.length;out.rows.push(e.transportRow);}else e.transportIndex=-1;
  out.groups.push({kind:'coulomb-disk',mu:Array.from(s.group.mu),normalLambda:contact.normalLambda,normalContact:contact,normalRow:normal,rowIndices:[e.rowStart,e.rowStart+1]});out.entries.push(e);
 }
 return out;
}
export function appendKirchhoffWallWitnessFriction(batch,rows,groups){
 if(batch.appended||rows===batch.rows||groups===batch.groups)throw new Error('Wall friction batch must append once to independent collectors');
 batch.rowOffset=rows.length;rows.push(...batch.rows);for(const g of batch.groups)groups.push({...g,rowIndices:g.rowIndices.map(i=>i+batch.rowOffset)});batch.appended=true;
}
export function commitKirchhoffWallWitnessFriction(batch,increments,scale){
 if(!batch.appended||batch.committed||!Number.isFinite(scale)||scale<0||scale>1)throw new Error('Uncommitted wall friction batch and common scale required');
 for(const e of batch.entries)for(let axis=0;axis<2;axis++){
  const delta=scale*increments[batch.rowOffset+e.rowStart+axis];e.lambda[axis]=e.rows[axis].lambda+delta;
  if(!Number.isFinite(delta)||!Number.isFinite(e.lambda[axis]))throw new RangeError('Nonfinite wall friction increment');
 }
 for(const e of batch.entries)if(e.transportIndex>=0){
  const delta=scale*increments[batch.rowOffset+e.transportIndex];
  if(!Number.isFinite(delta)||delta < -1||delta > 0)throw new RangeError('Invalid wall friction transport increment');
  e.transportDelta=delta;
 }
 for(const e of batch.entries){
  if(e.transportIndex>=0)recordKirchhoffWallSurfaceReaction(e.witness.ledger,e.transport.wrenches,e.transportDelta);
  for(let axis=0;axis<2;axis++)recordKirchhoffWallSurfaceReaction(e.witness.ledger,e.frozen[axis],e.lambda[axis]-e.rows[axis].lambda);
  e.contact.tangentLambda.set(e.lambda);e.contact.tangentU.set(e.surface.axes[0]);e.contact.tangentV.set(e.surface.axes[1]);
 }
 batch.committed=true;
}
export function measureKirchhoffWallWitnessFriction(component,batch){
 if(batch.component!==component)throw new Error('Wall friction batch belongs to another component');
 const measured=buildKirchhoffWallWitnessFriction(component,batch.normalRows,batch.dt,batch.measurementBatch??={});
 const out=batch.measurement??={};out.maximumDisplacementResidualMm=out.maximumConeViolation=0;out.finite=true;out.pendingTransport=0;out.maximumTransportPositionMm=out.maximumTransportAngleRad=0;out.contactCount=measured.entries.length;
 const bodies=kirchhoffComponentBodies(component), corrections=out._transportCorrections??=[];
 for(let side=0;side<bodies.length;side++){
  if(corrections[side]?.length!==bodies[side].count*6)corrections[side]=new Float64Array(bodies[side].count*6);
  corrections[side].fill(0);
 }
 for(const e of measured.entries){
  if(e.hasTransport){
   out.pendingTransport++;
   for(const g of e.transportRow.gradients)corrections[g.side][g.dof]-=g.value;
  }
  const kkt=evaluateKirchhoffSurfaceFrictionKKT(e.surface.rows.map(r=>r.lambda),e.surface.rows.map(r=>r.strain),e.contact.normalLambda,e.surface.group.mu,e.residual);
  out.maximumDisplacementResidualMm=Math.max(out.maximumDisplacementResidualMm,kkt.residualMm);out.maximumConeViolation=Math.max(out.maximumConeViolation,kkt.coneViolation);
  out.finite&&=Number.isFinite(kkt.residualMm)&&Number.isFinite(kkt.coneViolation);
 }
 // Measure the retained transport correction in the same physical units and
 // active/hard-frame mobility masks as apply. This does not trim the journal:
 // even sub-tolerance carriers remain in the next assembled global block.
 for(let side=0;side<bodies.length;side++){
  const body=bodies[side],values=corrections[side],start=body.activeStart??0,end=body.activeEnd??body.count-1;
  for(let node=start;node<=end;node++){
   const offset=node*6;
   const position=body.inverseMass[node]*Math.hypot(values[offset],values[offset+1],values[offset+2]);
   const angle=node===end||body.orientationControlCompliance===0&&node===body.orientationControlSegment ? 0 :
    Math.hypot(values[offset+3]*body.inverseInertia1[node],values[offset+4]*body.inverseInertia2[node],values[offset+5]*body.inverseInertia3[node]);
   out.maximumTransportPositionMm=Math.max(out.maximumTransportPositionMm,position);
   out.maximumTransportAngleRad=Math.max(out.maximumTransportAngleRad,angle);
  }
 }
 out.finite&&=Number.isFinite(out.maximumTransportPositionMm)&&Number.isFinite(out.maximumTransportAngleRad);
 out._batch=measured;out.maximumMeritMm=measureKirchhoffFrictionMerit(measured,out._merit??={}).maximumMm;
 out.finite&&=Number.isFinite(out.maximumMeritMm);
 return out;
}

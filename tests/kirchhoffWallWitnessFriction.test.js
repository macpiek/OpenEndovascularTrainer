import {beginKirchhoffWallWitnessFrictionModes as beginModes,evaluateKirchhoffWallWitnessFrictionCandidate as decideModes,prepareKirchhoffWallWitnessFrictionRetry as retryModes} from '../src/physics/kirchhoffWallWitnessFrictionMode.js';
import test from 'node:test';import assert from 'node:assert/strict';
import {BufferGeometry,Float32BufferAttribute} from 'three';import {MeshBVH} from 'three-mesh-bvh';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {beginKirchhoffWallWitnessStep as begin,collectKirchhoffWallWitnessRows as collect,commitKirchhoffWallWitnessMultipliers as normals} from '../src/physics/kirchhoffWallWitnessRows.js';
import {buildKirchhoffWallWitnessFriction as build,appendKirchhoffWallWitnessFriction as append,commitKirchhoffWallWitnessFriction as commit,measureKirchhoffWallWitnessFriction as measure} from '../src/physics/kirchhoffWallWitnessFriction.js';
import {solveKirchhoffCoupledSystem,applyKirchhoffCoupledCorrection} from '../src/physics/kirchhoffCoupledSystem.js';
const dt=1/120;
function fixture(){
 const geometry=new BufferGeometry().setAttribute('position',new Float32BufferAttribute([-2,-2,0,2,-2,0,0,2,0,-2,-2,-.1,2,-2,-.1,0,2,-.1],3));geometry.boundsTree=new MeshBVH(geometry);
 const field={fallbackGeometry:geometry},world=new EndovascularPhysicsWorld(),body=world.createRod('friction-body',2,.2,{radius:.5});
 body.setNodePosition(0,.1,.1,.4);body.setNodePosition(1,.3,.1,.4);body.wallActive[0]=1;body.wallFaceIndex[0]=0;body.wallT[0]=.5;body.wallGap[0]=-.1;body.wallCompliance=0;
 body.wallStaticFriction=body.wallKineticFriction=.2;
 const c={bodies:[body],kirchhoffContacts:[]};begin(c);return {body,field,c};
}
test('distinct witnesses have independent Coulomb loads and only real-body surface torque rows',()=>{
 const {body,field,c}=fixture();collect(c,field,[],dt);normals(c,[1],1);
 body.wallFaceIndex[0]=1;body.wallGap[0]=0;collect(c,field,[],dt);normals(c,[0,2],1);
 const rows=collect(c,field,[],dt),batch=build(c,rows,dt);assert.equal(batch.groups.length,2);assert.deepEqual(batch.groups.map(g=>g.normalLambda),[1,2]);
 assert.notEqual(batch.entries[0].contact,batch.entries[1].contact);
 batch.entries[0].contact.tangentLambda[0]=.1;
 assert.equal(batch.entries[1].contact.tangentLambda[0],0);
 assert.equal(batch.entries[0].contact.tangentLambda,batch.entries[0].witness.ledger.tangentLambda);
 const groups=[];append(batch,rows,groups);
 assert.equal(groups[0].normalRow,rows[0]);assert.equal(groups[1].normalRow,rows[1]);
 assert.ok(batch.rows.every(row=>row.gradients.every(g=>g.side===0)));
 assert.ok(batch.rows.some(row=>row.gradients.some(g=>g.dof%6>=3&&g.value!==0)),'radius torque is in real rod DOFs');
 assert.throws(()=>append(batch,rows,groups),/once/);
 body.wallKineticFriction=.1;assert.throws(()=>build(c,rows,dt),/controller/);
});
test('actual global block solves surface slip and torque with the witness normal in the same Coulomb cone',()=>{
 const {body,field,c}=fixture();body.previousY[0]-=.03;body.previousY[1]-=.03;
 const rows=collect(c,field,[],dt),batch=build(c,rows,dt),groups=[];append(batch,rows,groups);
 const r=solveKirchhoffCoupledSystem(c,dt,{additionalRows:rows,groups,resolveNormalLoads:true,activeCondensation:true,simultaneousCoulomb:true,tolerance:1e-9});
 assert.ok(r.diagnostics.converged,JSON.stringify(r.diagnostics));
 assert.ok(r.inner.correction.some((v,i)=>i%6>=3&&Math.abs(v)>1e-8),'surface friction rotates the rod');
 applyKirchhoffCoupledCorrection(c,r);normals(c,r.additionalIncrement,r.scale);commit(batch,r.additionalIncrement,r.scale);
 const e=batch.entries[0],lambda=e.contact.tangentLambda,n=e.contact.normalLambda;
 assert.ok(n>0);assert.ok(Math.hypot(...lambda)<=.2*n+1e-8);assert.ok(Math.hypot(...lambda)>0);
 assert.ok(e.witness.ledger.wrenches.some(w=>Math.abs(w.mx)+Math.abs(w.my)+Math.abs(w.mz)>0));
 assert.equal(measure(c,batch).finite,true);
 assert.throws(()=>commit(batch,r.additionalIncrement,r.scale),/Uncommitted/);
});

test('basis transport preserves world tangent force and measurement does not mutate frozen apply rows',()=>{
 const {body,field,c}=fixture();collect(c,field,[],dt);normals(c,[1],1);const rows=collect(c,field,[],dt),first=build(c,rows,dt),e=first.entries[0];
 e.contact.tangentLambda.set([.02,-.01]);e.contact.tangentU.set(e.surface.axes[0]);e.contact.tangentV.set(e.surface.axes[1]);
 const worldForce=Array.from(e.contact.tangentU,(u,i)=>u*.02-e.contact.tangentV[i]*.01);
 // Rotate the material frame around the fixed wall normal, changing only the basis.
 body.orientationZ[0]=Math.sin(.35);body.orientationW[0]=Math.cos(.35);
 const next=build(c,rows,dt),n=next.entries[0];
 const transported=Array.from(n.surface.axes[0],(u,i)=>u*n.rows[0].lambda+n.surface.axes[1][i]*n.rows[1].lambda);
 transported.forEach((value,i)=>assert.ok(Math.abs(value-worldForce[i])<1e-12));
 const frozen=JSON.stringify(next.rows),journal=JSON.stringify(n.frozen);
 const result=measure(c,next);
 assert.ok(Number.isFinite(result.maximumMeritMm));assert.equal(result.finite,true);
 assert.equal(JSON.stringify(next.rows),frozen);assert.equal(JSON.stringify(n.frozen),journal);
 assert.notEqual(result._batch.entries[0].surface,next.entries[0].surface);
});
test('zero friction yields empty batch and cannot silently discard a loaded tangent history',()=>{
 const {body,field,c}=fixture();const rows=collect(c,field,[],dt);
 body.wallStaticFriction=body.wallKineticFriction=0;
 const batch=build(c,rows,dt);assert.equal(batch.rows.length,0);assert.equal(batch.groups.length,0);
 const result=measure(c,batch);assert.equal(result.maximumMeritMm,0);assert.equal(result.maximumDisplacementResidualMm,0);
 rows[0].witness.ledger.tangentLambda[0]=.1;
 assert.throws(()=>build(c,rows,dt),/Release loaded/);
});

for(const fraction of [1,.25])test(`finite edge normal change releases transport defect in actual global block at scale ${fraction}`,()=>{
 const {body,field,c}=fixture();body.previousX[0]-=.03;body.previousX[1]-=.03;
 let rows=collect(c,field,[],dt),batch=build(c,rows,dt),groups=[];append(batch,rows,groups);
 const options={additionalRows:rows,groups,resolveNormalLoads:true,activeCondensation:true,simultaneousCoulomb:true,tolerance:1e-9};
 let solved=solveKirchhoffCoupledSystem(c,dt,options);
 assert.ok(solved.diagnostics.converged);applyKirchhoffCoupledCorrection(c,solved);normals(c,solved.additionalIncrement,solved.scale);commit(batch,solved.additionalIncrement,solved.scale);
 const witness=batch.entries[0].witness;
 assert.ok(Math.hypot(...witness.ledger.tangentLambda)>0);
 // Move the material point beyond the finite triangle's edge while retaining
 // the same triangle witness. Its distance normal now tilts from the face normal.
 for(let i=0;i<2;i++){body.x[i]+=1.2;body.previousX[i]+=1.2;}
 rows=collect(c,field,[],dt);assert.equal(witness.geometry.feature,'edge');
 batch=build(c,rows,dt);const e=batch.entries[0];assert.equal(e.hasTransport,true);
 const expected=new Map(witness.ledger.wrenches.map(v=>[v.node,{...v}]));
 const normalFrozen=rows[0].witness.frozen.map(v=>({...v}));
 const defect=e.transport.wrenches.map(v=>({...v}));
 assert.ok(defect.some(v=>Math.abs(v.fz)>1e-10),'projecting a loaded force would discard its out-of-plane component');
 groups=[];append(batch,rows,groups);
 solved=solveKirchhoffCoupledSystem(c,dt,{...options,additionalRows:rows,groups});
 assert.ok(solved.diagnostics.converged,JSON.stringify(solved.diagnostics));
 assert.ok(Math.abs(solved.additionalIncrement[batch.rowOffset+e.transportIndex]+1)<1e-10);
 const scale=solved.scale*fraction;
 for(const [frozen,delta] of [[normalFrozen,solved.additionalIncrement[0]*scale],[defect,solved.additionalIncrement[batch.rowOffset+e.transportIndex]*scale],
  ...e.frozen.map((f,i)=>[f,solved.additionalIncrement[batch.rowOffset+e.rowStart+i]*scale])]){
  for(const v of frozen)for(const key of ['fx','fy','fz','mx','my','mz'])expected.get(v.node)[key]+=v[key]*delta;
 }
 // Apply and commit the exact same partial correction; no force-only clipping.
 applyKirchhoffCoupledCorrection(c,solved,scale);normals(c,solved.additionalIncrement,scale);commit(batch,solved.additionalIncrement,scale);
 for(const v of witness.ledger.wrenches)for(const key of ['fx','fy','fz','mx','my','mz'])assert.ok(Math.abs(v[key]-expected.get(v.node)[key])<1e-11,`${key} retained exactly`);
 assert.ok(witness.ledger.wrenches.every(v=>Object.values(v).every(Number.isFinite)));
 if(fraction<1){
  const measured=measure(c,batch);assert.ok(measured.pendingTransport>0,'partial transport remains pending');
  const gradients=new Float64Array(body.count*6);
  for(const entry of measured._batch.entries)if(entry.hasTransport)for(const g of entry.transportRow.gradients)gradients[g.dof]-=g.value;
  const expectedPosition=Math.max(...Array.from({length:body.count},(_,node)=>body.inverseMass[node]*Math.hypot(...gradients.slice(node*6,node*6+3))));
  const expectedAngle=Math.hypot(gradients[3]*body.inverseInertia1[0],gradients[4]*body.inverseInertia2[0],gradients[5]*body.inverseInertia3[0]);
  assert.ok(Math.abs(measured.maximumTransportPositionMm-expectedPosition)<1e-14);
  assert.ok(Math.abs(measured.maximumTransportAngleRad-expectedAngle)<1e-14);
  const journal=JSON.stringify(witness.ledger.wrenches);
  body.inverseMass.fill(0);body.orientationControlSegment=0;body.orientationControlCompliance=0;
  const prescribed=measure(c,batch);
  assert.equal(prescribed.maximumTransportPositionMm,0);assert.equal(prescribed.maximumTransportAngleRad,0);
  assert.ok(prescribed.pendingTransport>0);assert.equal(JSON.stringify(witness.ledger.wrenches),journal,'mobility gates never erase reaction');
 }
});

test('opt-in unequal provider uses exact static and zero kinetic cones in the real global block',()=>{
 const {body,field,c}=fixture();body.wallStaticFriction=.6;body.wallKineticFriction=0;
 beginModes(c,{dt,step:1,displacementToleranceMm:1e-8});
 let rows=collect(c,field,[],dt),batch=build(c,rows,dt);assert.deepEqual(batch.groups[0].mu,[.6,.6]);
 // Explicit converged-candidate evidence drives only the mode protocol. The
 // actual zero-width cone below is independently solved by the real core.
 const entry=batch.entries[0],evidence={...batch,entries:[{...entry,contact:{normalLambda:1},surface:{group:{mu:[.6,.6]},rows:[{lambda:-.6,strain:.2},{lambda:0,strain:0}]}}]};
 const decision=decideModes(c,evidence,{converged:true});assert.equal(decision.status,'restart');retryModes(c,decision);
 rows=collect(c,field,[],dt);batch=build(c,rows,dt);assert.equal(batch.rows.length,2);assert.deepEqual(batch.groups[0].mu,[0,0]);
 const groups=[];append(batch,rows,groups);
 const solved=solveKirchhoffCoupledSystem(c,dt,{additionalRows:rows,groups,resolveNormalLoads:true,activeCondensation:true,simultaneousCoulomb:true,tolerance:1e-9});
 assert.ok(solved.diagnostics.converged);applyKirchhoffCoupledCorrection(c,solved);normals(c,solved.additionalIncrement,solved.scale);commit(batch,solved.additionalIncrement,solved.scale);
 assert.deepEqual([...batch.entries[0].contact.tangentLambda],[0,0]);
 assert.equal(body.wallStaticFriction,.6);assert.equal(body.wallKineticFriction,0);
});

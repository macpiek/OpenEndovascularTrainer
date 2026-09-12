import test from 'node:test';import assert from 'node:assert/strict';
import {BufferGeometry,Float32BufferAttribute} from 'three';import {MeshBVH} from 'three-mesh-bvh';
import {EndovascularPhysicsWorld} from '../src/physics/endovascularPhysicsWorld.js';
import {beginKirchhoffWallWitnessStep as begin,collectKirchhoffWallWitnessRows as collect,commitKirchhoffWallWitnessMultipliers as commit,measureKirchhoffWallWitnessResidual as measure} from '../src/physics/kirchhoffWallWitnessRows.js';
const dt=1/120;
function fixture(){
 const geometry=new BufferGeometry().setAttribute('position',new Float32BufferAttribute([-2,-2,0,2,-2,0,0,2,0,-2,-2,-.1,2,-2,-.1,0,2,-.1],3));geometry.boundsTree=new MeshBVH(geometry);
 const field={fallbackGeometry:geometry},world=new EndovascularPhysicsWorld(),body=world.createRod('witness-body',2,.2,{radius:.5});
 body.setNodePosition(0,.1,.1,.4);body.setNodePosition(1,.3,.1,.4);body.wallActive[0]=1;body.wallFaceIndex[0]=0;body.wallT[0]=.5;body.wallGap[0]=-.1;body.wallCompliance=0;
 const component={bodies:[body]};begin(component);return {body,field,component};
}
test('new face owns zero load while retained face keeps its reaction and preapply journal',()=>{
 const {body,field,component}=fixture(),rows=collect(component,field,[],dt);assert.equal(rows.length,1);
 const frozen=structuredClone(rows[0].witness.frozen);body.x[0]+=.01;
 commit(component,[1],1);assert.deepEqual(rows[0].witness.ledger.wrenches,frozen);
 body.wallFaceIndex[0]=1;body.wallGap[0]=0;
 const next=collect(component,field,[],dt);assert.equal(next.length,2);assert.equal(next[0].lambda,1);assert.equal(next[1].lambda,0);
 assert.ok(next.every(row=>row.kind==='wall-witness'));commit(component,[0,.2],1);
 assert.ok(Math.abs(body.wallLambda[0]-1.2)<1e-6);assert.equal(measure(component).finite,true);
});
test('loss of collision support releases retained reaction in the same block at a partial scale',()=>{
 const {body,field,component}=fixture();collect(component,field,[],dt);commit(component,[1],1);
 body.collisionStartSegment=1;const rows=collect(component,field,[],dt);assert.equal(rows.length,1);assert.equal(rows[0].kind,'wall-release');
 assert.equal(measure(component).pending,1);commit(component,[-1],.5);assert.equal(component._wallWitnessRows.witnesses[0].ledger.lambda,.5);
 collect(component,field,[],dt);commit(component,[-1],1);assert.equal(measure(component).pending,0);assert.equal(body.wallLambda[0],0);
});
test('unrepresented contact and changed material cannot silently discard collisions or loaded force',()=>{
 const {body,field,component}=fixture();body.wallFaceIndex[0]=-1;assert.throws(()=>collect(component,field,[],dt),/represented/);
 body.wallFaceIndex[0]=0;collect(component,field,[],dt);commit(component,[1],1);
 body.materialCoordinate[0]+=.01;assert.throws(()=>collect(component,field,[],dt),/material identity/);
});
test('retained finite geometry and signed branch changes fail explicitly',()=>{
 const {body,field,component}=fixture();collect(component,field,[],dt);commit(component,[1],1);
 body.z.fill(-.2);assert.throws(()=>measure(component),/interior branch/);
 body.z.fill(.4);field.fallbackGeometry.attributes.position.setZ(0,.01);assert.throws(()=>measure(component),/geometry changed/);
});
test('new physical step clears forces without transferring them to another witness',()=>{
 const {field,component}=fixture();collect(component,field,[],dt);commit(component,[1],1);begin(component);
 const rows=collect(component,field,[],dt);assert.equal(rows[0].lambda,0);assert.equal(rows[0].witness.ledger.wrenches.length,0);
 assert.throws(()=>commit(component,[NaN],1),/increment/);
});

test('measurement flags undiscovered trial faces without overwriting frozen rows',()=>{
 const {body,field,component}=fixture();const rows=collect(component,field,[],dt),old=rows[0];
 body.wallFaceIndex[0]=1;body.wallGap[0]=0;
 assert.equal(measure(component).pendingDiscoveries,1);
 assert.equal(component._wallWitnessRows.witnesses.length,1);assert.equal(component._wallWitnessRows.rows[0],old);
 commit(component,[.2],1);
 collect(component,field,[],dt);assert.equal(measure(component).pendingDiscoveries,0);
});

test('unloaded changing discoveries retire in place with bounded storage',()=>{
 const {body,field,component}=fixture();collect(component,field,[],dt);
 const state=component._wallWitnessRows,list=state.witnesses,map=state.lookup;
 for(let i=0;i<200;i++){
  body.wallFaceIndex[0]=i%2;body.wallT[0]=(i%99)/100;
  body.materialCoordinate[0]=i;body.materialCoordinate[1]=i+1;
  collect(component,field,[],dt);
  assert.equal(state.witnesses,list);assert.equal(state.lookup,map);
  assert.equal(list.length,1);assert.equal(map.size,1);
 }
 body.wallActive[0]=0;begin(component);assert.equal(list.length,0);assert.equal(map.size,0);
});
test('loaded old face survives discovery changes and retires only after complete release',()=>{
 const {body,field,component}=fixture();collect(component,field,[],dt);commit(component,[1],1);
 const state=component._wallWitnessRows,old=state.witnesses[0];
 body.wallFaceIndex[0]=1;collect(component,field,[],dt);assert.equal(state.witnesses.length,2);assert.equal(state.witnesses[0],old);
 body.collisionStartSegment=1;collect(component,field,[],dt);assert.equal(state.witnesses.length,1);
 commit(component,[-1],.5);collect(component,field,[],dt);assert.equal(state.witnesses[0],old);
 commit(component,[-1],1);collect(component,field,[],dt);assert.equal(state.witnesses.length,0);
});
test('zero scalar lambda with nonzero applied journal is retained until release',()=>{
 const {body,field,component}=fixture();collect(component,field,[],dt);commit(component,[1],1);
 const state=component._wallWitnessRows,old=state.witnesses[0];
 // Moving the node between equal/opposite force increments leaves a moment
 // in the ledger even when the scalar multiplier returns exactly to zero.
 body.x[0]+=.1;collect(component,field,[],dt);commit(component,[-1],1);
 assert.equal(old.ledger.lambda,0);assert.ok(old.ledger.wrenches.some(w=>w.mx!==0||w.my!==0||w.mz!==0));
 body.collisionStartSegment=1;const rows=collect(component,field,[],dt);
 assert.equal(state.witnesses[0],old);assert.equal(rows[0].kind,'wall-release');
 commit(component,[-1],1);collect(component,field,[],dt);assert.equal(state.witnesses.length,0);
});

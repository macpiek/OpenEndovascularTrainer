import test from 'node:test';
import assert from 'node:assert/strict';
import {BufferGeometry,Float32BufferAttribute} from 'three';
import {createSharedAxisNative,assembleSharedAxisNative,applySharedAxisNativeIncrement,extendSharedAxisNativeRows,
    captureSharedAxisNative,restoreSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {createSharedAxisConstraintRowPool,snapshotSharedAxisConstraintMeasure} from '../src/physics/kirchhoffSharedAxisConstraintRows.js';
import {createSharedAxisVesselWitness} from '../src/physics/kirchhoffSharedAxisVesselWitnesses.js';
import {iterateSharedAxisTimeStep,stepSharedAxis} from '../src/physics/kirchhoffSharedAxisTimeStep.js';
const numeric=m=>({energy:m.energy,force:m.force,torque:m.torque,constraint:m.constraint,rows:m.rows.map(r=>({
    kind:r.kind,id:r.id,gap:r.gap,multiplier:r.multiplier,jacobian:Array.from(r.jacobian),hessian:r.geometricHessian?.slice(),
    forceDofs:r.extraForceDofs?.slice(),forceJ:r.extraForceJacobian?.slice()}))});
function fixture(){return createSharedAxisNative({tools:[{id:'wire',insertion:35},{id:'catheter',insertion:20}],maxBendAngle:Math.PI/4});}
function compare(s,bank,withTangent=true) {
 s.mechanicalAssemblyCache=null;
 const ref=assembleSharedAxisNative(s,{withTangent,wasmMaterial:true,reuseConstraintWork:true}),gradient=s.chain.gradient.slice();
 s.mechanicalAssemblyCache=null;
 const actual=assembleSharedAxisNative(s,{withTangent,wasmMaterial:true,reuseConstraintWork:true,rowStorage:bank});
 assert.deepEqual(numeric(actual),numeric(ref));assert.deepEqual(s.chain.gradient,gradient);return actual;
}
test('three banks preserve base and uncorrected trial while repeated corrected poses reuse numeric storage',()=>{
 const s=fixture(),pool=createSharedAxisConstraintRowPool(),saved=captureSharedAxisNative(s);
 s.multipliers.fill(.2);const publicMeasure=assembleSharedAxisNative(s),publicSnapshot=numeric(publicMeasure);
 const base=compare(s,pool.acquire()),baseSnapshot=numeric(base);
 const change=()=>applySharedAxisNativeIncrement(s,Float64Array.from(s.chain.gradient,(_,i)=>.0001*Math.sin(i)),new Float64Array(s.multipliers.length));
 change();const trial=compare(s,pool.acquire([base.rows]),false),trialSnapshot=numeric(trial);
 const scratch=pool.acquire([base.rows,trial.rows]);compare(s,scratch);
 const rows=scratch.rows.slice(),jacobians=rows.map(r=>r.jacobian),hessians=scratch.hessians.slice();
 for(let i=0;i<30;i++) {
  change();s.multipliers.fill(i%2?.2:0);
  const bank=pool.acquire([base.rows,trial.rows]);assert.equal(bank,scratch);compare(s,bank,i%3!==0);
  assert.deepEqual(numeric(base),baseSnapshot);assert.deepEqual(numeric(trial),trialSnapshot);
  assert.deepEqual(numeric(publicMeasure),publicSnapshot);
  rows.forEach((r,k)=>{assert.equal(bank.rows[k],r);assert.equal(r.jacobian,jacobians[k]);if(hessians[k])assert.equal(bank.hessians[k],hessians[k]);});
 }
 assert.equal(pool.banks.length,3);restoreSharedAxisNative(s,saved);compare(s,scratch);
 assert.deepEqual(numeric(base),baseSnapshot);
});
test('time step cancellation and completion release private contact storage and preserve atomic state',()=>{
 const s=fixture();s.loads[s.layout.positions.at(-1)+1]=.01;
 const before=captureSharedAxisNative(s),options={reuseRowBuffers:true,wasmMaterial:true,reuseConstraintWork:true,reuseMatrixAssembly:true};
 const iterator=iterateSharedAxisTimeStep(s,1/60,options);
 assert.equal(iterator.next().done,false);assert.ok(s.bufferedWallGeometryCache instanceof Map);
 iterator.return();assert.equal(s.bufferedWallGeometryCache,null);assert.deepEqual(captureSharedAxisNative(s),before);
 const result=stepSharedAxis(s,1/60,options);
 assert.ok(result.converged,result.status);assert.equal(s.bufferedWallGeometryCache,null);
});
test('reused rows clear inactive derivatives and update definitions after discovery; observed measures own their data',()=>{
 const s=fixture(),pool=createSharedAxisConstraintRowPool(),bank=pool.acquire();s.multipliers.fill(.3);
 const first=compare(s,bank),snapshot=snapshotSharedAxisConstraintMeasure(first),expected=numeric(snapshot);
 first.rows[0].extraForceDofs=[0];first.rows[0].extraForceJacobian=[13];s.multipliers.fill(0);
 compare(s,bank,false);for(const r of bank.rows){assert.equal(r.geometricHessian,undefined);assert.equal(r.extraForceDofs,undefined);assert.equal(r.extraForceJacobian,undefined);}
 const e=1,dofs=[s.layout.positions[e],s.layout.positions[e+1]].flatMap(p=>[p,p+1,p+2]);
 extendSharedAxisNativeRows(s,[{kind:'wall',edge:e,id:'new-buffered-contact',dofs,evaluate:()=>({gap:1,jacobian:[0,.3,0,0,.7,0]})}]);
 compare(s,bank);assert.equal(bank.rows.length,s.definitions.length);assert.equal(bank.rows.at(-1).id,'new-buffered-contact');
 assert.deepEqual(numeric(snapshot),expected);
});
test('buffered finite-triangle contacts preserve retained rows across feature changes, Hessian promotion and throwing evaluations',()=>{
 const geometry=new BufferGeometry().setAttribute('position',new Float32BufferAttribute([0,0,0,10,0,0,0,10,0],3));
 const field={fallbackGeometry:geometry},s=fixture(),e=1,dofs=[s.layout.positions[e],s.layout.positions[e+1]].flatMap(p=>[p,p+1,p+2]);
 const row=createSharedAxisVesselWitness(field,{kind:'wall',edge:e,id:'triangle-buffer',dofs,witness:{face:0,t:.3}});
 extendSharedAxisNativeRows(s,[row]);s.cacheMechanicalAssembly=true;s.multipliers.fill(.2);
 const pool=createSharedAxisConstraintRowPool(),bank=pool.acquire(),retained=[];
 try {
  for(const point of [[2,2,3],[-2,-1,1],[4,-1,2],[11,-1,1],[6,6,2],[2,2,3]]) {
   s.positions[e]=point.slice();s.positions[e+1]=point.map((v,i)=>v+(i===0?.1:0));s.geometryKey=Symbol();
   const out=compare(s,bank,false),copy=snapshotSharedAxisConstraintMeasure(out);retained.push([copy,numeric(copy)]);
   assert.equal(bank.contacts.get(row).withHessian,false);
   compare(s,bank,true); // First derivatives upgraded to full Hessian at this pose.
   const record=bank.contacts.get(row),hessian=record.output.hessian;
   assert.equal(record.withHessian,true);if(point[0]<0)assert.ok(hessian);
   s.cacheMechanicalAssembly=false;
   assembleSharedAxisNative(s,{withTangent:false,rowStorage:bank});
   assert.equal(record.key,s.geometryKey,'An uncached fallback must preserve the retained Newton geometry');
   assert.equal(record.withHessian,true);
   s.cacheMechanicalAssembly=true;
   const publicRows=assembleSharedAxisNative(s),publicSnapshot=numeric(publicRows);
   const hits=s.wallGeometryCacheHits;compare(s,bank,true);
   assert.ok(s.wallGeometryCacheHits>hits);assert.equal(record.output.hessian,hessian);
   assert.deepEqual(numeric(publicRows),publicSnapshot);
  }
  const key=s.geometryKey,previous=s.positions.map(p=>p.slice());
  s.positions[e]=[2,2,0];s.positions[e+1]=[2.1,2,0];s.geometryKey=Symbol();
  assert.throws(()=>assembleSharedAxisNative(s,{rowStorage:bank}),/surface|finite|nondegenerate|exact edge Jacobian/i);
  assert.equal(bank.contacts.get(row).key,null);
  s.positions=previous;s.geometryKey=key;compare(s,bank);
  for(const [m,copy]of retained)assert.deepEqual(numeric(m),copy);
 }finally{geometry.dispose();}
});

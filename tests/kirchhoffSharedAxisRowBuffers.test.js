import test from 'node:test';
import assert from 'node:assert/strict';
import {BufferGeometry,Float32BufferAttribute} from 'three';
import {createSharedAxisNative,assembleSharedAxisNative,applySharedAxisNativeIncrement,extendSharedAxisNativeRows,
    captureSharedAxisNative,restoreSharedAxisNative} from '../src/physics/kirchhoffSharedAxisNative.js';
import {createSharedAxisConstraintRowPool,leaseSharedAxisConstraintRowPool,snapshotSharedAxisConstraintMeasure} from '../src/physics/kirchhoffSharedAxisConstraintRows.js';
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

test('leased banks reuse numeric buffers across changed topology without retaining owners or sharing live rows',()=>{
 const first=leaseSharedAxisConstraintRowPool(),other=leaseSharedAxisConstraintRowPool();
 assert.notEqual(first.pool,other.pool);
 const s=fixture(),bank=first.pool.acquire(),live=compare(s,other.pool.acquire());
 s.multipliers.fill(.2);compare(s,bank);
 const row=bank.rows[0],jacobian=row.jacobian,hessian=bank.hessians[0],liveCopy=numeric(live);
 const observed=snapshotSharedAxisConstraintMeasure(compare(s,bank)),observedCopy=numeric(observed);
 first.release();first.release();
 assert.equal(bank.definitions[0],undefined);assert.equal(row.dofs,undefined);
 assert.equal(bank.geometryCache.geometry.positions,null);assert.equal(bank.contacts.size,0);
 const next=leaseSharedAxisConstraintRowPool();assert.equal(next.pool,first.pool);
 try {
  for(const insertion of [15,55,20]) {
   const t=createSharedAxisNative({tools:[{id:'wire',insertion}]});t.multipliers.fill(.3);
   const reused=next.pool.acquire();compare(t,reused);
   assert.equal(reused.rows[0],row);assert.equal(row.jacobian,jacobian);assert.equal(reused.hessians[0],hessian);
  }
  assert.deepEqual(numeric(live),liveCopy);assert.deepEqual(numeric(observed),observedCopy);
 }finally{next.release();other.release();}
});

test('native contact output buffers survive a lease while keys and custom owners do not',()=>{
 const first=leaseSharedAxisConstraintRowPool(),concurrent=leaseSharedAxisConstraintRowPool();
 const s=fixture(),other=fixture();s.cacheMechanicalAssembly=other.cacheMechanicalAssembly=true;
 s.multipliers.fill(.2);other.multipliers.fill(.3);
 const bank=first.pool.acquire(),otherBank=concurrent.pool.acquire();
 compare(s,bank);compare(other,otherBank);
 const index=s.definitions.findIndex(r=>r.subtype==='bend-limit'),definition=s.definitions[index];
 const output=bank.contacts.get(definition).output,independent=otherBank.contacts.get(other.definitions[index]).output;
 assert.notEqual(output,independent);assert.notEqual(output.contact.jacobian,independent.contact.jacobian);
 const otherCopy=structuredClone(independent);
 first.release();assert.equal(bank.contacts.size,0);
 assert.ok(bank.geometryCache.contactOutputOwners.every(owner=>owner===undefined));
 const next=leaseSharedAxisConstraintRowPool();assert.equal(next.pool,first.pool);
 try{
  const t=fixture();t.cacheMechanicalAssembly=true;t.multipliers.fill(-.4);
  for(let i=0;i<t.positions.length;i++)t.positions[i][1]=.05*Math.sin(i);
  t.geometryKey=Symbol('new-owner');const reused=next.pool.acquire();compare(t,reused);
  assert.equal(reused.contacts.get(t.definitions[index]).output,output);
  assert.equal(reused.contacts.has(definition),false);
  assert.deepEqual(independent,otherCopy,'A concurrent live lease owns its scratch');
  // A custom sampler can use its storage for owner-bearing scratch. Without
  // the native numeric-only promise that output must die with the lease.
  const def=t.definitions[index],custom=input=>{
   const storage=input.contactStorage??{contact:{}};storage.owner=input.state;
   return Object.assign(storage.contact,{gap:1,jacobian:new Array(9).fill(0),hessian:undefined});
  };
  custom.contactStorageSupported=true;
  t.definitions[index]={...def,evaluate:custom};t.geometryKey=Symbol('custom');compare(t,reused);
  const customOutput=reused.contacts.get(t.definitions[index]).output;
  assert.equal(customOutput.owner,t);assert.notEqual(customOutput,output);
  assert.equal(reused.geometryCache.contactOutputs.includes(customOutput),false);
 }finally{next.release();concurrent.release();}
});

test('different definitions at the same row index cannot overwrite a live cached contact',()=>{
 const a=fixture(),b=fixture(),pool=createSharedAxisConstraintRowPool();
 for(const s of [a,b]){s.cacheMechanicalAssembly=true;s.multipliers.fill(.2);}
 for(let i=0;i<b.positions.length;i++)b.positions[i][1]=.1*Math.sin(i);
 b.geometryKey=Symbol('other-pose');
 const bank=pool.acquire(),first=compare(a,bank),saved=numeric(first),index=a.definitions.findIndex(r=>r.subtype==='bend-limit');
 const otherBank=pool.acquire([first.rows]);compare(b,otherBank);
 assert.notEqual(bank.contacts.get(a.definitions[index]).output,otherBank.contacts.get(b.definitions[index]).output);
 assert.deepEqual(numeric(first),saved);
 assert.deepEqual(numeric(compare(a,bank)),saved,'Returning to the first pose must use its original exact derivatives');
});

test('interleaved timesteps retain exclusive contact banks through cancellation and acceptance',()=>{
 const a=fixture(),b=fixture(),reference=fixture();
 for(const s of [a,b,reference])s.loads[s.layout.positions.at(-1)+1]=.01;
 const options={reuseRowBuffers:true,wasmMaterial:true,reuseConstraintWork:true,reuseMatrixAssembly:true};
 const before=captureSharedAxisNative(a),first=iterateSharedAxisTimeStep(a,1/60,options),second=iterateSharedAxisTimeStep(b,1/60,options);
 assert.equal(first.next().done,false);assert.equal(second.next().done,false);
 assert.notEqual(a.constraintRowPool,b.constraintRowPool);
 const secondPool=b.constraintRowPool;first.return();
 assert.deepEqual(captureSharedAxisNative(a),before);assert.equal(b.constraintRowPool,secondPool);
 const expected=stepSharedAxis(reference,1/60,options);
 let next;do{next=second.next();}while(!next.done);
 assert.ok(next.value.converged);assert.ok(expected.converged);
 const {geometryKey:actualKey,...actualPose}=captureSharedAxisNative(b);
 const {geometryKey:referenceKey,...referencePose}=captureSharedAxisNative(reference);
 assert.deepEqual(actualPose,referencePose);
 assert.deepEqual(b.acceptedWallGaps,reference.acceptedWallGaps);
 assert.equal(b.constraintRowPool,undefined);assert.equal(a.constraintRowPool,undefined);
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

test('rebound row storage replaces custom metadata and restores owned numerical fields',()=>{
 const s=fixture(),pool=createSharedAxisConstraintRowPool(),bank=pool.acquire();
 const def=s.definitions[0];def.extraLabel={owner:s};
 def.gap=NaN;def.multiplier=NaN;def.jacobian=[NaN];def.penalty=NaN;
 const first=compare(s,bank),row=first.rows[0],jacobian=row.jacobian;
 assert.equal(row.extraLabel,def.extraLabel);assert.equal(row.penalty,1);
 const next=fixture();compare(next,bank);
 assert.equal(bank.rows[0],row);assert.equal(row.jacobian,jacobian);
 assert.equal(row.extraLabel,undefined);assert.equal(row.penalty,1);
 assert.equal(row.multiplier,0);assert.ok(Number.isFinite(row.gap));
});

for(const buffered of [false,true])test(`contact validation rejects missing and nonfinite derivatives, including retained cache values (buffered=${buffered})`,()=>{
 const s=fixture(),pool=createSharedAxisConstraintRowPool(),bank=buffered?pool.acquire():null;
 const e=1,dofs=[s.layout.positions[e],s.layout.positions[e+1]].flatMap(p=>[p,p+1,p+2]);
 let output={gap:1,jacobian:[0,.3,0,0,.7,0]};
 const sample=()=>output;sample.contactOutputOwned=true;
 const def={kind:'wall',edge:e,id:'validation',dofs,evaluate:sample};
 extendSharedAxisNativeRows(s,[def]);s.cacheMechanicalAssembly=true;
 const run=()=>assembleSharedAxisNative(s,{rowStorage:bank,withTangent:false});
 run();
 // Even an owned result may be corrupted by a caller. A cache hit must not
 // bypass the finite derivative certificate or preserve its valid pose key.
 for(const bad of [NaN,Infinity,undefined]) {
  output.jacobian[1]=bad;
  assert.throws(run,/exact edge Jacobian/);
  if(buffered)assert.equal(bank.contacts.get(def).key,null);
  output.jacobian[1]=.3;run();
 }
 delete output.jacobian[1];assert.throws(run,/exact edge Jacobian/);
 output.jacobian[1]=.3;run();
 for(const bad of [null,{gap:NaN,jacobian:Array(6).fill(0)},{gap:1,jacobian:[0,0]}]) {
  output=bad;s.geometryKey=Symbol();assert.throws(run,/exact edge Jacobian/);
 }
 output={gap:1,jacobian:Float64Array.of(0,.3,0,0,.7,0)};s.geometryKey=Symbol();run();
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

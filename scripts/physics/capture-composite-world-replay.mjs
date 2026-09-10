/** Data-only World capture/import. Never installs a solver or restores executable
 * World instances. CLI: --capture --source-root FROZEN_ROOT --output DIRECTORY;
 * --verify --input DIRECTORY; --self-test. All snapshot buffers own their bytes. */
import fs from 'node:fs';
import path from 'node:path';
import os from 'node:os';
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { pathToFileURL, fileURLToPath } from 'node:url';
const CLASS=Symbol('capturedClass'),SCHEMA='oet-world-physical-replay-v1';
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
const json=value=>JSON.stringify(value);
const bytes=view=>new Uint8Array(view.buffer,view.byteOffset,view.byteLength);
const typed={Int8Array,Uint8Array,Uint8ClampedArray,Int16Array,Uint16Array,Int32Array,Uint32Array,Float32Array,Float64Array,BigInt64Array,BigUint64Array};

export function encodeReplayData(value,external=new Map()) {
 const nodes=[],seen=new Map();
 function visit(v) {
  if(v===undefined)return {$undefined:true};
  if(typeof v==='number'&&(!Number.isFinite(v)||Object.is(v,-0)))return {$number:Object.is(v,-0)?'-0':String(v)};
  if(typeof v==='bigint')return {$bigint:String(v)};
  if(typeof v==='function')return {$function:v.name||'anonymous',callable:false};
  if(v===null||typeof v!=='object')return v;
  if(external.has(v))return {$external:external.get(v)};
  if(Object.isFrozen(v)&&v.$function&&v.callable===false)return {...v};
  if(seen.has(v))return {$ref:seen.get(v)};
  const id=nodes.length;seen.set(v,id);nodes.push(null);let node;
  if(ArrayBuffer.isView(v)) {
   const raw=bytes(v);node={kind:'buffer',type:v.constructor.name,length:v.length??null,byteLength:raw.length,sha256:sha(raw),base64:Buffer.from(raw).toString('base64')};
  } else if(v instanceof ArrayBuffer) {const raw=new Uint8Array(v);node={kind:'buffer',type:'ArrayBuffer',length:null,byteLength:raw.length,sha256:sha(raw),base64:Buffer.from(raw).toString('base64')};}
  else if(Array.isArray(v))node={kind:'array',items:Array.from(v,visit)};
  else if(v instanceof Map)node={kind:'map',entries:[...v].map(([a,b])=>[visit(a),visit(b)])};
  else if(v instanceof Set)node={kind:'set',values:[...v].map(visit)};
  else node={kind:'object',className:v[CLASS]??v.constructor?.name??'Object',properties:Object.fromEntries(Object.keys(v).sort().map(k=>[k,visit(v[k])]))};
  nodes[id]=node;return {$ref:id};
 }
 const root=visit(value),payload={root,nodes};
 return {schema:SCHEMA,byteOrder:os.endianness(),contentSha256:sha(json(payload)),...payload};
}
export function importReplayData(envelope) {
 assert.equal(envelope.schema,SCHEMA);assert.equal(envelope.byteOrder,os.endianness(),'Raw numeric byte order must match this host');
 assert.equal(sha(json({root:envelope.root,nodes:envelope.nodes})),envelope.contentSha256,'Snapshot content hash mismatch');
 const values=envelope.nodes.map(node=>{
  if(node.kind==='buffer') {
   const raw=Buffer.from(node.base64,'base64');assert.equal(raw.length,node.byteLength);assert.equal(sha(raw),node.sha256,'Numeric buffer hash mismatch');
   const owned=Uint8Array.from(raw).buffer;
   if(node.type==='ArrayBuffer')return owned;if(node.type==='DataView')return new DataView(owned);
   assert.ok(Object.hasOwn(typed,node.type),'Unsupported typed array');const v=new typed[node.type](owned);assert.equal(v.length,node.length);return v;
  }
  if(node.kind==='array')return [];if(node.kind==='map')return new Map();if(node.kind==='set')return new Set();
  assert.equal(node.kind,'object');const value={};Object.defineProperty(value,CLASS,{value:node.className});return value;
 });
 function read(v) {
  if(v===null||typeof v!=='object')return v;
  if(Object.hasOwn(v,'$ref')){assert.ok(Number.isInteger(v.$ref)&&v.$ref>=0&&v.$ref<values.length);return values[v.$ref];}
  if(v.$undefined)return undefined;if(v.$number)return v.$number==='-0'?-0:Number(v.$number);if(v.$bigint)return BigInt(v.$bigint);
  // External references and functions stay explicit DATA tokens; no code runs.
  if(v.$external||v.$function)return Object.freeze({...v});throw new Error('Unknown replay value');
 }
 envelope.nodes.forEach((node,i)=>{
  const v=values[i];if(node.kind==='array')node.items.forEach(x=>v.push(read(x)));
  if(node.kind==='map')node.entries.forEach(([a,b])=>v.set(read(a),read(b)));
  if(node.kind==='set')node.values.forEach(x=>v.add(read(x)));
  if(node.kind==='object')for(const [k,x] of Object.entries(node.properties))Object.defineProperty(v,k,{value:read(x),writable:true,enumerable:true,configurable:true});
 });
 return read(envelope.root);
}
// Re-encode imported external/function DATA tokens exactly, rather than treating
// them as executable classes. The wire format is still an ordinary graph.
function roundtrip(envelope) {
 const imported=importReplayData(envelope),externals=new Map();
 const visited=new Set();function walk(v){if(v===null||typeof v!=='object'||visited.has(v))return;visited.add(v);
  if(v.$external){externals.set(v,v.$external);return;}if(ArrayBuffer.isView(v)||v instanceof ArrayBuffer)return;
  if(v instanceof Map){for(const [a,b]of v){walk(a);walk(b);}}else if(v instanceof Set){for(const a of v)walk(a);}else Object.values(v).forEach(walk);}
 walk(imported);
 // Functions have been deliberately omitted from selected runtime data below.
 const again=encodeReplayData(imported,externals);assert.equal(again.contentSha256,envelope.contentSha256,'Full graph roundtrip mismatch');
 const a=envelope.nodes.filter(n=>n.kind==='buffer'),b=again.nodes.filter(n=>n.kind==='buffer');assert.deepEqual(a,b);
 return {imported,verification:{numericBuffers:a.length,numericBytes:a.reduce((s,n)=>s+n.byteLength,0),contentSha256:envelope.contentSha256,roundtripContentSha256:again.contentSha256,exactBufferBytes:true}};
}
function project(o,excluded,omissions,prefix) {
 const out={};for(const k of Object.keys(o).sort()) {
  if(excluded.has(k)||typeof o[k]==='function'){omissions.push({path:`${prefix}.${k}`,reason:typeof o[k]==='function'?'executable function; not replay data':'runtime scratch, rendering, or separately supplied anatomy/body reference'});continue;}
  out[k]=o[k];
 }return out;
}
const primitives=o=>Object.fromEntries(Object.entries(o).filter(([,v])=>v===null||['number','boolean','string','undefined'].includes(typeof v)));
function captureFixture(f,phase,command,commandBefore,attempt) {
 const omissions=[];
 const external=new Map([[f.world.contactField,'anatomy.field'],[f.vessel,'fixture.vessel'],[f.wire,'fixture.rodState'],[f.catheter,'fixture.catheter']]);
 for(const body of f.world.bodies)external.set(body,`body:${body.id}`);
 const bodies=f.world.bodies.map(b=>project(b,new Set(['kirchhoffScratch','contactField']),omissions,`body:${b.id}`));
 // Constraint graphs retain every own value, including manifold/portal/load
 // history. References to physical bodies are explicit stable ID tokens.
 const constraints={sheaths:f.world.sheaths,containments:f.world.containments,externalContacts:f.world.toolContacts};
 const catheter=project(f.catheter,new Set(['wire','vessel','physicsBody','tailProgressRef','material','tipMarkerMaterial','shaftMesh','tipMarker','mesh']),omissions,'catheter');
 const transport=project(f.transport,new Set(['rod','sheath','lumenSampler']),omissions,'transport');
 const snapshot=f.snapshot();
 const data={phase,attempt,command:{requested:command,before:commandBefore,after:{wireMm:snapshot.wireMm,catheterMm:snapshot.catheterMm,wireRotation:snapshot.wireRotation,catheterRotation:snapshot.catheterRotation},
  actualWireDeltaMm:snapshot.wireMm-commandBefore.wireMm,actualCatheterDeltaMm:snapshot.catheterMm-commandBefore.catheterMm,dt:f.world.fixedDt,
  preparedOnce:phase==='prepared-before-world-stepFixed',commandsUseFixtureRates:true},
  fixture:{snapshot,config:{...f.config},rodState:{segmentLength:f.wire.segmentLength,nodeStorage:f.wire.nodeStorage},transport,catheter},
  world:{...primitives(f.world),pendingSplitSubstep:f.world._pendingSplitSubstep,lastStepResult:f.world.lastStepResult},bodies,constraints,
  profiles:{guidewire:{id:f.config.guidewireType,shaftScale:f.config.guidewireShaftStiffness,tipScale:f.config.guidewireTipStiffness,tipCoordinate:f.config.guidewireLength,
    appliedRange:[0,f.wireBody.count-1],materialCoordinatesSource:'bodies[guidewire].materialCoordinate',factory:'kirchhoffMaterialProfile + applyKirchhoffMaterialProfile'},
   catheter:{id:f.catheter.type,shaftScale:f.catheter.shaftStiffnessScale,tipScale:f.catheter.tipStiffnessScale,tipCoordinate:f.catheter.progress,
    appliedRange:[f.catheterBody.activeStart,f.catheterBody.activeEnd],materialCoordinatesSource:'catheter._centerlineDistances; active body materialCoordinate',factory:'kirchhoffMaterialProfile + applyKirchhoffMaterialProfile'}},
  originalContactHistory:{provenance:'Owned original World fields at snapshot phase; cached contacts may precede prepared actuation. They are not fresh Joint rows.',
   manifoldContacts:f.world.containments.map(c=>[...c.manifold.contacts()]),kirchhoffContacts:f.world.containments.map(c=>c.kirchhoffContacts)},
  missing:{continuousMaterialWinding:null,unwrappedGuidewireRotation:null,unwrappedCatheterRotation:null,referenceMaterialHistorySampler:null,jointBoundaryIds:null,
   physicalMassCalibrationSI:null,physicalRigidityCalibrationSI:null,acceptedOnlyHistoryCommitCounter:null},omissions};
 return encodeReplayData(data,external);
}
function contactData(q) {
 const out={};for(const k of ['signedDistance','signedGap','distance','penetration','inside','violation','conservative','source','faceIndex','branchId','segmentT','timeOfImpact','insideClearance','capsuleSampleCount'])out[k]=q[k];
 for(const k of ['point','target','closestPoint','normal','inward'])out[k]=Array.from(q[k].values);return out;
}
async function queryImported(data,anatomy,createContactResult) {
 const records=[],counts={};
 for(const body of data.bodies) {
  const start=Math.max(0,body.activeStart,body.collisionStartSegment),end=Math.min(body.activeEnd,body.collisionEndSegment+1,body.segmentCount);
  if(body.collisionEndSegment<body.collisionStartSegment)continue;
  for(let segment=start;segment<end;segment++) {
   const hintFace=body.wallFaceIndex[segment],hintBranch=body.wallActive[segment]?body.wallBranchId[segment]:-1;
   const q=anatomy.field.queryCapsuleSoA(body.x,body.y,body.z,body.nodeRadius,segment,createContactResult(),hintFace,false,false,hintBranch);
   const result=contactData(q);counts[result.source]=(counts[result.source]??0)+1;
   records.push({bodyId:body.id,segment,method:'queryCapsuleSoA',hintFace,hintBranch,knownInside:false,measureInsideClearance:false,
    activationThreshold:data.world.contactActivation,withinActivation:result.signedGap<=data.world.contactActivation,result});
  }
 }
 return {provenance:'Fresh original VesselContactField queries on imported physical bytes and exact World exposure domain; no predictor, correction, contact-row compiler, or solver executed. Includes sleeping bodies for read-only inspection.',
  counts,queriedCapsules:records.length,records,nonWall:'Original lumen/portal/sheath/external fields and manifold records are preserved separately; no replacement query algorithm or invented classification is used.'};
}
function contactHistorySummary(data) {
 const summary={manifold:[],kirchhoff:[]};
 for(const contacts of data.originalContactHistory.manifoldContacts)for(const c of contacts)summary.manifold.push(Object.fromEntries(['id','kind','feature','innerSegment','outerSegment','normalLambda','lastSeenStep'].map(k=>[k,c[k]])));
 for(const contacts of data.originalContactHistory.kirchhoffContacts)for(const c of contacts)summary.kirchhoff.push(Object.fromEntries(['kind','feature','innerSegment','outerSegment','innerT','outerT','gap','penetration','normalLambda'].map(k=>[k,c[k]])));
 return summary;
}
function verifySources(root,manifest) {for(const [file,hash]of Object.entries(manifest.files))assert.equal(sha(fs.readFileSync(path.join(root,file))),hash,`Frozen source changed: ${file}`);}
async function modules(root) {
 const imp=p=>import(pathToFileURL(path.join(root,p)));
 const [fixture,selection,contact]=await Promise.all([imp('tests/helpers/coupledRuntimeFixture.js'),imp('src/physics/coupledSolverSelection.js'),imp('src/physics/collision/vesselContactField.js')]);
 return {...fixture,...selection,...contact};
}
export async function captureWorldReplay(sourceRoot,output) {
 fs.mkdirSync(output,{recursive:true});
 const manifest=JSON.parse(fs.readFileSync(path.join(sourceRoot,'source-manifest.json')));verifySources(sourceRoot,manifest);
 const api=await modules(sourceRoot),anatomy=await api.loadCoupledRuntimeAnatomy(pathToFileURL(sourceRoot+'/'));
 const selection=api.createCoupledSolverSelection('reference');
 const f=api.createCoupledRuntimeFixture({vessel:anatomy.vessel,field:anatomy.field,coupledSystem:selection.coupledSystem,jointMotionMode:selection.jointMotionMode,
  fixedDt:1/120,guidewireTargetMm:318,guidewireShaftStiffness:10,guidewireTipStiffness:4.55,catheterShaftStiffness:25,catheterTipStiffness:5,catheterType:'berenstein'});
 const zero={guidewireAdvance:0,guidewireRotation:0,catheterAdvance:0,catheterRotation:0,catheterType:'berenstein'};
 let attempt=0,lastAccepted=null,lastPrepared=null,lastAfter=null,failure=null,targetPrepared=null;
 const attempts=[],initial=captureFixture(f,'initialized',{...zero},f.snapshot(),0);
 let certified=0;
 function step(command) {
  attempt++;const before=f.snapshot(),worldBefore=f.world.stepCount,commands={...zero,...command},original=f.world.stepFixed;
  let worldResult,result,error,prepared;
  f.world.stepFixed=function(){prepared=captureFixture(f,'prepared-before-world-stepFixed',commands,before,attempt);return worldResult=original.call(this);};
  try {result=f.step(commands);}catch(e){error={name:e.name,message:e.message,stack:e.stack};}finally{f.world.stepFixed=original;}
  if(!prepared)throw new Error('Fixture did not reach the required pre-World hook');
  const stats=f.world.getStats(),after=f.snapshot();
  const accepted=!error&&result?.accepted!==false&&stats.coupledClosureConverged===true&&f.world.stepCount-worldBefore===1;
  const status=error?'reference-threw':result?.accepted===false?result.status:!stats.coupledClosureConverged?'reference-uncertified-closure':f.world.stepCount-worldBefore!==1?'reference-step-accounting-mismatch':'accepted';
  const record={attempt,commands,before,after,accepted,status,worldReturn:worldResult??null,fixtureReturn:result??null,error,
   rawWorldStepDelta:f.world.stepCount-worldBefore,rawFixtureStepDelta:after.executedSteps-before.executedSteps,certifiedExecutedSteps:certified+Number(accepted),
   actualWorldStatus:worldResult?.status??null,legacyVoidReturn:worldResult===undefined,stats};
  lastPrepared=prepared;lastAfter=captureFixture(f,accepted?'after-accepted-fixture-step':'after-uncertified-reference-attempt',commands,before,attempt);
  if(accepted){certified++;lastAccepted=lastAfter;}else failure=record;
  attempts.push(record);
  if(attempt%100===0||!accepted||after.catheterMm>0)process.stderr.write(json({attempt,wireMm:after.wireMm,catheterMm:after.catheterMm,accepted,status})+'\n');
  return accepted;
 }
 try {
  while(f.transport.progress<318-1e-9&&!failure) {
   const remaining=318-f.transport.progress;step({guidewireAdvance:Math.min(1,remaining/(f.config.wireAdvanceRate*f.world.fixedDt))});
  }
  while(f.catheter.progress<9-1e-9&&!failure) {
   const remaining=9-f.catheter.progress;step({catheterAdvance:Math.min(1,remaining/(f.config.catheterAdvanceRate*f.world.fixedDt))});
  }
  if(!failure) {
   // One genuine hold preparation at 318/9, captured before the original
   // reference step. Its result is checked by the same gates as every dt.
   step({});if(!failure)targetPrepared=lastPrepared;
  }
  const envelopes={initial,lastAccepted,terminalPrepared:lastPrepared,terminalAfter:lastAfter,...(targetPrepared?{targetPrepared}: {})};
  // Persist terminal evidence BEFORE any import/query verification. An export
  // validation error must never require replaying a failed physics attempt.
  for(const [name,envelope]of Object.entries(envelopes))if(envelope)fs.writeFileSync(path.join(output,`${name}.json`),JSON.stringify(envelope)+'\n');
  fs.writeFileSync(path.join(output,'capture-outcome.json'),JSON.stringify(encodeReplayData({failure,attempts,certified,sourceManifest:manifest,config:f.config}))+'\n');
  const snapshots={},queries={};
  for(const [name,envelope]of Object.entries(envelopes))if(envelope) {
   const filename=`${name}.json`;fs.writeFileSync(path.join(output,filename),JSON.stringify(envelope)+'\n');
   const disk=JSON.parse(fs.readFileSync(path.join(output,filename))),a=roundtrip(disk),b=roundtrip(disk);
   for(let i=0;i<a.imported.bodies.length;i++)for(const [k,v]of Object.entries(a.imported.bodies[i]))if(ArrayBuffer.isView(v)) {
    const other=b.imported.bodies[i][k];assert.notEqual(v.buffer,other.buffer);assert.deepEqual(bytes(v),bytes(other));
   }
   snapshots[name]={file:filename,fileSha256:sha(fs.readFileSync(path.join(output,filename))),...a.verification,phase:a.imported.phase,fixture:a.imported.fixture.snapshot,
    bodyBuffers:a.imported.bodies.map(body=>({id:body.id,count:body.count,activeStart:body.activeStart,activeEnd:body.activeEnd,buffers:Object.fromEntries(Object.entries(body).filter(([,v])=>ArrayBuffer.isView(v)).map(([k,v])=>[k,{type:v.constructor.name,length:v.length,byteLength:v.byteLength,sha256:sha(bytes(v))}]))}))};
   if(name!=='initial') {
    const q=await queryImported(a.imported,anatomy,api.createContactResult),q2=await queryImported(b.imported,anatomy,api.createContactResult);
    assert.deepEqual(q,q2,'Imported buffer queries must reproduce exactly');
    const encoded=encodeReplayData({freshWall:q,originalHistory:contactHistorySummary(a.imported)}),file=`${name}-contacts.json`;
    fs.writeFileSync(path.join(output,file),JSON.stringify(encoded)+'\n');queries[name]={file,fileSha256:sha(fs.readFileSync(path.join(output,file))),counts:q.counts,queriedCapsules:q.queriedCapsules,exactRepeatedQueryEquality:true};
   }
  }
  const dependencies={};for(const name of ['three','three-mesh-bvh']) {const file=path.join(sourceRoot,'node_modules',name,'package.json'),p=JSON.parse(fs.readFileSync(file));dependencies[name]={version:p.version,packageJsonSha256:sha(fs.readFileSync(file))};}
  const report={schema:SCHEMA,status:failure?'stopped-at-original-reference-failure':'target-hold-prepared-and-reference-accepted',sourceRoot,sourceManifest:manifest,dependencies,
   scriptSha256:sha(fs.readFileSync(fileURLToPath(import.meta.url))),host:{node:process.version,endianness:os.endianness()},
   protocol:{solver:'reference',jointMotionMode:selection.jointMotionMode,wireTargetMm:318,catheterTargetMm:9,dt:1/120,config:f.config,
    acceptance:'No exception, no accepted:false, original coupledClosureConverged === true, World stepCount delta exactly1; stop at first failure, no retries or tolerance changes',
    targetPreparation:'Feed to318/9 with clamped final fractional command, then one original zero-command hold preparation; only exists if every earlier dt passes',
    purpose:'Data-only one-time JointWorldAdapter import; does not approve contacts:none on anatomy'},
   snapshots,queries,attempts,certifiedExecutedSteps:certified,failure,
   anatomy:{geometryPositionSha256:sha(bytes(anatomy.geometry.attributes.position.array)),geometryIndexSha256:anatomy.geometry.index?sha(bytes(anatomy.geometry.index.array)):null,
    boundingBox:{min:anatomy.geometry.boundingBox.min.toArray(),max:anatomy.geometry.boundingBox.max.toArray()},bvhValidationDistance:.02,capsuleBvhValidation:-.1},
   limitations:['Fixture reports wrapped handle angles, not continuous material winding. Zero spin commands do not prove zero winding or define a Joint frame branch.',
    'World velocities are legacy reconstructed nodal values, not a reference-material history sampler at arbitrary labels.',
    'Guidewire materialCoordinate labels and catheter current centerline-distance labels retain their original meanings; no common chart, remap, dsDx, or Joint BC IDs are inferred.',
    'Rigidity and mass presets retain existing nominal model units; no SI calibration or lambda-to-force conversion is invented.',
    'Imported objects are owned replay data, not executable World/THREE instances. Render objects, body solver scratch and functions listed in omissions are excluded.',
    'Original contact history is labelled by capture phase; fresh wall provider queries are separate and do not refresh lumen/portal/sheath/external mechanics.'],
   checks:{sourceStable:true,numericRoundtrip:true,independentImportedOwnership:true,repeatedOriginalQueryEquality:true,newSolverCalls:0}};
  verifySources(sourceRoot,manifest);
  // Tagged graph retains nonfinite values in diagnostics as well as buffers.
  fs.writeFileSync(path.join(output,'run.json'),JSON.stringify(encodeReplayData(report))+'\n');
  fs.writeFileSync(path.join(output,'index.json'),JSON.stringify({schema:SCHEMA,status:report.status,sourceRoot,scriptSha256:report.scriptSha256,snapshots,queries,
   certifiedExecutedSteps:certified,failure:failure?{attempt:failure.attempt,status:failure.status,actualWorldStatus:failure.actualWorldStatus,legacyVoidReturn:failure.legacyVoidReturn,
    wireMm:failure.after.wireMm,catheterMm:failure.after.catheterMm,rawWorldStepDelta:failure.rawWorldStepDelta,rawFixtureStepDelta:failure.rawFixtureStepDelta}:null,checks:report.checks},null,2)+'\n');
  return {status:report.status,certifiedExecutedSteps:certified,failure:failure?{attempt:failure.attempt,status:failure.status,wireMm:failure.after.wireMm,catheterMm:failure.after.catheterMm}:null,output};
 }finally{f.dispose();anatomy.dispose();}
}
export async function verifyWorldReplay(input) {
 const index=JSON.parse(fs.readFileSync(path.join(input,'index.json'))),run=importReplayData(JSON.parse(fs.readFileSync(path.join(input,'run.json'))));
 verifySources(index.sourceRoot,run.sourceManifest);const api=await modules(index.sourceRoot),anatomy=await api.loadCoupledRuntimeAnatomy(pathToFileURL(index.sourceRoot+'/'));
 const checks=[];
 try {for(const [name,saved]of Object.entries(index.snapshots)) {
  const file=fs.readFileSync(path.join(input,saved.file));assert.equal(sha(file),saved.fileSha256);const r=roundtrip(JSON.parse(file));
  assert.equal(r.verification.contentSha256,saved.contentSha256);
  if(index.queries[name]) {
   const qfile=fs.readFileSync(path.join(input,index.queries[name].file));assert.equal(sha(qfile),index.queries[name].fileSha256);
   const expected=importReplayData(JSON.parse(qfile));const actual=await queryImported(r.imported,anatomy,api.createContactResult);assert.deepEqual(actual,expected.freshWall);
  }
  checks.push({name,...r.verification});
 }return {status:'PASS',solverSteps:0,sourceHashes:true,snapshots:checks};}finally{anatomy.dispose();}
}
function selfTest() {
 const raw=new Uint8Array(24);new DataView(raw.buffer).setBigUint64(0,0x7ff8000000000021n,true);new DataView(raw.buffer).setFloat64(8,-0,true);new DataView(raw.buffer).setFloat64(16,Infinity,true);
 const v={a:new Float64Array(raw.buffer),b:new Float32Array([1.25,-Infinity]),mask:new Uint8Array([0,1]),negativeZero:-0,nan:NaN,missing:undefined,map:new Map(),external:{}};v.alias=v.a;v.map.set('self',v);
 const ext=new Map([[v.external,'test.external']]),envelope=encodeReplayData(v,ext),r=roundtrip(envelope);assert.equal(r.imported.alias,r.imported.a);assert.equal(r.imported.map.get('self'),r.imported);assert.notEqual(r.imported.a.buffer,v.a.buffer);
 r.imported.a[1]=19;assert.ok(Object.is(v.a[1],-0));const corrupt=structuredClone(envelope);corrupt.nodes.find(n=>n.kind==='buffer').base64='AAAA';assert.throws(()=>importReplayData(corrupt),/hash mismatch/);
 console.log(JSON.stringify({status:'PASS',cases:['raw NaN payload','negative zero','nonfinite metadata','Map/cycle/alias','independent owned buffers','content corruption rejection','external references'],...r.verification}));
}
if(process.argv[1]&&path.resolve(process.argv[1])===fileURLToPath(import.meta.url)) {
 const args=process.argv.slice(2),option=k=>{const i=args.indexOf(k);if(i<0||!args[i+1])throw new Error(`Required ${k}`);return path.resolve(args[i+1]);};
 if(args.includes('--self-test'))selfTest();
 else if(args.includes('--capture'))console.log(JSON.stringify(await captureWorldReplay(option('--source-root'),option('--output')),null,2));
 else if(args.includes('--verify'))console.log(JSON.stringify(await verifyWorldReplay(option('--input')),null,2));
 else throw new Error('Use --self-test, --capture --source-root ROOT --output DIR, or --verify --input DIR');
}

import {readFile,writeFile} from 'node:fs/promises';
import {pathToFileURL} from 'node:url';
const config=JSON.parse(await readFile('reports/hard-material-stop-source.json','utf8'));
const root=config.runtime;
const {fixture,DT}=await import(pathToFileURL(root+'/tests/fixtures/splitMotionAnalyticWorld.js'));
const {assembleKirchhoffDirect}=await import(pathToFileURL(root+'/src/physics/kirchhoffDirectSolver.js'));
const {rotateVectorByQuaternion}=await import(pathToFileURL(root+'/src/physics/discreteKirchhoffRod.js'));
const f=fixture({wall:true,y:-.5});
f.wire.wallStaticFriction=.6;f.wire.wallKineticFriction=.2;
for(const a of [1,2,3])f.wire['inverseInertia'+a].fill(0);
const records=[];
for(let i=0;i<3;i++){
 const b=f.wire;
 const before={x:[...b.x],vx:[...b.velocityX],restLength:[...b.restLength]};
 b.forceX.fill(.4/DT);b.forceY.fill(1/DT);
 const r=f.world.stepFixed();
 const s=f.constraint._splitMotion;
 const direct=assembleKirchhoffDirect(b,DT);
 const rows=[];
 for(let row=0;row<direct.rowCount;row++){
  const gradients=[];
  for(let dof=direct.start*6;dof<=(direct.end*6+2);dof++)for(let slot=0;slot<direct.degree[dof];slot++){
   const k=dof*direct.degreeCapacity+slot;
   if(direct.rows[k]===row)gradients.push({dof,value:direct.gradients[k]});
  }
  rows.push({row,alpha:direct.alpha[row],strain:direct.strain[row],rhs:direct.rhs[row],gradients});
 }
 const batch=f.constraint._jointSplitWallFrictionResidual?._batch;
 records.push({i,accepted:r.accepted,before,after:{x:[...b.x],vx:[...b.velocityX]},
   stateKeys:s?Object.keys(s):null,motion:s?Object.fromEntries(Object.entries(s.bodies[0]).map(([k,v])=>[k,[...v]])):null,
   incoming:s?.wallFrictionModes?.incoming.bodies[0],start:s?.start[0],directKeys:Object.keys(direct),rows,
   certificate:r.diagnostics.wallFrictionCertificate,
   entries:batch?.entries.map(e=>({key:e.contact.key,axes:e.surface.axes,gradients:e.surface.rows.map(r=>r.gradients),lever:e.surface.levers[0]})),
   history:f.constraint._wallFrictionHistory?Array.from(f.constraint._wallFrictionHistory.records):null,
   failure:r.accepted?null:r.diagnostics});
}
await writeFile(process.argv[2] ?? 'reports/hard-material-stop-dual-replay.json',JSON.stringify(records,(k,v)=>ArrayBuffer.isView(v)?[...v]:v,2)+'\n');
console.log(JSON.stringify(records.map(r=>({i:r.i,accepted:r.accepted,before:r.before,after:r.after,rows:r.rows,contacts:r.certificate.contacts?.map(c=>({key:c.key,Fn:c.normalLambda,disp:c.displacement,stop:c.stopCertificate})),entries:r.entries,keys:r.directKeys})),null,2));

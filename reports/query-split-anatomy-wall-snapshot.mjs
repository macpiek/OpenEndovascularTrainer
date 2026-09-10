// Replays the saved worst witness, field queries and row algebra. Never steps World.
// Usage: node reports/query-split-anatomy-wall-snapshot.mjs [--fixture file.json]
import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {fileURLToPath,pathToFileURL} from 'node:url';
const args=process.argv.slice(2),index=args.indexOf('--fixture');
const input=index<0?fileURLToPath(new URL('./split-anatomy-wall-worst-fixture.json',import.meta.url)):path.resolve(args[index+1]);
const saved=JSON.parse(fs.readFileSync(input)),runtime=saved.runtime,w=saved.worst,n=w.nodes[0],dt=saved.dt;
const hash=p=>createHash('sha256').update(fs.readFileSync(p)).digest('hex');
for(const [p,h] of Object.entries({...saved.sourceFiles,...saved.instrumentedHashes,...saved.assetHashes}))
  assert.equal(hash(path.join(runtime,p)),h,'Frozen source/asset changed: '+p);
const {loadCoupledRuntimeAnatomy}=await import(pathToFileURL(path.join(runtime,'tests/helpers/coupledRuntimeFixture.js')));
const {createContactResult}=await import(pathToFileURL(path.join(runtime,'src/physics/collision/vesselContactField.js')));
const {appendKirchhoffSplitPointWalls}=await import(pathToFileURL(path.join(runtime,'src/physics/kirchhoffSplitMotion.js')));
const anatomy=await loadCoupledRuntimeAnatomy(pathToFileURL(runtime+'/'));
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),minus=(a,b)=>a.map((v,i)=>v-b[i]);
const qdata=q=>Object.fromEntries(['signedDistance','signedGap','inside','conservative','source','faceIndex','branchId','segmentT','point','closestPoint','inward'].map(k=>[k,q[k]?.values?Array.from(q[k].values):q[k]]));
const sphere=p=>qdata(anatomy.field.querySphere(p,n.radius,createContactResult()));
const capsule=(a,b)=>qdata(anatomy.field.queryCapsule(a,b,n.radius,createContactResult()));
try{
  const initial=sphere(n.start),current=sphere(n.current);
  assert.ok(Math.abs(current.signedGap-w.raw)<1e-9);
  assert.ok(Math.abs(initial.signedGap-saved.controlQueries.startNode.signedGap)<1e-9);
  const J=current.inward,projectedDisplacement=dot(J,minus(n.current,n.start));
  const startGap=current.signedGap-projectedDisplacement,motion=dt*dot(J,n.velocity),historyGap=Math.max(0,startGap)+motion;
  assert.ok(Math.abs(startGap-w.startGap)<1e-9);
  assert.ok(Math.abs(motion-w.motion)<1e-9);
  assert.equal(w.lambda,0);assert.equal(w.alpha,0);
  const unilateralResidual=Math.max(0,-historyGap),delta=minus(n.current,n.start);
  assert.equal(unilateralResidual,0);
  // Characterize the actual production selector bug on the frozen implementation.
  const domain=saved.bodyDomain,body={...domain,wallCompliance:0,nodeRadius:[],x:[],y:[],z:[]};
  body.nodeRadius[n.node]=n.radius;['x','y','z'].forEach((a,i)=>body[a][n.node]=n.current[i]);
  const empty={activeStart:0,activeEnd:0,collisionStartSegment:1,collisionEndSegment:-1};
  const joint={innerBody:empty,outerBody:body,_splitMotion:{dt,pointWalls:[new Map(),new Map()]}};
  const rows=[];appendKirchhoffSplitPointWalls(joint,{contactField:anatomy.field,contactActivation:.2},rows);
  assert.ok(domain.collisionEndSegment<domain.collisionStartSegment);
  assert.equal(rows.length,1,'Frozen bug: a point row is emitted from an empty capsule domain');
  assert.equal(rows[0].node,17);assert.equal(rows[0].kind,'split-point-wall');
  assert.ok(Math.abs(rows[0].strain-w.raw)<1e-9);
  const result={status:'PASS: reproduced saved geometry, algebra and empty-domain selector defect without stepping World',sourceHashesVerified:true,
    current,exactStart:initial,exactStartViaDegenerateCapsule:capsule(n.start,n.start),
    currentViaDegenerateCapsule:capsule(n.current,n.current),
    adjacentCapsuleCurrent:capsule(...saved.adjacentSegment.current),
    J,dt,motion,projectedDisplacement,velocityVsPoseProjectionDifference:motion-projectedDisplacement,startGap,historyGap,unilateralResidual,
    displacement:delta,displacementMagnitude:Math.hypot(...delta),normalDot:dot(current.inward,initial.inward),
    currentPointOnInitialPlane:dot(minus(n.current,initial.closestPoint),initial.inward)-n.radius,
    domain,emittedRows:rows.map(r=>({kind:r.kind,side:r.side,node:r.node,rawGap:r.strain,lambda:r.lambda}))};
  const output=fileURLToPath(new URL('./split-anatomy-wall-verification.json',import.meta.url));
  fs.writeFileSync(output,JSON.stringify(result,null,2));console.log(JSON.stringify(result,null,2));
}finally{anatomy.dispose();}

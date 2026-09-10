import {performance} from 'node:perf_hooks';
import {EndovascularPhysicsWorld} from '../../src/physics/endovascularPhysicsWorld.js';
import {buildKirchhoffCoupledFrictionRows,measureKirchhoffCoupledFrictionResidual} from '../../src/physics/kirchhoffCoupledFrictionRows.js';
import {captureKirchhoffCoupledTrialState as capture,restoreKirchhoffCoupledTrialState as restore} from '../../src/physics/kirchhoffCoupledTrialState.js';
function fixture(count){
 const world=new EndovascularPhysicsWorld(),inner=world.createRod('wire',201,5,{radius:.4445}),outer=world.createRod('catheter',201,5,{radius:.8});
 for(let i=0;i<inner.count;i++)inner.setNodePosition(i,i*5,.0405,0);
 const c=world.addContainment(inner,outer,{innerRadius:.485,axialFriction:.2});
 for(let i=0;i<count;i++){
  const segment=i%200,contact=c.manifold.upsertContact({id:'contact-'+i,innerMaterialSegmentId:i,outerMaterialSegmentId:i,innerSegmentIndex:segment,outerSegmentIndex:segment,normal:[0,1,0],tangentU:[1,0,0]});contact.normalLambda=1;contact.tangentLambda.set([.01,.001]);
  c.kirchhoffContacts.push({id:'record-'+i,kind:'side',gap:0,normal:[0,1,0],_innerSegmentIndex:segment,_outerSegmentIndex:segment,innerT:.5,outerT:.5,innerWeights:[.5,.5],outerWeights:[.5,.5],manifoldContact:contact});
 }
 c._kirchhoffRuntimeRecords=[...c.kirchhoffContacts];c._jointFrictionBatch=buildKirchhoffCoupledFrictionRows(c,1/120);
 c._jointFrictionResidual=measureKirchhoffCoupledFrictionResidual(c,1/120);
 c._jointOptions={additionalRows:[...c._jointFrictionBatch.rows],groups:[...c._jointFrictionBatch.groups]};
 c._jointTrialState={};return{world,c};
}
const results=[];
for(const count of[2,50,479]) for(const optimized of [false,true]){
 const{world,c}=fixture(count),out=c._jointTrialState;
 capture(c,{world,reusePropertyLayout:optimized,frozenFrictionBatches:optimized},out);restore(out);
 const times=[];
 for(let i=0;i<12;i++){
  let start=performance.now();capture(c,{world,reusePropertyLayout:optimized,frozenFrictionBatches:optimized},out);const captureMs=performance.now()-start;
  start=performance.now();restore(out);times.push({captureMs,restoreMs:performance.now()-start});
 }
 const kinds={};let propertyCount=0;
 for(const r of out.records){kinds[r.kind]=(kinds[r.kind]??0)+1;propertyCount+=Object.keys(r.descriptors??{}).length;}
 results.push({optimized,contacts:count,nodesPerBody:201,bytes:out.bytes,objects:out.objectCount,kinds,propertyCount,times});
}
console.log(JSON.stringify({notes:'Synthetic stored contact graph only; no physics solve, anatomy, wall/external contacts or full replay. Current root helper with persistent-out barrier.',results}));

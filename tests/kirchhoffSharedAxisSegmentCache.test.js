import test from 'node:test';
import assert from 'node:assert/strict';
import {BufferGeometry,Float32BufferAttribute} from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {createSharedAxisSegmentContact} from '../src/physics/kirchhoffSharedAxisSegmentContact.js';
import {createSharedAxisVesselDiscovery} from '../src/physics/kirchhoffSharedAxisVesselWitnesses.js';
const geometry=()=>{const g=new BufferGeometry();g.setAttribute('position',new Float32BufferAttribute([0,-10,-10,0,10,-10,0,0,10],3));g.boundsTree=new MeshBVH(g);return g;};
test('whole-segment certificates accumulate motion and invalidate for a larger radius',()=>{
 const g=geometry(),q=createSharedAxisSegmentContact(g,{cacheCapacity:4}),ref=createSharedAxisSegmentContact(g);
 const query=(x,r=.4)=>q.query([x,-2,0],[x,2,0],r,{cacheKey:'rod'});
 query(1);query(.95);assert.equal(q.stats.queries,1);assert.equal(q.stats.clearanceHits,1);
 for(const x of [.8,.6,.42,.399,.2])assert.deepEqual(query(x),ref.query([x,-2,0],[x,2,0],.4));
 assert.ok(query(.2).face>=0,'slow accumulated motion cannot reuse a stale clear pose');
 query(2);assert.ok(query(2,2.1).face>=0,'increased radius triggers exact discovery');
 g.dispose();
});
test('exact cache hits preserve crossing/closest point and refitted geometry invalidates certificates',()=>{
 const g=geometry(),q=createSharedAxisSegmentContact(g,{cacheCapacity:2});
 const a=[-2,0,0],b=[3,0,0],options={cacheKey:'segment'};
 const first=q.query(a,b,.5,options);assert.equal(first.crossing,true);
 assert.deepEqual(q.query(a,b,.5,options),first);assert.equal(q.stats.cacheHits,1);
 first.face=999;assert.equal(q.query(a,b,.5,options).face,0,'caller cannot corrupt cache');
 const p=g.attributes.position;for(let i=0;i<p.count;i++)p.setX(i,10);p.needsUpdate=true;g.boundsTree.refit();
 assert.equal(q.query(a,b,.5,options).face,-1);
 q.clearCache();const before=q.stats.queries;q.query(a,b,.5,options);assert.equal(q.stats.queries,before+1);
 g.dispose();
});
test('cached and uncached queries agree throughout deterministic moving/rotating capsule paths',()=>{
 const g=geometry(),q=createSharedAxisSegmentContact(g,{cacheCapacity:8}),ref=createSharedAxisSegmentContact(g);
 for(let i=0;i<250;i++){
 const a=[1.3*Math.cos(i*.13),Math.sin(i*.17)*12,0],b=[1.1*Math.cos(i*.13+.2),3+Math.sin(i*.07)*12,0],r=.2+(i%7)*.04;
 assert.deepEqual(q.query(a,b,r,{cacheKey:'rod'}),ref.query(a,b,r));
 }
 g.dispose();
});
test('continuous discovery batches finite-radius contacts but still stops a centerline crossing',()=>{
 const g=geometry(),field={fallbackGeometry:g,voxelSize:10,queryCapsuleSoA(x,y,z,r,edge,out){return Object.assign(out,{signedDistance:2,signedGap:2-r[0],inside:true,faceIndex:0,segmentT:0});}};
 const d=createSharedAxisVesselDiscovery(field,0,{continuousSegmentContacts:true});
 const state={origin:[0,0,0],definitions:[],definitionIds:new Set(),layout:{positions:[0,3,6]},segmentContactTolerance:1e-4};
 d({state,a:[.2,-2,0],b:[.2,0,0],edge:0,radius:.4,coordinateA:1,coordinateB:3});
 d({state,a:[.2,0,0],b:[.2,2,0],edge:1,radius:.4,coordinateA:3,coordinateB:5});
 assert.equal(state.pendingVesselRows.size,2,'all contacts collected before the native assembly restarts');
 assert.throws(()=>d({state,a:[-1,0,0],b:[1,0,0],edge:0,radius:.4,coordinateA:1,coordinateB:3}),/shared-axis-wall-discovery/);
 g.dispose();
});
test('sub-budget penetration does not manufacture redundant rows; larger overlaps do',()=>{
 const g=geometry(),field={fallbackGeometry:g,voxelSize:1,queryCapsuleSoA(x,y,z,r,edge,out){return Object.assign(out,{signedDistance:2,signedGap:2-r[0],inside:true,faceIndex:0,segmentT:0});}};
 const d=createSharedAxisVesselDiscovery(field,0,{continuousSegmentContacts:true});
 const state={origin:[0,0,0],definitions:[],definitionIds:new Set(),layout:{positions:[0,3]},segmentContactTolerance:1e-4};
 d({state,a:[.4-5e-5,-2,0],b:[.4-5e-5,2,0],edge:0,radius:.4,coordinateA:1,coordinateB:3});
 assert.equal(state.pendingVesselRows?.size??0,0);
 d({state,a:[.4-2e-4,-2,0],b:[.4-2e-4,2,0],edge:0,radius:.4,coordinateA:1,coordinateB:3});
 assert.equal(state.pendingVesselRows.size,1);
 g.dispose();
});
test('sample clearance cannot certify the unsampled middle of a long segment',()=>{
 const g=geometry(),field={fallbackGeometry:g,voxelSize:10,queryCapsuleSoA(){throw Error('continuous guard was skipped');}};
 const d=createSharedAxisVesselDiscovery(field,0,{continuousSegmentContacts:true}),a=[-2,0,0],b=[3,0,0],key='1/6/0/1';
 const certificate=d.discoveryCache.begin(key,{a,b,sampleCount:1,gridToken:key,geometryToken:g.boundsTree},{insideCertified:true});
 for(const [t,distance] of [[0,2],[1,3]])certificate.visit({source:'sparse-sdf-bvh',faceIndex:0,signedDistance:distance,inside:true},t);
 assert.equal(certificate.commit(),true);
 const state={origin:[0,0,0],definitions:[],definitionIds:new Set(),layout:{positions:[0,3]}};
 assert.throws(()=>d({state,a,b,edge:0,radius:.4,coordinateA:1,coordinateB:6}),/shared-axis-wall-discovery/);
 assert.equal(d.segmentContact.stats.crossings,1);g.dispose();
});

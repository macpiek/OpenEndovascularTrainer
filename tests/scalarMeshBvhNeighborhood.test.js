import assert from 'node:assert/strict';
import test from 'node:test';
import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {closestPointToPointScalarBvh,createScalarMeshBvhScratch} from '../src/physics/collision/scalarMeshBvhClosestPoint.js';

const target=()=>({point:new THREE.Vector3(),distance:Infinity,faceIndex:-1});
function compare(tree,point,cached,reference,{maximum=Infinity,hint=-1}={}) {
    const a=target(),b=target();
    const hit=closestPointToPointScalarBvh(tree,...point,a,cached,maximum,hint);
    const expected=closestPointToPointScalarBvh(tree,...point,b,reference,maximum,hint);
    assert.equal(hit,expected);
    if(hit)assert.deepEqual(a,b,'Exact point, distance and tie-selected face');
    return a;
}
function twoPlanes() {
    const geometry=new THREE.BufferGeometry();
    geometry.setAttribute('position',new THREE.Float32BufferAttribute([
        -1,-1,0, 1,-1,0, 0,1,0,
        -1,-1,.2, 1,-1,.2, 0,1,.2
    ],3));
    geometry.boundsTree=new MeshBVH(geometry,{maxLeafTris:1});
    return geometry;
}

test('nearby queries preserve exact closest points and changing hints',()=>{
    const geometry=new THREE.TorusKnotGeometry(12,2.5,160,12);
    geometry.boundsTree=new MeshBVH(geometry);
    const cached=createScalarMeshBvhScratch(),reference=createScalarMeshBvhScratch(64,{reuseNeighborhood:false});
    let seed=314159,previous=-1;
    const random=()=>((seed=(Math.imul(seed,1664525)+1013904223)>>>0)/2**32);
    const vertices=geometry.attributes.position;
    for(let group=0;group<150;group++) {
        const at=Math.floor(random()*vertices.count);
        const origin=[vertices.getX(at),vertices.getY(at),vertices.getZ(at)].map(v=>v+(random()-.5)*2);
        for(let step=0;step<25;step++) {
            const point=origin.map(v=>v+(random()-.5)*.002);
            const hint=step%4===0?-1:step%4===1?Math.floor(random()*geometry.index.count/3):previous;
            const a=compare(geometry.boundsTree,point,cached,reference,{hint});
            previous=a.faceIndex;
            if(step%7===0)compare(geometry.boundsTree,point,cached,reference,{hint,maximum:a.distance*.5});
        }
    }
    assert.ok(cached.neighborhoodHits>500,'Exercise certified reuse, not just full-search fallback');
    assert.ok(cached.neighborhoodWinnerHits>500,'Exercise strict winner-separation proofs');
    geometry.dispose();
});

test('movement invalidates omitted-node proof before a different wall becomes closer',()=>{
    const geometry=twoPlanes(),cached=createScalarMeshBvhScratch(),reference=createScalarMeshBvhScratch(64,{reuseNeighborhood:false});
    const first=compare(geometry.boundsTree,[.1,0,.05],cached,reference);
    compare(geometry.boundsTree,[.1,0,.0501],cached,reference);
    assert.equal(cached.neighborhoodHits,1);
    const moved=compare(geometry.boundsTree,[.1,0,.15],cached,reference);
    assert.notEqual(moved.faceIndex,first.faceIndex);
    assert.equal(cached.neighborhoodHits,1,'No stale neighborhood accepted across closer-wall transition');
    geometry.dispose();
});

test('ties and bounded misses retain reference traversal and hint ordering',()=>{
    const geometry=new THREE.PlaneGeometry(2,2),cached=createScalarMeshBvhScratch(),reference=createScalarMeshBvhScratch(64,{reuseNeighborhood:false});
    geometry.boundsTree=new MeshBVH(geometry,{maxLeafTris:1});
    for(let repeat=0;repeat<5;repeat++)for(const hint of [-1,0,1]) {
        compare(geometry.boundsTree,[0,0,.1],cached,reference,{hint});
        compare(geometry.boundsTree,[0,0,.1],cached,reference,{hint,maximum:.05});
    }
    assert.equal(cached.neighborhoodHits,0,'Shared diagonal is deliberately ambiguous');
    geometry.dispose();
});

test('cache is bounded and a different BVH cannot inherit old certificates',()=>{
    const geometry=twoPlanes(),cached=createScalarMeshBvhScratch(),reference=createScalarMeshBvhScratch(64,{reuseNeighborhood:false});
    for(let i=0;i<2100;i++)compare(geometry.boundsTree,[i*.26,0,.05],cached,reference);
    assert.equal(cached.neighborhoods.size,2048);
    assert.equal(cached.neighborhoodBytes,[...cached.neighborhoods.values()].reduce((n,r)=>n+r.faces.byteLength+r.distances.byteLength,0));
    const replacement=twoPlanes();replacement.translate(0,0,1);replacement.boundsTree=new MeshBVH(replacement,{maxLeafTris:1});
    compare(replacement.boundsTree,[.1,0,.05],cached,reference);
    assert.equal(cached.neighborhoods.size,1);
    assert.equal(cached.neighborhoodBytes,[...cached.neighborhoods.values()].reduce((n,r)=>n+r.faces.byteLength+r.distances.byteLength,0));
    geometry.dispose();replacement.dispose();
});

test('independent BVH roots and tiny movements retain the exact winner',()=>{
    const geometry=new THREE.BoxGeometry(2,2,2,4,4,4);
    geometry.boundsTree=new MeshBVH(geometry,{maxLeafTris:2});
    assert.ok(geometry.boundsTree._roots.length>1);
    const cached=createScalarMeshBvhScratch(),reference=createScalarMeshBvhScratch(64,{reuseNeighborhood:false});
    for(let i=0;i<1000;i++) {
        const p=[.37+Math.sin(i)*.001,.29+Math.cos(i)*.001,.9+Math.sin(i*.1)*.02];
        compare(geometry.boundsTree,p,cached,reference,{hint:i%5===0?i%100:-1});
    }
    assert.ok(cached.neighborhoodHits>500);
    assert.equal(cached.neighborhoodBytes,[...cached.neighborhoods.values()].reduce((n,r)=>n+r.faces.byteLength+r.distances.byteLength,0));
    assert.ok(cached.neighborhoodBytes<=2048*256*12);
    geometry.dispose();
});

test('the cache key groups queries but never rounds the physical point',()=>{
    const geometry=twoPlanes(),cached=createScalarMeshBvhScratch(),reference=createScalarMeshBvhScratch(64,{reuseNeighborhood:false});
    const first=compare(geometry.boundsTree,[.1,0,.05],cached,reference);
    const next=compare(geometry.boundsTree,[.1,0,.05+1e-10],cached,reference);
    assert.ok(cached.neighborhoodHits>0);
    assert.notEqual(first.distance,next.distance);
    geometry.dispose();
});

test('non-Float32 vertices retain full traversal instead of borrowing neighborhood precision',()=>{
    const geometry=twoPlanes();
    geometry.setAttribute('position',new THREE.BufferAttribute(new Float64Array(geometry.attributes.position.array),3));
    geometry.boundsTree=new MeshBVH(geometry,{maxLeafTris:1});
    const cached=createScalarMeshBvhScratch(),reference=createScalarMeshBvhScratch(64,{reuseNeighborhood:false});
    for(let i=0;i<20;i++)compare(geometry.boundsTree,[.1,0,.05+i*1e-5],cached,reference);
    assert.equal(cached.neighborhoodHits,0);
    assert.equal(cached.neighborhoods.size,0);
    geometry.dispose();
});

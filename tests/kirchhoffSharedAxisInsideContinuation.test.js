import test from 'node:test';import assert from 'node:assert/strict';import {BoxGeometry} from 'three';import {MeshBVH} from 'three-mesh-bvh';
import {createSharedAxisInsideContinuation} from '../src/physics/kirchhoffSharedAxisInsideContinuation.js';
import {createSharedAxisVesselDiscovery} from '../src/physics/kirchhoffSharedAxisVesselWitnesses.js';
const geometry=()=>{const g=new BoxGeometry(.1,10,10);g.boundsTree=new MeshBVH(g);return g;};
test('local continuation can move along a thin wall but cannot cross it even if both endpoints are outside solid wall material',()=>{
    const g=geometry(),proof=createSharedAxisInsideContinuation(g,{maxDistance:2});proof.remember(0,[-.2,0,0],.15);
    assert.equal(proof.contains(0,[-.2,.1,0]),true);assert.equal(proof.stats.distanceProofs,1);
    assert.equal(proof.contains(0,[-.2,1,0]),true);assert.equal(proof.stats.pathProofs,1);
    assert.equal(proof.contains(0,[.2,0,0]),false);assert.equal(proof.stats.blocked,1);
    assert.equal(proof.contains(0,[-.2,3,0]),false,'The continuation is strictly local');g.dispose();
});
test('remeshed sites can inherit a nearby proof, replay snapshots own data and BVH replacement invalidates all old evidence',()=>{
    const g=geometry(),proof=createSharedAxisInsideContinuation(g,{capacity:2});const p=[-.2,0,0];proof.remember(1,p,.15);p[0]=10;
    assert.equal(proof.contains(1.25,[-.2,1,0]),true);
    const saved=proof.capture(),restored=createSharedAxisInsideContinuation(g);restored.restore(saved);saved[0].point[0]=10;
    assert.equal(restored.contains(1,[-.2,1,0]),true);assert.equal(restored.contains(1,[.2,0,0]),false);
    g.boundsTree=new MeshBVH(g);assert.equal(proof.contains(1,[-.2,0,0]),false);
    assert.equal(proof.capture().length,0);g.dispose();
});
test('a cold SDF sign reversal is repaired only on the connected side of the wall; reference discovery is unchanged',()=>{
    const g=geometry(),field={voxelSize:1,fallbackGeometry:g,negative:false,queryCapsuleSoA(...args){
        const distance=this.negative?-.15:.15,c={signedDistance:distance,signedGap:distance-.1,inside:!this.negative,source:'sparse-sdf-bvh',faceIndex:0,segmentT:0};
        for(let i=0;i<=args[12];i++)args[15]({...c},i/args[12]);return c;
    }};
    const state={origin:[0,0,0],definitions:[],layout:{positions:[0,3]}},input={state,edge:0,a:[-.2,0,0],b:[-.2,0,1],coordinateA:1,coordinateB:2,radius:.1};
    const realtime=createSharedAxisVesselDiscovery(field,0,{continuousDiscoverySign:true,queryReuse:false}),reference=createSharedAxisVesselDiscovery(field,0,{queryReuse:false});
    realtime(input);field.negative=true;
    assert.throws(()=>reference(input),/crossed/);assert.doesNotThrow(()=>realtime({...input,a:[-.2,1,0],b:[-.2,1,1]}));
    assert.throws(()=>realtime({...input,a:[.2,1,0],b:[.2,1,1]}),/crossed/);g.dispose();
});

test('clearance-ball evidence survives skipped motion beyond the local ray limit and remeshing',()=>{
    const g=geometry(),proof=createSharedAxisInsideContinuation(g,{maxDistance:2});
    proof.remember(0,[-10.05,0,0],10);
    assert.equal(proof.contains(0,[-3,0,0]),true,'Seven millimetres remain inside the certified wall-free ball');
    assert.equal(proof.contains(123,[-3,0,0]),true,'New mesh site can inherit the same geometric proof');
    assert.equal(proof.stats.distanceProofs,2);assert.equal(proof.stats.pathProofs,0);
    assert.equal(proof.contains(0,[.2,0,0]),false,'A point across the wall lies outside the ball and ray reach');
    assert.equal(proof.contains(0,[-10.05,11,0]),false,'A distant point beyond all certified evidence is not guessed');
    g.dispose();
});

test('nonfinite queries and invalid limits cannot produce geometric evidence',()=>{
    const g=geometry(),proof=createSharedAxisInsideContinuation(g,{maxDistance:2});proof.remember(0,[-.2,0,0],.15);
    for(const p of [[NaN,0,0],[Infinity,0,0],[],null])assert.equal(proof.contains(0,p),false);
    for(const options of [{capacity:0},{capacity:1.5},{maxDistance:NaN},{maxDistance:Infinity},{maxDistance:0}])
        assert.throws(()=>createSharedAxisInsideContinuation(g,options),/limits/);
    g.dispose();
});

test('a remeshed sample can prove membership along an entire coarse edge, but never through a thin wall',()=>{
    const g=geometry(),proof=createSharedAxisInsideContinuation(g);
    proof.remember(0,[-.2,0,0],.15);
    assert.equal(proof.contains(20,[-.2,20,0]),true,'Exact ray proves a 20 mm wall-free path');
    assert.equal(proof.stats.pathProofs,1);
    proof.remember(1,[-20,0,0],19.95);
    assert.equal(proof.contains(2,[20,0,0]),false,'Both endpoints clear the wall, but their connecting ray crosses it');
    assert.equal(proof.stats.blocked,1);g.dispose();
});

import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {mergeGeometries} from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import {MeshBVH} from 'three-mesh-bvh';
import {surfaceFixture} from './helpers/contrastSurfaceFixture.js';
import {buildAnatomicalContrastSurface} from '../src/contrast/anatomicalContrastSurface.js';
import {createLumenSurfaceMapping} from '../src/contrast/lumenSurfaceMapping.js';

// Both the near and far walls must read only their own vessel's concentration.
// The neighbour's axis is deliberately closer to part of the larger lumen.
for(const smallRadius of [1,.2])for(const indexed of [false,true])
test(`adjacent unequal lumens keep all inner faces and never exchange iodine (r=${smallRadius}, BVH=${indexed})`,()=>{
    const large=surfaceFixture(8),small=surfaceFixture(smallRadius),x=8+smallRadius+.4;
    large.volume.dispose();small.volume.dispose();small.geometry.translate(x,0,0);
    const geometry=mergeGeometries([large.geometry,small.geometry]);
    if(indexed)geometry.boundsTree=new MeshBVH(geometry);
    const edge=(index,cx,r)=>({index,start:new THREE.Vector3(cx,5,0),end:new THREE.Vector3(cx,-5,0),
        axis:new THREE.Vector3(0,-1,0),length:10,radiusStart:r,radiusEnd:r});
    const network={edges:[edge(0,0,8),edge(1,x,smallRadius)]};
    const stencil={stencil:(index,t)=>[index,index,Math.max(0,Math.min(1,t)),-1]};
    const before=geometry.attributes.position.array.slice(),beforeIndex=geometry.index?.array.slice();
    const built=buildAnatomicalContrastSurface(network,{fallbackGeometry:geometry},stencil);
    try {
        assert.equal(built.triangleEdges.length,(large.geometry.attributes.position.count+small.geometry.attributes.position.count)/6,
            'Every inner triangle survives, with no outer-wall coating');
        const p=built.geometry.attributes.position,s=built.geometry.attributes.flowStencil;
        const counts=[0,0];
        for(let i=0;i<built.triangleEdges.length;i++) {
            const owner=built.triangleEdges[i];counts[owner]++;
            for(let j=0;j<3;j++) {
                const v=i*3+j,cx=owner?x:0,r=owner?smallRadius:8;
                assert.ok(Math.abs(Math.hypot(p.getX(v)-cx,p.getZ(v))-r)<1e-5,'Owner must lie in this physical lumen');
                assert.equal(s.getX(v),owner);assert.equal(s.getY(v),owner);
                // Only vessel 0 contains contrast: the other lumen stays empty.
                const concentrations=[1,0];
                assert.equal(concentrations[s.getX(v)],owner===0?1:0);
            }
        }
        assert.ok(counts.every(n=>n>0));
        assert.deepEqual(geometry.attributes.position.array,before);
        assert.deepEqual(geometry.index?.array,beforeIndex,'Private BVH cannot mutate the source index');
    } finally {built.geometry.dispose();geometry.dispose();large.geometry.dispose();small.geometry.dispose();}
});

test('an unrepresented neighbouring lumen cannot borrow iodine across its wall',()=>{
    const a=surfaceFixture(1),b=surfaceFixture(.2);a.volume.dispose();b.volume.dispose();b.geometry.translate(2,0,0);
    const geometry=mergeGeometries([a.geometry,b.geometry]);geometry.boundsTree=new MeshBVH(geometry);
    const network=a.network,built=buildAnatomicalContrastSurface(network,{fallbackGeometry:geometry},{stencil:()=>[0,0,0,-1]});
    try {
        assert.equal(built.triangleEdges.length,a.geometry.attributes.position.count/6);
        const p=built.geometry.attributes.position;
        for(let v=0;v<p.count;v++)assert.ok(Math.abs(Math.hypot(p.getX(v),p.getZ(v))-1)<1e-5);
    } finally {built.geometry.dispose();geometry.dispose();a.geometry.dispose();b.geometry.dispose();}
});

test('an open lumen connection remains eligible while an intervening wall blocks the same axis',()=>{
    const f=surfaceFixture(1);f.volume.dispose();f.geometry.boundsTree=new MeshBVH(f.geometry);
    const map=createLumenSurfaceMapping(f.network,{wallTree:f.geometry.boundsTree});
    const inside=map(1,0,0,{},new THREE.Vector3(-1,0,0),5);
    assert.equal(inside.edgeIndex,0);
    const outside=map(2,0,0,{},new THREE.Vector3(-1,0,0),5);
    assert.equal(outside.edgeIndex,-1);
    f.geometry.dispose();
});

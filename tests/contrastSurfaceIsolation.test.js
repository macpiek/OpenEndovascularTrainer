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

test('seeded and territory searches preserve exact nearest-axis and wall-visibility decisions', () => {
    const edges = Array.from({length: 65}, (_, index) => {
        const start = new THREE.Vector3(Math.sin(index * .7) * 10, Math.cos(index * .53) * 11, (index % 7) - 3);
        const end = start.clone().add(new THREE.Vector3(Math.cos(index) * 3, 2, Math.sin(index) * 2));
        const axis = end.clone().sub(start), length = axis.length(); axis.normalize();
        return {index, start, end, axis, length, radiusStart: 1 + index % 4, radiusEnd: 1.5 + index % 4,
            transportExcluded: index % 17 === 0, lumenMappingExcluded: index % 19 === 0};
    });
    // A tie must retain the lower edge index, independent of BVH construction.
    Object.assign(edges[64], {...edges[4], index: 64});
    const territories = Int32Array.from(edges, e => e.index % 5);
    const wall = new THREE.PlaneGeometry(80, 80).rotateY(Math.PI / 2);
    wall.boundsTree = new MeshBVH(wall);
    const map = createLumenSurfaceMapping({edges}, {territories, radiusWeighted: true, wallTree: wall.boundsTree});
    const ray = new THREE.Ray();
    try {
        for (let i = 0; i < 400; i++) {
            const p = new THREE.Vector3(Math.sin(i * .37) * 12, Math.cos(i * .61) * 12, Math.sin(i * .83) * 4);
            const normal = i % 3 ? new THREE.Vector3(Math.cos(i), Math.sin(i), .3).normalize() : null;
            const territory = i % 8 - 2, maxDistance = i % 4 ? Infinity : 4;
            const candidates = [];
            for (const e of edges) {
                if (e.transportExcluded || e.lumenMappingExcluded || (territory >= 0 && territories[e.index] !== territory)) continue;
                const t = Math.max(0, Math.min(1, ((p.x - e.start.x) * e.axis.x +
                    (p.y - e.start.y) * e.axis.y + (p.z - e.start.z) * e.axis.z) / e.length));
                const dx = p.x - e.start.x - e.axis.x * e.length * t;
                const dy = p.y - e.start.y - e.axis.y * e.length * t;
                const dz = p.z - e.start.z - e.axis.z * e.length * t;
                const d = dx * dx + dy * dy + dz * dz;
                if (d > maxDistance * maxDistance) continue;
                if (normal && (d > (2 * Math.max(e.radiusStart, e.radiusEnd) + 1) ** 2 ||
                    normal.x * -dx + normal.y * -dy + normal.z * -dz <= 1e-5)) continue;
                if (normal) {
                    ray.origin.copy(p); ray.direction.set(-dx, -dy, -dz).divideScalar(Math.sqrt(d));
                    if (wall.boundsTree.raycastFirst(ray, THREE.DoubleSide, 1e-4, Math.sqrt(d) - 1e-4)) continue;
                }
                const radius = e.radiusStart + (e.radiusEnd - e.radiusStart) * t;
                candidates.push({e, t, d, score: d / (territory < 0 ? Math.max(1e-8, radius * radius) : 1)});
            }
            candidates.sort((a, b) => a.score - b.score || a.e.index - b.e.index);
            const expected = candidates[0], actual = map(p.x, p.y, p.z, {}, normal, maxDistance, territory);
            assert.equal(actual.edgeIndex, expected?.e.index ?? -1, `Query ${i}, territory ${territory}`);
            if (expected) {
                assert.equal(actual.t, expected.t);
                assert.equal(actual.distance, Math.sqrt(expected.d));
                assert.equal(actual.radius, expected.e.radiusStart + (expected.e.radiusEnd - expected.e.radiusStart) * expected.t);
            }
        }
    } finally {wall.dispose();}
});

test('wall-plane pruning keeps exact orientation threshold decisions at large coordinates', () => {
    for (const base of [0, 1e6, 1e9]) {
        const edges = Array.from({length: 20}, (_, index) => ({index,
            start: new THREE.Vector3(base + index % 2, -2, 0), end: new THREE.Vector3(base + index % 2, 2, 0),
            axis: new THREE.Vector3(0, 1, 0), length: 4, radiusStart: 1, radiusEnd: 1}));
        const map = createLumenSurfaceMapping({edges});
        for (const sign of [-1, 1]) for (const delta of [-1e-8, -1e-12, 0, 1e-12, 1e-8]) {
            const x = base - sign * (1e-5 + delta), normal = new THREE.Vector3(sign, 0, 0);
            const eligible = edges.filter(e => {
                const dx = x - e.start.x;
                return dx * dx <= 9 && normal.x * -dx > 1e-5;
            }).sort((a, b) => Math.abs(x - a.start.x) - Math.abs(x - b.start.x) || a.index - b.index);
            assert.equal(map(x, 0, 0, {}, normal).edgeIndex, eligible[0]?.index ?? -1,
                `Origin ${base}, orientation ${sign}, threshold delta ${delta}`);
        }
    }
});

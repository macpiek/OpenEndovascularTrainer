import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {loadCoupledRuntimeAnatomy} from './helpers/coupledRuntimeFixture.js';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';
import {createLumenSurfaceMapping} from '../src/contrast/lumenSurfaceMapping.js';
import {sampleFlowConcentration} from '../src/contrast/flowConcentrationField.js';
import {AnatomicalContrastSurface} from '../src/contrast/anatomicalContrastSurface.js';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {MeshBVH} from 'three-mesh-bvh';

test('concave anterior ostium retains inner faces that point away from the nearest axis', () => {
    const fixture = JSON.parse(gunzipSync(readFileSync(new URL('./fixtures/contrast-anterior-ostium.json.gz', import.meta.url))));
    const {centerline: data, centerlineEdges: ends, centerlineStride: stride, view} = fixture;
    const network = new ContrastFlowNetwork(Array.from({length: ends.length / 2}, (_, id) => ({id,
        start: new THREE.Vector3().fromArray(data, id * stride), end: new THREE.Vector3().fromArray(data, id * stride + 3),
        radiusStart: data[id * stride + 6], radiusEnd: data[id * stride + 7], safeRadius: data[id * stride + 8],
        nodeStartId: ends[id * 2], nodeEndId: ends[id * 2 + 1]})));
    const source = new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(fixture.positions, 3));
    let cells = 0;
    const offsets = Uint32Array.from(network.edges, edge => {const offset = cells; cells += edge.cellCount; return offset;});
    const surface = new AnatomicalContrastSurface(network, offsets, {fallbackGeometry: source});
    const lumen = new THREE.BufferGeometry().setAttribute('position', surface.mesh.geometry.attributes.position);
    try {
        const tree = new MeshBVH(lumen, {indirect: true});
        const ray = new THREE.Ray(new THREE.Vector3(), new THREE.Vector3(0, 0, -1));
        // These five independent full-STL reference rays previously lost an
        // entry or exit face. Three cross the visible anterior aortic ostium;
        // two cross its fine continuation at a different local orientation.
        for (const [x, y] of [[102, 58], [102, 59], [77, 87], [74, 91], [73, 92]]) {
            ray.origin.set(view.left + (x + .5) / view.width * (view.right - view.left),
                view.bottom + (y + .5) / view.height * (view.top - view.bottom), view.front);
            const hits = tree.raycast(ray, THREE.DoubleSide).sort((a, b) => a.distance - b.distance)
                .filter((hit, i, list) => !i || hit.distance - list[i - 1].distance > 1e-5);
            let length = 0;
            for (let i = 0; i + 1 < hits.length; i++)
                if (hits[i].face.normal.dot(ray.direction) > 0 && hits[i + 1].face.normal.dot(ray.direction) < 0)
                    length += hits[i + 1].distance - hits[i].distance;
            const expected = fixture.referencePathLengths[x + y * view.width];
            assert.ok(expected > 1, 'Exercise a filled blood column, not its silhouette');
            assert.ok(Math.abs(length - expected) < 1e-5,
                `Ostium ray ${x},${y} must retain the original ${expected} mm blood interval, got ${length}`);
        }
    } finally {lumen.dispose(); surface.dispose(); source.dispose();}
});

test('pelvic anatomical surface cannot read concentration through another vessel wall',async()=>{
    const anatomy=await loadCoupledRuntimeAnatomy(undefined,'baseline');
    try {
        const {field,geometry:g}=anatomy,data=field.centerline,stride=field.centerlineStride,ends=field.arrays.centerlineEdges;
        const network=new ContrastFlowNetwork(Array.from({length:data.length/stride},(_,id)=>{
            const o=id*stride;return {id,start:new THREE.Vector3(...data.slice(o,o+3)),end:new THREE.Vector3(...data.slice(o+3,o+6)),
                radiusStart:data[o+6],radiusEnd:data[o+7],safeRadius:data[o+8],nodeStartId:ends[id*2],nodeEndId:ends[id*2+1]};
        }));
        const map=createLumenSurfaceMapping(network,{wallTree:g.boundsTree});
        const a=new THREE.Vector3(),b=a.clone(),c=a.clone(),ab=a.clone(),normal=a.clone(),center=a.clone(),ray=new THREE.Ray();
        const read=(v,i)=>v.fromBufferAttribute(g.attributes.position,g.index?g.index.getX(i):i);
        let wrong=0,reassigned=0;
        for(let i=0;i<(g.index?.count??g.attributes.position.count);i+=30) {
            read(a,i);read(b,i+1);read(c,i+2);center.copy(a).add(b).add(c).multiplyScalar(1/3);
            if(center.y < -400||center.y > -160)continue;
            const old=map(center.x,center.y,center.z,{});if(old.edgeIndex<0||old.radius<3)continue;
            normal.crossVectors(ab.subVectors(b,a),c.clone().sub(a)).normalize();
            ray.origin.copy(center);ray.direction.set(old.centerX-center.x,old.centerY-center.y,old.centerZ-center.z).normalize();
            if(normal.dot(ray.direction)<=0)continue;
            if(!g.boundsTree.raycastFirst(ray,THREE.DoubleSide,.001,old.distance-.001))continue;
            wrong++;
            const safe=map(center.x,center.y,center.z,{},normal,old.searchRadius);
            if(safe.edgeIndex<0)continue; // No valid local axis: never paint an unrelated lumen.
            reassigned++;
            ray.direction.set(safe.centerX-center.x,safe.centerY-center.y,safe.centerZ-center.z).normalize();
            assert.ok(normal.dot(ray.direction)>0);
            assert.equal(g.boundsTree.raycastFirst(ray,THREE.DoubleSide,1e-4,safe.distance-1e-4),null);
        }
        assert.ok(wrong>100,'The test must exercise the actual erroneous anatomical assignments');
        assert.ok(reassigned>100,'Valid surfaces must be reassigned, not all hidden');
    } finally {anatomy.dispose();}
});

test('aortic bolus remains graded over the actual pelvic side ostia', async () => {
    const anatomy = await loadCoupledRuntimeAnatomy(undefined, 'baseline');
    let surface, wall;
    try {
        const {field, geometry: g} = anatomy;
        const data = field.centerline, stride = field.centerlineStride, ends = field.arrays.centerlineEdges;
        const network = new ContrastFlowNetwork(Array.from({length: data.length / stride}, (_, id) => {
            const o = id * stride;
            return {id, start: new THREE.Vector3(...data.slice(o, o + 3)), end: new THREE.Vector3(...data.slice(o + 3, o + 6)),
                radiusStart: data[o + 6], radiusEnd: data[o + 7], safeRadius: data[o + 8],
                nodeStartId: ends[id * 2], nodeEndId: ends[id * 2 + 1]};
        }));
        const p = g.attributes.position, positions = [];
        for (let i = 0; i < (g.index?.count ?? p.count); i += 3) {
            const vertices = [0, 1, 2].map(j => g.index ? g.index.getX(i + j) : i + j);
            if (!vertices.every(v => p.getY(v) > -280 && p.getY(v) < -230 &&
                p.getX(v) > -20 && p.getX(v) < 20 && p.getZ(v) > -10 && p.getZ(v) < 30)) continue;
            for (const v of vertices) positions.push(p.getX(v), p.getY(v), p.getZ(v));
        }
        wall = new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        let count = 0;
        const profile = y => 1 / (1 + Math.exp((y + 259) / 3.5));
        const offsets = Uint32Array.from(network.edges, e => {
            const offset = count; count += e.cellCount;
            if (e.radiusStart > 5 && e.radiusEnd > 5) for (let i = 0; i < e.cellCount; i++) {
                const y = e.start.y + (e.end.y - e.start.y) * (i + .5) / e.cellCount;
                e.massMg[i] = e.volumes[i] * .3 * profile(y);
            }
            return offset;
        });
        surface = new AnatomicalContrastSurface(network, offsets, {fallbackGeometry: wall});
        surface.update(new Float32Array(count), .3, false);
        const {position, flowStencil: s, flowAxis: axis} = surface.samplingGeometry.attributes;
        let checked = 0, shared = 0, maxSharedJump = 0;
        const sharedConcentrations = new Map();
        for (let i = 0; i < position.count; i++) {
            const x = position.getX(i), y = position.getY(i), z = position.getZ(i);
            const c = sampleFlowConcentration(surface.current, s.getX(i), s.getY(i), s.getZ(i), s.getW(i));
            // The previous narrow central patch missed side-ostial seams at
            // x≈3, z≈-1/21: one actual STL point had C=0 in one triangle and
            // C≈.45 in its neighbour, despite a smooth supplied bolus field.
            const key = `${x},${y},${z}`;
            if (sharedConcentrations.has(key)) {
                shared++;
                maxSharedJump = Math.max(maxSharedJump, Math.abs(c - sharedConcentrations.get(key)));
            } else sharedConcentrations.set(key, c);
            if (x < -5 || x > 1 || y < -262 || y > -244 || z < -1 || z > 20) continue;
            assert.ok(c >= profile(y + 4) && c <= profile(y - 4),
                `Side ostia must not erase or flatten the aortic front at (${x}, ${y}, ${z}): ${c}`);
            assert.ok(Math.abs(axis.getZ(i)) < .5, 'No false beam-parallel chord at a short aortic connector');
            checked++;
        }
        assert.ok(checked > 1000, 'Exercise the wall patch that previously became an empty jagged hole');
        assert.ok(shared > 10000, 'Exercise the complete actual aortic/side-ostial wall crop');
        assert.ok(maxSharedJump < 1e-7, `A shared blood-facing wall point cannot have two concentrations: ${maxSharedJump}`);
    } finally {surface?.dispose(); wall?.dispose(); anatomy.dispose();}
});

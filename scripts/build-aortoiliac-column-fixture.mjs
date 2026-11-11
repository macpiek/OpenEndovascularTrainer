// Keep the actual lumen and recorded transport, but omit remote anatomy.
import {readFileSync, writeFileSync} from 'node:fs';
import {gzipSync, gunzipSync} from 'node:zlib';
import * as THREE from 'three';
import {decodeCollisionAsset} from '../src/physics/collision/collisionAssetFormat.js';
import {ContrastFlowNetwork} from '../src/contrast/flowNetwork.js';
// Preserve the historical renderer when regenerating anatomy/transport.
const fixtureUrl = new URL('../tests/fixtures/aortoiliac-column.json.gz', import.meta.url);
const {referenceShader} = JSON.parse(gunzipSync(readFileSync(fixtureUrl)));
const bytes = readFileSync(new URL('../res/Aorta_plain_renal.collision.bin', import.meta.url));
const asset = decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength));
const a = asset.arrays.centerlineSegments, links = asset.arrays.centerlineEdges, stride = asset.metadata.centerline.stride;
const segments = Array.from({length: links.length / 2}, (_, id) => ({id,
    start: new THREE.Vector3().fromArray(a, id * stride), end: new THREE.Vector3().fromArray(a, id * stride + 3),
    radiusStart: a[id * stride + 6], radiusEnd: a[id * stride + 7], safeRadius: a[id * stride + 8],
    nodeStartId: links[id * 2], nodeEndId: links[id * 2 + 1]}));
for (const r of asset.metadata.renal?.segments ?? []) Object.assign(segments[r.index],
    {renalSide:r.side, renalSector:r.sector, renalHilar:r.hilar ?? false, renalLateral:r.lateral ?? false, source:'renal-mesh'});
const network = new ContrastFlowNetwork(segments);
const recordings = JSON.parse(readFileSync(new URL('../tests/actualBifurcationMixing.transport.json', import.meta.url)));
if (recordings.some(frame => frame.cells.length !== network.edges.length)) throw Error('Transport topology changed');
const selected = network.edges.filter(e => Math.max(e.start.y, e.end.y) > -390 && Math.min(e.start.y, e.end.y) < -220);
const indices = new Map(selected.map((e, i) => [e.index, i]));
const edges = selected.map(e => ({sourceIndex:e.index, start:e.start.toArray(), end:e.end.toArray(),
    startNodeId:e.startNodeId, endNodeId:e.endNodeId, radiusStart:e.radiusStart, radiusEnd:e.radiusEnd,
    parentEdgeIndex:indices.get(e.parentEdgeIndex) ?? -1, childEdgeIndices:e.childEdgeIndices.filter(i=>indices.has(i)).map(i=>indices.get(i)),
    areas:Array.from(e.areas), volumes:Array.from(e.volumes), transportExcluded:e.transportExcluded, lumenMappingExcluded:e.lumenMappingExcluded}));
const source = JSON.parse(readFileSync(new URL('../tests/actualBifurcationMixing.fixture.json', import.meta.url)));
const result = {positions:source.geometry.data.attributes.position.array, edges, referenceShader,
    frames:recordings.map(r=>({phase:r.phase, elapsed:r.elapsed, time:r.time,
        mass:selected.map(e=>r.cells[e.index]), boundaryFlows:selected.map(e=>r.boundaryFlows[e.index])}))};
const packed = gzipSync(JSON.stringify(result));
writeFileSync(new URL('../tests/fixtures/aortoiliac-column.json.gz', import.meta.url), packed);
console.log(JSON.stringify({edges:edges.length, triangles:result.positions.length/9, bytes:packed.length, frames:result.frames.length}));

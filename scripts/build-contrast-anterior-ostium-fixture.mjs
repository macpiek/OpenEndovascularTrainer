import fs from 'node:fs';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import {gzipSync} from 'node:zlib';
import * as THREE from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {loadCoupledRuntimeAnatomy} from '../tests/helpers/coupledRuntimeFixture.js';

// The browser downloads only this small, unmodified STL crop and graph.
// Reference path lengths are measured against the COMPLETE shipped model,
// before constructing the contrast surface or closing any crop boundary.
const anatomy = await loadCoupledRuntimeAnatomy(undefined, 'baseline');
const {geometry, field} = anatomy;
const bounds = {min: [-20, -280, -10], max: [20, -230, 30]};
const view = {width: 160, height: 192, left: -10, right: 15, bottom: -269, top: -239, front: 100};
const p = geometry.attributes.position, indices = geometry.index, positions = [];
for (let i = 0; i < (indices?.count ?? p.count); i += 3) {
    const vertices = [0, 1, 2].map(j => indices ? indices.getX(i + j) : i + j);
    if (!vertices.every(v => [p.getX(v), p.getY(v), p.getZ(v)]
        .every((x, axis) => x >= bounds.min[axis] && x <= bounds.max[axis]))) continue;
    for (const v of vertices) positions.push(p.getX(v), p.getY(v), p.getZ(v));
}
const crop = new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
const cropTree = new MeshBVH(crop, {indirect: true}), direction = new THREE.Vector3(0, 0, -1);
const reference = [], ray = new THREE.Ray(new THREE.Vector3(), direction);
let comparedRays = 0, supportedPixels = 0, totalIntervals = 0;
const orderedHits = tree => {
    const hits = tree.raycast(ray, THREE.DoubleSide, view.front - bounds.max[2], view.front - bounds.min[2])
        .sort((a, b) => a.distance - b.distance);
    return hits.filter((hit, i) => !i || hit.distance - hits[i - 1].distance > 1e-5);
};
for (let y = 0; y < view.height; y++) for (let x = 0; x < view.width; x++) {
    ray.origin.set(view.left + (x + .5) / view.width * (view.right - view.left),
        view.bottom + (y + .5) / view.height * (view.top - view.bottom), view.front);
    const full = orderedHits(geometry.boundsTree), local = orderedHits(cropTree);
    assert.equal(local.length, full.length, `Crop cuts affect source intersections at pixel ${x},${y}`);
    for (let i = 0; i < full.length; i++) assert.ok(Math.abs(full[i].distance - local[i].distance) < 1e-5);
    comparedRays++;
    let pathLength = 0;
    for (let i = 0; i + 1 < full.length; i++) {
        const entry = full[i], exit = full[i + 1];
        if (entry.face.normal.dot(direction) <= 0 || exit.face.normal.dot(direction) >= 0) continue;
        const midpoint = ray.at((entry.distance + exit.distance) * .5, new THREE.Vector3());
        // Packed anatomical interior labels disambiguate blood from the gap
        // between separate vessels. They do not set the measured boundaries.
        if (!field.packedLumenField.isInsideCoordinates(midpoint.x, midpoint.y, midpoint.z)) continue;
        pathLength += exit.distance - entry.distance; totalIntervals++;
    }
    reference.push(pathLength); if (pathLength > 1) supportedPixels++;
}
assert.ok(supportedPixels > 10000, 'Reference must exercise the real aortic lumen and anterior ostium');
const graph = field.centerline, links = field.arrays.centerlineEdges;
const output = {
    source: 'res/Aorta_plain_renal.stl',
    sourceSha256: crypto.createHash('sha256').update(fs.readFileSync(new URL('../res/Aorta_plain_renal.stl', import.meta.url))).digest('hex'),
    description: 'Original anterior abdominal ostium (source edge 1566); unchanged STL faces, complete transport graph, AP path-length reference from full anatomical model.',
    bounds, view, validation: {comparedRays, supportedPixels, totalIntervals, cropTriangles: positions.length / 9},
    positions, centerline: Array.from(graph), centerlineStride: field.centerlineStride,
    centerlineEdges: Array.from(links), referencePathLengths: reference
};
const destination = new URL('../tests/fixtures/contrast-anterior-ostium.json.gz', import.meta.url);
fs.writeFileSync(destination, gzipSync(JSON.stringify(output), {level: 9}));
console.log(JSON.stringify({file: destination.pathname, bytes: fs.statSync(destination).size, ...output.validation}));
crop.dispose(); anatomy.dispose();

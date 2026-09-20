/** Offline atlas correction. Never overwrites its source directory.
 * Usage: PYTHON=/path/to/python-with-numpy node scripts/align-subclavian-arteries.mjs INPUT_RES OUTPUT_RES
 * Then rebuild the collision asset from the resulting STL before installing either asset.
 */
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import * as THREE from 'three';
import { createSubclavianDeformation, inDeformationRegion } from './anatomy/subclavianDeformation.mjs';
import { deformOutletManifest } from './anatomy/deformOutletManifest.mjs';
import { repairSubclavianLumens, transportedShoulderBranches, SUBCLAVIAN_LUMEN_REPAIR } from './anatomy/repairSubclavianLumens.mjs';

const [input, output] = process.argv.slice(2);
if (!input || !output || path.resolve(input) === path.resolve(output)) {
    throw Error('Specify distinct INPUT_RES and OUTPUT_RES directories');
}
const configPath = fileURLToPath(new URL('./anatomy/subclavian-landmarks.json', import.meta.url));
const config = JSON.parse(fs.readFileSync(configPath));
const hash = bytes => crypto.createHash('sha256').update(bytes).digest('hex');
const original = fs.readFileSync(path.join(input, 'Aorta_plain.stl'));
if (hash(original) !== config.sourceSha256) throw Error('Landmarks do not describe the input STL');
const closures = JSON.parse(fs.readFileSync(path.join(input, 'Aorta_plain.outlet-closures.json')));
if (closures.outputSha256 !== config.sourceSha256) throw Error('Closure manifest does not describe the input STL');
fs.mkdirSync(output, { recursive: true });
const splitPath = path.join(output, 'subclavian-subdivision.tmp.stl');
execFileSync(process.env.PYTHON || 'python3', [
    fileURLToPath(new URL('./anatomy/subdivide-stl.py', import.meta.url)),
    path.join(input, 'Aorta_plain.stl'), splitPath, configPath
], { stdio: 'inherit' });
const source = fs.readFileSync(splitPath);
let result = Buffer.from(source);
const t = config.transform;
const world = (x, y, z) => [(x - t.sourceCenter[0]) * t.scale + t.targetCenter[0],
    (z - t.sourceCenter[2]) * t.scale + t.targetCenter[1],
    -(y - t.sourceCenter[1]) * t.scale + t.targetCenter[2]];
const points = [], references = [], unique = new Map();
const triangles = source.readUInt32LE(80);
for (let i = 0; i < triangles; i++) for (let k = 0; k < 3; k++) {
    const offset = 84 + i * 50 + 12 + k * 12;
    const x = source.readFloatLE(offset), y = source.readFloatLE(offset + 4), z = source.readFloatLE(offset + 8);
    const p = world(x, y, z);
    if (!inDeformationRegion(p)) continue;
    const key = `${x},${y},${z}`;
    let id = unique.get(key);
    if (id === undefined) { id = points.length; unique.set(key, id); points.push(p); }
    references.push([offset, id]);
}
const deformation = createSubclavianDeformation(config.curves);
deformation.move(points, progress => console.log(`Subclavian transport: ${Math.round(progress * 100)}%`));
for (const [offset, id] of references) {
    const p = points[id];
    const old = world(source.readFloatLE(offset), source.readFloatLE(offset + 4), source.readFloatLE(offset + 8));
    if (Math.hypot(...p.map((v, k) => v - old[k])) < 1e-10) continue;
    result.writeFloatLE((p[0] - t.targetCenter[0]) / t.scale + t.sourceCenter[0], offset);
    result.writeFloatLE(-(p[2] - t.targetCenter[2]) / t.scale + t.sourceCenter[1], offset + 4);
    result.writeFloatLE((p[1] - t.targetCenter[1]) / t.scale + t.sourceCenter[2], offset + 8);
}
const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3();
for (let i = 0; i < triangles; i++) {
    const offset = 84 + i * 50;
    if (result.subarray(offset + 12, offset + 48).equals(source.subarray(offset + 12, offset + 48))) continue;
    for (let k = 0; k < 3; k++) [a, b, c][k].set(result.readFloatLE(offset + 12 + k * 12),
        result.readFloatLE(offset + 16 + k * 12), result.readFloatLE(offset + 20 + k * 12));
    b.sub(a).cross(c.sub(a)).normalize();
    for (let k = 0; k < 3; k++) result.writeFloatLE(b.getComponent(k), offset + k * 4);
}
const curves = config.curves.map(curve => ({ ...curve,
    actual: deformation.move(curve.old.map(p => p.slice())) }));
const transportedSha256 = hash(result);
const shoulderBranches = transportedShoulderBranches(deformation);
result = repairSubclavianLumens(result, curves, shoulderBranches);
const sha256 = hash(result), outputTriangles = result.readUInt32LE(80);
const manifest = deformOutletManifest(closures, deformation, { sha256, triangles: outputTriangles });
manifest.deformation.transportedSha256 = transportedSha256;
manifest.deformation.lumenRepair = SUBCLAVIAN_LUMEN_REPAIR;
fs.writeFileSync(path.join(output, 'Aorta_plain.stl'), result);
fs.writeFileSync(path.join(output, 'Aorta_plain.outlet-closures.json'), JSON.stringify(manifest, null, 2));
fs.writeFileSync(path.join(output, 'Aorta_plain.subclavian-alignment.json'), JSON.stringify({
    version: config.version, sourceSha256: config.sourceSha256, outputSha256: sha256,
    triangles: outputTriangles, sourceRevision: config.sourceRevision, landmarks: curves,
    lumenRepair: SUBCLAVIAN_LUMEN_REPAIR, shoulderBranches
}, null, 2));
fs.unlinkSync(splitPath);
console.log({ output, sha256, triangles: outputTriangles });

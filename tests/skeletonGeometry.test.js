import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { Box3, MeshBasicMaterial } from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { packSkeletonGeometry, skeletonTransferBuffers, unpackSkeletonGeometry } from '../src/skeletonGeometry.js';
import { releaseOwnedBuffers } from '../src/releaseOwnedBuffers.js';

const source = new OBJLoader().parse(readFileSync(new URL('../res/skeleton.obj', import.meta.url), 'utf8'));
const expectedBounds = new Box3().setFromObject(source);
const packets = packSkeletonGeometry(source);
const hash = array => createHash('sha256').update(new Uint8Array(array.buffer, array.byteOffset, array.byteLength)).digest('hex');
const expectedHashes = packets.map(packet => Object.fromEntries(Object.entries(packet.attributes)
    .map(([name, attribute]) => [name, hash(attribute.array)])));
const buffers = skeletonTransferBuffers(packets);
const bytes = buffers.reduce((sum, buffer) => sum + buffer.byteLength, 0);
const transferred = structuredClone(packets, { transfer: buffers });
assert.ok(buffers.every(buffer => buffer.byteLength === 0), 'transfer must detach the worker buffers');
const restored = unpackSkeletonGeometry(transferred, new MeshBasicMaterial());
assert.equal(restored.children.length, source.children.length);
assert.deepEqual(new Box3().setFromObject(restored), expectedBounds);
for (const [index, mesh] of restored.children.entries()) {
    assert.deepEqual(mesh.geometry.groups, source.children[index].geometry.groups);
    for (const [name, attribute] of Object.entries(mesh.geometry.attributes)) {
        assert.equal(hash(attribute.array), expectedHashes[index][name], 'geometry must be byte-identical');
    }
}
console.log(`Skeleton worker transfer: ${restored.children.length} meshes, ${bytes} bytes; exact geometry and bounds preserved`);

// The source STL may be discarded after parsing: rendering/collisions must
// retain their own complete vertex and normal arrays, not views into the file.
const file = readFileSync(new URL('../res/Aorta_plain.stl', import.meta.url));
const sourceBuffer = file.buffer.slice(file.byteOffset, file.byteOffset + file.byteLength);
const aorta = new STLLoader().parse(sourceBuffer);
const before = Object.values(aorta.attributes).map(attribute => hash(attribute.array));
releaseOwnedBuffers([sourceBuffer]);
assert.deepEqual(Object.values(aorta.attributes).map(attribute => hash(attribute.array)), before);
if (typeof ArrayBuffer.prototype.transfer === 'function') assert.equal(sourceBuffer.byteLength, 0);
console.log('Aorta source release: geometry preserved after the STL source buffer was discarded');

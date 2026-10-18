// Offline only: pin the two anatomical revisions used by September replays.
// Test execution reads the generated bundle and never requires Git history.
import {execFileSync} from 'node:child_process';
import {writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {gzipSync, inflateSync} from 'node:zlib';
import {encodeCollisionAsset} from '../../src/physics/collision/collisionAssetFormat.js';

const closedRevision = 'e1fc5c46d303ed223efe8dc50bdc7861b6e4c283';
const openRevision = `${closedRevision}^`;
const git = (...args) => execFileSync('git', args, {maxBuffer: 256 * 1024 * 1024});
const blob = (revision, extension) => git('show', `${revision}:res/Aorta_plain.${extension}`);
const sha256 = bytes => createHash('sha256').update(bytes).digest('hex');
const closedMesh = blob(closedRevision, 'stl'), openMesh = blob(openRevision, 'stl');
const closedAsset = blob(closedRevision, 'collision.bin'), openAsset = blob(openRevision, 'collision.bin');
if (!closedMesh.subarray(84, openMesh.length).equals(openMesh.subarray(84)))
    throw Error('Expected outlet closures to append faces without changing the original wall');

// Preserve every coordinate bit and face order, but store repeated STL vertices
// only once. The loader expands them before the ordinary transform/BVH build.
const vertexIds = new Map(), bits = [], corners = new Uint32Array(closedMesh.readUInt32LE(80) * 3);
for (let corner = 0; corner < corners.length; corner++) {
    const offset = 84 + Math.floor(corner / 3) * 50 + 12 + (corner % 3) * 12;
    const xyz = [0, 4, 8].map(k => closedMesh.readUInt32LE(offset + k)), key = xyz.join('/');
    let id = vertexIds.get(key);
    if (id === undefined) { id = bits.length / 3; bits.push(...xyz); vertexIds.set(key, id); }
    corners[corner] = id;
}

// Git's lossless copy/literal delta stores the small difference between the
// two collision fields. Only this generator needs Git/base85 decoding.
const patch = git('diff', '--binary', '--full-index', closedRevision, openRevision, '--', 'res/Aorta_plain.collision.bin').toString();
const block = patch.match(/GIT binary patch\ndelta (\d+)\n([^]*?)\n\n/);
if (!block) throw Error('Expected a binary delta between the collision fields');
const alphabet = '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz!#$%&()*+-;<=>?@^_`{|}~';
const chunks = block[2].split('\n').map(line => {
    const count = line[0] <= 'Z' ? line.charCodeAt(0) - 64 : line.charCodeAt(0) - 70;
    const bytes = Buffer.alloc(Math.ceil(count / 4) * 4);
    for (let i = 1, at = 0; i < line.length; i += 5, at += 4) {
        let value = 0;
        for (let j = 0; j < 5; j++) {
            const digit = alphabet.indexOf(line[i + j]);
            if (digit < 0) throw Error('Invalid Git base85 digit');
            value = value * 85 + digit;
        }
        bytes.writeUInt32BE(value, at);
    }
    return bytes.subarray(0, count);
});
const delta = inflateSync(Buffer.concat(chunks));
if (delta.length !== Number(block[1])) throw Error('Invalid delta length');
const metadata = {kind: 'shared-axis-replay-anatomy', revisions: {open: openRevision, closed: closedRevision},
    triangles: {open: openMesh.readUInt32LE(80), closed: closedMesh.readUInt32LE(80)},
    sourceSha256: {open: sha256(openMesh), closed: sha256(closedMesh)},
    collisionSha256: {open: sha256(openAsset), closed: sha256(closedAsset)},
    meshIdentity: {
        open: 'mesh-v1/3/Float32Array:9129699/Uint32Array:3043233/eefb98f4/af7758cd',
        closed: 'mesh-v1/3/Float32Array:9411615/Uint32Array:3137205/3126656e/725db843'
    }};
const packed = encodeCollisionAsset(metadata, {positionBits: Uint32Array.from(bits), corners,
    closedCollisionAsset: new Uint8Array(closedAsset), openCollisionDelta: new Uint8Array(delta)});
const output = new URL('../../tests/fixtures/shared-axis/anatomy-2026-09-19.bin.gz', import.meta.url);
const compressed = gzipSync(new Uint8Array(packed), {level: 9});
writeFileSync(output, compressed);
console.log(JSON.stringify({output: output.pathname, bytes: compressed.length, uniqueVertices: bits.length / 3}));

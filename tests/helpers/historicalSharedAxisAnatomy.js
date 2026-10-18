import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {createHash} from 'node:crypto';
import {BufferGeometry, BufferAttribute} from 'three';
import {MeshBVH} from 'three-mesh-bvh';
import {decodeCollisionAsset} from '../../src/physics/collision/collisionAssetFormat.js';
import {VesselContactField} from '../../src/physics/collision/vesselContactField.js';
import {sharedAxisGeometryIdentity} from '../../src/physics/kirchhoffSharedAxisGeometryIdentity.js';
import {transformAortaGeometry} from '../../src/aortaTransform.js';
import {generateVessel} from '../../src/vesselGeometry.js';

const buffer = bytes => bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
const sha256 = bytes => createHash('sha256').update(bytes).digest('hex');

// A Git binary delta is a size header followed by base-copy/literal commands.
// Validate every range and the reconstructed SHA before using the asset.
function applyDelta(base, delta) {
    let at = 0;
    const read = () => { if (at >= delta.length) throw Error('Truncated anatomy delta'); return delta[at++]; };
    const size = () => { let value = 0, shift = 0, byte; do {
        byte = read(); value += (byte & 127) * 2 ** shift; shift += 7;
        if (shift > 35) throw Error('Invalid anatomy delta size');
    } while (byte & 128); return value; };
    if (size() !== base.length) throw Error('Anatomy delta base size mismatch');
    const output = new Uint8Array(size()); let written = 0;
    while (at < delta.length) {
        const command = read(); let offset = 0, length = 0, source = base;
        if (command & 128) {
            for (let i = 0; i < 4; i++) if (command & (1 << i)) offset += read() * 2 ** (8 * i);
            for (let i = 0; i < 3; i++) if (command & (16 << i)) length += read() * 2 ** (8 * i);
            if (!length) length = 0x10000;
        } else {
            if (!command) throw Error('Invalid anatomy delta command');
            source = delta; offset = at; length = command; at += length;
        }
        if (offset + length > source.length || written + length > output.length) throw Error('Anatomy delta exceeds its bounds');
        output.set(source.subarray(offset, offset + length), written); written += length;
    }
    if (written !== output.length) throw Error('Incomplete anatomy delta');
    return output;
}

/** Original geometry and collision field for captured September regressions.
 * Modern-anatomy integration tests must keep using loadCoupledRuntimeAnatomy.
 * Never relabel a replay certificate or contact face onto the current mesh. */
export function loadHistoricalSharedAxisAnatomy(variant) {
    if (!['open', 'closed'].includes(variant)) throw new RangeError('Unknown historical anatomy');
    const bundle = decodeCollisionAsset(buffer(gunzipSync(readFileSync(new URL('../fixtures/shared-axis/anatomy-2026-09-19.bin.gz', import.meta.url)))));
    const {metadata, arrays} = bundle;
    if (metadata.kind !== 'shared-axis-replay-anatomy') throw Error('Invalid historical anatomy bundle');
    const bytes = variant === 'closed' ? arrays.closedCollisionAsset : applyDelta(arrays.closedCollisionAsset, arrays.openCollisionDelta);
    if (sha256(bytes) !== metadata.collisionSha256[variant]) throw Error('Historical collision asset hash mismatch');
    const asset = decodeCollisionAsset(buffer(bytes));
    if (asset.metadata.source.stlSha256 !== metadata.sourceSha256[variant]) throw Error('Historical mesh/field pair mismatch');
    const positions = new Uint32Array(metadata.triangles[variant] * 9);
    for (let i = 0; i < positions.length / 3; i++) {
        const source = arrays.corners[i] * 3;
        if (source + 2 >= arrays.positionBits.length) throw Error('Invalid historical mesh vertex');
        for (let k = 0; k < 3; k++) positions[i * 3 + k] = arrays.positionBits[source + k];
    }
    const geometry = new BufferGeometry();
    geometry.setAttribute('position', new BufferAttribute(new Float32Array(positions.buffer), 3));
    const {vessel} = generateVessel(140, 0);
    transformAortaGeometry(geometry, vessel); geometry.boundsTree = new MeshBVH(geometry);
    if (sharedAxisGeometryIdentity(geometry) !== metadata.meshIdentity[variant]) {
        geometry.dispose(); throw Error('Historical mesh identity mismatch');
    }
    const field = new VesselContactField(asset, {fallbackGeometry: geometry, bvhValidationDistance: .02, capsuleBvhValidation: -.1});
    return {vessel, geometry, field, dispose: () => geometry.dispose()};
}

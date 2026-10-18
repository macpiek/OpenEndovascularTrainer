import {readFileSync} from 'node:fs';
import {decodeCollisionAsset} from '../../src/physics/collision/collisionAssetFormat.js';
import {createCrossoverFlowFromAsset} from './crossoverFlowFromAsset.js';

/** Flow settings observed during demonstration 001, with the real anatomy. */
export function createCrossoverFlow() {
    const bytes=readFileSync(new URL('../../res/Aorta_infrarenal_aneurysm.collision.bin',import.meta.url));
    return createCrossoverFlowFromAsset(decodeCollisionAsset(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength)));
}

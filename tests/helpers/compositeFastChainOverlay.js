import fs from 'node:fs';
import { createHash } from 'node:crypto';
import * as chain from '../../src/physics/kirchhoffCompositeChain.js';

/** The integrated chain exposes its backend explicitly. This adapter keeps
 * the paired harness API without dynamic source rewriting. */
export async function loadCompositeFastChainOverlay() {
    const sourcePath=new URL('../../src/physics/kirchhoffCompositeChain.js',import.meta.url);
    const source=fs.readFileSync(sourcePath,'utf8');
    return {module:{...chain,createCompositeChainWorkspace:layout=>chain.createCompositeChainWorkspace(layout,{elementBackend:'wasm'})},
        sourceSha256:createHash('sha256').update(source).digest('hex'),
        overlay:'Integrated chain with the explicit wasm element backend; no source rewriting'};
}

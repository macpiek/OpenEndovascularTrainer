/** Research-only alternate finite contact mesh; never selected by the app. */
import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader.js';
import {MeshBVH} from 'three-mesh-bvh';

export function installExperimentalCollisionGeometry(anatomy,path,expected=null){
    const bytes=readFileSync(path),manifest=JSON.parse(readFileSync(path+'.json','utf8'));
    const hash=b=>createHash('sha256').update(b).digest('hex');
    const sha256=hash(bytes);
    if(manifest.coordinateSystem!=='world-mm'||manifest.meshSha256!==sha256)throw new Error('Collision experiment manifest mismatch');
    if(expected&&expected.meshSha256!==sha256)throw new Error('Replay contact triangle identities require the original experiment mesh');
    if(manifest.sourceSha256!==hash(readFileSync(new URL('../../res/Aorta_plain.stl',import.meta.url))))throw new Error('Collision experiment anatomy mismatch');
    const geometry=new STLLoader().parse(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
    geometry.boundsTree=new MeshBVH(geometry);anatomy.field.setFallbackGeometry(geometry);
    return {metadata:{...manifest,path,replayRequiresThisGeometry:true},dispose:()=>geometry.dispose()};
}

import {readFileSync} from 'node:fs';
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader.js';
import {MeshBVH} from 'three-mesh-bvh';
import {VesselContactField} from '../../src/physics/collision/vesselContactField.js';
import {decodeCollisionAsset} from '../../src/physics/collision/collisionAssetFormat.js';
import {transformAortaGeometry} from '../../src/aortaTransform.js';
import {generateVessel} from '../../src/vesselGeometry.js';

export function createCompositeAnatomyField() {
    const binary=name=>{const b=readFileSync(new URL(`../../res/${name}`,import.meta.url));return b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength);};
    const asset=decodeCollisionAsset(binary('Aorta_plain.collision.bin')),
        geometry=new STLLoader().parse(binary('Aorta_plain.stl'));
    transformAortaGeometry(geometry,generateVessel(140,0).vessel);geometry.boundsTree=new MeshBVH(geometry);
    return {field:new VesselContactField(asset,{fallbackGeometry:geometry,bvhValidationDistance:.02,capsuleBvhValidation:-.1}),
        dispose(){geometry.dispose();}};
}

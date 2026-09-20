import assert from 'node:assert/strict';
import fs from 'node:fs';
import * as THREE from 'three';
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader.js';
import {MeshBVH} from 'three-mesh-bvh';
import {decodeCollisionAsset} from '../../src/physics/collision/collisionAssetFormat.js';
import {VesselContactField} from '../../src/physics/collision/vesselContactField.js';
import {transformAortaGeometry} from '../../src/aortaTransform.js';
import {generateVessel} from '../../src/vesselGeometry.js';
import {StentGraftSystem} from '../../src/devices/stentGraftSystem.js';
import {DevicePath,createAorticRoutes} from '../../src/devices/stentGraftPaths.js';

const arrayBuffer=b=>b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength);
const {vessel}=generateVessel();
export function fixture(variant='Aorta_infrarenal_aneurysm') {
    const read=ext=>fs.readFileSync(new URL(`../../res/${variant}.${ext}`,import.meta.url));
    const asset=decodeCollisionAsset(arrayBuffer(read('collision.bin'))),segments=[];
    const data=asset.arrays.centerlineSegments;
    for(let i=0;i<data.length;i+=9)segments.push({id:i/9,start:new THREE.Vector3().fromArray(data,i),end:new THREE.Vector3().fromArray(data,i+3),
        radiusStart:data[i+6],radiusEnd:data[i+7],safeRadius:data[i+8],nodeStartId:asset.arrays.centerlineEdges[i/9*2],nodeEndId:asset.arrays.centerlineEdges[i/9*2+1]});
    const geometry=new STLLoader().parse(arrayBuffer(read('stl')));transformAortaGeometry(geometry,vessel);geometry.boundsTree=new MeshBVH(geometry);
    const contactField=new VesselContactField(asset,{fallbackGeometry:geometry});
    const anatomy={geometry,contactField,centerlineBroadPhase:{segments}};
    const routes=createAorticRoutes(segments,vessel.sheaths),sources={};
    for(const side of ['right','left']) {
        const route=routes[side],points=[new THREE.Vector3(...Object.values(vessel.sheaths[side].start)),...route.points];
        const path=new DevicePath(points);
        sources[side]={nodes:path.points,coordinate:i=>path.coordinates[i],catheterMm:0};
    }
    const system=new StentGraftSystem({readAccess:side=>sources[side],readAnatomy:()=>anatomy,sheaths:vessel.sheaths});
    return {system,sources,geometry,contactField,segments,dispose(){system.dispose();geometry.dispose();}};
}
export function place(system,side,type) {
    assert.equal(system.load(side,type).ok,true);
    const result=system.positionAtTarget(side);assert.equal(result.ok,true,result.reason);
    for(let i=0;i<1200;i++)system.updateAccess(side,1/60);
    const check=system.validation(side);assert.equal(check.ok,true,check.reason);
}
export function finish(system,side) {for(let i=0;i<190;i++)system.updateAccess(side,1/60);}


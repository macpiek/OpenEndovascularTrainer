import * as THREE from 'three';
import {StentGraftSystem} from '../../src/devices/stentGraftSystem.js';
import {DevicePath,AORTIC_NECK,AORTIC_BIFURCATION} from '../../src/devices/stentGraftPaths.js';

// An unconstrained Y fixture isolates deployment geometry from atlas loading.
export function previewFixture(side='right',type='body',startRelease=true,modelId) {
    const routes={},sources={};
    for(const side of ['right','left']) {
        const sign=side==='right'?-1:1;
        const path=new DevicePath([new THREE.Vector3(sign*55,-370,10),new THREE.Vector3(sign*25,-300,10),
            AORTIC_BIFURCATION,AORTIC_NECK,new THREE.Vector3(8,-160,-10)]);
        routes[side]=path;sources[side]={nodes:path.points,coordinate:i=>path.coordinates[i],catheterMm:0};
    }
    const system=new StentGraftSystem({readAccess:side=>sources[side],readAnatomy:()=>null,sheaths:{}});
    system.routes=routes;system.ensureRoutes=()=>true;system.load(side,type,modelId);
    const device=system.accesses[side].device;
    device.position=routes[side].nearest(AORTIC_NECK).s;device.target=device.position;
    if(startRelease)system.deploy(side);return {system,device,sources};
}

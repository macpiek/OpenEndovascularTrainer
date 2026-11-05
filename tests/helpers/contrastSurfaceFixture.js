import * as THREE from 'three';
import {mergeGeometries} from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import {ContrastFlowNetwork} from '../../src/contrast/flowNetwork.js';
import {AnatomicalContrastSurface} from '../../src/contrast/anatomicalContrastSurface.js';

export function surfaceFixture(radius = .2) {
    const parts = [radius, radius + .15].map((r, i) => {
        const geometry = new THREE.CylinderGeometry(r, r, 10, 24, 12, true).toNonIndexed();
        if (i === 0) {
            const a = geometry.attributes.position.array;
            for (let v = 0; v < a.length; v += 9) for (let j = 0; j < 3; j++) {
                const t = a[v + 3 + j]; a[v + 3 + j] = a[v + 6 + j]; a[v + 6 + j] = t;
            }
            geometry.computeVertexNormals();
        }
        return geometry;
    });
    const geometry = mergeGeometries(parts);
    const network = new ContrastFlowNetwork([{id: 0, start: new THREE.Vector3(0, 5, 0),
        end: new THREE.Vector3(0, -5, 0), radiusStart: radius, radiusEnd: radius,
        nodeStartId: 0, nodeEndId: 1}]);
    const offsets = new Uint32Array([0]);
    const plume = new Float32Array(network.edges[0].cellCount);
    const volume = new AnatomicalContrastSurface(network, offsets, {fallbackGeometry: geometry});
    parts.forEach(p => p.dispose());
    return {geometry, network, volume, plume};
}


export function adjacentSurfaceFixture(smallRadius = 1) {
    const large=surfaceFixture(8),small=surfaceFixture(smallRadius),x=8+smallRadius+.4;
    large.volume.dispose();small.volume.dispose();small.geometry.translate(x,0,0);
    const geometry=mergeGeometries([large.geometry,small.geometry]);
    const segment=(id,a,b,r,from,to)=>({id,start:new THREE.Vector3(...a),end:new THREE.Vector3(...b),
        radiusStart:r,radiusEnd:r,nodeStartId:from,nodeEndId:to});
    const network=new ContrastFlowNetwork([
        segment(0,[0,5,0],[0,-5,0],8,0,1),
        segment(1,[0,5,0],[x,5,0],smallRadius,0,2),
        segment(2,[x,5,0],[x,-5,0],smallRadius,2,3)
    ]);
    // The joining branch is outside this two-cylinder image fixture.
    const connector=network.edges.find(e=>Math.abs(e.axis.x)>.9);connector.lumenMappingExcluded=true;
    const offsets=new Uint32Array(network.edges.length);let count=0;
    for(const e of network.edges){offsets[e.index]=count;count+=e.cellCount;}
    const volume=new AnatomicalContrastSurface(network,offsets,{fallbackGeometry:geometry});
    return {geometry,network,volume,plume:new Float32Array(count),largeGeometry:large.geometry,smallGeometry:small.geometry,
        dispose(){volume.dispose();geometry.dispose();large.geometry.dispose();small.geometry.dispose();}};
}

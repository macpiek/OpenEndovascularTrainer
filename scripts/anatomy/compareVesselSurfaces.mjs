/** Deterministic symmetric surface sampling; not a proof of watertightness. */
import fs from 'node:fs';
import * as THREE from 'three';
import {STLLoader} from 'three/examples/jsm/loaders/STLLoader.js';
import {MeshBVH} from 'three-mesh-bvh';
const [source, repaired, reportPath] = process.argv.slice(2);
if (!reportPath) throw Error('Usage: compareVesselSurfaces.mjs SOURCE.stl REPAIRED.stl REPORT.json');
const meshes=[source,repaired].map(path=>{
    const bytes=fs.readFileSync(path);
    const geometry=new STLLoader().parse(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength));
    geometry.boundsTree=new MeshBVH(geometry);return geometry;
});
const results=[];
for (let direction=0;direction<2;direction++) {
    const a=meshes[direction],b=meshes[1-direction],positions=a.attributes.position;
    const point=new THREE.Vector3(),corner=new THREE.Vector3(),hit={point:new THREE.Vector3()};
    let maxDistance=0,samples=0;
    const triangles=positions.count/3,stride=Math.max(1,Math.floor(triangles/10000));
    for(let face=0;face<triangles;face+=stride) {
        point.set(0,0,0);
        for(let k=0;k<3;k++)point.add(corner.fromBufferAttribute(positions,face*3+k));
        point.multiplyScalar(1/3);
        b.boundsTree.closestPointToPoint(point,hit);
        maxDistance=Math.max(maxDistance,hit.distance);samples++;
    }
    results.push({from:direction===0?'original':'repaired',samples,maxDistanceSourceUnits:maxDistance});
}
meshes.forEach(g=>g.dispose());
fs.writeFileSync(reportPath,JSON.stringify({source,repaired,results},null,2)+'\n');
console.log(JSON.stringify(results));
if (results.some(r=>r.maxDistanceSourceUnits>.001))throw Error('Surface deviation exceeds 0.001 source units');

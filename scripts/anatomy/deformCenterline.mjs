import * as THREE from 'three';
import {createInfrarenalDeformation} from './infrarenalAneurysm.mjs';

// Preserve the established branch topology while transporting it through the
// same map as the surface. Short subdivisions follow curved displaced branches.
export function deformCenterline(asset,report,geometry) {
    const {move}=createInfrarenalDeformation(report),segments=[];
    const selected=report.selectedCenterlineSegments ? new Set(report.selectedCenterlineSegments) : null;
    const data=asset.arrays.centerlineSegments,edges=asset.arrays.centerlineEdges;
    const target={point:new THREE.Vector3()};
    let inserted=0;
    for(let i=0;i<edges.length/2;i++) {
        const a=new THREE.Vector3().fromArray(data,i*9),b=new THREE.Vector3().fromArray(data,i*9+3);
        const changed=(!selected || selected.has(i)) && Math.max(a.y,b.y)>report.distalY && Math.min(a.y,b.y)<report.proximalY;
        const count=changed?Math.max(1,Math.ceil(a.distanceTo(b)/.35)):1;
        const points=[];
        for(let j=0;j<=count;j++) {
            const original=a.clone().lerp(b,j/count),point=changed ? new THREE.Vector3(...move(...original.toArray())) : original.clone();
            const moved=point.distanceToSquared(original)>1e-14;
            const radius=moved ? geometry.boundsTree.closestPointToPoint(point,target).distance
                : data[i*9+6]+(data[i*9+7]-data[i*9+6])*j/count;
            points.push({point,radius,id:j===0?edges[2*i]:j===count?edges[2*i+1]:`deformed:${i}:${j}`});
        }
        for(let j=0;j<count;j++)segments.push({start:points[j].point,end:points[j+1].point,
            nodeStartId:points[j].id,nodeEndId:points[j+1].id,
            radiusStart:points[j].radius,radiusEnd:points[j+1].radius});
        inserted+=count-1;
    }
    return {segments,diagnostics:{source:'deformed-baseline-centerline',
        baselineSha256:report.sourceSha256,componentCount:asset.metadata.centerline.diagnostics.componentCount,
        centerlineGraphCycleCount:asset.metadata.centerline.diagnostics.centerlineGraphCycleCount,
        centerlineGraphNodeCount:asset.metadata.centerline.nodeCount+inserted,
        insertedNodeCount:inserted}};
}

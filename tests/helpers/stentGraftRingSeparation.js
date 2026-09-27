import * as THREE from 'three';
import {closestSegmentSegment} from '../../src/physics/kirchhoffLumenContact.js';
export function wireIntersections(parts) {
    const rings=[];
    parts.forEach((part,pi)=>{
        const a=part.rings.geometry.attributes.instanceStart,b=part.rings.geometry.attributes.instanceEnd;
        let first=0;
        part.scaffoldRings.forEach((ring,ri)=>{
            const segments=[],bounds=new THREE.Box3();
            for(let i=first;i<first+ring.samples;i++){
                const p=new THREE.Vector3().fromBufferAttribute(a,i),q=new THREE.Vector3().fromBufferAttribute(b,i);
                bounds.expandByPoint(p);bounds.expandByPoint(q);
                segments.push({p:p.toArray(),q:q.toArray(),box:new THREE.Box3().setFromPoints([p,q]).expandByScalar(.09)});
            }
            rings.push({pi,ri,segments,bounds:bounds.expandByScalar(.09)});first+=ring.samples;
        });
    });
    const hits=[];
    for(let i=0;i<rings.length;i++)for(let j=i+1;j<rings.length;j++) {
        const a=rings[i],b=rings[j];if(!a.bounds.intersectsBox(b.bounds))continue;
        let min=Infinity;
        for(const p of a.segments)for(const q of b.segments)if(p.box.intersectsBox(q.box))min=Math.min(min,closestSegmentSegment(p.p,p.q,q.p,q.q).distance);
        if(min<.089)hits.push({a:[a.pi,a.ri],b:[b.pi,b.ri],distance:min});
    }
    return hits;
}

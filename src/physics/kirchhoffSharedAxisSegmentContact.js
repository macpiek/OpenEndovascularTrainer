import {Box3,DoubleSide,Line3,Ray,Vector3} from 'three';

/** Exact spatial segment/triangle distance within a supplied search radius.
 * Sample-grid certificates do not cover this query. The returned cutoff is a
 * conservative distance bound when no triangle is closer. Not temporal CCD.
 */
export function createSharedAxisSegmentContact(geometry) {
    const segment=new Line3(),ray=new Ray(),bounds=new Box3(),surface=new Vector3(),axisPoint=new Vector3();
    const stats={queries:0,triangles:0,crossings:0};
    return {stats,query(a,b,cutoff,{axisOnly=false}={}) {
        if(!(Number.isFinite(cutoff)&&cutoff>0)||a.length!==3||b.length!==3||!a.every(Number.isFinite)||!b.every(Number.isFinite))
            throw new RangeError('Finite segment and positive cutoff required');
        const tree=geometry?.boundsTree;if(!tree)throw new Error('Continuous contact requires mesh BVH');
        segment.start.fromArray(a);segment.end.fromArray(b);
        const length=segment.start.distanceTo(segment.end);stats.queries++;
        ray.origin.copy(segment.start);ray.direction.subVectors(segment.end,segment.start);
        if(length>0) {
            ray.direction.multiplyScalar(1/length);
            const hit=tree.raycastFirst(ray,DoubleSide,0,length);
            if(hit){stats.crossings++;return {distance:0,t:hit.distance/length,face:hit.faceIndex,crossing:true};}
        }
        if(axisOnly)return {distance:cutoff,t:0,face:-1,crossing:false};
        bounds.makeEmpty().expandByPoint(segment.start).expandByPoint(segment.end).expandByScalar(cutoff);
        let distance=cutoff,t=0,face=-1;
        tree.shapecast({intersectsBounds:box=>box.intersectsBox(bounds),intersectsTriangle:(triangle,index)=>{
            stats.triangles++;
            // Explicit ray intersection above covers the face-interior case
            // absent from ExtendedTriangle.closestPointToSegment.
            const d=triangle.closestPointToSegment(segment,surface,axisPoint);
            if(d<distance){distance=d;t=length?segment.closestPointToPointParameter(axisPoint,true):0;face=index;}
            return false;
        }});
        return {distance,t,face,crossing:false};
    }};
}

import {Box3,DoubleSide,Line3,Ray,Vector3} from 'three';

/** Exact spatial segment/triangle distance within a supplied search radius.
 * Sample-grid certificates do not cover this query. The returned cutoff is a
 * conservative distance bound when no triangle is closer. Not temporal CCD.
 */
export function createSharedAxisSegmentContact(geometry,{cacheCapacity=0}={}) {
    if(!Number.isInteger(cacheCapacity)||cacheCapacity<0)throw new RangeError('Invalid segment cache capacity');
    // Opt in only for static geometry, or bump position/index versions after
    // edits and refit/rebuild the BVH. Deforming graft surfaces remain uncached.
    const cache=new Map();
    let cacheTree,cachePositions,cacheVersion,cacheIndex,cacheIndexVersion;
    const segment=new Line3(),ray=new Ray(),bounds=new Box3(),surface=new Vector3(),axisPoint=new Vector3();
    const stats={queries:0,triangles:0,crossings:0,cacheHits:0,clearanceHits:0};
    return {stats,clearCache(){cache.clear();},query(a,b,cutoff,{axisOnly=false,cacheKey=null}={}) {
        if(!(Number.isFinite(cutoff)&&cutoff>0)||a.length!==3||b.length!==3||!a.every(Number.isFinite)||!b.every(Number.isFinite))
            throw new RangeError('Finite segment and positive cutoff required');
        const tree=geometry?.boundsTree;if(!tree)throw new Error('Continuous contact requires mesh BVH');
        const position=geometry.attributes.position,index=geometry.index;
        if(cacheTree!==tree||cachePositions!==position||cacheVersion!==position.version||cacheIndex!==index||cacheIndexVersion!==index?.version){
            cache.clear();cacheTree=tree;cachePositions=position;cacheVersion=position.version;cacheIndex=index;cacheIndexVersion=index?.version;
        }
        const useCache=cacheCapacity>0&&cacheKey!==null&&!axisOnly;
        const old=useCache?cache.get(cacheKey):null;
        if(old){
            const movement=Math.max(Math.hypot(...a.map((v,k)=>v-old.a[k])),Math.hypot(...b.map((v,k)=>v-old.b[k])));
            if(movement===0&&old.cutoff===cutoff){stats.cacheHits++;return {...old.result};}
            // The Hausdorff distance of affine segments is bounded by the
            // largest endpoint displacement. Keep the original certificate
            // endpoints, so a sequence of small moves cannot accumulate drift.
            const epsilon=256*Number.EPSILON*Math.max(1,cutoff,...a.map(Math.abs),...b.map(Math.abs),...old.a.map(Math.abs),...old.b.map(Math.abs));
            if(old.bound-movement>cutoff+epsilon){stats.clearanceHits++;return {distance:cutoff,t:0,face:-1,crossing:false};}
        }
        const save=(result,bound)=>{
            if(useCache){
                if(cache.size>=cacheCapacity&&!cache.has(cacheKey))cache.delete(cache.keys().next().value);
                cache.set(cacheKey,{a:Array.from(a),b:Array.from(b),cutoff,result:{...result},bound});
            }
            return result;
        };
        segment.start.fromArray(a);segment.end.fromArray(b);
        const length=segment.start.distanceTo(segment.end);stats.queries++;
        ray.origin.copy(segment.start);ray.direction.subVectors(segment.end,segment.start);
        if(length>0) {
            ray.direction.multiplyScalar(1/length);
            const hit=tree.raycastFirst(ray,DoubleSide,0,length);
            if(hit){stats.crossings++;return save({distance:0,t:hit.distance/length,face:hit.faceIndex,crossing:true},0);}
        }
        if(axisOnly)return {distance:cutoff,t:0,face:-1,crossing:false};
        const searchCutoff=useCache?cutoff+.5:cutoff;
        bounds.makeEmpty().expandByPoint(segment.start).expandByPoint(segment.end).expandByScalar(searchCutoff);
        let distance=searchCutoff,t=0,face=-1;
        tree.shapecast({intersectsBounds:box=>box.intersectsBox(bounds),intersectsTriangle:(triangle,index)=>{
            stats.triangles++;
            // Explicit ray intersection above covers the face-interior case
            // absent from ExtendedTriangle.closestPointToSegment.
            const d=triangle.closestPointToSegment(segment,surface,axisPoint);
            if(d<distance){distance=d;t=length?segment.closestPointToPointParameter(axisPoint,true):0;face=index;}
            return false;
        }});
        return save(distance<cutoff?{distance,t,face,crossing:false}:{distance:cutoff,t:0,face:-1,crossing:false},distance);
    }};
}

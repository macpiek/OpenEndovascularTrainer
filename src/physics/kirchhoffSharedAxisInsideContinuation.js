import {DoubleSide,Ray,Vector3} from 'three';

/** Continue lumen membership from a previously classified point through a
 * local wall-free path. Ray parity is inappropriate: the STL is wall material,
 * whose complement contains both lumen and exterior. No surface crossing is
 * allowed, even when endpoint signs happen to agree. Evidence is geometry-only
 * and may outlive a rejected Newton trial, like distance certificates. */
export function createSharedAxisInsideContinuation(geometry,{capacity=4096,maxDistance=64}={}) {
    if(!Number.isInteger(capacity)||capacity<1||!Number.isFinite(maxDistance)||maxDistance<=0)throw new RangeError('Invalid inside-continuation limits');
    const entries=new Map(),ray=new Ray(new Vector3(),new Vector3());let token=geometry?.boundsTree;
    const stats={distanceProofs:0,pathProofs:0,blocked:0,misses:0};
    const current=()=>{if(token!==geometry?.boundsTree){entries.clear();token=geometry?.boundsTree;}};
    const distance=(p,q)=>Math.hypot(p[0]-q[0],p[1]-q[1],p[2]-q[2]);
    return {stats,
        remember(site,point,clearance){
            current();if(!Number.isFinite(site)||!(clearance>0)||!point.every(Number.isFinite))return;
            if(entries.size>=capacity&&!entries.has(site))entries.delete(entries.keys().next().value);
            entries.set(site,{site,point:point.slice(),clearance,
                scale:Math.max(1,Math.abs(point[0]),Math.abs(point[1]),Math.abs(point[2]))});
        },
        contains(site,point){
            current();if(!point||point.length!==3||!point.every(Number.isFinite)||!token||typeof token.raycastFirst!=='function')return false;
            let old=entries.get(site),length=old?distance(point,old.point):Infinity;
            const scale=Math.max(1,Math.abs(point[0]),Math.abs(point[1]),Math.abs(point[2]));
            const guardFor=entry=>1e-8*Math.max(scale,entry.scale);
            // A certified empty ball remains valid beyond the local ray limit.
            // Far from walls discovery can skip many millimetres, then remesh
            // to a new sample grid. The proof must not expire merely because
            // those independently safe skips moved more than maxDistance.
            if(old&&old.clearance-length>guardFor(old)){stats.distanceProofs++;return true;}
            for(const entry of entries.values()){
                const d=distance(point,entry.point);
                if(entry.clearance-d>guardFor(entry)){stats.distanceProofs++;return true;}
                if(d<length){old=entry;length=d;}
            }
            // Cached discovery can skip a whole coarse edge. A fresh sample
            // may then be tens of millimetres from the last exact inside site.
            // The ray still proves no wall crossing over its complete length.
            if(!old||length>maxDistance){stats.misses++;return false;}
            const guard=guardFor(old);
            if(length<=guard){stats.misses++;return false;}
            ray.origin.fromArray(old.point);ray.direction.set(point[0]-old.point[0],point[1]-old.point[1],point[2]-old.point[2]).multiplyScalar(1/length);
            const hit=token.raycastFirst(ray,DoubleSide,0,length+guard);
            if(hit){stats.blocked++;return false;}
            stats.pathProofs++;return true;
        },
        capture(){current();return [...entries.values()].map(({site,point,clearance})=>({site,point:point.slice(),clearance}));},
        restore(values){current();entries.clear();for(const e of values??[])this.remember(e.site,e.point,e.clearance);}
    };
}

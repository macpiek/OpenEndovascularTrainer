import * as THREE from 'three';

/** Ideal sealed EVAR reduction of the existing finite-volume network. Keep
 * edge/cell identities (injection couplers retain them) and conserve iodine. */
export function applyStentGraftFlow(network,surface) {
    if(!surface?.sealed||network.stentGraftRevision===surface.revision)return false;
    const covered=new Map(),excluded=new Set(),trapped=new Map();
    for(const edge of network.edges) {
        if(edge.transportExcluded)continue;
        const middle=edge.start.clone().lerp(edge.end,.5),section=surface.sectionAt(middle);
        if(section&&surface.contains(middle))covered.set(edge.index,section);
    }
    for(const node of network.nodes.values()) {
        if(!covered.has(node.parentEdgeIndex)||!surface.contains(node.point))continue;
        for(const index of [...node.childEdgeIndices]) {
            if(covered.has(index))continue;
            const queue=[index];
            for(let i=0;i<queue.length;i++){excluded.add(queue[i]);queue.push(...network.edges[queue[i]].childEdgeIndices);}
            node.childEdgeIndices.splice(node.childEdgeIndices.indexOf(index),1);
        }
    }
    let trappedMass=0;
    for(const edge of network.edges) {
        const section=covered.get(edge.index),blocked=excluded.has(edge.index);
        if(!section&&!blocked)continue;
        const originalVolumes=Float64Array.from(edge.volumes),masses=new Float64Array(edge.cellCount);
        if(blocked) {
            edge.transportExcluded=true;edge.meanFlowMm3PerS=0;edge.active=false;
            network._activeEdgeIndices.delete(edge.index);
        } else {
            edge.graftCovered=true;
            edge.radiusStart=Math.min(edge.radiusStart,surface.sectionAt(edge.start)?.radius??section.radius);
            edge.radiusEnd=Math.min(edge.radiusEnd,surface.sectionAt(edge.end)?.radius??section.radius);
            edge.safeRadius=Math.min(edge.safeRadius,edge.radiusStart,edge.radiusEnd);
            edge.totalVolume=0;
            for(let i=0;i<edge.cellCount;i++) {
                const radius=THREE.MathUtils.lerp(edge.radiusStart,edge.radiusEnd,(i+.5)/edge.cellCount);
                edge.areas[i]=Math.min(edge.areas[i],Math.PI*radius*radius);
                edge.volumes[i]=Math.min(edge.volumes[i],edge.areas[i]*edge.cellLength);
                edge.totalVolume+=edge.volumes[i];
            }
            edge.resistance=8*network.hemodynamics.bloodViscosityPaS*edge.length/(Math.PI*((edge.radiusStart+edge.radiusEnd)/2)**4);
        }
        for(let i=0;i<edge.cellCount;i++) {
            const fraction=blocked?1:Math.max(0,1-edge.volumes[i]/originalVolumes[i]);
            masses[i]=edge.massMg[i]*fraction;edge.massMg[i]-=masses[i];trappedMass+=masses[i];
        }
        trapped.set(edge.index,{mass:masses,volumes:originalVolumes});
    }
    network.stentGraftRevision=surface.revision;
    network.stentGraftRemodeling={surface,trapped,coveredEdges:covered.size,excludedEdges:excluded.size,trappedIodineMassMg:trappedMass};
    network._computeHydraulicDistribution();network._updateMaximumMeanVelocity();
    network._spatialIndex.clear();network._buildSpatialIndex();network.clearFlowOverrides();network._updateEdgeConcentrations();
    return true;
}

/** Fluid cannot cross fabric. Open portals still use the anatomical field. */
export function graftFluidContactField(anatomy,surface) {
    const point=new THREE.Vector3(),region=surface.bounds.clone().expandByScalar(35);
    return {querySphere(position,radius,out) {
        const result=anatomy.querySphere(position,radius,out);point.copy(position);
        if(!region.containsPoint(point))return result;
        const closed=surface.solid.boundsTree.closestPointToPoint(point,{}),wall=surface.nearest(point);
        const inside=surface.contains(point);
        if(!wall||!inside&&closed.distance+.01<wall.distance)return result; // outside an open portal
        const distance=wall.distance*(inside?1:-1);
        if(distance>=result.signedDistance)return result;
        const inward=inside?point.clone().sub(wall.point):wall.point.clone().sub(point);inward.normalize();
        result.signedDistance=distance;result.signedGap=distance-radius;result.distance=wall.distance;
        result.inside=inside;result.violation=distance<radius;result.penetration=Math.max(0,radius-distance);result.source='stent-graft';
        for(const key of ['x','y','z']) {
            result.inward[key]=inward[key];result.normal[key]=-inward[key];result.closestPoint[key]=wall.point[key];
            result.target[key]=position[key]+inward[key]*result.penetration;
        }
        return result;
    }};
}

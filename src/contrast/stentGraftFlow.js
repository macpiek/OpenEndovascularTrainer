import * as THREE from 'three';
import {createGraftSacTransport} from './graftSacTransport.js';

const alignmentLimit=.6;
function nativeGeometry(network) {
    return network._preGraftGeometry??=network.edges.map(edge=>({
        radiusStart:edge.radiusStart,radiusEnd:edge.radiusEnd,safeRadius:edge.safeRadius,
        areas:edge.areas.slice(),volumes:edge.volumes.slice(),totalVolume:edge.totalVolume,
        resistance:edge.resistance,excluded:!!edge.transportExcluded,
        children:[...edge.childEdgeIndices]
    }));
}
function sectionForEdge(surface,edge,point,radius,throughGraft=false) {
    const section=surface.sectionAt(point);
    if(!section)return null;
    if(section.tangent&&!throughGraft) {
        if(Math.abs(section.tangent.dot(edge.axis))<alignmentLimit||section.distance>radius)return null;
    } else if(!section.tangent&&!surface.contains(point))return null;
    return section;
}

/** Finite-volume reduction of deployed fabric. Preserve edge/cell identities,
 * ostia outside the covered segment, and circulating + trapped iodine across
 * every revision. A main body with an open gate separates graft lumen and sac; only the
 * distal gate feeds that sac. Complete exclusion requires a connected seal. */
export function applyStentGraftFlow(network,surface) {
    if(!surface||network.stentGraftRevision===surface.revision)return false;
    // Older/test surfaces without geometric paths describe only a sealed model.
    if(!surface.empty&&!surface.sealed&&!surface.paths?.length)return false;
    const originals=nativeGeometry(network),old=network.stentGraftRemodeling;
    const covered=new Map(),excluded=new Set(),point=new THREE.Vector3();
    // Follow anatomical connectivity between the fabric portals. Nearest-axis
    // tests alone can confuse a side branch with the parent in an aneurysm and
    // accidentally exclude the downstream aorta when the graft is off-centre.
    let throughPaths=null;
    if(surface.portals?.length) {
        throughPaths=new Set();
        for(const edge of network.edges)edge.transportExcluded=originals[edge.index].excluded;
        const destinations=[...(surface.flowOutlets??[]),...surface.portals.map(port=>port.center)];
        for(const point of destinations) {
            let index=network.findNearestLocation(point,{}).edgeIndex;
            while(index>=0&&!throughPaths.has(index)) {throughPaths.add(index);index=network.edges[index].parentEdgeIndex;}
        }
    }
    for(const edge of network.edges) {
        const original=originals[edge.index];
        if(original.excluded||throughPaths&&!throughPaths.has(edge.index))continue;
        const cells=Array.from({length:edge.cellCount},(_,i)=>{
            const t=(i+.5)/edge.cellCount;
            point.copy(edge.start).lerp(edge.end,t);
            return sectionForEdge(surface,edge,point,THREE.MathUtils.lerp(original.radiusStart,original.radiusEnd,t),!!throughPaths);
        });
        if(cells.some(Boolean))covered.set(edge.index,cells);
    }
    for(const node of network.nodes.values()) {
        const parent=network.edges[node.parentEdgeIndex];
        if(!parent||!covered.has(parent.index))continue;
        const section=sectionForEdge(surface,parent,node.point,originals[parent.index].radiusEnd,!!throughPaths);
        if(!section)continue;
        for(const index of originals[parent.index].children) {
            if(covered.has(index)||throughPaths?.has(index)||originals[index].excluded)continue;
            // In an open system, an ostium is occluded only where fabric is
            // apposed to it. A branch outside a loose graft still has bypass.
            const branch=network.edges[index];
            const exitsPortal=surface.portals?.some(port=>{
                const delta=node.point.clone().sub(port.center);
                return Math.abs(delta.dot(port.normal))<.05&&delta.length()<=section.radius+1&&branch.axis.dot(port.normal)>.2;
            });
            if(exitsPortal)continue;
            const ostium=node.point.clone().addScaledVector(branch.axis,originals[parent.index].radiusEnd);
            const apposed=surface.paths?.length&&surface.nearest(ostium)?.distance<1;
            if(!surface.sealed&&!surface.openGate&&!apposed)continue;
            const queue=[index];
            for(let i=0;i<queue.length;i++) {
                if(excluded.has(queue[i]))continue;
                excluded.add(queue[i]);queue.push(...originals[queue[i]].children);
            }
        }
    }
    const trapped=new Map(),unmappedTrappedIodineMassMg=old?.unmappedTrappedIodineMassMg??0;let trappedMass=unmappedTrappedIodineMassMg;
    for(const edge of network.edges) {
        const original=originals[edge.index],sections=covered.get(edge.index),blocked=excluded.has(edge.index);
        const oldTrapped=old?.trapped.get(edge.index),wasBlocked=old?.excludedEdgeIndices?.has(edge.index)??false;
        const oldVolumes=Float64Array.from(edge.volumes,v=>wasBlocked?0:v);
        edge.transportExcluded=original.excluded||blocked;
        edge.graftCovered=!!sections;
        edge.graftSections=sections??null;
        edge.radiusStart=original.radiusStart;edge.radiusEnd=original.radiusEnd;edge.safeRadius=original.safeRadius;
        edge.areas.set(original.areas);edge.volumes.set(original.volumes);edge.resistance=original.resistance;
        if(sections&&(surface.sealed||surface.openGate)) {
            for(let i=0;i<edge.cellCount;i++)if(sections[i]) {
                const area=Math.min(original.areas[i],Math.PI*sections[i].radius**2);
                edge.areas[i]=area;edge.volumes[i]=original.volumes[i]*area/original.areas[i];
            }
            edge.radiusStart=Math.sqrt(edge.areas[0]/Math.PI);
            edge.radiusEnd=Math.sqrt(edge.areas.at(-1)/Math.PI);
            edge.safeRadius=Math.min(original.safeRadius,edge.radiusStart,edge.radiusEnd);
            edge.resistance=0;
            for(const area of edge.areas)edge.resistance+=8*network.hemodynamics.bloodViscosityPaS*edge.cellLength*Math.PI/(area*area);
        }
        edge.totalVolume=edge.volumes.reduce((a,b)=>a+b,0);
        const masses=new Float64Array(edge.cellCount),volumes=new Float64Array(edge.cellCount);
        for(let i=0;i<edge.cellCount;i++) {
            const nextVolume=blocked?0:edge.volumes[i],previousVolume=oldVolumes[i];
            let held=oldTrapped?.mass[i]??0;
            if(nextVolume<previousVolume) {
                const transferred=edge.massMg[i]*(1-nextVolume/previousVolume);
                edge.massMg[i]-=transferred;held+=transferred;
            } else if(nextVolume>previousVolume&&held>0) {
                const fraction=Math.min(1,(nextVolume-previousVolume)/Math.max(1e-9,original.volumes[i]-previousVolume));
                const released=held*fraction;held-=released;edge.massMg[i]+=released;
            }
            masses[i]=held;volumes[i]=Math.max(0,original.volumes[i]-nextVolume);trappedMass+=held;
        }
        if(masses.some(v=>v>0)||volumes.some(v=>v>0))trapped.set(edge.index,{mass:masses,volumes});
        // Mutate in place: nodes and directed edges share these adjacency arrays.
        edge.childEdgeIndices.splice(0,edge.childEdgeIndices.length,...original.children.filter(i=>!excluded.has(i)));
        edge.nextMassMg.set(edge.massMg);
        if(blocked) {edge.meanFlowMm3PerS=0;edge.active=false;network._activeEdgeIndices.delete(edge.index);}
        else if(edge.massMg.some(v=>v>0)) {edge.active=true;network._activeEdgeIndices.add(edge.index);}
    }
    network.stentGraftRevision=surface.revision;
    network.stentGraftRemodeling={surface,trapped,coveredEdges:covered.size,excludedEdges:excluded.size,excludedEdgeIndices:excluded,trappedIodineMassMg:trappedMass,unmappedTrappedIodineMassMg};
    network._computeHydraulicDistribution();network._updateMaximumMeanVelocity();
    network._spatialIndex.clear();network._buildSpatialIndex();network.clearFlowOverrides();network._updateEdgeConcentrations();
    network.stentGraftRemodeling.sac=createGraftSacTransport(network,network.stentGraftRemodeling);
    return true;
}

/** Keep a parcel on its existing side of fabric; caps only identify portals.
 * A fresh injection outside an unsealed graft belongs to the native bypass. */
export function graftFluidContactField(anatomy,surface) {
    const point=new THREE.Vector3(),region=surface.bounds.clone().expandByScalar(35);
    const inward=new THREE.Vector3(),closedHit={},wallHit={};
    const query=(position,radius,out,previous)=>{
        const result=anatomy.querySphere(position,radius,out);point.copy(position);
        if(!region.containsPoint(point))return result;
        const closed=surface.solid.boundsTree.closestPointToPoint(point,closedHit);
        const wall=surface.geometry.boundsTree.closestPointToPoint(point,wallHit);
        const inside=surface.contains(point);
        if(!wall||!closed||!inside&&closed.distance+.01<wall.distance)return result;
        const crossedPortal=previous&&surface.portals?.some(port=>{
            const a=(previous.x-port.center.x)*port.normal.x+(previous.y-port.center.y)*port.normal.y+(previous.z-port.center.z)*port.normal.z;
            const b=(point.x-port.center.x)*port.normal.x+(point.y-port.center.y)*port.normal.y+(point.z-port.center.z)*port.normal.z;
            if(a*b>0||Math.abs(a-b)<1e-12)return false;
            const t=a/(a-b),x=previous.x+(point.x-previous.x)*t-port.center.x,
                y=previous.y+(point.y-previous.y)*t-port.center.y,z=previous.z+(point.z-previous.z)*t-port.center.z;
            return x*x+y*y+z*z<(port.radius??0)**2;
        });
        const lumen=crossedPortal?inside:previous?surface.contains(previous):surface.sealed||inside;
        const distance=wall.distance*(inside===lumen?1:-1);
        if(distance>=result.signedDistance)return result;
        inward.copy(inside?point:wall.point).sub(inside?wall.point:point).normalize();
        if(!lumen)inward.negate();
        result.signedDistance=distance;result.signedGap=distance-radius;result.distance=wall.distance;
        result.inside=distance>=0;result.violation=distance<radius;result.penetration=Math.max(0,radius-distance);result.source='stent-graft';
        for(const key of ['x','y','z']) {
            result.inward[key]=inward[key];result.normal[key]=-inward[key];result.closestPoint[key]=wall.point[key];
            result.target[key]=position[key]+inward[key]*result.penetration;
        }
        return result;
    };
    return {querySphere:(position,radius,out)=>query(position,radius,out,null),
        querySphereFrom:(position,radius,out,previous)=>query(position,radius,out,previous)};
}
